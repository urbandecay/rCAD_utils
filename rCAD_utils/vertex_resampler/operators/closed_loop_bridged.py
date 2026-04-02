# closed_loop_bridged.py — Resample bridged closed-loop selections.

from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
from .resample_common import execute_aligned_loops_logic


def _vert_is_usable(vert):
    try:
        return bool(vert is not None and vert.is_valid)
    except Exception:
        return False


def _loop_centroid(loop):
    valid = [vert for vert in loop if _vert_is_usable(vert)]
    if not valid:
        return None
    count = float(len(valid))
    return (
        sum(vert.co.x for vert in valid) / count,
        sum(vert.co.y for vert in valid) / count,
        sum(vert.co.z for vert in valid) / count,
    )


def _loop_relative_positions(loop):
    valid = [vert for vert in loop if _vert_is_usable(vert)]
    centroid = _loop_centroid(valid)
    if not valid or centroid is None:
        return []
    cx, cy, cz = centroid
    return [
        (vert.co.x - cx, vert.co.y - cy, vert.co.z - cz)
        for vert in valid
    ]


def _rotate_loop(loop, start_index):
    if not loop:
        return list(loop)
    start_index %= len(loop)
    return list(loop[start_index:]) + list(loop[:start_index])


def _reverse_loop_with_fixed_start(loop):
    if len(loop) <= 1:
        return list(loop)
    return [loop[0]] + list(reversed(loop[1:]))


def _relative_loop_score(loop_a, loop_b):
    rel_a = _loop_relative_positions(loop_a)
    rel_b = _loop_relative_positions(loop_b)
    if len(rel_a) != len(rel_b) or not rel_a:
        return None
    return sum(
        ((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2) ** 0.5
        for (ax, ay, az), (bx, by, bz) in zip(rel_a, rel_b)
    )


def _updated_group_with_loops(group_data, loops, is_closed):
    forced_seam_verts = [
        {loop_item[0]} if loop_item else set()
        for loop_item in loops
    ]
    if isinstance(group_data, dict):
        updated = dict(group_data)
    else:
        updated = {}
    updated['rings'] = (loops, is_closed)
    updated['use_seams'] = True
    updated['migrate_seams'] = True
    updated['max_seams'] = 2
    updated['forced_seam_verts'] = forced_seam_verts
    return updated


def _group_centroid_sort_key(group_data):
    loops = group_data['rings'][0] if isinstance(group_data, dict) else group_data[0]
    centroids = [
        centroid for centroid in (_loop_centroid(loop) for loop in loops)
        if centroid is not None
    ]
    if not centroids:
        return (float("inf"), float("inf"), float("inf"))
    count = float(len(centroids))
    return (
        sum(item[0] for item in centroids) / count,
        sum(item[1] for item in centroids) / count,
        sum(item[2] for item in centroids) / count,
    )


def _normalize_group_anchor_phase(groups):
    if len(groups or []) < 2:
        return groups

    ordered = sorted(
        enumerate(groups),
        key=lambda item: _group_centroid_sort_key(item[1]),
    )
    reference_index, reference_group = ordered[0]
    reference_loops = reference_group['rings'][0] if isinstance(reference_group, dict) else reference_group[0]
    if not reference_loops:
        return groups
    reference_loop = list(reference_loops[0])
    if not reference_loop:
        return groups

    normalized = list(groups)
    normalized[reference_index] = _updated_group_with_loops(reference_group, [list(loop) for loop in reference_loops], True)

    for group_index, group_data in ordered[1:]:
        loops, is_closed = group_data['rings'] if isinstance(group_data, dict) else group_data
        if not is_closed or not loops or len(loops[0]) != len(reference_loop):
            normalized[group_index] = group_data
            continue

        best = None
        for start_index in range(len(loops[0])):
            rotated_loops = [
                _rotate_loop(loop_item, start_index)
                for loop_item in loops
            ]
            forward_score = _relative_loop_score(rotated_loops[0], reference_loop)
            reversed_loops = [
                _reverse_loop_with_fixed_start(loop_item)
                for loop_item in rotated_loops
            ]
            reverse_score = _relative_loop_score(reversed_loops[0], reference_loop)

            candidates = []
            if forward_score is not None:
                candidates.append((forward_score, False))
            if reverse_score is not None:
                candidates.append((reverse_score, True))
            if not candidates:
                continue

            score, reverse_group = min(candidates, key=lambda item: item[0])
            candidate = (score, start_index, reverse_group)
            if best is None or candidate < best:
                best = candidate

        if best is None:
            normalized[group_index] = group_data
            continue

        _score, start_index, reverse_group = best
        rotated_loops = [
            _rotate_loop(loop_item, start_index)
            for loop_item in loops
        ]
        if reverse_group:
            rotated_loops = [
                _reverse_loop_with_fixed_start(loop_item)
                for loop_item in rotated_loops
            ]
        normalized[group_index] = _updated_group_with_loops(group_data, rotated_loops, is_closed)

    return normalized


def _selected_face_set(bm, sel_set):
    return {
        face for face in bm.faces
        if len(face.verts) >= 3 and (
            face.select
            or all(vert in sel_set for vert in face.verts)
            or all(edge.select for edge in face.edges)
        )
    }


def _selected_face_neighbors(face, selected_faces):
    neighbors = set()
    for edge in face.edges:
        for other_face in edge.link_faces:
            if other_face is face or other_face not in selected_faces:
                continue
            neighbors.add(other_face)
    return neighbors


def _is_shaft_face(face, selected_faces):
    if len(face.verts) != 4:
        return False

    internal_edges = []
    for edge in face.edges:
        has_selected_quad_neighbor = any(
            other_face is not face
            and other_face in selected_faces
            and len(other_face.verts) == 4
            for other_face in edge.link_faces
        )
        if has_selected_quad_neighbor:
            internal_edges.append(edge)

    if len(internal_edges) != 2:
        return False

    if set(internal_edges[0].verts).intersection(internal_edges[1].verts):
        return False

    return True


def _face_components(faces):
    visited = set()
    components = []

    for face in sorted(faces, key=lambda item: item.index):
        if face in visited:
            continue

        component = set()
        stack = [face]
        visited.add(face)

        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in sorted(
                _selected_face_neighbors(current, faces),
                key=lambda item: item.index,
            ):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

        components.append(component)

    return components


def _component_edge_face_counts(component):
    counts = {}
    for face in component:
        for edge in face.edges:
            counts[edge] = counts.get(edge, 0) + 1
    return counts


def _order_boundary_loop(edges):
    adjacency = {}
    for edge in edges:
        v1, v2 = edge.verts
        adjacency.setdefault(v1, []).append(v2)
        adjacency.setdefault(v2, []).append(v1)

    if not adjacency:
        return None
    if any(len(neighbors) != 2 for neighbors in adjacency.values()):
        return None

    start = min(adjacency, key=lambda vert: vert.index)
    ordered = [start]
    previous = None
    current = start

    while True:
        next_vert = None
        for neighbor in sorted(adjacency[current], key=lambda vert: vert.index):
            if neighbor is previous:
                continue
            next_vert = neighbor
            break

        if next_vert is None:
            return None
        if next_vert is start:
            break
        if next_vert in ordered:
            return None

        ordered.append(next_vert)
        previous = current
        current = next_vert

    return ordered if len(ordered) == len(adjacency) else None


def _boundary_loops(boundary_edges):
    edge_neighbors = {}
    vert_edges = {}
    for edge in boundary_edges:
        edge_neighbors[edge] = set()
        for vert in edge.verts:
            vert_edges.setdefault(vert, set()).add(edge)

    for edge in boundary_edges:
        for vert in edge.verts:
            edge_neighbors[edge].update(vert_edges[vert])
        edge_neighbors[edge].discard(edge)

    visited = set()
    loops = []

    for edge in sorted(boundary_edges, key=lambda item: item.index):
        if edge in visited:
            continue

        component_edges = set()
        stack = [edge]
        visited.add(edge)

        while stack:
            current = stack.pop()
            component_edges.add(current)
            for other in sorted(edge_neighbors[current], key=lambda item: item.index):
                if other in visited:
                    continue
                visited.add(other)
                stack.append(other)

        ordered = _order_boundary_loop(component_edges)
        if ordered is None:
            return None
        loops.append(ordered)

    return loops


def _aligned_partner_loop(ring0, ring1, edge_face_counts):
    ring1_set = set(ring1)
    aligned = []

    for vert in ring0:
        partner = None
        for edge in sorted(vert.link_edges, key=lambda item: item.index):
            if edge_face_counts.get(edge) != 2:
                continue
            other = edge.other_vert(vert)
            if other in ring1_set:
                partner = other
                break

        if partner is None:
            return None
        aligned.append(partner)

    return aligned


def _rings_from_component(component):
    edge_face_counts = _component_edge_face_counts(component)
    boundary_edges = [edge for edge, count in edge_face_counts.items() if count == 1]
    loops = _boundary_loops(boundary_edges)
    if loops is None or len(loops) != 2:
        return None

    ring0, ring1 = loops
    if len(ring0) != len(ring1) or len(ring0) < 3:
        return None

    ring1_aligned = _aligned_partner_loop(ring0, ring1, edge_face_counts)
    if ring1_aligned is not None:
        return ([ring0, ring1_aligned], True)

    ring0_aligned = _aligned_partner_loop(ring1, ring0, edge_face_counts)
    if ring0_aligned is not None:
        return ([ring0_aligned, ring0], True)

    return None


def _detect_shaft_face_groups(bm):
    sel_verts = [vert for vert in bm.verts if vert.select]
    if len(sel_verts) < 4:
        return None

    selected_faces = _selected_face_set(bm, set(sel_verts))
    if not selected_faces:
        return None

    shaft_faces = {
        face for face in selected_faces
        if _is_shaft_face(face, selected_faces)
    }
    if not shaft_faces:
        return None

    if selected_faces - shaft_faces:
        return None

    groups = []
    invalid_components = 0
    for component in _face_components(shaft_faces):
        rings_data = _rings_from_component(component)
        if rings_data is None:
            invalid_components += 1
            continue
        groups.append({
            'rings': rings_data,
            'use_seams': True,
            'migrate_seams': True,
            'max_seams': 2,
        })

    return {
        'groups': groups,
        'invalid_components': invalid_components,
        'mode_label': 'Closed loop bridged',
    }


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and bridged_data[1] and len(bridged_data[0]) >= 2:
        return {
            'groups': [bridged_data],
            'mode_label': 'Closed loop bridged',
        }

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        return {
            'groups': [auto_bridged],
            'mode_label': 'Closed loop bridged',
        }

    shaft_face_data = _detect_shaft_face_groups(bm)
    if shaft_face_data is not None and (
        shaft_face_data['groups'] or shaft_face_data['invalid_components']
    ):
        return shaft_face_data

    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        if report is not None:
            report({'ERROR'}, "Could not detect a valid closed loop bridge.")
        return {'CANCELLED'}

    if not data['groups']:
        if report is not None:
            if data.get('invalid_components'):
                report(
                    {'ERROR'},
                    "Detected closed loop bridge faces, but no valid bridge groups were found.",
                )
            else:
                report({'ERROR'}, "Could not detect a valid closed loop bridge.")
        return {'CANCELLED'}

    if data.get('invalid_components') and report is not None:
        report(
            {'WARNING'},
            f"Skipped {data['invalid_components']} invalid bridge component(s).",
        )

    if report is not None:
        report(
            {'INFO'},
            f"Closed loop bridge exec: groups={len(data['groups'])}, direction={direction}.",
        )

    data = {
        **data,
        'groups': _normalize_group_anchor_phase(data['groups']),
    }

    finished_groups = 0
    for group_index, group_data in enumerate(data['groups'], start=1):
        before_vert_count = len(bm.verts)
        if isinstance(group_data, dict) and group_data.get('strip_faces'):
            live_faces = {
                face for face in group_data['strip_faces']
                if face is not None and getattr(face, "is_valid", False)
            }
            if live_faces:
                rebuilt_rings = _rings_from_component(live_faces)
                if rebuilt_rings is not None:
                    group_data = dict(group_data)
                    group_data['rings'] = rebuilt_rings

        loops = group_data['rings'][0] if isinstance(group_data, dict) else group_data[0]
        if any(not _vert_is_usable(vert) for loop in loops for vert in loop):
            if report is not None:
                report(
                    {'WARNING'},
                    f"Bridge group {group_index} skipped because its verts were invalidated by an earlier edit.",
                )
            continue

        if isinstance(group_data, dict) and group_data.get('strip_faces'):
            live_faces = {
                face for face in group_data['strip_faces']
                if face is not None and getattr(face, "is_valid", False)
            }
            if not live_faces:
                if report is not None:
                    report(
                        {'WARNING'},
                        f"Bridge group {group_index} skipped because its faces were invalidated by an earlier edit.",
                    )
                continue
        if isinstance(group_data, dict):
            result = execute_aligned_loops_logic(
                bm,
                obj,
                group_data['rings'],
                direction,
                report=report,
                use_seams=group_data.get('use_seams', True),
                migrate_seams=group_data.get('migrate_seams'),
                max_seams=group_data.get('max_seams'),
                forced_seam_verts=group_data.get('forced_seam_verts'),
            )
            after_vert_count = len(bm.verts)
            if result == {'FINISHED'}:
                finished_groups += 1
            elif report is not None:
                report(
                    {'WARNING'},
                    f"Bridge group {group_index} cancelled during execution "
                    f"(mesh verts {before_vert_count}->{after_vert_count}).",
                )
            continue

        result = execute_aligned_loops_logic(
            bm,
            obj,
            group_data,
            direction,
            report=report,
        )
        after_vert_count = len(bm.verts)
        if result == {'FINISHED'}:
            finished_groups += 1
        elif report is not None:
            report(
                {'WARNING'},
                f"Bridge group {group_index} cancelled during execution "
                f"(mesh verts {before_vert_count}->{after_vert_count}).",
            )
    if finished_groups == 0:
        if report is not None:
            report({'ERROR'}, "Closed loop bridge exec failed: all groups cancelled.")
        return {'CANCELLED'}
    return {'FINISHED'}
