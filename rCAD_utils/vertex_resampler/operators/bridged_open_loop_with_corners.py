# bridged_open_loop_with_corners.py — Resample bridged open loops with corners.

import bmesh

from . import bridged_open_loop
from .open_strip_common import (
    _face_components,
    _faces_to_component_verts,
    _detect_open_strip_component,
    _strip_length_metrics,
    _component_span_metrics,
    _strip_candidate_sort_key,
    _selected_face_set,
    detect_open_strip_selection,
)
from .resample_common import execute_aligned_loops_logic

_POSITION_TOLERANCE = 1.0e-6
_FACE_POSITION_TOLERANCE = 1.0e-5


def _groups_are_open(groups):
    return bool(groups) and all(not rings_data[1] for rings_data in groups)


def _position_key(vec, tolerance):
    return (
        round(vec.x / tolerance),
        round(vec.y / tolerance),
        round(vec.z / tolerance),
    )


def _debug_step(step, **details):
    print(f"[vertex_resampler:open_corner_split] {step}")
    for key, value in details.items():
        print(f"  {key}: {value}")


def _split_positions_from_edges(edges):
    positions = []
    for edge in edges:
        if not getattr(edge, "is_valid", False):
            continue
        positions.append(tuple(vert.co.copy() for vert in edge.verts))
    return positions


def _face_indices(faces):
    return {
        face.index for face in faces
        if face is not None and getattr(face, "is_valid", False)
    }


def _face_position(face):
    if face is None or not getattr(face, "is_valid", False):
        return None
    count = len(face.verts)
    if count == 0:
        return None
    center = face.verts[0].co.copy()
    for vert in face.verts[1:]:
        center += vert.co
    center /= count
    return center


def _face_positions(faces):
    return [
        position for position in (_face_position(face) for face in faces)
        if position is not None
    ]


def _live_faces_from_indices(bm, face_indices):
    index_set = set(face_indices)
    return {
        face for face in bm.faces
        if getattr(face, "is_valid", False) and face.index in index_set
    }


def _build_face_position_lookup(bm, tolerance=_FACE_POSITION_TOLERANCE):
    lookup = {}
    for face in bm.faces:
        if not getattr(face, "is_valid", False):
            continue
        face_position = _face_position(face)
        if face_position is None:
            continue
        lookup.setdefault(_position_key(face_position, tolerance), []).append((face_position, face))
    return lookup


def _live_faces_from_positions(bm, positions, tolerance=_FACE_POSITION_TOLERANCE, lookup=None):
    live_faces = set()
    if not positions:
        return live_faces

    if lookup is None:
        lookup = _build_face_position_lookup(bm, tolerance=tolerance)

    for position in positions:
        for candidate_position, face in lookup.get(_position_key(position, tolerance), []):
            if (candidate_position - position).length <= tolerance:
                live_faces.add(face)
                break

    return live_faces


def _selected_faces(bm):
    return {
        face for face in bm.faces
        if getattr(face, "is_valid", False) and face.select
    }


def _connected_quad_faces(start_faces):
    live_start_faces = {
        face for face in start_faces
        if face is not None and getattr(face, "is_valid", False) and len(face.verts) == 4
    }
    if not live_start_faces:
        return set()

    visited = set(live_start_faces)
    stack = list(live_start_faces)

    while stack:
        face = stack.pop()
        for edge in face.edges:
            for neighbor in edge.link_faces:
                if (
                    neighbor in visited
                    or not getattr(neighbor, "is_valid", False)
                    or len(neighbor.verts) != 4
                ):
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

    return visited


def _edge_components(edges):
    live_edges = {
        edge for edge in edges
        if edge is not None and getattr(edge, "is_valid", False)
    }
    if not live_edges:
        return []

    vert_to_edges = {}
    for edge in live_edges:
        vert_a, vert_b = edge.verts
        vert_to_edges.setdefault(vert_a, set()).add(edge)
        vert_to_edges.setdefault(vert_b, set()).add(edge)

    components = []
    visited = set()
    for edge in live_edges:
        if edge in visited:
            continue
        stack = [edge]
        component = set()

        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            component.add(current)
            for vert in current.verts:
                for neighbor in vert_to_edges.get(vert, ()):
                    if neighbor not in visited:
                        stack.append(neighbor)

        components.append(component)

    return components


def _ordered_face_edges(face):
    loops = list(face.loops)
    if len(loops) != 4:
        return None
    return [loop.edge for loop in loops]


def _quad_face_neighbor(face, edge, selected_faces):
    return next(
        (
            other_face for other_face in edge.link_faces
            if other_face is not face
            and other_face in selected_faces
            and len(other_face.verts) == 4
        ),
        None,
    )


def _edge_orientation_bit(face, edge):
    ordered_edges = _ordered_face_edges(face)
    if ordered_edges is None:
        return None

    try:
        edge_index = ordered_edges.index(edge)
    except ValueError:
        return None

    return edge_index % 2


def _open_candidate_strip_faces(start_face, start_bit, selected_faces):
    oriented_faces = {start_face: start_bit}
    stack = [start_face]

    while stack:
        current = stack.pop()
        ordered_edges = _ordered_face_edges(current)
        if ordered_edges is None:
            return None

        bit = oriented_faces[current]
        chosen_edges = (
            ordered_edges[bit],
            ordered_edges[(bit + 2) % 4],
        )

        for edge in chosen_edges:
            neighbor = _quad_face_neighbor(current, edge, selected_faces)
            if neighbor is None:
                continue

            neighbor_bit = _edge_orientation_bit(neighbor, edge)
            if neighbor_bit is None:
                return None

            existing_bit = oriented_faces.get(neighbor)
            if existing_bit is None:
                oriented_faces[neighbor] = neighbor_bit
                stack.append(neighbor)
                continue
            if existing_bit != neighbor_bit:
                return None

    subset = set(oriented_faces)
    if len(subset) < 2:
        return None

    endpoint_faces = 0
    for face, bit in oriented_faces.items():
        ordered_edges = _ordered_face_edges(face)
        if ordered_edges is None:
            return None

        neighbors = []
        for edge in (ordered_edges[bit], ordered_edges[(bit + 2) % 4]):
            neighbor = _quad_face_neighbor(face, edge, subset)
            if neighbor is not None:
                neighbors.append(neighbor)

        distinct_neighbors = len(set(neighbors))
        if distinct_neighbors not in {1, 2}:
            return None
        if distinct_neighbors == 1:
            endpoint_faces += 1

    if endpoint_faces != 2:
        return None

    return subset


def _open_face_strip_candidates(face_component):
    quad_faces = {
        face for face in face_component
        if len(face.verts) == 4
    }
    if len(quad_faces) < 2:
        return []

    adjacency = {}
    for face in sorted(quad_faces, key=lambda item: item.index):
        ordered_edges = _ordered_face_edges(face)
        if ordered_edges is None:
            continue
        for bit in (0, 1):
            node = (face, bit)
            adjacency.setdefault(node, set())
            for edge in (ordered_edges[bit], ordered_edges[(bit + 2) % 4]):
                neighbor = _quad_face_neighbor(face, edge, quad_faces)
                if neighbor is None:
                    continue
                neighbor_bit = _edge_orientation_bit(neighbor, edge)
                if neighbor_bit is None:
                    continue
                adjacency[node].add((neighbor, neighbor_bit))

    candidates = []
    visited = set()

    for start_node in sorted(
        adjacency.keys(),
        key=lambda item: (item[0].index, item[1]),
    ):
        if start_node in visited:
            continue

        stack = [start_node]
        component_nodes = set()
        while stack:
            node = stack.pop()
            if node in visited:
                continue
            visited.add(node)
            component_nodes.add(node)
            for neighbor in adjacency.get(node, ()):
                if neighbor not in visited:
                    stack.append(neighbor)

        strip_faces = {face for face, _bit in component_nodes}
        if len(strip_faces) < 2:
            continue
        if len(strip_faces) != len(component_nodes):
            continue

        endpoint_count = 0
        valid = True
        for node in component_nodes:
            distinct_neighbors = len(adjacency.get(node, set()) & component_nodes)
            if distinct_neighbors not in {1, 2}:
                valid = False
                break
            if distinct_neighbors == 1:
                endpoint_count += 1

        if not valid or endpoint_count != 2:
            continue

        rings_data = _detect_open_strip_component(
            _faces_to_component_verts(strip_faces)
        )
        if rings_data is None:
            continue

        extra_faces = set(face_component) - set(strip_faces)
        boundary_edges = _shaft_corner_boundary_edges(strip_faces, extra_faces)
        raw_boundary_components = _boundary_path_components(boundary_edges)
        boundary_components = []
        for boundary_component in raw_boundary_components:
            chain = _ordered_chain_verts(boundary_component)
            key = _canonical_chain_key(chain)
            if key is None or not _is_corner_boundary_chain(chain):
                continue
            boundary_components.append({
                'key': key,
                'edges': set(boundary_component),
            })

        candidates.append({
            'strip_faces': set(strip_faces),
            'rings': rings_data,
            'extra_faces': extra_faces,
            'face_count': len(strip_faces),
            'boundary_components': boundary_components,
            **_strip_length_metrics(rings_data),
            **_component_span_metrics(face_component, rings_data),
        })

    return sorted(
        candidates,
        key=_strip_candidate_sort_key,
        reverse=True,
    )


def _boundary_path_components(edges):
    live_edges = {
        edge for edge in edges
        if edge is not None and getattr(edge, "is_valid", False)
    }
    if not live_edges:
        return []

    adjacency = {}
    for edge in live_edges:
        vert_a, vert_b = edge.verts
        adjacency.setdefault(vert_a, []).append(edge)
        adjacency.setdefault(vert_b, []).append(edge)

    branch_verts = {
        vert for vert, linked_edges in adjacency.items()
        if len(linked_edges) != 2
    }

    visited_edges = set()
    path_components = []

    def _walk_path(start_vert, start_edge):
        component = []
        current_vert = start_vert
        current_edge = start_edge

        while current_edge is not None and current_edge not in visited_edges:
            visited_edges.add(current_edge)
            component.append(current_edge)
            next_vert = current_edge.other_vert(current_vert)
            if next_vert in branch_verts:
                break

            next_edge = next(
                (
                    edge for edge in adjacency.get(next_vert, [])
                    if edge is not current_edge and edge not in visited_edges
                ),
                None,
            )
            current_vert = next_vert
            current_edge = next_edge

        if component:
            path_components.append(set(component))

    for vert in sorted(branch_verts, key=lambda item: item.index):
        for edge in sorted(adjacency.get(vert, []), key=lambda item: item.index):
            if edge in visited_edges:
                continue
            _walk_path(vert, edge)

    for edge in sorted(live_edges, key=lambda item: item.index):
        if edge in visited_edges:
            continue
        cycle_component = set()
        stack = [edge]
        visited_edges.add(edge)
        while stack:
            current = stack.pop()
            cycle_component.add(current)
            for vert in current.verts:
                for neighbor in adjacency.get(vert, []):
                    if neighbor in visited_edges:
                        continue
                    visited_edges.add(neighbor)
                    stack.append(neighbor)
        if cycle_component:
            path_components.append(cycle_component)

    return path_components


def _shaft_corner_boundary_edges(shaft_faces, corner_faces):
    split_edges = set()
    for corner_face in corner_faces:
        for edge in corner_face.edges:
            if any(
                other_face is not corner_face and other_face in shaft_faces
                for other_face in edge.link_faces
            ):
                split_edges.add(edge)
    return split_edges


def _boundary_tip_verts(split_edges):
    vert_counts = {}
    for edge in split_edges:
        if not getattr(edge, "is_valid", False):
            continue
        for vert in edge.verts:
            vert_counts[vert] = vert_counts.get(vert, 0) + 1

    return {
        vert for vert, count in vert_counts.items()
        if count == 1 and getattr(vert, "is_valid", False)
    }


def _ordered_chain_verts(edges):
    adjacency = {}
    for edge in edges:
        if not getattr(edge, "is_valid", False):
            continue
        vert_a, vert_b = edge.verts
        adjacency.setdefault(vert_a, []).append(vert_b)
        adjacency.setdefault(vert_b, []).append(vert_a)

    if not adjacency:
        return []

    endpoints = [
        vert for vert, neighbors in adjacency.items()
        if len(neighbors) == 1
    ]
    if endpoints:
        start = min(endpoints, key=lambda vert: vert.index)
    else:
        start = min(adjacency, key=lambda vert: vert.index)

    ordered = [start]
    visited = {start}
    current = start

    while len(ordered) < len(adjacency):
        next_vert = None
        for neighbor in sorted(adjacency[current], key=lambda vert: vert.index):
            if neighbor not in visited:
                next_vert = neighbor
                break
        if next_vert is None:
            break
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered


def _boundary_chain_verts(split_edges):
    verts = set()
    for edge in split_edges:
        if not getattr(edge, "is_valid", False):
            continue
        for vert in edge.verts:
            if getattr(vert, "is_valid", False):
                verts.add(vert)
    return verts


def _is_corner_boundary_chain(chain):
    if len(chain) < 2:
        return False

    last_index = len(chain) - 1
    for index, vert in enumerate(chain):
        if not getattr(vert, "is_valid", False):
            return False

        edge_count = len(vert.link_edges)
        if index == 0 or index == last_index:
            if edge_count not in {2, 3}:
                return False
        elif edge_count != 4:
            return False

    return True


def _split_control_verts(split_edges):
    tip_verts = _boundary_tip_verts(split_edges)
    control_verts = set()
    for vert in _boundary_chain_verts(split_edges):
        if not getattr(vert, "is_valid", False):
            continue

        edge_count = len(vert.link_edges)
        if vert in tip_verts:
            if edge_count in {2, 3}:
                control_verts.add(vert)
        elif edge_count == 4:
            control_verts.add(vert)

    return control_verts


def _tip_shaft_rail_edges(split_edges, shaft_faces, corner_faces):
    extra_edges = set()
    for vert in _boundary_tip_verts(split_edges):
        for edge in vert.link_edges:
            if edge in split_edges or not getattr(edge, "is_valid", False):
                continue

            linked_faces = {
                face for face in edge.link_faces
                if getattr(face, "is_valid", False)
            }
            if not (linked_faces & shaft_faces):
                continue
            if linked_faces & corner_faces:
                continue

            extra_edges.add(edge)

    return extra_edges


def _select_only_faces(bm, selected_faces):
    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    face_set = {
        face for face in selected_faces
        if face is not None and getattr(face, "is_valid", False)
    }
    for face in face_set:
        face.select = True
        for edge in face.edges:
            if edge.is_valid:
                edge.select = True
        for vert in face.verts:
            if vert.is_valid:
                vert.select = True


def _select_only_edges(bm, selected_edges):
    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    edge_set = {
        edge for edge in selected_edges
        if edge is not None and getattr(edge, "is_valid", False)
    }
    for edge in edge_set:
        edge.select = True
        for vert in edge.verts:
            if vert.is_valid:
                vert.select = True


def _select_only_cross_section(bm, seam_record):
    boundary_edges = {
        edge for edge in seam_record.get('boundary_edges', set())
        if edge is not None and getattr(edge, "is_valid", False)
    }
    _select_only_edges(bm, boundary_edges)


def _subtract_live_faces(face_set, removed_faces):
    removed = {
        face for face in removed_faces
        if face is not None and getattr(face, "is_valid", False)
    }
    return {
        face for face in face_set
        if face is not None and getattr(face, "is_valid", False) and face not in removed
    }


def _canonical_chain_key(chain):
    ids = tuple(
        vert.index for vert in chain
        if vert is not None and getattr(vert, "is_valid", False)
    )
    if not ids:
        return None
    reversed_ids = tuple(reversed(ids))
    return min(ids, reversed_ids)


def _all_seam_records(bm):
    selected_verts = {
        vert for vert in bm.verts
        if getattr(vert, "is_valid", False) and vert.select
    }
    selected_faces = _selected_face_set(bm, selected_verts)
    if not selected_faces:
        return []

    seam_records = {}
    all_shaft_faces = set()
    for face_component in _face_components(selected_faces):
        candidates = _open_face_strip_candidates(face_component)
        if not candidates:
            _debug_step("candidate filter", component_face_count=len(face_component), kept=0)
            continue

        chosen_candidates = []
        occupied_faces = set()
        for candidate in candidates:
            strip_faces = {
                face for face in candidate.get('strip_faces', set())
                if face is not None and getattr(face, "is_valid", False)
            }
            if not strip_faces or strip_faces & occupied_faces:
                continue
            chosen_candidates.append(candidate)
            occupied_faces.update(strip_faces)

        _debug_step(
            "candidate filter",
            component_face_count=len(face_component),
            kept=len(chosen_candidates),
        )
        for candidate in chosen_candidates:
            all_shaft_faces.update({
                face for face in candidate.get('strip_faces', set())
                if face is not None and getattr(face, "is_valid", False)
            })

        seam_entries = {}
        for candidate_index, candidate in enumerate(chosen_candidates, start=1):
            shaft_faces = {
                face for face in candidate.get('strip_faces', set())
                if face is not None and getattr(face, "is_valid", False)
            }
            corner_faces = {
                face for face in candidate.get('extra_faces', set())
                if face is not None and getattr(face, "is_valid", False)
            }
            if not shaft_faces or not corner_faces:
                continue

            boundary_components = candidate.get('boundary_components', [])
            _debug_step(
                "candidate seams",
                component_face_count=len(face_component),
                candidate_index=candidate_index,
                shaft_face_count=len(shaft_faces),
                corner_face_count=len(corner_faces),
                seam_count=len(boundary_components),
            )
            for boundary_component in boundary_components:
                key = boundary_component['key']
                record = seam_entries.setdefault(
                    key,
                    {
                        'boundary_edges': set(),
                        'candidate_ids': set(),
                        'candidate_data': {},
                    },
                )
                record['boundary_edges'].update(boundary_component['edges'])
                record['candidate_ids'].add(candidate_index)
                record['candidate_data'][candidate_index] = {
                    'shaft_faces': set(shaft_faces),
                    'corner_faces': set(corner_faces),
                    'sort_key': _strip_candidate_sort_key(candidate),
                }
        _debug_step(
            "candidate chain",
            component_face_count=len(face_component),
            path=[candidate_index for candidate_index in range(1, len(chosen_candidates) + 1)],
        )

        component_strong_records = {}
        component_fallback_records = {}
        for seam_key, seam_entry in seam_entries.items():
            representative_id = max(
                seam_entry['candidate_ids'],
                key=lambda candidate_id: seam_entry['candidate_data'][candidate_id]['sort_key'],
            )
            representative = seam_entry['candidate_data'][representative_id]
            record = {
                'key': seam_key,
                'boundary_edges': set(seam_entry['boundary_edges']),
                'shaft_faces': set(representative['shaft_faces']),
                'corner_faces': set(representative['corner_faces']),
                'support': len(seam_entry['candidate_ids']),
                'shaft_face_count': len(representative['shaft_faces']),
                'sort_key': representative['sort_key'],
            }
            if record['support'] >= 2:
                component_strong_records[seam_key] = record
            else:
                component_fallback_records[seam_key] = record

        chosen_component_records = (
            component_strong_records
            if component_strong_records else component_fallback_records
        )
        seam_records.update(chosen_component_records)

    filtered_records = sorted(
        seam_records.values(),
        key=lambda item: (
            -item['support'],
            tuple(-value if isinstance(value, (int, float)) else value for value in item.get('sort_key', ())),
            item['key'],
        )
    )
    return filtered_records, _face_positions(all_shaft_faces)


def _build_vert_position_lookup(bm, tolerance=_POSITION_TOLERANCE):
    lookup = {}
    for vert in bm.verts:
        if not getattr(vert, "is_valid", False):
            continue
        lookup.setdefault(_position_key(vert.co, tolerance), []).append(vert)
    return lookup


def _split_infos_from_boundary_components(bm, boundary_components, vert_lookup=None):
    split_infos = []
    for component in boundary_components:
        boundary_edges = component.get('boundary_edges', set())
        chain = _ordered_chain_verts(boundary_edges)
        if not chain:
            continue

        positions = [vert.co.copy() for vert in chain]
        clusters = []
        for position in positions:
            cluster = _verts_at_position(bm, position, lookup=vert_lookup)
            if len(cluster) >= 2:
                clusters.append({
                    'position': position.copy(),
                    'verts': cluster,
                })

        if clusters:
            split_infos.append({
                'anchor_home': positions[0].copy(),
                'clusters': clusters,
            })

    return split_infos


def _same_position(vec_a, vec_b, tolerance=_POSITION_TOLERANCE):
    return (vec_a - vec_b).length <= tolerance


def _group_loops(group_data):
    if isinstance(group_data, dict):
        rings = group_data.get('rings')
        if rings is None:
            return []
        return list(rings[0])
    return list(group_data[0])


def _tip_neighbor_edge(edge, tip_position, neighbor_position):
    if not getattr(edge, "is_valid", False):
        return False
    vert_a, vert_b = edge.verts
    return (
        (_same_position(vert_a.co, tip_position) and _same_position(vert_b.co, neighbor_position))
        or (_same_position(vert_b.co, tip_position) and _same_position(vert_a.co, neighbor_position))
    )


def _separate_stuck_boundary_tips(bm, boundary_components, seam_index=None):
    changed = False

    for component_index, component in enumerate(boundary_components, start=1):
        boundary_edges = component.get('boundary_edges', set())
        corner_faces = {
            face for face in component.get('corner_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        chain = _ordered_chain_verts(boundary_edges)
        if len(chain) < 2:
            continue

        tip_specs = (
            (chain[0].co.copy(), chain[1].co.copy()),
            (chain[-1].co.copy(), chain[-2].co.copy()),
        )

        for tip_position, neighbor_position in tip_specs:
            live_tip_verts = _verts_at_position(bm, tip_position)
            _debug_step(
                "tip inspect",
                seam_index=seam_index,
                component_index=component_index,
                live_tip_count=len(live_tip_verts),
                corner_face_count=len(corner_faces),
            )
            if len(live_tip_verts) != 1:
                continue

            tip_vert = live_tip_verts[0]
            seam_edges = [
                edge for edge in tip_vert.link_edges
                if _same_position(edge.other_vert(tip_vert).co, neighbor_position)
            ]
            _debug_step(
                "tip seam edges",
                seam_index=seam_index,
                component_index=component_index,
                seam_edge_count=len(seam_edges),
                edge_count=len(tip_vert.link_edges),
            )
            if not seam_edges:
                continue

            separated = bmesh.utils.vert_separate(tip_vert, seam_edges)
            _debug_step(
                "tip vert separate",
                seam_index=seam_index,
                component_index=component_index,
                result_count=len(separated),
            )
            if len(separated) > 1:
                changed = True
                continue

            live_tip_verts = _verts_at_position(bm, tip_position)
            if len(live_tip_verts) != 1:
                continue

            face_tip_vert = live_tip_verts[0]
            candidate_loops = [
                loop for loop in face_tip_vert.link_loops
                if getattr(loop.face, "is_valid", False) and loop.face in corner_faces
            ]
            if not candidate_loops:
                candidate_loops = list(face_tip_vert.link_loops)

            _debug_step(
                "tip loop fallback",
                seam_index=seam_index,
                component_index=component_index,
                loop_count=len(candidate_loops),
            )
            for loop in candidate_loops:
                live_tip_verts = _verts_at_position(bm, tip_position)
                if len(live_tip_verts) > 1:
                    break
                try:
                    bmesh.utils.loop_separate(loop)
                    _debug_step(
                        "tip loop separated",
                        seam_index=seam_index,
                        component_index=component_index,
                        face_index=getattr(loop.face, "index", None),
                        live_tip_count=len(_verts_at_position(bm, tip_position)),
                    )
                    changed = True
                except Exception:
                    _debug_step(
                        "tip loop failed",
                        seam_index=seam_index,
                        component_index=component_index,
                        face_index=getattr(loop.face, "index", None),
                    )
                    continue

    if changed:
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.select_flush_mode()
        bm.normal_update()

    return changed


def _split_seam_records(bm, seam_records):
    if not seam_records:
        return []

    boundary_components = []
    interface_chain_verts = set()
    split_edges = set()
    for record in seam_records:
        boundary_edges = {
            edge for edge in record.get('boundary_edges', set())
            if edge is not None and getattr(edge, "is_valid", False)
        }
        shaft_faces = {
            face for face in record.get('shaft_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        corner_faces = {
            face for face in record.get('corner_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        if not boundary_edges or not shaft_faces or not corner_faces:
            continue

        boundary_components.append({
            'boundary_edges': set(boundary_edges),
            'corner_faces': set(corner_faces),
        })
        interface_chain_verts.update(_split_control_verts(boundary_edges))
        split_edges.update(boundary_edges)
        split_edges.update(_tip_shaft_rail_edges(boundary_edges, shaft_faces, corner_faces))

    if not split_edges or not boundary_components:
        return []

    bmesh.ops.split_edges(
        bm,
        edges=list(split_edges),
        verts=list(interface_chain_verts),
        use_verts=bool(interface_chain_verts),
    )
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    _separate_stuck_boundary_tips(bm, boundary_components)
    bm.select_flush_mode()
    bm.normal_update()
    vert_lookup = _build_vert_position_lookup(bm)
    return _split_infos_from_boundary_components(bm, boundary_components, vert_lookup=vert_lookup)


def _split_single_seam_record(bm, seam_record, seam_index=None):
    boundary_edges = {
        edge for edge in seam_record.get('boundary_edges', set())
        if edge is not None and getattr(edge, "is_valid", False)
    }
    shaft_faces = {
        face for face in seam_record.get('shaft_faces', set())
        if face is not None and getattr(face, "is_valid", False)
    }
    corner_faces = {
        face for face in seam_record.get('corner_faces', set())
        if face is not None and getattr(face, "is_valid", False)
    }
    if not boundary_edges or not shaft_faces or not corner_faces:
        return False

    split_edges = set(boundary_edges)
    split_edges.update(_tip_shaft_rail_edges(boundary_edges, shaft_faces, corner_faces))
    interface_chain_verts = _split_control_verts(boundary_edges)
    boundary_component = {
        'boundary_edges': set(boundary_edges),
        'corner_faces': set(corner_faces),
    }

    bmesh.ops.split_edges(
        bm,
        edges=list(split_edges),
        verts=list(interface_chain_verts),
        use_verts=bool(interface_chain_verts),
    )
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    _separate_stuck_boundary_tips(bm, [boundary_component], seam_index=seam_index)
    bm.select_flush_mode()
    bm.normal_update()
    vert_lookup = _build_vert_position_lookup(bm)
    return _split_infos_from_boundary_components(bm, [boundary_component], vert_lookup=vert_lookup)


def _verts_at_position(bm, position, tolerance=_POSITION_TOLERANCE, lookup=None):
    if lookup is None:
        lookup = _build_vert_position_lookup(bm, tolerance=tolerance)

    return [
        vert for vert in lookup.get(_position_key(position, tolerance), [])
        if getattr(vert, "is_valid", False)
        and (vert.co - position).length <= tolerance
    ]


def _boundary_positions_from_records(seam_records):
    positions = []
    seen = set()
    for record in seam_records:
        boundary_edges = {
            edge for edge in record.get('boundary_edges', set())
            if edge is not None and getattr(edge, "is_valid", False)
        }
        if not boundary_edges:
            continue
        chain = _ordered_chain_verts(boundary_edges)
        for vert in chain:
            if not getattr(vert, "is_valid", False):
                continue
            pos = vert.co.copy()
            key = (round(pos.x, 6), round(pos.y, 6), round(pos.z, 6))
            if key not in seen:
                seen.add(key)
                positions.append(pos)
    return positions


def _weld_boundary_positions(bm, positions):
    if not positions:
        return 0

    vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for position in positions:
        live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
        if len(live_verts) < 2:
            continue
        target = live_verts[0]
        for vert in live_verts[1:]:
            if vert is not target:
                targetmap[vert] = target

    if not targetmap:
        _debug_step("weld skip", reason="no duplicate verts at boundary positions")
        return 0

    try:
        bmesh.ops.weld_verts(bm, targetmap=targetmap)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.normal_update()
        _debug_step("weld done", merged=len(targetmap))
        return len(targetmap)
    except Exception:
        _debug_step("weld failed", targetmap_size=len(targetmap))
        return 0


def _cluster_loop_match_count(loop, split_info):
    loop_set = {vert for vert in loop if getattr(vert, "is_valid", False)}
    return sum(
        1
        for cluster in split_info['clusters']
        if any(vert in loop_set for vert in cluster['verts'] if getattr(vert, "is_valid", False))
    )


def _open_group_matches(groups, split_info):
    matches = []
    for group_index, group_data in enumerate(groups or [], start=1):
        loops = _group_loops(group_data)
        best_match_count = 0
        best_loop_index = None
        best_loop_size = 0
        for loop_index, loop in enumerate(loops, start=1):
            match_count = _cluster_loop_match_count(loop, split_info)
            if match_count <= 0:
                continue
            loop_size = sum(
                1 for vert in loop
                if getattr(vert, "is_valid", False)
            )
            candidate = (match_count, loop_size, -loop_index)
            if (
                best_loop_index is None
                or candidate > (best_match_count, best_loop_size, -best_loop_index)
            ):
                best_match_count = match_count
                best_loop_index = loop_index
                best_loop_size = loop_size
        if best_loop_index is None:
            continue
        matches.append({
            'group_index': group_index,
            'loop_index': best_loop_index,
            'loop_size': best_loop_size,
            'matched_positions': best_match_count,
        })
    return matches


def _anchor_index_for_open_loop(loop, anchor_home):
    if not loop:
        return None
    start_dist = (loop[0].co - anchor_home).length if getattr(loop[0], "is_valid", False) else None
    end_dist = (loop[-1].co - anchor_home).length if getattr(loop[-1], "is_valid", False) else None
    if start_dist is None and end_dist is None:
        return None
    if end_dist is not None and (start_dist is None or end_dist < start_dist):
        return -1
    return 0


def _reverse_loop_with_fixed_start(loop):
    if len(loop) <= 1:
        return list(loop)
    return [loop[0]] + list(reversed(loop[1:]))


def _pair_distance_score(loop_a, loop_b):
    if len(loop_a) != len(loop_b):
        return None
    return sum(
        (vert_a.co - vert_b.co).length
        for vert_a, vert_b in zip(loop_a, loop_b)
        if getattr(vert_a, "is_valid", False) and getattr(vert_b, "is_valid", False)
    )


def _live_open_loop_pair(groups, split_info):
    matches = _open_group_matches(groups, split_info)
    if len(matches) < 2:
        return None

    matches = sorted(
        matches,
        key=lambda item: (
            item['matched_positions'],
            item['loop_size'],
            -item['group_index'],
            -item['loop_index'],
        ),
        reverse=True,
    )
    first = matches[0]
    second = next(
        (item for item in matches[1:] if item['group_index'] != first['group_index']),
        None,
    )
    if second is None:
        return None

    pair_loops = []
    for match in (first, second):
        group_data = groups[match['group_index'] - 1]
        loops = _group_loops(group_data)
        loop = list(loops[match['loop_index'] - 1])
        if not loop:
            return None
        anchor_index = _anchor_index_for_open_loop(loop, split_info['anchor_home'])
        if anchor_index == -1:
            loop = list(reversed(loop))
        pair_loops.append(loop)

    loop_a, loop_b = pair_loops
    if len(loop_a) != len(loop_b):
        return None

    reversed_b = _reverse_loop_with_fixed_start(loop_b)
    forward_score = _pair_distance_score(loop_a, loop_b)
    reverse_score = _pair_distance_score(loop_a, reversed_b)
    if reverse_score is not None and (
        forward_score is None or reverse_score < forward_score
    ):
        loop_b = reversed_b

    return (loop_a, loop_b)


def _weld_live_open_loops(bm, split_infos):
    live_data = bridged_open_loop.detect(bm)
    groups = live_data.get('groups') if live_data else []
    if not groups:
        return 0

    targetmap = {}
    for split_info in split_infos:
        pair = _live_open_loop_pair(groups, split_info)
        if pair is None:
            continue
        loop_a, loop_b = pair
        for vert_a, vert_b in zip(loop_a, loop_b):
            if (
                not getattr(vert_a, "is_valid", False)
                or not getattr(vert_b, "is_valid", False)
                or vert_a is vert_b
            ):
                continue
            targetmap[vert_b] = vert_a

    if not targetmap:
        return 0

    try:
        bmesh.ops.weld_verts(bm, targetmap=targetmap)
    except Exception:
        return 0

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    return len(targetmap)


def _targetmap_from_split_clusters(split_infos):
    targetmap = {}
    for split_info in split_infos:
        for cluster in split_info['clusters']:
            valid_verts = [
                vert for vert in cluster['verts']
                if getattr(vert, "is_valid", False)
            ]
            if len(valid_verts) < 2:
                continue
            target = valid_verts[0]
            for vert in valid_verts[1:]:
                if vert is not target:
                    targetmap[vert] = target
    return targetmap


def _targetmap_from_live_split_positions(bm, split_infos):
    vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for split_info in split_infos:
        for cluster in split_info['clusters']:
            live_verts = _verts_at_position(bm, cluster['position'], lookup=vert_lookup)
            if len(live_verts) < 2:
                continue
            target = live_verts[0]
            for vert in live_verts[1:]:
                if vert is not target:
                    targetmap[vert] = target
    return targetmap


def _anchor_open_group_endpoints(rings_data):
    loops, _is_closed = rings_data
    forced_seam_verts = []
    for loop in loops:
        seam_verts = set()
        if loop:
            first_vert = loop[0]
            last_vert = loop[-1]
            if getattr(first_vert, "is_valid", False):
                seam_verts.add(first_vert)
            if getattr(last_vert, "is_valid", False):
                seam_verts.add(last_vert)
        forced_seam_verts.append(seam_verts)

    return {
        'rings': rings_data,
        'use_seams': True,
        'migrate_seams': False,
        'max_seams': 2,
        'forced_seam_verts': forced_seam_verts,
    }


def _seam_records_from_components(components):
    seam_records = {}
    all_shaft_faces = set()

    for component in components or []:
        component_faces = {
            face for face in component.get('component_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        shaft_faces = {
            face for face in component.get('shaft_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        corner_faces = {
            face for face in component.get('extra_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        group = component.get('group')

        if not shaft_faces:
            continue

        all_shaft_faces.update(shaft_faces)

        if not corner_faces or group is None:
            continue

        candidate_metrics = {
            **_strip_length_metrics(group),
            **_component_span_metrics(component_faces or shaft_faces, group),
            'extra_faces': corner_faces,
        }
        sort_key = _strip_candidate_sort_key(candidate_metrics)
        boundary_edges = _shaft_corner_boundary_edges(shaft_faces, corner_faces)

        for boundary_component in _boundary_path_components(boundary_edges):
            chain = _ordered_chain_verts(boundary_component)
            key = _canonical_chain_key(chain)
            if key is None or not _is_corner_boundary_chain(chain):
                continue

            existing = seam_records.get(key)
            record = {
                'key': key,
                'boundary_edges': set(boundary_component),
                'shaft_faces': set(shaft_faces),
                'corner_faces': set(corner_faces),
                'support': 1,
                'shaft_face_count': len(shaft_faces),
                'sort_key': sort_key,
            }
            if existing is None or record['sort_key'] > existing.get('sort_key', ()):
                seam_records[key] = record

    filtered_records = sorted(
        seam_records.values(),
        key=lambda item: (
            -item['support'],
            tuple(-value if isinstance(value, (int, float)) else value for value in item.get('sort_key', ())),
            item['key'],
        )
    )
    return filtered_records, _face_positions(all_shaft_faces)


def detect(bm):
    data = detect_open_strip_selection(bm)
    if data is None:
        return None
    if not _groups_are_open(data['groups']):
        return None
    if not (
        data['has_extra_selected_faces']
        or data['has_outside_side_faces']
    ):
        return None

    return {
        'groups': data['groups'],
        'components': data.get('components', []),
        'mode_label': 'Bridged open loop with corners',
    }


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    original_faces = _selected_faces(bm)
    original_face_indices = _face_indices(original_faces)
    original_face_positions = _face_positions(original_faces)
    seam_records, shaft_face_positions = _all_seam_records(bm)
    if not seam_records:
        seam_records, shaft_face_positions = _seam_records_from_components(
            data.get('components', []) if isinstance(data, dict) else []
        )
    if not seam_records:
        if report is not None:
            report({'ERROR'}, "Could not find open bridge corner seam edges.")
        return {'CANCELLED'}

    # --- Pre-split bookkeeping ---
    boundary_positions = _boundary_positions_from_records(seam_records)
    corner_face_positions = []
    for record in seam_records:
        corner_face_positions.extend(_face_positions(record.get('corner_faces', set())))

    # --- Split seams ---
    total_split_seams = 0
    split_infos = []
    for seam_index, seam_record in enumerate(seam_records, start=1):
        _debug_step(
            "seam split begin",
            seam_index=seam_index,
            support=seam_record['support'],
            boundary_edge_count=len(seam_record['boundary_edges']),
        )
        seam_split_infos = _split_single_seam_record(bm, seam_record, seam_index=seam_index)
        if seam_split_infos:
            total_split_seams += 1
            split_infos.extend(seam_split_infos)
            continue
        _debug_step("seam split failed", seam_index=seam_index)

    if total_split_seams == 0:
        if report is not None:
            report({'ERROR'}, "Could not split any open bridge corner cross sections.")
        return {'CANCELLED'}

    _debug_step(
        "split summary",
        seam_record_count=len(seam_records),
        total_split_seams=total_split_seams,
    )

    # --- Select only shaft faces and detect open loops per component ---
    face_lookup = _build_face_position_lookup(bm)
    shaft_faces = _live_faces_from_positions(bm, shaft_face_positions, lookup=face_lookup)
    _debug_step("shaft faces", count=len(shaft_faces))
    _select_only_faces(bm, shaft_faces)
    fresh_data = detect_open_strip_selection(bm)
    if (
        fresh_data is None
        or not _groups_are_open(fresh_data.get('groups', []))
        or fresh_data.get('has_extra_selected_faces')
        or fresh_data.get('has_outside_side_faces')
    ):
        open_groups = []
    else:
        open_groups = list(fresh_data['groups'])
    _debug_step("post-split filter", raw_count=len(open_groups), kept=len(open_groups))

    if not open_groups:
        _debug_step("post-split detection failed")
        if report is not None:
            report({'ERROR'}, "Could not detect open loop bridges after corner split.")
        _weld_boundary_positions(bm, boundary_positions)
        bmesh.update_edit_mesh(obj.data)
        return {'CANCELLED'}

    _debug_step("post-split detection", group_count=len(open_groups))

    # --- Execute open loop bridge resampling ---
    result = {'CANCELLED'}
    try:
        anchored_groups = [
            _anchor_open_group_endpoints(rings_data)
            for rings_data in open_groups
        ]
        for group_data in anchored_groups:
            group_result = execute_aligned_loops_logic(
                bm,
                obj,
                group_data['rings'],
                direction,
                report=report,
                use_seams=group_data['use_seams'],
                migrate_seams=group_data['migrate_seams'],
                max_seams=group_data['max_seams'],
                forced_seam_verts=group_data['forced_seam_verts'],
            )
            if group_result != {'FINISHED'}:
                result = group_result
        if result == {'CANCELLED'}:
            result = {'FINISHED'}
    finally:
        welded = _weld_live_open_loops(bm, split_infos)
        if welded == 0:
            welded = _weld_boundary_positions(bm, boundary_positions)
        if welded == 0:
            targetmap = _targetmap_from_live_split_positions(bm, split_infos)
            if not targetmap:
                targetmap = _targetmap_from_split_clusters(split_infos)
            if targetmap:
                try:
                    bmesh.ops.weld_verts(bm, targetmap=targetmap)
                    bm.verts.ensure_lookup_table()
                    bm.edges.ensure_lookup_table()
                    bm.faces.ensure_lookup_table()
                    bm.normal_update()
                except Exception:
                    pass
        current_shaft_faces = _selected_faces(bm)
        face_lookup = _build_face_position_lookup(bm)
        live_corner_faces = _live_faces_from_positions(bm, corner_face_positions, lookup=face_lookup)

        restored_seed_faces = set(current_shaft_faces) | set(live_corner_faces)
        if not restored_seed_faces:
            restored_seed_faces = _live_faces_from_positions(bm, original_face_positions, lookup=face_lookup)
        if not restored_seed_faces:
            restored_seed_faces = _live_faces_from_indices(bm, original_face_indices)

        restored_faces = _connected_quad_faces(restored_seed_faces)
        if not restored_faces:
            restored_faces = restored_seed_faces

        if restored_faces:
            _select_only_faces(bm, restored_faces)
            bm.select_flush_mode()
            bm.normal_update()
        bmesh.update_edit_mesh(obj.data)

    if report is not None:
        if result == {'FINISHED'}:
            report({'INFO'}, f"Open loop bridge with corners finished: seams={total_split_seams}.")
        else:
            report({'ERROR'}, "Open loop bridge with corners: resampling failed.")

    return result
