# closed_loop_bridged.py — Resample bridged closed-loop selections.

from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
from .resample_common import execute_aligned_loops_logic


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
        return {'CANCELLED'}

    for group_data in data['groups']:
        if isinstance(group_data, dict):
            execute_aligned_loops_logic(
                bm,
                obj,
                group_data['rings'],
                direction,
                report=report,
                use_seams=group_data.get('use_seams', True),
                migrate_seams=group_data.get('migrate_seams'),
                max_seams=group_data.get('max_seams'),
            )
            continue

        execute_aligned_loops_logic(bm, obj, group_data, direction, report=report)
    return {'FINISHED'}
