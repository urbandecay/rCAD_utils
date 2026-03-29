# hole_punch_solid.py — Detect punched holes that pass through solid mesh.

from ..debug import debug_log, loop_ref, mesh_stats


def _selected_face_set(bm, sel_set):
    return {
        face for face in bm.faces
        if len(face.verts) >= 3 and all(vert in sel_set for vert in face.verts)
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
        has_selected_neighbor = any(
            other_face is not face and other_face in selected_faces
            for other_face in edge.link_faces
        )
        if has_selected_neighbor:
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
    if loops is None:
        return None
    if len(loops) != 2:
        return None

    ring0, ring1 = loops
    if len(ring0) != len(ring1):
        return None
    if len(ring0) < 3:
        return None

    ring1_aligned = _aligned_partner_loop(ring0, ring1, edge_face_counts)
    if ring1_aligned is not None:
        return ([ring0, ring1_aligned], True)

    ring0_aligned = _aligned_partner_loop(ring1, ring0, edge_face_counts)
    if ring0_aligned is not None:
        return ([ring0_aligned, ring0], True)

    return None


def _quad_faces_for_edge(edge):
    return [face for face in edge.link_faces if len(face.verts) == 4]


def _is_selected_ring_edge(edge):
    if len(_quad_faces_for_edge(edge)) != 1:
        return False
    if len(edge.link_faces) < 2:
        return False
    return any(len(face.verts) != 4 for face in edge.link_faces)


def _connected_quad_component(seed_faces):
    if not seed_faces:
        return set()

    component = set()
    stack = sorted(seed_faces, key=lambda face: face.index)

    while stack:
        face = stack.pop()
        if face in component or len(face.verts) != 4:
            continue

        component.add(face)
        for edge in face.edges:
            for other_face in edge.link_faces:
                if other_face in component or len(other_face.verts) != 4:
                    continue
                stack.append(other_face)

    return component


def _match_selected_loop_pair(component, selected_ring):
    rings_data = _rings_from_component(component)
    if rings_data is None:
        return None

    loops, is_closed = rings_data
    if not is_closed or len(loops) != 2:
        return None

    selected_set = set(selected_ring)
    loop0_set = set(loops[0])
    loop1_set = set(loops[1])

    if selected_set == loop0_set:
        return rings_data
    if selected_set == loop1_set:
        return ([loops[1], loops[0]], is_closed)
    return None


def _upper_ring_hole_groups(bm):
    selected_edges = {edge for edge in bm.edges if edge.select}
    if not selected_edges:
        return None

    ring_edges = {
        edge for edge in selected_edges
        if _is_selected_ring_edge(edge)
    }
    if not ring_edges:
        return None

    selected_loops = _boundary_loops(ring_edges)
    if selected_loops is None:
        return {
            'groups': [],
            'invalid_components': 1,
        }

    groups = []
    invalid_components = 0
    covered_ring_edges = set()
    allowed_extra_edges = set()
    seen_components = set()

    for selected_ring in selected_loops:
        ring_set = set(selected_ring)
        local_ring_edges = {
            edge for edge in ring_edges
            if edge.verts[0] in ring_set and edge.verts[1] in ring_set
        }
        covered_ring_edges.update(local_ring_edges)

        local_cleanup_edges = {
            edge for edge in (selected_edges - ring_edges)
            if edge.verts[0] in ring_set or edge.verts[1] in ring_set
        }
        allowed_extra_edges.update(local_cleanup_edges)

        seed_faces = set()
        for edge in local_ring_edges:
            seed_faces.update(_quad_faces_for_edge(edge))

        component = _connected_quad_component(seed_faces)
        component_key = tuple(sorted(face.index for face in component))
        if component_key in seen_components:
            continue

        matched_rings = _match_selected_loop_pair(component, selected_ring)
        if matched_rings is None:
            invalid_components += 1
            continue

        seen_components.add(component_key)
        groups.append({
            'rings': matched_rings,
            'use_seams': True,
            'migrate_seams': True,
            'max_seams': 2,
            'cleanup_edges': {
                edge for edge in local_cleanup_edges if edge.is_valid
            },
            'cleanup_ring_edges': {
                edge for edge in local_ring_edges if edge.is_valid
            },
        })

    stray_edges = [
        edge for edge in selected_edges
        if edge.is_valid
        and edge not in covered_ring_edges
        and edge not in allowed_extra_edges
    ]
    if stray_edges:
        invalid_components += 1

    if any(edge.is_valid and edge not in covered_ring_edges for edge in ring_edges):
        invalid_components += 1

    return {
        'groups': groups,
        'invalid_components': invalid_components,
    }


def _shaft_face_hole_groups(bm):
    sel_verts = [vert for vert in bm.verts if vert.select]
    if len(sel_verts) < 4:
        return None

    sel_set = set(sel_verts)
    selected_faces = _selected_face_set(bm, sel_set)
    shaft_faces = {
        face for face in selected_faces
        if _is_shaft_face(face, selected_faces)
    }
    if not shaft_faces:
        debug_log(
            "hole_detect",
            "No shaft faces found from fully-selected face set.",
            mesh=mesh_stats(bm),
        )
        return None

    cylinder_pairs = []
    invalid_components = 0
    for component in _face_components(shaft_faces):
        rings_data = _rings_from_component(component)
        if rings_data is None:
            invalid_components += 1
            debug_log(
                "hole_detect",
                "Rejected shaft-face component because rings could not be extracted.",
                component_faces=[face.index for face in sorted(component, key=lambda face: face.index)],
            )
            continue
        debug_log(
            "hole_detect",
            "Detected punched-hole ring pair from selected shaft faces.",
            component_faces=[face.index for face in sorted(component, key=lambda face: face.index)],
            ring0=loop_ref(rings_data[0][0]),
            ring1=loop_ref(rings_data[0][1]),
        )
        cylinder_pairs.append({
            'rings': rings_data,
            'use_seams': True,
            'migrate_seams': True,
            'max_seams': 2,
        })

    return {
        'groups': cylinder_pairs,
        'invalid_components': invalid_components,
    }


def detect(bm):
    data = _shaft_face_hole_groups(bm)
    if data is not None and (data['groups'] or data['invalid_components']):
        debug_log(
            "hole_detect",
            "Using fully-selected shaft-face detection path.",
            groups=len(data['groups']),
            invalid_components=data['invalid_components'],
        )
        return data

    upper_ring_data = _upper_ring_hole_groups(bm)
    if upper_ring_data is not None and (
        upper_ring_data['groups'] or upper_ring_data['invalid_components']
    ):
        debug_log(
            "hole_detect",
            "Using upper-ring hole detection path.",
            groups=len(upper_ring_data['groups']),
            invalid_components=upper_ring_data['invalid_components'],
        )
        return upper_ring_data

    return None
