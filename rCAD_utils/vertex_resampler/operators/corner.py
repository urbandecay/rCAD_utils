# corner.py — Resample corner selections with extra faces on an open strip.

from .open_strip_common import detect_open_strip_selection
from .resample_common import execute_aligned_loops_logic


def _selected_face_set(bm, sel_set):
    return {
        face for face in bm.faces
        if len(face.verts) == 4 and (
            face.select
            or all(vert in sel_set for vert in face.verts)
            or all(edge.select for edge in face.edges)
        )
    }


def _selected_component_edges(component):
    edges = set()
    component_set = set(component)
    for vert in component:
        for edge in vert.link_edges:
            if edge.select and edge.other_vert(vert) in component_set:
                edges.add(edge)
    return list(edges)


def _fully_selected_faces(edge, sel_set):
    return sum(
        1 for face in edge.link_faces if all(vert in sel_set for vert in face.verts)
    )


def _order_path_from_edges(edges):
    adjacency = {}
    for edge in edges:
        v1, v2 = edge.verts
        adjacency.setdefault(v1, []).append(v2)
        adjacency.setdefault(v2, []).append(v1)

    endpoints = [vert for vert, neighbors in adjacency.items() if len(neighbors) == 1]
    if len(endpoints) != 2:
        return None

    ordered = [endpoints[0]]
    visited = {endpoints[0]}
    current = endpoints[0]

    while len(ordered) < len(adjacency):
        next_vert = None
        for neighbor in adjacency[current]:
            if neighbor not in visited:
                next_vert = neighbor
                break
        if next_vert is None:
            break
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered if len(ordered) == len(adjacency) else None


def _split_boundary_chains(boundary_edges, cut_edges):
    cut_set = set(cut_edges)
    remaining = [edge for edge in boundary_edges if edge not in cut_set]
    if not remaining:
        return None

    edge_neighbors = {}
    vert_edges = {}
    for edge in remaining:
        edge_neighbors[edge] = set()
        for vert in edge.verts:
            vert_edges.setdefault(vert, set()).add(edge)

    for edge in remaining:
        for vert in edge.verts:
            edge_neighbors[edge].update(vert_edges[vert])
        edge_neighbors[edge].discard(edge)

    visited = set()
    chains = []
    for edge in remaining:
        if edge in visited:
            continue
        stack = [edge]
        component_edges = []
        visited.add(edge)

        while stack:
            current = stack.pop()
            component_edges.append(current)
            for other in edge_neighbors[current]:
                if other in visited:
                    continue
                visited.add(other)
                stack.append(other)

        ordered = _order_path_from_edges(component_edges)
        if ordered is None:
            return None
        chains.append(ordered)

    return chains if len(chains) == 2 else None


def _edge_lookup(edges):
    return {frozenset(edge.verts): edge for edge in edges}


def _align_chain_pair(chain_a, chain_b, edge_map):
    for candidate in (chain_b, list(reversed(chain_b))):
        if len(candidate) != len(chain_a):
            continue
        if all(
            frozenset((va, vb)) in edge_map for va, vb in zip(chain_a, candidate)
        ):
            return candidate
    return None


def _detect_corner_component(component):
    component_set = set(component)
    component_edges = _selected_component_edges(component)
    if len(component_edges) < 4:
        return None

    selected_degree = {
        vert: sum(
            1
            for edge in vert.link_edges
            if edge.select and edge.other_vert(vert) in component_set
        )
        for vert in component
    }
    if any(degree < 2 or degree > 3 for degree in selected_degree.values()):
        return None

    boundary_edges = [
        edge for edge in component_edges
        if _fully_selected_faces(edge, component_set) == 1
    ]
    if len(boundary_edges) < 4:
        return None

    edge_map = _edge_lookup(component_edges)
    candidate_cut_edges = [
        edge for edge in boundary_edges
        if all(selected_degree[vert] == 2 for vert in edge.verts)
    ]
    if len(candidate_cut_edges) < 2:
        candidate_cut_edges = boundary_edges

    best_pair = None
    best_length = -1.0

    candidate_pairs = []
    if (
        len(candidate_cut_edges) == 2
        and not set(candidate_cut_edges[0].verts).intersection(
            candidate_cut_edges[1].verts
        )
    ):
        candidate_pairs.append((candidate_cut_edges[0], candidate_cut_edges[1]))
    else:
        for index, edge_a in enumerate(candidate_cut_edges):
            verts_a = set(edge_a.verts)
            for edge_b in candidate_cut_edges[index + 1:]:
                if verts_a.intersection(edge_b.verts):
                    continue
                candidate_pairs.append((edge_a, edge_b))

    for edge_a, edge_b in candidate_pairs:
        chains = _split_boundary_chains(boundary_edges, (edge_a, edge_b))
        if chains is None:
            continue

        chain_a, chain_b = chains
        if len(chain_a) != len(chain_b) or len(chain_a) < 2:
            continue

        aligned_b = _align_chain_pair(chain_a, chain_b, edge_map)
        if aligned_b is None:
            continue

        expected_edge_count = (2 * (len(chain_a) - 1)) + len(chain_a)
        if len(component_edges) != expected_edge_count:
            continue
        if len(boundary_edges) != 2 * len(chain_a):
            continue

        total_length = 0.0
        for idx in range(len(chain_a) - 1):
            total_length += (chain_a[idx].co - chain_a[idx + 1].co).length
            total_length += (aligned_b[idx].co - aligned_b[idx + 1].co).length

        if total_length > best_length:
            best_length = total_length
            best_pair = ([chain_a, aligned_b], False)

    return best_pair


def _legacy_corner_detect(bm):
    selected_verts = {vert for vert in bm.verts if vert.select}
    selected_faces = _selected_face_set(bm, selected_verts)
    if not selected_faces:
        return None

    selected_face_set = set(selected_faces)

    def face_neighbors(face, face_set):
        neighbors = set()
        for edge in face.edges:
            for other_face in edge.link_faces:
                if other_face is face or other_face not in face_set:
                    continue
                neighbors.add(other_face)
        return neighbors

    def face_components(faces):
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
                for neighbor in face_neighbors(current, faces):
                    if neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)
            components.append(component)
        return components

    def is_shaft_face(face, face_set):
        internal_edges = []
        for edge in face.edges:
            has_neighbor = any(
                other_face is not face and other_face in face_set
                for other_face in edge.link_faces
            )
            if has_neighbor:
                internal_edges.append(edge)

        if len(internal_edges) != 2:
            return False
        return not set(internal_edges[0].verts).intersection(internal_edges[1].verts)

    corner_groups = []
    covered_verts = set()

    for face_component in face_components(selected_face_set):
        shaft_faces = {
            face for face in face_component
            if is_shaft_face(face, face_component)
        }
        if not shaft_faces or shaft_faces == face_component:
            continue

        shaft_verts = []
        shaft_vert_seen = set()
        for face in sorted(shaft_faces, key=lambda item: item.index):
            for vert in face.verts:
                if vert in shaft_vert_seen:
                    continue
                shaft_vert_seen.add(vert)
                shaft_verts.append(vert)

        strip_group = _detect_corner_component(shaft_verts)
        if strip_group is None:
            continue

        corner_groups.append(strip_group)
        covered_verts.update(vert for face in face_component for vert in face.verts)

    if not corner_groups:
        return None

    if covered_verts != selected_verts:
        return None

    return {
        'groups': corner_groups,
        'mode_label': 'Corner',
    }


def detect(bm):
    print("[vertex_resampler:dispatch] checking Corner")
    data = detect_open_strip_selection(bm)
    if data is None:
        print("[vertex_resampler:dispatch] Corner rejected by shared detector")
        return None

    if data['has_extra_selected_faces'] or data['has_outside_side_faces']:
        print("[vertex_resampler:dispatch] Corner matched via shared detector")
        return {
            'groups': data['groups'],
            'mode_label': 'Corner',
        }

    print("[vertex_resampler:dispatch] Corner rejected by shared detector")
    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for group_data in data['groups']:
        execute_aligned_loops_logic(
            bm,
            obj,
            group_data,
            direction,
            report=report,
        )
    return {'FINISHED'}
