# open_strip_common.py — Shared detection for open-strip shaft selections.

from .bridge_utils import get_bridged_chain


_VECTOR_TOLERANCE = 1.0e-6


def _edge_between(vert_a, vert_b):
    for edge in vert_a.link_edges:
        if edge.other_vert(vert_a) is vert_b:
            return edge
    return None


def _loop_path_length(loop, is_closed):
    if len(loop) < 2:
        return 0.0

    pairs = list(zip(loop, loop[1:]))
    if is_closed:
        pairs.append((loop[-1], loop[0]))

    return sum(
        (vert_a.co - vert_b.co).length
        for vert_a, vert_b in pairs
    )


def _strip_length_metrics(strip_group):
    loops, is_closed = strip_group
    if not loops:
        return {
            'path_length': 0.0,
            'cross_length': 0.0,
            'path_cross_ratio': 0.0,
        }

    path_lengths = [_loop_path_length(loop, is_closed) for loop in loops if len(loop) >= 2]
    path_length = sum(path_lengths) / len(path_lengths) if path_lengths else 0.0

    cross_distances = []
    if len(loops) >= 2:
        pair_count = min(len(loop) for loop in loops)
        for index in range(pair_count):
            ring_positions = [loop[index].co for loop in loops]
            for pos_a, pos_b in zip(ring_positions, ring_positions[1:]):
                cross_distances.append((pos_a - pos_b).length)

    cross_length = sum(cross_distances) / len(cross_distances) if cross_distances else 0.0
    path_cross_ratio = (path_length / cross_length) if cross_length > 1.0e-12 else float('inf')
    return {
        'path_length': path_length,
        'cross_length': cross_length,
        'path_cross_ratio': path_cross_ratio,
    }


def _aligned_average_axis(vectors):
    accumulator = None
    reference = None

    for vec in vectors:
        if vec.length <= _VECTOR_TOLERANCE:
            continue

        current = vec.copy()
        if reference is None:
            reference = current.normalized()
        elif current.dot(reference) < 0.0:
            current.negate()

        accumulator = current if accumulator is None else (accumulator + current)

    if accumulator is None or accumulator.length <= _VECTOR_TOLERANCE:
        return None
    return accumulator.normalized()


def _span_metrics_from_verts(component_verts, strip_group):
    loops, is_closed = strip_group
    component_verts = {
        vert for vert in component_verts
        if getattr(vert, "is_valid", False)
    }
    if not component_verts:
        return {
            'component_path_span': 0.0,
            'component_cross_span': 0.0,
            'component_path_cross_ratio': 0.0,
        }

    path_vectors = []
    for loop in loops:
        if len(loop) < 2:
            continue
        pairs = list(zip(loop, loop[1:]))
        if is_closed:
            pairs.append((loop[-1], loop[0]))
        for vert_a, vert_b in pairs:
            path_vectors.append(vert_b.co - vert_a.co)

    cross_vectors = []
    if len(loops) >= 2:
        pair_count = min(len(loop) for loop in loops)
        for index in range(pair_count):
            ring_positions = [loop[index].co for loop in loops]
            for pos_a, pos_b in zip(ring_positions, ring_positions[1:]):
                cross_vectors.append(pos_b - pos_a)

    path_axis = _aligned_average_axis(path_vectors)
    cross_axis = _aligned_average_axis(cross_vectors)
    if path_axis is None or cross_axis is None:
        return {
            'component_path_span': 0.0,
            'component_cross_span': 0.0,
            'component_path_cross_ratio': 0.0,
        }

    # Keep the cross axis perpendicular enough to the path axis to measure width
    # across the whole face component instead of letting skew collapse the span.
    orthogonal_cross = cross_axis - (path_axis * cross_axis.dot(path_axis))
    if orthogonal_cross.length > _VECTOR_TOLERANCE:
        cross_axis = orthogonal_cross.normalized()

    path_projections = [vert.co.dot(path_axis) for vert in component_verts]
    cross_projections = [vert.co.dot(cross_axis) for vert in component_verts]
    path_span = max(path_projections) - min(path_projections)
    cross_span = max(cross_projections) - min(cross_projections)
    path_cross_ratio = (
        path_span / cross_span
        if cross_span > _VECTOR_TOLERANCE else float('inf')
    )

    return {
        'component_path_span': path_span,
        'component_cross_span': cross_span,
        'component_path_cross_ratio': path_cross_ratio,
    }


def _component_span_metrics(face_component, strip_group):
    component_verts = {
        vert
        for face in face_component
        for vert in face.verts
        if getattr(vert, "is_valid", False)
    }
    return _span_metrics_from_verts(component_verts, strip_group)


def _sortable_metric(value):
    if value == float('inf'):
        return float('inf')
    return round(value, 6)


def _strip_candidate_sort_key(candidate):
    return (
        _sortable_metric(candidate.get('component_path_span', 0.0)),
        _sortable_metric(candidate.get('component_path_cross_ratio', 0.0)),
        -_sortable_metric(candidate.get('component_cross_span', 0.0)),
        _sortable_metric(candidate['path_length']),
        _sortable_metric(candidate['path_cross_ratio']),
        -_sortable_metric(candidate['cross_length']),
        -len(candidate.get('extra_faces', ())),
    )


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


def _selected_components(bm):
    selected_verts = [vert for vert in bm.verts if vert.select]
    visited = set()
    components = []

    for vert in selected_verts:
        if vert in visited:
            continue
        stack = [vert]
        component = []
        visited.add(vert)

        while stack:
            current = stack.pop()
            component.append(current)
            for edge in current.link_edges:
                if not edge.select:
                    continue
                other = edge.other_vert(current)
                if other.select and other not in visited:
                    visited.add(other)
                    stack.append(other)

        if len(component) >= 2:
            components.append(component)

    return components


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
            current_verts = set(current.verts)
            for other in remaining:
                if other in visited:
                    continue
                if current_verts.intersection(other.verts):
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


def _detect_open_strip_component(component):
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
        edge
        for edge in component_edges
        if _fully_selected_faces(edge, component_set) == 1
    ]
    if len(boundary_edges) < 4:
        return None

    edge_map = _edge_lookup(component_edges)
    candidate_cut_edges = [
        edge
        for edge in boundary_edges
        if all(selected_degree[vert] == 2 for vert in edge.verts)
    ]
    if len(candidate_cut_edges) < 2:
        candidate_cut_edges = boundary_edges

    best_pair = None
    best_score = None

    for index, edge_a in enumerate(candidate_cut_edges):
        verts_a = set(edge_a.verts)
        for edge_b in candidate_cut_edges[index + 1:]:
            if verts_a.intersection(edge_b.verts):
                continue

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

            candidate_pair = ([chain_a, aligned_b], False)
            metrics = {
                **_strip_length_metrics(candidate_pair),
                **_span_metrics_from_verts(component_set, candidate_pair),
                'extra_faces': (),
            }
            score = _strip_candidate_sort_key(metrics)

            if best_score is None or score > best_score:
                best_score = score
                best_pair = candidate_pair

    return best_pair


def _selected_face_neighbors(face, selected_faces):
    neighbors = {}
    for edge in face.edges:
        neighbor = next(
            (
                other_face for other_face in edge.link_faces
                if other_face is not face
                and other_face in selected_faces
                and len(other_face.verts) == 4
            ),
            None,
        )
        if neighbor is not None:
            neighbors[edge] = neighbor
    return neighbors


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
                _selected_face_neighbors(current, faces).values(),
                key=lambda item: item.index,
            ):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

        components.append(component)

    return components


def _shared_edge(face_a, face_b):
    return next(
        (edge for edge in face_a.edges if face_b in edge.link_faces),
        None,
    )


def _opposite_edge(face, edge):
    if len(face.verts) != 4:
        return None

    edges = list(face.edges)
    try:
        index = edges.index(edge)
    except ValueError:
        return None
    return edges[(index + 2) % 4]


def _continue_strip_face(current_face, previous_face, selected_faces):
    shared = _shared_edge(current_face, previous_face)
    if shared is None:
        return None

    opposite = _opposite_edge(current_face, shared)
    if opposite is None:
        return None

    return _selected_face_neighbors(current_face, selected_faces).get(opposite)


def _trace_strip_direction(start_face, previous_face, selected_faces):
    path = [start_face]
    visited = {start_face}
    current = start_face
    prior = previous_face

    while True:
        next_face = _continue_strip_face(current, prior, selected_faces)
        if next_face is None or next_face in visited:
            break
        path.append(next_face)
        visited.add(next_face)
        prior = current
        current = next_face

    return path


def _max_strip_path(face_a, face_b, selected_faces):
    left_path = _trace_strip_direction(face_a, face_b, selected_faces)
    right_path = _trace_strip_direction(face_b, face_a, selected_faces)
    path = list(reversed(left_path)) + right_path

    seen = set()
    deduped = []
    for face in path:
        if face in seen:
            return None
        seen.add(face)
        deduped.append(face)

    return deduped


def _faces_to_component_verts(faces):
    ordered_verts = []
    seen_verts = set()

    for face in sorted(faces, key=lambda item: item.index):
        for vert in face.verts:
            if vert in seen_verts:
                continue
            seen_verts.add(vert)
            ordered_verts.append(vert)

    return ordered_verts


def _strip_candidates(face_component):
    candidates = []
    seen_paths = set()

    for face in sorted(face_component, key=lambda item: item.index):
        neighbor_map = _selected_face_neighbors(face, face_component)
        for neighbor in sorted(set(neighbor_map.values()), key=lambda item: item.index):
            shaft_faces = _max_strip_path(face, neighbor, face_component)
            if shaft_faces is None or len(shaft_faces) < 2:
                continue

            path_key = tuple(sorted(item.index for item in shaft_faces))
            if path_key in seen_paths:
                continue
            seen_paths.add(path_key)

            strip_group = _detect_open_strip_component(
                _faces_to_component_verts(shaft_faces)
            )
            if strip_group is None:
                continue

            outside_side_faces = set()
            loops, is_closed = strip_group
            for loop in loops:
                pairs = zip(loop, loop[1:])
                if is_closed:
                    pairs = list(pairs) + [(loop[-1], loop[0])]
                for vert_a, vert_b in pairs:
                    edge = _edge_between(vert_a, vert_b)
                    if edge is None:
                        continue
                    for face in edge.link_faces:
                        if face not in face_component:
                            outside_side_faces.add(face)

            shaft_face_set = set(shaft_faces)
            extra_faces = face_component - shaft_face_set
            candidates.append({
                'group': strip_group,
                'shaft_faces': shaft_face_set,
                'extra_faces': extra_faces,
                'outside_side_faces': outside_side_faces,
                'face_count': len(shaft_faces),
                'loop_size': len(strip_group[0][0]),
                **_strip_length_metrics(strip_group),
                **_component_span_metrics(face_component, strip_group),
            })

    return candidates


def _choose_strip_candidate(candidates):
    if not candidates:
        return None, "no candidates"

    exact_candidates = [
        candidate for candidate in candidates
        if not candidate['extra_faces']
    ]
    pool = exact_candidates or candidates

    chosen = max(
        pool,
        key=_strip_candidate_sort_key,
    )
    if exact_candidates:
        return chosen, "picked exact candidate with best geometric path/cross ratio"
    return chosen, "no exact candidate; picked best geometric path/cross ratio"


def _group_vert_set(group):
    loops, _is_closed = group
    return {vert for loop in loops for vert in loop}


def _matches_selected_verts(groups, selected_verts):
    shaft_verts = set()
    for group in groups:
        shaft_verts.update(_group_vert_set(group))
    return shaft_verts == selected_verts


def _detect_open_strips_from_selected_verts(bm):
    components = _selected_components(bm)
    if not components:
        return None

    strip_groups = []
    covered_verts = set()
    for component in components:
        strip_group = _detect_open_strip_component(component)
        if strip_group is None:
            continue
        strip_groups.append(strip_group)
        covered_verts.update(component)

    if not strip_groups:
        return None

    selected_verts = {vert for vert in bm.verts if vert.select}
    if covered_verts != selected_verts:
        return None

    return strip_groups


def detect_open_strip_selection(bm):
    selected_verts = {vert for vert in bm.verts if vert.select}
    if not selected_verts:
        return None

    selected_faces = _selected_face_set(bm, selected_verts)
    if selected_faces:
        groups = []
        components = []
        has_extra_selected_faces = False
        has_outside_side_faces = False
        covered_verts = set()

        for face_component in _face_components(selected_faces):
            candidates = _strip_candidates(face_component)
            match, reason = _choose_strip_candidate(candidates)
            if match is None:
                return None

            groups.append(match['group'])
            components.append({
                'group': match['group'],
                'component_faces': set(face_component),
                'shaft_faces': set(match['shaft_faces']),
                'extra_faces': set(match['extra_faces']),
                'outside_side_faces': set(match['outside_side_faces']),
                'face_count': match['face_count'],
                'loop_size': match['loop_size'],
            })
            if match['extra_faces']:
                has_extra_selected_faces = True
            if match['outside_side_faces']:
                has_outside_side_faces = True
            covered_verts.update(vert for face in face_component for vert in face.verts)

        if covered_verts != selected_verts:
            return None

        return {
            'groups': groups,
            'components': components,
            'has_extra_selected_faces': has_extra_selected_faces,
            'has_outside_side_faces': has_outside_side_faces,
        }

    bridged_data = get_bridged_chain(bm)
    if (
        bridged_data
        and not bridged_data[1]
        and len(bridged_data[0]) >= 2
        and _matches_selected_verts([bridged_data], selected_verts)
    ):
        return {
            'groups': [bridged_data],
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    strip_groups = _detect_open_strips_from_selected_verts(bm)
    if strip_groups and _matches_selected_verts(strip_groups, selected_verts):
        return {
            'groups': strip_groups,
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    return None
