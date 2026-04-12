# bridged_open_loop_with_corners.py — Resample bridged open loops with corners.

import bmesh

from .. import anchor_overlay
from .bridge_utils import _is_ordered_open_chain
from .detection_utils import get_selected_islands
from .resample_common import execute_aligned_loops_logic

_VECTOR_TOLERANCE = 1.0e-6
_POSITION_TOLERANCE = 1.0e-6
_FACE_POSITION_TOLERANCE = 1.0e-5
_SOURCE_ENDPOINT_SEAM_ID_LAYER = "rcad_source_endpoint_seam_id"


# Keep the cornered open-bridge detection local to this file so the attached
# outside-geometry path does not depend on sibling operator modules.
def _trace_detect(message, **details):
    return


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

    along_loop_lengths = [_loop_path_length(loop, is_closed) for loop in loops if len(loop) >= 2]
    along_loop_length = sum(along_loop_lengths) / len(along_loop_lengths) if along_loop_lengths else 0.0

    between_loop_distances = []
    if len(loops) >= 2:
        pair_count = min(len(loop) for loop in loops)
        for index in range(pair_count):
            ring_positions = [loop[index].co for loop in loops]
            for pos_a, pos_b in zip(ring_positions, ring_positions[1:]):
                between_loop_distances.append((pos_a - pos_b).length)

    between_loop_length = (
        sum(between_loop_distances) / len(between_loop_distances)
        if between_loop_distances else 0.0
    )
    path_length = between_loop_length
    cross_length = along_loop_length
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

    along_loop_vectors = []
    for loop in loops:
        if len(loop) < 2:
            continue
        pairs = list(zip(loop, loop[1:]))
        if is_closed:
            pairs.append((loop[-1], loop[0]))
        for vert_a, vert_b in pairs:
            along_loop_vectors.append(vert_b.co - vert_a.co)

    between_loop_vectors = []
    if len(loops) >= 2:
        pair_count = min(len(loop) for loop in loops)
        for index in range(pair_count):
            ring_positions = [loop[index].co for loop in loops]
            for pos_a, pos_b in zip(ring_positions, ring_positions[1:]):
                between_loop_vectors.append(pos_b - pos_a)

    along_loop_axis = _aligned_average_axis(along_loop_vectors)
    between_loop_axis = _aligned_average_axis(between_loop_vectors)
    if along_loop_axis is None or between_loop_axis is None:
        return {
            'component_path_span': 0.0,
            'component_cross_span': 0.0,
            'component_path_cross_ratio': 0.0,
        }

    orthogonal_cross = between_loop_axis - (
        along_loop_axis * between_loop_axis.dot(along_loop_axis)
    )
    if orthogonal_cross.length > _VECTOR_TOLERANCE:
        between_loop_axis = orthogonal_cross.normalized()

    along_loop_projections = [vert.co.dot(along_loop_axis) for vert in component_verts]
    between_loop_projections = [vert.co.dot(between_loop_axis) for vert in component_verts]
    along_loop_span = max(along_loop_projections) - min(along_loop_projections)
    between_loop_span = max(between_loop_projections) - min(between_loop_projections)
    path_span = between_loop_span
    cross_span = along_loop_span
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
        -_sortable_metric(candidate.get('component_cross_span', 0.0)),
        _sortable_metric(candidate.get('component_path_cross_ratio', 0.0)),
        _sortable_metric(candidate.get('component_path_span', 0.0)),
        -_sortable_metric(candidate['cross_length']),
        _sortable_metric(candidate['path_cross_ratio']),
        _sortable_metric(candidate['path_length']),
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


def _selected_vert_components(selected_verts):
    if not selected_verts:
        return []

    selected_set = set(selected_verts)
    visited = set()
    components = []

    for vert in sorted(selected_verts, key=lambda item: item.index):
        if vert in visited:
            continue

        stack = [vert]
        component = []
        visited.add(vert)

        while stack:
            current = stack.pop()
            component.append(current)
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other not in selected_set or other in visited:
                    continue
                visited.add(other)
                stack.append(other)

        if component:
            components.append(component)

    return components


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


def _order_cycle_from_edges(edges):
    live_edges = list(edges)
    if not live_edges:
        return None

    adjacency = {}
    for edge in live_edges:
        vert_a, vert_b = edge.verts
        adjacency.setdefault(vert_a, []).append(edge)
        adjacency.setdefault(vert_b, []).append(edge)

    if any(len(linked_edges) != 2 for linked_edges in adjacency.values()):
        return None

    start_edge = min(live_edges, key=lambda item: item.index)
    start_vert = min(start_edge.verts, key=lambda item: item.index)
    current_vert = start_edge.other_vert(start_vert)
    ordered_edges = [start_edge]
    ordered_verts = [start_vert, current_vert]
    visited_edges = {start_edge}

    while len(visited_edges) < len(live_edges):
        next_edges = [
            edge for edge in adjacency.get(current_vert, [])
            if edge not in visited_edges
        ]
        if len(next_edges) != 1:
            return None

        next_edge = next_edges[0]
        visited_edges.add(next_edge)
        ordered_edges.append(next_edge)
        current_vert = next_edge.other_vert(current_vert)
        if len(visited_edges) < len(live_edges):
            ordered_verts.append(current_vert)

    if current_vert is not start_vert or len(ordered_verts) != len(live_edges):
        return None

    return ordered_verts, ordered_edges


def _cycle_chain_between(ordered_verts, start_edge_index, end_edge_index):
    count = len(ordered_verts)
    if count == 0:
        return None

    ordered = []
    index = (start_edge_index + 1) % count
    while True:
        ordered.append(ordered_verts[index])
        if index == end_edge_index:
            break
        index = (index + 1) % count
        if len(ordered) > count:
            return None

    return ordered


def _candidate_pair_metrics(component_set, chain_a, aligned_b):
    if len(chain_a) != len(aligned_b) or len(chain_a) < 2:
        return None

    candidate_pair = ([chain_a, aligned_b], False)
    metrics = {
        **_strip_length_metrics(candidate_pair),
        **_span_metrics_from_verts(component_set, candidate_pair),
        'extra_faces': (),
    }
    return candidate_pair, _strip_candidate_sort_key(metrics)


def _fast_detect_open_strip_component(
    component_set,
    component_edges,
    boundary_edges,
    candidate_cut_edges,
    edge_map,
):
    ordered_cycle = _order_cycle_from_edges(boundary_edges)
    if ordered_cycle is None:
        return None

    ordered_verts, ordered_boundary_edges = ordered_cycle
    boundary_edge_count = len(ordered_boundary_edges)
    if boundary_edge_count < 4 or boundary_edge_count % 2 != 0:
        return None

    half_turn = boundary_edge_count // 2
    cut_edge_to_index = {
        edge: index for index, edge in enumerate(ordered_boundary_edges)
    }
    candidate_cut_set = set(candidate_cut_edges)

    best_pair = None
    best_score = None
    seen_pairs = set()

    for edge in candidate_cut_edges:
        start_index = cut_edge_to_index.get(edge)
        if start_index is None:
            continue
        opposite_index = (start_index + half_turn) % boundary_edge_count
        opposite_edge = ordered_boundary_edges[opposite_index]
        if opposite_edge not in candidate_cut_set:
            continue

        pair_key = tuple(sorted((edge.index, opposite_edge.index)))
        if pair_key in seen_pairs:
            continue
        seen_pairs.add(pair_key)

        chain_a = _cycle_chain_between(ordered_verts, start_index, opposite_index)
        chain_b = _cycle_chain_between(ordered_verts, opposite_index, start_index)
        if chain_a is None or chain_b is None:
            continue

        aligned_b = _align_chain_pair(chain_a, chain_b, edge_map)
        if aligned_b is None:
            continue

        expected_edge_count = (2 * (len(chain_a) - 1)) + len(chain_a)
        if len(component_edges) != expected_edge_count:
            continue
        if boundary_edge_count != 2 * len(chain_a):
            continue

        pair_data = _candidate_pair_metrics(component_set, chain_a, aligned_b)
        if pair_data is None:
            continue

        candidate_pair, score = pair_data
        if best_score is None or score > best_score:
            best_score = score
            best_pair = candidate_pair

    return best_pair


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


def _component_face_edges(face_component):
    edge_face_counts = {}
    for face in face_component:
        for edge in face.edges:
            edge_face_counts[edge] = edge_face_counts.get(edge, 0) + 1
    return edge_face_counts


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

    return _fast_detect_open_strip_component(
        component_set,
        component_edges,
        boundary_edges,
        candidate_cut_edges,
        edge_map,
    )


def _detect_open_strip_from_face_component(face_component):
    component_verts = _faces_to_component_verts(face_component)
    component_set = set(component_verts)
    if len(component_set) < 4:
        return None

    edge_face_counts = _component_face_edges(face_component)
    component_edges = list(edge_face_counts)
    if len(component_edges) < 4:
        return None

    selected_degree = {
        vert: sum(
            1
            for edge in component_edges
            if vert in edge.verts and edge.other_vert(vert) in component_set
        )
        for vert in component_verts
    }
    if any(degree < 2 or degree > 3 for degree in selected_degree.values()):
        return None

    boundary_edges = [
        edge for edge, count in edge_face_counts.items()
        if count == 1
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

    return _fast_detect_open_strip_component(
        component_set,
        component_edges,
        boundary_edges,
        candidate_cut_edges,
        edge_map,
    )


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

            strip_group = _detect_open_strip_from_face_component(shaft_faces)
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


def _selected_verts_match_terminal_loop(group, selected_verts):
    loops, is_closed = group
    if is_closed or not loops:
        return False

    terminal_loops = []
    if loops[0]:
        terminal_loops.append(set(loops[0]))
    if len(loops) > 1 and loops[-1]:
        terminal_loops.append(set(loops[-1]))

    return any(selected_verts <= terminal_loop for terminal_loop in terminal_loops)


def _terminal_subset_range(group, selected_verts):
    loops, is_closed = group
    if is_closed or not loops:
        return None

    terminal_indices = [0]
    if len(loops) > 1:
        terminal_indices.append(len(loops) - 1)

    for loop_index in terminal_indices:
        loop = loops[loop_index]
        loop_positions = {
            vert: index for index, vert in enumerate(loop)
        }
        if not selected_verts <= set(loop_positions):
            continue

        selected_indices = sorted(loop_positions[vert] for vert in selected_verts)
        if not selected_indices:
            continue

        return loop_index, selected_indices[0], selected_indices[-1]

    return None


def _trim_group_to_terminal_subset(group, selected_verts):
    subset_range = _terminal_subset_range(group, selected_verts)
    if subset_range is None:
        return None

    _loop_index, start_index, end_index = subset_range
    loops, is_closed = group
    trimmed_loops = [
        list(loop[start_index:end_index + 1])
        for loop in loops
    ]
    if any(len(loop) < 2 for loop in trimmed_loops):
        return None

    return (trimmed_loops, is_closed)


def _detect_open_strip_from_endpoint_seed(selected_verts):
    components = _selected_vert_components(selected_verts)
    if len(components) > 1:
        groups = []
        for component in components:
            component_data = _detect_open_strip_from_endpoint_seed(set(component))
            if component_data is None:
                return None
            groups.extend(component_data.get('groups', []))

        if not groups:
            return None

        return {
            'groups': groups,
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    start_faces = {
        face
        for vert in selected_verts
        for face in vert.link_faces
        if len(face.verts) == 4
    }
    if not start_faces:
        return None

    face_component = _connected_quad_faces(start_faces)
    if len(face_component) < 2:
        return None

    direct_group = _detect_open_strip_from_face_component(face_component)
    if direct_group is not None:
        trimmed_group = _trim_group_to_terminal_subset(direct_group, selected_verts)
        if trimmed_group is not None:
            return {
                'groups': [trimmed_group],
                'components': [],
                'has_extra_selected_faces': False,
                'has_outside_side_faces': False,
            }

    candidates = _strip_candidates(face_component)
    match, _reason = _choose_strip_candidate(candidates)
    if match is None:
        return None

    if not _selected_verts_match_terminal_loop(match['group'], selected_verts):
        return None

    return {
        'groups': [match['group']],
        'components': [{
            'group': match['group'],
            'component_faces': set(match['shaft_faces']),
            'shaft_faces': set(match['shaft_faces']),
            'extra_faces': set(),
            'outside_side_faces': set(match['outside_side_faces']),
            'face_count': match['face_count'],
            'loop_size': match['loop_size'],
        }],
        'has_extra_selected_faces': False,
        'has_outside_side_faces': bool(match['outside_side_faces']),
    }


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
    _trace_detect(
        "Starting local open strip detection.",
        selected_vert_count=len(selected_verts),
    )
    if not selected_verts:
        return None

    endpoint_seed_data = _detect_open_strip_from_endpoint_seed(selected_verts)
    if endpoint_seed_data is not None:
        return endpoint_seed_data

    selected_faces = _selected_face_set(bm, selected_verts)
    if selected_faces:
        groups = []
        components = []
        has_extra_selected_faces = False
        has_outside_side_faces = False
        covered_verts = set()

        for face_component in _face_components(selected_faces):
            candidates = _strip_candidates(face_component)
            match, _reason = _choose_strip_candidate(candidates)
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

    strip_groups = _detect_open_strips_from_selected_verts(bm)
    if strip_groups and _matches_selected_verts(strip_groups, selected_verts):
        return {
            'groups': strip_groups,
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    return None


def _attached_detect_open_strip_selection(bm):
    selected_verts = {vert for vert in bm.verts if vert.select}
    _trace_detect(
        "Starting attached-case open strip detection.",
        selected_vert_count=len(selected_verts),
    )
    if not selected_verts:
        return None

    endpoint_seed_data = _detect_open_strip_from_endpoint_seed(selected_verts)
    if endpoint_seed_data is not None:
        return endpoint_seed_data

    selected_faces = _selected_face_set(bm, selected_verts)
    if selected_faces:
        groups = []
        components = []
        has_extra_selected_faces = False
        has_outside_side_faces = False
        covered_verts = set()

        for face_component in _face_components(selected_faces):
            candidates = _strip_candidates(face_component)
            match, _reason = _choose_strip_candidate(candidates)
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

    strip_groups = _detect_open_strips_from_selected_verts(bm)
    if strip_groups and _matches_selected_verts(strip_groups, selected_verts):
        return {
            'groups': strip_groups,
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    return None


def _groups_are_open(groups):
    return bool(groups) and all(not rings_data[1] for rings_data in groups)


def _position_key(vec, tolerance):
    return (
        round(vec.x / tolerance),
        round(vec.y / tolerance),
        round(vec.z / tolerance),
    )


def _debug_step(step, **details):
    return


def _trace_corner(message, **details):
    return


def _trace_focus(message, **details):
    return


def _attached_trace_split_debug(message, **details):
    return


def _vert_debug_label(vert):
    if vert is None or not getattr(vert, "is_valid", False):
        return None
    return f"v{vert.index}"


def _edge_debug_label(edge):
    if edge is None or not getattr(edge, "is_valid", False):
        return None
    vert_ids = [_vert_debug_label(vert) for vert in edge.verts]
    return f"e{edge.index}[{','.join(vert_ids)}]"


def _group_debug_key(group):
    loops = group[0] if group else []
    keys = []
    for loop in loops:
        loop_key = _loop_key(loop)
        if loop_key is not None:
            keys.append(loop_key)
    return tuple(keys)


def _attached_shaft_component_summary(faces):
    components = _face_components(_live_bmesh_items(faces))
    return {
        'component_count': len(components),
        'component_face_sizes': sorted((len(component) for component in components), reverse=True),
    }


def _attached_section_cluster_summary(bm, section_positions_list, vert_lookup=None):
    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)

    summaries = []
    for section_index, section_positions in enumerate(section_positions_list, start=1):
        cluster_sizes = []
        for position in section_positions:
            cluster_sizes.append(len(_verts_at_position(bm, position, lookup=vert_lookup)))
        summaries.append({
            'section_index': section_index,
            'point_count': len(section_positions),
            'cluster_sizes': cluster_sizes,
            'split_point_count': sum(1 for size in cluster_sizes if size >= 2),
        })
    return summaries


def _safe_normalized(vec):
    if vec.length <= _POSITION_TOLERANCE:
        return None
    return vec.normalized()


def _selected_seed_loop(bm):
    islands = get_selected_islands(bm)
    _trace_corner(
        "Start detection: checking selected strip.",
        island_count=len(islands),
        selected_vert_count=len(_selected_verts(bm)),
    )
    if len(islands) == 1:
        island = islands[0]
        if not island.get('closed'):
            loop = [
                vert for vert in island.get('verts', [])
                if getattr(vert, "is_valid", False)
            ]
            is_chain = len(loop) >= 2 and _is_ordered_open_chain(loop)
            _trace_corner(
                "Start detection: edge-island candidate.",
                loop_size=len(loop),
                is_closed=bool(island.get('closed')),
                is_ordered_chain=is_chain,
            )
            if is_chain:
                return loop

    selected_components = _selected_vert_components(_selected_verts(bm))
    if len(selected_components) != 1:
        _trace_corner(
            "Start detection rejected.",
            reason="selected verts did not form exactly one connected component",
            component_count=len(selected_components),
            component_sizes=[len(component) for component in selected_components],
        )
        return None

    loop = _order_open_chain_component(selected_components[0])
    if loop is None:
        _trace_corner(
            "Start detection rejected.",
            reason="could not order selected component into an open chain",
            component_size=len(selected_components[0]),
        )
        return None

    is_chain = len(loop) >= 2 and _is_ordered_open_chain(loop)
    _trace_corner(
        "Start detection: ordered selected component.",
        component_size=len(selected_components[0]),
        ordered_size=len(loop),
        is_ordered_chain=is_chain,
        ordered_indices=[vert.index for vert in loop],
    )
    if len(loop) < 2 or not is_chain:
        _trace_corner(
            "Start detection rejected.",
            reason="ordered component was not a valid open chain",
            ordered_size=len(loop),
        )
        return None
    return loop


def _selected_seed_loops(bm):
    seed_loop = _selected_seed_loop(bm)
    if seed_loop is not None:
        return [seed_loop]

    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        return []

    seed_loops = []
    for component_index, component in enumerate(selected_components, start=1):
        loop = _order_open_chain_component(component)
        if loop is None:
            _trace_focus(
                "Selected seed loop failed.",
                component_index=component_index,
                reason="could not order component into an open chain",
                component_size=len(component),
            )
            return None

        if len(loop) < 2 or not _is_ordered_open_chain(loop):
            _trace_focus(
                "Selected seed loop failed.",
                component_index=component_index,
                reason="ordered component was not a valid open chain",
                component_size=len(component),
                loop_size=len(loop),
            )
            return None

        seed_loops.append(loop)

    if len(seed_loops) > 1:
        _trace_focus(
            "Selected seed loops built.",
            component_count=len(seed_loops),
            component_sizes=[len(loop) for loop in seed_loops],
        )
    return seed_loops


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


def _cached_face_match(position, tolerance, lookup, match_cache=None):
    cache_key = None
    if match_cache is not None:
        cache_key = (position.x, position.y, position.z)
        if cache_key in match_cache:
            return match_cache[cache_key]

    matched_face = None
    for candidate_position, face in lookup.get(_position_key(position, tolerance), []):
        if (candidate_position - position).length <= tolerance:
            matched_face = face
            break

    if match_cache is not None:
        match_cache[cache_key] = matched_face
    return matched_face


def _live_faces_from_positions(bm, positions, tolerance=_FACE_POSITION_TOLERANCE, lookup=None, match_cache=None):
    live_faces = set()
    if not positions:
        return live_faces

    if lookup is None:
        lookup = _build_face_position_lookup(bm, tolerance=tolerance)

    for position in positions:
        matched_face = _cached_face_match(
            position,
            tolerance,
            lookup,
            match_cache=match_cache,
        )
        if matched_face is not None:
            live_faces.add(matched_face)

    return live_faces


def _selected_faces(bm):
    return {
        face for face in bm.faces
        if getattr(face, "is_valid", False) and face.select
    }


def _selected_verts(bm):
    return {
        vert for vert in bm.verts
        if getattr(vert, "is_valid", False) and vert.select
    }


def _group_center(group_data):
    loops = _group_loops(group_data)
    centers = [
        center for center in (_loop_center(loop) for loop in loops)
        if center is not None
    ]
    if not centers:
        return None
    total = centers[0].copy()
    for center in centers[1:]:
        total += center
    return total / len(centers)


def _capture_selection_state(bm):
    return {
        'vert_positions': [
            vert.co.copy()
            for vert in bm.verts
            if getattr(vert, "is_valid", False) and vert.select
        ],
        'face_positions': _face_positions(_selected_faces(bm)),
    }


def _source_endpoint_seam_layer(bm):
    layer = bm.verts.layers.int.get(_SOURCE_ENDPOINT_SEAM_ID_LAYER)
    if layer is None:
        layer = bm.verts.layers.int.new(_SOURCE_ENDPOINT_SEAM_ID_LAYER)
    return layer


def _clear_source_endpoint_seam_ids(bm, layer=None):
    if layer is None:
        layer = bm.verts.layers.int.get(_SOURCE_ENDPOINT_SEAM_ID_LAYER)
        if layer is None:
            return

    for vert in bm.verts:
        if getattr(vert, "is_valid", False):
            vert[layer] = 0


def _tag_selected_source_loop_endpoints(bm, source_specs):
    if not source_specs:
        return None

    layer = _source_endpoint_seam_layer(bm)
    _clear_source_endpoint_seam_ids(bm, layer=layer)

    vert_lookup = _build_vert_position_lookup(bm)
    selected_verts = _selected_verts(bm)
    seam_id = 1
    seen_positions = set()

    for source_spec in source_specs:
        source_positions = source_spec.get('source_positions', [])
        if not source_positions:
            continue

        for position in (source_positions[0], source_positions[-1]):
            position_key = _position_key(position, _POSITION_TOLERANCE)
            if position_key in seen_positions:
                continue
            seen_positions.add(position_key)

            live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
            if not live_verts:
                continue

            target_vert = next(
                (vert for vert in live_verts if vert in selected_verts),
                live_verts[0],
            )
            target_vert[layer] = seam_id
            seam_id += 1

    return layer


def _restore_selection_state(bm, selection_state):
    if not selection_state:
        return

    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    face_lookup = _build_face_position_lookup(bm, tolerance=_FACE_POSITION_TOLERANCE)
    match_cache = {}
    for face in _live_faces_from_positions(
        bm,
        selection_state.get('face_positions', []),
        tolerance=_FACE_POSITION_TOLERANCE,
        lookup=face_lookup,
        match_cache=match_cache,
    ):
        face.select = True
        for edge in face.edges:
            if edge.is_valid:
                edge.select = True
        for vert in face.verts:
            if vert.is_valid:
                vert.select = True

    vert_lookup = _build_vert_position_lookup(bm)
    selected_verts = set()
    for position in selection_state.get('vert_positions', []):
        live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
        if not live_verts:
            continue
        vert = live_verts[0]
        vert.select = True
        selected_verts.add(vert)

    for edge in bm.edges:
        if (
            edge.is_valid
            and edge.verts[0] in selected_verts
            and edge.verts[1] in selected_verts
        ):
            edge.select = True

    bm.select_flush_mode()


def _source_loop_specs(groups, partial_ranges=None):
    specs = []
    for group_index, group_data in enumerate(groups or [], start=1):
        loops = _group_loops(group_data)
        if not loops:
            continue
        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]
        loop_index = 0
        if partial_range is not None:
            loop_index = partial_range.get('loop_index', 0)
        if loop_index < 0 or loop_index >= len(loops):
            loop_index = 0
        loop = loops[loop_index]
        loop_center = _loop_center(loops[loop_index])
        group_center = _group_center(group_data)
        if loop_center is None:
            continue
        loop_size = max(1, len(loop) - 1)
        start_index = 0
        end_index = len(loop) - 1
        if partial_range is not None:
            start_index = max(0, min(len(loop) - 1, partial_range.get('start_index', 0)))
            end_index = max(start_index, min(len(loop) - 1, partial_range.get('end_index', len(loop) - 1)))
        source_slice = loop[start_index:end_index + 1]
        specs.append({
            'group_index': group_index - 1,
            'loop_index': loop_index,
            'group_center': group_center.copy() if group_center is not None else None,
            'loop_center': loop_center.copy(),
            'start_ratio': start_index / loop_size if len(loop) > 1 else 0.0,
            'end_ratio': end_index / loop_size if len(loop) > 1 else 1.0,
            'source_positions': [
                vert.co.copy()
                for vert in source_slice
                if getattr(vert, "is_valid", False)
            ],
        })
    return specs


def _loop_slice_from_source_spec(loop, source_spec):
    if not loop:
        return []
    if len(loop) == 1:
        return list(loop)

    max_index = len(loop) - 1
    start_index = int(round(source_spec.get('start_ratio', 0.0) * max_index))
    end_index = int(round(source_spec.get('end_ratio', 1.0) * max_index))
    start_index = max(0, min(max_index, start_index))
    end_index = max(start_index, min(max_index, end_index))
    return list(loop[start_index:end_index + 1])


def _select_source_loops(bm, groups, source_specs):
    if not groups or not source_specs:
        return False

    group_candidates = []
    for group_index, group_data in enumerate(groups):
        loops = _group_loops(group_data)
        if not loops:
            continue
        group_center = _group_center(group_data)
        loop_entries = []
        for loop in loops:
            center = _loop_center(loop)
            if center is None:
                continue
            loop_entries.append({
                'loop': loop,
                'center': center,
            })
        if not loop_entries:
            continue
        group_candidates.append({
            'group_index': group_index,
            'group_center': group_center,
            'loops': loop_entries,
        })

    if not group_candidates:
        return False

    matched_loops = []
    remaining_groups = list(group_candidates)
    for source_spec in source_specs:
        direct_group = next(
            (
                candidate_group for candidate_group in remaining_groups
                if candidate_group['group_index'] == source_spec.get('group_index')
            ),
            None,
        )
        if direct_group is not None:
            remaining_groups.remove(direct_group)
            loop_entries = direct_group['loops']
            loop_index = source_spec.get('loop_index', 0)
            loop_index = max(0, min(len(loop_entries) - 1, loop_index))
            matched_loops.append(
                _loop_slice_from_source_spec(loop_entries[loop_index]['loop'], source_spec)
            )
            continue

        best_group_index = None
        best_group_score = None
        for index, candidate_group in enumerate(remaining_groups):
            group_center = candidate_group['group_center']
            if group_center is None or source_spec['group_center'] is None:
                group_distance = 0.0
            else:
                group_distance = (group_center - source_spec['group_center']).length
            loop_distance = min(
                (entry['center'] - source_spec['loop_center']).length
                for entry in candidate_group['loops']
            )
            score = (group_distance, loop_distance)
            if best_group_score is None or score < best_group_score:
                best_group_score = score
                best_group_index = index
        if best_group_index is None:
            continue
        candidate_group = remaining_groups.pop(best_group_index)
        loop_entries = candidate_group['loops']
        loop_index = source_spec.get('loop_index', 0)
        if 0 <= loop_index < len(loop_entries):
            matched_loops.append(
                _loop_slice_from_source_spec(loop_entries[loop_index]['loop'], source_spec)
            )
            continue
        best_loop = min(
            loop_entries,
            key=lambda entry: (entry['center'] - source_spec['loop_center']).length,
        )
        matched_loops.append(_loop_slice_from_source_spec(best_loop['loop'], source_spec))

    if not matched_loops:
        return False

    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    for loop in matched_loops:
        for vert in loop:
            if getattr(vert, "is_valid", False):
                vert.select = True
        for edge in _chain_edges(loop, is_closed=False):
            if getattr(edge, "is_valid", False):
                edge.select = True

    bm.select_flush_mode()
    return True


def _source_targets_for_fresh_groups(source_specs, groups):
    targets = []
    used = set()
    for source_index, source_spec in enumerate(source_specs):
        best = None
        for group_index, rings_data in enumerate(groups):
            loops = rings_data[0]
            if not loops:
                continue
            for loop_index, loop in enumerate(loops):
                candidate_key = (group_index, loop_index)
                if candidate_key in used:
                    continue

                sliced_loop = _loop_slice_from_source_spec(loop, source_spec)
                candidate_positions = [
                    vert.co.copy()
                    for vert in sliced_loop
                    if getattr(vert, "is_valid", False)
                ]
                if not candidate_positions:
                    continue

                source_positions = source_spec.get('source_positions', [])
                if not source_positions:
                    center = _loop_center(sliced_loop)
                    if center is None:
                        continue
                    distance_score = (center - source_spec['loop_center']).length
                    length_delta = 0
                else:
                    total_distance = 0.0
                    for source_position in source_positions:
                        total_distance += min(
                            (candidate_position - source_position).length
                            for candidate_position in candidate_positions
                        )
                    distance_score = total_distance / len(source_positions)
                    length_delta = abs(len(candidate_positions) - len(source_positions))

                score = (
                    distance_score,
                    length_delta,
                    abs(group_index - source_spec.get('group_index', 0)),
                    abs(loop_index - source_spec.get('loop_index', 0)),
                )
                if best is None or score < best['score']:
                    best = {
                        'source_index': source_index,
                        'group_index': group_index,
                        'loop_index': loop_index,
                        'score': score,
                    }
        if best is not None:
            used.add((best['group_index'], best['loop_index']))
            targets.append(best)
    return targets


def _attached_source_loop_specs(groups, partial_ranges=None):
    specs = []
    for group_index, group_data in enumerate(groups or [], start=1):
        loops = _group_loops(group_data)
        if not loops:
            continue
        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]
        loop_index = 0
        if partial_range is not None:
            loop_index = partial_range.get('loop_index', 0)
        if loop_index < 0 or loop_index >= len(loops):
            loop_index = 0
        loop = loops[loop_index]
        loop_center = _loop_center(loops[loop_index])
        group_center = _group_center(group_data)
        if loop_center is None:
            continue
        loop_size = max(1, len(loop) - 1)
        start_index = 0
        end_index = len(loop) - 1
        if partial_range is not None:
            start_index = max(0, min(len(loop) - 1, partial_range.get('start_index', 0)))
            end_index = max(start_index, min(len(loop) - 1, partial_range.get('end_index', len(loop) - 1)))
        source_slice = loop[start_index:end_index + 1]
        specs.append({
            'group_index': group_index - 1,
            'loop_index': loop_index,
            'group_center': group_center.copy() if group_center is not None else None,
            'loop_center': loop_center.copy(),
            'start_ratio': start_index / loop_size if len(loop) > 1 else 0.0,
            'end_ratio': end_index / loop_size if len(loop) > 1 else 1.0,
            'source_positions': [
                vert.co.copy()
                for vert in source_slice
                if getattr(vert, "is_valid", False)
            ],
        })
    return specs


def _attached_select_source_loops(bm, groups, source_specs):
    if not groups or not source_specs:
        return False

    group_candidates = []
    for group_index, group_data in enumerate(groups):
        loops = _group_loops(group_data)
        if not loops:
            continue
        group_center = _group_center(group_data)
        loop_entries = []
        for loop in loops:
            center = _loop_center(loop)
            if center is None:
                continue
            loop_entries.append({
                'loop': loop,
                'center': center,
            })
        if not loop_entries:
            continue
        group_candidates.append({
            'group_index': group_index,
            'group_center': group_center,
            'loops': loop_entries,
        })

    if not group_candidates:
        return False

    matched_loops = []
    remaining_groups = list(group_candidates)
    for source_spec in source_specs:
        direct_group = next(
            (
                candidate_group for candidate_group in remaining_groups
                if candidate_group['group_index'] == source_spec.get('group_index')
            ),
            None,
        )
        if direct_group is not None:
            remaining_groups.remove(direct_group)
            loop_entries = direct_group['loops']
            loop_index = source_spec.get('loop_index', 0)
            loop_index = max(0, min(len(loop_entries) - 1, loop_index))
            matched_loops.append(
                _loop_slice_from_source_spec(loop_entries[loop_index]['loop'], source_spec)
            )
            continue

        best_group_index = None
        best_group_score = None
        for index, candidate_group in enumerate(remaining_groups):
            group_center = candidate_group['group_center']
            if group_center is None or source_spec['group_center'] is None:
                group_distance = 0.0
            else:
                group_distance = (group_center - source_spec['group_center']).length
            loop_distance = min(
                (entry['center'] - source_spec['loop_center']).length
                for entry in candidate_group['loops']
            )
            score = (group_distance, loop_distance)
            if best_group_score is None or score < best_group_score:
                best_group_score = score
                best_group_index = index
        if best_group_index is None:
            continue
        candidate_group = remaining_groups.pop(best_group_index)
        loop_entries = candidate_group['loops']
        loop_index = source_spec.get('loop_index', 0)
        if 0 <= loop_index < len(loop_entries):
            matched_loops.append(
                _loop_slice_from_source_spec(loop_entries[loop_index]['loop'], source_spec)
            )
            continue
        best_loop = min(
            loop_entries,
            key=lambda entry: (entry['center'] - source_spec['loop_center']).length,
        )
        matched_loops.append(_loop_slice_from_source_spec(best_loop['loop'], source_spec))

    if not matched_loops:
        return False

    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    for loop in matched_loops:
        for vert in loop:
            if getattr(vert, "is_valid", False):
                vert.select = True
        for edge in _chain_edges(loop, is_closed=False):
            if getattr(edge, "is_valid", False):
                edge.select = True

    bm.select_flush_mode()
    return True


def _attached_source_targets_for_fresh_groups(source_specs, groups):
    targets = []
    used = set()
    for source_index, source_spec in enumerate(source_specs):
        best = None
        for group_index, rings_data in enumerate(groups):
            loops = rings_data[0]
            if not loops:
                continue
            for loop_index, loop in enumerate(loops):
                candidate_key = (group_index, loop_index)
                if candidate_key in used:
                    continue

                sliced_loop = _loop_slice_from_source_spec(loop, source_spec)
                candidate_positions = [
                    vert.co.copy()
                    for vert in sliced_loop
                    if getattr(vert, "is_valid", False)
                ]
                if not candidate_positions:
                    continue

                source_positions = source_spec.get('source_positions', [])
                if not source_positions:
                    center = _loop_center(sliced_loop)
                    if center is None:
                        continue
                    distance_score = (center - source_spec['loop_center']).length
                    length_delta = 0
                else:
                    total_distance = 0.0
                    for source_position in source_positions:
                        total_distance += min(
                            (candidate_position - source_position).length
                            for candidate_position in candidate_positions
                        )
                    distance_score = total_distance / len(source_positions)
                    length_delta = abs(len(candidate_positions) - len(source_positions))

                score = (
                    distance_score,
                    length_delta,
                    abs(group_index - source_spec.get('group_index', 0)),
                    abs(loop_index - source_spec.get('loop_index', 0)),
                )
                if best is None or score < best['score']:
                    best = {
                        'source_index': source_index,
                        'group_index': group_index,
                        'loop_index': loop_index,
                        'score': score,
                    }
        if best is not None:
            used.add((best['group_index'], best['loop_index']))
            targets.append(best)
    return targets


def _transpose_open_group(rings_data):
    loops, is_closed = rings_data
    if is_closed or len(loops) < 2:
        return None

    loop_size = len(loops[0]) if loops else 0
    if loop_size < 2:
        return None
    if any(len(loop) != loop_size for loop in loops):
        return None

    transposed_loops = []
    for vert_index in range(loop_size):
        transposed_loop = [loop[vert_index] for loop in loops]
        if len(transposed_loop) < 2:
            return None
        transposed_loops.append(transposed_loop)

    return transposed_loops, False


def _attached_best_source_target_for_group(source_specs, rings_data):
    matches = _attached_source_targets_for_fresh_groups(source_specs, [rings_data])
    if not matches:
        return None
    return min(matches, key=lambda item: item['score'])


def _attached_orient_fresh_group_to_source(source_specs, rings_data):
    transposed = _transpose_open_group(rings_data)
    if transposed is None or not source_specs:
        return rings_data

    original_match = _attached_best_source_target_for_group(source_specs, rings_data)
    transposed_match = _attached_best_source_target_for_group(source_specs, transposed)
    if transposed_match is None:
        return rings_data
    if original_match is None or transposed_match['score'] < original_match['score']:
        return transposed
    return rings_data


def _attached_segment_sections(groups, partial_ranges=None):
    sections = []
    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if is_closed or len(loops) < 3:
            continue

        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]

        for loop in loops[1:-1]:
            sections.append(_slice_loop_for_partial_range(loop, partial_range))

    return sections


def _attached_build_open_corner_execute_plan(data):
    end_sections, corner_sections, all_sections, collected_section_indices = _attached_detected_cross_sections(
        data.get('groups', []),
        partial_ranges=data.get('partial_ranges'),
    )
    segment_sections = _attached_segment_sections(
        data.get('groups', []),
        partial_ranges=data.get('partial_ranges'),
    )

    shaft_face_positions = _face_positions(
        {
            face
            for group in data.get('groups', [])
            for face in _shaft_faces_from_loop_sequence(group[0], is_closed=group[1], strict=False)
        }
    )

    segment_split_edges = set()
    split_edges = set()
    split_section_logs = []
    split_mode = "segment_sections"
    seam_records = []

    for section_index, section in enumerate(segment_sections, start=1):
        chain_edges = _chain_edges(section, is_closed=False)
        if not chain_edges:
            split_section_logs.append({
                'section_index': section_index,
                'vert_indices': [vert.index for vert in section if getattr(vert, "is_valid", False)],
                'edge_indices': None,
            })
            continue
        split_section_logs.append({
            'section_index': section_index,
            'vert_indices': [vert.index for vert in section if getattr(vert, "is_valid", False)],
            'edge_indices': [edge.index for edge in chain_edges if getattr(edge, "is_valid", False)],
            'edge_labels': [_edge_debug_label(edge) for edge in chain_edges if getattr(edge, "is_valid", False)],
        })
        segment_split_edges.update(chain_edges)
        split_edges.update(chain_edges)

    seam_records = _attached_outer_boundary_seam_records_from_groups(data.get('groups', []))
    if seam_records:
        split_mode = (
            "detach_and_segment_sections"
            if split_edges else "outer_chunk_boundaries"
        )
        split_edges.update(
            edge
            for record in seam_records
            for edge in record.get('boundary_edges', set())
            if getattr(edge, "is_valid", False)
        )
        split_section_logs.extend([
            {
                'section_index': index,
                'boundary_edge_count': len(record.get('boundary_edges', set())),
                'shaft_face_count': len(record.get('shaft_faces', set())),
                'outside_face_count': len(record.get('corner_faces', set())),
                'edge_indices': sorted(
                    edge.index for edge in record.get('boundary_edges', set())
                    if getattr(edge, "is_valid", False)
                ),
            }
            for index, record in enumerate(seam_records, start=1)
        ])

    return {
        'end_sections': end_sections,
        'corner_sections': corner_sections,
        'segment_sections': segment_sections,
        'segment_section_positions': [
            [
                vert.co.copy()
                for vert in section
                if getattr(vert, "is_valid", False)
            ]
            for section in segment_sections
        ],
        'segment_split_edges': segment_split_edges,
        'all_sections': all_sections,
        'collected_section_indices': collected_section_indices,
        'shaft_face_positions': shaft_face_positions,
        'split_edges': split_edges,
        'split_section_logs': split_section_logs,
        'split_mode': split_mode,
        'seam_records': seam_records,
    }


def _select_loops_from_positions(bm, loop_positions_list):
    if not loop_positions_list:
        return False

    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    vert_lookup = _build_vert_position_lookup(bm)
    any_selected = False
    for loop_positions in loop_positions_list:
        selected_verts = []
        for position in loop_positions:
            live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
            if not live_verts:
                continue
            vert = live_verts[0]
            vert.select = True
            selected_verts.append(vert)
            any_selected = True

        for vert_a, vert_b in zip(selected_verts, selected_verts[1:]):
            edge = next(
                (
                    item for item in vert_a.link_edges
                    if getattr(item, "is_valid", False) and item.other_vert(vert_a) is vert_b
                ),
                None,
            )
            if edge is not None:
                edge.select = True

    if any_selected:
        bm.select_flush_mode()
    return any_selected


def _open_groups_from_shaft_faces(shaft_faces):
    live_shaft_faces = _live_bmesh_items(shaft_faces)
    if not live_shaft_faces:
        return []

    open_groups = []
    for face_component in _face_components(live_shaft_faces):
        rings_data = _detect_open_strip_component(
            _faces_to_component_verts(face_component)
        )
        if rings_data is None or rings_data[1]:
            return []
        open_groups.append(rings_data)

    return open_groups


def _attached_open_groups_from_shaft_faces(shaft_faces):
    live_shaft_faces = _live_bmesh_items(shaft_faces)
    if not live_shaft_faces:
        return []

    open_groups = []
    for face_component in _face_components(live_shaft_faces):
        rings_data = _detect_open_strip_component(
            _faces_to_component_verts(face_component)
        )
        if rings_data is None or rings_data[1]:
            return []
        open_groups.append(rings_data)

    return open_groups


def _live_bmesh_items(items):
    return {
        item for item in items
        if item is not None and getattr(item, "is_valid", False)
    }


def _sort_key_desc(sort_key):
    return tuple(
        -value if isinstance(value, (int, float)) else value
        for value in sort_key
    )


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


def _quad_face_from_vert_set(verts):
    ordered_verts = [
        vert for vert in verts
        if vert is not None and getattr(vert, "is_valid", False)
    ]
    vert_set = set(ordered_verts)
    if len(vert_set) != 4:
        return None

    shared_faces = set(ordered_verts[0].link_faces)
    for vert in ordered_verts[1:]:
        shared_faces.intersection_update(vert.link_faces)

    return next(
        (
            face for face in shared_faces
            if getattr(face, "is_valid", False)
            and len(face.verts) == 4
            and set(face.verts) == vert_set
        ),
        None,
    )


def _shaft_faces_from_loop_sequence(loops, is_closed=False, strict=False):
    if not loops or len(loops) < 2:
        return set()

    shaft_faces = set()
    loop_pairs = list(zip(loops, loops[1:]))
    if is_closed and len(loops) > 2:
        loop_pairs.append((loops[-1], loops[0]))

    for current_loop, next_loop in loop_pairs:
        if len(current_loop) != len(next_loop) or len(current_loop) < 2:
            if strict:
                return set()
            continue

        for curr_a, curr_b, next_a, next_b in zip(
            current_loop,
            current_loop[1:],
            next_loop,
            next_loop[1:],
        ):
            face = _quad_face_from_vert_set((curr_a, curr_b, next_b, next_a))
            if face is None:
                if strict:
                    return set()
                continue
            shaft_faces.add(face)

    return shaft_faces


def _shaft_faces_from_open_group(group):
    loops, is_closed = group
    if is_closed or len(loops) < 2:
        return set()
    return _shaft_faces_from_loop_sequence(loops, is_closed=False, strict=True)


def _corner_face_component_from_shaft_faces(shaft_seed_faces):
    shaft_faces = _connected_quad_faces(shaft_seed_faces)
    if not shaft_faces:
        return set(), set()

    face_component = set(shaft_faces)
    stack = list(shaft_faces)
    visited = set(shaft_faces)

    while stack:
        face = stack.pop()
        for edge in face.edges:
            for neighbor in edge.link_faces:
                if neighbor in visited or not getattr(neighbor, "is_valid", False):
                    continue
                visited.add(neighbor)
                face_component.add(neighbor)
                if len(neighbor.verts) == 4:
                    stack.append(neighbor)

    return face_component, shaft_faces


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

        candidate = {
            'strip_faces': set(strip_faces),
            'rings': rings_data,
            'extra_faces': extra_faces,
            'face_count': len(strip_faces),
            'boundary_components': boundary_components,
            **_strip_length_metrics(rings_data),
            **_component_span_metrics(face_component, rings_data),
        }
        candidate['sort_key'] = _strip_candidate_sort_key(candidate)
        candidates.append(candidate)

    return sorted(
        candidates,
        key=lambda item: item['sort_key'],
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


def _canonical_chain_key(chain):
    ids = tuple(
        vert.index for vert in chain
        if vert is not None and getattr(vert, "is_valid", False)
    )
    if not ids:
        return None
    reversed_ids = tuple(reversed(ids))
    return min(ids, reversed_ids)


def _route_face_key(faces):
    return tuple(
        sorted(
            face.index for face in faces
            if face is not None and getattr(face, "is_valid", False)
        )
    )


def _all_seam_records_from_face_components(face_components):
    seam_records = {}
    all_shaft_faces = set()
    for face_component in face_components:
        candidates = _open_face_strip_candidates(face_component)
        if not candidates:
            _debug_step("candidate filter", component_face_count=len(face_component), kept=0)
            continue

        chosen_candidates = []
        occupied_faces = set()
        for candidate in candidates:
            strip_faces = _live_bmesh_items(candidate.get('strip_faces', set()))
            if not strip_faces or strip_faces & occupied_faces:
                continue
            normalized_candidate = {
                **candidate,
                'strip_faces': strip_faces,
                'extra_faces': _live_bmesh_items(candidate.get('extra_faces', set())),
            }
            normalized_candidate['sort_key'] = candidate.get('sort_key', _strip_candidate_sort_key(normalized_candidate))
            chosen_candidates.append(normalized_candidate)
            occupied_faces.update(strip_faces)

        _debug_step(
            "candidate filter",
            component_face_count=len(face_component),
            kept=len(chosen_candidates),
        )
        for candidate in chosen_candidates:
            all_shaft_faces.update(candidate.get('strip_faces', set()))

        seam_entries = {}
        for candidate_index, candidate in enumerate(chosen_candidates, start=1):
            shaft_faces = candidate.get('strip_faces', set())
            corner_faces = candidate.get('extra_faces', set())
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
                    'sort_key': candidate.get('sort_key', ()),
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
            _sort_key_desc(item.get('sort_key', ())),
            item['key'],
        )
    )
    return filtered_records, _face_positions(all_shaft_faces)


def _all_seam_records(bm):
    selected_faces = _selected_face_set(bm, _selected_verts(bm))
    if not selected_faces:
        return [], []
    return _all_seam_records_from_face_components(_face_components(selected_faces))


def _outer_boundary_seam_records_from_groups(groups):
    seam_records = []
    for group in groups or []:
        loops, is_closed = group
        shaft_faces = _shaft_faces_from_loop_sequence(
            loops,
            is_closed=is_closed,
            strict=False,
        )
        if not shaft_faces:
            continue

        boundary_edges = set()
        for shaft_face in shaft_faces:
            for edge in shaft_face.edges:
                linked_faces = {
                    face for face in edge.link_faces
                    if getattr(face, "is_valid", False)
                }
                if not linked_faces:
                    continue
                if any(face not in shaft_faces for face in linked_faces):
                    boundary_edges.add(edge)

        for boundary_component in _boundary_path_components(boundary_edges):
            component_edges = {
                edge for edge in boundary_component
                if edge is not None and getattr(edge, "is_valid", False)
            }
            if not component_edges:
                continue

            component_outside_faces = set()
            for edge in component_edges:
                component_outside_faces.update(
                    face for face in edge.link_faces
                    if getattr(face, "is_valid", False) and face not in shaft_faces
                )

            if not component_outside_faces:
                continue

            seam_records.append({
                'boundary_edges': component_edges,
                'shaft_faces': set(shaft_faces),
                'corner_faces': component_outside_faces,
            })

    return seam_records


def _attached_outer_boundary_seam_records_from_groups(groups):
    seam_records = []
    for group in groups or []:
        loops, is_closed = group
        shaft_faces = _shaft_faces_from_loop_sequence(
            loops,
            is_closed=is_closed,
            strict=False,
        )
        if not shaft_faces:
            continue

        boundary_edges = set()
        for shaft_face in shaft_faces:
            for edge in shaft_face.edges:
                linked_faces = {
                    face for face in edge.link_faces
                    if getattr(face, "is_valid", False)
                }
                if not linked_faces:
                    continue
                if any(face not in shaft_faces for face in linked_faces):
                    boundary_edges.add(edge)

        for boundary_component in _boundary_path_components(boundary_edges):
            component_edges = {
                edge for edge in boundary_component
                if edge is not None and getattr(edge, "is_valid", False)
            }
            if not component_edges:
                continue

            component_outside_faces = set()
            for edge in component_edges:
                component_outside_faces.update(
                    face for face in edge.link_faces
                    if getattr(face, "is_valid", False) and face not in shaft_faces
                )

            if not component_outside_faces:
                continue

            seam_records.append({
                'boundary_edges': component_edges,
                'shaft_faces': set(shaft_faces),
                'corner_faces': component_outside_faces,
            })

    return seam_records


def _detect_corner_endpoint_seed(bm):
    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        _trace_corner("Endpoint-seed corner detect rejected: no selected components.")
        return None

    route_records = {}
    for component_index, component in enumerate(selected_components, start=1):
        component_set = {
            vert for vert in component
            if getattr(vert, "is_valid", False)
        }
        if len(component_set) < 2:
            continue

        start_faces = {
            face
            for vert in component_set
            for face in vert.link_faces
            if getattr(face, "is_valid", False) and len(face.verts) == 4
        }
        if not start_faces:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="no adjacent quad faces",
            )
            return None

        face_component = _connected_quad_faces(start_faces)
        if len(face_component) < 2:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="connected quad component too small",
                face_count=len(face_component),
            )
            return None

        matched_candidate = None
        for candidate in _open_face_strip_candidates(face_component):
            if not candidate.get('extra_faces'):
                continue
            trimmed_group = _trim_group_to_terminal_subset(candidate['rings'], component_set)
            if trimmed_group is None:
                continue
            matched_candidate = candidate
            break

        if matched_candidate is None:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="no corner strip candidate matched terminal selection",
                component_face_count=len(face_component),
            )
            return None

        route_key = _route_face_key(matched_candidate.get('strip_faces', set()))
        if not route_key:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="matched candidate had no shaft faces",
            )
            return None

        route_record = route_records.setdefault(
            route_key,
            {
                'group': matched_candidate['rings'],
                'component_faces': set(),
                'shaft_faces': set(),
                'extra_faces': set(),
            },
        )
        route_record['component_faces'].update(face_component)
        route_record['shaft_faces'].update(
            _live_bmesh_items(matched_candidate.get('strip_faces', set()))
        )
        route_record['extra_faces'].update(
            _live_bmesh_items(matched_candidate.get('extra_faces', set()))
        )
        _trace_corner(
            "Endpoint-seed corner detect matched component.",
            component_index=component_index,
            component_face_count=len(face_component),
            shaft_face_count=len(route_record['shaft_faces']),
            corner_face_count=len(route_record['extra_faces']),
        )

    seed_components = [
        {
            'face_component': set(record['component_faces']),
            'shaft_faces': set(record['shaft_faces']),
            'extra_faces': set(record['extra_faces']),
        }
        for record in route_records.values()
        if record['extra_faces']
    ]
    if not seed_components:
        _trace_corner("Endpoint-seed corner detect rejected: no corner components found.")
        return None

    _trace_corner(
        "Endpoint-seed corner detect matched.",
        group_count=len(route_records),
        corner_component_count=len(seed_components),
    )
    return {
        'groups': [record['group'] for record in route_records.values()],
        'components': [],
        'seed_components': seed_components,
        'mode_label': 'Bridged open loop with corners',
    }


def _selected_verts_match_any_loop(rings_data, selected_verts):
    loops, is_closed = rings_data
    if is_closed or not loops:
        return None

    selected_set = {
        vert for vert in selected_verts
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_score = None
    for loop_index, loop in enumerate(loops, start=1):
        loop_set = {
            vert for vert in loop
            if getattr(vert, "is_valid", False)
        }
        match_count = len(selected_set & loop_set)
        if match_count == 0:
            continue

        selected_coverage = match_count / max(1, len(selected_set))
        loop_coverage = match_count / max(1, len(loop_set))
        score = (
            match_count,
            round(selected_coverage, 6),
            round(loop_coverage, 6),
            -loop_index,
        )
        if best_score is None or score > best_score:
            best_score = score

    return best_score


def _selected_subset_range_any_loop(rings_data, selected_component):
    loops, is_closed = rings_data
    if is_closed or not loops:
        return None

    selected_chain = _order_open_chain_component(selected_component)
    if selected_chain is None:
        selected_chain = [
            vert for vert in selected_component
            if getattr(vert, "is_valid", False)
        ]

    selected_set = {
        vert for vert in selected_chain
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_range = None
    best_score = None
    for loop_index, loop in enumerate(loops):
        loop_positions = {
            vert: index for index, vert in enumerate(loop)
            if getattr(vert, "is_valid", False)
        }
        matched_indices = [
            loop_positions[vert]
            for vert in selected_chain
            if vert in loop_positions
        ]
        average_distance = 0.0

        if len(matched_indices) < 2:
            matched_indices = []
            total_distance = 0.0
            valid_matches = 0
            loop_verts = [
                vert for vert in loop
                if getattr(vert, "is_valid", False)
            ]
            if len(loop_verts) < 2:
                continue
            for selected_vert in selected_chain:
                if not getattr(selected_vert, "is_valid", False):
                    continue
                nearest_index = min(
                    range(len(loop_verts)),
                    key=lambda index: (loop_verts[index].co - selected_vert.co).length,
                )
                total_distance += (loop_verts[nearest_index].co - selected_vert.co).length
                matched_indices.append(nearest_index)
                valid_matches += 1
            if valid_matches < 2:
                continue
            average_distance = total_distance / valid_matches

        matched_indices = sorted(set(matched_indices))
        if len(matched_indices) < 2:
            continue

        start_index = matched_indices[0]
        end_index = matched_indices[-1]
        span_count = end_index - start_index + 1
        if span_count < 2:
            continue

        match_count = len(matched_indices)
        selected_coverage = match_count / max(1, len(selected_set))
        span_fill = match_count / max(1, span_count)
        score = (
            match_count,
            round(selected_coverage, 6),
            round(span_fill, 6),
            -round(average_distance, 6),
            -loop_index,
        )
        if best_score is None or score > best_score:
            best_score = score
            best_range = {
                'loop_index': loop_index,
                'start_index': start_index,
                'end_index': end_index,
                'score': score,
            }

    return best_range


def _slice_loop_for_partial_range(loop, partial_range):
    if partial_range is None:
        return list(loop)

    start_index = partial_range.get('start_index', 0)
    end_index = partial_range.get('end_index', len(loop) - 1)
    if start_index < 0 or end_index >= len(loop) or start_index >= end_index:
        return list(loop)
    return list(loop[start_index:end_index + 1])


def _detect_corner_any_cross_section_seed(bm):
    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        _trace_focus("Cross-section input missing.")
        return None

    _trace_focus(
        "Cross-section input.",
        component_count=len(selected_components),
        component_sizes=[len(component) for component in selected_components],
    )

    matched_groups = []
    partial_ranges = []
    group_keys = []
    for component_index, component in enumerate(selected_components, start=1):
        component_set = {
            vert for vert in component
            if getattr(vert, "is_valid", False)
        }
        if len(component_set) < 2:
            _trace_focus(
                "Selected cross section skipped.",
                component_index=component_index,
                reason="component too small",
                component_size=len(component_set),
            )
            continue

        start_faces = {
            face
            for vert in component_set
            for face in vert.link_faces
            if getattr(face, "is_valid", False) and len(face.verts) == 4
        }
        if not start_faces:
            _trace_focus(
                "Selected cross section match failed.",
                component_index=component_index,
                reason="no adjacent quad faces",
                component_size=len(component_set),
            )
            return None

        face_component = _connected_quad_faces(start_faces)
        if len(face_component) < 2:
            _trace_focus(
                "Selected cross section match failed.",
                component_index=component_index,
                reason="connected quad component too small",
                component_size=len(component_set),
                start_face_count=len(start_faces),
                face_count=len(face_component),
            )
            return None

        candidates = _open_face_strip_candidates(face_component)
        _trace_focus(
            "Selected cross section candidate scan.",
            component_index=component_index,
            component_size=len(component_set),
            start_face_count=len(start_faces),
            component_face_count=len(face_component),
            candidate_count=len(candidates),
            candidate_loop_counts=[len(candidate['rings'][0]) for candidate in candidates],
            candidate_loop_sizes=[
                len(candidate['rings'][0][0]) if candidate['rings'][0] else 0
                for candidate in candidates
            ],
        )

        matched_candidate = None
        matched_score = None
        fallback_candidate = None
        for candidate_index, candidate in enumerate(candidates, start=1):
            if fallback_candidate is None:
                fallback_candidate = candidate
            candidate_score = _selected_verts_match_any_loop(candidate['rings'], component_set)
            _trace_focus(
                "Selected cross section candidate result.",
                component_index=component_index,
                candidate_index=candidate_index,
                loop_count=len(candidate['rings'][0]),
                loop_size=len(candidate['rings'][0][0]) if candidate['rings'][0] else 0,
                match_score=candidate_score,
            )
            if candidate_score is None:
                continue
            if matched_score is None or candidate_score > matched_score:
                matched_candidate = candidate
                matched_score = candidate_score

        if matched_candidate is None:
            matched_candidate = fallback_candidate
            if matched_candidate is None:
                _trace_focus(
                    "Selected cross section match failed.",
                    component_index=component_index,
                    reason="no bridge candidates in connected quad component",
                    component_size=len(component_set),
                    start_face_count=len(start_faces),
                    component_face_count=len(face_component),
                    candidate_count=0,
                )
                return None
            _trace_focus(
                "Selected cross section route fallback.",
                component_index=component_index,
                reason="no candidate loop overlapped the selected verts",
                component_size=len(component_set),
                component_face_count=len(face_component),
                candidate_count=len(candidates),
            )

        matched_groups.append(matched_candidate['rings'])
        partial_range = _selected_subset_range_any_loop(
            matched_candidate['rings'],
            component,
        )
        partial_ranges.append(partial_range)
        group_key = _group_debug_key(matched_candidate['rings'])
        group_keys.append(group_key)
        _trace_focus(
            "Selected cross section matched route.",
            component_index=component_index,
            component_size=len(component_set),
            component_face_count=len(face_component),
            loop_count=len(matched_candidate['rings'][0]),
            loop_size=len(matched_candidate['rings'][0][0]) if matched_candidate['rings'][0] else 0,
            match_score=matched_score,
            partial_range=partial_range,
            group_key=group_key,
        )

    if not matched_groups:
        _trace_focus("Cross-section input failed to match any route.")
        return None

    duplicate_route_count = len(group_keys) - len(set(group_keys))
    _trace_focus(
        "Cross-section routes matched.",
        group_count=len(matched_groups),
        duplicate_route_count=duplicate_route_count,
        group_keys=group_keys,
    )
    return {
        'groups': matched_groups,
        'components': [],
        'mode_label': 'Bridged open loop with corners',
        'detector_only': True,
        'partial_ranges': partial_ranges,
    }


def _attach_partial_ranges(bm, data):
    if not data or data.get('partial_ranges') is not None:
        return data

    groups = data.get('groups', [])
    if not groups:
        return data

    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        return data

    partial_ranges = []
    matched_any = False
    for group in groups:
        best_range = None
        best_score = None
        for component in selected_components:
            partial_range = _selected_subset_range_any_loop(group, component)
            if partial_range is None:
                continue
            range_score = partial_range.get('score')
            if best_score is None or range_score > best_score:
                best_range = partial_range
                best_score = range_score
        partial_ranges.append(best_range)
        matched_any = matched_any or best_range is not None

    if not matched_any:
        return data

    updated = dict(data)
    updated['partial_ranges'] = partial_ranges
    _trace_corner(
        "Attached partial ranges to detected groups.",
        group_count=len(groups),
        partial_ranges=partial_ranges,
    )
    return updated


def _detect_corner_from_open_groups(groups):
    _trace_corner(
        "Trying plain-route corner detect from open groups.",
        group_count=len(groups or []),
    )
    route_records = {}
    for group_index, group in enumerate(groups or [], start=1):
        shaft_faces = _shaft_faces_from_open_group(group)
        if not shaft_faces:
            _trace_corner(
                "Plain-route corner detect rejected.",
                group_index=group_index,
                reason="could not derive shaft faces from detected open group",
            )
            return None

        face_component, live_shaft_faces = _corner_face_component_from_shaft_faces(shaft_faces)
        extra_faces = {
            face for face in face_component
            if face not in live_shaft_faces and getattr(face, "is_valid", False)
        }
        if not extra_faces:
            _trace_corner(
                "Plain-route corner detect skipped non-corner group.",
                group_index=group_index,
                shaft_face_count=len(live_shaft_faces),
            )
            continue

        route_key = _route_face_key(live_shaft_faces)
        if not route_key:
            continue

        route_record = route_records.setdefault(
            route_key,
            {
                'group': group,
                'component_faces': set(),
                'shaft_faces': set(),
                'extra_faces': set(),
            },
        )
        route_record['component_faces'].update(face_component)
        route_record['shaft_faces'].update(live_shaft_faces)
        route_record['extra_faces'].update(extra_faces)
        _trace_corner(
            "Plain-route corner detect matched group.",
            group_index=group_index,
            component_face_count=len(face_component),
            shaft_face_count=len(live_shaft_faces),
            corner_face_count=len(extra_faces),
        )

    seed_components = [
        {
            'face_component': set(record['component_faces']),
            'shaft_faces': set(record['shaft_faces']),
            'extra_faces': set(record['extra_faces']),
        }
        for record in route_records.values()
        if record['extra_faces']
    ]
    if not seed_components:
        _trace_corner("Plain-route corner detect rejected: no corner components found.")
        return None

    _trace_corner(
        "Plain-route corner detect matched.",
        group_count=len(route_records),
        corner_component_count=len(seed_components),
    )
    return {
        'groups': [record['group'] for record in route_records.values()],
        'components': [],
        'seed_components': seed_components,
        'mode_label': 'Bridged open loop with corners',
    }


def _loop_walk_key(loop):
    return tuple(
        vert.index for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    )


def _order_open_chain_component(component_verts):
    component_set = {
        vert for vert in component_verts
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(component_set) < 2:
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component too small",
            component_size=len(component_set),
        )
        return None

    adjacency = {
        vert: sorted(
            (
                edge.other_vert(vert)
                for edge in vert.link_edges
                if edge.other_vert(vert) in component_set
            ),
            key=lambda item: item.index,
        )
        for vert in component_set
    }
    degree_counts = sorted(len(neighbors) for neighbors in adjacency.values())
    if any(len(neighbors) not in {1, 2} for neighbors in adjacency.values()):
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component degree pattern was not an open strip",
            component_size=len(component_set),
            degree_counts=degree_counts,
            bad_vertices=[
                vert.index for vert, neighbors in adjacency.items()
                if len(neighbors) not in {1, 2}
            ],
        )
        return None

    endpoints = [
        vert for vert, neighbors in adjacency.items()
        if len(neighbors) == 1
    ]
    if len(endpoints) != 2:
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component did not have exactly two endpoints",
            component_size=len(component_set),
            endpoint_count=len(endpoints),
            endpoint_indices=[vert.index for vert in endpoints],
            degree_counts=degree_counts,
        )
        return None

    start = min(endpoints, key=lambda vert: vert.index)
    ordered = [start]
    visited = {start}
    current = start

    while len(ordered) < len(component_set):
        next_vert = next(
            (neighbor for neighbor in adjacency[current] if neighbor not in visited),
            None,
        )
        if next_vert is None:
            _trace_corner(
                "Start detection ordering rejected.",
                reason="ordering walk got stuck before covering the component",
                component_size=len(component_set),
                ordered_size=len(ordered),
                current_vert=current.index,
            )
            return None
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered


def _adjacent_bridge_loops_from_seed(seed_loop):
    seed_set = {
        vert for vert in seed_loop
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(seed_set) != len(seed_loop):
        _trace_corner(
            "Adjacent-loop detect rejected.",
            reason="seed loop contained duplicate or invalid verts",
            seed_size=len(seed_loop),
            unique_size=len(seed_set),
        )
        return None

    partner_map = {}
    all_partners = set()
    for vert in seed_loop:
        partners = []
        for edge in vert.link_edges:
            other = edge.other_vert(vert)
            if other in seed_set or not getattr(other, "is_valid", False):
                continue
            partners.append(other)
        deduped_partners = sorted(set(partners), key=lambda item: item.index)
        if not deduped_partners:
            _trace_corner(
                "Adjacent-loop detect rejected.",
                reason="seed vert had no outward partners",
                seed_vert=vert.index,
            )
            return None
        partner_map[vert] = deduped_partners
        all_partners.update(deduped_partners)

    _trace_corner(
        "Adjacent-loop detect: gathered outward partners.",
        seed_size=len(seed_loop),
        partner_counts={
            vert.index: len(partners)
            for vert, partners in partner_map.items()
        },
        total_partner_count=len(all_partners),
    )

    visited = set()
    components = []
    for vert in sorted(all_partners, key=lambda item: item.index):
        if vert in visited:
            continue
        stack = [vert]
        component = set()
        visited.add(vert)
        while stack:
            current = stack.pop()
            component.add(current)
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other not in all_partners or other in visited:
                    continue
                visited.add(other)
                stack.append(other)
        components.append(component)

    full_size_components = [
        component for component in components
        if len(component) == len(seed_loop)
    ]
    ignored_components = [
        component for component in components
        if len(component) != len(seed_loop)
    ]

    if ignored_components:
        _trace_corner(
            "Adjacent-loop detect: ignoring non-matching side components.",
            seed_size=len(seed_loop),
            ignored_component_sizes=[len(component) for component in ignored_components],
        )

    if len(full_size_components) not in {1, 2}:
        _trace_corner(
            "Adjacent-loop detect rejected.",
            reason="outward partners did not form one or two full-size side components",
            component_count=len(components),
            component_sizes=[len(component) for component in components],
            full_size_component_count=len(full_size_components),
        )
        return None

    aligned_loops = []
    for component in full_size_components:
        aligned = []
        for vert in seed_loop:
            matches = [partner for partner in partner_map[vert] if partner in component]
            if len(matches) != 1:
                _trace_corner(
                    "Adjacent-loop detect rejected.",
                    reason="seed vert did not map cleanly into one side component",
                    seed_vert=vert.index,
                    match_count=len(matches),
                    component_size=len(component),
                )
                return None
            aligned.append(matches[0])

        if not _is_ordered_open_chain(aligned):
            reversed_aligned = list(reversed(aligned))
            if _is_ordered_open_chain(reversed_aligned):
                aligned = reversed_aligned
            else:
                ordered_component = _order_open_chain_component(component)
                if ordered_component is None:
                    return None
                if _is_ordered_open_chain(ordered_component):
                    aligned = ordered_component
                elif _is_ordered_open_chain(list(reversed(ordered_component))):
                    aligned = list(reversed(ordered_component))
                else:
                    _trace_corner(
                        "Adjacent-loop detect rejected.",
                        reason="side component could not be ordered into an open chain",
                        component_size=len(component),
                    )
                    return None

        aligned_loops.append(aligned)

    _trace_corner(
        "Derived adjacent bridge loops from selected cross section.",
        seed_size=len(seed_loop),
        adjacent_loop_count=len(aligned_loops),
    )
    return aligned_loops


def _next_partial_bridge_loop(current_loop, previous_loop=None):
    current_set = {
        vert for vert in current_loop
        if vert is not None and getattr(vert, "is_valid", False)
    }
    previous_set = {
        vert for vert in (previous_loop or ())
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(current_set) != len(current_loop):
        _trace_corner(
            "Partial-walk next-loop rejected.",
            reason="current loop contained duplicate or invalid verts",
            current_size=len(current_loop),
            unique_size=len(current_set),
        )
        return None

    partner_map = {}
    all_partners = set()
    for vert in current_loop:
        partners = []
        for edge in vert.link_edges:
            other = edge.other_vert(vert)
            if (
                other in current_set
                or other in previous_set
                or not getattr(other, "is_valid", False)
            ):
                continue
            partners.append(other)
        deduped_partners = sorted(set(partners), key=lambda item: item.index)
        partner_map[vert] = deduped_partners
        if deduped_partners:
            all_partners.update(deduped_partners)

    if not all_partners:
        return []

    visited = set()
    components = []
    for vert in sorted(all_partners, key=lambda item: item.index):
        if vert in visited:
            continue
        stack = [vert]
        component = set()
        visited.add(vert)
        while stack:
            current = stack.pop()
            component.add(current)
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other not in all_partners or other in visited:
                    continue
                visited.add(other)
                stack.append(other)
        components.append(component)

    full_size_components = [
        component for component in components
        if len(component) == len(current_loop)
    ]

    ignored_components = [
        component for component in components
        if len(component) != len(current_loop)
    ]
    if ignored_components:
        _trace_corner(
            "Partial-walk next-loop: ignoring non-matching side components.",
            current_size=len(current_loop),
            ignored_component_sizes=[len(component) for component in ignored_components],
        )

    if not full_size_components:
        _trace_corner(
            "Partial-walk next-loop reached terminal end.",
            current_size=len(current_loop),
            partner_component_sizes=[len(component) for component in components],
        )
        return []

    if len(full_size_components) != 1:
        _trace_corner(
            "Partial-walk next-loop rejected.",
            reason="could not isolate exactly one full-size next component",
            current_size=len(current_loop),
            component_sizes=[len(component) for component in components],
            full_size_component_count=len(full_size_components),
        )
        return None

    component = full_size_components[0]
    aligned = []
    for vert in current_loop:
        matches = [partner for partner in partner_map[vert] if partner in component]
        if len(matches) != 1:
            _trace_corner(
                "Partial-walk next-loop rejected.",
                reason="current vert did not map cleanly into next component",
                current_vert=vert.index,
                match_count=len(matches),
                component_size=len(component),
            )
            return None
        aligned.append(matches[0])

    if not _is_ordered_open_chain(aligned):
        reversed_aligned = list(reversed(aligned))
        if _is_ordered_open_chain(reversed_aligned):
            aligned = reversed_aligned
        else:
            ordered_component = _order_open_chain_component(component)
            if ordered_component is None:
                _trace_corner(
                    "Partial-walk next-loop rejected.",
                    reason="next component could not be ordered",
                    component_size=len(component),
                )
                return None
            if _is_ordered_open_chain(ordered_component):
                aligned = ordered_component
            elif _is_ordered_open_chain(list(reversed(ordered_component))):
                aligned = list(reversed(ordered_component))
            else:
                _trace_corner(
                    "Partial-walk next-loop rejected.",
                    reason="ordered next component was not an open chain",
                    component_size=len(component),
                )
                return None

    return aligned


def _walk_bridge_side(seed_loop, first_loop, visited_keys, seed_key=None, close_on_seed=False):
    if first_loop is None:
        return None
    if not first_loop:
        return {
            'loops': [],
            'closed': False,
        }

    walked = []
    previous_loop = seed_loop
    current_loop = first_loop

    while current_loop:
        current_key = _loop_walk_key(current_loop)
        if not current_key:
            return None
        if current_key in visited_keys:
            if close_on_seed and seed_key is not None and current_key == seed_key:
                _trace_corner(
                    "Partial-walk side closed back on seed.",
                    walked_loop_count=len(walked),
                )
                return {
                    'loops': walked,
                    'closed': True,
                }
            return None
        visited_keys.add(current_key)
        walked.append(current_loop)

        next_loop = _next_partial_bridge_loop(current_loop, previous_loop=previous_loop)
        if next_loop is None:
            return None

        previous_loop = current_loop
        current_loop = next_loop

    return {
        'loops': walked,
        'closed': False,
    }


def _expand_bridge_from_seed_loop(seed_loop, close_on_seed=False):
    if seed_loop is None or len(seed_loop) < 2:
        _trace_corner(
            "Seed expansion rejected.",
            reason="seed loop was missing or too small",
            seed_size=0 if seed_loop is None else len(seed_loop),
        )
        return None

    seed_key = _loop_walk_key(seed_loop)
    if not seed_key:
        _trace_corner(
            "Seed expansion rejected.",
            reason="seed loop key was empty",
            seed_size=len(seed_loop),
        )
        return None

    visited_keys = {seed_key}
    adjacent_loops = _adjacent_bridge_loops_from_seed(seed_loop)
    if adjacent_loops is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="could not derive adjacent loops from selected strip",
            seed_size=len(seed_loop),
        )
        return None

    first_side = adjacent_loops[0] if adjacent_loops else []
    second_side = adjacent_loops[1] if len(adjacent_loops) > 1 else []

    left_result = _walk_bridge_side(
        seed_loop,
        first_side,
        visited_keys,
        seed_key=seed_key,
        close_on_seed=close_on_seed,
    )
    if left_result is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="left-side walk failed",
            first_side_size=len(first_side) if first_side else 0,
        )
        return None
    left_loops = left_result['loops']

    if left_result.get('closed'):
        expanded_loops = [list(seed_loop)] + left_loops
        if len(expanded_loops) < 2:
            return None
        _trace_corner(
            "Expanded closed bridge from selected cross section.",
            seed_size=len(seed_loop),
            expanded_loop_count=len(expanded_loops),
        )
        return (expanded_loops, True)

    right_result = _walk_bridge_side(
        seed_loop,
        second_side,
        visited_keys,
        seed_key=seed_key,
        close_on_seed=close_on_seed,
    )
    if right_result is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="right-side walk failed",
            second_side_size=len(second_side) if second_side else 0,
        )
        return None
    right_loops = right_result['loops']

    if right_result.get('closed'):
        expanded_loops = [list(seed_loop)] + right_loops
        if len(expanded_loops) < 2:
            return None
        _trace_corner(
            "Expanded closed bridge from selected cross section.",
            seed_size=len(seed_loop),
            expanded_loop_count=len(expanded_loops),
        )
        return (expanded_loops, True)

    expanded_loops = list(reversed(left_loops)) + [list(seed_loop)] + right_loops
    if len(expanded_loops) < 2:
        return None

    _trace_corner(
        "Expanded bridge from selected cross section.",
        seed_size=len(seed_loop),
        expanded_loop_count=len(expanded_loops),
    )
    return (expanded_loops, False)


def _expand_open_bridge_from_seed_loop(seed_loop):
    return _expand_bridge_from_seed_loop(seed_loop, close_on_seed=False)


def _expand_closed_bridge_from_seed_loop(seed_loop):
    return _expand_bridge_from_seed_loop(seed_loop, close_on_seed=True)


def _find_group_from_seed_loop(groups, selected_verts):
    selected_set = {
        vert for vert in selected_verts
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_group = None
    best_score = None
    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if is_closed or not loops:
            continue
        for loop_index, loop in enumerate(loops, start=1):
            loop_set = {
                vert for vert in loop
                if getattr(vert, "is_valid", False)
            }
            match_count = len(selected_set & loop_set)
            if match_count == 0:
                continue
            score = (match_count, len(loop_set), -group_index, -loop_index)
            if best_score is None or score > best_score:
                best_score = score
                best_group = group

    if best_group is None:
        return None

    _trace_corner(
        "Selected cross section matched an open group.",
        selected_count=len(selected_set),
        loop_count=len(best_group[0]) if best_group and best_group[0] else 0,
    )
    return best_group


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
                'split_kind': 'boundary',
                'anchor_home': positions[0].copy(),
                'clusters': clusters,
            })

    return split_infos


def _split_infos_from_sections(bm, sections, vert_lookup=None):
    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)

    split_infos = []
    for section in sections:
        positions = [
            vert.co.copy()
            for vert in section
            if getattr(vert, "is_valid", False)
        ]
        if len(positions) < 2:
            continue

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
                'split_kind': 'section',
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


def _separate_stuck_boundary_tips(bm, boundary_components, seam_index=None):
    changed = False
    vert_lookup = _build_vert_position_lookup(bm)
    lookup_dirty = False

    def _live_verts(position):
        nonlocal vert_lookup, lookup_dirty
        if lookup_dirty:
            vert_lookup = _build_vert_position_lookup(bm)
            lookup_dirty = False
        return _verts_at_position(bm, position, lookup=vert_lookup)

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
            live_tip_verts = _live_verts(tip_position)
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
            lookup_dirty = True
            _debug_step(
                "tip vert separate",
                seam_index=seam_index,
                component_index=component_index,
                result_count=len(separated),
            )
            if len(separated) > 1:
                changed = True
                continue

            live_tip_verts = _live_verts(tip_position)
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
                live_tip_verts = _live_verts(tip_position)
                if len(live_tip_verts) > 1:
                    break
                try:
                    bmesh.utils.loop_separate(loop)
                    lookup_dirty = True
                    _debug_step(
                        "tip loop separated",
                        seam_index=seam_index,
                        component_index=component_index,
                        face_index=getattr(loop.face, "index", None),
                        live_tip_count=len(_live_verts(tip_position)),
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
        boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        shaft_faces = _live_bmesh_items(record.get('shaft_faces', set()))
        corner_faces = _live_bmesh_items(record.get('corner_faces', set()))
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


def _attached_split_seam_records(bm, seam_records, extra_split_edges=None):
    if not seam_records:
        return []

    boundary_components = []
    interface_chain_verts = set()
    split_edges = set()
    for record in seam_records:
        boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        shaft_faces = _live_bmesh_items(record.get('shaft_faces', set()))
        corner_faces = _live_bmesh_items(record.get('corner_faces', set()))
        if not boundary_edges or not shaft_faces or not corner_faces:
            continue

        boundary_components.append({
            'boundary_edges': set(boundary_edges),
            'corner_faces': set(corner_faces),
        })
        interface_chain_verts.update(_split_control_verts(boundary_edges))
        split_edges.update(boundary_edges)
        split_edges.update(_tip_shaft_rail_edges(boundary_edges, shaft_faces, corner_faces))

    if extra_split_edges:
        split_edges.update(
            edge for edge in extra_split_edges
            if edge is not None and getattr(edge, "is_valid", False)
        )

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
        boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
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


def _weld_boundary_positions(bm, positions, vert_lookup=None):
    if not positions:
        return 0

    if vert_lookup is None:
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


def _weld_selected_source_endpoint_seam_ids(bm, layer=None):
    if layer is None:
        layer = bm.verts.layers.int.get(_SOURCE_ENDPOINT_SEAM_ID_LAYER)
        if layer is None:
            return 0

    tagged_verts = {}
    for vert in bm.verts:
        if not getattr(vert, "is_valid", False):
            continue
        seam_id = vert[layer]
        if seam_id > 0 and seam_id not in tagged_verts:
            tagged_verts[seam_id] = vert

    if not tagged_verts:
        return 0

    vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for seam_id, target_vert in tagged_verts.items():
        live_verts = _verts_at_position(bm, target_vert.co.copy(), lookup=vert_lookup)
        for vert in live_verts:
            if vert is not target_vert and getattr(vert, "is_valid", False):
                targetmap[vert] = target_vert

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


def _pin_live_open_loop_pair_to_split_positions(loop_a, loop_b, split_info):
    clusters = split_info.get('clusters', [])
    positions = [
        cluster.get('position')
        for cluster in clusters
        if cluster.get('position') is not None
    ]
    if len(positions) < 2:
        return 0

    pinned = 0
    if len(positions) == len(loop_a) == len(loop_b):
        for position, vert_a, vert_b in zip(positions, loop_a, loop_b):
            if getattr(vert_a, "is_valid", False):
                vert_a.co = position.copy()
                pinned += 1
            if getattr(vert_b, "is_valid", False):
                vert_b.co = position.copy()
                pinned += 1
        return pinned

    endpoint_specs = (
        (positions[0], loop_a[0], loop_b[0]),
        (positions[-1], loop_a[-1], loop_b[-1]),
    )
    for position, vert_a, vert_b in endpoint_specs:
        if getattr(vert_a, "is_valid", False):
            vert_a.co = position.copy()
            pinned += 1
        if getattr(vert_b, "is_valid", False):
            vert_b.co = position.copy()
            pinned += 1
    return pinned


def _weld_open_groups(bm, groups, split_infos):
    if not groups:
        return 0

    total_merged = 0
    targetmap = {}
    for split_info in split_infos:
        pair = _live_open_loop_pair(groups, split_info)
        if pair is None:
            continue
        loop_a, loop_b = pair
        _pin_live_open_loop_pair_to_split_positions(loop_a, loop_b, split_info)
        for vert_a, vert_b in zip(loop_a, loop_b):
            if (
                not getattr(vert_a, "is_valid", False)
                or not getattr(vert_b, "is_valid", False)
                or vert_a is vert_b
            ):
                continue
            targetmap[vert_b] = vert_a

    if targetmap:
        try:
            bmesh.ops.weld_verts(bm, targetmap=targetmap)
            total_merged += len(targetmap)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            bm.normal_update()
        except Exception:
            return total_merged

    fallback_targetmap = _targetmap_from_live_split_positions(bm, split_infos)
    if fallback_targetmap:
        try:
            bmesh.ops.weld_verts(bm, targetmap=fallback_targetmap)
            total_merged += len(fallback_targetmap)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            bm.normal_update()
        except Exception:
            return total_merged

    return total_merged


def _weld_live_open_loops(bm, split_infos):
    live_data = detect_open_strip_selection(bm)
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


def _attached_weld_live_open_loops(bm, split_infos):
    live_data = _attached_detect_open_strip_selection(bm)
    groups = live_data.get('groups') if live_data else []
    return _weld_open_groups(bm, groups, split_infos)


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


def _targetmap_from_live_split_positions(bm, split_infos, vert_lookup=None):
    if vert_lookup is None:
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


def _find_unwelded_split_verts(bm, split_infos, vert_lookup=None):
    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)

    unmatched = []
    seen = set()
    for split_info in split_infos or []:
        split_kind = split_info.get('split_kind', 'unknown')
        for cluster_index, cluster in enumerate(split_info.get('clusters', []), start=1):
            position = cluster.get('position')
            if position is None:
                continue
            position_key = _position_key(position, _POSITION_TOLERANCE)
            dedupe_key = (split_kind, position_key)
            if dedupe_key in seen:
                continue
            seen.add(dedupe_key)

            live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
            if len(live_verts) <= 1:
                continue

            unmatched.append({
                'split_kind': split_kind,
                'cluster_index': cluster_index,
                'kept_vert': live_verts[0].index,
                'unwelded_vert_indices': [vert.index for vert in live_verts[1:]],
                'live_vert_indices': [vert.index for vert in live_verts],
            })

    return unmatched


def _log_unwelded_split_verts(bm, split_infos):
    unmatched = _find_unwelded_split_verts(bm, split_infos)
    for item in unmatched:
        print(
            "[vertex_resampler:unwelded] "
            f"split_kind={item['split_kind']} "
            f"cluster_index={item['cluster_index']} "
            f"kept_vert=v{item['kept_vert']} "
            f"unwelded={item['unwelded_vert_indices']} "
            f"live={item['live_vert_indices']}"
        )
    return unmatched


def _find_unwelded_source_loop_verts(bm, loop_positions_list, vert_lookup=None):
    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)

    unmatched = []
    seen = set()
    for loop_index, loop_positions in enumerate(loop_positions_list or [], start=1):
        if not loop_positions:
            continue

        position_specs = (
            ("start", loop_positions[0]),
            ("end", loop_positions[-1]),
        )
        for endpoint_label, position in position_specs:
            position_key = _position_key(position, _POSITION_TOLERANCE)
            dedupe_key = (loop_index, endpoint_label, position_key)
            if dedupe_key in seen:
                continue
            seen.add(dedupe_key)

            live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
            if len(live_verts) <= 1:
                continue

            unmatched.append({
                'loop_index': loop_index,
                'endpoint': endpoint_label,
                'kept_vert': live_verts[0].index,
                'unwelded_vert_indices': [vert.index for vert in live_verts[1:]],
                'live_vert_indices': [vert.index for vert in live_verts],
            })

    return unmatched


def _log_unwelded_source_loop_verts(bm, loop_positions_list):
    unmatched = _find_unwelded_source_loop_verts(bm, loop_positions_list)
    for item in unmatched:
        print(
            "[vertex_resampler:unwelded] "
            "split_kind=source_loop "
            f"loop_index={item['loop_index']} "
            f"endpoint={item['endpoint']} "
            f"kept_vert=v{item['kept_vert']} "
            f"unwelded={item['unwelded_vert_indices']} "
            f"live={item['live_vert_indices']}"
        )
    return unmatched


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


def _loop_key(loop):
    ids = tuple(
        vert.index for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    )
    if not ids:
        return None
    reversed_ids = tuple(reversed(ids))
    return min(ids, reversed_ids)


def _loop_center(loop):
    live_verts = [
        vert for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    ]
    if not live_verts:
        return None

    center = live_verts[0].co.copy()
    for vert in live_verts[1:]:
        center += vert.co
    center /= len(live_verts)
    return center


def _world_loop_segments(obj, loop, is_closed=False):
    if obj is None:
        return []

    matrix_world = obj.matrix_world
    segments = []
    for vert_a, vert_b in zip(loop, loop[1:]):
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            continue
        segments.append((
            matrix_world @ vert_a.co.copy(),
            matrix_world @ vert_b.co.copy(),
        ))
    if is_closed and len(loop) > 2:
        vert_a = loop[-1]
        vert_b = loop[0]
        if (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            segments.append((
                matrix_world @ vert_a.co.copy(),
                matrix_world @ vert_b.co.copy(),
            ))
    return segments


def _chain_edges(loop, is_closed=False):
    edges = []
    pair_iter = zip(loop, loop[1:])
    for vert_a, vert_b in pair_iter:
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            return None
        edge = next(
            (
                item for item in vert_a.link_edges
                if getattr(item, "is_valid", False) and item.other_vert(vert_a) is vert_b
            ),
            None,
        )
        if edge is None:
            return None
        edges.append(edge)

    if is_closed and len(loop) > 2:
        vert_a = loop[-1]
        vert_b = loop[0]
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            return None
        edge = next(
            (
                item for item in vert_a.link_edges
                if getattr(item, "is_valid", False) and item.other_vert(vert_a) is vert_b
            ),
            None,
        )
        if edge is None:
            return None
        edges.append(edge)

    return edges


def _sharp_corner_indices(points, threshold_degrees=45.0, is_closed=False):
    if len(points) < 3:
        return []

    threshold_cos = __import__("math").cos(__import__("math").radians(threshold_degrees))
    indices = []
    point_count = len(points)
    if is_closed:
        index_iter = range(point_count)
    else:
        index_iter = range(1, point_count - 1)

    for index in index_iter:
        prev_point = points[index - 1]
        current_point = points[index]
        next_point = points[(index + 1) % point_count]
        if prev_point is None or current_point is None or next_point is None:
            continue

        incoming = _safe_normalized(current_point - prev_point)
        outgoing = _safe_normalized(next_point - current_point)
        if incoming is None or outgoing is None:
            continue
        if incoming.dot(outgoing) <= threshold_cos:
            indices.append(index)

    return indices


def _cluster_corner_indices(indices, min_gap=2):
    ordered = sorted(set(indices))
    if not ordered:
        return []

    clusters = [[ordered[0]]]
    for index in ordered[1:]:
        if index - clusters[-1][-1] <= min_gap:
            clusters[-1].append(index)
        else:
            clusters.append([index])

    clustered = []
    for cluster in clusters:
        clustered.extend(cluster)
    return clustered


def _detected_cross_sections(groups, partial_ranges=None):
    end_sections = []
    corner_sections = []
    all_sections = []
    collected_section_indices = []

    _trace_focus(
        "Preview build started.",
        group_count=len(groups or []),
        partial_range_count=len(partial_ranges or []),
    )

    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if not loops:
            continue

        if len(loops) < 2:
            continue
        if is_closed and len(loops) < 3:
            continue

        centers = [_loop_center(loop) for loop in loops]
        end_indices = set() if is_closed else {0, len(loops) - 1}
        raw_corner_indices = [
            index for index in _sharp_corner_indices(centers, is_closed=is_closed)
            if is_closed or 0 < index < len(loops) - 1
        ]
        corner_indices = _cluster_corner_indices(raw_corner_indices)
        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]

        _trace_focus(
            "Preview group analysis.",
            group_index=group_index,
            loop_count=len(loops),
            loop_size=len(loops[0]) if loops and loops[0] else 0,
            is_closed=is_closed,
            end_indices=sorted(end_indices),
            raw_corner_indices=raw_corner_indices,
            corner_indices=corner_indices,
            partial_range=partial_range,
            group_key=_group_debug_key(group),
        )

        for index in sorted(end_indices):
            end_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        for index in corner_indices:
            corner_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        group_section_indices = []
        for loop_index, loop in enumerate(loops):
            all_sections.append(_slice_loop_for_partial_range(loop, partial_range))
            group_section_indices.append(loop_index)

        collected_section_indices.append({
            'group_index': group_index,
            'section_indices': group_section_indices,
        })

        _trace_focus(
            "Preview group output.",
            group_index=group_index,
            collected_section_indices=group_section_indices,
            collected_section_count=len(group_section_indices),
            end_section_count=len(sorted(end_indices)),
            corner_section_count=len(corner_indices),
        )

    return end_sections, corner_sections, all_sections, collected_section_indices


def _attached_detected_cross_sections(groups, partial_ranges=None):
    end_sections = []
    corner_sections = []
    all_sections = []
    collected_section_indices = []

    _trace_focus(
        "Attached-case preview build started.",
        group_count=len(groups or []),
        partial_range_count=len(partial_ranges or []),
    )

    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if not loops:
            continue

        if len(loops) < 2:
            continue
        if is_closed and len(loops) < 3:
            continue

        centers = [_loop_center(loop) for loop in loops]
        end_indices = set() if is_closed else {0, len(loops) - 1}
        raw_corner_indices = [
            index for index in _sharp_corner_indices(centers, is_closed=is_closed)
            if is_closed or 0 < index < len(loops) - 1
        ]
        corner_indices = _cluster_corner_indices(raw_corner_indices)
        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]

        _trace_focus(
            "Attached-case preview group analysis.",
            group_index=group_index,
            loop_count=len(loops),
            loop_size=len(loops[0]) if loops and loops[0] else 0,
            is_closed=is_closed,
            end_indices=sorted(end_indices),
            raw_corner_indices=raw_corner_indices,
            corner_indices=corner_indices,
            partial_range=partial_range,
            group_key=_group_debug_key(group),
        )

        for index in sorted(end_indices):
            end_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        for index in corner_indices:
            corner_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        group_section_indices = []
        for loop_index, loop in enumerate(loops):
            all_sections.append(_slice_loop_for_partial_range(loop, partial_range))
            group_section_indices.append(loop_index)

        collected_section_indices.append({
            'group_index': group_index,
            'section_indices': group_section_indices,
        })

        _trace_focus(
            "Attached-case preview group output.",
            group_index=group_index,
            collected_section_indices=group_section_indices,
            collected_section_count=len(group_section_indices),
            end_section_count=len(sorted(end_indices)),
            corner_section_count=len(corner_indices),
        )

    return end_sections, corner_sections, all_sections, collected_section_indices


def _capture_detected_cross_sections(obj, end_sections, corner_sections, all_sections=None):
    if obj is None:
        return

    end_points = []
    corner_points = []
    all_points = []
    segments = []
    matrix_world = obj.matrix_world

    for section in end_sections:
        center = _loop_center(section)
        if center is not None:
            end_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    for section in corner_sections:
        center = _loop_center(section)
        if center is not None:
            corner_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    for section in all_sections or []:
        center = _loop_center(section)
        if center is not None:
            all_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    # if segments:
    #     anchor_overlay.add_segments(segments)


def _attached_data_has_attached_outside_geometry(data):
    if not data:
        return False

    if any(component.get('outside_side_faces') for component in data.get('components', [])):
        return True

    groups = data.get('groups', [])
    if not groups:
        return False

    return bool(_attached_outer_boundary_seam_records_from_groups(groups))


def _detect_attached_open_corner_case(bm):
    _trace_focus("Detect started.")
    seed_loops = _selected_seed_loops(bm)
    if seed_loops:
        groups = []
        partial_ranges = []
        for component_index, seed_loop in enumerate(seed_loops, start=1):
            seed_group = _expand_open_bridge_from_seed_loop(seed_loop)
            if seed_group is None:
                seed_group = _expand_closed_bridge_from_seed_loop(seed_loop)
            if seed_group is None:
                _trace_focus(
                    "Selected seed expansion failed.",
                    component_index=component_index,
                    seed_size=len(seed_loop),
                )
                return None
            groups.append(seed_group)
            partial_range = _selected_subset_range_any_loop(seed_group, seed_loop)
            if partial_range is None:
                partial_range = {
                    'loop_index': 0,
                    'start_index': 0,
                    'end_index': len(seed_loop) - 1,
                    'score': (len(seed_loop), 1.0, 1.0, 0.0, 0),
                }
            partial_ranges.append(partial_range)

        _trace_focus(
            "Detect path chosen: selected seed loops.",
            group_count=len(groups),
            seed_sizes=[len(seed_loop) for seed_loop in seed_loops],
            loop_counts=[len(group[0]) for group in groups],
        )
        result = {
            'groups': groups,
            'components': [],
            'mode_label': 'Bridged open loop with corners',
            'detector_only': True,
            'partial_ranges': partial_ranges,
        }
        if _attached_data_has_attached_outside_geometry(result):
            result['outside_geometry_case'] = True
            return result
        return None

    data = _attached_detect_open_strip_selection(bm)
    if data is not None and _groups_are_open(data['groups']):
        selected_verts = _selected_verts(bm)
        seed_group = _find_group_from_seed_loop(data['groups'], selected_verts)
        if seed_group is not None:
            _trace_focus(
                "Detect path chosen: matched selected cross section inside open groups.",
                group_count=len(data.get('groups', [])),
            )
            result = _attach_partial_ranges(bm, {
                'groups': [seed_group],
                'components': data.get('components', []),
                'mode_label': 'Bridged open loop with corners',
                'detector_only': True,
            })
            if _attached_data_has_attached_outside_geometry(result):
                result['outside_geometry_case'] = True
                return result
            return None

        _trace_focus(
            "Detect path chosen: generic open-group scan.",
            group_count=len(data.get('groups', [])),
            loop_counts=[len(group[0]) for group in data.get('groups', [])],
            loop_sizes=[
                len(group[0][0]) if group[0] else 0
                for group in data.get('groups', [])
            ],
        )
        result = _attach_partial_ranges(bm, {
            'groups': data['groups'],
            'components': data.get('components', []),
            'mode_label': 'Bridged open loop with corners',
            'detector_only': True,
        })
        if _attached_data_has_attached_outside_geometry(result):
            result['outside_geometry_case'] = True
            return result
        return None

    _trace_focus("Detect failed.")
    return None


def detect(bm):
    data = _detect_attached_open_corner_case(bm)
    return data


def _attached_prepare_execution_context(bm, obj, data):
    selection_state = _capture_selection_state(bm)
    source_loop_specs = _attached_source_loop_specs(
        data.get('groups', []),
        partial_ranges=data.get('partial_ranges'),
    )
    source_endpoint_seam_layer = _tag_selected_source_loop_endpoints(
        bm,
        source_loop_specs,
    )
    execute_plan = _attached_build_open_corner_execute_plan(data)
    end_sections = execute_plan['end_sections']
    corner_sections = execute_plan['corner_sections']
    all_sections = execute_plan['all_sections']

    _capture_detected_cross_sections(
        obj,
        end_sections,
        corner_sections,
        all_sections=all_sections,
    )
    _trace_focus(
        "Preview build finished.",
        end_cross_section_count=len(end_sections),
        corner_cross_section_count=len(corner_sections),
        propagated_cross_section_count=len(all_sections),
        collected_section_indices=execute_plan['collected_section_indices'],
    )

    return {
        'selection_state': selection_state,
        'source_loop_specs': source_loop_specs,
        'source_endpoint_seam_layer': source_endpoint_seam_layer,
        'execute_plan': execute_plan,
        'end_sections': end_sections,
        'corner_sections': corner_sections,
        'segment_sections': execute_plan['segment_sections'],
        'segment_section_positions': execute_plan['segment_section_positions'],
        'collected_section_indices': execute_plan['collected_section_indices'],
        'shaft_face_positions': execute_plan['shaft_face_positions'],
        'segment_split_edges': execute_plan['segment_split_edges'],
        'split_edges': execute_plan['split_edges'],
        'split_section_logs': execute_plan['split_section_logs'],
        'split_mode': execute_plan['split_mode'],
        'seam_records': execute_plan['seam_records'],
    }


def _attached_trace_execution_plan(bm, context):
    initial_shaft_faces = _live_faces_from_positions(
        bm,
        context['shaft_face_positions'],
    )
    _attached_trace_split_debug(
        "Planned separation.",
        split_mode=context['split_mode'],
        end_section_count=len(context['end_sections']),
        corner_section_count=len(context['corner_sections']),
        interior_section_count=len(context['segment_sections']),
        seam_record_count=len(context['seam_records']),
        split_edge_count=len(context['split_edges']),
        initial_shaft_face_count=len(initial_shaft_faces),
        initial_shaft_components=_attached_shaft_component_summary(initial_shaft_faces),
        split_section_logs=context['split_section_logs'],
    )


def _attached_detach_outer_geometry(bm, context):
    split_infos = []

    if context['seam_records']:
        split_infos.extend(_attached_split_seam_records(bm, context['seam_records']))
        detached_shaft_faces = _live_faces_from_positions(
            bm,
            context['shaft_face_positions'],
        )
        _attached_trace_split_debug(
            "After outside detach split.",
            shaft_face_count=len(detached_shaft_faces),
            shaft_components=_attached_shaft_component_summary(detached_shaft_faces),
        )
        return split_infos

    bmesh.ops.split_edges(bm, edges=list(context['split_edges']))
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.select_flush_mode()
    bm.normal_update()
    split_infos.extend(_split_infos_from_sections(bm, context['segment_sections']))
    detached_shaft_faces = _live_faces_from_positions(
        bm,
        context['shaft_face_positions'],
    )
    _attached_trace_split_debug(
        "After direct section split.",
        shaft_face_count=len(detached_shaft_faces),
        shaft_components=_attached_shaft_component_summary(detached_shaft_faces),
    )
    return split_infos


def _attached_split_internal_sections(bm, context, split_infos):
    if not (context['seam_records'] and context['segment_split_edges']):
        return split_infos

    live_segment_edges = _live_bmesh_items(context['segment_split_edges'])
    live_segment_control_verts = _split_control_verts(live_segment_edges)
    if context['segment_section_positions']:
        vert_lookup = _build_vert_position_lookup(bm)
        _attached_trace_split_debug(
            "Before interior seam split.",
            section_cluster_summary=_attached_section_cluster_summary(
                bm,
                context['segment_section_positions'],
                vert_lookup=vert_lookup,
            ),
        )
    _attached_trace_split_debug(
        "Planned interior seam edges.",
        live_edge_count=len(live_segment_edges),
        live_control_vert_count=len(live_segment_control_verts),
        live_edge_indices=sorted(
            edge.index for edge in live_segment_edges
            if getattr(edge, "is_valid", False)
        ),
    )
    if not live_segment_edges:
        return split_infos

    bmesh.ops.split_edges(
        bm,
        edges=list(live_segment_edges),
        verts=list(live_segment_control_verts),
        use_verts=bool(live_segment_control_verts),
    )
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.select_flush_mode()
    bm.normal_update()
    split_infos.extend(_split_infos_from_sections(bm, context['segment_sections']))
    post_internal_split_faces = _live_faces_from_positions(
        bm,
        context['shaft_face_positions'],
    )
    split_details = {
        'shaft_face_count': len(post_internal_split_faces),
        'shaft_components': _attached_shaft_component_summary(post_internal_split_faces),
    }
    if context['segment_section_positions']:
        split_details['section_cluster_summary'] = _attached_section_cluster_summary(
            bm,
            context['segment_section_positions'],
        )
    _attached_trace_split_debug("After interior seam split.", **split_details)
    return split_infos


def _attached_recover_detached_shaft(bm, context):
    live_shaft_faces = _live_faces_from_positions(
        bm,
        context['shaft_face_positions'],
    )
    if not live_shaft_faces:
        return []

    _select_only_faces(bm, live_shaft_faces)
    bm.select_flush_mode()
    bm.normal_update()
    return live_shaft_faces


def _attached_resample_middle_segments(
    bm,
    obj,
    direction,
    report,
    context,
    groups,
):
    fresh_groups = [
        _attached_orient_fresh_group_to_source(
            context['source_loop_specs'],
            rings_data,
        )
        for rings_data in (groups or [])
    ]
    segment_count = len(fresh_groups)
    source_targets = _attached_source_targets_for_fresh_groups(
        context['source_loop_specs'],
        fresh_groups,
    )
    source_target_lookup = {
        target['group_index']: target for target in source_targets
    }
    final_source_loop_positions = []
    resampled_segment_count = 0
    resampled_groups = []

    _trace_focus(
        "Connected corner resample plan.",
        group_count=segment_count,
    )

    if not fresh_groups:
        return {
            'segment_count': segment_count,
            'resampled_segment_count': resampled_segment_count,
            'resampled_groups': resampled_groups,
            'final_source_loop_positions': final_source_loop_positions,
        }

    for group_index, rings_data in enumerate(fresh_groups, start=1):
        loops, is_closed = rings_data
        if is_closed or not loops:
            continue

        anchored_group = _anchor_open_group_endpoints(rings_data)
        _trace_focus(
            "Connected corner resample execute.",
            group_index=group_index,
            loop_count=len(loops),
            loop_size=len(loops[0]) if loops else 0,
        )
        result_info = {}
        result = execute_aligned_loops_logic(
            bm,
            obj,
            anchored_group['rings'],
            direction,
            report=report,
            use_seams=anchored_group.get('use_seams', True),
            migrate_seams=anchored_group.get('migrate_seams'),
            max_seams=anchored_group.get('max_seams'),
            forced_seam_verts=anchored_group.get('forced_seam_verts'),
            result_info=result_info,
        )
        if result == {'CANCELLED'}:
            continue
        resampled_segment_count += 1
        target = source_target_lookup.get(group_index - 1)
        ring_group = result_info.get('ring_group')
        if ring_group is not None:
            resampled_groups.append((
                [list(ring_info.verts) for ring_info in ring_group.rings],
                False,
            ))
        if target is not None and ring_group is not None:
            target_loop_index = target['loop_index']
            if 0 <= target_loop_index < len(ring_group.rings):
                final_source_loop_positions.append([
                    vert.co.copy()
                    for vert in ring_group.rings[target_loop_index].verts
                    if getattr(vert, "is_valid", False)
                ])

    return {
        'segment_count': segment_count,
        'resampled_segment_count': resampled_segment_count,
        'resampled_groups': resampled_groups,
        'final_source_loop_positions': final_source_loop_positions,
    }


def _attached_split_infos_by_kind(split_infos):
    section_split_infos = [
        split_info for split_info in split_infos
        if split_info.get('split_kind') == 'section'
    ]
    boundary_split_infos = [
        split_info for split_info in split_infos
        if split_info.get('split_kind') == 'boundary'
    ]
    return section_split_infos, boundary_split_infos


def _attached_weld_middle_segments(bm, resampled_groups, split_infos):
    section_split_infos, _boundary_split_infos = _attached_split_infos_by_kind(split_infos)
    return _weld_open_groups(
        bm,
        resampled_groups,
        section_split_infos,
    )


def _attached_boundary_record_position_specs(bm, seam_records, boundary_split_infos):
    record_specs = []
    used_split_indices = set()

    for record in seam_records or []:
        live_boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        live_corner_faces = _live_bmesh_items(record.get('corner_faces', set()))
        chain = _ordered_chain_verts(live_boundary_edges)
        if not chain or not live_corner_faces:
            continue

        anchor_position = chain[0].co.copy()
        best_split_index = None
        best_split_distance = None
        for split_index, split_info in enumerate(boundary_split_infos):
            if split_index in used_split_indices:
                continue
            distance = (split_info['anchor_home'] - anchor_position).length
            if best_split_distance is None or distance < best_split_distance:
                best_split_distance = distance
                best_split_index = split_index

        if best_split_index is None:
            continue

        used_split_indices.add(best_split_index)
        record_specs.append({
            'corner_faces': live_corner_faces,
            'positions': [
                cluster['position'].copy()
                for cluster in boundary_split_infos[best_split_index].get('clusters', [])
                if cluster.get('position') is not None
            ],
        })

    return record_specs


def _attached_outer_targetmap_from_seam_positions(bm, context, split_infos):
    _section_split_infos, boundary_split_infos = _attached_split_infos_by_kind(split_infos)
    if not boundary_split_infos:
        return {}

    record_specs = _attached_boundary_record_position_specs(
        bm,
        context.get('seam_records', []),
        boundary_split_infos,
    )
    if not record_specs:
        return {}

    vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for record_spec in record_specs:
        corner_faces = record_spec['corner_faces']
        for position in record_spec['positions']:
            live_verts = _verts_at_position(
                bm,
                position,
                lookup=vert_lookup,
            )
            if len(live_verts) < 2:
                continue

            outside_verts = [
                vert for vert in live_verts
                if any(face in corner_faces for face in vert.link_faces)
            ]
            if not outside_verts:
                continue

            target = outside_verts[0]
            for vert in live_verts:
                if vert is target or not getattr(vert, "is_valid", False):
                    continue
                targetmap[vert] = target

    return targetmap


def _attached_weld_outer_geometry(bm, context, split_infos):
    boundary_targetmap = _attached_outer_targetmap_from_seam_positions(
        bm,
        context,
        split_infos,
    )
    if not boundary_targetmap:
        _section_split_infos, boundary_split_infos = _attached_split_infos_by_kind(split_infos)
        if not boundary_split_infos:
            return 0

        boundary_targetmap = _targetmap_from_live_split_positions(
            bm,
            boundary_split_infos,
        )
    if not boundary_targetmap:
        return 0

    try:
        bmesh.ops.weld_verts(bm, targetmap=boundary_targetmap)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.normal_update()
        return len(boundary_targetmap)
    except Exception:
        return 0


def _attached_weld_selected_cross_section_cleanup(bm, context):
    return _weld_selected_source_endpoint_seam_ids(
        bm,
        layer=context['source_endpoint_seam_layer'],
    )


def _attached_restore_source_loop_selection(bm, final_source_loop_positions):
    if not final_source_loop_positions:
        return

    _log_unwelded_source_loop_verts(bm, final_source_loop_positions)
    _select_loops_from_positions(bm, final_source_loop_positions)


def _attached_select_outer_connection_verts(bm, context, split_infos):
    _section_split_infos, boundary_split_infos = _attached_split_infos_by_kind(split_infos)
    seam_records = context.get('seam_records', [])
    if not seam_records and not boundary_split_infos:
        return False

    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    vert_lookup = _build_vert_position_lookup(bm)
    position_keys = set()
    candidate_positions = []

    def _add_position(position):
        if position is None:
            return
        position_key = _position_key(position, _POSITION_TOLERANCE)
        if position_key in position_keys:
            return
        position_keys.add(position_key)
        candidate_positions.append(position.copy())

    any_selected = False
    seen_vert_indices = set()

    for split_info in boundary_split_infos:
        for cluster in split_info.get('clusters', []):
            live_cluster_verts = [
                vert for vert in cluster.get('verts', [])
                if getattr(vert, "is_valid", False)
            ]
            if live_cluster_verts:
                for vert in live_cluster_verts:
                    _add_position(vert.co)
            else:
                _add_position(cluster.get('position'))

    for record in seam_records:
        for edge in _live_bmesh_items(record.get('boundary_edges', set())):
            for vert in edge.verts:
                if getattr(vert, "is_valid", False):
                    _add_position(vert.co)

    for position in candidate_positions:
        for vert in _verts_at_position(bm, position, lookup=vert_lookup):
            if not getattr(vert, "is_valid", False):
                continue
            if vert.index in seen_vert_indices:
                continue
            seen_vert_indices.add(vert.index)
            vert.select = True
            any_selected = True

    for split_info in boundary_split_infos:
        for cluster in split_info.get('clusters', []):
            for vert in cluster.get('verts', []):
                if not getattr(vert, "is_valid", False):
                    continue
                if vert.index in seen_vert_indices:
                    continue
                seen_vert_indices.add(vert.index)
                vert.select = True
                any_selected = True

    for record in seam_records:
        live_boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        for edge in live_boundary_edges:
            for vert in edge.verts:
                if not getattr(vert, "is_valid", False):
                    continue
                if vert.index in seen_vert_indices:
                    continue
                seen_vert_indices.add(vert.index)
                vert.select = True
                any_selected = True

    return any_selected


def _attached_log_weld_leftovers(bm, split_infos):
    _log_unwelded_split_verts(bm, split_infos)


def _attached_restore_final_selection(bm, context, final_source_loop_positions=None):
    if final_source_loop_positions and _select_loops_from_positions(
        bm,
        final_source_loop_positions,
    ):
        return

    final_data = _attached_detect_open_strip_selection(bm)
    final_groups = final_data.get('groups', []) if final_data else []
    if _attached_select_source_loops(
        bm,
        final_groups,
        context['source_loop_specs'],
    ):
        return

    _restore_selection_state(bm, context['selection_state'])


def _attached_finalize_execution(bm, obj, context):
    _clear_source_endpoint_seam_ids(
        bm,
        layer=context['source_endpoint_seam_layer'],
    )
    _attached_restore_final_selection(bm, context)
    bmesh.update_edit_mesh(obj.data)


def _execute_attached_open_corner_case(bm, obj, direction, report=None, data=None):
    if data is None:
        data = _detect_attached_open_corner_case(bm)
    if not data:
        _trace_focus("Execute cancelled: detect returned no data.")
        return {'CANCELLED'}

    context = _attached_prepare_execution_context(bm, obj, data)
    try:
        resample_result = _attached_resample_middle_segments(
            bm,
            obj,
            direction,
            report,
            context,
            data.get('groups', []),
        )
        _attached_restore_final_selection(
            bm,
            context,
            final_source_loop_positions=resample_result['final_source_loop_positions'],
        )
    finally:
        _clear_source_endpoint_seam_ids(
            bm,
            layer=context['source_endpoint_seam_layer'],
        )

    if obj is not None:
        bmesh.update_edit_mesh(obj.data)

    if report is not None:
        report(
            {'INFO'},
            "Open loop bridge with corners resampled connected paths in place: "
            f"groups={resample_result['resampled_segment_count']}, "
            f"ends={len(context['end_sections'])}, corners={len(context['corner_sections'])}.",
        )

    return {'FINISHED'}


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        _trace_focus("Execute cancelled: detect returned no data.")
        return {'CANCELLED'}

    return _execute_attached_open_corner_case(
        bm,
        obj,
        direction,
        report=report,
        data=data,
    )
