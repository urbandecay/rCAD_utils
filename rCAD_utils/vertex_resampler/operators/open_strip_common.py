# open_strip_common.py — Shared detection for open-strip shaft selections.

from .bridge_utils import get_auto_open_bridged_chain, get_bridged_chain
from ..debug import face_refs, loop_ref, mesh_stats, vert_ref


_VECTOR_TOLERANCE = 1.0e-6
_LAST_DETECT_REASON = None


def _trace_detect(message, **details):
    print(f"[vertex_resampler:open_strip_detect] {message}")
    for key, value in details.items():
        print(f"  {key}: {value}")


def _set_last_detect_reason(reason):
    global _LAST_DETECT_REASON
    _LAST_DETECT_REASON = reason


def get_last_detect_reason():
    return _LAST_DETECT_REASON


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
    # The working bridge interpretation treats the between-loop direction as
    # the editable path and the along-loop direction as the cross section.
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

    # Keep the between-loop axis perpendicular enough to the along-loop axis.
    # across the whole face component instead of letting skew collapse the span.
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

    fast_pair = _fast_detect_open_strip_component(
        component_set,
        component_edges,
        boundary_edges,
        candidate_cut_edges,
        edge_map,
    )
    return fast_pair


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

        start_index = selected_indices[0]
        end_index = selected_indices[-1]
        expected = list(range(start_index, end_index + 1))
        if selected_indices != expected:
            return None

        return loop_index, start_index, end_index

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
    _trace_detect(
        "Trying endpoint-seed bridge expansion.",
        selected_vert_count=len(selected_verts),
        selected_verts=[vert_ref(vert) for vert in sorted(selected_verts, key=lambda item: item.index)],
    )
    start_faces = {
        face
        for vert in selected_verts
        for face in vert.link_faces
        if len(face.verts) == 4
    }
    if not start_faces:
        _trace_detect("Endpoint seed rejected: no adjacent quad faces.")
        return None

    face_component = _connected_quad_faces(start_faces)
    if len(face_component) < 2:
        _trace_detect(
            "Endpoint seed rejected: connected quad component too small.",
            start_faces=face_refs(sorted(start_faces, key=lambda item: item.index)),
            connected_face_count=len(face_component),
        )
        return None

    direct_group = _detect_open_strip_from_face_component(face_component)
    if direct_group is not None:
        trimmed_group = _trim_group_to_terminal_subset(direct_group, selected_verts)
        if trimmed_group is None:
            loops, _is_closed = direct_group
            _trace_detect(
                "Endpoint seed direct-face detect rejected: selection is not a contiguous subset of either terminal loop.",
                selected_verts=[vert_ref(vert) for vert in sorted(selected_verts, key=lambda item: item.index)],
                first_loop=loop_ref(loops[0]) if loops else [],
                last_loop=loop_ref(loops[-1]) if loops else [],
            )
        else:
            _trace_detect(
                "Endpoint seed matched direct face-component strip subset.",
                connected_face_count=len(face_component),
                loop_count=len(trimmed_group[0]),
                first_loop=loop_ref(trimmed_group[0][0]) if trimmed_group[0] else [],
                last_loop=loop_ref(trimmed_group[0][-1]) if trimmed_group[0] else [],
            )
            return {
                'groups': [trimmed_group],
                'components': [],
                'has_extra_selected_faces': False,
                'has_outside_side_faces': False,
            }

    candidates = _strip_candidates(face_component)
    match, _reason = _choose_strip_candidate(candidates)
    if match is None:
        _trace_detect(
            "Endpoint seed rejected: no strip candidate from connected quads.",
            start_faces=face_refs(sorted(start_faces, key=lambda item: item.index)),
            connected_faces=face_refs(sorted(face_component, key=lambda item: item.index)),
        )
        return None

    if not _selected_verts_match_terminal_loop(match['group'], selected_verts):
        loops, _is_closed = match['group']
        _trace_detect(
            "Endpoint seed rejected: selection is not a subset of either terminal loop.",
            selected_verts=[vert_ref(vert) for vert in sorted(selected_verts, key=lambda item: item.index)],
            first_loop=loop_ref(loops[0]) if loops else [],
            last_loop=loop_ref(loops[-1]) if loops else [],
        )
        return None

    _trace_detect(
        "Endpoint seed matched strip candidate.",
        connected_face_count=len(face_component),
        shaft_face_count=len(match['shaft_faces']),
        loop_count=len(match['group'][0]),
        first_loop=loop_ref(match['group'][0][0]) if match['group'][0] else [],
        last_loop=loop_ref(match['group'][0][-1]) if match['group'][0] else [],
    )
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
    _set_last_detect_reason(None)
    selected_verts = {vert for vert in bm.verts if vert.select}
    _trace_detect(
        "Starting open strip detection.",
        mesh=mesh_stats(bm),
    )
    if not selected_verts:
        _set_last_detect_reason("no selected verts")
        _trace_detect("Open strip detection rejected: no selected verts.")
        return None

    endpoint_seed_data = _detect_open_strip_from_endpoint_seed(selected_verts)
    if endpoint_seed_data is not None:
        _set_last_detect_reason("matched endpoint-seed expansion")
        return endpoint_seed_data

    selected_faces = _selected_face_set(bm, selected_verts)
    if selected_faces:
        _trace_detect(
            "Trying selected-face strip detection.",
            selected_face_count=len(selected_faces),
            selected_faces=face_refs(sorted(selected_faces, key=lambda item: item.index)),
        )
        groups = []
        components = []
        has_extra_selected_faces = False
        has_outside_side_faces = False
        covered_verts = set()

        for face_component in _face_components(selected_faces):
            candidates = _strip_candidates(face_component)
            match, reason = _choose_strip_candidate(candidates)
            if match is None:
                _set_last_detect_reason("selected-face strip detection found no candidates")
                _trace_detect(
                    "Selected-face strip detection rejected a component.",
                    component_faces=face_refs(sorted(face_component, key=lambda item: item.index)),
                    candidate_count=len(candidates),
                )
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
            _trace_detect(
                "Selected-face component matched strip candidate.",
                reason=reason,
                component_face_count=len(face_component),
                shaft_face_count=len(match['shaft_faces']),
                extra_face_count=len(match['extra_faces']),
                outside_side_face_count=len(match['outside_side_faces']),
            )

        if covered_verts != selected_verts:
            _set_last_detect_reason("selected-face strip coverage did not match selected verts")
            _trace_detect(
                "Selected-face strip detection rejected: covered verts mismatch.",
                covered_vert_count=len(covered_verts),
                selected_vert_count=len(selected_verts),
            )
            return None

        _set_last_detect_reason("matched selected-face strip detection")
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
        _set_last_detect_reason("matched explicit bridged chain")
        _trace_detect("Matched explicit bridged chain from selected islands.", loop_count=len(bridged_data[0]))
        return {
            'groups': [bridged_data],
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    auto_bridged_data = get_auto_open_bridged_chain(bm)
    if auto_bridged_data and len(auto_bridged_data[0]) >= 2:
        _set_last_detect_reason("matched auto open bridged chain")
        _trace_detect("Matched auto open bridged chain.", loop_count=len(auto_bridged_data[0]))
        return {
            'groups': [auto_bridged_data],
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    strip_groups = _detect_open_strips_from_selected_verts(bm)
    if strip_groups and _matches_selected_verts(strip_groups, selected_verts):
        _set_last_detect_reason("matched selected-vert open strips")
        _trace_detect("Matched selected-vert open strips.", group_count=len(strip_groups))
        return {
            'groups': strip_groups,
            'components': [],
            'has_extra_selected_faces': False,
            'has_outside_side_faces': False,
        }

    _set_last_detect_reason("no open-strip detector matched the selection")
    _trace_detect("Open strip detection failed: no detector matched.")
    return None
