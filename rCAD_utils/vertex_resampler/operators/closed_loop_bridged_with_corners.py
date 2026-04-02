# closed_loop_bridged_with_corners.py — Resample bridged closed loops with corners.

import bmesh

from . import closed_loop_bridged
from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
from ..debug import face_ref, face_refs, loop_ref
from ..seam_manager import load_seam_homes, save_seam_homes

_LAST_DETECT_REASON = None
_POSITION_TOLERANCE = 1.0e-9


def _set_last_detect_reason(reason):
    global _LAST_DETECT_REASON
    _LAST_DETECT_REASON = reason


def _position_key(vec, tolerance):
    return (
        round(vec.x / tolerance),
        round(vec.y / tolerance),
        round(vec.z / tolerance),
    )


def _group_loops(group_data):
    if isinstance(group_data, dict):
        return group_data['rings'][0]
    return group_data[0]


def _group_is_closed(group_data):
    if isinstance(group_data, dict):
        return group_data['rings'][1]
    return group_data[1]


def _count_corner_cross_sections(groups):
    seen_loops = set()
    corner_count = 0

    for group_data in groups:
        for loop in _group_loops(group_data):
            loop_key = tuple(sorted(vert.index for vert in loop))
            if loop_key in seen_loops:
                continue
            seen_loops.add(loop_key)

            if loop and all(len(vert.link_edges) == 4 for vert in loop):
                corner_count += 1

    return corner_count


def _result(groups, invalid_components=0):
    if not groups:
        return None
    if not all(_group_is_closed(group_data) for group_data in groups):
        return None

    corner_count = _count_corner_cross_sections(groups)

    return {
        'groups': groups,
        'invalid_components': invalid_components,
        'mode_label': 'Closed loop bridged with corners',
        'corner_count': corner_count,
    }


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


def _candidate_strip_faces(start_face, start_bit, selected_faces):
    oriented_faces = {start_face: start_bit}
    stack = [start_face]

    while stack:
        current = stack.pop()
        ordered_edges = _ordered_face_edges(current)
        if ordered_edges is None:
            return None, f"{face_ref(current)} is not a quad loop face"

        bit = oriented_faces[current]
        chosen_edges = (
            ordered_edges[bit],
            ordered_edges[(bit + 2) % 4],
        )

        for edge in chosen_edges:
            neighbor = _quad_face_neighbor(current, edge, selected_faces)
            if neighbor is None:
                return None, (
                    f"Missing quad neighbor from {face_ref(current)} "
                    f"across edge {edge.index} for bit {bit}"
                )

            neighbor_bit = _edge_orientation_bit(neighbor, edge)
            if neighbor_bit is None:
                return None, (
                    f"Could not orient neighbor {face_ref(neighbor)} "
                    f"from edge {edge.index}"
                )

            existing_bit = oriented_faces.get(neighbor)
            if existing_bit is None:
                oriented_faces[neighbor] = neighbor_bit
                stack.append(neighbor)
                continue
            if existing_bit != neighbor_bit:
                return None, (
                    f"Orientation conflict for {face_ref(neighbor)}: "
                    f"{existing_bit} vs {neighbor_bit}"
                )

    subset = set(oriented_faces)
    for face, bit in oriented_faces.items():
        ordered_edges = _ordered_face_edges(face)
        if ordered_edges is None:
            return None, f"{face_ref(face)} lost quad ordering"

        neighbors = []
        for edge in (ordered_edges[bit], ordered_edges[(bit + 2) % 4]):
            neighbor = _quad_face_neighbor(face, edge, subset)
            if neighbor is None:
                return None, (
                    f"Subset missing neighbor from {face_ref(face)} "
                    f"across edge {edge.index}"
                )
            neighbors.append(neighbor)

        if len(set(neighbors)) != 2:
            return None, (
                f"{face_ref(face)} does not have two distinct strip neighbors "
                f"for bit {bit}"
            )

    return subset, None


def _all_face_ring_candidates(face_component):
    quad_faces = {
        face for face in face_component
        if len(face.verts) == 4
    }
    diagnostics = {
        'component_faces': face_refs(face_component),
        'quad_face_count': len(quad_faces),
        'attempts': [],
    }
    if len(quad_faces) < 3:
        diagnostics['reason'] = "fewer than 3 quad faces"
        return [], diagnostics

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
        attempt = {
            'start_face': face_ref(start_node[0]),
            'bit': start_node[1],
            'reason': 'candidate built',
        }
        if start_node in visited:
            attempt['reason'] = "duplicate strip candidate"
            diagnostics['attempts'].append(attempt)
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
        if len(strip_faces) < 3:
            attempt['reason'] = "fewer than 3 strip faces"
            diagnostics['attempts'].append(attempt)
            continue
        if len(strip_faces) != len(component_nodes):
            attempt['reason'] = "orientation component contains duplicate faces"
            diagnostics['attempts'].append(attempt)
            continue

        valid = True
        for node in component_nodes:
            distinct_neighbors = len(adjacency.get(node, set()) & component_nodes)
            if distinct_neighbors != 2:
                valid = False
                break
        if not valid:
            attempt['reason'] = "component is not a closed strip"
            diagnostics['attempts'].append(attempt)
            continue

        rings_data = closed_loop_bridged._rings_from_component(strip_faces)
        if rings_data is None:
            attempt['reason'] = "rings_from_component rejected candidate"
            attempt['strip_faces'] = face_refs(strip_faces)
            diagnostics['attempts'].append(attempt)
            continue

        attempt['reason'] = "accepted candidate"
        attempt['strip_faces'] = face_refs(strip_faces)
        attempt['loops'] = [loop_ref(loop) for loop in rings_data[0]]
        diagnostics['attempts'].append(attempt)
        candidates.append({
            'strip_faces': strip_faces,
            'rings': rings_data,
            'extra_faces': set(face_component) - strip_faces,
            'loop_keys': [_loop_key(loop) for loop in rings_data[0]],
        })

    if not candidates:
        diagnostics['reason'] = "no valid strip candidates"
        return [], diagnostics

    candidates = sorted(
        candidates,
        key=lambda item: (
            len(item['strip_faces']),
            -len(item['extra_faces']),
            len(item['rings'][0][0]),
        ),
        reverse=True,
    )

    diagnostics['reason'] = "selected all strip candidates"
    diagnostics['chosen_strip_faces'] = [
        face_refs(candidate['strip_faces'])
        for candidate in candidates
    ]
    diagnostics['chosen_extra_faces'] = [
        face_refs(candidate['extra_faces'])
        for candidate in candidates
    ]
    diagnostics['chosen_loops'] = [
        [loop_ref(loop) for loop in candidate['rings'][0]]
        for candidate in candidates
    ]
    return candidates, diagnostics


def _selected_faces_and_candidates(bm):
    _set_last_detect_reason(None)
    sel_verts = [vert for vert in bm.verts if vert.select]
    if len(sel_verts) < 4:
        _set_last_detect_reason("need at least 4 selected verts")
        return None, []

    selected_faces = closed_loop_bridged._selected_face_set(bm, set(sel_verts))
    if not selected_faces:
        _set_last_detect_reason("no selected face set could be built from the selection")
        return None, []

    candidates = []
    component_failures = []
    for component in closed_loop_bridged._face_components(selected_faces):
        component_candidates, diagnostics = _all_face_ring_candidates(component)
        if not component_candidates:
            failure = diagnostics.get('reason') or "unknown component failure"
            attempts = diagnostics.get('attempts') or []
            if attempts:
                attempt_reason = attempts[0].get('reason')
                if attempt_reason:
                    failure = f"{failure}; first attempt: {attempt_reason}"
            component_failures.append(
                f"component {','.join(str(face.index) for face in sorted(component, key=lambda item: item.index))}: {failure}"
            )
            continue

        candidates.extend(component_candidates)

    if not candidates and not component_failures:
        _set_last_detect_reason("no face components found")
        return selected_faces, []
    if not candidates and component_failures:
        _set_last_detect_reason(component_failures[0])

    return selected_faces, candidates


def _detect_face_corner_groups(bm):
    _selected_faces, candidates = _selected_faces_and_candidates(bm)
    groups = []
    for candidate in candidates:
        groups.append({
            'rings': candidate['rings'],
            'strip_faces': list(candidate['strip_faces']),
            'use_seams': False,
            'migrate_seams': False,
        })

    if not groups:
        return None

    return _result(groups, invalid_components=0)


def _loop_key(loop):
    return tuple(sorted(vert.index for vert in loop if getattr(vert, "is_valid", False)))


def _loop_positions(loop):
    return [vert.co.copy() for vert in loop if getattr(vert, "is_valid", False)]


def _ring_avg_edge_length(positions):
    if len(positions) < 2:
        return 0.0
    total = 0.0
    count = len(positions)
    for index in range(count):
        total += (positions[index] - positions[(index + 1) % count]).length
    return total / count


def _shared_ring_loops(candidates):
    return [entry['loop'] for entry in _shared_loop_entries(candidates)]


def _selection_has_open_path_endpoints(selected_faces):
    if not selected_faces:
        return False

    selected_edges = {
        edge
        for face in selected_faces
        for edge in face.edges
        if getattr(edge, "is_valid", False)
    }
    selected_verts = {
        vert
        for face in selected_faces
        for vert in face.verts
        if getattr(vert, "is_valid", False)
    }
    return any(
        sum(1 for edge in vert.link_edges if edge in selected_edges) == 3
        for vert in selected_verts
    )


def _path_detection_summary(selected_faces):
    selected_edges = {
        edge
        for face in selected_faces or []
        for edge in face.edges
        if getattr(edge, "is_valid", False)
    }
    selected_verts = sorted(
        {
            vert
            for face in selected_faces or []
            for vert in face.verts
            if getattr(vert, "is_valid", False)
        },
        key=lambda vert: vert.index,
    )
    degree_counts = {}
    degree3_verts = []
    for vert in selected_verts:
        degree = sum(1 for edge in vert.link_edges if edge in selected_edges)
        degree_counts[degree] = degree_counts.get(degree, 0) + 1
        if degree == 3:
            degree3_verts.append(vert.index)

    return {
        'selected_vert_count': len(selected_verts),
        'degree_counts': degree_counts,
        'degree3_verts': degree3_verts,
        'is_open_path': bool(degree3_verts),
    }


def _shared_loop_entries(candidates):
    loop_map = {}
    for candidate_index, candidate in enumerate(candidates):
        loops = candidate['rings'][0]
        loop_keys = candidate.get('loop_keys')
        if loop_keys is None:
            loop_keys = [_loop_key(loop) for loop in loops]
        for loop, key in zip(loops, loop_keys):
            if len(key) != len(loop):
                continue
            entry = loop_map.setdefault(
                key,
                {
                    'key': key,
                    'loop': loop,
                    'candidate_indices': [],
                },
            )
            entry['candidate_indices'].append(candidate_index)

    shared_entries = []
    for entry in loop_map.values():
        if len(entry['candidate_indices']) >= 2:
            entry = dict(entry)
            entry['candidate_indices'] = sorted(set(entry['candidate_indices']))
            shared_entries.append(entry)
    return shared_entries


def _classify_path_type(selected_faces, candidates):
    shared_entries = _shared_loop_entries(candidates)
    if candidates and shared_entries:
        candidate_to_corners = {index: 0 for index in range(len(candidates))}
        shared_corner_candidate_counts = []
        for entry in shared_entries:
            count = len(entry['candidate_indices'])
            shared_corner_candidate_counts.append(count)
            for candidate_index in entry['candidate_indices']:
                candidate_to_corners[candidate_index] = candidate_to_corners.get(candidate_index, 0) + 1

        candidate_corner_counts = [
            candidate_to_corners.get(index, 0)
            for index in range(len(candidates))
        ]

        if (
            len(shared_entries) == len(candidates)
            and all(count == 2 for count in candidate_corner_counts)
            and all(count == 2 for count in shared_corner_candidate_counts)
        ):
            return {
                'path_type': 'closed',
                'reason': 'candidate cycle',
            }

        endpoint_count = sum(1 for count in candidate_corner_counts if count == 1)
        if (
            endpoint_count == 2
            and all(count in {1, 2} for count in candidate_corner_counts)
            and all(count == 2 for count in shared_corner_candidate_counts)
        ):
            return {
                'path_type': 'open',
                'reason': 'candidate chain',
            }

    fallback = _path_detection_summary(selected_faces)
    return {
        'path_type': 'open' if fallback['is_open_path'] else 'closed',
        'reason': 'selected-edge degree fallback',
    }


def _loop_centroid(loop):
    positions = _loop_positions(loop)
    if not positions:
        return None
    total = positions[0].copy()
    for position in positions[1:]:
        total += position
    return total / len(positions)


def _order_closed_path_shared_loops(candidates):
    ordered_loops, _diagnostics = _order_closed_path_shared_loops_with_diagnostics(candidates)
    return ordered_loops


def _order_closed_path_shared_loops_with_diagnostics(candidates):
    shared_entries = _shared_loop_entries(candidates)
    diagnostics = {
        'candidate_count': len(candidates or []),
        'shared_corner_count': len(shared_entries),
        'candidate_corner_counts': [],
        'shared_corner_candidate_counts': [],
        'start_corner_key': None,
        'ordered_corner_keys': [],
        'fallback_reason': None,
    }
    if not shared_entries:
        diagnostics['fallback_reason'] = "no shared corner entries"
        return [], diagnostics

    candidate_to_corners = {index: [] for index in range(len(candidates))}
    entry_by_key = {}
    for entry in shared_entries:
        key = entry['key']
        entry_by_key[key] = entry
        for candidate_index in entry['candidate_indices']:
            candidate_to_corners.setdefault(candidate_index, []).append(key)

    diagnostics['candidate_corner_counts'] = [
        len(candidate_to_corners[index])
        for index in range(len(candidates))
    ]
    diagnostics['shared_corner_candidate_counts'] = [
        len(entry['candidate_indices'])
        for entry in shared_entries
    ]

    if any(len(candidate_to_corners[index]) != 2 for index in range(len(candidates))):
        diagnostics['fallback_reason'] = "some strip candidates do not touch exactly 2 shared corners"
        return [entry['loop'] for entry in shared_entries], diagnostics
    if any(len(entry['candidate_indices']) != 2 for entry in shared_entries):
        diagnostics['fallback_reason'] = "some shared corners do not touch exactly 2 strip candidates"
        return [entry['loop'] for entry in shared_entries], diagnostics

    def _start_corner_key(item):
        centroid = _loop_centroid(item['loop'])
        if centroid is None:
            return (float("inf"), float("inf"), float("inf"), item['key'])
        return (centroid.x, centroid.y, centroid.z, item['key'])

    start_entry = min(shared_entries, key=_start_corner_key)
    start_key = start_entry['key']
    diagnostics['start_corner_key'] = start_key
    ordered_keys = []
    current_key = start_key
    previous_candidate = None

    while True:
        ordered_keys.append(current_key)
        current_entry = entry_by_key[current_key]
        candidate_indices = current_entry['candidate_indices']
        if previous_candidate is None:
            next_candidate = min(candidate_indices)
        else:
            next_candidate = next(
                (
                    candidate_index for candidate_index in candidate_indices
                    if candidate_index != previous_candidate
                ),
                None,
            )
        if next_candidate is None:
            diagnostics['fallback_reason'] = "cycle walk could not pick next strip candidate"
            return [entry['loop'] for entry in shared_entries], diagnostics

        next_corner_keys = [
            key for key in candidate_to_corners[next_candidate]
            if key != current_key
        ]
        if len(next_corner_keys) != 1:
            diagnostics['fallback_reason'] = "cycle walk found ambiguous next shared corner"
            return [entry['loop'] for entry in shared_entries], diagnostics

        previous_candidate = next_candidate
        current_key = next_corner_keys[0]
        if current_key == start_key:
            break
        if current_key in ordered_keys or len(ordered_keys) > len(shared_entries):
            diagnostics['fallback_reason'] = "cycle walk repeated before closing cleanly"
            return [entry['loop'] for entry in shared_entries], diagnostics

    if len(ordered_keys) != len(shared_entries):
        diagnostics['fallback_reason'] = "cycle walk did not visit all shared corners"
        return [entry['loop'] for entry in shared_entries], diagnostics

    diagnostics['ordered_corner_keys'] = ordered_keys
    return [entry_by_key[key]['loop'] for key in ordered_keys], diagnostics


def _group_ring_sizes(groups):
    sizes = []
    for group_data in groups or []:
        loops = _group_loops(group_data)
        if not loops:
            continue
        first_loop = loops[0]
        valid_count = sum(
            1 for vert in first_loop
            if getattr(vert, "is_valid", False)
        )
        if valid_count:
            sizes.append(valid_count)
    return sizes


def _post_weld_selection_summary(bm):
    selected_faces, candidates = _selected_faces_and_candidates(bm)
    if not selected_faces or not candidates:
        return {
            'shared_ring_sizes': [],
            'candidate_ring_sizes': [],
        }

    shared_loops = _shared_ring_loops(candidates)
    shared_ring_sizes = [
        len(_loop_key(loop))
        for loop in shared_loops
        if loop
    ]
    candidate_ring_sizes = [
        len(candidate['rings'][0][0])
        for candidate in candidates
        if candidate.get('rings') and candidate['rings'][0]
    ]
    return {
        'shared_ring_sizes': shared_ring_sizes,
        'candidate_ring_sizes': candidate_ring_sizes,
    }


def _corner_group_matches(groups, split_info):
    matches = []
    for group_index, group_data in enumerate(groups or [], start=1):
        loops = _group_loops(group_data)
        best_match_count = 0
        best_loop_index = None
        best_ring_size = 0
        for loop_index, loop in enumerate(loops, start=1):
            match_count = _cluster_loop_match_count(loop, split_info)
            if match_count <= 0:
                continue
            ring_size = sum(
                1 for vert in loop
                if getattr(vert, "is_valid", False)
            )
            candidate = (match_count, ring_size, -loop_index)
            if (
                best_loop_index is None
                or candidate > (best_match_count, best_ring_size, -best_loop_index)
            ):
                best_match_count = match_count
                best_loop_index = loop_index
                best_ring_size = ring_size
        if best_loop_index is None:
            continue
        matches.append({
            'group_index': group_index,
            'loop_index': best_loop_index,
            'ring_size': best_ring_size,
            'matched_positions': best_match_count,
        })
    return matches


def _corner_pre_resample_summaries(groups, split_infos, direction):
    summaries = []
    for corner_index, split_info in enumerate(split_infos, start=1):
        recorded_positions = len(split_info['clusters'])
        matches = _corner_group_matches(groups, split_info)
        predicted_sizes = []
        untracked_new_positions = []
        for match in matches:
            predicted_size = match['ring_size'] + direction
            if predicted_size < 0:
                predicted_size = 0
            predicted_sizes.append(predicted_size)
            if predicted_size > recorded_positions:
                untracked_new_positions.append(predicted_size - recorded_positions)
            else:
                untracked_new_positions.append(0)
        summaries.append({
            'corner_index': corner_index,
            'recorded_positions': recorded_positions,
            'matches': matches,
            'predicted_sizes': predicted_sizes,
            'untracked_new_positions': untracked_new_positions,
        })
    return summaries


def _corner_post_weld_summaries(split_infos):
    summaries = []
    for corner_index, split_info in enumerate(split_infos, start=1):
        live_cluster_counts = []
        remaining_duplicate_positions = 0
        for cluster in split_info['clusters']:
            live_count = sum(
                1 for vert in cluster['verts']
                if getattr(vert, "is_valid", False)
            )
            live_cluster_counts.append(live_count)
            if live_count > 1:
                remaining_duplicate_positions += 1
        summaries.append({
            'corner_index': corner_index,
            'recorded_positions': len(split_info['clusters']),
            'remaining_duplicate_positions': remaining_duplicate_positions,
            'live_cluster_counts': live_cluster_counts,
        })
    return summaries


def _choose_anchor_homes(shared_loops, obj):
    stored_homes = load_seam_homes(obj)
    used_home_indices = set()
    anchor_homes = []

    for loop in shared_loops:
        positions = _loop_positions(loop)
        if not positions:
            continue

        avg_edge_len = _ring_avg_edge_length(positions)
        match_limit = max(avg_edge_len * 0.75, 1.0e-6)

        best = None
        for position in positions:
            for home_index, home in enumerate(stored_homes):
                if home_index in used_home_indices:
                    continue
                dist = (position - home).length
                candidate = (dist, home_index, position)
                if best is None or candidate < best:
                    best = candidate

        if best is not None and best[0] <= match_limit:
            _dist, home_index, position = best
            used_home_indices.add(home_index)
            anchor_homes.append(position.copy())
            continue

        anchor = positions[0].copy()
        anchor_homes.append(anchor)
        stored_homes.append(anchor.copy())
        used_home_indices.add(len(stored_homes) - 1)

    save_seam_homes(obj, stored_homes)
    return anchor_homes


def _candidate_loop_keys(candidate):
    loop_keys = candidate.get('loop_keys')
    if loop_keys is None:
        loop_keys = [_loop_key(loop) for loop in candidate['rings'][0]]
    return loop_keys


def _loop_edges(loop):
    edges = []
    count = len(loop)
    for index in range(count):
        v1 = loop[index]
        v2 = loop[(index + 1) % count]
        edge = next(
            (
                item for item in v1.link_edges
                if item.is_valid and item.other_vert(v1) is v2
            ),
            None,
        )
        if edge is None:
            return None
        edges.append(edge)
    return edges


def _restore_face_selection(selected_faces):
    for face in selected_faces:
        if face is None or not getattr(face, "is_valid", False):
            continue
        face.select = True
        for edge in face.edges:
            if edge.is_valid:
                edge.select = True
        for vert in face.verts:
            if vert.is_valid:
                vert.select = True


def _split_shared_corner_rings(bm, selected_faces, shared_loops, anchor_homes):
    split_edges = set()
    split_infos = []
    for loop in shared_loops:
        loop_edges = _loop_edges(loop)
        if loop_edges is None:
            continue
        split_edges.update(loop_edges)

    if not split_edges:
        return False

    bmesh.ops.split_edges(bm, edges=list(split_edges))
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    _restore_face_selection(selected_faces)
    bm.select_flush_mode()
    bm.normal_update()
    vert_lookup = {}
    for vert in bm.verts:
        if not getattr(vert, "is_valid", False):
            continue
        vert_lookup.setdefault(_position_key(vert.co, _POSITION_TOLERANCE), []).append(vert)
    for loop, anchor_home in zip(shared_loops, anchor_homes):
        clusters = []
        for position in _loop_positions(loop):
            cluster = [
                vert for vert in vert_lookup.get(_position_key(position, _POSITION_TOLERANCE), [])
                if vert.is_valid and (vert.co - position).length <= _POSITION_TOLERANCE
            ]
            if len(cluster) >= 2:
                clusters.append({
                    'position': position.copy(),
                    'verts': cluster,
                })
        if clusters:
            split_infos.append({
                'anchor_home': anchor_home.copy(),
                'clusters': clusters,
            })
    return split_infos


def _cluster_loop_match_count(loop, split_info):
    loop_set = {vert for vert in loop if getattr(vert, "is_valid", False)}
    return sum(
        1
        for cluster in split_info['clusters']
        if any(vert in loop_set for vert in cluster['verts'] if getattr(vert, "is_valid", False))
    )


def _rotate_loop(loop, start_index):
    if not loop:
        return loop
    start_index %= len(loop)
    return loop[start_index:] + loop[:start_index]


def _reverse_loop_with_fixed_start(loop):
    if len(loop) <= 1:
        return list(loop)
    return [loop[0]] + list(reversed(loop[1:]))


def _anchor_index_for_loop(loop, anchor_home):
    best_anchor_index = None
    best_anchor_dist = None
    for index, vert in enumerate(loop):
        if not getattr(vert, "is_valid", False):
            continue
        dist = (vert.co - anchor_home).length
        candidate = (dist, index)
        if best_anchor_dist is None or candidate < best_anchor_dist:
            best_anchor_dist = candidate
            best_anchor_index = index
    return best_anchor_index


def _pair_distance_score(loop_a, loop_b):
    if len(loop_a) != len(loop_b):
        return None
    return sum(
        (vert_a.co - vert_b.co).length
        for vert_a, vert_b in zip(loop_a, loop_b)
        if getattr(vert_a, "is_valid", False) and getattr(vert_b, "is_valid", False)
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
    updated['migrate_seams'] = False
    updated['forced_seam_verts'] = forced_seam_verts
    return updated


def _anchor_group_to_split_loop(group_data, loop_index, split_info):
    loops, is_closed = group_data['rings'] if isinstance(group_data, dict) else group_data
    if not is_closed or not loops or loop_index < 0 or loop_index >= len(loops):
        return group_data

    anchor_index = _anchor_index_for_loop(loops[loop_index], split_info['anchor_home'])
    if anchor_index is None:
        return group_data

    rotated_loops = [
        _rotate_loop(list(loop_item), anchor_index)
        for loop_item in loops
    ]
    return _updated_group_with_loops(group_data, rotated_loops, is_closed)


def _phase_align_group_to_reference(group_data, loop_index, reference_loop):
    loops, is_closed = group_data['rings'] if isinstance(group_data, dict) else group_data
    if (
        not is_closed
        or not loops
        or loop_index < 0
        or loop_index >= len(loops)
        or not reference_loop
        or len(loops[loop_index]) != len(reference_loop)
    ):
        return group_data

    best = None
    for start_index in range(len(loops[loop_index])):
        rotated_target = _rotate_loop(list(loops[loop_index]), start_index)
        forward_score = _pair_distance_score(rotated_target, reference_loop)
        reversed_target = _reverse_loop_with_fixed_start(rotated_target)
        reverse_score = _pair_distance_score(reversed_target, reference_loop)

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
        return group_data

    _score, start_index, reverse_group = best
    rotated_loops = [
        _rotate_loop(list(loop_item), start_index)
        for loop_item in loops
    ]
    if reverse_group:
        rotated_loops = [
            _reverse_loop_with_fixed_start(loop_item)
            for loop_item in rotated_loops
        ]
    return _updated_group_with_loops(group_data, rotated_loops, is_closed)


def _group_centroid(group_data):
    centroids = [
        centroid
        for centroid in (_loop_centroid(loop) for loop in _group_loops(group_data))
        if centroid is not None
    ]
    if not centroids:
        return None
    total = centroids[0].copy()
    for centroid in centroids[1:]:
        total += centroid
    return total / len(centroids)


def _group_sort_key(group_data):
    centroid = _group_centroid(group_data)
    if centroid is None:
        return (float("inf"), float("inf"), float("inf"))
    return (centroid.x, centroid.y, centroid.z)


def _ordered_open_path_candidates(candidates):
    shared_entries = _shared_loop_entries(candidates)
    if not shared_entries:
        return None, None

    entry_by_key = {
        entry['key']: entry
        for entry in shared_entries
    }
    candidate_to_keys = {index: [] for index in range(len(candidates))}
    for entry in shared_entries:
        for candidate_index in entry['candidate_indices']:
            candidate_to_keys.setdefault(candidate_index, []).append(entry['key'])

    endpoints = [
        candidate_index
        for candidate_index, keys in candidate_to_keys.items()
        if len(keys) == 1
    ]
    if len(endpoints) != 2:
        return None, None

    def _candidate_key(candidate_index):
        return _group_sort_key({
            'rings': candidates[candidate_index]['rings'],
        })

    current_candidate = min(endpoints, key=_candidate_key)
    ordered_candidates = [current_candidate]
    ordered_entries = []
    previous_key = None

    while True:
        next_key = next(
            (
                key for key in candidate_to_keys.get(current_candidate, [])
                if key != previous_key
            ),
            None,
        )
        if next_key is None:
            break

        entry = entry_by_key.get(next_key)
        if entry is None:
            return None, None

        ordered_entries.append(entry)
        next_candidate = next(
            (
                candidate_index
                for candidate_index in entry['candidate_indices']
                if candidate_index != current_candidate
            ),
            None,
        )
        if next_candidate is None:
            return None, None

        ordered_candidates.append(next_candidate)
        previous_key = next_key
        current_candidate = next_candidate

        if len(ordered_candidates) > len(candidates):
            return None, None

    if len(ordered_candidates) != len(candidates):
        return None, None
    if len(ordered_entries) != max(len(candidates) - 1, 0):
        return None, None

    return ordered_candidates, ordered_entries


def _ordered_open_path_candidate_components(candidates):
    shared_entries = _shared_loop_entries(candidates)
    if not shared_entries:
        return []

    entry_by_key = {
        entry['key']: entry
        for entry in shared_entries
    }
    candidate_to_keys = {index: [] for index in range(len(candidates))}
    for entry in shared_entries:
        for candidate_index in entry['candidate_indices']:
            candidate_to_keys.setdefault(candidate_index, []).append(entry['key'])

    adjacency = {index: set() for index in range(len(candidates))}
    for entry in shared_entries:
        if len(entry['candidate_indices']) != 2:
            continue
        a, b = entry['candidate_indices']
        adjacency[a].add(b)
        adjacency[b].add(a)

    def _candidate_key(candidate_index):
        return _group_sort_key({
            'rings': candidates[candidate_index]['rings'],
        })

    components = []
    visited = set()
    for start_candidate in sorted(adjacency.keys(), key=_candidate_key):
        if start_candidate in visited or not adjacency[start_candidate]:
            continue

        component_candidates = set()
        stack = [start_candidate]
        visited.add(start_candidate)
        while stack:
            current = stack.pop()
            component_candidates.add(current)
            for neighbor in sorted(adjacency[current], key=_candidate_key):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

        endpoints = [
            candidate_index
            for candidate_index in component_candidates
            if sum(1 for key in candidate_to_keys.get(candidate_index, []) if entry_by_key[key]['candidate_indices']) == 1
        ]
        if len(endpoints) != 2:
            continue

        current_candidate = min(endpoints, key=_candidate_key)
        ordered_candidates = [current_candidate]
        ordered_entries = []
        previous_key = None

        while True:
            next_key = next(
                (
                    key for key in candidate_to_keys.get(current_candidate, [])
                    if key in entry_by_key
                    and current_candidate in entry_by_key[key]['candidate_indices']
                    and key != previous_key
                    and any(
                        candidate_index in component_candidates
                        for candidate_index in entry_by_key[key]['candidate_indices']
                    )
                ),
                None,
            )
            if next_key is None:
                break

            entry = entry_by_key[next_key]
            next_candidate = next(
                (
                    candidate_index
                    for candidate_index in entry['candidate_indices']
                    if candidate_index != current_candidate
                    and candidate_index in component_candidates
                ),
                None,
            )
            if next_candidate is None:
                break

            ordered_entries.append(entry)
            ordered_candidates.append(next_candidate)
            previous_key = next_key
            current_candidate = next_candidate

            if len(ordered_candidates) > len(component_candidates):
                break

        if (
            len(ordered_candidates) == len(component_candidates)
            and len(ordered_entries) == max(len(component_candidates) - 1, 0)
        ):
            components.append({
                'ordered_candidates': ordered_candidates,
                'ordered_entries': ordered_entries,
            })

    return components


def _open_path_group_edges(groups, split_infos):
    edges = []
    for split_index, split_info in enumerate(split_infos):
        matches = _corner_group_matches(groups, split_info)
        if len(matches) < 2:
            continue
        matches = sorted(
            matches,
            key=lambda item: (
                item['matched_positions'],
                item['ring_size'],
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
            continue
        edges.append({
            'split_index': split_index,
            'group_a': first['group_index'] - 1,
            'loop_a': first['loop_index'] - 1,
            'group_b': second['group_index'] - 1,
            'loop_b': second['loop_index'] - 1,
        })
    return edges


def _ordered_open_path_groups(groups, split_infos):
    edges = _open_path_group_edges(groups, split_infos)
    if not edges:
        return None, None

    adjacency = {index: [] for index in range(len(groups))}
    for edge in edges:
        adjacency[edge['group_a']].append((edge['group_b'], edge))
        adjacency[edge['group_b']].append((edge['group_a'], edge))

    endpoints = [
        group_index for group_index, neighbors in adjacency.items()
        if len(neighbors) == 1
    ]
    if len(endpoints) != 2:
        return None, None

    start_group = min(endpoints, key=lambda index: _group_sort_key(groups[index]))
    ordered_groups = [start_group]
    ordered_edges = []
    previous_group = None
    current_group = start_group

    while True:
        next_step = next(
            (
                (neighbor_group, edge)
                for neighbor_group, edge in adjacency.get(current_group, [])
                if neighbor_group != previous_group
            ),
            None,
        )
        if next_step is None:
            break
        neighbor_group, edge = next_step
        ordered_edges.append(edge)
        ordered_groups.append(neighbor_group)
        previous_group = current_group
        current_group = neighbor_group

    if len(ordered_groups) != len(groups):
        return None, None
    return ordered_groups, ordered_edges


def _ordered_open_path_group_components(groups, split_infos):
    edges = _open_path_group_edges(groups, split_infos)
    if not edges:
        return []

    adjacency = {index: [] for index in range(len(groups))}
    for edge in edges:
        adjacency[edge['group_a']].append((edge['group_b'], edge))
        adjacency[edge['group_b']].append((edge['group_a'], edge))

    components = []
    visited = set()
    for start_group in sorted(range(len(groups)), key=lambda index: _group_sort_key(groups[index])):
        if start_group in visited or not adjacency[start_group]:
            continue

        component_groups = set()
        stack = [start_group]
        visited.add(start_group)
        while stack:
            current = stack.pop()
            component_groups.add(current)
            for neighbor_group, _edge in adjacency.get(current, []):
                if neighbor_group in visited:
                    continue
                visited.add(neighbor_group)
                stack.append(neighbor_group)

        endpoints = [
            group_index
            for group_index in component_groups
            if sum(1 for neighbor_group, _edge in adjacency[group_index] if neighbor_group in component_groups) == 1
        ]
        if len(endpoints) != 2:
            continue

        current_group = min(endpoints, key=lambda index: _group_sort_key(groups[index]))
        ordered_groups = [current_group]
        ordered_edges = []
        previous_group = None

        while True:
            next_step = next(
                (
                    (neighbor_group, edge)
                    for neighbor_group, edge in adjacency.get(current_group, [])
                    if neighbor_group in component_groups and neighbor_group != previous_group
                ),
                None,
            )
            if next_step is None:
                break
            neighbor_group, edge = next_step
            ordered_edges.append(edge)
            ordered_groups.append(neighbor_group)
            previous_group = current_group
            current_group = neighbor_group

            if len(ordered_groups) > len(component_groups):
                break

        if (
            len(ordered_groups) == len(component_groups)
            and len(ordered_edges) == max(len(component_groups) - 1, 0)
        ):
            components.append({
                'ordered_groups': ordered_groups,
                'ordered_edges': ordered_edges,
            })

    return components


def _choose_anchor_home_for_loop(loop, obj):
    positions = _loop_positions(loop)
    if not positions:
        return None

    stored_homes = load_seam_homes(obj)
    avg_edge_len = _ring_avg_edge_length(positions)
    match_limit = max(avg_edge_len * 0.75, 1.0e-6)

    best = None
    for position in positions:
        for home_index, home in enumerate(stored_homes):
            dist = (position - home).length
            candidate = (dist, home_index, position)
            if best is None or candidate < best:
                best = candidate

    if best is not None and best[0] <= match_limit:
        return best[2].copy()

    anchor = positions[0].copy()
    stored_homes.append(anchor.copy())
    save_seam_homes(obj, stored_homes)
    return anchor


def _build_open_path_anchor_plan(candidates, obj):
    components = _ordered_open_path_candidate_components(candidates)
    if not components:
        return None

    shared_loops = []
    anchor_homes = []
    corner_directions = []
    master_reference_loop = None

    for component in components:
        ordered_candidates = component['ordered_candidates']
        ordered_entries = component['ordered_entries']
        if not ordered_candidates or not ordered_entries:
            continue

        anchored = {}
        first_candidate_index = ordered_candidates[0]
        first_candidate = candidates[first_candidate_index]
        first_loop_keys = _candidate_loop_keys(first_candidate)
        first_shared_key = ordered_entries[0]['key']
        try:
            first_shared_loop_index = first_loop_keys.index(first_shared_key)
        except ValueError:
            return None

        first_base_loop_index = next(
            (
                loop_index
                for loop_index in range(len(first_loop_keys))
                if loop_index != first_shared_loop_index
            ),
            None,
        )
        if first_base_loop_index is None:
            return None

        if master_reference_loop is None:
            base_anchor_home = _choose_anchor_home_for_loop(
                _group_loops(first_candidate)[first_base_loop_index],
                obj,
            )
            if base_anchor_home is None:
                return None
            anchored[first_candidate_index] = _anchor_group_to_split_loop(
                first_candidate,
                first_base_loop_index,
                {'anchor_home': base_anchor_home},
            )
            master_reference_loop = list(
                _group_loops(anchored[first_candidate_index])[first_base_loop_index]
            )
        else:
            anchored[first_candidate_index] = _phase_align_group_to_reference(
                first_candidate,
                first_base_loop_index,
                master_reference_loop,
            )

        for shared_entry, previous_candidate_index, current_candidate_index in zip(
            ordered_entries,
            ordered_candidates,
            ordered_candidates[1:],
        ):
            previous_group = anchored[previous_candidate_index]
            previous_loop_keys = _candidate_loop_keys(candidates[previous_candidate_index])
            current_loop_keys = _candidate_loop_keys(candidates[current_candidate_index])

            try:
                previous_loop_index = previous_loop_keys.index(shared_entry['key'])
                current_loop_index = current_loop_keys.index(shared_entry['key'])
            except ValueError:
                return None

            reference_loop = list(_group_loops(previous_group)[previous_loop_index])
            if not reference_loop:
                return None

            shared_loops.append(shared_entry['loop'])
            anchor_homes.append(reference_loop[0].co.copy())

            anchored[current_candidate_index] = _phase_align_group_to_reference(
                candidates[current_candidate_index],
                current_loop_index,
                reference_loop,
            )
            current_loop = list(_group_loops(anchored[current_candidate_index])[current_loop_index])
            reversed_loop = _reverse_loop_with_fixed_start(current_loop)
            forward_score = _pair_distance_score(reference_loop, current_loop)
            reverse_score = _pair_distance_score(reference_loop, reversed_loop)
            corner_directions.append(
                'reversed'
                if reverse_score is not None and (
                    forward_score is None or reverse_score < forward_score
                )
                else 'forward'
            )

    if not shared_loops:
        return None

    return {
        'shared_loops': shared_loops,
        'anchor_homes': anchor_homes,
        'corner_directions': corner_directions,
    }


def _anchor_open_path_groups(groups, split_infos):
    components = _ordered_open_path_group_components(groups, split_infos)
    if not components:
        return [
            _anchor_group_rings(group_data, split_infos)
            for group_data in groups
        ]

    anchored = {}
    for component in components:
        ordered_groups = component['ordered_groups']
        ordered_edges = component['ordered_edges']
        if not ordered_groups or not ordered_edges:
            continue

        first_group_index = ordered_groups[0]
        first_edge = ordered_edges[0]
        first_loop_index = first_edge['loop_a'] if first_edge['group_a'] == first_group_index else first_edge['loop_b']
        anchored[first_group_index] = _anchor_group_to_split_loop(
            groups[first_group_index],
            first_loop_index,
            split_infos[first_edge['split_index']],
        )

        for edge, current_group_index, previous_group_index in zip(
            ordered_edges,
            ordered_groups[1:],
            ordered_groups,
        ):
            previous_group = anchored[previous_group_index]
            previous_loop_index = edge['loop_a'] if edge['group_a'] == previous_group_index else edge['loop_b']
            current_loop_index = edge['loop_a'] if edge['group_a'] == current_group_index else edge['loop_b']
            reference_loop = list(_group_loops(previous_group)[previous_loop_index])
            anchored[current_group_index] = _phase_align_group_to_reference(
                groups[current_group_index],
                current_loop_index,
                reference_loop,
            )

    return [
        anchored.get(group_index, groups[group_index])
        for group_index in range(len(groups))
    ]


def _anchor_group_rings(group_data, split_infos):
    loops, is_closed = group_data['rings'] if isinstance(group_data, dict) else group_data
    if not is_closed or not loops:
        return group_data

    best = None
    for split_index, split_info in enumerate(split_infos):
        for loop_index, loop in enumerate(loops):
            match_count = _cluster_loop_match_count(loop, split_info)
            candidate = (match_count, -loop_index, -split_index)
            if best is None or candidate > best[0]:
                best = (candidate, split_index, loop_index)

    if best is None or best[0][0] <= 0:
        return group_data

    split_info = split_infos[best[1]]
    loop_index = best[2]
    loop = loops[loop_index]

    best_anchor_index = None
    best_anchor_dist = None
    for index, vert in enumerate(loop):
        if not getattr(vert, "is_valid", False):
            continue
        dist = (vert.co - split_info['anchor_home']).length
        candidate = (dist, index)
        if best_anchor_dist is None or candidate < best_anchor_dist:
            best_anchor_dist = candidate
            best_anchor_index = index

    if best_anchor_index is None or best_anchor_index == 0:
        rotated_loops = [list(loop_item) for loop_item in loops]
    else:
        rotated_loops = [
            _rotate_loop(loop_item, best_anchor_index)
            for loop_item in loops
        ]

    return _updated_group_with_loops(group_data, rotated_loops, is_closed)


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


def _live_corner_loop_pair(groups, split_info):
    matches = _corner_group_matches(groups, split_info)
    if len(matches) < 2:
        return None

    matches = sorted(
        matches,
        key=lambda item: (
            item['matched_positions'],
            item['ring_size'],
            -item['group_index'],
            -item['loop_index'],
        ),
        reverse=True,
    )[:2]

    pair_loops = []
    for match in matches:
        group_data = groups[match['group_index'] - 1]
        loops = _group_loops(group_data)
        loop = list(loops[match['loop_index'] - 1])
        if not loop:
            return None
        pair_loops.append(loop)

    loop_a, loop_b = pair_loops
    if len(loop_a) != len(loop_b):
        return None

    anchor_home = split_info['anchor_home']
    anchor_index_a = _anchor_index_for_loop(loop_a, anchor_home)
    anchor_index_b = _anchor_index_for_loop(loop_b, anchor_home)
    if anchor_index_a is None or anchor_index_b is None:
        return None

    loop_a = _rotate_loop(loop_a, anchor_index_a)
    loop_b = _rotate_loop(loop_b, anchor_index_b)

    reversed_b = _reverse_loop_with_fixed_start(loop_b)
    forward_score = _pair_distance_score(loop_a, loop_b)
    reverse_score = _pair_distance_score(loop_a, reversed_b)
    if reverse_score is not None and (
        forward_score is None or reverse_score < forward_score
    ):
        loop_b = reversed_b
        chosen_direction = "reversed"
    else:
        chosen_direction = "forward"

    return {
        'loops': (loop_a, loop_b),
        'direction': chosen_direction,
        'group_matches': matches,
    }


def _weld_live_corner_rings(bm, split_infos):
    live_data = closed_loop_bridged.detect(bm)
    groups = live_data.get('groups') if live_data else []
    if not groups:
        return 0, []

    targetmap = {}
    diagnostics = []

    for corner_index, split_info in enumerate(split_infos, start=1):
        pair_data = _live_corner_loop_pair(groups, split_info)
        if pair_data is None:
            diagnostics.append({
                'corner_index': corner_index,
                'status': 'no_live_pair',
            })
            continue

        loop_a, loop_b = pair_data['loops']
        pair_count = 0
        for vert_a, vert_b in zip(loop_a, loop_b):
            if (
                not getattr(vert_a, "is_valid", False)
                or not getattr(vert_b, "is_valid", False)
                or vert_a is vert_b
            ):
                continue
            targetmap[vert_b] = vert_a
            pair_count += 1

        diagnostics.append({
            'corner_index': corner_index,
            'status': 'paired',
            'pair_count': pair_count,
            'ring_size': len(loop_a),
            'direction': pair_data['direction'],
        })

    if not targetmap:
        return 0, diagnostics

    try:
        bmesh.ops.weld_verts(
            bm,
            targetmap=targetmap,
        )
    except Exception:
        return 0, diagnostics

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    return len(targetmap), diagnostics


def _weld_open_path_corner_rings(bm, split_infos, corner_directions):
    live_data = closed_loop_bridged.detect(bm)
    groups = live_data.get('groups') if live_data else []
    if not groups:
        return 0, []

    groups = _anchor_open_path_groups(groups, split_infos)
    components = _ordered_open_path_group_components(groups, split_infos)
    ordered_edges = [
        edge
        for component in components
        for edge in component['ordered_edges']
    ]
    if not ordered_edges or len(ordered_edges) != len(split_infos) or len(corner_directions) != len(split_infos):
        return _weld_live_corner_rings(bm, split_infos)

    targetmap = {}
    diagnostics = []

    for corner_index, (edge, split_info, direction) in enumerate(
        zip(ordered_edges, split_infos, corner_directions),
        start=1,
    ):
        loop_a = list(_group_loops(groups[edge['group_a']])[edge['loop_a']])
        loop_b = list(_group_loops(groups[edge['group_b']])[edge['loop_b']])
        if len(loop_a) != len(loop_b):
            diagnostics.append({
                'corner_index': corner_index,
                'status': 'size_mismatch',
            })
            continue

        anchor_home = split_info['anchor_home']
        anchor_index_a = _anchor_index_for_loop(loop_a, anchor_home)
        anchor_index_b = _anchor_index_for_loop(loop_b, anchor_home)
        if anchor_index_a is None or anchor_index_b is None:
            diagnostics.append({
                'corner_index': corner_index,
                'status': 'missing_anchor',
            })
            continue

        loop_a = _rotate_loop(loop_a, anchor_index_a)
        loop_b = _rotate_loop(loop_b, anchor_index_b)
        if direction == 'reversed':
            loop_b = _reverse_loop_with_fixed_start(loop_b)

        pair_count = 0
        for vert_a, vert_b in zip(loop_a, loop_b):
            if (
                not getattr(vert_a, "is_valid", False)
                or not getattr(vert_b, "is_valid", False)
                or vert_a is vert_b
            ):
                continue
            targetmap[vert_b] = vert_a
            pair_count += 1

        diagnostics.append({
            'corner_index': corner_index,
            'status': 'paired',
            'pair_count': pair_count,
            'ring_size': len(loop_a),
            'direction': direction,
        })

    if not targetmap:
        return 0, diagnostics

    try:
        bmesh.ops.weld_verts(
            bm,
            targetmap=targetmap,
        )
    except Exception:
        return 0, diagnostics

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    return len(targetmap), diagnostics


def detect(bm):
    _set_last_detect_reason(None)
    face_corner_data = _detect_face_corner_groups(bm)
    if face_corner_data and face_corner_data['groups']:
        _set_last_detect_reason("matched face-corner groups")
        return face_corner_data

    bridged_data = get_bridged_chain(bm)
    if bridged_data and bridged_data[1] and len(bridged_data[0]) >= 2:
        result = _result([bridged_data])
        if result is not None:
            _set_last_detect_reason("matched direct bridged chain")
            return result

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        result = _result([auto_bridged])
        if result is not None:
            _set_last_detect_reason("matched auto bridged chain")
            return result

    return face_corner_data


def _execute_open_path(bm, obj, direction, report, selected_faces, candidates):
    path_label = 'open'
    anchor_plan = _build_open_path_anchor_plan(candidates, obj)
    shared_loops = anchor_plan['shared_loops'] if anchor_plan else _shared_ring_loops(candidates)
    if not shared_loops:
        data = closed_loop_bridged.detect(bm)
        if data is None:
            if report is not None:
                detail = _LAST_DETECT_REASON or "no detail"
                report(
                    {'ERROR'},
                    f"CLBWC exec failed: path={path_label}, could not detect any shared corner rings. Reason: {detail}",
                )
            return {'CANCELLED'}
        result = closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data,
        )
        if report is not None and result == {'FINISHED'}:
            report({'INFO'}, f"CLBWC exec finished: path={path_label}.")
        return result

    anchor_homes = (
        anchor_plan['anchor_homes']
        if anchor_plan is not None
        else _choose_anchor_homes(shared_loops, obj)
    )
    split_infos = _split_shared_corner_rings(bm, selected_faces, shared_loops, anchor_homes)
    if not split_infos:
        if report is not None:
            report({'ERROR'}, f"CLBWC exec failed: path={path_label}, could not split shared corner rings.")
        return {'CANCELLED'}

    try:
        fresh_data = closed_loop_bridged.detect(bm)
        if not fresh_data or not fresh_data.get('groups'):
            if report is not None:
                report({'ERROR'}, f"CLBWC exec failed: path={path_label}, disconnected bridge detection found no groups.")
            return {'CANCELLED'}

        fresh_data = {
            **fresh_data,
            'groups': _anchor_open_path_groups(fresh_data['groups'], split_infos),
            'skip_phase_normalization': True,
        }
        result = closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=fresh_data,
        )
    finally:
        if anchor_plan is not None:
            merged, _live_weld_diagnostics = _weld_open_path_corner_rings(
                bm,
                split_infos,
                anchor_plan['corner_directions'],
            )
        else:
            merged, _live_weld_diagnostics = _weld_live_corner_rings(bm, split_infos)
        if merged == 0:
            fallback_targetmap = _targetmap_from_split_clusters(split_infos)
            if fallback_targetmap:
                try:
                    bmesh.ops.weld_verts(
                        bm,
                        targetmap=fallback_targetmap,
                    )
                    bm.verts.ensure_lookup_table()
                    bm.edges.ensure_lookup_table()
                    bm.faces.ensure_lookup_table()
                    bm.normal_update()
                    merged = len(fallback_targetmap)
                except Exception:
                    merged = 0
        bmesh.update_edit_mesh(obj.data)

    if report is not None and result != {'FINISHED'}:
        report(
            {'ERROR'},
            f"CLBWC exec failed: path={path_label}, cancelled by closed loop bridge: result={sorted(result)}.",
        )
    elif report is not None:
        report({'INFO'}, f"CLBWC exec finished: path={path_label}.")
    return result


def _execute_closed_path(bm, obj, direction, report, selected_faces, candidates):
    path_label = 'closed'
    shared_loops, _closed_path_diagnostics = _order_closed_path_shared_loops_with_diagnostics(candidates)
    if not shared_loops:
        data = closed_loop_bridged.detect(bm)
        if data is None:
            if report is not None:
                detail = _LAST_DETECT_REASON or "no detail"
                report(
                    {'ERROR'},
                    f"CLBWC exec failed: path={path_label}, could not detect any shared corner rings. Reason: {detail}",
                )
            return {'CANCELLED'}
        result = closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data,
        )
        if report is not None and result == {'FINISHED'}:
            report({'INFO'}, f"CLBWC exec finished: path={path_label}.")
        return result

    synthetic_loop = [shared_loops[0]]
    synthetic_anchor_homes = _choose_anchor_homes(synthetic_loop, obj)
    synthetic_split_infos = _split_shared_corner_rings(
        bm,
        selected_faces,
        synthetic_loop,
        synthetic_anchor_homes,
    )
    if not synthetic_split_infos:
        if report is not None:
            report({'ERROR'}, f"CLBWC exec failed: path={path_label}, could not split synthetic seam ring.")
        return {'CANCELLED'}

    selected_faces, candidates = _selected_faces_and_candidates(bm)
    shared_loops = _shared_ring_loops(candidates)
    if not shared_loops:
        if report is not None:
            report({'ERROR'}, f"CLBWC exec failed: path={path_label}, synthetic seam created no shared corner rings.")
        merged, _live_weld_diagnostics = _weld_live_corner_rings(bm, synthetic_split_infos)
        if merged == 0:
            fallback_targetmap = _targetmap_from_split_clusters(synthetic_split_infos)
            if fallback_targetmap:
                try:
                    bmesh.ops.weld_verts(
                        bm,
                        targetmap=fallback_targetmap,
                    )
                    bm.verts.ensure_lookup_table()
                    bm.edges.ensure_lookup_table()
                    bm.faces.ensure_lookup_table()
                    bm.normal_update()
                except Exception:
                    pass
        bmesh.update_edit_mesh(obj.data)
        return {'CANCELLED'}

    anchor_homes = _choose_anchor_homes(shared_loops, obj)
    split_infos = _split_shared_corner_rings(bm, selected_faces, shared_loops, anchor_homes)
    if not split_infos:
        if report is not None:
            report({'ERROR'}, f"CLBWC exec failed: path={path_label}, could not split shared corner rings after synthetic seam.")
        merged, _live_weld_diagnostics = _weld_live_corner_rings(bm, synthetic_split_infos)
        if merged == 0:
            fallback_targetmap = _targetmap_from_split_clusters(synthetic_split_infos)
            if fallback_targetmap:
                try:
                    bmesh.ops.weld_verts(
                        bm,
                        targetmap=fallback_targetmap,
                    )
                    bm.verts.ensure_lookup_table()
                    bm.edges.ensure_lookup_table()
                    bm.faces.ensure_lookup_table()
                    bm.normal_update()
                except Exception:
                    pass
        return {'CANCELLED'}

    split_infos = synthetic_split_infos + split_infos

    try:
        fresh_data = closed_loop_bridged.detect(bm)
        if not fresh_data or not fresh_data.get('groups'):
            if report is not None:
                report({'ERROR'}, f"CLBWC exec failed: path={path_label}, disconnected bridge detection found no groups.")
            return {'CANCELLED'}

        fresh_data = {
            **fresh_data,
            'groups': [
                _anchor_group_rings(group_data, split_infos)
                for group_data in fresh_data['groups']
            ],
        }
        result = closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=fresh_data,
        )
    finally:
        merged, _live_weld_diagnostics = _weld_live_corner_rings(bm, split_infos)
        if merged == 0:
            fallback_targetmap = _targetmap_from_split_clusters(split_infos)
            if fallback_targetmap:
                try:
                    bmesh.ops.weld_verts(
                        bm,
                        targetmap=fallback_targetmap,
                    )
                    bm.verts.ensure_lookup_table()
                    bm.edges.ensure_lookup_table()
                    bm.faces.ensure_lookup_table()
                    bm.normal_update()
                    merged = len(fallback_targetmap)
                except Exception:
                    merged = 0
        bmesh.update_edit_mesh(obj.data)

    if report is not None and result != {'FINISHED'}:
        report(
            {'ERROR'},
            f"CLBWC exec failed: path={path_label}, cancelled by closed loop bridge: result={sorted(result)}.",
        )
    elif report is not None:
        report({'INFO'}, f"CLBWC exec finished: path={path_label}.")
    return result


def execute(bm, obj, direction, report=None, data=None):
    selected_faces, candidates = _selected_faces_and_candidates(bm)
    if not selected_faces:
        data = closed_loop_bridged.detect(bm)
        if data is None:
            if report is not None:
                detail = _LAST_DETECT_REASON or "no detail"
                report(
                    {'ERROR'},
                    f"Could not detect a valid closed loop bridge with corners. Reason: {detail}",
                )
            return {'CANCELLED'}
        return closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data,
        )

    path_info = _classify_path_type(selected_faces, candidates)
    if path_info['path_type'] == 'open':
        return _execute_open_path(
            bm,
            obj,
            direction,
            report,
            selected_faces,
            candidates,
        )

    return _execute_closed_path(
        bm,
        obj,
        direction,
        report,
        selected_faces,
        candidates,
    )
