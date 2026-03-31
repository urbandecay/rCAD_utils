# closed_loop_bridged_with_corners.py — Resample bridged closed loops with corners.

import bmesh

from . import closed_loop_bridged
from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
from ..debug import debug_log, face_ref, face_refs, loop_ref, mesh_stats
from ..seam_manager import load_seam_homes, save_seam_homes

_LAST_DETECT_REASON = None


def _set_last_detect_reason(reason):
    global _LAST_DETECT_REASON
    _LAST_DETECT_REASON = reason


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

    candidates = []
    seen = set()

    for face in sorted(quad_faces, key=lambda item: item.index):
        for bit in (0, 1):
            strip_faces, reason = _candidate_strip_faces(face, bit, quad_faces)
            attempt = {
                'start_face': face_ref(face),
                'bit': bit,
                'reason': reason or 'candidate built',
            }
            if not strip_faces:
                diagnostics['attempts'].append(attempt)
                continue

            key = frozenset(strip_faces)
            if key in seen:
                attempt['reason'] = "duplicate strip candidate"
                diagnostics['attempts'].append(attempt)
                continue
            seen.add(key)

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
    debug_log(
        "clbwc",
        "Face-corner detect start.",
        mesh=mesh_stats(bm),
        selected_face_count=len(selected_faces),
        selected_faces=face_refs(selected_faces),
    )
    if not selected_faces:
        _set_last_detect_reason("no selected face set could be built from the selection")
        return None, []

    candidates = []
    component_failures = []
    for component in closed_loop_bridged._face_components(selected_faces):
        component_candidates, diagnostics = _all_face_ring_candidates(component)
        debug_log(
            "clbwc",
            "Processed face component.",
            component_faces=diagnostics['component_faces'],
            quad_face_count=diagnostics['quad_face_count'],
            reason=diagnostics.get('reason'),
            chosen_strip_faces=diagnostics.get('chosen_strip_faces'),
            chosen_extra_faces=diagnostics.get('chosen_extra_faces'),
            chosen_loops=diagnostics.get('chosen_loops'),
            attempts=diagnostics['attempts'][:8],
        )
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
    loop_map = {}
    for candidate in candidates:
        for loop in candidate['rings'][0]:
            key = _loop_key(loop)
            if len(key) != len(loop):
                continue
            loop_map.setdefault(key, []).append(loop)

    shared_loops = []
    for loops in loop_map.values():
        if len(loops) >= 2:
            shared_loops.append(loops[0])
    return shared_loops


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
    tol = 1.0e-9
    for loop, anchor_home in zip(shared_loops, anchor_homes):
        clusters = []
        for position in _loop_positions(loop):
            cluster = [
                vert for vert in bm.verts
                if vert.is_valid and (vert.co - position).length <= tol
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

    forced_seam_verts = [
        {loop_item[0]} if loop_item else set()
        for loop_item in rotated_loops
    ]
    if isinstance(group_data, dict):
        updated = dict(group_data)
    else:
        updated = {}
    updated['rings'] = (rotated_loops, is_closed)
    updated['use_seams'] = True
    updated['migrate_seams'] = False
    updated['forced_seam_verts'] = forced_seam_verts
    return updated


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


def detect(bm):
    _set_last_detect_reason(None)
    debug_log(
        "clbwc",
        "Detect start.",
        mesh=mesh_stats(bm),
    )
    face_corner_data = _detect_face_corner_groups(bm)
    if face_corner_data and face_corner_data['groups']:
        _set_last_detect_reason("matched face-corner groups")
        debug_log(
            "clbwc",
            "Matched face-corner groups.",
            group_count=len(face_corner_data['groups']),
        )
        return face_corner_data

    bridged_data = get_bridged_chain(bm)
    if bridged_data and bridged_data[1] and len(bridged_data[0]) >= 2:
        result = _result([bridged_data])
        if result is not None:
            _set_last_detect_reason("matched direct bridged chain")
            debug_log(
                "clbwc",
                "Matched direct bridged chain.",
                loops=[loop_ref(loop) for loop in bridged_data[0]],
            )
            return result

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        result = _result([auto_bridged])
        if result is not None:
            _set_last_detect_reason("matched auto bridged chain")
            debug_log(
                "clbwc",
                "Matched auto bridged chain.",
                loops=[loop_ref(loop) for loop in auto_bridged[0]],
            )
            return result

    debug_log("clbwc", "No corner-group match; falling back to legacy bridge detection only.")
    return face_corner_data


def execute(bm, obj, direction, report=None, data=None):
    debug_log(
        "clbwc",
        "Execute start.",
        direction=direction,
        mesh=mesh_stats(bm),
    )
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

    shared_loops = _shared_ring_loops(candidates)
    if not shared_loops:
        data = closed_loop_bridged.detect(bm)
        if data is None:
            if report is not None:
                detail = _LAST_DETECT_REASON or "no detail"
                report(
                    {'ERROR'},
                    f"Could not detect any shared corner rings. Reason: {detail}",
                )
            return {'CANCELLED'}
        return closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data,
        )

    if report is not None:
        report(
            {'INFO'},
            f"CLBWC exec: splitting {len(shared_loops)} shared corner ring(s).",
        )

    anchor_homes = _choose_anchor_homes(shared_loops, obj)
    split_infos = _split_shared_corner_rings(bm, selected_faces, shared_loops, anchor_homes)
    if not split_infos:
        if report is not None:
            report({'ERROR'}, "CLBWC exec failed: could not split shared corner rings.")
        return {'CANCELLED'}

    try:
        fresh_data = closed_loop_bridged.detect(bm)
        if not fresh_data or not fresh_data.get('groups'):
            if report is not None:
                report({'ERROR'}, "CLBWC exec failed: disconnected bridge detection found no groups.")
            return {'CANCELLED'}

        if report is not None:
            report(
                {'INFO'},
                f"CLBWC exec: disconnected detect found {len(fresh_data['groups'])} group(s).",
            )
            group_sizes = _group_ring_sizes(fresh_data['groups'])
            if group_sizes:
                report(
                    {'INFO'},
                    f"CLBWC exec: disconnected group ring sizes={group_sizes}.",
                )
            for summary in _corner_pre_resample_summaries(
                fresh_data['groups'],
                split_infos,
                direction,
            ):
                matches = [
                    (
                        f"g{item['group_index']}"
                        f"/l{item['loop_index']}"
                        f":{item['matched_positions']}/{item['ring_size']}"
                    )
                    for item in summary['matches']
                ]
                report(
                    {'INFO'},
                    "CLBWC exec: "
                    f"corner {summary['corner_index']}: "
                    f"recorded_positions={summary['recorded_positions']}, "
                    f"matched={matches or ['none']}, "
                    f"predicted_sizes={summary['predicted_sizes'] or []}, "
                    f"untracked_new_positions={summary['untracked_new_positions'] or []}.",
                )
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
        merged, live_weld_diagnostics = _weld_live_corner_rings(bm, split_infos)
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
        post_weld = _post_weld_selection_summary(bm)
        corner_post_weld = _corner_post_weld_summaries(split_infos)
        bmesh.update_edit_mesh(obj.data)
        if report is not None:
            report({'INFO'}, f"CLBWC exec: welded {merged} corner vert pair(s).")
            for diagnostic in live_weld_diagnostics:
                if diagnostic['status'] != 'paired':
                    report(
                        {'INFO'},
                        f"CLBWC exec: corner {diagnostic['corner_index']} live weld={diagnostic['status']}.",
                    )
                    continue
                report(
                    {'INFO'},
                    "CLBWC exec: "
                    f"corner {diagnostic['corner_index']} live weld: "
                    f"ring_size={diagnostic['ring_size']}, "
                    f"pair_count={diagnostic['pair_count']}, "
                    f"direction={diagnostic['direction']}.",
                )
            for summary in corner_post_weld:
                report(
                    {'INFO'},
                    "CLBWC exec: "
                    f"corner {summary['corner_index']} post-weld: "
                    f"recorded_positions={summary['recorded_positions']}, "
                    f"remaining_duplicate_positions={summary['remaining_duplicate_positions']}, "
                    f"live_cluster_counts={summary['live_cluster_counts']}.",
                )
            shared_ring_sizes = post_weld['shared_ring_sizes']
            if shared_ring_sizes:
                report(
                    {'INFO'},
                    f"CLBWC exec: post-weld shared ring sizes={shared_ring_sizes}.",
                )
            candidate_ring_sizes = post_weld['candidate_ring_sizes']
            if candidate_ring_sizes:
                report(
                    {'INFO'},
                    f"CLBWC exec: post-weld candidate ring sizes={candidate_ring_sizes}.",
                )

    if report is not None and result != {'FINISHED'}:
        report(
            {'ERROR'},
            f"CLBWC exec cancelled by closed loop bridge: result={sorted(result)}.",
        )
    return result
