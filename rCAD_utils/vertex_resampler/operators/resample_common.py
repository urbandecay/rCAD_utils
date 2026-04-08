# resample_common.py — Shared resampling execution helpers.

import bmesh

from .. import anchor_overlay
from ..debug import debug_log, mesh_stats, pair_ref, ring_group_ref
from ..math_engine import CatmullRomSpline
from ..ring_analyzer import analyze_rings
from ..seam_manager import (
    load_seam_homes,
    save_seam_homes,
    match_seam_homes,
    migrate_drifted_seams,
)
from ..vert_deletion import find_safe_deletion_index, delete_at_index
from ..vert_insertion import find_safe_insertion_index, insert_at_index
from ..topology_repair import repair_after_dissolve
from .detection_utils import (
    get_centroid,
    get_selected_islands,
    align_islands_to_boss,
    get_kissing_chains,
    check_if_anchored,
    check_selected_junction,
    get_junction_chains,
    get_anchored_chains,
    get_sorted_verts_after_edit,
)


def _report(report, level, message):
    if report is not None:
        report(level, message)


def _vert_is_usable(vert):
    try:
        return bool(vert is not None and vert.is_valid)
    except Exception:
        return False


def _normalize_forced_seam_verts(ring_group, forced_seam_verts):
    if forced_seam_verts is None:
        return None

    if len(forced_seam_verts) != len(ring_group.rings):
        return None

    normalized = []
    for ring_info, seam_verts in zip(ring_group.rings, forced_seam_verts):
        if seam_verts is None:
            normalized.append(set())
            continue

        normalized.append({
            vert for vert in seam_verts
            if _vert_is_usable(vert) and vert in ring_info.verts
        })

    return normalized


def _capture_anchor_overlay_points(obj, ring_group):
    if obj is None:
        return

    points = []
    matrix_world = obj.matrix_world
    for ring_info in ring_group.rings:
        seam_verts = sorted(
            (
                vert for vert in ring_info.seam_verts
                if _vert_is_usable(vert)
            ),
            key=lambda vert: vert.index,
        )
        for vert in seam_verts:
            points.append(matrix_world @ vert.co.copy())

    if points:
        anchor_overlay.add_points(points)


def _capture_cross_section_overlay(obj, ring_group):
    if obj is None or len(ring_group.rings) < 2:
        return

    matrix_world = obj.matrix_world
    between_ring_segments = []
    between_ring_lengths = []
    for ring_a, ring_b in zip(ring_group.rings, ring_group.rings[1:]):
        pair_count = min(len(ring_a.verts), len(ring_b.verts))
        for index in range(pair_count):
            vert_a = ring_a.verts[index]
            vert_b = ring_b.verts[index]
            if not (_vert_is_usable(vert_a) and _vert_is_usable(vert_b)):
                continue
            between_ring_segments.append((
                matrix_world @ vert_a.co.copy(),
                matrix_world @ vert_b.co.copy(),
            ))
            between_ring_lengths.append((vert_a.co - vert_b.co).length)

    along_loop_segments = []
    along_loop_lengths = []
    for ring_info in ring_group.rings:
        verts = ring_info.verts
        if len(verts) < 2:
            continue
        for index in range(len(verts) - 1):
            vert_a = verts[index]
            vert_b = verts[index + 1]
            if not (_vert_is_usable(vert_a) and _vert_is_usable(vert_b)):
                continue
            along_loop_segments.append((
                matrix_world @ vert_a.co.copy(),
                matrix_world @ vert_b.co.copy(),
            ))
            along_loop_lengths.append((vert_a.co - vert_b.co).length)

    between_avg = (
        sum(between_ring_lengths) / len(between_ring_lengths)
        if between_ring_lengths else float('inf')
    )
    along_avg = (
        sum(along_loop_lengths) / len(along_loop_lengths)
        if along_loop_lengths else float('inf')
    )

    segments = between_ring_segments if between_avg <= along_avg else along_loop_segments
    if segments:
        anchor_overlay.add_segments(segments)


def _sanitize_chain_verts(verts, closed):
    valid_verts = [v for v in verts if v.is_valid]
    if len(valid_verts) < 2:
        return valid_verts
    return get_sorted_verts_after_edit(valid_verts, closed)


def _outside_edge_score(vert, all_ring_verts):
    lengths = [
        edge.calc_length()
        for edge in vert.link_edges
        if edge.other_vert(vert) not in all_ring_verts
    ]
    return max(lengths, default=0.0)


def _ring_vert_positions(ring_info):
    return {
        vert: index
        for index, vert in enumerate(ring_info.verts)
        if vert.is_valid
    }


def _ring_step_distance(ring_info, index_a, index_b):
    delta = abs(index_a - index_b)
    if ring_info.is_closed:
        count = len(ring_info.verts)
        return min(delta, count - delta)
    return delta


def _min_ring_spacing(ring_info, vert, other_verts, positions=None):
    if not other_verts:
        return len(ring_info.verts)

    if positions is None:
        positions = _ring_vert_positions(ring_info)

    index = positions.get(vert)
    if index is None:
        return -1

    valid_other_indices = [
        positions[other]
        for other in other_verts
        if other in positions
    ]
    if not valid_other_indices:
        return len(ring_info.verts)

    return min(
        _ring_step_distance(ring_info, index, other_index)
        for other_index in valid_other_indices
    )


def _seam_clearance_steps(ring_info):
    if not ring_info.is_closed:
        return 1
    return 2 if len(ring_info.verts) >= 8 else 1


def _pick_spaced_ring_verts(ring_group, ring_info, candidate_verts, limit):
    candidates = [
        vert for vert in candidate_verts
        if vert.is_valid
    ]
    if limit is None or len(candidates) <= limit:
        return set(candidates)

    positions = _ring_vert_positions(ring_info)
    candidates = [vert for vert in candidates if vert in positions]
    if len(candidates) <= limit:
        return set(candidates)

    selected = [
        max(
            candidates,
            key=lambda vert: (
                _outside_edge_score(vert, ring_group.all_ring_verts),
                -vert.index,
            ),
        )
    ]
    remaining = [vert for vert in candidates if vert not in selected]
    clearance = _seam_clearance_steps(ring_info)

    while remaining and len(selected) < limit:
        clear_candidates = [
            vert for vert in remaining
            if _min_ring_spacing(ring_info, vert, selected, positions) >= clearance
        ]
        candidate_pool = clear_candidates or remaining
        best = max(
            candidate_pool,
            key=lambda vert: (
                _outside_edge_score(vert, ring_group.all_ring_verts),
                _min_ring_spacing(ring_info, vert, selected, positions),
                -vert.index,
            ),
        )
        selected.append(best)
        remaining.remove(best)

    return set(selected)


def _limit_seam_sets(ring_group, seam_sets, max_seams):
    if max_seams is None:
        return seam_sets

    limited_sets = []
    for ring_info, seam_set in zip(ring_group.rings, seam_sets):
        limited_sets.append(
            _pick_spaced_ring_verts(ring_group, ring_info, seam_set, max_seams)
        )

    return limited_sets


def _enforce_max_seams(bm, ring_group, max_seams):
    if max_seams is None:
        return

    for ring_info in ring_group.rings:
        outside_edges = []
        candidate_edges_by_vert = {}
        for vert in ring_info.verts:
            vert_outside_edges = [
                edge
                for edge in vert.link_edges
                if edge.other_vert(vert) not in ring_group.all_ring_verts
            ]
            outside_edges.extend(vert_outside_edges)
            if vert_outside_edges:
                candidate_edges_by_vert[vert] = max(
                    vert_outside_edges,
                    key=lambda edge: (edge.calc_length(), -edge.index),
                )

        unique_edges = list({edge for edge in outside_edges if edge.is_valid})
        if len(unique_edges) <= max_seams:
            continue

        keep_verts = _pick_spaced_ring_verts(
            ring_group,
            ring_info,
            candidate_edges_by_vert.keys(),
            max_seams,
        )
        keep_edges = {
            candidate_edges_by_vert[vert]
            for vert in keep_verts
            if vert in candidate_edges_by_vert
        }
        drop_edges = [edge for edge in unique_edges if edge not in keep_edges]
        if drop_edges:
            bmesh.ops.dissolve_edges(
                bm,
                edges=drop_edges,
                use_verts=False,
                use_face_split=False,
            )

        ring_info.seam_verts = set(keep_verts)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()


def execute_aligned_loops_logic(
    bm,
    obj,
    data,
    direction,
    report=None,
    use_seams=True,
    migrate_seams=None,
    max_seams=None,
    stack_is_cyclic=False,
    forced_seam_verts=None,
    result_info=None,
):
    loops, is_closed = data

    if migrate_seams is None:
        migrate_seams = use_seams

    for loop_index, loop in enumerate(loops, start=1):
        if any(not _vert_is_usable(vert) for vert in loop):
            _report(
                report,
                {'WARNING'},
                f"Aligned resample skipped because loop {loop_index} contains stale verts.",
            )
            return {'CANCELLED'}

    splines = []
    for loop in loops:
        pts = [v.co.copy() for v in loop]
        splines.append(CatmullRomSpline(pts, is_closed=is_closed))

    ring_group = analyze_rings(
        loops,
        is_closed,
        stack_is_cyclic=stack_is_cyclic,
    )
    normalized_forced_seams = _normalize_forced_seam_verts(
        ring_group,
        forced_seam_verts,
    )
    if normalized_forced_seams is not None:
        for ring_info, seam_verts in zip(ring_group.rings, normalized_forced_seams):
            ring_info.seam_verts = set(seam_verts)
    elif use_seams and max_seams is not None:
        _enforce_max_seams(bm, ring_group, max_seams)
    current_count = len(loops[0])
    debug_log(
        "aligned",
        "Starting aligned loop resample.",
        direction=direction,
        is_closed=is_closed,
        current_count=current_count,
        loop_count=len(loops),
        ring_group=ring_group_ref(ring_group),
        mesh=mesh_stats(bm),
    )
    migration_seams = [set(ring_info.seam_verts) for ring_info in ring_group.rings]
    migration_seams = _limit_seam_sets(ring_group, migration_seams, max_seams)
    if not use_seams:
        for ring_info in ring_group.rings:
            ring_info.seam_verts = set()
    elif normalized_forced_seams is not None:
        for ring_info, seam_verts in zip(ring_group.rings, normalized_forced_seams):
            ring_info.seam_verts = set(seam_verts)
    elif max_seams is not None:
        for ring_info, seam_verts in zip(ring_group.rings, migration_seams):
            ring_info.seam_verts = set(seam_verts)

    target_count = current_count + direction
    has_seams = use_seams and any(
        ring_info.seam_verts for ring_info in ring_group.rings
    )
    min_limit = 4 if is_closed and has_seams else (3 if is_closed else 2)
    if target_count < min_limit:
        target_count = min_limit

    if report is not None:
        _report(
            report,
            {'INFO'},
            "Aligned resample exec: "
            f"loops={len(loops)}, current={current_count}, target={target_count}, "
            f"closed={is_closed}, seams={'yes' if has_seams else 'no'}.",
        )

    if direction < 0 and ring_group.vert_count <= min_limit:
        _report(
            report,
            {'WARNING'},
            f"Can't remove any more verts. Minimum is {min_limit}.",
        )
        return {'CANCELLED'}

    new_coords_stack = []
    for spline in splines:
        coords = []
        num_segs = len(spline.segments)
        for index in range(target_count):
            if is_closed:
                factor = index / target_count
            else:
                factor = index / (target_count - 1)
            t_sample = factor * num_segs
            coords.append(spline.eval_global(t_sample))
        new_coords_stack.append(coords)

    if direction < 0:
        reduction_step = 0
        while ring_group.vert_count > target_count:
            if ring_group.vert_count <= min_limit:
                _report(
                    report,
                    {'WARNING'},
                    f"Can't remove any more verts. Minimum is {min_limit}.",
                )
                break
            idx = find_safe_deletion_index(ring_group)
            if idx < 0:
                _report(
                    report,
                    {'WARNING'},
                    "Can't remove any more verts safely right now.",
                )
                break
            debug_log(
                "aligned",
                "Beginning reduction step.",
                step=reduction_step,
                delete_index=idx,
                ring_group=ring_group_ref(ring_group),
            )
            neighbor_pairs = delete_at_index(bm, ring_group, idx)
            debug_log(
                "aligned",
                "Deletion produced neighbor pairs.",
                step=reduction_step,
                neighbor_pairs=[
                    pair_ref(item["a"], item["b"]) if isinstance(item, dict) else pair_ref(*item)
                    for item in neighbor_pairs
                ],
            )
            repair_after_dissolve(bm, neighbor_pairs)
            debug_log(
                "aligned",
                "Finished reduction step.",
                step=reduction_step,
                ring_group=ring_group_ref(ring_group),
                mesh=mesh_stats(bm),
            )
            reduction_step += 1
        if reduction_step == 0 and report is not None and current_count > target_count:
            _report(
                report,
                {'WARNING'},
                "Subtract path made no reduction steps.",
            )

    elif direction > 0:
        insertion_made = False
        insertion_steps = 0
        while ring_group.vert_count < target_count:
            idx = find_safe_insertion_index(ring_group)
            if idx < 0:
                if not insertion_made:
                    _report(
                        report,
                        {'WARNING'},
                        "Can't add any more verts safely right now.",
                    )
                    return {'CANCELLED'}
                break
            prev_count = ring_group.vert_count
            repair_pairs = insert_at_index(bm, ring_group, idx)
            if ring_group.vert_count == prev_count:
                if not insertion_made:
                    _report(
                        report,
                        {'WARNING'},
                        "Add-vert step failed before any verts were inserted.",
                    )
                    return {'CANCELLED'}
                break
            insertion_made = True
            insertion_steps += 1
            repair_after_dissolve(bm, repair_pairs)
        if report is not None:
            _report(
                report,
                {'INFO'},
                f"Add path steps={insertion_steps}, final_ring_count={ring_group.vert_count}.",
            )

    if is_closed and migrate_seams:
        if not use_seams:
            for ring_info, seam_verts in zip(ring_group.rings, migration_seams):
                ring_info.seam_verts = {vert for vert in seam_verts if vert.is_valid}
        stored_homes = load_seam_homes(obj)

        loop0_verts = [v for v in ring_group.rings[0].verts if v.is_valid]
        count = len(loop0_verts)
        avg_edge_len = (
            sum(
                (loop0_verts[idx].co - loop0_verts[(idx + 1) % count].co).length
                for idx in range(count)
            ) / count
            if count >= 2 else 0.1
        )

        seam_homes = match_seam_homes(ring_group, stored_homes, avg_edge_len)
        save_seam_homes(obj, stored_homes)
    else:
        seam_homes = None

    bm.verts.ensure_lookup_table()
    for ring_index, ring_info in enumerate(ring_group.rings):
        target_coords = new_coords_stack[ring_index]
        loop = ring_info.verts
        min_len = min(len(loop), len(target_coords))
        for coord_index in range(min_len):
            if loop[coord_index].is_valid:
                loop[coord_index].co = target_coords[coord_index]
                loop[coord_index].select = True

    if is_closed and migrate_seams and seam_homes:
        edge_lengths = []
        for ring_info in ring_group.rings:
            loop = ring_info.verts
            for idx in range(len(loop) - 1):
                if loop[idx].is_valid and loop[idx + 1].is_valid:
                    edge_lengths.append((loop[idx].co - loop[idx + 1].co).length)
        threshold = (sum(edge_lengths) / len(edge_lengths) * 0.5) if edge_lengths else 0.1
        migrate_drifted_seams(bm, ring_group, seam_homes, threshold)
        _enforce_max_seams(bm, ring_group, max_seams)

    ring_vert_set = set()
    for ring_info in ring_group.rings:
        for vert in ring_info.verts:
            if vert.is_valid:
                ring_vert_set.add(vert)
    for face in bm.faces:
        if all(vert in ring_vert_set for vert in face.verts):
            face.select = True

    bmesh.update_edit_mesh(obj.data)
    _capture_anchor_overlay_points(obj, ring_group)
    _capture_cross_section_overlay(obj, ring_group)
    if report is not None:
        _report(
            report,
            {'INFO'},
            f"Aligned resample finished: final_ring_count={len(ring_group.rings[0].verts)}.",
        )
    debug_log(
        "aligned",
        "Finished aligned loop resample.",
        direction=direction,
        target_count=target_count,
        final_ring_group=ring_group_ref(ring_group),
        mesh=mesh_stats(bm),
    )
    if result_info is not None:
        result_info['ring_group'] = ring_group
    return {'FINISHED'}


def apply_resample(bm, verts, new_coords, closed, direction):
    verts[:] = _sanitize_chain_verts(verts, closed)
    if len(verts) < 2:
        return

    target_count = len(new_coords)

    if direction > 0:
        while len(verts) < target_count:
            max_len = -1.0
            best_idx = -1
            for k in range(len(verts) - 1):
                dist = (verts[k].co - verts[k + 1].co).length
                if dist > max_len:
                    max_len = dist
                    best_idx = k
            if best_idx != -1:
                v1, v2 = verts[best_idx], verts[best_idx + 1]
                e = bm.edges.get((v1, v2)) or bm.edges.get((v2, v1))
                if e:
                    res = bmesh.utils.edge_split(e, v1, 0.5)
                    new_v = (
                        res[0]
                        if isinstance(res[0], bmesh.types.BMVert)
                        else res[1]
                    )
                    verts.insert(best_idx + 1, new_v)
            else:
                break

    elif direction < 0:
        while len(verts) > target_count:
            if len(verts) <= 2:
                break
            v_kill = verts[1]
            verts.pop(1)
            if v_kill.is_valid:
                bmesh.ops.dissolve_verts(bm, verts=[v_kill])

    bm.verts.ensure_lookup_table()
    verts[:] = _sanitize_chain_verts(verts, closed)
    if len(verts) < 2:
        return

    min_len = min(len(verts), len(new_coords))
    for k in range(min_len):
        if verts[k].is_valid:
            verts[k].co = new_coords[k]
            verts[k].select = True

    bm.edges.ensure_lookup_table()
    for k in range(len(verts) - 1):
        v1, v2 = verts[k], verts[k + 1]
        if v1.is_valid and v2.is_valid:
            e = bm.edges.get((v1, v2)) or bm.edges.get((v2, v1))
            if e:
                e.select = True


def execute_floating_logic(bm, obj, direction, islands=None):
    if islands is None:
        islands = get_selected_islands(bm)
    if not islands:
        return {'CANCELLED'}
    islands.sort(key=lambda x: get_centroid(x['verts']).z, reverse=True)
    align_islands_to_boss(islands)
    boss_island = islands[0]
    boss_verts = boss_island['verts']
    target_count = len(boss_verts) + direction
    if target_count < 2:
        target_count = 2

    for i, island in enumerate(islands):
        verts = island['verts']
        closed = island['closed']
        pts = [v.co.copy() for v in verts]
        my_spline = CatmullRomSpline(pts, closed)
        start_t = 0.0
        if closed:
            start_t = my_spline.find_closest_t(boss_verts[0].co)

        num_segs = len(my_spline.segments)
        new_co_list = []
        for k in range(target_count):
            factor = k / target_count if closed else k / (target_count - 1)
            if closed:
                t_sample = (start_t + (factor * num_segs)) % num_segs
            else:
                t_sample = factor * num_segs
            new_co_list.append(my_spline.eval_global(t_sample))

        apply_resample(bm, verts, new_co_list, closed, direction)

    bmesh.update_edit_mesh(obj.data)
    return {'FINISHED'}


def execute_anchored_logic(bm, obj, direction, mode='ANCHORED', precalc_chains=None):
    if precalc_chains:
        chains = precalc_chains
    elif mode == 'JUNCTION':
        chains = get_junction_chains(bm)
    else:
        chains = get_anchored_chains(bm)

    if not chains:
        return {'CANCELLED'}

    for c in chains:
        verts = _sanitize_chain_verts(c['verts'], c['closed'])
        if len(verts) < 2:
            continue
        pts = [v.co.copy() for v in verts]
        is_closed = c['closed']
        spline = CatmullRomSpline(pts, is_closed)

        current_count = len(verts)
        target_count = current_count + direction
        if target_count < 2:
            target_count = 2

        new_coords = []
        num_segs = len(spline.segments)
        for i in range(target_count):
            t = (i / (target_count - 1)) * num_segs
            new_coords.append(spline.eval_global(t))

        c['verts'] = verts
        apply_resample(bm, c['verts'], new_coords, False, direction)

    bmesh.update_edit_mesh(obj.data)
    return {'FINISHED'}
