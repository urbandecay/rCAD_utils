# resample_common.py — Shared resampling execution helpers.

import bmesh

from ..math_engine import CatmullRomSpline
from ..ring_analyzer import analyze_rings
from ..seam_manager import (
    load_seam_homes,
    save_seam_homes,
    match_seam_homes,
    migrate_drifted_seams,
    load_face_seam_anchors,
    save_face_seam_anchors,
    match_face_seam_anchors,
    migrate_drifted_face_seams,
    ensure_all_seam_edges,
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


def _limit_seam_sets(ring_group, seam_sets, max_seams):
    if max_seams is None:
        return seam_sets

    limited_sets = []
    for seam_set in seam_sets:
        if len(seam_set) <= max_seams:
            limited_sets.append(set(seam_set))
            continue

        ranked = sorted(
            seam_set,
            key=lambda vert: (
                -_outside_edge_score(vert, ring_group.all_ring_verts),
                vert.index,
            ),
        )
        limited_sets.append(set(ranked[:max_seams]))

    return limited_sets


def _enforce_max_seams(bm, ring_group, max_seams):
    if max_seams is None:
        return

    for ring_info in ring_group.rings:
        outside_edges = []
        for vert in ring_info.verts:
            for edge in vert.link_edges:
                if edge.other_vert(vert) not in ring_group.all_ring_verts:
                    outside_edges.append(edge)

        unique_edges = list({edge for edge in outside_edges if edge.is_valid})
        if len(unique_edges) <= max_seams:
            continue

        keep_edges = set()
        for seam_vert in sorted(ring_info.seam_verts, key=lambda vert: vert.index):
            seam_edges = [
                edge for edge in unique_edges
                if seam_vert in edge.verts
            ]
            if not seam_edges:
                continue
            keep_edges.add(
                max(seam_edges, key=lambda edge: (edge.calc_length(), -edge.index))
            )

        if len(keep_edges) < max_seams:
            ranked_edges = sorted(
                unique_edges,
                key=lambda edge: (-edge.calc_length(), edge.index),
            )
            for edge in ranked_edges:
                if edge in keep_edges:
                    continue
                keep_edges.add(edge)
                if len(keep_edges) >= max_seams:
                    break

        drop_edges = [edge for edge in unique_edges if edge not in keep_edges]
        if drop_edges:
            bmesh.ops.dissolve_edges(
                bm,
                edges=drop_edges,
                use_verts=False,
                use_face_split=False,
            )

        seam_verts = set()
        for edge in keep_edges:
            for vert in edge.verts:
                if vert in ring_group.all_ring_verts:
                    seam_verts.add(vert)
        ring_info.seam_verts = seam_verts

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()


def _primary_seam_vert(ring_group, ring_info):
    if not ring_info.seam_verts:
        return None

    return min(
        ring_info.seam_verts,
        key=lambda vert: (
            -_outside_edge_score(vert, ring_group.all_ring_verts),
            vert.index,
        ),
    )


def _anchor_ring_group_to_seams(ring_group):
    for ring_info in ring_group.rings:
        anchor_vert = _primary_seam_vert(ring_group, ring_info)
        if anchor_vert is None:
            continue

        try:
            anchor_index = ring_info.verts.index(anchor_vert)
        except ValueError:
            continue

        if anchor_index > 0:
            ring_info.verts = (
                ring_info.verts[anchor_index:] + ring_info.verts[:anchor_index]
            )


def execute_aligned_loops_logic(
    bm,
    obj,
    data,
    direction,
    report=None,
    use_seams=True,
    migrate_seams=None,
    max_seams=None,
    seam_drift_factor=0.5,
    persist_seam_homes=True,
    anchor_to_seams=False,
    use_endpoint_seam_anchors=False,
):
    loops, is_closed = data

    loops = [list(loop) for loop in loops]

    ring_group = analyze_rings(loops, is_closed)
    migration_seams = [set(ring_info.seam_verts) for ring_info in ring_group.rings]
    migration_seams = _limit_seam_sets(ring_group, migration_seams, max_seams)
    if migrate_seams is None:
        migrate_seams = use_seams
    if not use_seams:
        for ring_info in ring_group.rings:
            ring_info.seam_verts = set()
    elif max_seams is not None:
        for ring_info, seam_verts in zip(ring_group.rings, migration_seams):
            ring_info.seam_verts = set(seam_verts)
    if is_closed and anchor_to_seams and use_seams:
        _anchor_ring_group_to_seams(ring_group)

    splines = []
    for ring_info in ring_group.rings:
        pts = [v.co.copy() for v in ring_info.verts]
        splines.append(CatmullRomSpline(pts, is_closed=is_closed))

    current_count = ring_group.vert_count
    target_count = current_count + direction
    has_seams = use_seams and any(
        ring_info.seam_verts for ring_info in ring_group.rings
    )
    min_limit = 4 if is_closed and has_seams else (3 if is_closed else 2)
    if target_count < min_limit:
        target_count = min_limit

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
            neighbor_pairs = delete_at_index(bm, ring_group, idx)
            repair_after_dissolve(bm, neighbor_pairs)

    elif direction > 0:
        while ring_group.vert_count < target_count:
            idx = find_safe_insertion_index(ring_group)
            if idx < 0:
                break
            repair_pairs = insert_at_index(bm, ring_group, idx)
            repair_after_dissolve(bm, repair_pairs)

    if is_closed and migrate_seams:
        if not use_seams:
            for ring_info, seam_verts in zip(ring_group.rings, migration_seams):
                ring_info.seam_verts = {vert for vert in seam_verts if vert.is_valid}

        loop0_verts = [v for v in ring_group.rings[0].verts if v.is_valid]
        count = len(loop0_verts)
        avg_edge_len = (
            sum(
                (loop0_verts[idx].co - loop0_verts[(idx + 1) % count].co).length
                for idx in range(count)
            ) / count
            if count >= 2 else 0.1
        )

        if use_endpoint_seam_anchors:
            stored_anchors = load_face_seam_anchors(obj)
            seam_homes = None
            face_seam_anchors = match_face_seam_anchors(
                ring_group,
                stored_anchors,
                avg_edge_len,
            )
            save_face_seam_anchors(
                obj,
                list(face_seam_anchors.values()),
            )
        else:
            stored_homes = load_seam_homes(obj) if persist_seam_homes else []
            seam_homes = match_seam_homes(ring_group, stored_homes, avg_edge_len)
            if persist_seam_homes:
                save_seam_homes(obj, stored_homes)
            face_seam_anchors = None
    else:
        seam_homes = None
        face_seam_anchors = None

    bm.verts.ensure_lookup_table()
    for ring_index, ring_info in enumerate(ring_group.rings):
        target_coords = new_coords_stack[ring_index]
        loop = ring_info.verts
        min_len = min(len(loop), len(target_coords))
        for coord_index in range(min_len):
            if loop[coord_index].is_valid:
                loop[coord_index].co = target_coords[coord_index]
                loop[coord_index].select = True

    if is_closed and migrate_seams and face_seam_anchors:
        edge_lengths = []
        for ring_info in ring_group.rings:
            loop = ring_info.verts
            for idx in range(len(loop) - 1):
                if loop[idx].is_valid and loop[idx + 1].is_valid:
                    edge_lengths.append((loop[idx].co - loop[idx + 1].co).length)
        threshold = (
            sum(edge_lengths) / len(edge_lengths) * seam_drift_factor
        ) if edge_lengths else 0.1
        migrate_drifted_face_seams(bm, ring_group, face_seam_anchors, threshold)
        _enforce_max_seams(bm, ring_group, max_seams)

    elif is_closed and migrate_seams and seam_homes:
        edge_lengths = []
        for ring_info in ring_group.rings:
            loop = ring_info.verts
            for idx in range(len(loop) - 1):
                if loop[idx].is_valid and loop[idx + 1].is_valid:
                    edge_lengths.append((loop[idx].co - loop[idx + 1].co).length)
        threshold = (
            sum(edge_lengths) / len(edge_lengths) * seam_drift_factor
        ) if edge_lengths else 0.1
        migrate_drifted_seams(bm, ring_group, seam_homes, threshold)
        _enforce_max_seams(bm, ring_group, max_seams)

    # Final safety net — after all migration AND enforce_max_seams,
    # guarantee every tracked seam still has an edge. Nothing runs after this.
    if is_closed and migrate_seams:
        ensure_all_seam_edges(bm, ring_group)

    ring_vert_set = set()
    for ring_info in ring_group.rings:
        for vert in ring_info.verts:
            if vert.is_valid:
                ring_vert_set.add(vert)
    for face in bm.faces:
        if all(vert in ring_vert_set for vert in face.verts):
            face.select = True

    bmesh.update_edit_mesh(obj.data)
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
