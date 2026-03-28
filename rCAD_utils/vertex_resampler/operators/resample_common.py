# resample_common.py — Shared resampling execution helpers.

import bmesh

from ..math_engine import CatmullRomSpline
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


def apply_resample(bm, verts, new_coords, closed, direction):
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
    if closed:
        valid_verts = [v for v in verts if v.is_valid]
        verts[:] = get_sorted_verts_after_edit(valid_verts, closed)

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
        verts = c['verts']
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

        apply_resample(bm, verts, new_coords, False, direction)

    bmesh.update_edit_mesh(obj.data)
    return {'FINISHED'}
