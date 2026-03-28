# closed_loop.py — Resample closed rings and bridged cylinder selections.

import bmesh

from ..math_engine import CatmullRomSpline
from ..ring_analyzer import analyze_rings
from ..vert_deletion import find_safe_deletion_index, delete_at_index
from ..vert_insertion import find_safe_insertion_index, insert_at_index
from ..topology_repair import repair_after_dissolve
from ..seam_manager import (
    load_seam_homes,
    save_seam_homes,
    match_seam_homes,
    migrate_drifted_seams,
)
from .detection_utils import get_selected_islands


def _report(report, level, message):
    if report is not None:
        report(level, message)


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and len(bridged_data[0]) >= 2:
        return bridged_data

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        return auto_bridged

    return None


def execute(bm, obj, direction, report=None):
    data = detect(bm)
    if not data:
        return {'CANCELLED'}

    groups = data if isinstance(data, list) else [data]
    for rings_data in groups:
        _execute_bridged_logic(bm, obj, rings_data, direction, report=report)
    return {'FINISHED'}


def _execute_bridged_logic(bm, obj, data, direction, report=None):
    loops, is_closed = data

    splines = []
    for loop in loops:
        pts = [v.co.copy() for v in loop]
        splines.append(CatmullRomSpline(pts, is_closed=is_closed))

    ring_group = analyze_rings(loops, is_closed)

    current_count = len(loops[0])
    target_count = current_count + direction
    has_seams = any(ring_info.seam_verts for ring_info in ring_group.rings)
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
    for sp in splines:
        coords = []
        num_segs = len(sp.segments)
        for k in range(target_count):
            if is_closed:
                factor = k / target_count
            else:
                factor = k / (target_count - 1)
            t_sample = factor * num_segs
            coords.append(sp.eval_global(t_sample))
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

    stored_homes = load_seam_homes(obj)

    loop0_verts = [v for v in ring_group.rings[0].verts if v.is_valid]
    _n = len(loop0_verts)
    avg_edge_len = (
        sum((loop0_verts[k].co - loop0_verts[(k + 1) % _n].co).length
            for k in range(_n)) / _n
        if _n >= 2 else 0.1
    )

    seam_homes = match_seam_homes(ring_group, stored_homes, avg_edge_len)
    save_seam_homes(obj, stored_homes)

    bm.verts.ensure_lookup_table()
    for i, ring_info in enumerate(ring_group.rings):
        target_coords = new_coords_stack[i]
        loop = ring_info.verts
        min_len = min(len(loop), len(target_coords))
        for k in range(min_len):
            if loop[k].is_valid:
                loop[k].co = target_coords[k]
                loop[k].select = True

    if seam_homes:
        edge_lengths = []
        for ring_info in ring_group.rings:
            loop = ring_info.verts
            for k in range(len(loop) - 1):
                if loop[k].is_valid and loop[k + 1].is_valid:
                    edge_lengths.append((loop[k].co - loop[k + 1].co).length)
        threshold = (sum(edge_lengths) / len(edge_lengths) * 0.5) if edge_lengths else 0.1
        migrate_drifted_seams(bm, ring_group, seam_homes, threshold)

    ring_vert_set = set()
    for ring_info in ring_group.rings:
        for v in ring_info.verts:
            if v.is_valid:
                ring_vert_set.add(v)
    for f in bm.faces:
        if all(v in ring_vert_set for v in f.verts):
            f.select = True

    bmesh.update_edit_mesh(obj.data)
    return {'FINISHED'}


def get_auto_bridged_chain(bm):
    raw_islands = get_selected_islands(bm)
    if len(raw_islands) != 1:
        return None

    island = raw_islands[0]
    if not island['closed']:
        return None

    verts = island['verts']
    sel_verts_set = set(v for v in bm.verts if v.select)

    shaft_map = {}
    for v in verts:
        for e in v.link_edges:
            if not e.select:
                other = e.other_vert(v)
                if other not in sel_verts_set:
                    shaft_map[v] = other
                    break

    if len(shaft_map) != len(verts):
        return None

    partner_set = set(shaft_map.values())
    start = shaft_map[verts[0]]
    partner_ordered = [start]
    visited = {start}
    curr = start
    for _ in range(len(verts) - 1):
        found = False
        for e in curr.link_edges:
            other = e.other_vert(curr)
            if other in partner_set and other not in visited:
                partner_ordered.append(other)
                visited.add(other)
                curr = other
                found = True
                break
        if not found:
            break

    if len(partner_ordered) != len(verts):
        return None

    aligned_partner = [shaft_map[v] for v in verts]
    return ([verts, aligned_partner], True)


def get_bridged_chain(bm):
    raw_islands = get_selected_islands(bm)

    if len(raw_islands) < 2:
        return None

    island_map = {i: raw_islands[i] for i in range(len(raw_islands))}
    is_stack_closed = raw_islands[0]['closed']

    adj = {i: [] for i in island_map}
    for idx_a, island_a in island_map.items():
        verts_a = set(island_a['verts'])
        for va in verts_a:
            for e in va.link_edges:
                if not e.select:
                    other = e.other_vert(va)
                    for idx_b, island_b in island_map.items():
                        if idx_a == idx_b:
                            continue
                        if other in island_b['verts']:
                            if idx_b not in adj[idx_a]:
                                adj[idx_a].append(idx_b)
                            break

    start_node = None
    for idx, neighbors in adj.items():
        if len(neighbors) == 1:
            start_node = idx
            break
    if start_node is None:
        start_node = 0

    ordered_indices = [start_node]
    visited_idx = {start_node}
    curr = start_node

    while True:
        neighbors = adj[curr]
        next_node = None
        for n in neighbors:
            if n not in visited_idx:
                next_node = n
                break
        if next_node is not None:
            ordered_indices.append(next_node)
            visited_idx.add(next_node)
            curr = next_node
        else:
            break

    aligned_loops = []
    first_loop = island_map[ordered_indices[0]]['verts']
    aligned_loops.append(first_loop)

    for i in range(len(ordered_indices) - 1):
        idx_next = ordered_indices[i + 1]
        curr_loop_verts = aligned_loops[-1]
        next_loop_verts = island_map[idx_next]['verts']

        connection_map = {}
        for v_curr in curr_loop_verts:
            for e in v_curr.link_edges:
                if not e.select:
                    other = e.other_vert(v_curr)
                    if other in next_loop_verts:
                        connection_map[v_curr] = other
                        break

        new_next_loop = []
        valid_chain = True
        for v_curr in curr_loop_verts:
            if v_curr in connection_map:
                new_next_loop.append(connection_map[v_curr])
            else:
                valid_chain = False
                break

        if valid_chain and len(new_next_loop) == len(curr_loop_verts):
            aligned_loops.append(new_next_loop)
        else:
            return None

    return (aligned_loops, is_stack_closed)
