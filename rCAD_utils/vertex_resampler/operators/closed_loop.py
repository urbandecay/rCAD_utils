# closed_loop.py — Resample closed rings and bridged cylinder selections.

import bmesh

from .detection_utils import get_selected_islands
from .resample_common import execute_aligned_loops_logic

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
        execute_aligned_loops_logic(bm, obj, rings_data, direction, report=report)
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
        for e in sorted(v.link_edges, key=lambda edge: edge.index):
            if not e.select:
                other = e.other_vert(v)
                if other not in sel_verts_set:
                    shaft_map[v] = other
                    break

    if len(shaft_map) != len(verts):
        return None

    partner_set = set(shaft_map.values())
    start = min(partner_set, key=lambda vert: vert.index)
    partner_ordered = [start]
    visited = {start}
    curr = start
    for _ in range(len(verts) - 1):
        found = False
        for e in sorted(curr.link_edges, key=lambda edge: edge.index):
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
        for va in sorted(verts_a, key=lambda vert: vert.index):
            for e in sorted(va.link_edges, key=lambda edge: edge.index):
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
    for idx, neighbors in sorted(adj.items()):
        if len(neighbors) == 1:
            start_node = idx
            break
    if start_node is None:
        start_node = 0

    ordered_indices = [start_node]
    visited_idx = {start_node}
    curr = start_node

    while True:
        neighbors = sorted(adj[curr])
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
            for e in sorted(v_curr.link_edges, key=lambda edge: edge.index):
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
