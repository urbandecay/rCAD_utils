# bridge_utils.py — Shared helpers for bridged loop detection.

from .detection_utils import get_selected_islands


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
    for vert in verts:
        for edge in sorted(vert.link_edges, key=lambda item: item.index):
            if edge.select:
                continue
            other = edge.other_vert(vert)
            if other not in sel_verts_set:
                shaft_map[vert] = other
                break

    if len(shaft_map) != len(verts):
        return None

    partner_set = set(shaft_map.values())
    start = min(partner_set, key=lambda vert: vert.index)
    partner_ordered = [start]
    visited = {start}
    current = start

    for _ in range(len(verts) - 1):
        found = False
        for edge in sorted(current.link_edges, key=lambda item: item.index):
            other = edge.other_vert(current)
            if other in partner_set and other not in visited:
                partner_ordered.append(other)
                visited.add(other)
                current = other
                found = True
                break
        if not found:
            break

    if len(partner_ordered) != len(verts):
        return None

    aligned_partner = [shaft_map[vert] for vert in verts]
    return ([verts, aligned_partner], True)


def get_bridged_chain(bm):
    raw_islands = get_selected_islands(bm)
    if len(raw_islands) < 2:
        return None

    island_map = {index: raw_islands[index] for index in range(len(raw_islands))}
    is_stack_closed = raw_islands[0]['closed']

    adj = {index: [] for index in island_map}
    for idx_a, island_a in island_map.items():
        verts_a = set(island_a['verts'])
        for vert_a in sorted(verts_a, key=lambda item: item.index):
            for edge in sorted(vert_a.link_edges, key=lambda item: item.index):
                if edge.select:
                    continue
                other = edge.other_vert(vert_a)
                for idx_b, island_b in island_map.items():
                    if idx_a == idx_b:
                        continue
                    if other in island_b['verts']:
                        if idx_b not in adj[idx_a]:
                            adj[idx_a].append(idx_b)
                        break

    start_node = None
    for index, neighbors in sorted(adj.items()):
        if len(neighbors) == 1:
            start_node = index
            break
    if start_node is None:
        start_node = 0

    ordered_indices = [start_node]
    visited_idx = {start_node}
    current = start_node

    while True:
        neighbors = sorted(adj[current])
        next_node = None
        for neighbor in neighbors:
            if neighbor not in visited_idx:
                next_node = neighbor
                break
        if next_node is None:
            break
        ordered_indices.append(next_node)
        visited_idx.add(next_node)
        current = next_node

    aligned_loops = []
    first_loop = island_map[ordered_indices[0]]['verts']
    aligned_loops.append(first_loop)

    for index in range(len(ordered_indices) - 1):
        idx_next = ordered_indices[index + 1]
        curr_loop_verts = aligned_loops[-1]
        next_loop_verts = island_map[idx_next]['verts']

        connection_map = {}
        for curr_vert in curr_loop_verts:
            for edge in sorted(curr_vert.link_edges, key=lambda item: item.index):
                if edge.select:
                    continue
                other = edge.other_vert(curr_vert)
                if other in next_loop_verts:
                    connection_map[curr_vert] = other
                    break

        new_next_loop = []
        valid_chain = True
        for curr_vert in curr_loop_verts:
            if curr_vert not in connection_map:
                valid_chain = False
                break
            new_next_loop.append(connection_map[curr_vert])

        if not valid_chain or len(new_next_loop) != len(curr_loop_verts):
            return None
        aligned_loops.append(new_next_loop)

    return (aligned_loops, is_stack_closed)
