# bridge_utils.py — Shared helpers for bridged loop detection.

from itertools import product

from .detection_utils import get_selected_islands


def _sorted_edges(vert):
    return sorted(vert.link_edges, key=lambda item: item.index)


def _edge_between(vert_a, vert_b):
    for edge in _sorted_edges(vert_a):
        if edge.other_vert(vert_a) is vert_b:
            return edge
    return None


def _has_quad_strip_face(curr_a, curr_b, next_a, next_b):
    quad_verts = {curr_a, curr_b, next_a, next_b}
    if len(quad_verts) != 4:
        return False

    shared_faces = set(curr_a.link_faces)
    shared_faces.intersection_update(curr_b.link_faces)
    if not shared_faces:
        return False

    return any(
        len(face.verts) == 4 and set(face.verts) == quad_verts
        for face in shared_faces
    )


def _strip_pair_score(current_loop, next_loop):
    if len(current_loop) != len(next_loop) or len(current_loop) < 2:
        return -1

    quad_score = 0
    for curr_a, curr_b, next_a, next_b in zip(
        current_loop,
        current_loop[1:],
        next_loop,
        next_loop[1:],
    ):
        if _has_quad_strip_face(curr_a, curr_b, next_a, next_b):
            quad_score += 1

    return quad_score


def _ordered_selected_vert_chain(bm):
    selected_verts = [vert for vert in bm.verts if vert.select]
    if len(selected_verts) < 2:
        return None

    selected_set = set(selected_verts)
    adjacency = {
        vert: [
            edge.other_vert(vert)
            for edge in _sorted_edges(vert)
            if edge.other_vert(vert) in selected_set
        ]
        for vert in selected_verts
    }

    if any(len(neighbors) > 2 for neighbors in adjacency.values()):
        return None

    visited = set()
    components = []
    for vert in sorted(selected_verts, key=lambda item: item.index):
        if vert in visited:
            continue
        stack = [vert]
        component = set()
        visited.add(vert)
        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in adjacency[current]:
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)
        if len(component) >= 2:
            components.append(component)

    if len(components) != 1:
        return None

    component = components[0]
    endpoints = [
        vert for vert in component
        if len([neighbor for neighbor in adjacency[vert] if neighbor in component]) == 1
    ]
    if len(endpoints) != 2:
        return None

    ordered = [min(endpoints, key=lambda vert: vert.index)]
    local_visited = {ordered[0]}
    current = ordered[0]
    while len(ordered) < len(component):
        next_vert = next(
            (
                neighbor for neighbor in adjacency[current]
                if neighbor in component and neighbor not in local_visited
            ),
            None,
        )
        if next_vert is None:
            return None
        ordered.append(next_vert)
        local_visited.add(next_vert)
        current = next_vert

    return ordered


def _is_ordered_open_chain(loop):
    if len(loop) < 2 or len(set(loop)) != len(loop):
        return False

    for vert_a, vert_b in zip(loop, loop[1:]):
        if _edge_between(vert_a, vert_b) is None:
            return False

    if len(loop) > 2 and _edge_between(loop[0], loop[-1]) is not None:
        return False

    return True


def _next_open_bridge_loop(current_loop, previous_loop=None):
    current_set = set(current_loop)
    previous_set = set(previous_loop or ())
    candidate_lists = []
    empty_count = 0

    for vert in current_loop:
        partners = []
        for edge in _sorted_edges(vert):
            other = edge.other_vert(vert)
            if other in current_set or other in previous_set:
                continue
            partners.append(other)

        if not partners:
            empty_count += 1
        candidate_lists.append(partners)

    if empty_count == len(current_loop):
        return []
    if empty_count != 0 or len(candidate_lists) != len(current_loop):
        return None

    ambiguous_indices = [
        index for index, partners in enumerate(candidate_lists)
        if len(partners) > 1
    ]
    combination_count = 1
    for partners in candidate_lists:
        combination_count *= max(1, len(partners))
    if combination_count > 256:
        return None

    option_ranges = [
        range(len(candidate_lists[index]))
        for index in ambiguous_indices
    ]
    best_loop = None
    best_score = -1
    for option_indices in product(*option_ranges):
        next_loop = []
        chosen_options = {
            candidate_index: option_index
            for candidate_index, option_index in zip(ambiguous_indices, option_indices)
        }
        for index, partners in enumerate(candidate_lists):
            partner_index = chosen_options.get(index, 0)
            next_loop.append(partners[partner_index])

        if len(set(next_loop)) != len(current_loop):
            continue
        if not _is_ordered_open_chain(next_loop):
            continue

        score = _strip_pair_score(current_loop, next_loop)
        if score > best_score:
            best_score = score
            best_loop = next_loop
        if score == len(current_loop) - 1:
            return next_loop

    return best_loop


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


def get_auto_open_bridged_chain(bm):
    start_loop = None
    raw_islands = get_selected_islands(bm)
    if len(raw_islands) == 1 and not raw_islands[0]['closed']:
        start_loop = raw_islands[0]['verts']
    else:
        start_loop = _ordered_selected_vert_chain(bm)

    if start_loop is None or len(start_loop) < 2 or not _is_ordered_open_chain(start_loop):
        return None

    aligned_loops = [start_loop]
    previous_loop = None
    current_loop = start_loop
    visited_loop_keys = {
        tuple(vert.index for vert in current_loop)
    }

    while True:
        next_loop = _next_open_bridge_loop(current_loop, previous_loop=previous_loop)
        if next_loop is None:
            return None
        if not next_loop:
            break

        loop_key = tuple(vert.index for vert in next_loop)
        if loop_key in visited_loop_keys:
            return None
        visited_loop_keys.add(loop_key)

        aligned_loops.append(next_loop)
        previous_loop = current_loop
        current_loop = next_loop

    if len(aligned_loops) < 2:
        return None

    return (aligned_loops, False)


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
