# detection_utils.py — Shared selection and chain helpers for resampler modes.

from mathutils import Vector, kdtree


def get_centroid(verts):
    if not verts:
        return Vector((0, 0, 0))
    return sum((v.co for v in verts), Vector((0, 0, 0))) / len(verts)


def _sorted_edges(edges):
    return sorted(edges, key=lambda edge: edge.index)


def get_selected_islands(bm):
    sel_verts = [v for v in bm.verts if v.select]
    if not sel_verts:
        return []
    visited = set()
    islands = []
    for v in sel_verts:
        if v in visited:
            continue
        component = []
        stack = [v]
        visited.add(v)
        while stack:
            curr = stack.pop()
            component.append(curr)
            for e in _sorted_edges(curr.link_edges):
                if not e.select:
                    continue
                other = e.other_vert(curr)
                if other not in visited and other.select:
                    visited.add(other)
                    stack.append(other)
        if len(component) < 2:
            continue
        endpoints = [
            cv for cv in component
            if len([e for e in cv.link_edges if e.select]) == 1
        ]
        ordered_verts = []
        closed = False
        if len(endpoints) == 2:
            curr = min(endpoints, key=lambda vert: vert.index)
            ordered_verts.append(curr)
            local_vis = {curr}
        elif len(endpoints) == 0:
            closed = True
            curr = min(component, key=lambda vert: vert.index)
            ordered_verts.append(curr)
            local_vis = {curr}
        else:
            continue
        while len(ordered_verts) < len(component):
            found = False
            for e in _sorted_edges(curr.link_edges):
                if not e.select:
                    continue
                other = e.other_vert(curr)
                if other not in local_vis and other in component:
                    local_vis.add(other)
                    ordered_verts.append(other)
                    curr = other
                    found = True
                    break
            if not found:
                break
        if len(ordered_verts) > 1:
            islands.append({'verts': ordered_verts, 'closed': closed})
    return islands


def get_sorted_verts_after_edit(verts, closed):
    if not verts:
        return []
    candidate_set = set(verts)
    start_v = min(verts, key=lambda vert: vert.index)

    if not closed:
        endpoints = []
        for vert in verts:
            neighbor_count = 0
            for e in vert.link_edges:
                other = e.other_vert(vert)
                if other in candidate_set:
                    neighbor_count += 1
            if neighbor_count <= 1:
                endpoints.append(vert)
        if endpoints:
            start_v = min(endpoints, key=lambda v: v.index)

    if not start_v.is_valid:
        return verts
    sorted_v = [start_v]
    visited = {start_v}
    current = start_v
    for _ in range(len(verts) - 1):
        found_next = False
        for e in _sorted_edges(current.link_edges):
            other = e.other_vert(current)
            if other in candidate_set and other not in visited:
                sorted_v.append(other)
                visited.add(other)
                current = other
                found_next = True
                break
        if not found_next:
            break
    return sorted_v


def order_loop(verts, closed):
    return get_sorted_verts_after_edit(verts, closed)


def get_anchored_chains(bm):
    sel_verts = [v for v in bm.verts if v.select]
    if not sel_verts:
        return []
    anchors = set()
    for v in sel_verts:
        if len(v.link_edges) > 2:
            anchors.add(v)
        else:
            for e in v.link_edges:
                if not e.select:
                    anchors.add(v)
                    break
    start_v = sel_verts[0]
    if anchors:
        for v in sel_verts:
            if v in anchors:
                start_v = v
                break
    visited_edges = set()
    chains = []
    if anchors:
        sorted_anchors = sorted(list(anchors), key=lambda v: v.index)
        for anchor in sorted_anchors:
            for e in anchor.link_edges:
                if e.select and e not in visited_edges:
                    visited_edges.add(e)
                    neighbor = e.other_vert(anchor)
                    sub_chain = [anchor, neighbor]
                    curr = neighbor
                    while True:
                        if curr in anchors:
                            break
                        found_next = False
                        for next_edge in curr.link_edges:
                            if next_edge.select and next_edge not in visited_edges:
                                other = next_edge.other_vert(curr)
                                if other in sel_verts:
                                    visited_edges.add(next_edge)
                                    sub_chain.append(other)
                                    curr = other
                                    found_next = True
                                    break
                        if not found_next:
                            break
                    chains.append({
                        'verts': sub_chain,
                        'closed': (sub_chain[0] == sub_chain[-1]),
                    })
    else:
        remaining = set(sel_verts)
        while remaining:
            start = next(iter(remaining))
            chain_verts = [start]
            curr = start
            while True:
                next_e = None
                for e in curr.link_edges:
                    if e.select and e not in visited_edges:
                        other = e.other_vert(curr)
                        if other in sel_verts:
                            next_e = e
                            next_v = other
                            break
                if next_e:
                    visited_edges.add(next_e)
                    chain_verts.append(next_v)
                    curr = next_v
                    if curr == start:
                        break
                else:
                    break
            chains.append({
                'verts': chain_verts,
                'closed': (chain_verts[0] == chain_verts[-1]),
            })
            for v in chain_verts:
                if v in remaining:
                    remaining.remove(v)
    return chains


def get_junction_chains(bm):
    sel_verts = [v for v in bm.verts if v.select]
    if not sel_verts:
        return []
    anchors = set()
    for v in sel_verts:
        sel_edges_count = len([e for e in v.link_edges if e.select])
        if sel_edges_count != 2:
            anchors.add(v)
    visited_edges = set()
    chains = []
    sorted_anchors = sorted(list(anchors), key=lambda v: v.index)
    for anchor in sorted_anchors:
        for e in anchor.link_edges:
            if not e.select:
                continue
            if e in visited_edges:
                continue
            visited_edges.add(e)
            chain_verts = [anchor]
            curr_v = e.other_vert(anchor)
            chain_verts.append(curr_v)
            while curr_v not in anchors:
                next_e = None
                for cand_e in curr_v.link_edges:
                    if cand_e.select and cand_e != e:
                        next_e = cand_e
                        break
                if next_e:
                    visited_edges.add(next_e)
                    e = next_e
                    curr_v = e.other_vert(curr_v)
                    chain_verts.append(curr_v)
                else:
                    break
            is_closed = (chain_verts[0] == chain_verts[-1])
            chains.append({'verts': chain_verts, 'closed': is_closed})
    return chains


def get_kissing_chains(bm, single_mode=False):
    sel_verts = [v for v in bm.verts if v.select]
    if not sel_verts:
        return []

    overlap_anchors = set()

    if single_mode:
        env_verts = [v for v in bm.verts if not v.select]
        if env_verts:
            kd = kdtree.KDTree(len(env_verts))
            for i, v in enumerate(env_verts):
                kd.insert(v.co, i)
            kd.balance()
            for v in sel_verts:
                matches = kd.find_range(v.co, 1e-4)
                if matches:
                    overlap_anchors.add(v)
    else:
        size = len(sel_verts)
        kd = kdtree.KDTree(size)
        for i, v in enumerate(sel_verts):
            kd.insert(v.co, i)
        kd.balance()

        visited_pairs = set()
        for v in sel_verts:
            for (co, index, dist) in kd.find_range(v.co, 1e-4):
                other = sel_verts[index]
                if other != v:
                    shared_edge = False
                    for e in v.link_edges:
                        if e.other_vert(v) == other:
                            shared_edge = True
                            break

                    if not shared_edge:
                        pair_key = tuple(sorted((v.index, other.index)))
                        if pair_key in visited_pairs:
                            continue
                        visited_pairs.add(pair_key)

                        midpoint = (v.co + other.co) * 0.5
                        v.co = midpoint
                        other.co = midpoint
                        overlap_anchors.add(v)
                        overlap_anchors.add(other)

    if not overlap_anchors:
        return None

    for v in sel_verts:
        sel_edges_count = len([e for e in v.link_edges if e.select])
        if sel_edges_count == 1:
            overlap_anchors.add(v)

    visited_edges = set()
    chains = []
    sorted_anchors = sorted(list(overlap_anchors), key=lambda v: v.index)

    for anchor in sorted_anchors:
        for e in anchor.link_edges:
            if e.select and e not in visited_edges:
                visited_edges.add(e)
                chain_verts = [anchor]
                curr_v = e.other_vert(anchor)
                chain_verts.append(curr_v)

                while True:
                    if curr_v in overlap_anchors:
                        break
                    next_e = None
                    for cand_e in curr_v.link_edges:
                        if cand_e.select and cand_e != e:
                            next_e = cand_e
                            break
                    if next_e:
                        visited_edges.add(next_e)
                        e = next_e
                        curr_v = e.other_vert(curr_v)
                        chain_verts.append(curr_v)
                    else:
                        break

                chains.append({
                    'verts': chain_verts,
                    'closed': (chain_verts[0] == chain_verts[-1]),
                })

    return chains


def check_if_anchored(bm):
    sel_verts = [v for v in bm.verts if v.select]
    for v in sel_verts:
        if len(v.link_edges) > 2:
            return True
        for e in v.link_edges:
            if not e.select:
                return True
    return False


def check_selected_junction(bm):
    for v in bm.verts:
        if not v.select:
            continue
        sel_count = len([e for e in v.link_edges if e.select])
        if sel_count > 2:
            return True
    return False


def align_islands_to_boss(islands):
    if len(islands) < 2:
        return
    boss = islands[0]
    boss_verts = boss['verts']
    if not boss['closed']:
        return
    boss_cent = get_centroid(boss_verts)
    boss_vec = (boss_verts[0].co - boss_cent).normalized()
    v0 = boss_verts[0].co
    v1 = boss_verts[1].co
    boss_cross = (v1 - v0).cross(boss_cent - v0)
    for i in range(1, len(islands)):
        island = islands[i]
        if not island['closed']:
            continue
        verts = island['verts']
        cent = get_centroid(verts)
        best_idx = 0
        max_dot = -2.0
        for k, v in enumerate(verts):
            vec = (v.co - cent).normalized()
            d = boss_vec.dot(vec)
            if d > max_dot:
                max_dot = d
                best_idx = k
        if best_idx != 0:
            verts[:] = verts[best_idx:] + verts[:best_idx]
        iv0 = verts[0].co
        iv1 = verts[1].co
        island_cross = (iv1 - iv0).cross(cent - iv0)
        if boss_cross.dot(island_cross) < 0:
            verts[:] = [verts[0]] + list(reversed(verts[1:]))
