# operators.py — Ported from PolyTangents (polytangents_4.py)
# Catmull-Rom spline resampling for verts, loops, and stacked geometry.

import bpy
import bmesh
import math
from mathutils import Vector, kdtree


# =========================================================================
# --- MATH ENGINE ---
# =========================================================================

class CubicSegment:
    def __init__(self, p0, p1, p2, p3):
        t0 = 0.0
        t1 = self._get_t(t0, p0, p1)
        t2 = self._get_t(t1, p1, p2)
        t3 = self._get_t(t2, p2, p3)
        self.t_start = t1
        self.t_end = t2
        self.dt = t2 - t1

        if abs(self.dt) < 1e-6:
            self.d = p1
            self.is_degenerate = True
            self.times = (0, 1, 2, 3)
            return

        self.is_degenerate = False
        self.p0, self.p1, self.p2, self.p3 = p0, p1, p2, p3
        self.times = (t0, t1, t2, t3)

    def _get_t(self, t, p0, p1):
        dist_sq = (p1 - p0).length_squared
        return t + pow(dist_sq, 0.25)

    def eval(self, t_global):
        if self.is_degenerate:
            return self.d
        t0, t1, t2, t3 = self.times
        t = t_global

        if abs(t1 - t0) < 1e-6 or abs(t2 - t1) < 1e-6 or abs(t3 - t2) < 1e-6:
            return self.p1

        a1 = (t1 - t) / (t1 - t0) * self.p0 + (t - t0) / (t1 - t0) * self.p1
        a2 = (t2 - t) / (t2 - t1) * self.p1 + (t - t1) / (t2 - t1) * self.p2
        a3 = (t3 - t) / (t3 - t2) * self.p2 + (t - t2) / (t3 - t2) * self.p3

        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3

        return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2


class CatmullRomSpline:
    def __init__(self, points, is_closed=False):
        self.segments = []
        self.is_closed = is_closed
        clean_pts = []
        if len(points) > 0:
            clean_pts.append(points[0])
            for p in points[1:]:
                if (p - clean_pts[-1]).length_squared > 1e-12:
                    clean_pts.append(p)

        if is_closed and len(clean_pts) > 1:
            if (clean_pts[0] - clean_pts[-1]).length_squared < 1e-12:
                clean_pts.pop()

        if len(clean_pts) < 2:
            return

        if not is_closed:
            start_ghost = clean_pts[0] + (clean_pts[0] - clean_pts[1])
            end_ghost = clean_pts[-1] + (clean_pts[-1] - clean_pts[-2])
            padded_pts = [start_ghost] + clean_pts + [end_ghost]
        else:
            padded_pts = [clean_pts[-1]] + clean_pts + clean_pts[:2]

        for i in range(len(padded_pts) - 3):
            seg = CubicSegment(
                padded_pts[i], padded_pts[i + 1],
                padded_pts[i + 2], padded_pts[i + 3],
            )
            self.segments.append(seg)

    def eval_global(self, t):
        if not self.segments:
            return Vector((0, 0, 0))
        num = len(self.segments)

        if self.is_closed:
            t = t % num
        else:
            t = max(0.0, min(float(num), t))

        p = math.floor(t)
        if abs(t - num) < 1e-5 and not self.is_closed:
            idx = num - 1
            local_t = self.segments[idx].t_end
            return self.segments[idx].eval(local_t)

        frac = t - p
        idx = int(p) % num
        seg = self.segments[idx]
        local_t = seg.t_start + (frac * seg.dt)
        return seg.eval(local_t)

    def find_closest_t(self, co, resolution=100):
        best_t = 0.0
        min_dist = float('inf')
        num_segs = len(self.segments)
        for i in range(resolution + 1):
            t = (i / resolution) * num_segs
            pos = self.eval_global(t)
            d = (pos - co).length_squared
            if d < min_dist:
                min_dist = d
                best_t = t
        return best_t


# =========================================================================
# --- HELPER FUNCTIONS ---
# =========================================================================

def get_centroid(verts):
    if not verts:
        return Vector((0, 0, 0))
    return sum((v.co for v in verts), Vector((0, 0, 0))) / len(verts)


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
            for e in curr.link_edges:
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
            curr = endpoints[0]
            ordered_verts.append(curr)
            local_vis = {curr}
        elif len(endpoints) == 0:
            closed = True
            curr = component[0]
            ordered_verts.append(curr)
            local_vis = {curr}
        else:
            continue
        while len(ordered_verts) < len(component):
            found = False
            for e in curr.link_edges:
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
    start_v = verts[0]
    if not start_v.is_valid:
        return verts
    sorted_v = [start_v]
    visited = {start_v}
    current = start_v
    candidate_set = set(verts)
    for _ in range(len(verts) - 1):
        found_next = False
        for e in current.link_edges:
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


# --- ANCHORED LOGIC ---

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


# --- JUNCTION LOGIC ---

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


# --- KISSING LOGIC ---

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


# --- MULTI-STACK BRIDGED LOGIC ---

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
    visited = {start_node}
    curr = start_node

    while True:
        neighbors = adj[curr]
        next_node = None
        for n in neighbors:
            if n not in visited:
                next_node = n
                break
        if next_node is not None:
            ordered_indices.append(next_node)
            visited.add(next_node)
            curr = next_node
        else:
            break

    aligned_loops = []
    first_loop = island_map[ordered_indices[0]]['verts']
    aligned_loops.append(first_loop)

    for i in range(len(ordered_indices) - 1):
        idx_curr = ordered_indices[i]
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


# =========================================================================
# --- OPERATORS ---
# =========================================================================

class RCAD_OT_ResampleCurve(bpy.types.Operator):
    bl_idname = "rcad.resample_curve"
    bl_label = "Resample Curve"
    bl_options = {'REGISTER', 'UNDO'}

    direction: bpy.props.IntProperty(default=0)

    def execute(self, context):
        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()

        # Priority 0: Solid/Bridged Loops (N-Stack Logic)
        bridged_data = get_bridged_chain(bm)
        if bridged_data and len(bridged_data[0]) >= 2:
            return self.execute_bridged_logic(bm, obj, bridged_data)

        # Priority 1: Junctions
        if check_selected_junction(bm):
            return self.execute_anchored_logic(bm, obj, mode='JUNCTION')

        # Priority 2: Structural Anchors
        elif check_if_anchored(bm):
            return self.execute_anchored_logic(bm, obj, mode='ANCHORED')

        # Priority 3: Smart Kissing Logic
        else:
            islands = get_selected_islands(bm)
            is_single_mode = (len(islands) == 1)
            kissing_chains = get_kissing_chains(bm, single_mode=is_single_mode)

            if kissing_chains:
                return self.execute_anchored_logic(
                    bm, obj, mode='KISSING', precalc_chains=kissing_chains,
                )
            else:
                return self.execute_floating_logic(bm, obj)

    def execute_bridged_logic(self, bm, obj, data):
        loops, is_closed = data

        splines = []
        for loop in loops:
            pts = [v.co.copy() for v in loop]
            splines.append(CatmullRomSpline(pts, is_closed=is_closed))

        current_count = len(loops[0])
        target_count = current_count + self.direction
        min_limit = 3 if is_closed else 2
        if target_count < min_limit:
            target_count = min_limit

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

        # Topology mutation
        if self.direction > 0:
            while len(loops[0]) < target_count:
                search_range = (
                    len(loops[0]) if is_closed else len(loops[0]) - 1
                )
                max_len = -1.0
                best_idx = -1

                for k in range(search_range):
                    next_k = (k + 1) % len(loops[0])
                    total_dist = 0.0
                    for loop in loops:
                        d = (loop[k].co - loop[next_k].co).length
                        total_dist += d
                    avg_dist = total_dist / len(loops)
                    if avg_dist > max_len:
                        max_len = avg_dist
                        best_idx = k

                if best_idx != -1:
                    new_verts_col = []
                    for i in range(len(loops)):
                        loop = loops[i]
                        v1 = loop[best_idx]
                        v2 = loop[(best_idx + 1) % len(loop)]

                        e = bm.edges.get((v1, v2)) or bm.edges.get((v2, v1))
                        if not e:
                            for test_e in v1.link_edges:
                                if test_e.other_vert(v1) == v2:
                                    e = test_e
                                    break
                        if not e:
                            new_verts_col = None
                            break

                        res = bmesh.utils.edge_split(e, v1, 0.5)
                        new_v = (
                            res[0]
                            if isinstance(res[0], bmesh.types.BMVert)
                            else res[1]
                        )
                        loop.insert(best_idx + 1, new_v)
                        new_verts_col.append(new_v)

                    if new_verts_col:
                        for i in range(len(new_verts_col) - 1):
                            v_top = new_verts_col[i]
                            v_bot = new_verts_col[i + 1]
                            bmesh.ops.connect_verts(bm, verts=[v_top, v_bot])
                    else:
                        break

        elif self.direction < 0:
            while len(loops[0]) > target_count:
                if len(loops[0]) <= min_limit:
                    break
                v_kills = []
                for loop in loops:
                    v_kills.append(loop[1])

                for i in range(len(v_kills) - 1):
                    v_top = v_kills[i]
                    v_bot = v_kills[i + 1]
                    common_edge = (
                        bm.edges.get((v_top, v_bot))
                        or bm.edges.get((v_bot, v_top))
                    )
                    if common_edge:
                        bmesh.ops.dissolve_edges(bm, edges=[common_edge])

                for i, loop in enumerate(loops):
                    v_victim = v_kills[i]
                    v_target = loop[0]
                    if v_victim.is_valid and v_target.is_valid:
                        bmesh.ops.pointmerge(
                            bm, verts=[v_victim, v_target],
                            merge_co=v_target.co,
                        )
                    loop.pop(1)

        # Apply coordinates
        bm.verts.ensure_lookup_table()
        for i, loop in enumerate(loops):
            target_coords = new_coords_stack[i]
            min_len = min(len(loop), len(target_coords))
            for k in range(min_len):
                if loop[k].is_valid:
                    loop[k].co = target_coords[k]
                    loop[k].select = True

        bmesh.update_edit_mesh(obj.data)
        return {'FINISHED'}

    def execute_floating_logic(self, bm, obj):
        islands = get_selected_islands(bm)
        if not islands:
            return {'CANCELLED'}
        islands.sort(key=lambda x: get_centroid(x['verts']).z, reverse=True)
        align_islands_to_boss(islands)
        boss_island = islands[0]
        boss_verts = boss_island['verts']
        target_count = len(boss_verts) + self.direction
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

            self.apply_resample(bm, verts, new_co_list, closed)

        bmesh.update_edit_mesh(obj.data)
        return {'FINISHED'}

    def execute_anchored_logic(self, bm, obj, mode='ANCHORED',
                               precalc_chains=None):
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
            target_count = current_count + self.direction
            if target_count < 2:
                target_count = 2

            new_coords = []
            num_segs = len(spline.segments)
            for i in range(target_count):
                t = (i / (target_count - 1)) * num_segs
                new_coords.append(spline.eval_global(t))

            self.apply_resample(bm, verts, new_coords, False)

        bmesh.update_edit_mesh(obj.data)
        return {'FINISHED'}

    def apply_resample(self, bm, verts, new_coords, closed):
        target_count = len(new_coords)

        if self.direction > 0:
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

        elif self.direction < 0:
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


classes = (RCAD_OT_ResampleCurve,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
