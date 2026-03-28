# operators.py — Ported from PolyTangents (polytangents_4.py)
# Catmull-Rom spline resampling for verts, loops, and stacked geometry.

import bpy
import bmesh
import math
from mathutils import Vector, kdtree

from .ring_analyzer import analyze_rings
from .vert_deletion import find_safe_deletion_index, delete_at_index
from .topology_repair import repair_after_dissolve
from .seam_manager import (load_seam_homes, save_seam_homes,
                           match_seam_homes, migrate_drifted_seams)


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


# --- RING DETECTION FROM FULL SELECTION ---

def get_rings_from_selected(bm):
    """
    Detect cylinder ring pairs from a fully selected cylinder selection.
    Works for any number of cylinders selected at once — returns a list of
    ([ring0, ring1], True) tuples, one per cylinder pair.
    """
    sel_verts = [v for v in bm.verts if v.select]
    if len(sel_verts) < 4:
        return None
    sel_set = set(sel_verts)

    def fully_selected_faces(e):
        return sum(1 for f in e.link_faces if all(v in sel_set for v in f.verts))

    # Ring edges = edges between selected verts with exactly 1 fully-selected adjacent face
    ring_adj = {v: [] for v in sel_verts}
    for v in sel_verts:
        for e in v.link_edges:
            other = e.other_vert(v)
            if other in sel_set and fully_selected_faces(e) == 1:
                ring_adj[v].append(other)

    # Find connected components using ring edges only
    visited = set()
    rings = []
    for v in sel_verts:
        if v in visited or not ring_adj[v]:
            continue
        comp = []
        stack = [v]
        visited.add(v)
        while stack:
            curr = stack.pop()
            comp.append(curr)
            for nb in ring_adj[curr]:
                if nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
        if len(comp) >= 3:
            rings.append(comp)

    if len(rings) < 2 or len(rings) % 2 != 0:
        return None

    # Group rings into cylinder pairs by shaft edge connectivity
    ring_sets = [set(r) for r in rings]
    paired = [False] * len(rings)
    cylinder_pairs = []

    for i in range(len(rings)):
        if paired[i]:
            continue
        partner = None
        for j in range(i + 1, len(rings)):
            if paired[j]:
                continue
            for v in rings[i]:
                for e in v.link_edges:
                    if fully_selected_faces(e) == 2:
                        if e.other_vert(v) in ring_sets[j]:
                            partner = j
                            break
                if partner is not None:
                    break
            if partner is not None:
                break

        if partner is None:
            return None  # Unpaired ring — bail out

        # Order ring i as a closed loop
        ring0_set = ring_sets[i]
        ring0_ordered = [rings[i][0]]
        vis = {rings[i][0]}
        curr = rings[i][0]
        for _ in range(len(rings[i]) - 1):
            for nb in ring_adj[curr]:
                if nb in ring0_set and nb not in vis:
                    ring0_ordered.append(nb)
                    vis.add(nb)
                    curr = nb
                    break

        # Align partner ring via shaft edges
        ring1_set = ring_sets[partner]
        ring1_aligned = []
        for v0 in ring0_ordered:
            for e in v0.link_edges:
                if fully_selected_faces(e) == 2:
                    other = e.other_vert(v0)
                    if other in ring1_set:
                        ring1_aligned.append(other)
                        break

        if len(ring1_aligned) != len(ring0_ordered):
            return None

        cylinder_pairs.append(([ring0_ordered, ring1_aligned], True))
        paired[i] = True
        paired[partner] = True

    return cylinder_pairs if cylinder_pairs else None


# --- MULTI-STACK BRIDGED LOGIC ---

def get_auto_bridged_chain(bm):
    """
    When only ONE closed island is selected but its verts have unselected
    shaft edges going to an unselected parallel ring, auto-detect that
    partner ring and return it as a bridged pair so the shaft gets updated.
    """
    raw_islands = get_selected_islands(bm)
    if len(raw_islands) != 1:
        return None

    island = raw_islands[0]
    if not island['closed']:
        return None

    verts = island['verts']
    sel_verts_set = set(v for v in bm.verts if v.select)

    # For each vert, find one unselected edge going to an unselected vert
    shaft_map = {}  # island vert -> partner vert
    for v in verts:
        for e in v.link_edges:
            if not e.select:
                other = e.other_vert(v)
                if other not in sel_verts_set:
                    shaft_map[v] = other
                    break

    # Every ring vert must have a shaft partner
    if len(shaft_map) != len(verts):
        return None

    partner_set = set(shaft_map.values())

    # Partner verts must form a closed ring of the same size
    # Traverse partner verts using ANY edges between them
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

    # Align partner order to match the selected ring via shaft edges
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

        # Priority 0: Full cylinder selection — handles any number of holes at once
        rings_list = get_rings_from_selected(bm)
        print(f"[RCAD] rings_list: {rings_list}")
        if rings_list:
            for rings_data in rings_list:
                self.execute_bridged_logic(bm, obj, rings_data)
            return {'FINISHED'}

        # Priority 1: Solid/Bridged Loops (N-Stack Logic)
        bridged_data = get_bridged_chain(bm)
        print(f"[RCAD] bridged_data: {bridged_data}")
        if bridged_data and len(bridged_data[0]) >= 2:
            return self.execute_bridged_logic(bm, obj, bridged_data)

        # Priority 1b: Single ring with shaft edges (cylinder single-ring select)
        auto_bridged = get_auto_bridged_chain(bm)
        print(f"[RCAD] auto_bridged: {auto_bridged}")
        if auto_bridged and len(auto_bridged[0]) >= 2:
            return self.execute_bridged_logic(bm, obj, auto_bridged)

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

        # Analyze ring topology — identifies seam verts per ring
        ring_group = analyze_rings(loops, is_closed)

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

        # Vert deletion — picks safe (non-seam) verts, repairs topology after each
        if self.direction < 0:
            while ring_group.vert_count > target_count:
                if ring_group.vert_count <= min_limit:
                    break
                idx = find_safe_deletion_index(ring_group)
                if idx < 0:
                    break
                neighbor_pairs = delete_at_index(bm, ring_group, idx)
                repair_after_dissolve(bm, neighbor_pairs)

        # Seam home management
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

        # Apply coordinates — respace remaining verts on the curve
        bm.verts.ensure_lookup_table()
        for i, ring_info in enumerate(ring_group.rings):
            target_coords = new_coords_stack[i]
            loop = ring_info.verts
            min_len = min(len(loop), len(target_coords))
            for k in range(min_len):
                if loop[k].is_valid:
                    loop[k].co = target_coords[k]
                    loop[k].select = True

        # Seam migration — drift check + migrate
        if seam_homes:
            edge_lengths = []
            for ring_info in ring_group.rings:
                loop = ring_info.verts
                for k in range(len(loop) - 1):
                    if loop[k].is_valid and loop[k + 1].is_valid:
                        edge_lengths.append((loop[k].co - loop[k + 1].co).length)
            threshold = (sum(edge_lengths) / len(edge_lengths) * 0.5) if edge_lengths else 0.1
            migrate_drifted_seams(bm, ring_group, seam_homes, threshold)

        # Re-select shaft faces so hole-in-mesh cylinders don't end up
        # with unselected faces after face_split
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
