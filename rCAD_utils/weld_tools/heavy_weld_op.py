# heavy_weld_op.py

import bpy
import bmesh
from mathutils import Vector
from collections import defaultdict

from .utils import (
    closest_point_on_segment,
    safe_edge_split_vert_only,
    edge_between,
    EPS,
)
from .deselect_manager import get_or_create_session, commit_if_owned


DEBUG_VERBOSE = False
def _dbg(msg):
    if DEBUG_VERBOSE:
        try:
            print(f"[Super Fuse][HVY] {msg}")
        except Exception:
            pass


T_TOL = 1e-8
T_END_T_GUARD = 1e-6


class _DSU:
    __slots__ = ("p", "r")
    def __init__(self, n):
        self.p = list(range(n))
        self.r = [0] * n
    def find(self, x):
        p = self.p
        while p[x] != x:
            p[x] = p[p[x]]
            x = p[x]
        return x
    def union(self, a, b):
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return False
        if self.r[ra] < self.r[rb]:
            self.p[ra] = rb
        elif self.r[ra] > self.r[rb]:
            self.p[rb] = ra
        else:
            self.p[rb] = ra
            self.r[ra] += 1
        return True


def _prefilter_edges_by_aabb_world(edges, mw, radius):
    r = float(max(radius, 0.0))
    boxed = []
    for e in edges:
        a = mw @ e.verts[0].co
        b = mw @ e.verts[1].co
        minx = min(a.x, b.x) - r
        maxx = max(a.x, b.x) + r
        miny = min(a.y, b.y) - r
        maxy = max(a.y, b.y) + r
        minz = min(a.z, b.z) - r
        maxz = max(a.z, b.z) + r
        boxed.append((e, minx, maxx, miny, maxy, minz, maxz))
    return boxed


def _edges_near_point_world(boxed_edges, p_w):
    for e, minx, maxx, miny, maxy, minz, maxz in boxed_edges:
        if (p_w.x >= minx and p_w.x <= maxx and
            p_w.y >= miny and p_w.y <= maxy and
            p_w.z >= minz and p_w.z <= maxz):
            yield e


class MESH_OT_super_fuse_heavy(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_heavy"
    bl_label = "Super Fuse: Heavy (..)"
    bl_description = "Heavy Weld: for each selected vertex, split ALL nearby selected edges and weld everything to a single central point"
    bl_options = {'REGISTER', 'UNDO'}

    def _scrub_select_history(self, bm):
        try:
            for elem in list(bm.select_history):
                if (elem is None) or (not getattr(elem, "is_valid", False)):
                    bm.select_history.remove(elem)
        except Exception:
            pass

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Edit Mode required.")
            return {'CANCELLED'}

        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a Mesh.")
            return {'CANCELLED'}

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        props = getattr(context.scene, "super_fuse", None)
        sr = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        sr = max(0.0, sr)

        # Selected, visible
        sel_verts = [v for v in bm.verts if v.select and not v.hide]
        sel_edges = [e for e in bm.edges if e.select and not e.hide]

        if not sel_verts:
            self.report({'ERROR'}, "Heavy Weld: select at least one vertex.")
            return {'CANCELLED'}
        if not sel_edges:
            self.report({'ERROR'}, "Heavy Weld: select at least one edge.")
            return {'CANCELLED'}

        mw = obj.matrix_world.copy()
        mw_inv = mw.inverted_safe()

        boxed_edges = _prefilter_edges_by_aabb_world(sel_edges, mw, sr)

        # For each vertex: find ALL nearby selected edges within sr
        v_records = []
        for v in sel_verts:
            p_w = mw @ v.co
            targets = []
            seen = set()
            for e in _edges_near_point_world(boxed_edges, p_w):
                a_w = mw @ e.verts[0].co
                b_w = mw @ e.verts[1].co
                c_w, t = closest_point_on_segment(a_w, b_w, p_w)
                d = (p_w - c_w).length
                if sr <= 0.0 or (d - sr) <= EPS:
                    # Dedup per edge
                    if e not in seen:
                        seen.add(e)
                        targets.append(e)
            if targets:
                # Job signature key based on edge object identities
                sig = tuple(sorted((id(e) for e in targets)))
                v_records.append({
                    "v": v,
                    "p_w": p_w,
                    "edges": targets,
                    "sig": sig,
                })

        if not v_records:
            self.report({'INFO'}, "Heavy Weld: no selected vertices within radius of selected edges.")
            return {'CANCELLED'}

        _dbg(f"candidate vertex records: {len(v_records)}")

        # Group by signature (same target edge set), then cluster by proximity
        groups_by_sig = defaultdict(list)
        for rec in v_records:
            groups_by_sig[rec["sig"]].append(rec)

        # Cluster tolerance: interpret "extremely close" as a fraction of sr, capped
        cluster_tol = max(1e-9, min(sr * 0.25, 1e-4))
        cluster_tol2 = cluster_tol * cluster_tol

        primitive_jobs = []
        for sig, lst in groups_by_sig.items():
            if not lst:
                continue
            m = len(lst)
            if m == 1:
                r = lst[0]
                primitive_jobs.append({
                    "src_verts": [r["v"]],
                    "edges": list(r["edges"]),
                    "center_w": Vector(r["p_w"]),
                })
                continue

            # DSU cluster by proximity in world space
            dsu = _DSU(m)
            for i in range(m - 1):
                pi = lst[i]["p_w"]
                for j in range(i + 1, m):
                    pj = lst[j]["p_w"]
                    if (pi - pj).length_squared <= cluster_tol2:
                        dsu.union(i, j)

            buckets = defaultdict(list)
            for i in range(m):
                buckets[dsu.find(i)].append(i)

            for root, idxs in buckets.items():
                src_vs = [lst[i]["v"] for i in idxs]
                pts = [lst[i]["p_w"] for i in idxs]
                edges = list(lst[idxs[0]]["edges"])  # same signature implies same edges
                center = Vector((0.0, 0.0, 0.0))
                for p in pts:
                    center += p
                center /= float(len(pts))
                primitive_jobs.append({
                    "src_verts": src_vs,
                    "edges": edges,
                    "center_w": center,
                })

        if not primitive_jobs:
            self.report({'INFO'}, "Heavy Weld: nothing to do after grouping.")
            return {'CANCELLED'}

        _dbg(f"primitive jobs: {len(primitive_jobs)}")

        # Unify primitive jobs that want essentially the same split point on any shared edge.
        # We do this by clustering per-edge target parameters and DSU-union their job indices.
        J = len(primitive_jobs)
        dsu_jobs = _DSU(J)

        # Prepare per-edge param buckets
        per_edge_hits = defaultdict(list)  # edge -> list of (t_abs, job_idx)
        for j, job in enumerate(primitive_jobs):
            W = job["center_w"]
            for e in job["edges"]:
                a_w = mw @ e.verts[0].co
                b_w = mw @ e.verts[1].co
                _c, t = closest_point_on_segment(a_w, b_w, W)
                per_edge_hits[e].append((float(t), j))

        # Union jobs whose t on the same edge are within T_TOL
        for e, items in per_edge_hits.items():
            if not items:
                continue
            items.sort(key=lambda it: it[0])
            cluster_start_idx = 0
            last_t = items[0][0]
            for k in range(1, len(items)):
                t_k, j_k = items[k]
                if abs(t_k - last_t) <= T_TOL:
                    # same cluster
                    dsu_jobs.union(items[cluster_start_idx][1], j_k)
                else:
                    # start new cluster
                    cluster_start_idx = k
                    last_t = t_k

        # Build super-jobs (merged)
        super_jobs_map = {}  # root -> dict
        for j in range(J):
            r = dsu_jobs.find(j)
            if r not in super_jobs_map:
                super_jobs_map[r] = {
                    "src_verts": [],
                    "edges": set(),
                    "centers": [],
                }
            super_jobs_map[r]["src_verts"].extend(primitive_jobs[j]["src_verts"])
            super_jobs_map[r]["edges"].update(primitive_jobs[j]["edges"])
            super_jobs_map[r]["centers"].append(Vector(primitive_jobs[j]["center_w"]))

        super_jobs = []
        for root, data in super_jobs_map.items():
            centers = data["centers"]
            csum = Vector((0.0, 0.0, 0.0))
            for p in centers:
                csum += p
            center_w = csum / float(len(centers))
            super_jobs.append({
                "src_verts": data["src_verts"],
                "edges": list(data["edges"]),
                "center_w": center_w,
            })

        if not super_jobs:
            self.report({'INFO'}, "Heavy Weld: no jobs after merging.")
            return {'CANCELLED'}

        _dbg(f"super jobs: {len(super_jobs)}")

        # Prepare deselection session
        session, owned_local = get_or_create_session()

        # Create anchors for each super-job and record deselection points
        anchors = []  # list of BMVert
        centers_w = []  # matching list of world positions
        for job in super_jobs:
            W = job["center_w"]
            L = mw_inv @ W
            v_anchor = bm.verts.new(L)
            anchors.append(v_anchor)
            centers_w.append(W)
            # Record for deselection (world space)
            session.add_world('HVY', Vector(W), sr)

        bm.verts.index_update()
        bm.verts.ensure_lookup_table()

        # Build per-edge split requests keyed by super-job index
        splits_by_edge = defaultdict(list)  # edge -> list of (t_abs, sj_idx, c_local)
        for sj_idx, job in enumerate(super_jobs):
            W = job["center_w"]
            for e in job["edges"]:
                a_w = mw @ e.verts[0].co
                b_w = mw @ e.verts[1].co
                c_w, t = closest_point_on_segment(a_w, b_w, W)
                c_l = mw_inv @ c_w
                splits_by_edge[e].append((float(t), sj_idx, c_l))

        # Perform splits per edge (sorted by t, ascending), mapping split verts to anchors
        targetmap = {}
        split_count = 0

        for e, items in splits_by_edge.items():
            if not items:
                continue

            # Dedup per super-job (if multiple identical entries occurred)
            per_job_best = {}
            for t, sj, c_l in items:
                if sj not in per_job_best or abs(t - per_job_best[sj][0]) > 0.0:
                    per_job_best[sj] = (float(t), c_l)
            uniq_items = [(t, sj, c_l) for sj, (t, c_l) in per_job_best.items()]
            uniq_items.sort(key=lambda it: it[0])

            base_v0 = e.verts[0]
            curr_edge = e
            curr_left = base_v0
            prev_t = 0.0

            for (t_abs, sj_idx, c_local) in uniq_items:
                if not (curr_edge and curr_edge.is_valid):
                    break

                v_anchor = anchors[sj_idx]
                if not (v_anchor and v_anchor.is_valid):
                    continue

                # Endpoint guard
                if (t_abs <= T_END_T_GUARD) or (t_abs >= 1.0 - T_END_T_GUARD):
                    v0c, v1c = curr_edge.verts
                    curr_right = v1c if v0c is curr_left else v0c
                    fallback = curr_left if t_abs <= 0.5 else curr_right
                    if fallback and fallback.is_valid and fallback is not v_anchor:
                        targetmap[fallback] = v_anchor
                    # Update for sequence
                    prev_t = t_abs
                    continue

                # Relative parameter for current fragment
                denom = max(1e-16, (1.0 - prev_t))
                fac_local = (t_abs - prev_t) / denom
                fac_local = max(1.0e-12, min(1.0 - 1.0e-12, fac_local))

                new_vert = safe_edge_split_vert_only(bm, curr_edge, curr_left, float(fac_local))
                if new_vert is None or not getattr(new_vert, "is_valid", False):
                    # Fallback to nearest endpoint
                    v0c, v1c = curr_edge.verts
                    curr_right = v1c if v0c is curr_left else v0c
                    fallback = v0c if fac_local < 0.5 else curr_right
                    if fallback and fallback.is_valid and fallback is not v_anchor:
                        targetmap[fallback] = v_anchor
                    prev_t = t_abs
                    continue

                # Force precise placement along edge (closest to W)
                try:
                    new_vert.co = Vector(c_local)
                except Exception:
                    pass

                if new_vert is not v_anchor:
                    targetmap[new_vert] = v_anchor
                split_count += 1

                # Advance along the edge chain
                v0c, v1c = curr_edge.verts
                curr_right = v1c if v0c is curr_left else v0c
                nxt_edge = edge_between(new_vert, curr_right)
                if nxt_edge is None:
                    # pick best aligned from new_vert
                    best_e = None
                    best_dot = -1e18
                    tgt = (curr_right.co - new_vert.co)
                    tgt_n = tgt.normalized() if tgt.length > 1.0e-12 else Vector((1, 0, 0))
                    for e2 in new_vert.link_edges:
                        other = e2.other_vert(new_vert)
                        d = other.co - new_vert.co
                        if d.length <= 1.0e-12:
                            continue
                        sc = d.normalized().dot(tgt_n)
                        if sc > best_dot:
                            best_dot = sc
                            best_e = e2
                    nxt_edge = best_e
                if nxt_edge is None:
                    break

                curr_left = new_vert
                curr_edge = nxt_edge
                prev_t = t_abs

        bm.verts.index_update()
        bm.verts.ensure_lookup_table()

        # Move and map source vertices to anchors
        for sj_idx, job in enumerate(super_jobs):
            v_anchor = anchors[sj_idx]
            if not (v_anchor and v_anchor.is_valid):
                continue
            anchor_co = Vector(v_anchor.co)
            for v_src in job["src_verts"]:
                if v_src and v_src.is_valid:
                    v_src.co = anchor_co
                    if v_src is not v_anchor:
                        targetmap[v_src] = v_anchor

        # Validate targetmap and weld
        valid_map = {s: t for s, t in targetmap.items()
                     if s and t and getattr(s, "is_valid", False) and getattr(t, "is_valid", False) and (s is not t)}

        if not valid_map:
            self._scrub_select_history(bm)
            # Apply deselection if we own it; otherwise global executor will apply later.
            commit_if_owned(context, session, owned_local)
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, f"Heavy Weld: splits={split_count}, nothing to weld.")
            return {'CANCELLED'}

        try:
            bmesh.ops.weld_verts(bm, targetmap=valid_map)
        except Exception as ex:
            self.report({'ERROR'}, f"Heavy Weld: weld failed: {ex}")
            return {'CANCELLED'}

        # Clean select history and apply deselection if local
        self._scrub_select_history(bm)
        commit_if_owned(context, session, owned_local)

        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
        self.report({'INFO'}, f"Heavy Weld: jobs={len(super_jobs)}, splits={split_count}, welded={len(valid_map)}.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_heavy,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
