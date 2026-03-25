import bpy
import bmesh
from mathutils import Vector
from collections import defaultdict

from .utils import closest_point_on_segment, safe_edge_split_vert_only, edge_between, EPS, in_batch_mode

from .deselect_manager import get_or_create_session, commit_if_owned

DEBUG_VERBOSE = True
def _dbg(msg):
    if DEBUG_VERBOSE:
        try:
            print(f"[Super Fuse][T] {msg}")
        except Exception:
            pass

T_TOL = 1e-8
HARD_SKIP_L = TRUE = True
T_END_T_GUARD = 1e-6

def _prefilter_edges_by_aabb(edges, radius):
    r = float(max(radius, 0.0))
    boxed = []
    for e in edges:
        a = e.verts[0].co
        b = e.verts[1].co
        minx = min(a.x, b.x) - r
        maxx = max(a.x, b.x) + r
        miny = min(a.y, b.y) - r
        maxy = max(a.y, b.y) + r
        minz = min(a.z, b.z) - r
        maxz = max(a.z, b.z) + r
        boxed.append((e, minx, maxx, miny, maxy, minz, maxz))
    return boxed

def _edges_near_point(boxed_edges, p):
    for e, minx, maxx, miny, maxy, minz, maxz in boxed_edges:
        if (p.x >= minx and p.x <= maxx and
            p.y >= miny and p.y <= maxy and
            p.z >= minz and p.z <= maxz):
            yield e

class MESH_OT_super_fuse_t(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_t"
    bl_label = "Super Fuse: T"
    bl_options = {'REGISTER', 'UNDO'}

    def _scrub_select_history(self, bm):
        try:
            for elem in list(bm.select_history):
                if (elem is None) or (not getattr(elem, "is_valid", False)):
                    bm.select_history.remove(elem)
        except Exception:
            pass

    def execute(self, context):
        _dbg("==== Super Fuse T START ====")

        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Edit Mode required.")
            _dbg("FAIL: not in Edit Mode")
            return {'CANCELLED'}

        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a Mesh.")
            _dbg("FAIL: active object not mesh")
            return {'CANCELLED'}

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        props = getattr(context.scene, "super_fuse", None)
        sr = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        endpoint_guard = max(sr, 1e-12)
        _dbg(f"search_radius={sr:.9g} endpoint_guard={endpoint_guard:.9g}")

        tgt_edges = [e for e in bm.edges if e.select and not e.hide]
        _dbg(f"selected target edges: {len(tgt_edges)}")
        if not tgt_edges:
            self.report({'ERROR'}, "T: Select at least one edge to merge onto.")
            _dbg("FAIL: no target edges selected")
            return {'CANCELLED'}

        selected_verts = [v for v in bm.verts if v.select and not v.hide]
        src_verts = [v for v in selected_verts if v.link_edges]
        _dbg(f"selected verts: {len(selected_verts)} | connected sources: {len(src_verts)}")
        if not src_verts:
            self.report({'ERROR'}, "T: No connected vertices selected. Select vertices that belong to at least one edge.")
            _dbg("FAIL: no connected source verts")
            return {'CANCELLED'}

        boxed_edges = _prefilter_edges_by_aabb(tgt_edges, sr)
        _dbg(f"boxed edges prepared: {len(boxed_edges)}")

        decisions = []
        skipped_out_of_radius = 0
        skipped_only_own_edge = 0
        skipped_L_endpoint = 0
        total_candidates = 0

        for v in src_verts:
            p = v.co
            cand_list = []
            near_ct = 0

            for e in _edges_near_point(boxed_edges, p):
                near_ct += 1
                a = e.verts[0].co
                b = e.verts[1].co
                c, t = closest_point_on_segment(a, b, p)
                d = (p - c).length
                contains_vert = (v in e.verts)
                total_candidates += 1
                _dbg(f"  v#{getattr(v,'index','?')} cand edge#{getattr(e,'index','?')} d={d:.6g} t={float(t):.6g} contains={contains_vert}")

                if contains_vert:
                    continue
                if not (sr <= 0.0 or (d - sr) <= EPS):
                    continue

                if HARD_SKIP_L:
                    if (t <= T_END_T_GUARD) or (t >= 1.0 - T_END_T_GUARD):
                        _dbg(f"    -> SKIP (t-endpoint) t={float(t):.6g}")
                        skipped_L_endpoint += 1
                        continue
                    da = (c - a).length
                    db = (c - b).length
                    if min(da, db) <= endpoint_guard:
                        _dbg(f"    -> SKIP (endpoint_guard) min_end_dist={min(da,db):.6g} <= {endpoint_guard:.6g}")
                        skipped_L_endpoint += 1
                        continue

                cand_list.append((d, float(t), e))

            _dbg(f"  v#{getattr(v,'index','?')}: near_edges={near_ct}, accepted_candidates={len(cand_list)}")

            if not cand_list:
                owns_any = any(v in e.verts for e in tgt_edges)
                if owns_any:
                    skipped_only_own_edge += 1
                else:
                    skipped_out_of_radius += 1
                continue

            cand_list.sort(key=lambda k: k[0])
            dsel, tsel, esel = cand_list[0]
            _dbg(f"   -> PICK edge#{getattr(esel,'index','?')} d={dsel:.6g} t={tsel:.6g}")
            decisions.append({"v_src": v, "edge": esel, "t": tsel, "dist": dsel})

        _dbg(f"decisions: {len(decisions)} | skipped_out_of_radius={skipped_out_of_radius} | skipped_only_own_edge={skipped_only_own_edge} | skipped_L_endpoint={skipped_L_endpoint} | total_candidates={total_candidates}")

        if not decisions:
            self.report({'INFO'}, "T: No valid interior edge hits (L/endpoints skipped).")
            _dbg("==== Super Fuse T END (no decisions) ====")
            return {'CANCELLED'}

        jobs_per_edge = defaultdict(list)
        for d in decisions:
            e, t, v = d["edge"], d["t"], d["v_src"]
            jobs_per_edge[e].append((t, v))

        dedup_jobs = {}
        for e, jobs in jobs_per_edge.items():
            if not jobs:
                continue
            jobs.sort(key=lambda x: x[0])
            merged = []
            cluster_t = None
            cluster_vs = []
            for t, v in jobs:
                if cluster_t is None or abs(t - cluster_t) <= T_TOL:
                    cluster_t = t if cluster_t is None else (cluster_t + t) * 0.5
                    cluster_vs.append(v)
                else:
                    merged.append((cluster_t, list(cluster_vs)))
                    cluster_t, cluster_vs = t, [v]
            if cluster_t is not None:
                merged.append((cluster_t, list(cluster_vs)))
            merged.sort(key=lambda x: x[0], reverse=True)
            dedup_jobs[e] = merged
            _dbg(f"edge#{getattr(e,'index','?')} clusters={len(merged)}")
            for i, (ct, vs) in enumerate(merged):
                v_ids = [getattr(v, 'index', -1) for v in vs]
                _dbg(f"  cluster[{i}] t={ct:.9f} vs={v_ids}")

        targetmap = {}

        for edge, clusters in dedup_jobs.items():
            if not clusters:
                continue

            base_v0 = edge.verts[0]
            current_edge = edge
            current_max = 1.0
            last_t = None
            last_vert = None

            _dbg(f"Splitting edge#{getattr(edge,'index','?')} base_v0=#{getattr(base_v0,'index','?')} clusters={len(clusters)}")

            for t_orig, vlist in clusters:
                if last_t is not None and abs(t_orig - last_t) <= T_TOL and last_vert and last_vert.is_valid:
                    _dbg(f"  reuse last split vert v#{getattr(last_vert,'index','?')} for t={t_orig:.9f}")
                    for v_src in vlist:
                        if v_src is not last_vert:
                            targetmap[v_src] = last_vert
                    continue

                rel_t = 0.0 if current_max <= 0.0 else max(1.0e-12, min(1.0 - 1.0e-12, t_orig / current_max))
                _dbg(f"  split request: t_orig={t_orig:.9f} current_max={current_max:.9f} rel_t={rel_t:.9f} on edge#{getattr(current_edge,'index','?')} from base_v0=#{getattr(base_v0,'index','?')}")

                new_vert = safe_edge_split_vert_only(bm, current_edge, base_v0, rel_t)

                if new_vert is None or not getattr(new_vert, "is_valid", False):
                    end0, end1 = current_edge.verts
                    fallback = end0 if rel_t < 0.5 else end1
                    _dbg(f"    split FAILED -> fallback to endpoint v#{getattr(fallback,'index','?')}")
                    for v_src in vlist:
                        if v_src is not fallback:
                            targetmap[v_src] = fallback
                    last_t, last_vert = t_orig, fallback
                    current_max = t_orig
                else:
                    _dbg(f"    split OK -> new_vert v#{getattr(new_vert,'index','?')}")
                    for v_src in vlist:
                        if v_src is not new_vert:
                            targetmap[v_src] = new_vert
                    last_t, last_vert = t_orig, new_vert
                    current_max = t_orig
                    left_edge = edge_between(base_v0, new_vert)
                    _dbg(f"    edge_between base_v0 and new_vert -> {('edge#'+str(getattr(left_edge,'index','?'))) if left_edge else 'None'}")
                    if left_edge is None:
                        # Cannot advance further reliably on this edge chain
                        break
                    current_edge = left_edge

        if not targetmap:
            self._scrub_select_history(bm)
            self.report({'INFO'}, "T: Nothing to weld (empty targetmap after splits).")
            _dbg("==== Super Fuse T END (empty targetmap) ====")
            return {'CANCELLED'}

        bm.verts.index_update()
        _dbg("Forced BMesh index update before weld.")

        moved = 0
        for v_src, v_tgt in list(targetmap.items()):
            if v_src.is_valid and v_tgt.is_valid:
                v_src.co = Vector(v_tgt.co)
                moved += 1
        _dbg(f"pre-weld moved count: {moved}")

        # Validate targetmap (no self or invalid refs)
        valid_map = {s: t for s, t in targetmap.items()
                     if s and t and getattr(s, "is_valid", False) and getattr(t, "is_valid", False) and (s is not t)}

        if not valid_map:
            self._scrub_select_history(bm)
            if not in_batch_mode():
                bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, "T: Nothing to weld after validation.")
            return {'CANCELLED'}

        try:
            bmesh.ops.weld_verts(bm, targetmap=valid_map)
        except Exception as ex:
            self.report({'ERROR'}, f"T: Weld failed: {ex}")
            _dbg(f"WELD EXCEPTION: {ex}")
            return {'CANCELLED'}

        # Record T anchor targets for deselection (world space)
        session, owned_local = get_or_create_session()
        try:
            tgt_verts = list(set(valid_map.values()))
            session.add_from_verts('T', tgt_verts, obj, sr)
        except Exception:
            pass

        self._scrub_select_history(bm)
        # Apply deselection if local session; global executor will apply at end otherwise.
        commit_if_owned(context, session, owned_local)

        if not in_batch_mode():
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

        self.report({'INFO'}, f"T: merged {len(valid_map)} vertices.")
        _dbg(f"==== Super Fuse T END | merged={len(valid_map)} ====")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_t,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)