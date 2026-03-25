import bpy
import bmesh
from mathutils import Vector
from collections import defaultdict
from bpy.props import EnumProperty

from .utils import closest_point_on_segment, safe_edge_split_vert_only, edge_between, EPS, in_batch_mode
from .deselect_manager import get_or_create_session, commit_if_owned

T_TOL = 1e-8
DEBUG_VERBOSE = False

def _dbg(msg):
    if DEBUG_VERBOSE:
        try:
            print(f"[Super Fuse][VERT] {msg}")
        except Exception:
            pass

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

class MESH_OT_super_fuse_vert(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_vert"
    bl_label = "Super Fuse: Verts->Edges"
    bl_options = {'REGISTER', 'UNDO'}

    log_type: EnumProperty(
        name="Log Type",
        description="Internal mode tag (T=connected, Dot=loose). No logging performed.",
        items=[('T', "T", ""), ('Dot', "Dot", "")],
        default='Dot',
        options={'HIDDEN'},
    )

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

        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a Mesh.")
            return {'CANCELLED'}

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        props = getattr(context.scene, "super_fuse", None)
        sr = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        typ = 'T' if self.log_type == 'T' else 'Dot'

        tgt_edges = [e for e in bm.edges if e.select and not e.hide]
        if not tgt_edges:
            self.report({'ERROR'}, "Select at least one edge to merge onto.")
            return {'CANCELLED'}

        selected_verts = [v for v in bm.verts if v.select and not v.hide]
        if typ == 'T':
            src_verts = [v for v in selected_verts if v.link_edges]
            if not src_verts:
                self.report({'ERROR'}, "T: No connected vertices selected. Select vertices that belong to at least one edge.")
                return {'CANCELLED'}
        else:
            src_verts = [v for v in selected_verts if not v.link_edges]
            if not src_verts:
                self.report({'ERROR'}, "Verts->Edges (.): No loose vertices selected. Select vertices with no connected edges.")
                return {'CANCELLED'}

        _dbg("==== Super Fuse Verts->Edges START ====")
        _dbg(f"Mode tag: {typ} | search_radius: {sr:.6g}")
        _dbg(f"Selected edges: {len(tgt_edges)}")
        _dbg(f"Selected verts (filtered): {len(src_verts)}")

        boxed_edges = _prefilter_edges_by_aabb(tgt_edges, sr)

        decisions = []
        for v in src_verts:
            p = v.co
            candidates = []
            for e in _edges_near_point(boxed_edges, p):
                a, b = e.verts[0].co, e.verts[1].co
                c, t = closest_point_on_segment(a, b, p)
                d = (p - c).length
                contains_vert = (v in e.verts)
                candidates.append((d, float(t), e, contains_vert))

            if not candidates:
                continue

            candidates.sort(key=lambda k: k[0])

            picked = None
            for d, t, e, contains in candidates:
                if contains:
                    continue
                if sr <= 0.0 or (d - sr) <= EPS:
                    picked = (d, t, e, False)
                    break

            if picked is None:
                continue

            dsel, tsel, esel, _ = picked
            decisions.append({"v_src": v, "edge": esel, "t": tsel, "dist": dsel})

        if not decisions:
            msg = "T: No valid connected vertices near selected edges." if typ == 'T' else "No selected loose vertices within radius of selected edges."
            self.report({'INFO'}, msg)
            _dbg("==== Super Fuse Verts->Edges END (no decisions) ====")
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

        targetmap = {}

        for edge, clusters in dedup_jobs.items():
            if not clusters:
                continue

            base_v0 = edge.verts[0]
            current_edge = edge
            current_max = 1.0
            last_t = None
            last_vert = None

            for t_orig, vlist in clusters:
                if last_t is not None and abs(t_orig - last_t) <= T_TOL and last_vert and last_vert.is_valid:
                    for v_src in vlist:
                        if v_src is not last_vert:
                            targetmap[v_src] = last_vert
                    continue

                rel_t = 0.0 if current_max <= 0.0 else max(1.0e-12, min(1.0 - 1.0e-12, t_orig / current_max))
                new_vert = safe_edge_split_vert_only(bm, current_edge, base_v0, rel_t)

                if new_vert is None or not getattr(new_vert, "is_valid", False):
                    end0, end1 = current_edge.verts
                    fallback = end0 if rel_t < 0.5 else end1
                    for v_src in vlist:
                        if v_src is not fallback:
                            targetmap[v_src] = fallback
                    last_t, last_vert = t_orig, fallback
                    current_max = t_orig
                else:
                    for v_src in vlist:
                        if v_src is not new_vert:
                            targetmap[v_src] = new_vert
                    last_t, last_vert = t_orig, new_vert
                    current_max = t_orig
                    left_edge = edge_between(base_v0, new_vert)
                    if left_edge is None:
                        # Cannot continue splitting this chain reliably
                        break
                    current_edge = left_edge

        if not targetmap:
            self._scrub_select_history(bm)
            self.report({'INFO'}, "Nothing to merge.")
            _dbg("==== Super Fuse Verts->Edges END (empty targetmap) ====")
            return {'CANCELLED'}

        bm.verts.index_update()
        _dbg("Forced BMesh index update before welding.")

        for v_src, v_tgt in list(targetmap.items()):
            if v_src.is_valid and v_tgt.is_valid:
                v_src.co = Vector(v_tgt.co)

        # Validate and weld
        valid_map = {s: t for s, t in targetmap.items()
                     if s and t and getattr(s, "is_valid", False) and getattr(t, "is_valid", False) and (s is not t)}
        if not valid_map:
            self._scrub_select_history(bm)
            if not in_batch_mode():
                bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, "Nothing to weld after validation.")
            return {'CANCELLED'}

        try:
            bmesh.ops.weld_verts(bm, targetmap=valid_map)
        except Exception as ex:
            self.report({'ERROR'}, f"Weld failed: {ex}")
            return {'CANCELLED'}

        # Record targets for deselection (Dot or T mode)
        session, owned_local = get_or_create_session()
        try:
            tgt_verts = list(set(valid_map.values()))
            session.add_from_verts('T' if typ == 'T' else 'Dot', tgt_verts, obj, sr)
        except Exception:
            pass

        self._scrub_select_history(bm)
        # Apply deselection if local; otherwise the executor will apply at the end.
        commit_if_owned(context, session, owned_local)

        if not in_batch_mode():
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
        label = "T (Verts->Edges)" if typ == 'T' else "Verts->Edges (.)"
        self.report({'INFO'}, f"{label}: merged {len(valid_map)} vertices.")
        _dbg(f"==== Super Fuse Verts->Edges END | merged={len(valid_map)} ====")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_vert,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)