import bpy
import bmesh
from mathutils import Vector

from .utils import (
    safe_edge_split_vert_only, edge_between, EPS, deferred_update_edit_mesh
)
from .x_weld_brute_force import find_brute_force_x_intersections
from .deselect_manager import get_or_create_session, commit_if_owned


class MESH_OT_super_fuse_x(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_x"
    bl_label = "Super Fuse: X"
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
        props = getattr(context.scene, "super_fuse", None)
        merge_dist = float(getattr(props, "search_radius", 1e-5)) if props else 1e-5
        merge_dist = max(0.0, merge_dist)

        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        mw = obj.matrix_world.copy()
        mw_inv = mw.inverted_safe()

        # Selected, visible edges only
        edges = [e for e in bm.edges if e.select and not e.hide]
        if len(edges) < 2:
            self.report({'WARNING'}, "Select at least two edges.")
            return {'CANCELLED'}

        # Build world-space segments and sid->edge map
        segments = []
        sid_to_edge = {}
        sid = 0
        for e in edges:
            v0 = e.verts[0]; v1 = e.verts[1]
            p0w = mw @ v0.co
            p1w = mw @ v1.co
            if (p0w - p1w).length_squared <= EPS:
                continue
            segments.append({'sid': sid, 'p0w': p0w, 'p1w': p1w})
            sid_to_edge[sid] = e
            sid += 1
        if len(segments) < 2:
            self.report({'WARNING'}, "Not enough valid edges for intersections.")
            return {'CANCELLED'}

        # Precise 3D X detection
        per_seg_cuts, all_keys, stats, key_world = find_brute_force_x_intersections(
            segments, merge_dist, plane=None
        )
        if not all_keys:
            self.report({'INFO'}, "No crossings found.")
            return {'CANCELLED'}

        key_to_edge_splits = {}
        matches_per_key = {}

        for s in segments:
            sid_i = s['sid']
            e = sid_to_edge.get(sid_i)
            if not e or not e.is_valid:
                continue
            cuts = per_seg_cuts.get(sid_i, [])
            if not cuts:
                continue

            seen = set()
            for t_abs, key in cuts:
                if key in seen:
                    continue
                seen.add(key)
                t01 = max(0.0, min(1.0, float(t_abs)))
                key_to_edge_splits.setdefault(key, []).append((e, t01))

        for key, lst in key_to_edge_splits.items():
            matches_per_key[key] = len(lst)

        # Create crossing verts for true Xs
        cv_by_key = {}
        for key, count in matches_per_key.items():
            if count < 2:
                continue
            hit_w = key_world.get(key)
            if hit_w is None:
                continue
            hit_l = mw_inv @ Vector(hit_w)
            v_cv = bm.verts.new(hit_l)
            cv_by_key[key] = v_cv
        bm.verts.index_update()
        bm.verts.ensure_lookup_table()

        if not cv_by_key:
            self._scrub_select_history(bm)
            deferred_update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, "X: No valid crossings with >=2 contributing edges.")
            return {'CANCELLED'}

        # Prepare deselection session (record X crossing world positions)
        session, owned_local = get_or_create_session()
        for key in cv_by_key.keys():
            hit_w = key_world.get(key)
            if hit_w is not None:
                session.add_world('X', Vector(hit_w), merge_dist)

        # Per-edge splits
        splits_by_edge = {}
        for key, pairs in key_to_edge_splits.items():
            if key not in cv_by_key:
                continue
            for (e, t_abs) in pairs:
                if not e or not e.is_valid:
                    continue
                if t_abs <= 1e-6 or t_abs >= 1.0 - 1e-6:
                    continue
                splits_by_edge.setdefault(e, []).append((float(t_abs), key))

        if not splits_by_edge:
            self._scrub_select_history(bm)
            # Commit deselection if we own the session locally (global will commit later)
            commit_if_owned(context, session, owned_local)
            deferred_update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, f"X: crossings={len(cv_by_key)}, splits=0, welded=0.")
            return {'CANCELLED'}

        split_count = 0
        weld_targetmap = {}
        KEY_POS_LOCAL = {k: (mw_inv @ Vector(key_world[k])) for k in cv_by_key.keys()}

        for edge, items in splits_by_edge.items():
            if not edge or not edge.is_valid:
                continue
            if not items:
                continue

            seen_keys = set()
            uniq = []
            for t_abs, key in items:
                if key in seen_keys:
                    continue
                seen_keys.add(key)
                uniq.append((t_abs, key))
            uniq.sort(key=lambda it: it[0])

            base_v0 = edge.verts[0]
            curr_edge = edge
            curr_left = base_v0
            prev_t = 0.0

            for (t_abs, key) in uniq:
                if not (curr_edge and curr_edge.is_valid):
                    break
                v_cv = cv_by_key.get(key)
                if not v_cv or not v_cv.is_valid:
                    continue

                denom = max(1e-16, (1.0 - prev_t))
                fac_local = (t_abs - prev_t) / denom
                fac_local = max(1.0e-12, min(1.0 - 1.0e-12, fac_local))

                new_vert = safe_edge_split_vert_only(bm, curr_edge, curr_left, float(fac_local))
                if new_vert is None or not getattr(new_vert, "is_valid", False):
                    try:
                        ret = bmesh.ops.subdivide_edges(
                            bm,
                            edges=[curr_edge],
                            cuts=1,
                            use_grid_fill=False,
                            edge_percents={curr_edge: fac_local}
                        )
                        cand = [ele for ele in ret.get("geom_split", []) if isinstance(ele, bmesh.types.BMVert)]
                        new_vert = cand[-1] if cand else None
                    except Exception:
                        new_vert = None
                    if new_vert is None or not getattr(new_vert, "is_valid", False):
                        continue

                p_loc = KEY_POS_LOCAL.get(key)
                if p_loc is not None:
                    new_vert.co = Vector(p_loc)

                if new_vert is not v_cv:
                    weld_targetmap[new_vert] = v_cv

                split_count += 1

                v0c, v1c = curr_edge.verts
                curr_right = v1c if v0c is curr_left else v0c
                nxt_edge = edge_between(new_vert, curr_right)
                if nxt_edge is None:
                    best_e = None
                    best_dot = -1e18
                    tgt = (curr_right.co - new_vert.co)
                    tgt_n = tgt.normalized() if tgt.length > 1e-12 else Vector((1, 0, 0))
                    for e2 in new_vert.link_edges:
                        other = e2.other_vert(new_vert)
                        d = other.co - new_vert.co
                        if d.length <= 1e-12:
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

        welded_count = 0
        if weld_targetmap:
            try:
                bmesh.ops.weld_verts(bm, targetmap=weld_targetmap)
                welded_count = len(weld_targetmap)
            except Exception:
                welded_count = 0

        self._scrub_select_history(bm)

        # Apply deselection if we own the session; otherwise, global executor will handle it.
        commit_if_owned(context, session, owned_local)

        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

        plane_info = ""
        try:
            plane_info = f" plane={stats.get('plane')}" if isinstance(stats, dict) else ""
        except Exception:
            pass
        self.report({'INFO'}, f"X: crossings={len(cv_by_key)}, splits={split_count}, welded={welded_count}.{plane_info}")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_x,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)