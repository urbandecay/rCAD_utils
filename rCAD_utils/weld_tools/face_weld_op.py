# face_weld_op.py

import bpy
import bmesh
from mathutils import Vector
from collections import defaultdict, deque

from .utils import (
    closest_point_on_segment,
    closest_points_on_segments,
    safe_edge_split_vert_only,
    edge_between,
    EPS,
)
from .deselect_manager import get_or_create_session, commit_if_owned


DEBUG_VERBOSE = False
def _dbg(msg):
    if DEBUG_VERBOSE:
        try:
            print(f"[Super Fuse][SQR] {msg}")
        except Exception:
            pass


T_TOL = 1e-8
T_END_T_GUARD = 1e-6


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


def _world_normal(obj, face):
    n = face.normal
    nw = obj.matrix_world.to_3x3() @ n
    L = nw.length
    if L > 1e-20:
        return nw / L
    return nw


def _dist_to_face_plane_world(obj, face, p_w):
    v0_w = obj.matrix_world @ face.verts[0].co
    n_w = _world_normal(obj, face)
    return abs((p_w - v0_w).dot(n_w))


def _project_to_face_plane_world(obj, face, p_w):
    v0_w = obj.matrix_world @ face.verts[0].co
    n_w = _world_normal(obj, face)
    return p_w - ((p_w - v0_w).dot(n_w)) * n_w


def _edge_loops_from_selected_edges(bm, selected_edges, exclude_faces_set):
    """
    Build closed loops from selected edges, excluding edges that belong to any face in exclude_faces_set.
    Returns list of dicts: {'verts': [BMVert...], 'edges': [BMEdge...], 'origin': 'edges'}
    """
    edges = [e for e in selected_edges if not any((f in exclude_faces_set) for f in e.link_faces)]
    if not edges:
        return []

    # Build adjacency restricted to these edges
    e_set = set(edges)
    v_edges = defaultdict(list)
    for e in edges:
        v0, v1 = e.verts
        v_edges[v0].append(e)
        v_edges[v1].append(e)

    # Connected components of edges
    visited_e = set()
    loops = []

    # A helper to walk a cycle given a start edge and start vertex
    def walk_cycle(start_edge, start_vert):
        path_edges = []
        path_verts = []

        curr_edge = start_edge
        curr_vert = start_vert
        prev_vert = None

        while True:
            path_edges.append(curr_edge)
            path_verts.append(curr_vert)
            # Choose the next vertex on curr_edge that isn't prev_vert
            v0, v1 = curr_edge.verts
            nxt_vert = v1 if v0 is curr_vert else v0
            if nxt_vert is prev_vert:
                # pick the other side
                nxt_vert = v0 if v1 is curr_vert else v1

            # Pick next edge from nxt_vert among selected edges excluding curr_edge and not yet consumed,
            # prefer degree-2 loop continuation
            next_edge = None
            for e2 in v_edges[nxt_vert]:
                if e2 in e_set and e2 is not curr_edge:
                    next_edge = e2
                    break

            prev_vert, curr_vert = curr_vert, nxt_vert
            if next_edge is None:
                # Dead end: not a closed loop
                return None, None
            if next_edge is start_edge and curr_vert is start_vert:
                # Closed
                path_verts.append(curr_vert)  # close loop
                return path_verts[:-1], path_edges  # return without duplicate last vert
            curr_edge = next_edge

    # Discover loops in components where all vertices have degree 2
    # Build components by BFS over edges
    # First, mark vertices' degree (within selected edges)
    deg = {v: len([e for e in v_edges[v] if e in e_set]) for v in v_edges.keys()}

    for e in edges:
        if e in visited_e:
            continue

        # BFS edges for this component
        comp_edges = []
        dq = deque([e])
        visited_e.add(e)
        while dq:
            ce = dq.popleft()
            comp_edges.append(ce)
            for v in ce.verts:
                for ne in v_edges[v]:
                    if ne in e_set and ne not in visited_e:
                        visited_e.add(ne)
                        dq.append(ne)

        # Check component's vertices degrees
        comp_verts = set()
        for ce in comp_edges:
            comp_verts.update(ce.verts)
        if not comp_verts:
            continue
        if not all(deg.get(v, 0) == 2 for v in comp_verts):
            # Not a pure closed loop component; skip
            continue

        # Walk cycles, mark edges as consumed
        consumed = set()
        for se in comp_edges:
            if se in consumed:
                continue
            # Choose an oriented start
            v0, v1 = se.verts
            loop_verts, loop_edges = walk_cycle(se, v0)
            if loop_verts and loop_edges:
                for le in loop_edges:
                    consumed.add(le)
                loops.append({'verts': loop_verts, 'edges': loop_edges, 'origin': 'edges'})

    return loops


def _loops_from_selected_faces(selected_faces):
    """
    Returns loops derived from selected faces' boundary cycles.
    [{'verts': [BMVert...], 'edges': [BMEdge...], 'origin': 'face', 'face': face}]
    """
    loops = []
    for f in selected_faces:
        vs = list(f.verts)
        es = list(f.edges)
        # Ensure verts are in loop order; BMFace.verts already is
        loops.append({'verts': vs, 'edges': es, 'origin': 'face', 'face': f})
    return loops


class MESH_OT_super_fuse_square(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_square"
    bl_label = "Super Fuse: Square (■)"
    bl_description = "Weld a closed polygon/face sitting on selected face(s): split nearby face edges and merge polygon's vertices to them"
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
        bm.faces.ensure_lookup_table()

        props = getattr(context.scene, "super_fuse", None)
        sr = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        sr = max(0.0, sr)

        # Selection pools
        selected_faces = [f for f in bm.faces if f.select and not f.hide]
        selected_edges = [e for e in bm.edges if e.select and not e.hide]
        selected_verts = [v for v in bm.verts if v.select and not v.hide]

        if not selected_faces:
            self.report({'ERROR'}, "Square: select at least one target face.")
            return {'CANCELLED'}

        if not selected_edges and not selected_verts and not selected_faces:
            self.report({'ERROR'}, "Square: select a closed polygon (face or edge loop) plus a target face.")
            return {'CANCELLED'}

        mw = obj.matrix_world.copy()
        mw_inv = mw.inverted_safe()

        # Build candidate source loops: from selected faces AND from selected edges not belonging to those faces
        sel_face_set = set(selected_faces)

        loops = []
        loops += _loops_from_selected_faces([f for f in selected_faces if f and f.is_valid])
        loops += _edge_loops_from_selected_edges(bm, selected_edges, sel_face_set)

        if not loops:
            self.report({'INFO'}, "Square: no closed loops detected in selection.")
            return {'CANCELLED'}

        # Prepare deselection session
        session, owned_local = get_or_create_session()

        # For each loop, find a target face whose plane is within sr for all loop verts and which doesn't share verts with the loop
        jobs = []
        loops_considered = 0
        for L in loops:
            loop_vs = [v for v in L['verts'] if v and v.is_valid]
            if len(loop_vs) < 3:
                continue

            # World positions
            Pws = [mw @ v.co for v in loop_vs]
            # Candidate faces (exclude same face if origin=face)
            candidates = []
            for tf in selected_faces:
                if L.get('origin') == 'face' and L.get('face') is tf:
                    continue
                # Ensure loop doesn't share verts with target face
                tf_verts = set(tf.verts)
                if any((v in tf_verts) for v in loop_vs):
                    continue
                # Check plane distance
                dists = [_dist_to_face_plane_world(obj, tf, p) for p in Pws]
                if max(dists) <= sr + 1.0e-12:
                    avgd = sum(dists) / float(len(dists))
                    candidates.append((avgd, tf))

            if not candidates:
                continue

            candidates.sort(key=lambda t: t[0])
            tf = candidates[0][1]

            # Record loop job
            jobs.append({'loop': L, 'target_face': tf})
            loops_considered += 1

        if not jobs:
            self.report({'INFO'}, "Square: no loop lying on a selected face within search radius.")
            return {'CANCELLED'}

        _dbg(f"jobs: {len(jobs)} (from {loops_considered} loops)")

        total_splits = 0
        targetmap = {}
        tgt_verts_used = set()

        # For mapping overlay verts to split targets later
        split_verts_by_edge = defaultdict(list)  # e -> list of (t_abs, v_split)

        for job in jobs:
            loop = job['loop']
            tf = job['target_face']
            loop_vs = [v for v in loop['verts'] if v and v.is_valid]

            # Project loop verts to face plane (world)
            loop_Pw = [mw @ v.co for v in loop_vs]
            loop_Pw_proj = [_project_to_face_plane_world(obj, tf, p) for p in loop_Pw]

            # Prepare target face edge tests
            tf_edges = list(tf.edges)
            boxed_tf_edges = _prefilter_edges_by_aabb_world(tf_edges, mw, sr)

            # Gather split requests per edge: from loop vertices and loop edges near edges
            per_edge_requests = defaultdict(list)  # e -> list of (t_abs, c_local)

            # 1) From loop vertices near edges
            for idx, v in enumerate(loop_vs):
                p_w = loop_Pw[idx]
                for e in _edges_near_point_world(boxed_tf_edges, p_w):
                    a_w = mw @ e.verts[0].co
                    b_w = mw @ e.verts[1].co
                    c_w, t = closest_point_on_segment(a_w, b_w, p_w)
                    d = (p_w - c_w).length
                    if sr <= 0.0 or (d - sr) <= EPS:
                        c_l = mw_inv @ c_w
                        per_edge_requests[e].append((float(t), c_l))

            # 2) From loop edges near face edges
            loop_edges_world = []
            if loop.get('origin') == 'face':
                vs = loop_vs
                n = len(vs)
                for i in range(n):
                    a = mw @ vs[i].co
                    b = mw @ vs[(i + 1) % n].co
                    loop_edges_world.append((a, b))
            else:
                # Origin edges: use provided list in loop dict if present, else build from ordered verts
                if loop.get('edges'):
                    # edges might not be ordered; still OK for proximity tests
                    for e in loop['edges']:
                        a = mw @ e.verts[0].co
                        b = mw @ e.verts[1].co
                        loop_edges_world.append((a, b))
                else:
                    vs = loop_vs
                    n = len(vs)
                    for i in range(n):
                        a = mw @ vs[i].co
                        b = mw @ vs[(i + 1) % n].co
                        loop_edges_world.append((a, b))

            r2 = sr * sr
            for e, minx, maxx, miny, maxy, minz, maxz in boxed_tf_edges:
                a_w = mw @ e.verts[0].co
                b_w = mw @ e.verts[1].co
                for (la, lb) in loop_edges_world:
                    # Quick AABB overlap test to reduce work
                    la_minx = min(la.x, lb.x) - sr; la_maxx = max(la.x, lb.x) + sr
                    la_miny = min(la.y, lb.y) - sr; la_maxy = max(la.y, lb.y) + sr
                    la_minz = min(la.z, lb.z) - sr; la_maxz = max(la.z, lb.z) + sr
                    if not (la_minx <= maxx and la_maxx >= minx and
                            la_miny <= maxy and la_maxy >= miny and
                            la_minz <= maxz and la_maxz >= minz):
                        continue

                    c1, c2, s, t = closest_points_on_segments(a_w, b_w, la, lb)
                    d2 = (c1 - c2).length_squared
                    if d2 <= r2 + 1e-24:
                        c_l = mw_inv @ c1
                        per_edge_requests[e].append((float(s), c_l))

            # Deduplicate and sort per edge; then split sequentially
            for e, items in per_edge_requests.items():
                if not e or not e.is_valid or not items:
                    continue

                # Cluster t by T_TOL and keep representative average; keep last c_l
                items.sort(key=lambda it: it[0])
                merged = []
                cluster_t = None
                cluster_cl = None
                cluster_ct = 0
                for t_abs, c_l in items:
                    if cluster_t is None or abs(t_abs - cluster_t) <= T_TOL:
                        if cluster_t is None:
                            cluster_t = float(t_abs)
                            cluster_cl = Vector(c_l)
                            cluster_ct = 1
                        else:
                            cluster_t = 0.5 * (cluster_t + float(t_abs))
                            cluster_cl = Vector(c_l)  # it's fine to just use last c_l
                            cluster_ct += 1
                    else:
                        merged.append((cluster_t, cluster_cl))
                        cluster_t = float(t_abs)
                        cluster_cl = Vector(c_l)
                        cluster_ct = 1
                if cluster_t is not None:
                    merged.append((cluster_t, cluster_cl))

                # Sequential splitting along edge
                base_v0 = e.verts[0]
                curr_edge = e
                curr_left = base_v0
                prev_t = 0.0

                merged.sort(key=lambda it: it[0])
                for (t_abs, c_local) in merged:
                    if not (curr_edge and curr_edge.is_valid):
                        break

                    # Guard endpoints
                    if (t_abs <= T_END_T_GUARD) or (t_abs >= 1.0 - T_END_T_GUARD):
                        # Do not split at exact endpoints; no need to add split vertex
                        continue

                    denom = max(1e-16, (1.0 - prev_t))
                    fac_local = (t_abs - prev_t) / denom
                    fac_local = max(1.0e-12, min(1.0 - 1.0e-12, fac_local))

                    new_vert = safe_edge_split_vert_only(bm, curr_edge, curr_left, float(fac_local))
                    if new_vert is None or not getattr(new_vert, "is_valid", False):
                        # Fallback: try to continue anyway
                        v0c, v1c = curr_edge.verts
                        curr_right = v1c if v0c is curr_left else v0c
                        nxt_edge = edge_between(curr_left, curr_right)
                        if nxt_edge is None:
                            break
                        curr_left = curr_left
                        curr_edge = nxt_edge
                        prev_t = t_abs
                        continue

                    # Snap precisely
                    try:
                        new_vert.co = Vector(c_local)
                    except Exception:
                        pass

                    split_verts_by_edge[e].append((float(t_abs), new_vert))
                    total_splits += 1

                    # Advance for subsequent splits
                    v0c, v1c = curr_edge.verts
                    curr_right = v1c if v0c is curr_left else v0c
                    nxt_edge = edge_between(new_vert, curr_right)
                    if nxt_edge is None:
                        # Try best continuation from the new vertex toward curr_right
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

            # Build mapping from loop vertices to nearest target vertices (split or existing) on target face edges
            # Collect candidate target vertices on face (existing vertices and newly created split verts)
            face_verts_pool = set(tf.verts[:])
            for e in tf.edges:
                for (_, vs) in split_verts_by_edge.get(e, []):
                    if vs and vs.is_valid:
                        face_verts_pool.add(vs)

            # Map each loop vertex
            for idx, v_src in enumerate(loop_vs):
                p_w = loop_Pw[idx]

                # Find closest face vertex
                best_v = None
                best_d = 1e30

                for vt in face_verts_pool:
                    d = (mw @ vt.co - p_w).length
                    if d < best_d:
                        best_d = d
                        best_v = vt

                if best_v and (sr <= 0.0 or best_d <= sr + 1e-12):
                    # Snap and map
                    v_src.co = Vector(best_v.co)
                    if v_src is not best_v:
                        targetmap[v_src] = best_v
                        tgt_verts_used.add(best_v)
                else:
                    # Fallback: find nearest edge on target face to project-and-split (if not already split nearby)
                    e_best = None
                    e_best_cw = None
                    e_best_t = None
                    e_best_d = 1e30
                    for e in tf.edges:
                        a_w = mw @ e.verts[0].co
                        b_w = mw @ e.verts[1].co
                        c_w, t = closest_point_on_segment(a_w, b_w, p_w)
                        d = (p_w - c_w).length
                        if d < e_best_d:
                            e_best = e
                            e_best_cw = c_w
                            e_best_t = float(t)
                            e_best_d = d

                    if e_best and (sr <= 0.0 or e_best_d <= sr + 1e-12):
                        # Ensure a split exists near this t
                        existing = split_verts_by_edge.get(e_best, [])
                        picked_v = None
                        if existing:
                            # Choose nearest split to c_w
                            best_sd = 1e30
                            for (_t, vv) in existing:
                                dw = (mw @ vv.co - e_best_cw).length
                                if dw < best_sd:
                                    best_sd = dw
                                    picked_v = vv
                        if picked_v is None:
                            # Create a split at this t
                            base_v0 = e_best.verts[0]
                            curr_edge = e_best
                            curr_left = base_v0
                            prev_t = 0.0
                            # Split at single t_abs
                            t_abs = e_best_t
                            if t_abs <= T_END_T_GUARD:
                                picked_v = e_best.verts[0]
                            elif t_abs >= 1.0 - T_END_T_GUARD:
                                picked_v = e_best.verts[1]
                            else:
                                denom = max(1e-16, (1.0 - prev_t))
                                fac_local = (t_abs - prev_t) / denom
                                fac_local = max(1.0e-12, min(1.0 - 1.0e-12, fac_local))
                                new_vert = safe_edge_split_vert_only(bm, curr_edge, curr_left, float(fac_local))
                                if new_vert and getattr(new_vert, "is_valid", False):
                                    new_vert.co = mw_inv @ e_best_cw
                                    split_verts_by_edge[e_best].append((t_abs, new_vert))
                                    picked_v = new_vert
                                    total_splits += 1
                        if picked_v:
                            v_src.co = Vector(picked_v.co)
                            if v_src is not picked_v:
                                targetmap[v_src] = picked_v
                                tgt_verts_used.add(picked_v)
                    else:
                        # Could not place this vertex; skip mapping (left as-is)
                        pass

        if not targetmap:
            self._scrub_select_history(bm)
            # Apply deselection if we own it; otherwise global executor will do it later.
            commit_if_owned(context, session, owned_local)
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, f"Square: splits={total_splits}, nothing to weld.")
            return {'CANCELLED'}

        # Validate and weld
        valid_map = {s: t for s, t in targetmap.items()
                     if s and t and getattr(s, "is_valid", False) and getattr(t, "is_valid", False) and (s is not t)}
        if not valid_map:
            self._scrub_select_history(bm)
            commit_if_owned(context, session, owned_local)
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            self.report({'INFO'}, f"Square: splits={total_splits}, nothing to weld after validation.")
            return {'CANCELLED'}

        try:
            bmesh.ops.weld_verts(bm, targetmap=valid_map)
        except Exception as ex:
            self.report({'ERROR'}, f"Square: weld failed: {ex}")
            return {'CANCELLED'}

        # Record deselection points for used targets
        try:
            session.add_from_verts('SQR', list(tgt_verts_used), obj, sr)
        except Exception:
            pass

        # Clean and apply deselection if local
        self._scrub_select_history(bm)
        commit_if_owned(context, session, owned_local)

        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
        self.report({'INFO'}, f"Square: splits={total_splits}, welded={len(valid_map)}.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_square,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
