# face_weld_op.py
# Uses knife_project to fuse smaller faces into larger coplanar faces.
# Handles multiple source faces on multiple targets in one operation.
# Also supports edge-on-face welding (edges cut into a target face).

import bpy
import bmesh
from mathutils import Vector
import mathutils
from collections import defaultdict

from .deselect_manager import get_or_create_session, commit_if_owned


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


def _face_centroid_key(f, mw):
    c = mw @ f.calc_center_median()
    return (round(c.x, 5), round(c.y, 5), round(c.z, 5))


def _find_view3d():
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    return area, space
    return None, None


def _align_view_to_face(space, center_world, normal_world, radius):
    r3d = space.region_3d
    try:
        q = (-normal_world).to_track_quat('-Z', 'Y')
    except ValueError:
        import mathutils
        q = mathutils.Quaternion()
    r3d.view_rotation = q
    r3d.view_location = center_world
    r3d.view_perspective = 'ORTHO'
    if hasattr(r3d, "ortho_scale"):
        r3d.ortho_scale = max(radius * 4.0, 0.1)


def _find_fuse_jobs(selected_faces, obj, mw, plane_tol):
    """Group selected faces by plane. Largest in each group = target, rest = sources."""
    groups = defaultdict(list)

    for f in selected_faces:
        n_w = _world_normal(obj, f)
        # Normalize direction so opposing normals hash to the same plane
        if (n_w.z < -1e-6 or
                (abs(n_w.z) <= 1e-6 and n_w.y < -1e-6) or
                (abs(n_w.z) <= 1e-6 and abs(n_w.y) <= 1e-6 and n_w.x < -1e-6)):
            n_w = -n_w
        key_n = (round(n_w.x, 3), round(n_w.y, 3), round(n_w.z, 3))
        v0_w = mw @ f.verts[0].co
        plane_d = round(v0_w.dot(n_w), 4)
        groups[(key_n, plane_d)].append(f)

    jobs = []
    for faces in groups.values():
        if len(faces) < 2:
            continue
        faces.sort(key=lambda f: f.calc_area(), reverse=True)
        target_face = faces[0]
        source_faces = faces[1:]
        jobs.append((target_face, source_faces))

    return jobs


class MESH_OT_super_fuse_square(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_square"
    bl_label = "Super Fuse: Square (■)"
    bl_description = "Fuse smaller faces into larger coplanar faces. Works on multiple faces at once."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Edit Mode required.")
            return {'CANCELLED'}

        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a Mesh.")
            return {'CANCELLED'}

        props = getattr(context.scene, "super_fuse", None)
        sr = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        weld_rad = max(sr, 1e-5)

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        selected_faces = [f for f in bm.faces if f.select and not f.hide]
        selected_edges = [e for e in bm.edges if e.select and not e.hide]

        # --- EDGE-ON-FACE PATH ---
        # If we have exactly 1 face and some edges not on that face, do edge-on-face weld
        if len(selected_faces) == 1 and selected_edges:
            target_face = selected_faces[0]
            face_edge_set = set(target_face.edges)
            source_edges = [e for e in selected_edges if e not in face_edge_set]
            if source_edges:
                return self._edge_on_face_weld(context, obj, bm, me,
                                               target_face, source_edges, weld_rad)

        if len(selected_faces) < 2:
            self.report({'ERROR'}, "Square: select at least two coplanar faces, "
                        "or one face + edges to weld onto it.")
            return {'CANCELLED'}

        mw = obj.matrix_world.copy()

        # Snapshot originally hidden faces by centroid (stable across deletions)
        originally_hidden_keys = {_face_centroid_key(f, mw) for f in bm.faces if f.hide}

        jobs = _find_fuse_jobs(selected_faces, obj, mw, max(sr, 1e-6))
        if not jobs:
            self.report({'ERROR'}, "Square: no coplanar face pairs found in selection.")
            return {'CANCELLED'}

        # Capture view state once
        area, space = _find_view3d()
        orig_rot = orig_loc = orig_persp = None
        if space:
            r3d = space.region_3d
            orig_rot = r3d.view_rotation.copy()
            orig_loc = r3d.view_location.copy()
            orig_persp = r3d.view_perspective

        total_fused = 0
        cutter_obj = None
        cutter_mesh = None

        try:
            # Strategy 3: Batch-collect all job data, then batch-delete
            # all source faces in ONE pass instead of N separate deletes.
            job_data = []
            all_source_faces = []
            for (target_face, source_faces) in jobs:
                arc_coords = []
                edge_pairs_world = []
                for sf in source_faces:
                    arc_coords.extend(v.co.copy() for v in sf.verts)
                    edge_pairs_world.extend(
                        (mw @ e.verts[0].co, mw @ e.verts[1].co) for e in sf.edges
                    )

                face_center_world = mw @ target_face.calc_center_median()
                face_normal_world = _world_normal(obj, target_face)
                face_radius = max((mw @ v.co - face_center_world).length for v in target_face.verts)
                face_radius = max(face_radius, 0.1)

                job_data.append((target_face, source_faces, arc_coords,
                                 edge_pairs_world, face_center_world,
                                 face_normal_world, face_radius))
                all_source_faces.extend(source_faces)

            # Batch delete all source faces at once (single update)
            if all_source_faces:
                bm = bmesh.from_edit_mesh(me)
                bmesh.ops.delete(bm, geom=all_source_faces, context='FACES_ONLY')
                bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

            for (target_face, source_faces, all_arc_coords,
                 all_edge_pairs_world, face_center_world,
                 face_normal_world, face_radius) in job_data:

                # Build combined cutter from all source edges
                cutter_mesh = bpy.data.meshes.new("__face_weld_cutter__")
                bm_cut = bmesh.new()
                added = {}
                for (a_w, b_w) in all_edge_pairs_world:
                    key_a = (round(a_w.x, 8), round(a_w.y, 8), round(a_w.z, 8))
                    key_b = (round(b_w.x, 8), round(b_w.y, 8), round(b_w.z, 8))
                    if key_a not in added:
                        added[key_a] = bm_cut.verts.new(a_w)
                    if key_b not in added:
                        added[key_b] = bm_cut.verts.new(b_w)
                    try:
                        bm_cut.edges.new([added[key_a], added[key_b]])
                    except Exception:
                        pass
                bm_cut.to_mesh(cutter_mesh)
                bm_cut.free()
                cutter_mesh.update()

                cutter_obj = bpy.data.objects.new("__face_weld_cutter__", cutter_mesh)
                context.collection.objects.link(cutter_obj)

                # Show only the target face
                bm = bmesh.from_edit_mesh(me)
                bm.faces.ensure_lookup_table()
                bpy.ops.mesh.select_all(action='DESELECT')
                for f in bm.faces:
                    if f is target_face:
                        f.hide = False
                        f.select = True
                    else:
                        f.hide = True
                bmesh.update_edit_mesh(me)

                # Align view and knife project
                if space:
                    _align_view_to_face(space, face_center_world, face_normal_world, face_radius)
                    try:
                        bpy.ops.wm.redraw_timer(type='DRAW_WIN', iterations=1)
                    except Exception:
                        pass
                    context.view_layer.update()

                cutter_obj.select_set(True)
                obj.select_set(True)
                context.view_layer.objects.active = obj
                bpy.ops.mesh.knife_project(cut_through=True)

                # Teleport cut verts to ideal positions
                bm_final = bmesh.from_edit_mesh(me)
                bm_final.verts.ensure_lookup_table()
                search_rad_sq = (weld_rad * 2.5) ** 2
                for v in bm_final.verts:
                    if v.hide:
                        continue
                    if v.select:
                        for co in all_arc_coords:
                            if (v.co - co).length_squared < search_rad_sq:
                                v.co = co.copy()
                                break
                    else:
                        for co in all_arc_coords:
                            if (v.co - co).length_squared < 1e-8:
                                v.select = True
                                break
                bmesh.update_edit_mesh(me)
                bpy.ops.mesh.remove_doubles(threshold=weld_rad)

                # Unhide all for next job
                bm = bmesh.from_edit_mesh(me)
                for f in bm.faces:
                    f.hide = False
                    f.select = False
                    for v in f.verts:
                        v.hide = False
                    for e in f.edges:
                        e.hide = False
                bmesh.update_edit_mesh(me)

                # Clean up cutter
                bpy.data.objects.remove(cutter_obj, do_unlink=True)
                bpy.data.meshes.remove(cutter_mesh)
                cutter_obj = None
                cutter_mesh = None

                total_fused += len(source_faces)

            # Restore originally hidden faces
            bm = bmesh.from_edit_mesh(me)
            for f in bm.faces:
                if _face_centroid_key(f, mw) in originally_hidden_keys:
                    f.hide = True
            bmesh.update_edit_mesh(me)


            # Restore view
            if space and orig_rot:
                space.region_3d.view_rotation = orig_rot
                space.region_3d.view_location = orig_loc
                space.region_3d.view_perspective = orig_persp

        except Exception as ex:
            self.report({'ERROR'}, f"Square: failed: {ex}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

        finally:
            if cutter_obj and cutter_obj.name in bpy.data.objects:
                bpy.data.objects.remove(cutter_obj, do_unlink=True)
            if cutter_mesh and cutter_mesh.name in bpy.data.meshes:
                bpy.data.meshes.remove(cutter_mesh)

        self.report({'INFO'}, f"Square: fused {total_fused} face(s).")
        return {'FINISHED'}

    # ------------------------------------------------------------------
    # EDGE-ON-FACE WELD  (ported from radCAD arc_weld_manager)
    # ------------------------------------------------------------------
    def _edge_on_face_weld(self, context, obj, bm, me, target_face, source_edges, weld_rad):
        """Weld selected edges onto a selected target face.

        Three-phase approach matching the drawing tools:
          Phase 0 – Integrate source edge verts into the target face boundary
                    by splitting boundary edges at the nearest point.  This is
                    what the drawing tool's perform_heavy_weld does — without
                    it, verts float near the face but aren't topologically part
                    of it, so face_split can't find them.
          Phase 1 – Direct face_split for edges whose verts both sit on
                    the face boundary but the edge itself isn't part of it.
          Phase 2 – Knife-project for edges that cross the face interior
                    (verts not yet on the boundary).
        """
        DEBUG = True  # flip to False once diagonal welding is solid

        mw = obj.matrix_world.copy()

        # Snapshot hidden faces by centroid so we can restore later
        originally_hidden_keys = {_face_centroid_key(f, mw) for f in bm.faces if f.hide}

        # Collect ideal coords BEFORE any topology changes
        arc_coords = [v.co.copy() for v in
                      {v for e in source_edges for v in e.verts}]

        # ------ PHASE 0: INTEGRATE VERTS INTO FACE BOUNDARY ------
        # The drawing tool's pre-weld splits boundary edges so that source
        # edge endpoints become actual verts OF the target face.  Without
        # this step, face_split fails because the verts aren't in the face's
        # vertex loop — even if they're geometrically sitting right on top
        # of a boundary edge.
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        source_verts = list({v for e in source_edges for v in e.verts})

        if DEBUG:
            print(f"\n{'='*60}")
            print(f"[FACE_WELD] _edge_on_face_weld START")
            print(f"  target_face index={target_face.index}, "
                  f"verts={[v.index for v in target_face.verts]}")
            print(f"  source_edges: {len(source_edges)}")
            for e in source_edges:
                if e.is_valid:
                    print(f"    edge {e.index}: v{e.verts[0].index}"
                          f"({e.verts[0].co!s}) -> v{e.verts[1].index}"
                          f"({e.verts[1].co!s})")
            print(f"  source_verts: {[v.index for v in source_verts]}")
            for sv in source_verts:
                in_face = sv in target_face.verts
                print(f"    v{sv.index} co={sv.co!s}  in_target_face={in_face}  "
                      f"link_faces={[f.index for f in sv.link_faces]}")

        integrate_count = 0
        for sv in source_verts:
            if not sv.is_valid:
                continue
            if sv in target_face.verts:
                if DEBUG:
                    print(f"  [P0] v{sv.index} already in target face — skip")
                continue  # already part of the face

            # Find the closest boundary edge of the target face to this vert
            best_edge = None
            best_dist = float('inf')
            best_proj = None
            for be in target_face.edges:
                a, b = be.verts[0].co, be.verts[1].co
                ab = b - a
                ab_len_sq = ab.length_squared
                if ab_len_sq < 1e-12:
                    continue
                t = max(0.0, min(1.0, (sv.co - a).dot(ab) / ab_len_sq))
                proj = a + ab * t
                d = (sv.co - proj).length
                if d < best_dist:
                    best_dist = d
                    best_edge = be
                    best_proj = proj

            if DEBUG:
                if best_edge:
                    print(f"  [P0] v{sv.index}: closest boundary edge={best_edge.index} "
                          f"(v{best_edge.verts[0].index}-v{best_edge.verts[1].index}), "
                          f"dist={best_dist:.6f}, proj={best_proj!s}")
                else:
                    print(f"  [P0] v{sv.index}: NO boundary edge found!")

            # If vert is close to a boundary edge, split that edge and merge
            if best_edge and best_dist < weld_rad * 5.0:
                # Check if vert is near one of the edge endpoints — just merge
                for ev in best_edge.verts:
                    if (sv.co - ev.co).length < weld_rad * 2.0:
                        if DEBUG:
                            print(f"  [P0] v{sv.index} near endpoint v{ev.index} "
                                  f"(dist={( sv.co - ev.co).length:.6f}) — "
                                  f"merging via remove_doubles")
                        # Merge sv into the existing boundary vert
                        bmesh.ops.pointmerge(bm, verts=[sv, ev], merge_co=ev.co)
                        integrate_count += 1
                        best_edge = None  # skip the split
                        break

                if best_edge and best_edge.is_valid:
                    # Split the boundary edge at the projection point to
                    # insert our vert into the face's vertex loop
                    a, b = best_edge.verts[0].co, best_edge.verts[1].co
                    ab = b - a
                    ab_len_sq = ab.length_squared
                    t = (sv.co - a).dot(ab) / ab_len_sq if ab_len_sq > 1e-12 else 0.5
                    t = max(0.01, min(0.99, t))

                    if DEBUG:
                        print(f"  [P0] Splitting edge {best_edge.index} at t={t:.4f} "
                              f"for v{sv.index}")

                    try:
                        new_edge, new_vert = bmesh.utils.edge_split(best_edge, best_edge.verts[0], t)
                        # Now merge the new split vert with our source vert
                        if DEBUG:
                            print(f"  [P0] Split created new_vert={new_vert.index} "
                                  f"at {new_vert.co!s}, merging with v{sv.index}")
                        bmesh.ops.pointmerge(bm, verts=[sv, new_vert], merge_co=sv.co)
                        integrate_count += 1
                    except Exception as ex:
                        if DEBUG:
                            print(f"  [P0] edge_split FAILED: {ex}")
            elif DEBUG:
                print(f"  [P0] v{sv.index} too far from boundary "
                      f"(dist={best_dist:.6f} > threshold={weld_rad * 5.0:.6f})")

            # Refresh lookup tables after topology changes
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()

        if integrate_count:
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
            bm = bmesh.from_edit_mesh(me)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()

        if DEBUG:
            print(f"  [P0] Integrated {integrate_count} verts into face boundary")

        # Re-collect source edges after topology changes — find edges
        # connecting our arc_coords that are selected
        source_edges_new = []
        for e in bm.edges:
            if not e.is_valid or not e.select or e.hide:
                continue
            v1_match = any((e.verts[0].co - ac).length < weld_rad * 3 for ac in arc_coords)
            v2_match = any((e.verts[1].co - ac).length < weld_rad * 3 for ac in arc_coords)
            if v1_match and v2_match:
                source_edges_new.append(e)
        source_edges = source_edges_new

        # Also re-find the target face (it may have been split by Phase 0)
        target_face_candidates = []
        for f in bm.faces:
            if f.hide:
                continue
            fn = _world_normal(obj, f)
            fc = mw @ f.calc_center_median()
            orig_n = _world_normal(obj, target_face) if target_face.is_valid else Vector((0, 0, 1))
            orig_c = mw @ target_face.calc_center_median() if target_face.is_valid else Vector()
            if abs(fn.dot(orig_n)) > 0.99:
                if abs(fc.dot(orig_n) - orig_c.dot(orig_n)) < 0.001:
                    target_face_candidates.append(f)

        if DEBUG:
            print(f"  [P0] Re-found {len(source_edges)} source edges, "
                  f"{len(target_face_candidates)} target face candidates")
            for e in source_edges:
                v1, v2 = e.verts
                v1_faces = [f.index for f in v1.link_faces]
                v2_faces = [f.index for f in v2.link_faces]
                common = set(v1.link_faces) & set(v2.link_faces)
                print(f"    edge {e.index}: v{v1.index} faces={v1_faces}, "
                      f"v{v2.index} faces={v2_faces}, common={[f.index for f in common]}")

        # ------ PHASE 1: DIRECT FACE SPLIT ------
        face_splits = 0
        changed = True
        while changed:
            changed = False
            for e in list(source_edges):
                if not e.is_valid:
                    continue
                v1, v2 = e.verts
                if not v1.is_valid or not v2.is_valid:
                    continue
                # Faces that contain BOTH verts but NOT this edge
                common_faces = set(v1.link_faces) & set(v2.link_faces)
                if DEBUG:
                    print(f"  [P1] edge {e.index}: v{v1.index}-v{v2.index}, "
                          f"common_faces={[f.index for f in common_faces]}")
                for f in common_faces:
                    if e in f.edges:
                        if DEBUG:
                            print(f"    [P1] face {f.index} — edge already on boundary, skip")
                        continue          # already on boundary
                    if f.hide:
                        continue
                    if DEBUG:
                        print(f"    [P1] Attempting face_split on face {f.index} "
                              f"(verts={[v.index for v in f.verts]})")
                    split_ok = False
                    # Use use_exist=True — matching the drawing tool approach.
                    # Phase 0 now ensures verts are properly in the face, so
                    # the ghost-edge problem from before shouldn't happen.
                    try:
                        bmesh.utils.face_split(f, v1, v2, use_exist=True)
                        split_ok = True
                        if DEBUG:
                            print(f"    [P1] face_split SUCCESS")
                    except Exception as ex:
                        if DEBUG:
                            print(f"    [P1] face_split FAILED: {ex}")
                    if not split_ok:
                        try:
                            res = bmesh.ops.connect_vert_pair(
                                bm, verts=[v1, v2])
                            if res and res.get('edges'):
                                split_ok = True
                                if DEBUG:
                                    print(f"    [P1] connect_vert_pair SUCCESS")
                            elif DEBUG:
                                print(f"    [P1] connect_vert_pair — no edges returned")
                        except Exception as ex:
                            if DEBUG:
                                print(f"    [P1] connect_vert_pair FAILED: {ex}")
                    if split_ok:
                        face_splits += 1
                        changed = True
                if changed:
                    bm.faces.ensure_lookup_table()
                    break                 # restart after topology change

        if face_splits:
            bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

        # ------ PHASE 2: KNIFE PROJECT for remaining edges ------
        # Re-fetch bmesh after topology changes
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        # Find edges still selected that aren't yet part of any face boundary
        # around the target.  We identify them by checking if both verts share
        # a common face with each other (if face_split already handled them,
        # the edge is now on the boundary and we skip it).
        knife_edges = []
        for e in bm.edges:
            if not e.is_valid or not e.select or e.hide:
                continue
            # Skip edges that are already part of a face boundary
            if any(True for f in e.link_faces if not f.hide):
                # Edge is on a face boundary already — might have been split in phase 1
                continue
            knife_edges.append(e)

        if DEBUG:
            print(f"\n  [P2] Phase 1 did {face_splits} split(s)")
            print(f"  [P2] Remaining knife_edges: {len(knife_edges)}")
            for e in knife_edges:
                print(f"    edge {e.index}: v{e.verts[0].index}-v{e.verts[1].index}, "
                      f"link_faces={[f.index for f in e.link_faces]}")

        if not knife_edges:
            # Phase 0+1 handled everything, clean up and return
            bpy.ops.mesh.remove_doubles(threshold=weld_rad)
            if DEBUG:
                print(f"\n  [DONE] All handled by Phase 0+1: "
                      f"{integrate_count} integrated, {face_splits} split(s), "
                      f"no knife_project needed")
                print(f"{'='*60}\n")
            self.report({'INFO'}, f"Square: edge-on-face weld — "
                        f"{integrate_count} integrated, "
                        f"{face_splits} direct split(s).")
            return {'FINISHED'}

        # Build edge pairs in world space for the cutter
        edge_pairs_world = [(mw @ e.verts[0].co, mw @ e.verts[1].co)
                            for e in knife_edges]

        # Target face info
        face_center_world = mw @ target_face.calc_center_median() if target_face.is_valid else mw @ bm.faces[0].calc_center_median()
        face_normal_world = _world_normal(obj, target_face) if target_face.is_valid else Vector((0, 0, 1))

        # Find the target face(s) — could have been split in phase 1
        # so look for all faces on the same plane
        target_faces = []
        if target_face.is_valid:
            target_faces.append(target_face)
        # Also grab any new faces from phase 1 splits on the same plane
        sample_n = face_normal_world
        sample_d = face_center_world.dot(sample_n)
        for f in bm.faces:
            if f.hide or f in target_faces:
                continue
            fn = _world_normal(obj, f)
            if abs(fn.dot(sample_n)) < 0.99:
                continue
            fc = mw @ f.calc_center_median()
            if abs(fc.dot(sample_n) - sample_d) < 0.001:
                target_faces.append(f)

        face_radius = 1.0
        if target_faces:
            face_radius = max(
                max((mw @ v.co - face_center_world).length
                    for v in f.verts)
                for f in target_faces
            )
            face_radius = max(face_radius, 0.1)

        # Build cutter mesh — offset slightly along face normal so
        # knife_project doesn't fail on exactly-coplanar geometry
        cutter_mesh = bpy.data.meshes.new("__edge_weld_cutter__")
        bm_cut = bmesh.new()
        added = {}
        normal_offset = face_normal_world * 0.001
        for (a_w, b_w) in edge_pairs_world:
            a_off = a_w + normal_offset
            b_off = b_w + normal_offset
            key_a = (round(a_w.x, 8), round(a_w.y, 8), round(a_w.z, 8))
            key_b = (round(b_w.x, 8), round(b_w.y, 8), round(b_w.z, 8))
            if key_a not in added:
                added[key_a] = bm_cut.verts.new(a_off)
            if key_b not in added:
                added[key_b] = bm_cut.verts.new(b_off)
            try:
                bm_cut.edges.new([added[key_a], added[key_b]])
            except Exception:
                pass
        bm_cut.to_mesh(cutter_mesh)
        bm_cut.free()
        cutter_mesh.update()

        cutter_obj = bpy.data.objects.new("__edge_weld_cutter__", cutter_mesh)
        context.collection.objects.link(cutter_obj)

        # Capture view state
        area, space = _find_view3d()
        orig_rot = orig_loc = orig_persp = None
        if space:
            r3d = space.region_3d
            orig_rot = r3d.view_rotation.copy()
            orig_loc = r3d.view_location.copy()
            orig_persp = r3d.view_perspective

        try:
            # Show only target faces
            bm = bmesh.from_edit_mesh(me)
            bm.faces.ensure_lookup_table()
            bpy.ops.mesh.select_all(action='DESELECT')

            target_set = set(f for f in target_faces if f.is_valid)
            for f in bm.faces:
                if f in target_set:
                    f.hide = False
                    f.select = True
                else:
                    f.hide = True
            bmesh.update_edit_mesh(me)

            # Align view and knife project
            if space:
                _align_view_to_face(space, face_center_world,
                                    face_normal_world, face_radius)
                try:
                    bpy.ops.wm.redraw_timer(type='DRAW_WIN', iterations=1)
                except Exception:
                    pass
                context.view_layer.update()

            cutter_obj.select_set(True)
            obj.select_set(True)
            context.view_layer.objects.active = obj

            if DEBUG:
                print(f"  [P2] Running knife_project with {len(edge_pairs_world)} "
                      f"cutter edge(s), {len(target_set)} target face(s)")
                print(f"  [P2] normal_offset={normal_offset!s}")

            bpy.ops.mesh.knife_project(cut_through=True)

            # Teleport cut verts to ideal positions
            bm_final = bmesh.from_edit_mesh(me)
            bm_final.verts.ensure_lookup_table()
            search_rad_sq = (weld_rad * 2.5) ** 2
            for v in bm_final.verts:
                if v.hide:
                    continue
                if v.select:
                    for co in arc_coords:
                        if (v.co - co).length_squared < search_rad_sq:
                            v.co = co.copy()
                            break
                else:
                    for co in arc_coords:
                        if (v.co - co).length_squared < 1e-8:
                            v.select = True
                            break
            bmesh.update_edit_mesh(me)
            bpy.ops.mesh.remove_doubles(threshold=weld_rad)

            # Unhide everything
            bm = bmesh.from_edit_mesh(me)
            for f in bm.faces:
                f.hide = False
                f.select = False
                for v in f.verts:
                    v.hide = False
                for e in f.edges:
                    e.hide = False
            bmesh.update_edit_mesh(me)

            # Restore originally hidden faces
            bm = bmesh.from_edit_mesh(me)
            for f in bm.faces:
                if _face_centroid_key(f, mw) in originally_hidden_keys:
                    f.hide = True
            bmesh.update_edit_mesh(me)

        finally:
            # Restore view
            if space and orig_rot:
                space.region_3d.view_rotation = orig_rot
                space.region_3d.view_location = orig_loc
                space.region_3d.view_perspective = orig_persp
            # Clean up cutter
            if cutter_obj.name in bpy.data.objects:
                bpy.data.objects.remove(cutter_obj, do_unlink=True)
            if cutter_mesh.name in bpy.data.meshes:
                bpy.data.meshes.remove(cutter_mesh)

        if DEBUG:
            print(f"\n  [DONE] edge-on-face weld complete: "
                  f"P0 integrated {integrate_count} vert(s), "
                  f"P1 split {face_splits} face(s), "
                  f"P2 knife_project on {len(knife_edges)} edge(s)")
            print(f"{'='*60}\n")

        self.report({'INFO'}, f"Square: edge-on-face weld — "
                    f"{integrate_count} integrated, "
                    f"{face_splits} split(s) + knife project.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_square,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
