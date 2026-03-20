# face_weld_op.py
# Uses knife_project to fuse smaller faces into larger coplanar faces.
# Handles multiple source faces on multiple targets in one operation.

import bpy
import bmesh
from mathutils import Vector
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
        if len(selected_faces) < 2:
            self.report({'ERROR'}, "Square: select at least two coplanar faces.")
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
            for (target_face, source_faces) in jobs:
                # Collect all source data before touching anything
                all_arc_coords = []
                all_edge_pairs_world = []
                for sf in source_faces:
                    all_arc_coords.extend(v.co.copy() for v in sf.verts)
                    all_edge_pairs_world.extend(
                        (mw @ e.verts[0].co, mw @ e.verts[1].co) for e in sf.edges
                    )

                face_center_world = mw @ target_face.calc_center_median()
                face_normal_world = _world_normal(obj, target_face)
                face_radius = max((mw @ v.co - face_center_world).length for v in target_face.verts)
                face_radius = max(face_radius, 0.1)

                # Delete all source faces for this job (keep edges/verts)
                bm = bmesh.from_edit_mesh(me)
                bmesh.ops.delete(bm, geom=source_faces, context='FACES_ONLY')
                bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

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


classes = (MESH_OT_super_fuse_square,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
