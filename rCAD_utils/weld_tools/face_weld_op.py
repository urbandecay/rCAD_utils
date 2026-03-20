# face_weld_op.py
# Uses knife_project to cut source face outline into target face — same approach as radCAD arc tool.

import bpy
import bmesh
from mathutils import Vector

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


class MESH_OT_super_fuse_square(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_square"
    bl_label = "Super Fuse: Square (■)"
    bl_description = "Fuse a smaller face onto a larger coplanar face"
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
            self.report({'ERROR'}, "Square: select two coplanar faces.")
            return {'CANCELLED'}

        mw = obj.matrix_world.copy()

        # Smallest face = source (gets cut into the target), largest = target
        selected_faces.sort(key=lambda f: f.calc_area())
        source_face = selected_faces[0]
        target_face = selected_faces[-1]

        # Confirm coplanar
        source_world_pts = [mw @ v.co for v in source_face.verts]
        dists = [_dist_to_face_plane_world(obj, target_face, p) for p in source_world_pts]
        if max(dists) > max(sr, 1e-6) + 1e-12:
            self.report({'ERROR'}, "Square: faces must be coplanar. Increase search radius.")
            return {'CANCELLED'}

        # Capture ideal coordinates (local space) and edge pairs (world space for cutter)
        # before we touch anything
        arc_coords = [v.co.copy() for v in source_face.verts]
        edge_pairs_world = [(mw @ e.verts[0].co, mw @ e.verts[1].co) for e in source_face.edges]

        # Target face reference and visibility data
        originally_hidden = [f.index for f in bm.faces if f.hide]
        face_center_world = mw @ target_face.calc_center_median()
        face_normal_world = _world_normal(obj, target_face)
        face_radius = max((mw @ v.co - face_center_world).length for v in target_face.verts)
        face_radius = max(face_radius, 0.1)

        # Delete source face — edges and verts stay, they'll be cleaned by remove_doubles later
        bmesh.ops.delete(bm, geom=[source_face], context='FACES_ONLY')
        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)

        # Build cutter object from source edge pairs (world space coords, identity transform)
        cutter_mesh = bpy.data.meshes.new("__face_weld_cutter__")
        bm_cut = bmesh.new()
        added = {}
        for (a_w, b_w) in edge_pairs_world:
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

        try:
            # Hide everything except target face
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

            # Align view to face normal so knife_project cuts correctly
            area, space = _find_view3d()
            orig_rot = orig_loc = orig_persp = None
            if space:
                r3d = space.region_3d
                orig_rot = r3d.view_rotation.copy()
                orig_loc = r3d.view_location.copy()
                orig_persp = r3d.view_perspective
                _align_view_to_face(space, face_center_world, face_normal_world, face_radius)
                try:
                    bpy.ops.wm.redraw_timer(type='DRAW_WIN', iterations=1)
                except Exception:
                    pass
                context.view_layer.update()

            # Run knife project
            cutter_obj.select_set(True)
            obj.select_set(True)
            context.view_layer.objects.active = obj

            bpy.ops.mesh.knife_project(cut_through=True)

            # Teleport newly cut verts to ideal positions (radCAD GPS approach)
            bm_final = bmesh.from_edit_mesh(me)
            bm_final.verts.ensure_lookup_table()
            search_rad_sq = (weld_rad * 2.5) ** 2

            for v in bm_final.verts:
                if v.hide:
                    continue
                if v.select:
                    # Newly cut vert — snap to nearest ideal coord
                    for target_co in arc_coords:
                        if (v.co - target_co).length_squared < search_rad_sq:
                            v.co = target_co.copy()
                            break
                else:
                    # Existing vert — flag for merging if it's already at an ideal coord
                    for target_co in arc_coords:
                        if (v.co - target_co).length_squared < 1e-8:
                            v.select = True
                            break

            bmesh.update_edit_mesh(me)
            bpy.ops.mesh.remove_doubles(threshold=weld_rad)

            # Restore visibility
            bm_clean = bmesh.from_edit_mesh(me)
            bm_clean.faces.ensure_lookup_table()
            hidden_set = set(originally_hidden)
            for f in bm_clean.faces:
                f.hide = f.index in hidden_set
                if not f.hide:
                    for v in f.verts:
                        v.hide = False
                    for e in f.edges:
                        e.hide = False
                    f.select = False
            bmesh.update_edit_mesh(me)

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
            if cutter_obj.name in bpy.data.objects:
                bpy.data.objects.remove(cutter_obj, do_unlink=True)
            if cutter_mesh.name in bpy.data.meshes:
                bpy.data.meshes.remove(cutter_mesh)

        self.report({'INFO'}, "Square: fuse complete.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_square,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
