"""Pick a mesh material in the viewport and apply it to another mesh."""

import bmesh
import bpy
from bpy_extras import view3d_utils


def _region_mouse_coord(area, region, event):
    """Convert a window event to region coordinates across Blender versions."""
    mouse_x = getattr(event, "mouse_x", None)
    mouse_y = getattr(event, "mouse_y", None)
    if mouse_x is None or mouse_y is None:
        return None

    # Region.x/y are window-relative in current Blender versions.  Older
    # builds exposed them area-relative, so validate both coordinate systems.
    candidates = (
        (mouse_x - region.x, mouse_y - region.y),
        (mouse_x - area.x - region.x, mouse_y - area.y - region.y),
    )
    for coord_x, coord_y in candidates:
        if 0 <= coord_x < region.width and 0 <= coord_y < region.height:
            return coord_x, coord_y
    return None


def _view_window_region(context, event):
    """Return the View3D window region and mouse coordinate under the cursor."""
    area = context.area
    if area is None or area.type != 'VIEW_3D':
        return None, None, None

    space = context.space_data
    region_3d = getattr(space, "region_3d", None)
    if region_3d is None:
        return None, None, None

    for region in area.regions:
        if region.type != 'WINDOW':
            continue
        coord = _region_mouse_coord(area, region, event)
        if coord is not None:
            return region, region_3d, coord

    region = context.region
    if region is not None and region.type == 'WINDOW':
        coord = (event.mouse_region_x, event.mouse_region_y)
        if 0 <= coord[0] < region.width and 0 <= coord[1] < region.height:
            return region, region_3d, coord
    return None, None, None


def _mouse_ray(context, event):
    region, region_3d, coord = _view_window_region(context, event)
    if region is None:
        return None

    origin = view3d_utils.region_2d_to_origin_3d(region, region_3d, coord)
    direction = view3d_utils.region_2d_to_vector_3d(region, region_3d, coord)
    if direction.length == 0.0:
        return None
    return origin, direction.normalized()


def _scene_ray_cast(context, origin, direction):
    """Ray-cast with compatibility for Blender versions supported by rCAD."""
    scene = context.scene
    try:
        depsgraph = context.evaluated_depsgraph_get()
    except AttributeError:
        depsgraph = context.view_layer.depsgraph

    try:
        return scene.ray_cast(
            depsgraph=depsgraph,
            origin=origin,
            direction=direction,
        )
    except TypeError:
        return scene.ray_cast(
            view_layer=context.view_layer,
            origin=origin,
            direction=direction,
        )


def _original_object(obj):
    """Return the scene object behind an evaluated ray-cast result."""
    original = getattr(obj, "original", None)
    return original if original is not None else obj


def _material_from_hit(hit_object, face_index):
    """Get the material assigned to the face hit by the viewport ray."""
    if hit_object is None or hit_object.type != 'MESH':
        return None

    mesh = hit_object.data
    material_index = getattr(hit_object, "active_material_index", 0)
    if hit_object.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(mesh)
        bm.faces.ensure_lookup_table()
        if 0 <= face_index < len(bm.faces):
            material_index = bm.faces[face_index].material_index
    elif 0 <= face_index < len(mesh.polygons):
        material_index = mesh.polygons[face_index].material_index

    if 0 <= material_index < len(mesh.materials):
        return mesh.materials[material_index]
    return None


def _material_slot(mesh, material):
    for index, slot_material in enumerate(mesh.materials):
        if slot_material == material:
            return index
    return -1


def _connected_edit_faces(bm, face_index):
    """Return the disconnected mesh island containing one edit-mode face."""
    bm.faces.ensure_lookup_table()
    if face_index < 0 or face_index >= len(bm.faces):
        return []

    component = []
    visited = set()
    pending = [bm.faces[face_index]]
    while pending:
        face = pending.pop()
        if face.index in visited:
            continue
        visited.add(face.index)
        component.append(face)
        for edge in face.edges:
            pending.extend(linked for linked in edge.link_faces if linked.index not in visited)
    return component


def _apply_material(target, material, face_index=None):
    """Assign material to the target object or clicked edit-mode island."""
    mesh = target.data
    slot_index = _material_slot(mesh, material)
    if slot_index == -1:
        mesh.materials.append(material)
        slot_index = len(mesh.materials) - 1

    if target.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(mesh)
        faces = _connected_edit_faces(bm, face_index)
        if not faces:
            raise RuntimeError("The clicked edit-mode face could not be identified.")
        for face in faces:
            face.material_index = slot_index
        bmesh.update_edit_mesh(mesh, destructive=True)
    else:
        for polygon in mesh.polygons:
            polygon.material_index = slot_index
    target.active_material_index = slot_index
    mesh.update()
    return slot_index


def _same_edit_island(obj, first_face_index, second_face_index):
    bm = bmesh.from_edit_mesh(obj.data)
    first_faces = _connected_edit_faces(bm, first_face_index)
    return any(face.index == second_face_index for face in first_faces)


class MESH_OT_TextureSampler(bpy.types.Operator):
    """Pick a face material, then apply it to a second mesh."""

    bl_idname = "mesh.texture_sampler"
    bl_label = "Texture Sampler"
    bl_description = "Click a textured mesh, then click a mesh to apply its material"
    bl_options = {'REGISTER', 'UNDO'}

    def _set_cursor(self, cursor):
        self._modal_cursor_active = False
        try:
            self._window.cursor_modal_set(cursor)
            self._modal_cursor_active = True
        except (AttributeError, TypeError, ValueError):
            # EYEDROPPER is not available in a few older Blender builds.
            if cursor == 'EYEDROPPER':
                try:
                    self._window.cursor_modal_set('CROSSHAIR')
                    self._modal_cursor_active = True
                except (AttributeError, TypeError, ValueError):
                    try:
                        self._window.cursor_set('CROSSHAIR')
                    except (AttributeError, TypeError, ValueError):
                        pass

    def _restore_cursor(self):
        window = getattr(self, "_window", None)
        if window is None:
            return
        try:
            if getattr(self, "_modal_cursor_active", False):
                window.cursor_modal_restore()
            else:
                window.cursor_set('DEFAULT')
        except (AttributeError, TypeError, ValueError):
            pass

    def _refresh_cursor(self):
        """Keep Edit Mode from replacing the modal cursor on mouse movement."""
        if not getattr(self, "_modal_cursor_active", False):
            return
        try:
            self._window.cursor_modal_set('EYEDROPPER')
        except (AttributeError, TypeError, ValueError):
            try:
                self._window.cursor_modal_set('CROSSHAIR')
            except (AttributeError, TypeError, ValueError):
                pass

    def _end(self, context, result):
        self._restore_cursor()
        if context.area is not None:
            context.area.tag_redraw()
        return result

    def _pick_mesh(self, context, event):
        ray = _mouse_ray(context, event)
        if ray is None:
            return None, None, None

        origin, direction = ray
        hit, _location, _normal, face_index, hit_object, _matrix = _scene_ray_cast(
            context,
            origin,
            direction,
        )
        if not hit or hit_object is None or hit_object.type != 'MESH':
            return None, None, None

        scene_object = _original_object(hit_object)
        if getattr(scene_object, "mode", None) == 'EDIT':
            material = _material_from_hit(scene_object, face_index)
            if material is None and scene_object is not hit_object:
                material = _material_from_hit(hit_object, face_index)
        else:
            material = _material_from_hit(hit_object, face_index)
            if material is None and scene_object is not hit_object:
                material = _material_from_hit(scene_object, face_index)
        return scene_object, material, face_index

    def invoke(self, context, event):
        if context.area is None or context.area.type != 'VIEW_3D':
            self.report({'ERROR'}, "Texture Sampler must be started in a 3D Viewport.")
            return {'CANCELLED'}

        self._window = context.window
        self._source_object = None
        self._source_material = None
        self._source_face_index = -1
        self._phase = 'SOURCE'
        self._set_cursor('EYEDROPPER')
        context.window_manager.modal_handler_add(self)
        self.report({'INFO'}, "Click a textured mesh to sample it. Esc cancels.")
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'ESC', 'RIGHTMOUSE'} and event.value == 'PRESS':
            return self._end(context, {'CANCELLED'})

        if event.type == 'MOUSEMOVE':
            self._refresh_cursor()
            return {'RUNNING_MODAL'}

        if event.type != 'LEFTMOUSE' or event.value != 'PRESS':
            return {'PASS_THROUGH'}

        try:
            picked_object, material, _face_index = self._pick_mesh(context, event)
        except (RuntimeError, TypeError, ValueError) as error:
            self.report({'ERROR'}, f"Could not sample the viewport: {error}")
            return {'RUNNING_MODAL'}
        if picked_object is None:
            self.report({'WARNING'}, "Click a visible mesh in the 3D Viewport.")
            return {'RUNNING_MODAL'}

        if self._phase == 'SOURCE':
            if material is None:
                self.report({'WARNING'}, f'"{picked_object.name}" has no material on the clicked face.')
                return {'RUNNING_MODAL'}

            self._source_object = picked_object
            self._source_material = material
            self._source_face_index = _face_index
            self._phase = 'TARGET'
            self.report(
                {'INFO'},
                f'Sampled "{material.name}". Click the mesh to apply it to, or Esc to cancel.',
            )
            return {'RUNNING_MODAL'}

        if picked_object == self._source_object:
            if picked_object.mode != 'EDIT' or _same_edit_island(
                picked_object,
                self._source_face_index,
                _face_index,
            ):
                self.report({'WARNING'}, "Choose a different mesh island for the target.")
                return {'RUNNING_MODAL'}
        elif picked_object.data == self._source_object.data:
            self.report({'WARNING'}, "Choose a mesh with separate mesh data for the target.")
            return {'RUNNING_MODAL'}

        try:
            target_face_index = _face_index if picked_object.mode == 'EDIT' else None
            _apply_material(picked_object, self._source_material, target_face_index)
        except (RuntimeError, TypeError, ValueError) as error:
            self.report({'ERROR'}, f"Could not apply the material: {error}")
            return {'RUNNING_MODAL'}

        self.report(
            {'INFO'},
            f'Applied "{self._source_material.name}" to "{picked_object.name}".',
        )
        return self._end(context, {'FINISHED'})

    def execute(self, context):
        # This keeps F3 and scripted invocation useful as well as the sidebar
        # button, which explicitly uses INVOKE_DEFAULT.
        return self.invoke(context, None)


classes = (MESH_OT_TextureSampler,)
