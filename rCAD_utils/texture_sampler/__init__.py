"""Pick a mesh material in the viewport and apply it to another mesh."""

import math

import bmesh
import bpy
from bpy_extras import view3d_utils
from mathutils import Vector


_UV_EPSILON = 1.0e-8


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


def _event_in_ui_region(context, event):
    """Check whether a modal event is physically over the sidebar region."""
    area = context.area
    if area is None or area.type != 'VIEW_3D':
        return False
    return any(
        region.type == 'UI' and _region_mouse_coord(area, region, event) is not None
        for region in area.regions
    )


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


def _material_uv_map_name(material):
    """Return a named UV map explicitly used by the material, if any."""
    node_tree = getattr(material, "node_tree", None)
    if node_tree is None:
        return None

    for node in node_tree.nodes:
        if node.bl_idname != 'ShaderNodeUVMap':
            continue
        if any(output.is_linked for output in node.outputs):
            return node.uv_map or None
    return None


def _uv_layer_for_material(bm, material, create=False):
    preferred_name = _material_uv_map_name(material)
    uv_layers = bm.loops.layers.uv
    uv_layer = uv_layers.get(preferred_name) if preferred_name else uv_layers.active
    if uv_layer is None and create:
        uv_layer = uv_layers.new(preferred_name or "UVMap")
    return uv_layer


def _face_uv_area(face, uv_layer):
    if uv_layer is None or len(face.loops) < 3:
        return 0.0

    area = 0.0
    previous = face.loops[-1][uv_layer].uv
    for loop in face.loops:
        current = loop[uv_layer].uv
        area += previous.x * current.y - current.x * previous.y
        previous = current
    return abs(area) * 0.5


def _face_world_area(obj, face):
    if len(face.verts) < 3:
        return 0.0

    world_vertices = [obj.matrix_world @ vert.co for vert in face.verts]
    origin = world_vertices[0]
    area = 0.0
    for index in range(1, len(world_vertices) - 1):
        area += (world_vertices[index] - origin).cross(
            world_vertices[index + 1] - origin
        ).length * 0.5
    return area


def _object_bmesh(obj):
    if obj.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        return bm, False

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    return bm, True


def _source_uv_density(obj, face_index, material):
    """Measure UV units per world unit on the sampled source face."""
    bm, owns_bmesh = _object_bmesh(obj)
    try:
        if face_index < 0 or face_index >= len(bm.faces):
            return 0.0
        uv_layer = _uv_layer_for_material(bm, material)
        uv_area = _face_uv_area(bm.faces[face_index], uv_layer)
        world_area = _face_world_area(obj, bm.faces[face_index])
        if uv_area <= _UV_EPSILON or world_area <= _UV_EPSILON:
            return 0.0
        return math.sqrt(uv_area / world_area)
    finally:
        if owns_bmesh:
            bm.free()


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


def _component_uv_density(obj, faces, uv_layer):
    uv_area = sum(_face_uv_area(face, uv_layer) for face in faces)
    world_area = sum(_face_world_area(obj, face) for face in faces)
    if uv_area <= _UV_EPSILON or world_area <= _UV_EPSILON:
        return 0.0
    return math.sqrt(uv_area / world_area)


def _edit_objects(context):
    objects = getattr(context, "objects_in_mode_unique_data", None)
    if objects is None:
        objects = context.selected_objects
    return [obj for obj in objects if obj.type == 'MESH' and obj.mode == 'EDIT']


def _capture_edit_selection(context):
    state = []
    for obj in _edit_objects(context):
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        state.append(
            (
                obj,
                [vert.select for vert in bm.verts],
                [edge.select for edge in bm.edges],
                [face.select for face in bm.faces],
            )
        )
    return state


def _restore_edit_selection(state):
    for obj, vert_selection, edge_selection, face_selection in state:
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        for vert, selected in zip(bm.verts, vert_selection):
            vert.select_set(selected)
        for edge, selected in zip(bm.edges, edge_selection):
            edge.select_set(selected)
        for face, selected in zip(bm.faces, face_selection):
            face.select_set(selected)
        bmesh.update_edit_mesh(obj.data)


def _unwrap_target_component(
    context,
    target,
    face_index,
    material,
    source_object,
    source_face_index,
    face_only=False,
):
    """Unwrap the clicked target island and match the source UV density."""
    if target.mode != 'EDIT':
        return False

    bm = bmesh.from_edit_mesh(target.data)
    if face_only:
        bm.faces.ensure_lookup_table()
        target_faces = (
            [bm.faces[face_index]]
            if 0 <= face_index < len(bm.faces)
            else []
        )
    else:
        target_faces = _connected_edit_faces(bm, face_index)
    if not target_faces:
        raise RuntimeError("The clicked target face could not be found for UV mapping.")

    uv_layer = _uv_layer_for_material(bm, material, create=True)
    if uv_layer is None:
        raise RuntimeError("Could not create a UV map on the target mesh.")

    # Unwrap only the clicked island. Preserve the user's current edit
    # selection because the sampler itself is not a selection tool.
    selection_state = _capture_edit_selection(context)
    try:
        for edit_obj in _edit_objects(context):
            edit_bm = bmesh.from_edit_mesh(edit_obj.data)
            for face in edit_bm.faces:
                face.select_set(False)
            for edge in edit_bm.edges:
                edge.select_set(False)
            for vert in edit_bm.verts:
                vert.select_set(False)

        for face in target_faces:
            face.select_set(True)
            for edge in face.edges:
                edge.select_set(True)
            for vert in face.verts:
                vert.select_set(True)

        bmesh.update_edit_mesh(target.data)
        context.view_layer.objects.active = target
        region = next(
            (item for item in context.area.regions if item.type == 'WINDOW'),
            None,
        )
        if region is None:
            raise RuntimeError("Could not find the 3D Viewport region for UV unwrap.")

        with context.temp_override(
            window=context.window,
            area=context.area,
            region=region,
        ):
            result = bpy.ops.uv.smart_project(
                angle_limit=math.radians(66.0),
                island_margin=0.001,
                scale_to_bounds=False,
            )
        if 'FINISHED' not in result:
            raise RuntimeError("Blender could not create UVs for the target island.")
    finally:
        _restore_edit_selection(selection_state)

    # Re-fetch the edit mesh after Blender's UV operator has rebuilt its data.
    bm = bmesh.from_edit_mesh(target.data)
    if face_only:
        bm.faces.ensure_lookup_table()
        mapped_faces = (
            [bm.faces[face_index]]
            if 0 <= face_index < len(bm.faces)
            else []
        )
    else:
        mapped_faces = _connected_edit_faces(bm, face_index)
    uv_layer = _uv_layer_for_material(bm, material)
    if not mapped_faces or uv_layer is None:
        raise RuntimeError("The target UV map was not available after unwrapping.")

    source_density = _source_uv_density(
        source_object,
        source_face_index,
        material,
    )
    target_density = _component_uv_density(target, mapped_faces, uv_layer)
    if source_density > _UV_EPSILON and target_density > _UV_EPSILON:
        loops = [loop for face in mapped_faces for loop in face.loops]
        center = Vector((
            sum(loop[uv_layer].uv.x for loop in loops) / len(loops),
            sum(loop[uv_layer].uv.y for loop in loops) / len(loops),
        ))
        scale = source_density / target_density
        for loop in loops:
            uv = loop[uv_layer].uv.copy()
            loop[uv_layer].uv = center + (uv - center) * scale
        bmesh.update_edit_mesh(target.data)
    return True


def _apply_material(target, material, face_index=None, face_only=False):
    """Assign material to the target object or clicked edit-mode island."""
    mesh = target.data
    slot_index = _material_slot(mesh, material)
    if slot_index == -1:
        mesh.materials.append(material)
        slot_index = len(mesh.materials) - 1

    if target.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(mesh)
        if face_only:
            bm.faces.ensure_lookup_table()
            faces = (
                [bm.faces[face_index]]
                if 0 <= face_index < len(bm.faces)
                else []
            )
        else:
            faces = _connected_edit_faces(bm, face_index)
        if not faces:
            raise RuntimeError("The clicked edit-mode face could not be identified.")
        for face in faces:
            face.material_index = slot_index
        bmesh.update_edit_mesh(mesh, destructive=True)
    else:
        polygons = (
            [mesh.polygons[face_index]]
            if face_only and face_index is not None and 0 <= face_index < len(mesh.polygons)
            else mesh.polygons
        )
        for polygon in polygons:
            polygon.material_index = slot_index
    target.active_material_index = slot_index
    mesh.update()
    return slot_index


def _same_edit_island(obj, first_face_index, second_face_index):
    bm = bmesh.from_edit_mesh(obj.data)
    first_faces = _connected_edit_faces(bm, first_face_index)
    return any(face.index == second_face_index for face in first_faces)


def _rotate_face_uvs(face, uv_layer):
    """Rotate one face's UVs 90 degrees counter-clockwise around its center."""
    if uv_layer is None or not face.loops:
        return False

    center = Vector((
        sum(loop[uv_layer].uv.x for loop in face.loops) / len(face.loops),
        sum(loop[uv_layer].uv.y for loop in face.loops) / len(face.loops),
    ))
    for loop in face.loops:
        offset = loop[uv_layer].uv - center
        loop[uv_layer].uv = center + Vector((-offset.y, offset.x))
    return True


class MESH_OT_TextureSamplerRotateUV(bpy.types.Operator):
    """Rotate the selected Edit Mode faces' active UVs by 90 degrees."""

    bl_idname = "mesh.texture_sampler_rotate_uv"
    bl_label = "Rotate UV 90 Degrees"
    bl_description = "Rotate the selected faces' UVs 90 degrees"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Enter Edit Mode and select one or more faces first.")
            return {'CANCELLED'}

        rotated_faces = 0
        for obj in _edit_objects(context):
            bm = bmesh.from_edit_mesh(obj.data)
            uv_layer = bm.loops.layers.uv.active
            if uv_layer is None:
                continue
            for face in bm.faces:
                if face.select and _rotate_face_uvs(face, uv_layer):
                    rotated_faces += 1
            bmesh.update_edit_mesh(obj.data)

        if rotated_faces == 0:
            self.report({'WARNING'}, "Select one or more faces with UVs to rotate.")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Rotated UVs on {rotated_faces} face(s) by 90 degrees.")
        return {'FINISHED'}


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
        self._face_only = getattr(context.scene, "rcad_texture_sampler_face_only", False)
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

        if _event_in_ui_region(context, event):
            return {'PASS_THROUGH'}

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
            if self._face_only:
                same_target = self._source_face_index == _face_index
            else:
                same_target = (
                    picked_object.mode != 'EDIT'
                    or _same_edit_island(
                        picked_object,
                        self._source_face_index,
                        _face_index,
                    )
                )
            if same_target:
                self.report({'WARNING'}, "Choose a different mesh island for the target.")
                return {'RUNNING_MODAL'}
        elif picked_object.data == self._source_object.data:
            self.report({'WARNING'}, "Choose a mesh with separate mesh data for the target.")
            return {'RUNNING_MODAL'}

        try:
            if picked_object.mode == 'EDIT':
                try:
                    _unwrap_target_component(
                        context,
                        picked_object,
                        _face_index,
                        self._source_material,
                        self._source_object,
                        self._source_face_index,
                        self._face_only,
                    )
                except (RuntimeError, TypeError, ValueError) as error:
                    self.report({'WARNING'}, f"Material applied, but UVs were not updated: {error}")

            target_face_index = _face_index if picked_object.mode == 'EDIT' else None
            _apply_material(
                picked_object,
                self._source_material,
                target_face_index,
                self._face_only,
            )
        except (RuntimeError, TypeError, ValueError) as error:
            self.report({'ERROR'}, f"Could not apply the material: {error}")
            return {'RUNNING_MODAL'}

        self.report(
            {'INFO'},
            f'Applied "{self._source_material.name}" to "{picked_object.name}".',
        )
        self._phase = 'TARGET'
        if context.area is not None:
            context.area.tag_redraw()
        self.report({'INFO'}, "Click another target to apply the sampled texture, or Esc to finish.")
        return {'RUNNING_MODAL'}

    def execute(self, context):
        # This keeps F3 and scripted invocation useful as well as the sidebar
        # button, which explicitly uses INVOKE_DEFAULT.
        return self.invoke(context, None)


classes = (MESH_OT_TextureSamplerRotateUV, MESH_OT_TextureSampler,)
