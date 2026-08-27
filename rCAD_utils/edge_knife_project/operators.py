"""Viewport-managed knife projection for selected Edit Mode mesh edges."""

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree


_EPSILON = 1.0e-10
_TEMP_OBJECT_NAME = "__rcad_edge_knife_cutter__"


def _active_edit_mesh(context):
    obj = getattr(context, "edit_object", None)
    if obj is None:
        obj = getattr(context, "active_object", None)
    if obj is None or obj.type != 'MESH' or obj.mode != 'EDIT':
        return None
    return obj


def _view3d_context(context):
    """Return the 3D View area, space, and window region used by the operator."""
    area = getattr(context, "area", None)
    if area is None or area.type != 'VIEW_3D':
        screen = getattr(context, "screen", None)
        area = next(
            (candidate for candidate in getattr(screen, "areas", ())
             if candidate.type == 'VIEW_3D'),
            None,
        )
    if area is None:
        return None, None, None

    space = getattr(area.spaces, "active", None)
    if space is None or space.type != 'VIEW_3D':
        return None, None, None

    region = next(
        (candidate for candidate in area.regions if candidate.type == 'WINDOW'),
        None,
    )
    if region is None:
        return None, None, None
    return area, space, region


def _world_normal(obj, face):
    normal = obj.matrix_world.to_3x3().inverted_safe().transposed() @ face.normal
    if normal.length <= _EPSILON:
        return Vector()
    return normal.normalized()


def _world_face_points(obj, faces):
    points = []
    seen = set()
    for face in faces:
        for vert in face.verts:
            # A face can share vertices with many other target faces.  Avoid
            # weighting shared vertices multiple times when finding the view.
            if vert in seen:
                continue
            seen.add(vert)
            points.append(obj.matrix_world @ vert.co)
    return points


def _average(points):
    if not points:
        return Vector()
    result = Vector()
    for point in points:
        result += point
    return result / len(points)


def _selected_inputs(bm):
    """Separate explicit cutter edges from selected target-face boundaries.

    Blender selects a face's boundary edges along with the face.  Those edges
    describe the target and must not be copied into the temporary cutter.
    """
    target_faces = [
        face
        for face in bm.faces
        if face.select and not face.hide and len(face.verts) >= 3
    ]
    target_set = set(target_faces)
    source_edges = [
        edge
        for edge in bm.edges
        if (
            edge.select
            and not edge.hide
            and not any(face in target_set for face in edge.link_faces)
        )
    ]
    return source_edges, target_faces


def _capture_bmesh_state(bm):
    return {
        "selected_verts": tuple(vert for vert in bm.verts if vert.select),
        "selected_edges": tuple(edge for edge in bm.edges if edge.select),
        "selected_faces": tuple(face for face in bm.faces if face.select),
        "hidden_verts": tuple(vert for vert in bm.verts if vert.hide),
        "hidden_edges": tuple(edge for edge in bm.edges if edge.hide),
        "hidden_faces": tuple(face for face in bm.faces if face.hide),
        "select_history": tuple(
            element for element in bm.select_history
            if element.is_valid and element.select
        ),
    }


def _is_live(element, collection):
    try:
        return element.is_valid and element in collection
    except (ReferenceError, RuntimeError, TypeError):
        return False


def _restore_visibility(bm, state):
    """Remove the temporary target-face hiding and restore original hides."""
    for vert in bm.verts:
        vert.hide = False
    for edge in bm.edges:
        edge.hide = False
    for face in bm.faces:
        face.hide = False

    for vert in state["hidden_verts"]:
        if _is_live(vert, bm.verts):
            vert.hide = True
    for edge in state["hidden_edges"]:
        if _is_live(edge, bm.edges):
            edge.hide = True
    for face in state["hidden_faces"]:
        if _is_live(face, bm.faces):
            face.hide = True


def _restore_selection(bm, state):
    for face in bm.faces:
        face.select_set(False)
    for edge in bm.edges:
        edge.select_set(False)
    for vert in bm.verts:
        vert.select_set(False)

    for vert in state["selected_verts"]:
        if _is_live(vert, bm.verts):
            vert.select_set(True)
    for edge in state["selected_edges"]:
        if _is_live(edge, bm.edges):
            edge.select_set(True)
    for face in state["selected_faces"]:
        if _is_live(face, bm.faces):
            face.select_set(True)

    bm.select_history.clear()
    for element in state["select_history"]:
        try:
            if element.is_valid and element.select:
                bm.select_history.add(element)
        except (ReferenceError, RuntimeError):
            continue


def _prepare_target(bm, target_faces):
    target_set = set(target_faces)
    target_edges = {edge for face in target_faces for edge in face.edges}
    target_verts = {vert for face in target_faces for vert in face.verts}

    for face in bm.faces:
        face.hide = face not in target_set
        face.select_set(False)

    for edge in bm.edges:
        edge.hide = edge not in target_edges
        edge.select_set(False)
    for vert in bm.verts:
        vert.hide = vert not in target_verts
        vert.select_set(False)

    # Knife Project works on the active object's visible edit geometry.  Make
    # target faces and their boundary fully visible/selected for compatibility
    # with Blender versions that inspect the edit selection during the call.
    for face in target_faces:
        face.hide = False
        face.select_set(True)
        for edge in face.edges:
            edge.hide = False
            edge.select_set(True)
        for vert in face.verts:
            vert.hide = False
            vert.select_set(True)


def _create_cutter(context, edge_points, offset=Vector()):
    mesh = bpy.data.meshes.new(_TEMP_OBJECT_NAME)
    cutter_bmesh = bmesh.new()
    vertices = {}

    try:
        for first, second in edge_points:
            first = first + offset
            second = second + offset
            first_key = tuple(round(value, 10) for value in first)
            second_key = tuple(round(value, 10) for value in second)
            first_vert = vertices.get(first_key)
            if first_vert is None:
                first_vert = cutter_bmesh.verts.new(first)
                vertices[first_key] = first_vert
            second_vert = vertices.get(second_key)
            if second_vert is None:
                second_vert = cutter_bmesh.verts.new(second)
                vertices[second_key] = second_vert
            if first_vert is second_vert:
                continue
            try:
                cutter_bmesh.edges.new((first_vert, second_vert))
            except ValueError:
                # Multiple selected edges can have the same endpoints after
                # a previous operation.  Keep one wire edge in the cutter.
                pass

        cutter_bmesh.to_mesh(mesh)
    except Exception:
        bpy.data.meshes.remove(mesh)
        raise
    finally:
        cutter_bmesh.free()

    collection = getattr(context, "collection", None)
    if collection is None and context.edit_object and context.edit_object.users_collection:
        collection = context.edit_object.users_collection[0]
    if collection is None:
        collection = context.scene.collection

    cutter = bpy.data.objects.new(_TEMP_OBJECT_NAME, mesh)
    collection.objects.link(cutter)
    cutter.matrix_world = Matrix.Identity(4)
    cutter.display_type = 'WIRE'
    cutter.hide_render = True
    return cutter


def _current_view_direction(space):
    try:
        direction = -space.region_3d.view_matrix.inverted_safe().col[2].to_3d()
    except (AttributeError, TypeError, ValueError):
        return Vector()
    if direction.length <= _EPSILON:
        return Vector()
    return direction.normalized()


def _nearest_target_surface(source_point, obj, target_faces):
    """Return the nearest world-space point and normal on selected faces."""
    target_verts = []
    target_polygons = []
    vert_indices = {}

    for face in target_faces:
        polygon = []
        for vert in face.verts:
            index = vert_indices.get(vert)
            if index is None:
                index = len(target_verts)
                vert_indices[vert] = index
                target_verts.append(obj.matrix_world @ vert.co)
            polygon.append(index)
        if len(polygon) >= 3:
            target_polygons.append(polygon)

    if not target_polygons:
        return None, None

    try:
        tree = BVHTree.FromPolygons(
            target_verts,
            target_polygons,
            all_triangles=False,
            epsilon=0.0,
        )
        location, normal, _face_index, _distance = tree.find_nearest(source_point)
    except (RuntimeError, TypeError, ValueError):
        return None, None
    return location, normal


def _choose_view_direction(space, obj, source_points, target_faces, target_points):
    source_center = _average(source_points)
    nearest_point, nearest_normal = _nearest_target_surface(
        source_center,
        obj,
        target_faces,
    )
    toward_target = (
        nearest_point - source_center
        if nearest_point is not None
        else Vector()
    )

    # Use the shortest path to the selected surface, not the target selection's
    # centroid.  A face/object centroid can be above or below the cutter and
    # incorrectly tilt an otherwise horizontal projection across a box.
    if toward_target.length > _EPSILON:
        return toward_target.normalized()

    if nearest_normal is not None and nearest_normal.length > _EPSILON:
        return -nearest_normal.normalized()

    normal_sum = Vector()
    for face in target_faces:
        normal = _world_normal(obj, face)
        if normal.length > _EPSILON:
            normal_sum += normal
    if normal_sum.length > _EPSILON:
        return -normal_sum.normalized()

    current = _current_view_direction(space)
    if current.length > _EPSILON:
        return current
    return Vector((0.0, 0.0, -1.0))


def _point_on_projected_segment(point, first, second, direction, tolerance):
    """Test a world point against a cutter segment in projection space."""
    segment = second - first
    screen_segment = segment - direction * segment.dot(direction)
    length_squared = screen_segment.length_squared
    if length_squared <= _EPSILON:
        return False

    offset = point - first
    screen_offset = offset - direction * offset.dot(direction)
    factor = screen_offset.dot(screen_segment) / length_squared
    parameter_tolerance = tolerance / max(length_squared ** 0.5, tolerance)
    if factor < -parameter_tolerance or factor > 1.0 + parameter_tolerance:
        return False

    closest = screen_segment * factor
    return (screen_offset - closest).length_squared <= tolerance * tolerance


def _projected_seam_edges(obj, candidate_edges, cutter_segments, direction, tolerance):
    """Return new interior edges that lie on a projected cutter segment."""
    seams = []
    for edge in candidate_edges:
        if not edge.is_valid or len(edge.link_faces) < 2:
            continue
        first_point = obj.matrix_world @ edge.verts[0].co
        second_point = obj.matrix_world @ edge.verts[1].co
        if any(
            _point_on_projected_segment(
                first_point,
                cutter_first,
                cutter_second,
                direction,
                tolerance,
            )
            and _point_on_projected_segment(
                second_point,
                cutter_first,
                cutter_second,
                direction,
                tolerance,
            )
            for cutter_first, cutter_second in cutter_segments
        ):
            seams.append(edge)
    return seams


def _view_center_and_scale(source_points, target_points):
    points = list(source_points) + list(target_points)
    center = _average(points)
    radius = max((point - center).length for point in points) if points else 1.0
    return center, max(radius * 3.0, 0.1)


def _capture_view(space):
    region_3d = space.region_3d
    state = {
        "view_rotation": region_3d.view_rotation.copy(),
        "view_location": region_3d.view_location.copy(),
        "view_perspective": region_3d.view_perspective,
    }
    for name in (
        "ortho_scale",
        "view_distance",
        "view_camera_zoom",
        "view_camera_offset",
    ):
        value = getattr(region_3d, name, None)
        state[name] = value.copy() if hasattr(value, "copy") else value
    return state


def _set_projection_view(space, direction, center, ortho_scale):
    region_3d = space.region_3d
    region_3d.view_rotation = direction.to_track_quat('-Z', 'Y')
    region_3d.view_location = center
    region_3d.view_perspective = 'ORTHO'
    if hasattr(region_3d, "ortho_scale"):
        region_3d.ortho_scale = ortho_scale
    if hasattr(region_3d, "view_distance"):
        region_3d.view_distance = max(ortho_scale, 0.1)
    update = getattr(region_3d, "update", None)
    if update is not None and not bpy.app.background:
        update()


def _restore_view(space, state):
    region_3d = space.region_3d
    try:
        region_3d.view_rotation = state["view_rotation"]
        region_3d.view_location = state["view_location"]
        if state.get("ortho_scale") is not None and hasattr(region_3d, "ortho_scale"):
            region_3d.ortho_scale = state["ortho_scale"]
        if state.get("view_distance") is not None and hasattr(region_3d, "view_distance"):
            region_3d.view_distance = state["view_distance"]
        if state.get("view_camera_zoom") is not None and hasattr(region_3d, "view_camera_zoom"):
            region_3d.view_camera_zoom = state["view_camera_zoom"]
        if state.get("view_camera_offset") is not None and hasattr(region_3d, "view_camera_offset"):
            region_3d.view_camera_offset = state["view_camera_offset"]
        region_3d.view_perspective = state["view_perspective"]
        update = getattr(region_3d, "update", None)
        if update is not None and not bpy.app.background:
            update()
    except (AttributeError, ReferenceError, RuntimeError, TypeError, ValueError):
        # View restoration must not prevent the mesh operation from finishing.
        pass


def _redraw_and_update(context, area, region, space):
    try:
        with context.temp_override(
            window=getattr(context, "window", None),
            area=area,
            region=region,
            space_data=space,
        ):
            bpy.ops.wm.redraw_timer(type='DRAW_WIN', iterations=1)
    except (AttributeError, RuntimeError, TypeError, ValueError):
        pass
    context.view_layer.update()


def _run_knife_project(context, area, region, space, cut_through=True):
    if hasattr(context, "temp_override"):
        with context.temp_override(
            window=getattr(context, "window", None),
            area=area,
            region=region,
            space_data=space,
        ):
            return bpy.ops.mesh.knife_project(cut_through=cut_through)
    return bpy.ops.mesh.knife_project(cut_through=cut_through)


def _restore_object_selection(context, selected_objects, active_object):
    for obj in context.view_layer.objects:
        try:
            obj.select_set(False)
        except (ReferenceError, RuntimeError):
            continue
    for obj in selected_objects:
        try:
            if obj.name in bpy.data.objects:
                obj.select_set(True)
        except (ReferenceError, RuntimeError):
            continue
    try:
        if active_object is not None and active_object.name in bpy.data.objects:
            context.view_layer.objects.active = active_object
    except (ReferenceError, RuntimeError):
        pass


class MESH_OT_RCAD_EdgeKnifeProject(bpy.types.Operator):
    """Project selected edge wires, knife the target, and split the seam."""

    bl_idname = "mesh.rcad_edge_knife_project"
    bl_label = "Project Edge & Knife"
    bl_description = (
        "Project selected edge(s), knife the selected target faces, and disconnect the seam"
    )
    bl_options = {'REGISTER', 'UNDO'}

    cut_through: bpy.props.BoolProperty(
        name="Cut Through",
        description="Cut every target face along the projected edge, including hidden-by-depth faces",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        obj = _active_edit_mesh(context)
        return obj is not None

    def execute(self, context):
        obj = _active_edit_mesh(context)
        if obj is None:
            self.report({'ERROR'}, "Active Mesh Edit Mode is required.")
            return {'CANCELLED'}

        area, space, region = _view3d_context(context)
        if area is None:
            self.report({'ERROR'}, "Run Project Edge & Knife from a 3D Viewport.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        source_edges, target_faces = _selected_inputs(bm)
        if not source_edges:
            self.report(
                {'ERROR'},
                "Select at least one cutter edge that is not part of a selected target face.",
            )
            return {'CANCELLED'}

        if not target_faces:
            self.report({'ERROR'}, "Select one or more target faces.")
            return {'CANCELLED'}

        source_points = [
            obj.matrix_world @ vert.co
            for edge in source_edges
            for vert in edge.verts
        ]
        edge_points = [
            (obj.matrix_world @ edge.verts[0].co, obj.matrix_world @ edge.verts[1].co)
            for edge in source_edges
        ]
        target_points = _world_face_points(obj, target_faces)
        if not source_points or not target_points:
            self.report({'ERROR'}, "The selected edge or target mesh has no usable geometry.")
            return {'CANCELLED'}

        bmesh_state = _capture_bmesh_state(bm)
        before_edge_count = len(bm.edges)
        original_edges = set(bm.edges)
        selected_objects = tuple(context.selected_objects)
        active_object = context.view_layer.objects.active
        view_state = _capture_view(space)
        cutter = None
        success = False
        cut_count = 0
        seam_count = 0
        error_message = None

        try:
            _prepare_target(bm, target_faces)
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

            direction = _choose_view_direction(
                space,
                obj,
                source_points,
                target_faces,
                target_points,
            )
            center, ortho_scale = _view_center_and_scale(source_points, target_points)
            # A wire exactly coplanar with a target can be ignored by Knife
            # Project.  The offset is along the projection ray, so it changes
            # depth without changing the projected edge location.
            cutter_offset = direction * max(ortho_scale * 1.0e-4, 1.0e-6)
            cutter = _create_cutter(context, edge_points, cutter_offset)

            for scene_object in context.view_layer.objects:
                scene_object.select_set(False)
            obj.select_set(True)
            cutter.select_set(True)
            context.view_layer.objects.active = obj

            _set_projection_view(space, direction, center, ortho_scale)
            _redraw_and_update(context, area, region, space)

            result = _run_knife_project(
                context,
                area,
                region,
                space,
                cut_through=self.cut_through,
            )
            if 'FINISHED' not in result:
                raise RuntimeError("Blender's Knife Project operator was cancelled.")

            bm_after = bmesh.from_edit_mesh(obj.data)
            bm_after.verts.ensure_lookup_table()
            bm_after.edges.ensure_lookup_table()
            bm_after.faces.ensure_lookup_table()
            cut_count = max(0, len(bm_after.edges) - before_edge_count)
            new_edges = [edge for edge in bm_after.edges if edge not in original_edges]
            # Knife Project can clip one projected line into many short edges
            # on a dense mesh.  The endpoint test used here previously dropped
            # some of those valid pieces, so only part of the seam was split.
            # Every newly-created interior edge is part of this operation's
            # cut and must be disconnected from its neighboring faces.
            seam_edges = [
                edge
                for edge in new_edges
                if edge.is_valid and len(edge.link_faces) >= 2
            ]
            if seam_edges:
                bmesh.ops.split_edges(bm_after, edges=seam_edges)
                seam_count = len(seam_edges)

            _restore_visibility(bm_after, bmesh_state)
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
            success = cut_count > 0 and seam_count > 0
            if cut_count <= 0:
                error_message = "Knife Project finished, but created no new edges."
            elif seam_count <= 0:
                error_message = "The projected cut was created, but no interior seam could be split."
        except (ReferenceError, RuntimeError, TypeError, ValueError) as exc:
            error_message = str(exc)
        finally:
            if not success:
                try:
                    bm_restore = bmesh.from_edit_mesh(obj.data)
                    bm_restore.verts.ensure_lookup_table()
                    bm_restore.edges.ensure_lookup_table()
                    bm_restore.faces.ensure_lookup_table()
                    _restore_visibility(bm_restore, bmesh_state)
                    _restore_selection(bm_restore, bmesh_state)
                    bmesh.update_edit_mesh(
                        obj.data,
                        loop_triangles=False,
                        destructive=False,
                    )
                except (ReferenceError, RuntimeError, TypeError, ValueError):
                    pass

            _restore_view(space, view_state)

            if cutter is not None:
                try:
                    cutter_mesh = cutter.data
                    if cutter.name in bpy.data.objects:
                        bpy.data.objects.remove(cutter, do_unlink=True)
                    if cutter_mesh is not None and cutter_mesh.name in bpy.data.meshes:
                        bpy.data.meshes.remove(cutter_mesh)
                except (ReferenceError, RuntimeError):
                    pass

            _restore_object_selection(context, selected_objects, active_object)

        if error_message is not None:
            self.report({'WARNING'}, error_message)
            return {'CANCELLED'}

        self.report(
            {'INFO'},
            f"Projected {len(source_edges)} cutter edge(s) and split {seam_count} seam edge(s).",
        )
        return {'FINISHED'}


classes = (MESH_OT_RCAD_EdgeKnifeProject,)
