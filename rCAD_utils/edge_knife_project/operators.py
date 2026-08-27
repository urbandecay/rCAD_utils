"""Viewport-managed knife projection for selected Edit Mode mesh edges."""

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree


_EPSILON = 1.0e-10
_TEMP_OBJECT_NAME = "__rcad_edge_knife_cutter__"
_PREVIEW_STATE = {
    "handle": None,
    "data": None,
    "shader": None,
}


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


def _projected_segment_factor(point, first, second, direction):
    """Return a point's position along a cutter after removing depth."""
    segment = second - first
    screen_segment = segment - direction * segment.dot(direction)
    length_squared = screen_segment.length_squared
    if length_squared <= _EPSILON:
        return None

    offset = point - first
    screen_offset = offset - direction * offset.dot(direction)
    factor = screen_offset.dot(screen_segment) / length_squared
    distance = (screen_offset - screen_segment * factor).length
    return factor, distance


def _triangle_plane_segment(triangle, plane_point, plane_normal, tolerance):
    """Return the part of a triangle cut by a plane, if it has one."""
    intersections = []
    values = [
        (point - plane_point).dot(plane_normal)
        for point in triangle
    ]
    for index, first in enumerate(triangle):
        second = triangle[(index + 1) % 3]
        first_value = values[index]
        second_value = values[(index + 1) % 3]
        if abs(first_value) <= tolerance:
            intersections.append(first)
        if (
            (first_value < -tolerance and second_value > tolerance)
            or (first_value > tolerance and second_value < -tolerance)
        ):
            factor = first_value / (first_value - second_value)
            intersections.append(first.lerp(second, factor))

    unique = []
    tolerance_squared = tolerance * tolerance
    for point in intersections:
        if not any((point - other).length_squared <= tolerance_squared for other in unique):
            unique.append(point)
    if len(unique) < 2:
        return None

    first = unique[0]
    second = max(unique[1:], key=lambda point: (point - first).length_squared)
    if (second - first).length_squared <= tolerance_squared:
        return None
    return first, second


def _preview_projected_segments(
    obj,
    target_faces,
    cutter_segments,
    direction,
    tolerance,
):
    """Build the visible 3D line segments Knife Project will create.

    Each cutter edge and the projection direction define a plane.  Intersect
    that plane with the selected target triangles, then clip the result to the
    finite cutter edge.  This keeps the preview aligned with the same
    projection geometry without changing the mesh.
    """
    preview_segments = []
    for first, second in cutter_segments:
        cutter_vector = second - first
        plane_normal = cutter_vector.cross(direction)
        if plane_normal.length <= _EPSILON:
            continue
        plane_normal.normalize()

        for face in target_faces:
            face_points = [obj.matrix_world @ vert.co for vert in face.verts]
            if len(face_points) < 3:
                continue
            for index in range(1, len(face_points) - 1):
                triangle = (
                    face_points[0],
                    face_points[index],
                    face_points[index + 1],
                )
                result = _triangle_plane_segment(
                    triangle,
                    first,
                    plane_normal,
                    tolerance,
                )
                if result is None:
                    continue
                point_a, point_b = result
                factor_a = _projected_segment_factor(
                    point_a,
                    first,
                    second,
                    direction,
                )
                factor_b = _projected_segment_factor(
                    point_b,
                    first,
                    second,
                    direction,
                )
                if factor_a is None or factor_b is None:
                    continue
                factor_a = factor_a[0]
                factor_b = factor_b[0]
                if factor_a > factor_b:
                    point_a, point_b = point_b, point_a
                    factor_a, factor_b = factor_b, factor_a

                if factor_b < 0.0 or factor_a > 1.0:
                    continue
                clipped_a = max(factor_a, 0.0)
                clipped_b = min(factor_b, 1.0)
                if clipped_b - clipped_a <= _EPSILON:
                    continue
                factor_span = factor_b - factor_a
                if factor_span <= _EPSILON:
                    continue
                original_a, original_b = point_a, point_b
                point_a = original_a.lerp(
                    original_b,
                    (clipped_a - factor_a) / factor_span,
                )
                point_b = original_a.lerp(
                    original_b,
                    (clipped_b - factor_a) / factor_span,
                )
                if (point_b - point_a).length_squared <= tolerance * tolerance:
                    continue
                preview_segments.extend((point_a, point_b))
    return preview_segments


def _preview_direction_segments(source_points, target_points, direction, scale):
    """Return an arrow showing the direction used for projection."""
    source_center = _average(source_points)
    target_center = _average(target_points)
    projected_distance = abs((target_center - source_center).dot(direction))
    arrow_length = max(projected_distance, scale * 0.35, 0.1)
    arrow_start = source_center
    arrow_end = arrow_start + direction * arrow_length
    head_size = min(max(scale * 0.045, 0.03), arrow_length * 0.35)

    side = direction.cross(Vector((0.0, 0.0, 1.0)))
    if side.length <= _EPSILON:
        side = direction.cross(Vector((0.0, 1.0, 0.0)))
    if side.length <= _EPSILON:
        return [arrow_start, arrow_end]
    side.normalize()
    arrow_back = arrow_end - direction * head_size
    return [
        arrow_start,
        arrow_end,
        arrow_end,
        arrow_back + side * head_size * 0.55,
        arrow_end,
        arrow_back - side * head_size * 0.55,
    ]


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


def _draw_preview_lines(gpu, batch_for_shader, shader, points, color, width):
    if len(points) < 2:
        return
    batch = batch_for_shader(shader, 'LINES', {"pos": points})
    shader.bind()
    shader.uniform_float("color", color)
    gpu.state.line_width_set(width)
    batch.draw(shader)


def _draw_preview_callback():
    data = _PREVIEW_STATE["data"]
    if not data:
        return
    gpu = None
    try:
        import gpu
        from gpu_extras.batch import batch_for_shader

        shader = _PREVIEW_STATE["shader"]
        if shader is None:
            shader = gpu.shader.from_builtin('UNIFORM_COLOR')
            _PREVIEW_STATE["shader"] = shader

        gpu.state.blend_set('ALPHA')
        gpu.state.depth_test_set('NONE')
        _draw_preview_lines(
            gpu,
            batch_for_shader,
            shader,
            data["source_lines"],
            (1.0, 0.65, 0.05, 1.0),
            2.0,
        )
        _draw_preview_lines(
            gpu,
            batch_for_shader,
            shader,
            data["cut_lines"],
            data["cut_color"],
            4.0,
        )
        _draw_preview_lines(
            gpu,
            batch_for_shader,
            shader,
            data["direction_lines"],
            (1.0, 0.25, 0.05, 1.0),
            3.0,
        )
    except (AttributeError, RuntimeError, TypeError, ValueError):
        # Drawing is best-effort; it must never interrupt the modal tool.
        pass
    finally:
        if gpu is not None:
            try:
                gpu.state.line_width_set(1.0)
                gpu.state.depth_test_set('LESS_EQUAL')
                gpu.state.blend_set('NONE')
            except (AttributeError, RuntimeError, TypeError, ValueError):
                pass


def _remove_preview_handler(context):
    handle = _PREVIEW_STATE["handle"]
    if handle is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(handle, 'WINDOW')
        except (ReferenceError, RuntimeError, TypeError, ValueError):
            pass
        _PREVIEW_STATE["handle"] = None
    _PREVIEW_STATE["data"] = None
    _PREVIEW_STATE["shader"] = None
    screen = getattr(context, "screen", None)
    for area in getattr(screen, "areas", ()):
        if area.type == 'VIEW_3D':
            area.tag_redraw()


def stop_preview(context=None):
    """Remove an active preview, including when the add-on is reloaded."""
    _remove_preview_handler(context or bpy.context)


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
            seam_tolerance = max(ortho_scale * 1.0e-3, 1.0e-5)
            # A straight projection can land exactly on edges that already
            # existed before Knife Project ran.  Looking only at new edges
            # misses that seam, even though the cut itself succeeds.  Search
            # the final mesh so both new cut segments and pre-existing seam
            # edges are disconnected.
            seam_edges = _projected_seam_edges(
                obj,
                bm_after.edges,
                edge_points,
                direction,
                seam_tolerance,
            )
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


class MESH_OT_RCAD_EdgeKnifePreview(bpy.types.Operator):
    """Preview the projected cut and wait for confirmation."""

    bl_idname = "mesh.rcad_edge_knife_preview"
    bl_label = "Preview Cut"
    bl_description = (
        "Show the projected cut line and direction; click to apply or press Esc to cancel"
    )
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return _active_edit_mesh(context) is not None

    def _tag_redraw(self, context):
        screen = getattr(context, "screen", None)
        for area in getattr(screen, "areas", ()):
            if area.type == 'VIEW_3D':
                area.tag_redraw()

    def _build_preview_data(self, context):
        obj = _active_edit_mesh(context)
        if obj is None:
            raise RuntimeError("Active Mesh Edit Mode is required.")

        area, space, _region = _view3d_context(context)
        if area is None:
            raise RuntimeError("Run Preview Cut from a 3D Viewport.")

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        source_edges, target_faces = _selected_inputs(bm)
        if not source_edges:
            raise RuntimeError(
                "Select at least one cutter edge that is not part of a selected target face."
            )
        if not target_faces:
            raise RuntimeError("Select one or more target faces.")

        source_points = [
            obj.matrix_world @ vert.co
            for edge in source_edges
            for vert in edge.verts
        ]
        cutter_segments = [
            (obj.matrix_world @ edge.verts[0].co,
             obj.matrix_world @ edge.verts[1].co)
            for edge in source_edges
        ]
        target_points = _world_face_points(obj, target_faces)
        if not source_points or not target_points:
            raise RuntimeError("The selected geometry has no usable points.")

        direction = _choose_view_direction(
            space,
            obj,
            source_points,
            target_faces,
            target_points,
        )
        _center, scale = _view_center_and_scale(source_points, target_points)
        tolerance = max(scale * 1.0e-5, 1.0e-6)
        cut_lines = _preview_projected_segments(
            obj,
            target_faces,
            cutter_segments,
            direction,
            tolerance,
        )
        return {
            "source_lines": [point for segment in cutter_segments for point in segment],
            "cut_lines": cut_lines,
            "direction_lines": _preview_direction_segments(
                source_points,
                target_points,
                direction,
                scale,
            ),
            "cut_color": (
                (0.1, 1.0, 0.15, 1.0)
                if cut_lines
                else (1.0, 0.1, 0.05, 1.0)
            ),
        }

    def _refresh_preview(self, context):
        try:
            _PREVIEW_STATE["data"] = self._build_preview_data(context)
        except (ReferenceError, RuntimeError, TypeError, ValueError) as exc:
            _PREVIEW_STATE["data"] = {
                "source_lines": [],
                "cut_lines": [],
                "direction_lines": [],
                "cut_color": (1.0, 0.1, 0.05, 1.0),
                "error": str(exc),
            }
        self._tag_redraw(context)

    def invoke(self, context, _event):
        if _PREVIEW_STATE["handle"] is not None:
            _remove_preview_handler(context)
        try:
            _PREVIEW_STATE["data"] = self._build_preview_data(context)
        except (ReferenceError, RuntimeError, TypeError, ValueError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        _PREVIEW_STATE["handle"] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_preview_callback,
            (),
            'WINDOW',
            'POST_VIEW',
        )
        context.window_manager.modal_handler_add(self)
        self._tag_redraw(context)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'ESC', 'RIGHTMOUSE'} and event.value == 'PRESS':
            _remove_preview_handler(context)
            return {'CANCELLED'}

        if event.type in {'LEFTMOUSE', 'RET', 'NUMPAD_ENTER'} and event.value == 'PRESS':
            _remove_preview_handler(context)
            return bpy.ops.mesh.rcad_edge_knife_project('EXEC_DEFAULT')

        if event.type in {
            'MOUSEMOVE',
            'MIDDLEMOUSE',
            'WHEELUPMOUSE',
            'WHEELDOWNMOUSE',
            'NDOF_MOTION',
        }:
            self._refresh_preview(context)
        return {'PASS_THROUGH'}


classes = (MESH_OT_RCAD_EdgeKnifeProject, MESH_OT_RCAD_EdgeKnifePreview)
