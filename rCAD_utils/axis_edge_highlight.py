"""Viewport overlay for identifying edges parallel to the world axes."""

import math

import bpy
import bmesh
from bpy.props import BoolProperty, EnumProperty, FloatProperty
from mathutils import Vector


_DRAW_HANDLER = None
_SHADER = None

_AXIS_X = Vector((1.0, 0.0, 0.0))
_AXIS_Y = Vector((0.0, 1.0, 0.0))
_AXIS_Z = Vector((0.0, 0.0, 1.0))

_X_COLOR = (1.0, 0.08, 0.08, 1.0)
_Y_COLOR = (0.08, 1.0, 0.12, 1.0)
_Z_COLOR = (0.12, 0.35, 1.0, 1.0)
_OTHER_COLOR = (0.65, 0.65, 0.65, 0.28)


def _tag_viewports():
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        if screen is None:
            continue
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


def _active_mesh_edges(obj):
    """Return endpoints plus the local normals of each edge's linked faces."""
    mesh = obj.data

    if obj.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(mesh)
        return [
            (
                edge.verts[0].co.copy(),
                edge.verts[1].co.copy(),
                [face.normal.copy() for face in edge.link_faces],
            )
            for edge in bm.edges
        ]

    vertices = mesh.vertices
    edge_lookup = {
        tuple(sorted(edge.vertices)): edge.index
        for edge in mesh.edges
    }
    edge_normals = [[] for _ in mesh.edges]
    for polygon in mesh.polygons:
        for edge_key in polygon.edge_keys:
            edge_index = edge_lookup.get(tuple(sorted(edge_key)))
            if edge_index is not None:
                edge_normals[edge_index].append(polygon.normal.copy())

    return [
        (
            vertices[edge.vertices[0]].co.copy(),
            vertices[edge.vertices[1]].co.copy(),
            edge_normals[edge.index],
        )
        for edge in mesh.edges
    ]


def _axis_for_direction(direction, tolerance_degrees):
    if direction.length == 0.0:
        return None

    direction.normalize()
    threshold = math.cos(math.radians(tolerance_degrees))
    candidates = (
        ('X', abs(direction.dot(_AXIS_X))),
        ('Y', abs(direction.dot(_AXIS_Y))),
        ('Z', abs(direction.dot(_AXIS_Z))),
    )
    axis, amount = max(candidates, key=lambda item: item[1])
    return axis if amount >= threshold else None


def _get_shader():
    global _SHADER
    if _SHADER is not None:
        return _SHADER

    import gpu

    try:
        _SHADER = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
    except Exception:
        _SHADER = gpu.shader.from_builtin('UNIFORM_COLOR')
    return _SHADER


def _draw_lines(coords, color):
    if not coords:
        return

    import gpu
    from gpu_extras.batch import batch_for_shader

    shader = _get_shader()
    batch = batch_for_shader(shader, 'LINES', {"pos": coords})
    shader.bind()
    shader.uniform_float("color", color)
    batch.draw(shader)


def _normal_overlay_offset(obj):
    """Return a small camera-facing offset that prevents surface z-fighting."""
    try:
        region_data = bpy.context.region_data
        if region_data is None:
            return Vector()

        view_origin = region_data.view_matrix.inverted().to_translation()
        direction = view_origin - obj.matrix_world.translation
        if direction.length <= 1.0e-8:
            direction = -(region_data.view_rotation @ Vector((0.0, 0.0, -1.0)))
        direction.normalize()

        # Scale the bias with the object so it works for both small and large
        # models, while remaining far smaller than the visible line width.
        amount = max(obj.dimensions.length, 1.0) * 1.0e-5
        return direction * amount
    except Exception:
        return Vector()


def _view_toward_camera():
    try:
        region_data = bpy.context.region_data
        if region_data is None:
            return None
        direction = region_data.view_rotation @ Vector((0.0, 0.0, 1.0))
        if direction.length <= 1.0e-8:
            return None
        return direction.normalized()
    except Exception:
        return None


def _edge_has_front_face(local_normals, normal_matrix, toward_camera):
    if toward_camera is None or not local_normals:
        return False

    for local_normal in local_normals:
        world_normal = normal_matrix @ local_normal
        if world_normal.length > 1.0e-8:
            # Include silhouette faces as well as faces turned toward the view.
            if world_normal.normalized().dot(toward_camera) >= -0.02:
                return True
    return False


def _draw_overlay():
    scene = bpy.context.scene
    if not getattr(scene, 'rcad_axis_highlight_enabled', False):
        return

    obj = bpy.context.active_object
    if obj is None or obj.type != 'MESH':
        return

    try:
        edge_pairs = _active_mesh_edges(obj)
    except Exception:
        return

    tolerance = getattr(scene, 'rcad_axis_highlight_tolerance', 1.0)
    show_other = getattr(scene, 'rcad_axis_highlight_show_other', False)
    depth_mode = getattr(scene, 'rcad_axis_highlight_depth_mode', 'NORMAL')
    line_width = getattr(scene, 'rcad_axis_highlight_line_width', 3.0)
    matrix = obj.matrix_world
    overlay_offset = (
        _normal_overlay_offset(obj)
        if depth_mode == 'NORMAL'
        else Vector()
    )

    axis_coords = {'X': [], 'Y': [], 'Z': []}
    other_coords = []
    surface_coords = {'X': [], 'Y': [], 'Z': []}
    surface_other_coords = []
    toward_camera = _view_toward_camera() if depth_mode == 'NORMAL' else None
    normal_matrix = matrix.to_3x3().inverted().transposed()
    normal_offset_amount = (
        max(obj.dimensions.length, 1.0) * 1.0e-5
        * max(line_width / 3.0, 1.0)
    )

    for local_a, local_b, local_normals in edge_pairs:
        world_a = matrix @ local_a
        world_b = matrix @ local_b
        if depth_mode == 'NORMAL':
            edge_normal = Vector()
            for local_normal in local_normals:
                world_normal = normal_matrix @ local_normal
                if world_normal.length > 1.0e-8:
                    edge_normal += world_normal.normalized()
            if edge_normal.length > 1.0e-8:
                edge_normal.normalize()
                edge_offset = edge_normal * normal_offset_amount
            else:
                edge_offset = overlay_offset
            world_a += edge_offset
            world_b += edge_offset
        axis = _axis_for_direction(world_b - world_a, tolerance)
        if axis is None:
            if show_other:
                other_coords.extend((world_a, world_b))
                if _edge_has_front_face(local_normals, normal_matrix, toward_camera):
                    surface_other_coords.extend((world_a, world_b))
        else:
            axis_coords[axis].extend((world_a, world_b))
            if _edge_has_front_face(local_normals, normal_matrix, toward_camera):
                surface_coords[axis].extend((world_a, world_b))

    import gpu

    try:
        gpu.state.blend_set('ALPHA')
        gpu.state.line_width_set(line_width)
        try:
            gpu.state.depth_test_set('NONE' if depth_mode == 'XRAY' else 'LESS_EQUAL')
            gpu.state.depth_mask_set(False)
        except Exception:
            pass

        _draw_lines(axis_coords['X'], _X_COLOR)
        _draw_lines(axis_coords['Y'], _Y_COLOR)
        _draw_lines(axis_coords['Z'], _Z_COLOR)
        _draw_lines(other_coords, _OTHER_COLOR)

        if depth_mode == 'NORMAL':
            # A wide 3D line can still be clipped by either face meeting at an
            # edge. Put edges belonging to a front-facing surface over the
            # mesh so the whole requested thickness remains visible.
            gpu.state.depth_test_set('NONE')
            _draw_lines(surface_coords['X'], _X_COLOR)
            _draw_lines(surface_coords['Y'], _Y_COLOR)
            _draw_lines(surface_coords['Z'], _Z_COLOR)
            _draw_lines(surface_other_coords, _OTHER_COLOR)
    finally:
        try:
            gpu.state.line_width_set(1.0)
            gpu.state.blend_set('NONE')
            gpu.state.depth_test_set('LESS_EQUAL')
            gpu.state.depth_mask_set(True)
        except Exception:
            pass


def _start_overlay():
    global _DRAW_HANDLER
    if _DRAW_HANDLER is None:
        _DRAW_HANDLER = bpy.types.SpaceView3D.draw_handler_add(
            _draw_overlay, (), 'WINDOW', 'POST_VIEW'
        )
    _tag_viewports()


def _stop_overlay():
    global _DRAW_HANDLER, _SHADER
    if _DRAW_HANDLER is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_DRAW_HANDLER, 'WINDOW')
        _DRAW_HANDLER = None
    _SHADER = None
    _tag_viewports()


class RCAD_OT_ToggleAxisEdgeHighlight(bpy.types.Operator):
    """Show or hide world-axis colors on the active mesh's edges."""

    bl_idname = 'rcad.toggle_axis_edge_highlight'
    bl_label = 'Highlight Axis Edges'
    bl_description = (
        'Color edges parallel to world X, Y, and Z on the active mesh '
        '(X red, Y green, Z blue)'
    )
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH'

    def execute(self, context):
        scene = context.scene
        if scene.rcad_axis_highlight_enabled:
            scene.rcad_axis_highlight_enabled = False
            _stop_overlay()
            self.report({'INFO'}, 'Axis edge highlighting disabled')
            return {'FINISHED'}

        if context.active_object is None or context.active_object.type != 'MESH':
            self.report({'WARNING'}, 'Select a mesh object first')
            return {'CANCELLED'}

        scene.rcad_axis_highlight_enabled = True
        _start_overlay()
        self.report({'INFO'}, 'Axis edge highlighting enabled')
        return {'FINISHED'}


class RCAD_OT_AxisEdgeHighlightPreferences(bpy.types.Operator):
    """Edit Axis Edge Highlighter display settings in a gear popup."""

    bl_idname = 'rcad.axis_edge_highlight_preferences'
    bl_label = 'Axis Edge Highlighter Preferences'
    bl_options = {'REGISTER'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=360)

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.use_property_split = True
        layout.use_property_decorate = False

        layout.label(text='Display')
        layout.prop(scene, 'rcad_axis_highlight_depth_mode', text='Visibility', expand=True)
        layout.prop(scene, 'rcad_axis_highlight_line_width', text='Line Thickness')

        layout.separator()
        layout.label(text='Detection')
        layout.prop(scene, 'rcad_axis_highlight_tolerance', text='Axis Tolerance (degrees)')
        layout.prop(scene, 'rcad_axis_highlight_show_other', text='Show Other Edges')

    def execute(self, context):
        _tag_viewports()
        return {'FINISHED'}


class RCAD_OT_ClearAxisEdgeHighlight(bpy.types.Operator):
    """Turn off the axis edge overlay."""

    bl_idname = 'rcad.clear_axis_edge_highlight'
    bl_label = 'Clear Axis Highlight'
    bl_options = {'REGISTER'}

    def execute(self, context):
        context.scene.rcad_axis_highlight_enabled = False
        _stop_overlay()
        return {'FINISHED'}


classes = (
    RCAD_OT_ToggleAxisEdgeHighlight,
    RCAD_OT_AxisEdgeHighlightPreferences,
    RCAD_OT_ClearAxisEdgeHighlight,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.rcad_axis_highlight_enabled = BoolProperty(
        name='Axis Edge Highlight',
        description='Show world-axis colors on the active mesh edges',
        default=False,
    )
    bpy.types.Scene.rcad_axis_highlight_tolerance = FloatProperty(
        name='Axis Tolerance (degrees)',
        description='Maximum angle from an axis for an edge to be highlighted',
        min=0.01,
        max=45.0,
        default=1.0,
        update=lambda self, context: _tag_viewports(),
    )
    bpy.types.Scene.rcad_axis_highlight_depth_mode = EnumProperty(
        name='Visibility',
        description='Draw only visible edges or draw through the mesh',
        items=(
            (
                'NORMAL',
                'Normal',
                'Show visible highlighted edges with a surface-safe overlay bias',
            ),
            ('XRAY', 'X-Ray', 'Draw highlighted edges through the mesh'),
        ),
        default='NORMAL',
        update=lambda self, context: _tag_viewports(),
    )
    bpy.types.Scene.rcad_axis_highlight_line_width = FloatProperty(
        name='Line Thickness',
        description='Viewport thickness of highlighted edges',
        min=1.0,
        max=12.0,
        default=3.0,
        update=lambda self, context: _tag_viewports(),
    )
    bpy.types.Scene.rcad_axis_highlight_show_other = BoolProperty(
        name='Show Other Edges',
        description='Also draw edges that are not parallel to a world axis',
        default=False,
        update=lambda self, context: _tag_viewports(),
    )


def unregister():
    _stop_overlay()
    for prop in (
        'rcad_axis_highlight_show_other',
        'rcad_axis_highlight_line_width',
        'rcad_axis_highlight_depth_mode',
        'rcad_axis_highlight_tolerance',
        'rcad_axis_highlight_enabled',
    ):
        if hasattr(bpy.types.Scene, prop):
            delattr(bpy.types.Scene, prop)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
