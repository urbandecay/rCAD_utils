# anchor_overlay.py — Viewport overlay for anchor/seam points.

import bpy
import blf
import gpu
from bpy.types import Operator, SpaceView3D
from bpy_extras.view3d_utils import location_3d_to_region_2d
from gpu_extras.batch import batch_for_shader

_POINT_HANDLER = None
_TEXT_HANDLER = None
_ACTIVE_POINTS = []
_PENDING_POINTS = []
_ACTIVE_SEGMENTS = []
_PENDING_SEGMENTS = []


def _tag_redraw():
    wm = getattr(bpy.context, "window_manager", None)
    if wm is None:
        return
    for window in wm.windows:
        screen = window.screen
        if screen is None:
            continue
        for area in screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()


def _ensure_handlers():
    global _POINT_HANDLER, _TEXT_HANDLER
    if _POINT_HANDLER is None:
        _POINT_HANDLER = SpaceView3D.draw_handler_add(
            _draw_points,
            (),
            'WINDOW',
            'POST_VIEW',
        )
    if _TEXT_HANDLER is None:
        _TEXT_HANDLER = SpaceView3D.draw_handler_add(
            _draw_labels,
            (),
            'WINDOW',
            'POST_PIXEL',
        )


def _remove_handlers():
    global _POINT_HANDLER, _TEXT_HANDLER
    if _POINT_HANDLER is not None:
        SpaceView3D.draw_handler_remove(_POINT_HANDLER, 'WINDOW')
        _POINT_HANDLER = None
    if _TEXT_HANDLER is not None:
        SpaceView3D.draw_handler_remove(_TEXT_HANDLER, 'WINDOW')
        _TEXT_HANDLER = None


def _draw_points():
    if not _ACTIVE_SEGMENTS:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')

    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('NONE')
    if _ACTIVE_SEGMENTS:
        coords = []
        for item in _ACTIVE_SEGMENTS:
            coords.extend((item['a'], item['b']))
        batch = batch_for_shader(shader, 'LINES', {"pos": coords})
        gpu.state.line_width_set(2.0)
        shader.bind()
        shader.uniform_float("color", (0.1, 1.0, 1.0, 0.9))
        batch.draw(shader)
        gpu.state.line_width_set(1.0)
    gpu.state.depth_test_set('LESS_EQUAL')
    gpu.state.blend_set('NONE')


def _draw_labels():
    return


def begin_capture():
    _PENDING_POINTS.clear()
    _PENDING_SEGMENTS.clear()


def add_points(points, label_prefix="A"):
    return


def add_segments(segments):
    for point_a, point_b in segments:
        _PENDING_SEGMENTS.append({
            'a': point_a,
            'b': point_b,
        })


def end_capture():
    _ACTIVE_POINTS[:] = list(_PENDING_POINTS)
    _ACTIVE_SEGMENTS[:] = list(_PENDING_SEGMENTS)
    _PENDING_POINTS.clear()
    _PENDING_SEGMENTS.clear()
    if _ACTIVE_POINTS or _ACTIVE_SEGMENTS:
        _ensure_handlers()
    else:
        _remove_handlers()
    _tag_redraw()


def clear_overlay():
    _PENDING_POINTS.clear()
    _PENDING_SEGMENTS.clear()
    _ACTIVE_POINTS.clear()
    _ACTIVE_SEGMENTS.clear()
    _remove_handlers()
    _tag_redraw()


class RCAD_OT_ClearAnchorOverlay(Operator):
    bl_idname = "rcad.clear_anchor_overlay"
    bl_label = "Clear Anchor Overlay"

    def execute(self, context):
        clear_overlay()
        return {'FINISHED'}


classes = (
    RCAD_OT_ClearAnchorOverlay,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    clear_overlay()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
