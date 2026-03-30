# operators package — traffic cop for resampler shape modes.

import bpy
import bmesh

from . import (
    bridged_open_loop,
    bridged_open_loop_with_corners,
    closed_loop,
    closed_loop_bridged,
    closed_loop_bridged_with_corners,
    corner,
    hole_in_mesh,
    hole_punch_face,
    hole_punch_solid,
    open_loop,
    pipe,
)
from ..mode_options import MODE_LABELS
from .resample_common import execute_anchored_logic
from .detection_utils import (
    get_selected_islands,
    get_kissing_chains,
)


class RCAD_OT_ResampleCurve(bpy.types.Operator):
    bl_idname = "rcad.resample_curve"
    bl_label = "Resample Curve"
    bl_options = {'REGISTER', 'UNDO'}

    direction: bpy.props.IntProperty(default=0)

    def _report_mode(self, label):
        self.report({'INFO'}, f"Mode: {label}")

    def execute(self, context):
        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        scene = context.scene
        mode = getattr(scene, "rcad_vertex_resampler_mode", 'NONE')

        if mode == 'NONE':
            self.report({'WARNING'}, "Pick a mesh type first.")
            return {'CANCELLED'}

        self._report_mode(MODE_LABELS[mode])

        if mode == 'CLOSED_LOOP_BRIDGED_WITH_CORNERS':
            return closed_loop_bridged_with_corners.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'CLOSED_LOOP_BRIDGED':
            return closed_loop_bridged.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'BRIDGED_OPEN_LOOP_WITH_CORNERS':
            return bridged_open_loop_with_corners.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'CORNER':
            return corner.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'BRIDGED_OPEN_LOOP':
            return bridged_open_loop.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'PIPE':
            return pipe.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'SOLID_HOLE_PUNCH':
            data = hole_punch_solid.detect(bm, report=self.report)
            if not data:
                return {'CANCELLED'}
            return hole_in_mesh.execute(
                bm, obj, self.direction, report=self.report, data=data
            )

        if mode == 'FACE_HOLE_PUNCH':
            data = hole_punch_face.detect(bm, report=self.report)
            if not data:
                return {'CANCELLED'}
            return hole_in_mesh.execute(
                bm, obj, self.direction, report=self.report, data=data
            )

        if mode == 'CLOSED_LOOP':
            return closed_loop.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'OPEN_LOOP':
            return open_loop.execute(
                bm, obj, self.direction, report=self.report
            )

        if mode == 'JUNCTION':
            return execute_anchored_logic(
                bm, obj, self.direction, mode='JUNCTION'
            )

        if mode == 'ANCHORED':
            return execute_anchored_logic(
                bm, obj, self.direction, mode='ANCHORED'
            )

        if mode == 'KISSING':
            islands = get_selected_islands(bm)
            kissing_chains = get_kissing_chains(
                bm,
                single_mode=(len(islands) == 1),
            )
            return execute_anchored_logic(
                bm,
                obj,
                self.direction,
                mode='KISSING',
                precalc_chains=kissing_chains,
            )

        self.report({'ERROR'}, f"Unsupported mesh type: {mode}")
        return {'CANCELLED'}


classes = (RCAD_OT_ResampleCurve,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
