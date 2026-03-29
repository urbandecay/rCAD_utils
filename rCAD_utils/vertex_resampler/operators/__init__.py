# operators package — traffic cop for resampler shape modes.

import bpy
import bmesh

from . import (
    bridged_open_loop,
    closed_loop,
    closed_loop_bridged,
    corner,
    hole_in_mesh,
    open_loop,
    pipe,
)
from .resample_common import execute_anchored_logic, execute_floating_logic
from .detection_utils import (
    get_selected_islands,
    get_kissing_chains,
    check_if_anchored,
    check_selected_junction,
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

        corner_data = corner.detect(bm)
        if corner_data:
            self._report_mode(corner_data['mode_label'])
            return corner.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=corner_data,
            )

        bridged_open_data = bridged_open_loop.detect(bm)
        if bridged_open_data:
            self._report_mode(bridged_open_data['mode_label'])
            return bridged_open_loop.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=bridged_open_data,
            )

        bridged_closed_data = closed_loop_bridged.detect(bm)
        if bridged_closed_data:
            self._report_mode(bridged_closed_data['mode_label'])
            return closed_loop_bridged.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=bridged_closed_data,
            )

        hole_data = hole_in_mesh.detect(bm, report=self.report)
        if hole_data and (
            hole_data.get('groups') or hole_data.get('invalid_components')
        ):
            self._report_mode(hole_data.get('mode_label', 'Hole punch'))
            return hole_in_mesh.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=hole_data,
            )

        closed_data = closed_loop.detect(bm)
        if closed_data:
            self._report_mode(closed_data['mode_label'])
            return closed_loop.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=closed_data,
            )

        pipe_data = pipe.detect(bm)
        if pipe_data:
            self._report_mode("Pipe")
            return pipe.execute(bm, obj, self.direction)

        open_data = open_loop.detect(bm)
        if open_data:
            self._report_mode(open_data['mode_label'])
            return open_loop.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=open_data,
            )

        if check_selected_junction(bm):
            self._report_mode("Junction")
            return execute_anchored_logic(bm, obj, self.direction, mode='JUNCTION')
        elif check_if_anchored(bm):
            self._report_mode("Anchored")
            return execute_anchored_logic(bm, obj, self.direction, mode='ANCHORED')
        else:
            islands = get_selected_islands(bm)
            is_single_mode = (len(islands) == 1)
            kissing_chains = get_kissing_chains(bm, single_mode=is_single_mode)

            if kissing_chains:
                self._report_mode("Kissing")
                return execute_anchored_logic(
                    bm,
                    obj,
                    self.direction,
                    mode='KISSING',
                    precalc_chains=kissing_chains,
                )
            self._report_mode("Closed loop")
            return execute_floating_logic(bm, obj, self.direction, islands=islands)


classes = (RCAD_OT_ResampleCurve,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
