# operators package — traffic cop for resampler shape modes.

import bpy
import bmesh

from . import closed_loop, hole_in_mesh, pipe, open_loop
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

    def execute(self, context):
        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()

        hole_data = hole_in_mesh.detect(bm)
        if hole_data and hole_data.get('groups'):
            return hole_in_mesh.execute(
                bm,
                obj,
                self.direction,
                report=self.report,
                data=hole_data,
            )

        closed_data = closed_loop.detect(bm)
        if closed_data:
            return closed_loop.execute(bm, obj, self.direction, report=self.report)

        pipe_data = pipe.detect(bm)
        if pipe_data:
            return pipe.execute(bm, obj, self.direction)

        open_data = open_loop.detect(bm)
        if open_data:
            return open_loop.execute(bm, obj, self.direction, report=self.report)

        if check_selected_junction(bm):
            return execute_anchored_logic(bm, obj, self.direction, mode='JUNCTION')
        elif check_if_anchored(bm):
            return execute_anchored_logic(bm, obj, self.direction, mode='ANCHORED')
        else:
            islands = get_selected_islands(bm)
            is_single_mode = (len(islands) == 1)
            kissing_chains = get_kissing_chains(bm, single_mode=is_single_mode)

            if kissing_chains:
                return execute_anchored_logic(
                    bm,
                    obj,
                    self.direction,
                    mode='KISSING',
                    precalc_chains=kissing_chains,
                )
            return execute_floating_logic(bm, obj, self.direction, islands=islands)


classes = (RCAD_OT_ResampleCurve,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
