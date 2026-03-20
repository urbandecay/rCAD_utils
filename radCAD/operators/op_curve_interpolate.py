import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_curve_interpolate(bpy.types.Operator):
    bl_idname = "view3d.curve_interpolate"
    bl_label = "Interpolate Curve Tool"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "CURVE_INTERPOLATE"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)