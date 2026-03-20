import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_point_center(bpy.types.Operator):
    bl_idname = "view3d.point_center"
    bl_label = "Point Center"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "POINT_CENTER"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)
