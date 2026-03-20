import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_circle_3pt(bpy.types.Operator):
    bl_idname = "view3d.circle_3pt"
    bl_label = "3 Point Circle"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "CIRCLE_3POINT"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)