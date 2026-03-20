import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_rectangle_cen_cor(bpy.types.Operator):
    bl_idname = "view3d.rectangle_cen_cor"
    bl_label = "Rectangle (Center-Corner)"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "RECTANGLE_CENTER_CORNER"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)