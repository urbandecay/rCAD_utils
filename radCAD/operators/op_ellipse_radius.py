import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_ellipse_radius(bpy.types.Operator):
    bl_idname = "view3d.ellipse_radius"
    bl_label = "Ellipse (From Radius)"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "ELLIPSE_RADIUS"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)