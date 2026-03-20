import bpy
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_point_by_arcs(bpy.types.Operator):
    bl_idname = "view3d.point_by_arcs"
    bl_label = "Point by Arcs"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "POINT_BY_ARCS"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)