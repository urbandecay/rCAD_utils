import bpy
# Relative imports to reach the engine
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state

class VIEW3D_OT_arc_2pt(bpy.types.Operator):
    bl_idname = "view3d.arc_2pt"
    bl_label = "2-Point Arc Tool"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        # Set mode BEFORE starting the modal so the Manager loads the right worker
        state["tool_mode"] = "2POINT"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)