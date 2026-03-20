import bpy
# Relative imports to reach the engine
from ..modal_core import begin_modal, modal_arc_common, finish_modal
from ..modal_state import state 

class VIEW3D_OT_arc_overlay_preview(bpy.types.Operator):
    bl_idname = "view3d.arc_overlay_preview"
    bl_label = "Arc Tool (Rotating Compass)"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        print("--- INVOKE 1-POINT ARC ---") 
        # Set mode BEFORE starting modal so the Draw Manager sees it immediately
        state["tool_mode"] = "1POINT"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)