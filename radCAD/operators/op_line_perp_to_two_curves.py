import bpy
from ..modal_state import state

class VIEW3D_OT_line_perp_to_two_curves(bpy.types.Operator):
    bl_idname = "view3d.line_perp_to_two_curves"
    bl_label = "Line Perpendicular to Two Curves"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        state["tool_mode"] = "LINE_PERP_TO_TWO_CURVES"
        return bpy.ops.view3d.radcad_modal('INVOKE_DEFAULT')