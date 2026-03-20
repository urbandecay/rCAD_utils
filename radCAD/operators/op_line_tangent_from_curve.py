import bpy
from ..modal_state import state

class VIEW3D_OT_line_tangent_from_curve(bpy.types.Operator):
    bl_idname = "view3d.line_tangent_from_curve"
    bl_label = "Line Tangent from Curve"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        state["tool_mode"] = "LINE_TANGENT_FROM_CURVE"
        return bpy.ops.view3d.radcad_modal('INVOKE_DEFAULT')