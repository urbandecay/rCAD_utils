import bpy
from ..modal_state import state

class VIEW3D_OT_line_tan_tan(bpy.types.Operator):
    bl_idname = "view3d.line_tan_tan"
    bl_label = "Line Tangent to Two Curves"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        state["tool_mode"] = "LINE_TAN_TAN"
        return bpy.ops.view3d.radcad_modal('INVOKE_DEFAULT')