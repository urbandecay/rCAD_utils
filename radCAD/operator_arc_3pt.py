# operator_arc_3pt.py

import bpy


class VIEW3D_OT_arc_3pt(bpy.types.Operator):
    bl_idname = "view3d.arc_3pt"
    bl_label = "Arc Tool (3-Point) - not implemented yet"

    def execute(self, ctx):
        self.report({'INFO'}, "3-point arc operator not implemented yet")
        return {'CANCELLED'}

