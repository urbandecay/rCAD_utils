# operator_arc_2pt.py

import bpy


class VIEW3D_OT_arc_2pt(bpy.types.Operator):
    bl_idname = "view3d.arc_2pt"
    bl_label = "Arc Tool (2-Point) - not implemented yet"

    def execute(self, ctx):
        self.report({'INFO'}, "2-point arc operator not implemented yet")
        return {'CANCELLED'}

