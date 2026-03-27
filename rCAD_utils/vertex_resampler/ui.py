# ui.py — Panel for Vertex Resampler (ported from PolyTangents)

import bpy
from bpy.types import Panel


class RCAD_PT_VertexResampler(Panel):
    bl_label = "Vertex Resampler"
    bl_idname = "RCAD_PT_VertexResampler"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout

        box = layout.box()
        col = box.column(align=True)
        col.label(text="Smart Mode", icon='CURVE_DATA')

        row = col.row(align=True)
        op_min = row.operator("rcad.resample_curve", text="Rem Verts (-)")
        op_min.direction = -1
        op_plus = row.operator("rcad.resample_curve", text="Add Verts (+)")
        op_plus.direction = 1


classes = (RCAD_PT_VertexResampler,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
