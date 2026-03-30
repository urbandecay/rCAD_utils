# ui.py — Panel for Vertex Resampler (ported from PolyTangents)

import bpy
from bpy.props import EnumProperty, StringProperty
from bpy.types import Operator, Panel

from .mode_options import MODE_ITEMS, MODE_LABELS


class RCAD_OT_SetResamplerMode(Operator):
    bl_idname = "rcad.set_resampler_mode"
    bl_label = "Set Resampler Mode"

    mode: StringProperty()

    def execute(self, context):
        scene = context.scene
        current_mode = getattr(scene, "rcad_vertex_resampler_mode", 'NONE')
        if current_mode == self.mode:
            scene.rcad_vertex_resampler_mode = 'NONE'
        else:
            scene.rcad_vertex_resampler_mode = self.mode
        return {'FINISHED'}


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
        scene = context.scene
        active_mode = getattr(scene, "rcad_vertex_resampler_mode", 'NONE')

        box = layout.box()
        col = box.column(align=True)
        col.label(text="Manual Mode", icon='CURVE_DATA')

        if active_mode == 'NONE':
            col.label(text="Pick one mesh type.", icon='INFO')
        else:
            col.label(text=f"Selected: {MODE_LABELS[active_mode]}", icon='CHECKMARK')

        for mode_id, label, _description in MODE_ITEMS:
            row = col.row(align=True)
            is_active = (active_mode == mode_id)
            if active_mode != 'NONE' and not is_active:
                row.enabled = False
            op = row.operator(
                "rcad.set_resampler_mode",
                text=label,
                icon='CHECKBOX_HLT' if is_active else 'CHECKBOX_DEHLT',
                depress=is_active,
            )
            op.mode = mode_id

        row = col.row(align=True)
        row.enabled = (active_mode != 'NONE')
        op_min = row.operator("rcad.resample_curve", text="Rem Verts (-)")
        op_min.direction = -1
        op_plus = row.operator("rcad.resample_curve", text="Add Verts (+)")
        op_plus.direction = 1


classes = (
    RCAD_OT_SetResamplerMode,
    RCAD_PT_VertexResampler,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.rcad_vertex_resampler_mode = EnumProperty(
        name="Resampler Mode",
        items=[('NONE', "None", "")] + MODE_ITEMS,
        default='NONE',
    )


def unregister():
    if hasattr(bpy.types.Scene, "rcad_vertex_resampler_mode"):
        del bpy.types.Scene.rcad_vertex_resampler_mode
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
