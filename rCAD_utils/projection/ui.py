"""Sidebar UI for the projection workflow."""

import bpy

from .storage import projection_state


def _target_label():
    target = projection_state.target
    if target is None:
        return "Target: not stored"
    try:
        if target.is_face_selection:
            count = len(target.face_indices)
            noun = "face" if count == 1 else "faces"
            return f"Target: {count} {noun} on {target.obj.name}"
        return f"Target: object {target.obj.name}"
    except (ReferenceError, RuntimeError):
        return "Target: unavailable (store again)"


class RCAD_PT_Projection(bpy.types.Panel):
    bl_label = "Projection"
    bl_idname = "RCAD_PT_Projection"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD Utils'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 100

    def draw(self, context):
        layout = self.layout
        box = layout.box()

        source_row = box.row(align=True)
        source_row.label(text="Source:")
        source_row.operator("mesh.rcad_store_projection_source", text="Store")

        project = box.row(align=True)
        project.enabled = (
            context.mode == 'EDIT_MESH'
            and projection_state.has_source()
        )
        project.operator("mesh.rcad_project_stored_geometry", text="Project")

        direction_row = box.row(align=True)
        direction_row.label(text="Direction:")
        direction_row.prop(context.scene, "rcad_projection_direction", text="")

        weld_row = box.row(align=True)
        weld_row.prop(context.scene, "rcad_projection_weld", text="Weld")


classes = (RCAD_PT_Projection,)
