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

        row = box.row(align=True)
        row.label(text="1. Source")
        row.operator("mesh.rcad_store_projection_source", text="Store Source")
        if projection_state.has_source():
            count = projection_state.vertex_count
            objects = projection_state.source_object_count
            noun = "vertex" if count == 1 else "vertices"
            object_note = "" if objects == 1 else f" on {objects} objects"
            box.label(text=f"Stored: {count} {noun}{object_note}", icon='CHECKMARK')
        else:
            box.label(text="Select source vertices in Edit Mode.", icon='INFO')

        box.separator()
        row = box.row(align=True)
        row.label(text="2. Target")
        target_row = row.row(align=True)
        target_row.enabled = projection_state.has_source()
        target_row.operator("mesh.rcad_store_projection_target", text="Store Target")
        box.label(
            text=_target_label(),
            icon='CHECKMARK' if projection_state.has_target() else 'INFO',
        )
        if context.mode == 'OBJECT':
            box.label(text="Object Mode: select exactly one mesh object.")
        elif context.mode == 'EDIT_MESH':
            box.label(text="Edit Mode: select target faces on one object.")
        else:
            box.label(text="Use Object Mode or Mesh Edit Mode.")

        box.separator()
        row = box.row(align=True)
        project = row.row(align=True)
        project.enabled = (
            context.mode == 'EDIT_MESH'
            and projection_state.has_source()
            and projection_state.has_target()
        )
        project.operator("mesh.rcad_project_stored_geometry", text="3. Project")
        row.operator("mesh.rcad_clear_projection_state", text="", icon='X')

        if projection_state.has_target() and context.mode != 'EDIT_MESH':
            box.label(text="Return the source to Edit Mode to Project.", icon='ERROR')

        direction_box = box.box()
        direction_box.prop(context.scene, "rcad_projection_direction", text="Direction")
        if context.scene.rcad_projection_direction == 'HORIZONTAL':
            direction_box.prop(context.scene, "rcad_projection_horizontal_axis", text="Axis")


classes = (RCAD_PT_Projection,)
