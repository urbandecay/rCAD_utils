import bpy

from .storage import projection_source


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
        box = self.layout.box()

        row = box.row(align=True)
        row.label(text="Geometry:")
        row.operator("mesh.rcad_store_projection_source", text="Store")

        if projection_source.has_source():
            box.label(
                text=f"Stored: {projection_source.vertex_count} vertices",
                icon='CHECKMARK',
            )
        else:
            box.label(text="Store source geometry first.", icon='INFO')

        row = box.row(align=True)
        row.operator("mesh.rcad_project_stored_geometry", text="Project")


classes = (RCAD_PT_Projection,)
