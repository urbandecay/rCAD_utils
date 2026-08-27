"""Sidebar panel for the edge projection and knife operation."""

import bpy


class RCAD_PT_EdgeKnifeProject(bpy.types.Panel):
    bl_label = "Edge Project & Knife"
    bl_idname = "RCAD_PT_EdgeKnifeProject"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD Utils'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 110

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.label(text="Select cutter edge(s) + target face(s)")
        box.operator(
            "mesh.rcad_edge_knife_preview",
            text="Preview Cut",
        )
        box.operator(
            "mesh.rcad_edge_knife_project",
            text="Project, Knife & Split",
        )


classes = (RCAD_PT_EdgeKnifeProject,)
