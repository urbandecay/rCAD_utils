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
        box.prop(context.scene, "rcad_knife_separate_split")
        box.operator(
            "mesh.rcad_edge_knife_preview",
            text="Preview Cut",
        )
        project = box.operator(
            "mesh.rcad_edge_knife_project",
            text="Project, Knife & Split" if context.scene.rcad_knife_separate_split else "Project & Knife",
        )
        project.use_view = False
        project.separate_split = context.scene.rcad_knife_separate_split
        by_view = box.operator(
            "mesh.rcad_edge_knife_project",
            text="Knife Project by View",
        )
        by_view.use_view = True
        by_view.separate_split = context.scene.rcad_knife_separate_split
        box.separator()
        box.label(text="Select cutter faces; target element last")
        box.operator("mesh.rcad_cut_by_face", text="Cut by Face").separate_split = context.scene.rcad_knife_separate_split


classes = (RCAD_PT_EdgeKnifeProject,)
