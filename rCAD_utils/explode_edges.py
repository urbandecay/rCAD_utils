"""Explode selected mesh edges so they no longer share vertices."""

import bmesh
import bpy


class MESH_OT_ExplodeEdges(bpy.types.Operator):
    bl_idname = "mesh.explode_edges"
    bl_label = "Explode"
    bl_description = "Explode the selected edges so each edge has its own vertices"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.edit_object is not None

    def execute(self, context):
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a mesh.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        selected_edges = [edge for edge in bm.edges if edge.select and not edge.hide]

        if not selected_edges:
            self.report({'INFO'}, "No edges selected.")
            return {'CANCELLED'}

        try:
            bmesh.ops.split_edges(bm, edges=selected_edges)
        except Exception as ex:
            self.report({'ERROR'}, f"Could not explode selected edges: {ex}")
            return {'CANCELLED'}

        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
        self.report({'INFO'}, f"Exploded {len(selected_edges)} edge(s).")
        return {'FINISHED'}


def draw_split_menu(self, _context):
    self.layout.separator()
    self.layout.operator(
        MESH_OT_ExplodeEdges.bl_idname,
        text="Explode",
    )
