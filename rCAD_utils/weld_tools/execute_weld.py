# execute_weld.py

import bpy
import bmesh
from .deselect_manager import start_global_session, end_global_session

class OSC_OT_super_fuse_execute(bpy.types.Operator):
    bl_idname = "osc.super_fuse_execute"
    bl_label = "Run Enabled Fusions"
    bl_description = "Run enabled fuse ops in order (L → T → . → .. → X → ■) with current selection."
    bl_options = {'REGISTER', 'UNDO'}

    # --- helpers ---

    def _get_bm(self, context):
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            return None, None
        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        return obj, bm

    def _scrub_select_history(self, context):
        obj, bm = self._get_bm(context)
        if not bm:
            return
        try:
            dirty = False
            for elem in list(bm.select_history):
                if (elem is None) or (not getattr(elem, "is_valid", False)):
                    bm.select_history.remove(elem)
                    dirty = True
            if dirty:
                bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
        except Exception:
            pass

    def _stabilize_editmesh(self, context):
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            return
        try:
            # This rebuilds the EditMesh and flushes internal caches safely.
            bpy.ops.object.mode_set(mode='OBJECT')
            context.view_layer.update()
            bpy.ops.object.mode_set(mode='EDIT')
        except Exception:
            pass

    def _call_mesh_op(self, idname):
        try:
            op = getattr(bpy.ops.mesh, idname)
            return op('EXEC_DEFAULT')
        except Exception as ex:
            print(f"[Super Fuse] operator {idname} exception: {ex}")
            return {'CANCELLED'}

    # --- main ---

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Edit Mode required.")
            return {'CANCELLED'}

        props = getattr(context.scene, "super_fuse", None)
        if props is None:
            self.report({'ERROR'}, "Super Fuse properties not found.")
            return {'CANCELLED'}

        plan = []
        if bool(getattr(props, "l", False)):        plan.append(('L', 'super_fuse_l'))
        if bool(getattr(props, "t", False)):        plan.append(('T', 'super_fuse_t'))
        if bool(getattr(props, "dot", False)):      plan.append(('.', 'super_fuse_vert'))
        if bool(getattr(props, "dotdot", False)):   plan.append(('..', 'super_fuse_heavy'))
        if bool(getattr(props, "x", False)):        plan.append(('X', 'super_fuse_x'))
        if bool(getattr(props, "square", False)):   plan.append(('■', 'super_fuse_square'))

        if not plan:
            self.report({'INFO'}, "No operations enabled.")
            return {'CANCELLED'}

        # Start a global deselection session so ops can add their points; we apply once at the end.
        start_global_session()

        ran = []
        for name, op_id in plan:
            # Pre-step cleanup: scrub invalid select history
            self._scrub_select_history(context)

            res = self._call_mesh_op(op_id)
            if res == {'FINISHED'}:
                ran.append(name)

            # Post-step cleanup: scrub select history only (no per-step mode switch)
            self._scrub_select_history(context)

        # One final mesh rebuild at the end instead of per-step mode switches
        obj = context.edit_object
        if obj and obj.type == 'MESH':
            bm = bmesh.from_edit_mesh(obj.data)
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
            context.view_layer.update()

        # Apply deselection for all recorded welds (all enabled ops) and clear session
        end_global_session(context)

        self.report({'INFO'}, "Ran: " + (", ".join(ran) if ran else "none"))
        return {'FINISHED'}

classes = (OSC_OT_super_fuse_execute,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)