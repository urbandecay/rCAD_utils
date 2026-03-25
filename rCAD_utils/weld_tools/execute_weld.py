# execute_weld.py

import bpy
import bmesh
from .deselect_manager import start_global_session, end_global_session
from .utils import set_defer_mesh_update

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
                from .utils import deferred_update_edit_mesh
                deferred_update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
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

        # Defer bmesh.update_edit_mesh() inside sub-operators so we pay the
        # full-mesh rebuild cost only once at the very end, not per-operator.
        # Face weld (square) is excluded — it needs intermediate updates for knife_project.
        NON_DEFERRABLE = {'super_fuse_square'}

        ran = []
        try:
            for name, op_id in plan:
                # Pre-step cleanup: scrub invalid select history
                self._scrub_select_history(context)

                # Enable deferral for operators that don't need intermediate syncs
                defer = op_id not in NON_DEFERRABLE
                if defer:
                    set_defer_mesh_update(True)

                res = self._call_mesh_op(op_id)

                if defer:
                    set_defer_mesh_update(False)

                if res == {'FINISHED'}:
                    ran.append(name)

                # Post-step cleanup: scrub select history (no stabilize needed mid-run)
                self._scrub_select_history(context)

        finally:
            # Always restore the flag even if something blows up
            set_defer_mesh_update(False)

        # ONE full mesh rebuild at the end instead of per-operator
        obj = context.edit_object
        if obj and obj.type == 'MESH':
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
        self._stabilize_editmesh(context)

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