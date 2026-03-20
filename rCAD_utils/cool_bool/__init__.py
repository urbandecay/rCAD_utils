import bpy
import bmesh
import mathutils
from collections import deque


# --- CORE HELPERS ---

def find_selected_islands(bm):
    islands_indices = []
    visited = set()
    selected_verts = [v for v in bm.verts if v.select]
    if not selected_verts: return islands_indices
    selected_verts_dict = {v.index: v for v in selected_verts}
    for start_vert in selected_verts:
        if start_vert.index not in visited:
            current_indices = set()
            q = deque([start_vert])
            visited.add(start_vert.index)
            current_indices.add(start_vert.index)
            while q:
                v = q.popleft()
                for edge in v.link_edges:
                    other = edge.other_vert(v)
                    if other.index in selected_verts_dict and other.index not in visited:
                        visited.add(other.index)
                        current_indices.add(other.index)
                        q.append(other)
            if current_indices: islands_indices.append(current_indices)
    return islands_indices


def select_island_geometry_by_positions(bm, positions_set):
    for v in bm.verts: v.select_set(False)
    for e in bm.edges: e.select_set(False)
    for f in bm.faces: f.select_set(False)
    count = 0
    for v in bm.verts:
        coord = (round(v.co.x, 6), round(v.co.y, 6), round(v.co.z, 6))
        if coord in positions_set:
            v.select_set(True)
            count += 1
    for e in bm.edges:
        if all(v.select for v in e.verts): e.select_set(True)
    for f in bm.faces:
        if all(v.select for v in f.verts): f.select_set(True)
    return count > 0


def separate_and_find_new(context, objects_before):
    bpy.ops.mesh.separate(type='SELECTED')
    objects_after = set(context.scene.objects)
    new_objects = list(objects_after - objects_before)
    if len(new_objects) == 1: return new_objects[0]
    return None


# --- OPERATOR ---

class MESH_OT_CoolBool(bpy.types.Operator):
    bl_idname = "mesh.cool_bool"
    bl_label = "Cool Bool"
    bl_options = {'REGISTER', 'UNDO'}

    operation_mode: bpy.props.EnumProperty(
        items=[('UNION', "Union", ""), ('SUBTRACT', "Subtract", ""), ('INTERSECT', "Intersect", "")],
        default='UNION',
    )
    keep_cutter: bpy.props.BoolProperty(name="Keep Cutter", default=False)
    intersect_with_cutter: bpy.props.BoolProperty(name="Intersect with Cutter", default=False)
    merge_intersections: bpy.props.BoolProperty(name="Merge Intersections", default=False)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "keep_cutter")
        if self.operation_mode == 'INTERSECT':
            row = layout.row()
            row.prop(self, "intersect_with_cutter")
            if self.intersect_with_cutter:
                row.prop(self, "merge_intersections")

    def execute(self, context):
        wm = context.window_manager
        wm.progress_begin(0, 100)

        try:
            solver_mode = context.scene.cool_bool_solver
            keep_cutter_setting = self.keep_cutter

            original_obj = context.active_object
            original_name = original_obj.name

            bm = bmesh.from_edit_mesh(original_obj.data)
            bm.verts.ensure_lookup_table()
            islands = find_selected_islands(bm)
            if len(islands) < 2:
                self.report({'ERROR'}, "Need 2+ islands.")
                wm.progress_end()
                return {'CANCELLED'}

            cutter_index = 0
            active_index = -1
            if bm.select_history.active:
                elem = bm.select_history.active
                if isinstance(elem, bmesh.types.BMVert): active_index = elem.index
                elif isinstance(elem, bmesh.types.BMEdge): active_index = elem.verts[0].index
                elif isinstance(elem, bmesh.types.BMFace): active_index = elem.verts[0].index
            if active_index != -1:
                for i, island in enumerate(islands):
                    if active_index in island: cutter_index = i; break

            islands_data = []
            for island in islands:
                pos_set = set()
                for idx in island:
                    v = bm.verts[idx]
                    pos_set.add((round(v.co.x, 6), round(v.co.y, 6), round(v.co.z, 6)))
                islands_data.append(pos_set)

            cutter_pos = islands_data[cutter_index]
            target_pos_list = [d for i, d in enumerate(islands_data) if i != cutter_index]

            bpy.ops.object.mode_set(mode='EDIT')
            bm = bmesh.from_edit_mesh(original_obj.data)
            select_island_geometry_by_positions(bm, cutter_pos)
            bmesh.update_edit_mesh(original_obj.data)
            objs_before = set(context.scene.objects)
            cutter_obj = separate_and_find_new(context, objs_before)
            cutter_obj.name = "CB_Main_Temp"

            target_objects = []
            for i, t_pos in enumerate(target_pos_list):
                context.view_layer.objects.active = original_obj
                bpy.ops.object.mode_set(mode='EDIT')
                bm = bmesh.from_edit_mesh(original_obj.data)
                if select_island_geometry_by_positions(bm, t_pos):
                    bmesh.update_edit_mesh(original_obj.data)
                    objs_before = set(context.scene.objects)
                    t_obj = separate_and_find_new(context, objs_before)
                    if t_obj: t_obj.name = f"CB_Target_{i}"; target_objects.append(t_obj)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            final_objects = []
            objects_to_delete = []

            def apply_bool_and_clean(main, operand, op):
                if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
                mod = main.modifiers.new("CB", 'BOOLEAN')
                mod.operation = op
                mod.object = operand
                mod.solver = solver_mode
                context.view_layer.objects.active = main
                bpy.ops.object.modifier_apply(modifier=mod.name)
                bpy.ops.object.mode_set(mode='EDIT')
                bpy.ops.mesh.select_all(action='SELECT')
                bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                bpy.ops.object.mode_set(mode='OBJECT')

            if self.operation_mode == 'UNION':
                main_obj = cutter_obj
                for t_obj in target_objects:
                    apply_bool_and_clean(main_obj, t_obj, 'UNION')
                    objects_to_delete.append(t_obj)
                final_objects.append(main_obj)

            elif self.operation_mode == 'SUBTRACT':
                for t_obj in target_objects:
                    apply_bool_and_clean(t_obj, cutter_obj, 'DIFFERENCE')
                    final_objects.append(t_obj)
                if keep_cutter_setting: final_objects.append(cutter_obj)
                else: objects_to_delete.append(cutter_obj)

            elif self.operation_mode == 'INTERSECT':
                if self.intersect_with_cutter:
                    intersection_results = []
                    for t_obj in target_objects:
                        apply_bool_and_clean(t_obj, cutter_obj, 'INTERSECT')
                        intersection_results.append(t_obj)
                    if keep_cutter_setting:
                        final_objects.append(cutter_obj)
                    else:
                        objects_to_delete.append(cutter_obj)
                    if self.merge_intersections and len(intersection_results) > 1:
                        merged = intersection_results[0]
                        for r_obj in intersection_results[1:]:
                            apply_bool_and_clean(merged, r_obj, 'UNION')
                            objects_to_delete.append(r_obj)
                        final_objects.append(merged)
                    else:
                        final_objects.extend(intersection_results)
                else:
                    all_objects = [cutter_obj] + target_objects
                    result_obj = all_objects[0]
                    for i in range(1, len(all_objects)):
                        next_obj = all_objects[i]
                        apply_bool_and_clean(result_obj, next_obj, 'INTERSECT')
                        objects_to_delete.append(next_obj)
                    final_objects.append(result_obj)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.select_all(action='DESELECT')
            for obj in objects_to_delete:
                try: bpy.data.objects.remove(obj, do_unlink=True)
                except: pass
            valid_finals = [o for o in final_objects if o.name in bpy.data.objects]
            for o in valid_finals: o.select_set(True)
            original_obj.select_set(True)
            context.view_layer.objects.active = original_obj
            bpy.ops.object.join()
            original_obj.name = original_name
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='DESELECT')
            self.report({'INFO'}, "Cool Bool Finished")

        except Exception as e:
            self.report({'ERROR'}, f"Script Failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            wm.progress_end()
        return {'FINISHED'}
