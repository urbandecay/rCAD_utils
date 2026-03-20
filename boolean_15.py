bl_info = {
    "name": "Cool Bool (Grid Fix + Intersect)",
    "author": "You & AI",
    "version": (25, 1, 1),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Temp Tool",
    "description": "v25 Logic with Graph-Based Row Detection + 'Cookie Cutter' Intersect.",
    "category": "Mesh",
}

import bpy
import bmesh
import mathutils
from collections import deque

# --- 1. CORE HELPERS (UNCHANGED) ---

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

def get_center(obj):
    local_bbox_center = 0.125 * sum((mathutils.Vector(b) for b in obj.bound_box), mathutils.Vector())
    return obj.matrix_world @ local_bbox_center

def get_world_bounds(obj):
    corners = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
    min_x = min(c.x for c in corners); min_y = min(c.y for c in corners); min_z = min(c.z for c in corners)
    max_x = max(c.x for c in corners); max_y = max(c.y for c in corners); max_z = max(c.z for c in corners)
    return mathutils.Vector((min_x, min_y, min_z)), mathutils.Vector((max_x, max_y, max_z))

def get_overlap_center(obj1, obj2):
    min1, max1 = get_world_bounds(obj1)
    min2, max2 = get_world_bounds(obj2)
    # Intersection Box
    overlap_min = mathutils.Vector((max(min1.x, min2.x), max(min1.y, min2.y), max(min1.z, min2.z)))
    overlap_max = mathutils.Vector((min(max1.x, max2.x), min(max1.y, max2.y), min(max1.z, max2.z)))
    # Validate intersection
    if overlap_min.x >= overlap_max.x or overlap_min.y >= overlap_max.y or overlap_min.z >= overlap_max.z:
        return None 
    return (overlap_min + overlap_max) / 2

# --- 2. THE FIX: GRAPH-BASED ROW CLUSTERING ---

def share_horizontal_row(obj1, obj2):
    """Returns True if objects share significant Z-space (Height)."""
    min1, max1 = get_world_bounds(obj1)
    min2, max2 = get_world_bounds(obj2)
    
    # Check overlap
    overlap_start = max(min1.z, min2.z)
    overlap_end = min(max1.z, max2.z)
    overlap_amount = overlap_end - overlap_start
    
    if overlap_amount <= 0: return False
    
    # Require overlap to be > 50% of the shorter object's height
    height1 = max1.z - min1.z
    height2 = max2.z - min2.z
    min_height = min(height1, height2)
    
    return overlap_amount > (min_height * 0.5)

def cluster_into_rows(objects):
    """
    Groups objects into rows using a Graph approach (Adjacency).
    This fixes the 'Linear Sort' bug that was merging complex shapes incorrectly.
    """
    if not objects: return []
    
    # 1. Build Adjacency List (Who overlaps with whom?)
    adj = {obj: [] for obj in objects}
    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            if share_horizontal_row(objects[i], objects[j]):
                adj[objects[i]].append(objects[j])
                adj[objects[j]].append(objects[i])
    
    # 2. Find Connected Components (The distinct Rows)
    visited = set()
    rows = []
    
    for obj in objects:
        if obj not in visited:
            current_row = []
            stack = [obj]
            visited.add(obj)
            while stack:
                curr = stack.pop()
                current_row.append(curr)
                for neighbor in adj[curr]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)
            rows.append(current_row)
            
    # 3. Sort internally by X (Left to Right)
    for row in rows:
        row.sort(key=lambda o: get_center(o).x)
        
    # 4. Sort rows by Z (Bottom to Top)
    rows.sort(key=lambda c: get_center(c[0]).z)
    
    return rows

# --- 3. THE OPERATOR (v25 + NEW INTERSECT) ---

class MESH_OT_CoolBool(bpy.types.Operator):
    bl_idname = "mesh.cool_bool"
    bl_label = "Cool Bool"
    bl_options = {'REGISTER', 'UNDO'}

    operation_mode: bpy.props.EnumProperty(
          items=[('UNION', "Union", ""), ('SUBTRACT', "Subtract", ""), ('INTERSECT', "Intersect", "")],
          default='UNION',
    )

    def execute(self, context):
        wm = context.window_manager
        wm.progress_begin(0, 100)
        
        try:
            solver_mode = context.scene.cool_bool_solver
            do_dissolve = context.scene.cool_bool_dissolve
            keep_cutter_setting = context.scene.cool_bool_keep_cutter
            do_mirror_weld = context.scene.cool_bool_mirror_weld
            debug_bounds = context.scene.cool_bool_debug_bounds
            
            original_obj = context.active_object
            original_name = original_obj.name
            
            # --- SEPARATION ---
            bm = bmesh.from_edit_mesh(original_obj.data)
            bm.verts.ensure_lookup_table()
            islands = find_selected_islands(bm)
            if len(islands) < 2:
                self.report({'ERROR'}, "Need 2+ islands.")
                wm.progress_end()
                return {'CANCELLED'}

            # Identify Roles
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

            # Store & Separate
            islands_data = []
            for island in islands:
                pos_set = set()
                for idx in island:
                    v = bm.verts[idx]
                    pos_set.add((round(v.co.x, 6), round(v.co.y, 6), round(v.co.z, 6)))
                islands_data.append(pos_set)
            
            # Standard Mode Separates Cutter vs Targets
            # Grid Mode treats all equal, but we use the same separation code
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

            # --- GRID MODE (TWO-PASS) ---
            if do_mirror_weld and self.operation_mode == 'UNION':
                
                all_parts = [cutter_obj] + target_objects
                
                # --- PASS 1: HORIZONTAL MERGE (X-AXIS) ---
                # Fixed: Uses Cluster Logic now instead of Linear Sort
                rows = cluster_into_rows(all_parts)
                horizontal_bars = []
                
                for row_objs in rows:
                    if not row_objs: continue
                    
                    current_bar = row_objs[0]
                    
                    for i in range(1, len(row_objs)):
                        next_part = row_objs[i]
                        
                        # Find Overlap
                        p_co = get_overlap_center(current_bar, next_part)
                        
                        if p_co:
                            # Strict Vertical Plane (X-Axis Merge)
                            p_no = mathutils.Vector((1, 0, 0)) # Points Right
                            
                            # Clip Left Side (Clear Right)
                            bpy.ops.object.select_all(action='DESELECT')
                            context.view_layer.objects.active = current_bar
                            current_bar.select_set(True)
                            bpy.ops.object.mode_set(mode='EDIT')
                            bpy.ops.mesh.select_all(action='SELECT')
                            bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_outer=True, threshold=0.0001)
                            bpy.ops.object.mode_set(mode='OBJECT')
                            
                            # Clip Right Side (Clear Left)
                            bpy.ops.object.select_all(action='DESELECT')
                            context.view_layer.objects.active = next_part
                            next_part.select_set(True)
                            bpy.ops.object.mode_set(mode='EDIT')
                            bpy.ops.mesh.select_all(action='SELECT')
                            bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_inner=True, threshold=0.0001)
                            bpy.ops.object.mode_set(mode='OBJECT')
                            
                            # Join
                            bpy.ops.object.select_all(action='DESELECT')
                            context.view_layer.objects.active = current_bar
                            current_bar.select_set(True)
                            next_part.select_set(True)
                            bpy.ops.object.join()
                            
                            # Weld
                            bpy.ops.object.mode_set(mode='EDIT')
                            bpy.ops.mesh.select_all(action='SELECT')
                            bpy.ops.mesh.remove_doubles(threshold=0.001)
                            if do_dissolve: bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                            bpy.ops.object.mode_set(mode='OBJECT')
                        else:
                            # No overlap, just join
                            bpy.ops.object.select_all(action='DESELECT')
                            context.view_layer.objects.active = current_bar
                            current_bar.select_set(True)
                            next_part.select_set(True)
                            bpy.ops.object.join()
                            
                    horizontal_bars.append(current_bar)
                
                # --- PASS 2: VERTICAL MERGE (Z-AXIS) ---
                # Now we have [BottomBar, TopBar]. Stack them up.
                horizontal_bars.sort(key=lambda o: get_world_bounds(o)[0].z)
                
                final_structure = horizontal_bars[0]
                
                for i in range(1, len(horizontal_bars)):
                    next_bar = horizontal_bars[i]
                    
                    p_co = get_overlap_center(final_structure, next_bar)
                    
                    if p_co:
                        # Strict Horizontal Plane (Z-Axis Merge)
                        p_no = mathutils.Vector((0, 0, 1)) # Points Up
                        
                        # Clip Bottom Bar (Clear Top)
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = final_structure
                        final_structure.select_set(True)
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_outer=True, threshold=0.0001)
                        bpy.ops.object.mode_set(mode='OBJECT')
                        
                        # Clip Top Bar (Clear Bottom)
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = next_bar
                        next_bar.select_set(True)
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_inner=True, threshold=0.0001)
                        bpy.ops.object.mode_set(mode='OBJECT')
                        
                        # Join
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = final_structure
                        final_structure.select_set(True)
                        next_bar.select_set(True)
                        bpy.ops.object.join()
                        
                        # Weld
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.remove_doubles(threshold=0.001)
                        if do_dissolve: bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                        bpy.ops.object.mode_set(mode='OBJECT')
                    else:
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = final_structure
                        final_structure.select_set(True)
                        next_bar.select_set(True)
                        bpy.ops.object.join()
                
                final_objects.append(final_structure)

            # --- SUBTRACT / INTERSECT or FALLBACK ---
            else:
                # Flush Trim / Standard Boolean fallback logic
                # (Same as before, using Contact Solver)
                
                def apply_bool_and_clean(main, operand, op):
                    if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
                    mod = main.modifiers.new("CB", 'BOOLEAN')
                    mod.operation = op
                    mod.object = operand
                    mod.solver = solver_mode 
                    context.view_layer.objects.active = main
                    bpy.ops.object.modifier_apply(modifier=mod.name)
                    if do_dissolve:
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665) 
                        bpy.ops.object.mode_set(mode='OBJECT')

                if self.operation_mode == 'UNION': # Fallback if Mirror Weld OFF
                    main_obj = cutter_obj
                    for t_obj in target_objects:
                        apply_bool_and_clean(main_obj, t_obj, 'UNION')
                        objects_to_delete.append(t_obj)
                    final_objects.append(main_obj)

                elif self.operation_mode == 'SUBTRACT':
                    # Grid Mode Subtract: Use Overlap Bisect (Flush Trim)
                    if do_mirror_weld:
                        main_obj = cutter_obj
                        for t_obj in target_objects:
                            p_co = get_overlap_center(main_obj, t_obj)
                            if p_co:
                                # Standard normal for trims
                                c1 = get_center(main_obj); c2 = get_center(t_obj)
                                p_no = (c2 - c1).normalized()
                                bpy.ops.object.select_all(action='DESELECT')
                                context.view_layer.objects.active = main_obj
                                main_obj.select_set(True)
                                bpy.ops.object.mode_set(mode='EDIT')
                                bpy.ops.mesh.select_all(action='SELECT')
                                bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_outer=True, threshold=0.0001)
                                bpy.ops.object.mode_set(mode='OBJECT')
                            if not keep_cutter_setting: objects_to_delete.append(t_obj)
                        final_objects.append(main_obj)
                    else:
                        for t_obj in target_objects:
                            apply_bool_and_clean(t_obj, cutter_obj, 'DIFFERENCE')
                            final_objects.append(t_obj)
                        if keep_cutter_setting: final_objects.append(cutter_obj)
                        else: objects_to_delete.append(cutter_obj)
                        
                elif self.operation_mode == 'INTERSECT':
                    # --- UPDATED LOGIC HERE ---
                    # We iterate through TARGETS and cut them using the cutter as reference.
                    for t_obj in target_objects:
                        apply_bool_and_clean(t_obj, cutter_obj, 'INTERSECT')
                        final_objects.append(t_obj)
                    
                    if keep_cutter_setting: 
                        final_objects.append(cutter_obj)
                    else: 
                        objects_to_delete.append(cutter_obj)

            # --- CLEANUP ---
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

# --- UI & REGISTER ---

class VIEW3D_PT_CoolBool(bpy.types.Panel):
    bl_label = "Cool Bool"
    bl_idname = "VIEW3D_PT_cool_bool"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Temp Tool'
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.label(text="Settings:")
        layout.prop(scene, "cool_bool_solver", text="")
        layout.prop(scene, "cool_bool_dissolve", text="Auto Clean")
        layout.prop(scene, "cool_bool_mirror_weld", text="Two-Pass Grid")
        layout.prop(scene, "cool_bool_debug_bounds", text="Show Debug Bounds")
        layout.separator()
        col = layout.column(align=True)
        row = col.row(align=True)
        row.operator("mesh.cool_bool", text="Union").operation_mode = 'UNION'
        row.operator("mesh.cool_bool", text="Subtract").operation_mode = 'SUBTRACT'
        row.operator("mesh.cool_bool", text="Intersect").operation_mode = 'INTERSECT'
        col.separator()
        col.prop(context.scene, "cool_bool_keep_cutter", text="Keep Cutter")

classes = (MESH_OT_CoolBool, VIEW3D_PT_CoolBool)

def register():
    for cls in classes: bpy.utils.register_class(cls)
    bpy.types.Scene.cool_bool_keep_cutter = bpy.props.BoolProperty(name="Keep Cutter", default=False)
    bpy.types.Scene.cool_bool_dissolve = bpy.props.BoolProperty(name="Limited Dissolve", default=False)
    bpy.types.Scene.cool_bool_mirror_weld = bpy.props.BoolProperty(name="Two-Pass Grid", default=False)
    bpy.types.Scene.cool_bool_debug_bounds = bpy.props.BoolProperty(name="Show Bounds", default=False)
    bpy.types.Scene.cool_bool_solver = bpy.props.EnumProperty(name="Solver", items=[('FLOAT', "Fast", ""), ('EXACT', "Exact", ""), ('MANIFOLD', "Manifold", "")], default='EXACT')

def unregister():
    for cls in reversed(classes): bpy.utils.unregister_class(cls)
    del bpy.types.Scene.cool_bool_keep_cutter
    del bpy.types.Scene.cool_bool_solver
    del bpy.types.Scene.cool_bool_dissolve
    del bpy.types.Scene.cool_bool_mirror_weld
    del bpy.types.Scene.cool_bool_debug_bounds

if __name__ == "__main__":
    register()
