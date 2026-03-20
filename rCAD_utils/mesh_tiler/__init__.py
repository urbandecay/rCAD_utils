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
    overlap_min = mathutils.Vector((max(min1.x, min2.x), max(min1.y, min2.y), max(min1.z, min2.z)))
    overlap_max = mathutils.Vector((min(max1.x, max2.x), min(max1.y, max2.y), min(max1.z, max2.z)))
    if overlap_min.x >= overlap_max.x or overlap_min.y >= overlap_max.y or overlap_min.z >= overlap_max.z:
        return None
    return (overlap_min + overlap_max) / 2


def share_horizontal_row(obj1, obj2):
    min1, max1 = get_world_bounds(obj1)
    min2, max2 = get_world_bounds(obj2)
    overlap_start = max(min1.z, min2.z)
    overlap_end = min(max1.z, max2.z)
    overlap_amount = overlap_end - overlap_start
    if overlap_amount <= 0: return False
    height1 = max1.z - min1.z
    height2 = max2.z - min2.z
    min_height = min(height1, height2)
    return overlap_amount > (min_height * 0.5)


def cluster_into_rows(objects):
    if not objects: return []
    adj = {obj: [] for obj in objects}
    for i in range(len(objects)):
        for j in range(i + 1, len(objects)):
            if share_horizontal_row(objects[i], objects[j]):
                adj[objects[i]].append(objects[j])
                adj[objects[j]].append(objects[i])
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
    for row in rows:
        row.sort(key=lambda o: get_center(o).x)
    rows.sort(key=lambda c: get_center(c[0]).z)
    return rows


# --- OPERATOR ---

class MESH_OT_MeshTiler(bpy.types.Operator):
    bl_idname = "mesh.mesh_tiler"
    bl_label = "Tile"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        wm = context.window_manager
        wm.progress_begin(0, 100)

        try:
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
            cutter_obj.name = "MT_Main_Temp"

            target_objects = []
            for i, t_pos in enumerate(target_pos_list):
                context.view_layer.objects.active = original_obj
                bpy.ops.object.mode_set(mode='EDIT')
                bm = bmesh.from_edit_mesh(original_obj.data)
                if select_island_geometry_by_positions(bm, t_pos):
                    bmesh.update_edit_mesh(original_obj.data)
                    objs_before = set(context.scene.objects)
                    t_obj = separate_and_find_new(context, objs_before)
                    if t_obj: t_obj.name = f"MT_Target_{i}"; target_objects.append(t_obj)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')

            all_parts = [cutter_obj] + target_objects
            rows = cluster_into_rows(all_parts)
            horizontal_bars = []

            for row_objs in rows:
                if not row_objs: continue
                current_bar = row_objs[0]
                for i in range(1, len(row_objs)):
                    next_part = row_objs[i]
                    p_co = get_overlap_center(current_bar, next_part)
                    if p_co:
                        p_no = mathutils.Vector((1, 0, 0))
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = current_bar
                        current_bar.select_set(True)
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_outer=True, threshold=0.0001)
                        bpy.ops.object.mode_set(mode='OBJECT')

                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = next_part
                        next_part.select_set(True)
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_inner=True, threshold=0.0001)
                        bpy.ops.object.mode_set(mode='OBJECT')

                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = current_bar
                        current_bar.select_set(True)
                        next_part.select_set(True)
                        bpy.ops.object.join()

                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        bpy.ops.mesh.remove_doubles(threshold=0.001)
                        bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                        bpy.ops.object.mode_set(mode='OBJECT')
                    else:
                        bpy.ops.object.select_all(action='DESELECT')
                        context.view_layer.objects.active = current_bar
                        current_bar.select_set(True)
                        next_part.select_set(True)
                        bpy.ops.object.join()
                horizontal_bars.append(current_bar)

            horizontal_bars.sort(key=lambda o: get_world_bounds(o)[0].z)
            final_structure = horizontal_bars[0]

            for i in range(1, len(horizontal_bars)):
                next_bar = horizontal_bars[i]
                p_co = get_overlap_center(final_structure, next_bar)
                if p_co:
                    p_no = mathutils.Vector((0, 0, 1))
                    bpy.ops.object.select_all(action='DESELECT')
                    context.view_layer.objects.active = final_structure
                    final_structure.select_set(True)
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_outer=True, threshold=0.0001)
                    bpy.ops.object.mode_set(mode='OBJECT')

                    bpy.ops.object.select_all(action='DESELECT')
                    context.view_layer.objects.active = next_bar
                    next_bar.select_set(True)
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.bisect(plane_co=p_co, plane_no=p_no, clear_inner=True, threshold=0.0001)
                    bpy.ops.object.mode_set(mode='OBJECT')

                    bpy.ops.object.select_all(action='DESELECT')
                    context.view_layer.objects.active = final_structure
                    final_structure.select_set(True)
                    next_bar.select_set(True)
                    bpy.ops.object.join()

                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.remove_doubles(threshold=0.001)
                    bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                    bpy.ops.object.mode_set(mode='OBJECT')
                else:
                    bpy.ops.object.select_all(action='DESELECT')
                    context.view_layer.objects.active = final_structure
                    final_structure.select_set(True)
                    next_bar.select_set(True)
                    bpy.ops.object.join()

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.select_all(action='DESELECT')
            if final_structure.name in bpy.data.objects:
                final_structure.select_set(True)
            original_obj.select_set(True)
            context.view_layer.objects.active = original_obj
            bpy.ops.object.join()
            original_obj.name = original_name
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='DESELECT')
            self.report({'INFO'}, "Mesh Tiler Finished")

        except Exception as e:
            self.report({'ERROR'}, f"Script Failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            wm.progress_end()
        return {'FINISHED'}
