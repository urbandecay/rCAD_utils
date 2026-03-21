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


def find_all_cuts(parts):
    """Check every pair of meshes. The bounding box intersection shape tells us
    which way to cut: taller than wide → vertical, wider than tall → horizontal."""
    cut_list = []
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            min_i, max_i = get_world_bounds(parts[i])
            min_j, max_j = get_world_bounds(parts[j])

            # Bounding box intersection
            ix0 = max(min_i.x, min_j.x); ix1 = min(max_i.x, max_j.x)
            iy0 = max(min_i.y, min_j.y); iy1 = min(max_i.y, max_j.y)
            iz0 = max(min_i.z, min_j.z); iz1 = min(max_i.z, max_j.z)

            # Must overlap or touch in both X and Z (tolerance for touching)
            if ix0 > ix1 + 0.001 or iz0 > iz1 + 0.001:
                continue

            inter_x = max(0, ix1 - ix0)
            inter_z = max(0, iz1 - iz0)
            p_co = mathutils.Vector(((ix0 + ix1) / 2, (iy0 + iy1) / 2, (iz0 + iz1) / 2))

            if inter_z >= inter_x:
                # Taller than wide → side by side → vertical cut
                p_no = mathutils.Vector((1, 0, 0))
                ci = (min_i.x + max_i.x) / 2
                cj = (min_j.x + max_j.x) / 2
                if ci < cj:
                    left, right = parts[i], parts[j]
                else:
                    left, right = parts[j], parts[i]
                cut_list.append((left, p_co, p_no, False, True))
                cut_list.append((right, p_co, p_no, True, False))
            else:
                # Wider than tall → stacked → horizontal cut
                p_no = mathutils.Vector((0, 0, 1))
                ci = (min_i.z + max_i.z) / 2
                cj = (min_j.z + max_j.z) / 2
                if ci < cj:
                    bot, top = parts[i], parts[j]
                else:
                    bot, top = parts[j], parts[i]
                cut_list.append((bot, p_co, p_no, False, True))
                cut_list.append((top, p_co, p_no, True, False))
    return cut_list


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

            islands_data = []
            for island in islands:
                pos_set = set()
                for idx in island:
                    v = bm.verts[idx]
                    pos_set.add((round(v.co.x, 6), round(v.co.y, 6), round(v.co.z, 6)))
                islands_data.append(pos_set)

            # Separate all islands into individual objects
            all_parts = []
            for i, pos_set in enumerate(islands_data):
                context.view_layer.objects.active = original_obj
                bpy.ops.object.mode_set(mode='EDIT')
                bm = bmesh.from_edit_mesh(original_obj.data)
                if select_island_geometry_by_positions(bm, pos_set):
                    bmesh.update_edit_mesh(original_obj.data)
                    objs_before = set(context.scene.objects)
                    part = separate_and_find_new(context, objs_before)
                    if part:
                        part.name = f"MT_Part_{i}"
                        all_parts.append(part)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')

            # Find ALL cut planes from pairwise bounding box intersections
            cut_list = find_all_cuts(all_parts)

            # Group cuts by object and apply all cuts to each object at once
            cuts_by_obj = {}
            for obj, plane_co, plane_no, clear_inner, clear_outer in cut_list:
                if obj not in cuts_by_obj:
                    cuts_by_obj[obj] = []
                cuts_by_obj[obj].append((plane_co, plane_no, clear_inner, clear_outer))

            for obj, cuts in cuts_by_obj.items():
                bpy.ops.object.select_all(action='DESELECT')
                context.view_layer.objects.active = obj
                obj.select_set(True)
                bpy.ops.object.mode_set(mode='EDIT')
                bm = bmesh.from_edit_mesh(obj.data)

                # Apply all cuts to this object's bmesh
                for plane_co, plane_no, clear_inner, clear_outer in cuts:
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.bisect(plane_co=plane_co, plane_no=plane_no,
                                        clear_inner=clear_inner, clear_outer=clear_outer,
                                        threshold=0.0001)

                bmesh.update_edit_mesh(obj.data)
                bpy.ops.object.mode_set(mode='OBJECT')

            # Step 6: Done — leave all parts as separate objects
            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            self.report({'INFO'}, f"Mesh Tiler: {len(all_parts)} pieces")

        except Exception as e:
            self.report({'ERROR'}, f"Script Failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            wm.progress_end()
        return {'FINISHED'}
