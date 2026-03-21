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
    
    # 1. Gather all raw islands
    raw_islands = []
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
            if current_indices: raw_islands.append(current_indices)

    # 2. Filter out debris (tiny islands)
    if not raw_islands:
        return []

    # Calculate bounding box diagonals to find the largest island
    island_diagonals = []
    
    for indices in raw_islands:
        # Vertex count check
        if len(indices) < 4:
            continue

        # Face count check (must be a 3D volume, so >= 4 faces)
        unique_faces = set()
        for idx in indices:
            v = bm.verts[idx]
            for f in v.link_faces:
                unique_faces.add(f.index)
        
        if len(unique_faces) < 4:
            continue
            
        # Bounds check
        coords = [bm.verts[i].co for i in indices]
        min_c = mathutils.Vector((min(c.x for c in coords), min(c.y for c in coords), min(c.z for c in coords)))
        max_c = mathutils.Vector((max(c.x for c in coords), max(c.y for c in coords), max(c.z for c in coords)))
        diagonal = (max_c - min_c).length
        island_diagonals.append((diagonal, indices))

    if not island_diagonals:
        return []

    max_diag = max(d[0] for d in island_diagonals)
    threshold = max_diag * 0.10  # Filter out anything smaller than 10% of the largest piece

    for diag, indices in island_diagonals:
        if diag > threshold:
            islands_indices.append(indices)

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

            # Must overlap or touch in all 3 axes (tolerance for touching)
            if ix0 > ix1 + 0.001 or iy0 > iy1 + 0.001 or iz0 > iz1 + 0.001:
                continue

            p_co = mathutils.Vector(((ix0 + ix1) / 2, (iy0 + iy1) / 2, (iz0 + iz1) / 2))

            dx = abs((min_i.x + max_i.x) / 2 - (min_j.x + max_j.x) / 2)
            dz = abs((min_i.z + max_i.z) / 2 - (min_j.z + max_j.z) / 2)

            x_overlap = max(0, ix1 - ix0)
            z_overlap = max(0, iz1 - iz0)
            min_width = min(max_i.x - min_i.x, max_j.x - min_j.x)
            min_height = min(max_i.z - min_i.z, max_j.z - min_j.z)

            if dx >= dz:
                # Side by side → vertical cut (skip diagonal pairs)
                if z_overlap < min_height * 0.5:
                    continue
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
                # Stacked → horizontal cut (skip diagonal pairs)
                if x_overlap < min_width * 0.5:
                    continue
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
                if len(island) < 4: continue  # Ignore tiny debris (points/lines/tris)
                pos_set = set()
                for idx in island:
                    v = bm.verts[idx]
                    pos_set.add((round(v.co.x, 6), round(v.co.y, 6), round(v.co.z, 6)))
                islands_data.append(pos_set)

            if len(islands_data) < 2:
                self.report({'WARNING'}, "Need 2+ valid islands (>=4 verts).")
                wm.progress_end()
                return {'CANCELLED'}

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
                
                mwi = obj.matrix_world.inverted()
                bpy.ops.object.mode_set(mode='EDIT')

                # Apply all cuts to this object
                for plane_co, plane_no, clear_inner, clear_outer in cuts:
                    # Convert world-space plane to local-space
                    local_co = mwi @ plane_co
                    local_no = (obj.matrix_world.to_3x3().transposed() @ plane_no).normalized()
                    
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.bisect(plane_co=local_co, plane_no=local_no,
                                        clear_inner=clear_inner, clear_outer=clear_outer,
                                        threshold=0.0001)

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
