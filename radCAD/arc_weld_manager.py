import bpy
import bmesh
import math
import mathutils
from . import weld_utils
from .modal_state import state

DEBUG_MODE = True

def dbg(msg):
    if DEBUG_MODE:
        print(f"[ArcWeld DEBUG] {msg}")

# --- MATH HELPERS ---
def align_view_to_face_robust(space, center_world, normal_world, radius):
    r3d = space.region_3d
    try:
        q = (-normal_world).to_track_quat('-Z', 'Y')
    except ValueError:
        q = mathutils.Quaternion() 
    r3d.view_rotation = q
    r3d.view_location = center_world
    r3d.view_perspective = 'ORTHO'
    if hasattr(r3d, "ortho_scale"):
        r3d.ortho_scale = max(radius * 4.0, 0.1)

def find_view3d():
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    return area, space
    return None, None

# --- MAIN EXECUTION ---

def run(ctx, arc_verts, arc_edges):
    if not state.get("auto_weld", True):
        return

    dbg("--- STARTING WELD RUN ---")

    # 1. SNAPSHOT
    original_select_mode = ctx.tool_settings.mesh_select_mode[:]
    obj = ctx.edit_object
    me = obj.data
    bm = bmesh.from_edit_mesh(me)
    mw = obj.matrix_world
    
    # Get user preference for weld radius
    base_radius = state.get("weld_radius", 0.001)
    radius = max(base_radius, 0.0001)
    
    # 2. ENSURE SELECTION
    bpy.ops.mesh.select_all(action='DESELECT')
    for v in arc_verts: 
        if v.is_valid: v.select = True
    for e in arc_edges: 
        if e.is_valid: e.select = True
        
    dbg(f"Initial Arc Verts Selected: {len([v for v in arc_verts if v.select])}")

    # --- PHASE 1: PRE-WELD (Snap Arc Ends to Existing Geometry) ---
    # This phase moves the ARC to the MESH. It is generally safe.
    target_verts, target_edges = weld_utils.find_nearby_geometry(bm, arc_verts, radius * 2.0, mw)
    
    # CRITICAL: Snap endpoints to vertices/edges FIRST
    weld_utils.perform_heavy_weld(bm, arc_verts, (target_verts, target_edges), radius, mw)

    # NEW: SELF-INTERSECTION PASS
    # Detect if the new drawing crosses itself and create junctions.
    weld_utils.perform_self_x_weld(bm, arc_edges, radius, mw)
    
    # Standard intersection weld
    weld_utils.perform_x_weld(bm, arc_edges, target_edges, radius * 1.5, mw)
    
    for v in target_verts:
        if v.is_valid: v.select = True
    
    bmesh.ops.remove_doubles(bm, verts=[v for v in bm.verts if v.select], dist=radius)
    bmesh.update_edit_mesh(me)
    
    dbg("Phase 1 (Endpoint Weld) Complete.")
    
    # --- PHASE 2: KNIFE PROJECT ---
    if state.get("weld_to_faces", True):
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        
        # Save indices and coords to find them after the cut
        final_arc_edge_indices = [e.index for e in bm.edges if e.select]
        
        # CRITICAL: These are the "Ideal" coordinates where the vertices SHOULD be.
        final_arc_coords = [v.co.copy() for v in bm.verts if v.select]
        
        dbg(f"Preparing Knife Project for {len(final_arc_edge_indices)} edges.")
        
        if final_arc_edge_indices:
            _run_knife_project_final(ctx, obj, final_arc_edge_indices, final_arc_coords)

    try:
        ctx.tool_settings.mesh_select_mode = original_select_mode
    except Exception:
        pass
    
    dbg("--- RUN COMPLETE ---")


def _run_knife_project_final(ctx, obj, arc_edge_indices, arc_coords_to_find):
    # FIXED: Removed the guard clause that prevented cutting in Perpendicular mode.
    # if state.get("is_perpendicular", False):
    #    return

    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    
    mw = obj.matrix_world
    Zp = state.get("Zp", mathutils.Vector((0,0,1)))
    originally_hidden_faces = [f.index for f in bm.faces if f.hide]

    # 1. IDENTIFY TARGET FACES & CALCULATE BOUNDS
    if not arc_coords_to_find: return
    
    min_v = mathutils.Vector((float('inf'),)*3)
    max_v = mathutils.Vector((float('-inf'),)*3)
    
    for co in arc_coords_to_find:
        w = mw @ co
        min_v.x = min(min_v.x, w.x); min_v.y = min(min_v.y, w.y); min_v.z = min(min_v.z, w.z)
        max_v.x = max(max_v.x, w.x); max_v.y = max(max_v.y, w.y); max_v.z = max(max_v.z, w.z)
    
    arc_center = (min_v + max_v) * 0.5
    arc_radius = (max_v - min_v).length * 0.5
    if arc_radius == 0: arc_radius = 1.0

    sample_point_world = mw @ arc_coords_to_find[0]
    
    candidates = [] 
    mw_rot = mw.to_3x3()
    best_align_normal = Zp
    best_align_dot = -1.0
    
    for f in bm.faces:
        if f.hide or f.select: continue
        f_norm_world = mw_rot @ f.normal
        
        # NOTE: This check ensures we only cut faces parallel to the drawing plane.
        # In Perpendicular mode, this correctly ignores the floor (dot ~ 0) 
        # and targets walls (dot ~ 1) that the arc might be attached to.
        dot = f_norm_world.dot(Zp)
        if abs(dot) < 0.9: continue 
        
        plane_co_world = mw @ f.verts[0].co
        dist = abs(mathutils.geometry.distance_point_to_plane(sample_point_world, plane_co_world, f_norm_world))
        
        if dist > 0.3: continue
        candidates.append((f, dist, f_norm_world, abs(dot)))

    if not candidates: 
        dbg("No target faces found.")
        return

    candidates.sort(key=lambda x: x[1])
    target_faces = []
    tolerance = 0.0001 
    
    # 2. BUILD CUTTER (RELATIVE LIFT)
    c_verts = []
    c_edges = []
    v_map = {}
    
    # Reduce lift to prevent parallax drift
    lift_amount = arc_radius * 0.001
    lift_vec = Zp * lift_amount
    
    closest_dist = candidates[0][1]
    for f, dist, norm, dot in candidates:
        if dist <= closest_dist + tolerance:
            target_faces.append(f)
            if dot > best_align_dot:
                best_align_dot = dot
                best_align_normal = norm
        else:
            break
            
    for idx in arc_edge_indices:
        if idx >= len(bm.edges): continue
        e = bm.edges[idx]
        v1, v2 = e.verts
        if v1 not in v_map:
            v_map[v1] = len(c_verts)
            c_verts.append((mw @ v1.co) + lift_vec)
        if v2 not in v_map:
            v_map[v2] = len(c_verts)
            c_verts.append((mw @ v2.co) + lift_vec)
        c_edges.append((v_map[v1], v_map[v2]))
    
    mesh_data = bpy.data.meshes.new("TempArcCutter_Mesh")
    mesh_data.from_pydata(c_verts, c_edges, [])
    cutter_obj = bpy.data.objects.new("TempArcCutter", mesh_data)
    ctx.collection.objects.link(cutter_obj)
    
    try:
        # 3. PREPARE SELECTION
        bpy.ops.mesh.select_all(action='DESELECT')
        
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        target_set = set(target_faces)
        
        for f in bm.faces:
            if f in target_set:
                f.hide = False
                f.select = True
            else:
                f.hide = True
                
        bmesh.update_edit_mesh(obj.data)
        
        # 4. ALIGN VIEW
        area, space = find_view3d()
        orig_rot, orig_loc, orig_persp = None, None, None
        if space:
            r3d = space.region_3d
            orig_rot = r3d.view_rotation.copy()
            orig_loc = r3d.view_location.copy()
            orig_persp = r3d.view_perspective
            align_view_to_face_robust(space, arc_center, Zp, arc_radius)
            try: bpy.ops.wm.redraw_timer(type='DRAW_WIN', iterations=1)
            except Exception: pass
            ctx.view_layer.update()
        
        # 5. KNIFE PROJECT
        cutter_obj.select_set(True)
        obj.select_set(True)
        ctx.view_layer.objects.active = obj
        
        try:
            dbg("Executing Knife Project (Cut Through=True)...")
            res = bpy.ops.mesh.knife_project(cut_through=True)
            
            # 6. MERGE LOGIC (The Teleporter)
            bm_final = bmesh.from_edit_mesh(obj.data)
            bm_final.verts.ensure_lookup_table()
            
            # Use user preference for the final weld threshold
            weld_rad = state.get("weld_radius", 0.001)
            
            # Search area to catch the sloppy cut (slightly reduced from 3.0 to 2.5 for safety)
            search_rad_sq = (weld_rad * 2.5) ** 2
            
            reselected = 0
            
            for v in bm_final.verts:
                if v.hide: continue
                
                if v.select:
                    # --- NEWLY CUT VERTEX ---
                    # It was selected by the Knife Project. Teleport it to the ideal mathematical position.
                    for target_co in arc_coords_to_find:
                        dist_sq = (v.co - target_co).length_squared
                        if dist_sq < search_rad_sq:
                            v.co = target_co
                            reselected += 1
                            break
                else:
                    # --- ORIGINAL ARC VERTEX ---
                    # It was unselected before the cut. If it's exactly at an ideal coordinate, 
                    # select it now so it gets merged with the cut geometry and disappears!
                    for target_co in arc_coords_to_find:
                        if (v.co - target_co).length_squared < 1e-8:
                            v.select = True
                            break
            
            dbg(f"GPS Reselected & Teleported {reselected} verts.")
            
            bmesh.update_edit_mesh(obj.data)
            
            # Now remove doubles. Since we teleported them to EXACTLY 0.0 distance,
            # this will 100% succeed for the intended verts only.
            ret = bpy.ops.mesh.remove_doubles(threshold=weld_rad)
            dbg(f"Remove Doubles Result: {ret}")

            # 7. CLEANUP VISIBILITY
            bm_clean = bmesh.from_edit_mesh(obj.data)
            bm_clean.faces.ensure_lookup_table()
            
            hidden_indices = set(originally_hidden_faces)
            for f in bm_clean.faces:
                if f.index in hidden_indices:
                    f.hide = True
                else:
                    f.hide = False
            
            for f in bm_clean.faces:
                if not f.hide:
                    f.select = False 
                    for v in f.verts: v.hide = False
                    for e in f.edges: e.hide = False
                    
            bmesh.update_edit_mesh(obj.data)
            
        except Exception as e:
            print(f"[ArcWeld ERROR] Knife Project Failed: {e}")
            import traceback
            traceback.print_exc()
            
            bm_fail = bmesh.from_edit_mesh(obj.data)
            for f in bm_fail.faces: 
                f.hide = False
                for v in f.verts: v.hide = False
                for e in f.edges: e.hide = False
            bmesh.update_edit_mesh(obj.data)

        if space and orig_rot:
            space.region_3d.view_rotation = orig_rot
            space.region_3d.view_location = orig_loc
            space.region_3d.view_perspective = orig_persp

    finally:
        if cutter_obj.name in bpy.data.objects:
            bpy.data.objects.remove(cutter_obj, do_unlink=True)
        if mesh_data.name in bpy.data.meshes:
            bpy.data.meshes.remove(mesh_data)
