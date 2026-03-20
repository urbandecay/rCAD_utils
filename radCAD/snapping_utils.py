# snapping_utils.py

import bmesh
from mathutils import Vector, geometry
from bpy_extras import view3d_utils
from bpy_extras.view3d_utils import location_3d_to_region_2d

ELEMENT_SNAP_RADIUS_PX = 15.0

def raycast_under_mouse(ctx, x, y):
    region, rv3d = ctx.region, ctx.region_data
    coord = (x, y)
    view_vec = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    depsgraph = ctx.evaluated_depsgraph_get()
    
    hit, loc, norm, face_index, obj, _ = ctx.scene.ray_cast(depsgraph, ray_origin, view_vec)
    
    if hit and obj and obj.type == 'MESH':
        return loc, norm, obj
    return None, None, None

def is_visible_to_view(ctx, target_co, tolerance=0.1):
    """
    Checks if a point is visible from the viewport.
    Uses view3d_utils to generate the ray, which correctly handles 
    both PERSPECTIVE (conical) and ORTHO (parallel) projections.
    """
    region, rv3d = ctx.region, ctx.region_data
    depsgraph = ctx.evaluated_depsgraph_get()
    
    # 1. Project target to 2D screen space
    p2d = location_3d_to_region_2d(region, rv3d, target_co)
    if p2d is None: 
        return False

    # 2. Get Ray Origin/Vector from Screen Point
    # This automatically calculates the correct origin for Ortho vs Persp
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, p2d)
    ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, p2d)
    
    # 3. Cast Ray into scene
    success, hit_loc, hit_normal, face_idx, hit_obj, hit_mat = ctx.scene.ray_cast(
        depsgraph, 
        ray_origin, 
        ray_vector, 
        distance=10000.0
    )
    
    if success:
        # Distance from near plane to hit
        dist_hit = (hit_loc - ray_origin).length
        # Distance from near plane to target vertex
        dist_target = (target_co - ray_origin).length
        
        # If we hit something closer (with a small margin for precision), it's occluded
        if dist_hit < (dist_target - tolerance):
            return False
            
    return True

def snap_to_mesh_components(ctx, obj, x, y, max_px=ELEMENT_SNAP_RADIUS_PX, 
                            do_verts=True, 
                            do_edges=True, 
                            do_edge_center=True, 
                            do_face_center=True,
                            **kwargs):
    """
    Snap logic with strict Priority:
    0. Verts (Top Priority)
    1. Edge/Face Centers
    2. Nearest Edge (Bottom Priority)
    """
    if obj is None or obj.type != 'MESH':
        return None

    region, rv3d = ctx.region, ctx.region_data
    mouse = Vector((x, y))
    bm = bmesh.from_edit_mesh(obj.data)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    mw = obj.matrix_world
    
    # Check for X-Ray / Wireframe
    allow_occluded = False
    if ctx.space_data.type == 'VIEW_3D':
        shading = ctx.space_data.shading
        if shading.type == 'WIREFRAME' or shading.show_xray:
            allow_occluded = True

    # Candidates list stores: (PRIORITY, DIST_SQ, WORLD_CO)
    # Lower Priority number wins.
    candidates = []
    limit_sq = max_px * max_px

    # 1. Verts (Priority 0)
    if do_verts:
        for v in bm.verts:
            if v.hide: continue
            wco = mw @ v.co
            p2d = location_3d_to_region_2d(region, rv3d, wco)
            if p2d is None: continue
            
            d2 = (mouse - p2d).length_squared
            if d2 < limit_sq:
                candidates.append((0, d2, wco))

    # 2. Edge Centers (Priority 1)
    if do_edge_center:
        for e in bm.edges:
            if e.hide: continue
            wco = mw @ ((e.verts[0].co + e.verts[1].co) * 0.5)
            p2d = location_3d_to_region_2d(region, rv3d, wco)
            if p2d is None: continue
            
            d2 = (mouse - p2d).length_squared
            if d2 < limit_sq:
                candidates.append((1, d2, wco))

    # 3. Face Centers (Priority 1)
    if do_face_center:
        for f in bm.faces:
            if f.hide: continue
            wco = mw @ f.calc_center_median()
            p2d = location_3d_to_region_2d(region, rv3d, wco)
            if p2d is None: continue
            
            d2 = (mouse - p2d).length_squared
            if d2 < limit_sq:
                candidates.append((1, d2, wco))

    # 4. Nearest Point on Edge (Priority 2)
    # Only checks if do_edges is True
    if do_edges:
        for e in bm.edges:
            if e.hide: continue
            
            v1_world = mw @ e.verts[0].co
            v2_world = mw @ e.verts[1].co
            
            p1_2d = location_3d_to_region_2d(region, rv3d, v1_world)
            p2_2d = location_3d_to_region_2d(region, rv3d, v2_world)
            
            if p1_2d and p2_2d:
                # 2D Intersection
                intersect_2d = geometry.intersect_point_line(mouse, p1_2d, p2_2d)
                
                if intersect_2d:
                    pt_on_seg_2d = intersect_2d[0]
                    
                    # Bound check (is point actually on the segment?)
                    # Add a tiny buffer (5px) to handle corner cases
                    min_x, max_x = min(p1_2d.x, p2_2d.x), max(p1_2d.x, p2_2d.x)
                    min_y, max_y = min(p1_2d.y, p2_2d.y), max(p1_2d.y, p2_2d.y)
                    
                    if (min_x - 5 <= pt_on_seg_2d.x <= max_x + 5) and \
                       (min_y - 5 <= pt_on_seg_2d.y <= max_y + 5):
                        
                        dist2 = (mouse - pt_on_seg_2d).length_squared
                        
                        if dist2 < limit_sq:
                            # Calculate 3D point via interpolation
                            seg_len = (p2_2d - p1_2d).length
                            if seg_len > 0.001:
                                factor = (pt_on_seg_2d - p1_2d).length / seg_len
                                pt_3d = v1_world + (v2_world - v1_world) * factor
                                candidates.append((2, dist2, pt_3d))

    # 5. Sort & Select
    # Sort primarily by Priority (0 wins), then by Distance
    candidates.sort(key=lambda item: (item[0], item[1]))
    
    for prio, dist_sq, co in candidates:
        if allow_occluded:
            return co
            
        if is_visible_to_view(ctx, co):
            return co
            
    return None