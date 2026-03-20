# plane_utils.py
# The Vector Fixer: Handles 3D-2D projections and raycasting.

import bpy
from mathutils import Vector
from bpy_extras import view3d_utils
from bpy_extras.view3d_utils import location_3d_to_region_2d

def world_to_plane(v, Xp, Yp):
    """Projects a 3D vector v onto the 2D plane defined by basis vectors Xp, Yp."""
    if Xp is None or Yp is None:
        return Vector((0, 0))
    return Vector((v.dot(Xp), v.dot(Yp)))

def plane_to_world(v2, Xp, Yp):
    """Converts a 2D plane vector v2 back into 3D world space."""
    if Xp is None or Yp is None:
        return Vector((0, 0, 0))
    return Xp * v2.x + Yp * v2.y

def raycast_under_mouse(ctx, x, y):
    """
    Shoots a ray from the mouse into the scene.
    """
    region, rv3d = ctx.region, ctx.region_data
    coord = (x, y)
    view_vec = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    
    depsgraph = ctx.evaluated_depsgraph_get()
    hit, loc, norm, face_index, obj, _ = ctx.scene.ray_cast(depsgraph, ray_origin, view_vec)
    
    if hit and obj and obj.type == 'MESH':
        return loc, norm, obj
    return None, None, None

def project_mouse_to_ground(ctx, x, y):
    """
    Projects the mouse onto the appropriate Major Grid Plane (XY, XZ, or YZ).
    """
    region, rv3d = ctx.region, ctx.region_data
    coord = (x, y)
    view_vec = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    
    plane_normal = Vector((0, 0, 1))
    plane_point = Vector((0, 0, 0))
    
    # Ortho Snap Logic
    if rv3d.view_perspective == 'ORTHO':
        x_align = abs(view_vec.dot(Vector((1, 0, 0))))
        y_align = abs(view_vec.dot(Vector((0, 1, 0))))
        limit = 0.98 
        
        if x_align > limit: plane_normal = Vector((1, 0, 0))
        elif y_align > limit: plane_normal = Vector((0, 1, 0))
    
    denom = view_vec.dot(plane_normal)
    if abs(denom) < 1e-9:
        return plane_point, plane_normal 
        
    t = (plane_point - ray_origin).dot(plane_normal) / denom
    return ray_origin + view_vec * t, plane_normal

def world_radius_for_pixel_size(ctx, center, Xp, Yp, size_px):
    """
    Calculates 3D radius for a fixed screen pixel size.
    Uses Camera Basis to ensure stability at oblique angles.
    """
    region, rv3d = ctx.region, ctx.region_data
    
    # 1. Project Center to 2D
    p1 = location_3d_to_region_2d(region, rv3d, center)
    if not p1: 
        return 0.5
        
    # 2. Get Camera 'Right' Vector (Always perpendicular to view)
    view_inv = rv3d.view_matrix.inverted()
    cam_right = view_inv.to_3x3() @ Vector((1.0, 0.0, 0.0))
    
    # 3. Project a point 1 meter to the right of center
    p2 = location_3d_to_region_2d(region, rv3d, center + cam_right)
    if not p2: 
        return 0.5
        
    # 4. Calculate Pixels Per Meter at this depth
    px_per_meter = (p2 - p1).length
    
    if px_per_meter < 0.1: 
        return 0.5
        
    # 5. Result: Desired Radius (px) / PxPerMeter
    # We use size_px / 2 because we want Radius, input is Diameter (usually 125)
    return (size_px * 0.5) / px_per_meter