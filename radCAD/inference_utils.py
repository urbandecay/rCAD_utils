# radCAD/inference_utils.py

import bpy
from mathutils import Vector, geometry
from bpy_extras.view3d_utils import (
    region_2d_to_origin_3d,
    region_2d_to_vector_3d,
    location_3d_to_region_2d,
)

def get_axis_snapped_location(start, mouse_xy, context, snap_threshold=0.9):
    """
    Compute a snapped location based on how the line from the
    start point to the mouse appears in the viewport.

    The method works by:
      1. Projecting the start point to screen space.
      2. Calculating the 2D direction from the start to the mouse.
      3. For each global axis (X, Y, Z) offset from the start,
         project the resulting point to screen space.
      4. Compare the normalized screen-space displacement with the unit
         screen-space direction of the global axis.
      5. If the best matching axis has a dot product above snap_threshold,
         perform a 3D Ray-Line intersection to find the exact point on the axis
         that lies under the mouse cursor.
    """
    region = context.region
    rv3d = context.space_data.region_3d

    # Validate required items
    if not (start and region and rv3d):
        return None, None, None

    # Get the screen coordinates of the start point.
    start_screen = location_3d_to_region_2d(region, rv3d, start)
    if not start_screen:
        return None, None, None

    # Calculate the displacement on screen from the start to current mouse position.
    mouse_screen = Vector(mouse_xy)
    d_screen = mouse_screen - start_screen
    
    # If mouse hasn't moved, no direction to infer
    if d_screen.length < 1.0: 
        return None, None, None
        
    d_screen_norm = d_screen.normalized()

    # Define the three global axes.
    axes = {
        "X": Vector((1, 0, 0)),
        "Y": Vector((0, 1, 0)),
        "Z": Vector((0, 0, 1)),
    }
    best_axis = None
    best_dot = -1.0

    # Evaluate how each axis projects onto the screen from the start point.
    for axis_name, axis_vec in axes.items():
        # Compute the screen position of a point offset by the unit axis.
        target = start + axis_vec
        target_screen = location_3d_to_region_2d(region, rv3d, target)
        if not target_screen:
            continue

        screen_dir = target_screen - start_screen
        if screen_dir.length < 1e-6:
            continue
        screen_dir.normalize()

        # Use absolute value for the dot comparison so snapping occurs
        # regardless of whether the user is dragging in the positive or negative direction.
        dot_val = abs(d_screen_norm.dot(screen_dir))
        if dot_val > best_dot:
            best_dot = dot_val
            best_axis = axis_name

    # If the best-matching axis is above our threshold, calculate the 3D intersection.
    if best_axis and best_dot >= snap_threshold:
        chosen_axis_vec = axes[best_axis]
        
        # 1. Generate the 3D Ray from the mouse cursor
        ray_origin = region_2d_to_origin_3d(region, rv3d, mouse_xy)
        ray_vector = region_2d_to_vector_3d(region, rv3d, mouse_xy)
        
        # 2. Define the Axis Line (Start Point + Infinite Axis Vector)
        # We define a segment long enough to cover the scene
        axis_p1 = start
        axis_p2 = start + (chosen_axis_vec * 10000.0) 
        
        # 3. Intersect Mouse Ray with Axis Line
        # intersect_line_line returns a tuple of points (point_on_ray, point_on_axis)
        # We want the point on the axis.
        res = geometry.intersect_line_line(ray_origin, ray_origin + ray_vector * 10000.0, axis_p1, axis_p2)
        
        if res:
            snap_loc = res[1] # The point on the second line (the axis)
            
            # Recalculate chosen_axis with correct sign based on where the point landed relative to start
            diff = snap_loc - start
            if diff.dot(chosen_axis_vec) < 0:
                final_axis_vec = -chosen_axis_vec
            else:
                final_axis_vec = chosen_axis_vec
                
            return snap_loc, final_axis_vec, best_axis

    return None, None, None