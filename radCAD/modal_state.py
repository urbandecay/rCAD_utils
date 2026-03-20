# radCAD/modal_state.py
import math
import bpy
from mathutils import Vector

# NO IMPORT HERE - FIXES CIRCULAR DEPENDENCY

state = {
    "active": False,
    "tool_mode": "1POINT",
    "pivot": None,
    "start": None,
    "p1": None, 
    "p2": None,
    "current": None,
    "radius": 0.0,

    "Xp": None, "Yp": None, "Zp": None,
    "a0": 0.0, "a1": 0.0, "a_prev_raw": 0.0, "accum_angle": 0.0,

    "segments": 32,
    "min_dist": 0.05,
    
    "snap_verts": True,        
    "snap_edges": False,       
    "snap_edge_center": False,
    "snap_face_center": False,
    "use_angle_snap": True,
    "arc_2pt_use_axis_colors": True,
    "arc_2pt_overlay_col": (0.2, 0.2, 0.2, 1.0),
    "arc_3pt_use_axis_colors": True,
    "arc_3pt_overlay_col": (0.2, 0.2, 0.2, 1.0),
    "circle_2pt_use_axis_colors": True,
    "circle_2pt_overlay_col": (0.2, 0.2, 0.2, 1.0),
    "circle_3pt_use_axis_colors": True,
    "circle_3pt_overlay_col": (0.2, 0.2, 0.2, 1.0),
    
    "snap_strength": 6.0,
    "use_radians": False,
    "snap_px": 20.0,
    "show_measure": True,
    "color": (1.0, 0.6, 0.0, 1.0),
    "use_axis_colors": True,
    "axis_color_dim": 1.0,

    "line_perp_show_catmull": True,
    "line_perp_col_catmull": (0.0, 0.8, 1.0, 0.5),
    "line_perp_width_catmull": 2.0,

    "line_perp2_show_catmull": True,
    "line_perp2_col_catmull": (0.0, 0.8, 1.0, 0.5),
    "line_perp2_width_catmull": 2.0,

    "line_tangent_show_catmull": True,
    "line_tangent_col_catmull": (0.0, 0.8, 1.0, 0.5),
    "line_tangent_width_catmull": 2.0,

    "line_tan_tan_show_catmull": True,
    "line_tan_tan_col_catmull": (0.0, 0.8, 1.0, 0.5),
    "line_tan_tan_width_catmull": 2.0,

    "circle_tan3_show_curves": True,
    "circle_tan3_col_curves": (0.0, 0.8, 1.0, 0.5),
    "circle_tan3_width_curves": 2.0,
    "circle_tan3_show_tangent": True,
    "circle_tan3_col_tangent": (0.0, 0.8, 1.0, 0.5),
    "circle_tan3_width_tangent": 2.0,

    "circle_tan2_show_curves": True,
    "circle_tan2_col_curves": (0.0, 0.8, 1.0, 0.5),
    "circle_tan2_width_curves": 2.0,
    "circle_tan2_show_tangent": True,
    "circle_tan2_col_tangent": (0.0, 0.8, 1.0, 0.5),
    "circle_tan2_width_tangent": 2.0,
    
    "color_arc_start": (0.8, 0.8, 0.2, 1.0),
    "color_arc_end": (0.2, 0.8, 0.2, 1.0),
    
    "compass_size": 125,
    "angle_increment": 15.0,
    "overlay_offset_x": 25,
    "overlay_offset_y": 10,
    
    "display_precision": 3,

    "auto_weld": True, 
    "weld_radius": 0.001,
    "weld_to_faces": True,
    
    "show_hotkeys": True,
    "hotkeys_offset_x": 1000,
    "hotkeys_offset_y": 20,

    "stage": 0,
    "preview_pts": [],
    "visual_pts": [],       # NEW: Stores high-res overlay
    "catmull_spline_previews": [], # NEW: Stores tool-specific curve overlays
    "arc1_pts": [], 
    "intersection_pts": [],    
    # NEW: Tan-Tan specific state keys to prevent persistence
    "smooth_curve_1": [],
    "smooth_curve_2": [],
    "tan_1_p1": None,
    "tan_1_p2": None,
    "tan_2_p1": None,
    "tan_2_p2": None,
    "tan_solution_active": False,
    
    "handles": [],
    "compass_rot": 0.0,

    "locked": False,
    "locked_normal": None,
    "locked_plane_point": None,
    "last_surface_hit": None,
    "last_surface_normal": None,
    
    "input_mode": None, 
    "input_string": "",
    "input_screen_pos": None,
    "cursor_index": 0,
    "geometry_snap": False,
    
    "is_perpendicular": False,
    "ui_hitboxes": {},
    "snap_point": None,
    "current_axis_vector": None # NEW: Tracks axis alignment for coloring
}

style = {
    "line_width": 1.0,
    "point_px": 5,
    "font_size": 14,
    "font_color": (0.9, 0.9, 0.9, 1.0),
    "bg_color": (0.1, 0.1, 0.1, 0.75),        
    "bg_color_active": (0.0, 0.3, 0.8, 0.9), 
    "bg_padding": 6,
    "font_size_key": 12, 
    "font_size_label": 10 
}

def reset_state_from_context(ctx):
    # IMPORT INSIDE FUNCTION TO FIX CRASH
    from .preferences import get_prefs
    
    scene = ctx.scene
    prefs = get_prefs()
    
    # Defaults
    c_size = 125; a_inc = 15.0; off_x = 25; off_y = 10
    use_rad = False; strength = 6.0; precision = 3 
    w_rad = 0.001; w_faces = True
    show_keys = True; keys_x = 20; keys_y = 20
    col_start = (0.8, 0.8, 0.2, 1.0); col_end = (0.2, 0.8, 0.2, 1.0)
    fs_key = 12; fs_lbl = 10; pt_sz = 5
    line_perp_show = True; line_perp_col = (0.0, 0.8, 1.0, 0.5)
    line_perp_width = 2.0
    line_perp2_show = True; line_perp2_col = (0.0, 0.8, 1.0, 0.5)
    line_perp2_width = 2.0
    line_tangent_show = True; line_tangent_col = (0.0, 0.8, 1.0, 0.5)
    line_tangent_width = 2.0
    line_tan_tan_show = True; line_tan_tan_col = (0.0, 0.8, 1.0, 0.5)
    line_tan_tan_width = 2.0
    circle_tan3_show_c = True; circle_tan3_col_c = (0.0, 0.8, 1.0, 0.5); circle_tan3_width_c = 2.0
    circle_tan3_show_t = True; circle_tan3_col_t = (0.0, 0.8, 1.0, 0.5); circle_tan3_width_t = 2.0
    circle_tan2_show_c = True; circle_tan2_col_c = (0.0, 0.8, 1.0, 0.5); circle_tan2_width_c = 2.0
    circle_tan2_show_t = True; circle_tan2_col_t = (0.0, 0.8, 1.0, 0.5); circle_tan2_width_t = 2.0

    if prefs:
        c_size = prefs.compass_size
        use_rad = getattr(prefs, "use_radians", False)
        try:
            if use_rad: a_inc = float(prefs.angle_snap_type_rad)
            else: a_inc = float(prefs.angle_snap_type)
        except: a_inc = 15.0
        off_x = getattr(prefs, "overlay_offset_x", 25)
        off_y = getattr(prefs, "overlay_offset_y", 10)
        strength = getattr(prefs, "snap_strength", 6.0)
        precision = getattr(prefs, "display_precision", 3)
        use_axis_cols = getattr(prefs, "use_axis_colors", True)
        axis_dim = getattr(prefs, "axis_color_dim", 1.0)
        w_rad = getattr(prefs, "weld_radius", 0.001)
        w_faces = getattr(prefs, "weld_to_faces", True)
        show_keys = getattr(prefs, "show_hotkeys", True)
        keys_x = getattr(prefs, "hotkeys_offset_x", 20)
        keys_y = getattr(prefs, "hotkeys_offset_y", 20)
        col_start = tuple(getattr(prefs, "color_arc_start", (0.8, 0.8, 0.2, 1.0)))
        col_end = tuple(getattr(prefs, "color_arc_end", (0.2, 0.8, 0.2, 1.0)))
        fs_key = getattr(prefs, "font_size_hotkey", 12)
        fs_lbl = getattr(prefs, "font_size_label", 10)
        pt_sz = getattr(prefs, "preview_vertex_size", 5)

        line_perp_show = getattr(prefs, "line_perp_show_catmull", True)
        line_perp_col = tuple(getattr(prefs, "line_perp_col_catmull", (0.0, 0.8, 1.0, 0.5)))
        line_perp_width = getattr(prefs, "line_perp_width_catmull", 2.0)

        line_perp2_show = getattr(prefs, "line_perp2_show_catmull", True)
        line_perp2_col = tuple(getattr(prefs, "line_perp2_col_catmull", (0.0, 0.8, 1.0, 0.5)))
        line_perp2_width = getattr(prefs, "line_perp2_width_catmull", 2.0)

        line_tangent_show = getattr(prefs, "line_tangent_show_catmull", True)
        line_tangent_col = tuple(getattr(prefs, "line_tangent_col_catmull", (0.0, 0.8, 1.0, 0.5)))
        line_tangent_width = getattr(prefs, "line_tangent_width_catmull", 2.0)

        line_tan_tan_show = getattr(prefs, "line_tan_tan_show_catmull", True)
        line_tan_tan_col = tuple(getattr(prefs, "line_tan_tan_col_catmull", (0.0, 0.8, 1.0, 0.5)))
        line_tan_tan_width = getattr(prefs, "line_tan_tan_width_catmull", 2.0)

        circle_tan3_show_c = getattr(prefs, "circle_tan3_show_curves", True)
        circle_tan3_col_c = tuple(getattr(prefs, "circle_tan3_col_curves", (0.0, 0.8, 1.0, 0.5)))
        circle_tan3_width_c = getattr(prefs, "circle_tan3_width_curves", 2.0)

        circle_tan3_show_t = getattr(prefs, "circle_tan3_show_tangent", True)
        circle_tan3_col_t = tuple(getattr(prefs, "circle_tan3_col_tangent", (0.0, 0.8, 1.0, 0.5)))
        circle_tan3_width_t = getattr(prefs, "circle_tan3_width_tangent", 2.0)

        circle_tan2_show_c = getattr(prefs, "circle_tan2_show_curves", True)
        circle_tan2_col_c = tuple(getattr(prefs, "circle_tan2_col_curves", (0.0, 0.8, 1.0, 0.5)))
        circle_tan2_width_c = getattr(prefs, "circle_tan2_width_curves", 2.0)

        circle_tan2_show_t = getattr(prefs, "circle_tan2_show_tangent", True)
        circle_tan2_col_t = tuple(getattr(prefs, "circle_tan2_col_tangent", (0.0, 0.8, 1.0, 0.5)))
        circle_tan2_width_t = getattr(prefs, "circle_tan2_width_tangent", 2.0)

        ellipse_foci_col_foci = tuple(getattr(prefs, "ellipse_foci_col_foci_lines", (0.0, 1.0, 0.0, 1.0)))

    try:
        theme = ctx.preferences.themes[0].view_3d
        sys_prefs = ctx.preferences.system
        style["point_px"] = pt_sz
        style["line_width"] = max(1.0, sys_prefs.pixel_size) 
    except Exception:
        style["point_px"] = pt_sz
        style["line_width"] = 1.0
        
    style["font_size_key"] = fs_key
    style["font_size_label"] = fs_lbl

    # Persistence Logic
    keep_s_verts = state.get("snap_verts", True)
    keep_s_edges = state.get("snap_edges", False)
    keep_s_ecen  = state.get("snap_edge_center", False)
    keep_s_fcen  = state.get("snap_face_center", False)
    keep_compass = state.get("use_angle_snap", True)
    keep_axis_2pt = state.get("arc_2pt_use_axis_colors", True)
    keep_overlay_2pt = state.get("arc_2pt_overlay_col", (0.2, 0.2, 0.2, 1.0))
    keep_axis_3pt = state.get("arc_3pt_use_axis_colors", True)
    keep_overlay_3pt = state.get("arc_3pt_overlay_col", (0.2, 0.2, 0.2, 1.0))
    keep_axis_c2pt = state.get("circle_2pt_use_axis_colors", True)
    keep_overlay_c2pt = state.get("circle_2pt_overlay_col", (0.2, 0.2, 0.2, 1.0))
    keep_axis_c3pt = state.get("circle_3pt_use_axis_colors", True)
    keep_overlay_c3pt = state.get("circle_3pt_overlay_col", (0.2, 0.2, 0.2, 1.0))
    keep_weld    = state.get("auto_weld", True)
    keep_min_dist = state.get("min_dist", 0.05)
    
    current_mode = state.get("tool_mode", "1POINT")

    state.update({
        "active": True, 
        "tool_mode": current_mode, 
        "pivot": None, "start": None, "p1": None, "p2": None, "current": None, "radius": 0.0,
        "Xp": None, "Yp": None, "Zp": None, "a0": 0.0, "a1": 0.0, "a_prev_raw": 0.0, "accum_angle": 0.0,
        "segments": getattr(scene, "arc_segments", 32),
        
        "snap_verts": keep_s_verts,
        "snap_edges": keep_s_edges,
        "snap_edge_center": keep_s_ecen,
        "snap_face_center": keep_s_fcen,
        "use_angle_snap": keep_compass,
        "arc_2pt_use_axis_colors": keep_axis_2pt,
        "arc_2pt_overlay_col": keep_overlay_2pt,
        "arc_3pt_use_axis_colors": keep_axis_3pt,
        "arc_3pt_overlay_col": keep_overlay_3pt,
        "circle_2pt_use_axis_colors": keep_axis_c2pt,
        "circle_2pt_overlay_col": keep_overlay_c2pt,
        "circle_3pt_use_axis_colors": keep_axis_c3pt,
        "circle_3pt_overlay_col": keep_overlay_c3pt,
        "auto_weld": keep_weld,
        "min_dist": keep_min_dist,
        
        "use_radians": use_rad, "snap_strength": strength,
        "use_axis_colors": use_axis_cols, "axis_color_dim": axis_dim,
        "line_perp_show_catmull": line_perp_show, "line_perp_col_catmull": line_perp_col,
        "line_perp_width_catmull": line_perp_width,
        "line_perp2_show_catmull": line_perp2_show, "line_perp2_col_catmull": line_perp2_col,
        "line_perp2_width_catmull": line_perp2_width,
        "line_tangent_show_catmull": line_tangent_show, "line_tangent_col_catmull": line_tangent_col,
        "line_tangent_width_catmull": line_tangent_width,
        "line_tan_tan_show_catmull": line_tan_tan_show, "line_tan_tan_col_catmull": line_tan_tan_col,
        "line_tan_tan_width_catmull": line_tan_tan_width,
        "circle_tan3_show_curves": circle_tan3_show_c, "circle_tan3_col_curves": circle_tan3_col_c,
        "circle_tan3_width_curves": circle_tan3_width_c,
        "circle_tan3_show_tangent": circle_tan3_show_t, "circle_tan3_col_tangent": circle_tan3_col_t,
        "circle_tan3_width_tangent": circle_tan3_width_t,
        "circle_tan2_show_curves": circle_tan2_show_c, "circle_tan2_col_curves": circle_tan2_col_c,
        "circle_tan2_width_curves": circle_tan2_width_c,
        "circle_tan2_show_tangent": circle_tan2_show_t, "circle_tan2_col_tangent": circle_tan2_col_t,
        "circle_tan2_width_tangent": circle_tan2_width_t,
        "ellipse_foci_col_foci": ellipse_foci_col_foci,
        "show_measure": getattr(scene, "arc_show_measurements", True),
        "color": tuple(getattr(scene, "arc_color", (1.0, 0.6, 0.0, 1.0))),
        "color_arc_start": col_start, "color_arc_end": col_end,
        "compass_size": c_size, "angle_increment": a_inc,
        "overlay_offset_x": off_x, "overlay_offset_y": off_y,
        "display_precision": precision,
        "weld_radius": w_rad, "weld_to_faces": w_faces,
        "show_hotkeys": show_keys, "hotkeys_offset_x": keys_x, "hotkeys_offset_y": keys_y,
        
        "stage": 0, 
        "preview_pts": [], 
        "visual_pts": [],       # CLEARED: High-res overlay
        "catmull_spline_previews": [], # CLEARED: Tool-specific curve overlays
        "arc1_pts": [], 
        "intersection_pts": [],
        
        # CLEARED: Tan-Tan specific state keys to prevent persistence
        "smooth_curve_1": [],
        "smooth_curve_2": [],
        "tan_1_p1": None,
        "tan_1_p2": None,
        "tan_2_p1": None,
        "tan_2_p2": None,
        "tan_solution_active": False,
        "viz_tangent_line": [],
        "viz_diameter_line": [],
        
        "handles": [], "compass_rot": 0.0,
        "locked": False, "locked_normal": None, "locked_plane_point": None,
        "last_surface_hit": None, "last_surface_normal": None,
        "input_mode": None, "input_string": "", "input_screen_pos": None, "cursor_index": 0,
        "geometry_snap": False, "is_perpendicular": False, "ui_hitboxes": {}, "snap_point": None,
        "current_axis_vector": None # Reset axis tracker
    })