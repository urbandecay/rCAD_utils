# radCAD/tool_previews.py
import math
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector
import bpy

from .modal_state import state, style
from .plane_utils import world_radius_for_pixel_size
from .orientation_utils import orthonormal_basis_from_normal

# =========================================================================
#  SHADER MANAGEMENT
# =========================================================================

def get_shaders():
    """
    Returns a dict containing the two necessary shaders.
    """
    shaders = {}
    try:
        shaders["POLYLINE"] = gpu.shader.from_builtin('POLYLINE_UNIFORM_COLOR')
    except Exception:
        try:
            shaders["POLYLINE"] = gpu.shader.from_builtin('UNIFORM_COLOR')
        except:
            shaders["POLYLINE"] = gpu.shader.from_builtin('3D_UNIFORM_COLOR')

    try:
        shaders["UNIFORM"] = gpu.shader.from_builtin('UNIFORM_COLOR')
    except Exception:
        shaders["UNIFORM"] = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
        
    try:
        shaders["FLAT"] = gpu.shader.from_builtin('FLAT_COLOR')
    except Exception:
        shaders["FLAT"] = gpu.shader.from_builtin('3D_FLAT_COLOR')
        
    return shaders

def apply_view_bias(points, ctx, amount=0.002, lift_mult=10.0, persp_percent=0.2):
    """Pulls geometry towards the camera to prevent Z-fighting."""
    rv3d = ctx.region_data
    if not rv3d: return points

    is_ortho = (rv3d.view_perspective == 'ORTHO')
    
    if is_ortho:
        view_inv = rv3d.view_matrix.inverted()
        view_z_axis = Vector((view_inv[0][2], view_inv[1][2], view_inv[2][2]))
        current_scale = getattr(rv3d, "ortho_scale", 1.0)
        bias_dist = current_scale * amount * lift_mult 
        bias_vec = view_z_axis * bias_dist
        return [p + bias_vec for p in points]
    else:
        real_amount = persp_percent / 100.0
        view_inv = rv3d.view_matrix.inverted()
        cam_pos = view_inv.translation
        factor = 1.0 - real_amount
        cam_part = cam_pos * real_amount
        return [p * factor + cam_part for p in points]

def get_axis_aligned_color(vec, default_col, settings=None, toggle_key="USE_AXIS_COLORS"):
    if vec.length_squared < 1e-6: return default_col

    # --- FIX: Respect the requested toggle ---
    if settings and not settings.get(toggle_key, True):
        return default_col

    dim = settings.get("AXIS_DIM", 1.0) if settings else 1.0

    v_norm = vec.normalized()
    tol = 0.9999
    if abs(v_norm.dot(Vector((1, 0, 0)))) > tol: return (1.0 * dim, 0.1 * dim, 0.1 * dim, 1.0) # Brighter Red
    if abs(v_norm.dot(Vector((0, 1, 0)))) > tol: return (0.1 * dim, 0.7 * dim, 0.1 * dim, 1.0) # Toned Green
    if abs(v_norm.dot(Vector((0, 0, 1)))) > tol: return (0.2 * dim, 0.5 * dim, 1.0 * dim, 1.0) # Brighter Blue
    return default_col

def get_render_settings(ctx):
    """Fetches colors, sizes, and crucial 4K scaling factors."""
    prefs = {
        "LIFT_COMPASS": 4.0, "LIFT_ARC": 20.0, "LIFT_PERSP": 0.2,
        "LINE_COL": (1.0, 1.0, 0.0, 0.7),
        "COL_START": (0.8, 0.8, 0.2, 1.0), "COL_END": (0.2, 0.8, 0.2, 1.0),
        "COL_CHORD": (0.2, 0.8, 0.2, 1.0), "COL_HEIGHT": (0.2, 0.8, 0.2, 1.0),
        "PREVIEW_VERTEX_SIZE": 5,
        "UI_SCALE": 1.0,
        "VIEWPORT_SIZE": (100.0, 100.0),
        "USE_AXIS_COLORS": True,
        "AXIS_DIM": 1.0,
        "LINE_PERP_SHOW_CATMULL": True,
        "LINE_PERP_COL_CATMULL": (0.0, 0.8, 1.0, 0.5),
        "LINE_PERP_WIDTH_CATMULL": 2.0,
        "LINE_PERP2_SHOW_CATMULL": True,
        "LINE_PERP2_COL_CATMULL": (0.0, 0.8, 1.0, 0.5),
        "LINE_PERP2_WIDTH_CATMULL": 2.0,
        "LINE_TANGENT_SHOW_CATMULL": True,
        "LINE_TANGENT_COL_CATMULL": (0.0, 0.8, 1.0, 0.5),
        "LINE_TANGENT_WIDTH_CATMULL": 2.0,
        "LINE_TAN_TAN_SHOW_CATMULL": True,
        "LINE_TAN_TAN_COL_CATMULL": (0.0, 0.8, 1.0, 0.5),
        "LINE_TAN_TAN_WIDTH_CATMULL": 2.0,
        "CIRCLE_TAN3_SHOW_CURVES": True,
        "CIRCLE_TAN3_COL_CURVES": (0.0, 0.8, 1.0, 0.5),
        "CIRCLE_TAN3_WIDTH_CURVES": 2.0,
        "CIRCLE_TAN3_SHOW_TANGENT": True,
        "CIRCLE_TAN3_COL_TANGENT": (0.0, 0.8, 1.0, 0.5),
        "CIRCLE_TAN3_COL_PREVIEW": (0.0, 0.0, 0.0, 1.0),
        "CIRCLE_TAN3_WIDTH_TANGENT": 2.0,
        "CIRCLE_TAN2_SHOW_CURVES": True,
        "CIRCLE_TAN2_COL_CURVES": (0.0, 0.8, 1.0, 0.5),
        "CIRCLE_TAN2_COL_PREVIEW": (0.0, 0.0, 0.0, 1.0),
        "CIRCLE_TAN2_WIDTH_CURVES": 2.0,
        "CIRCLE_TAN2_SHOW_TANGENT": True,
        "CIRCLE_TAN2_COL_TANGENT": (0.0, 0.8, 1.0, 0.5),
        "CIRCLE_TAN2_WIDTH_TANGENT": 2.0,
        "ELLIPSE_FOCI_COL_FOCI": (0.0, 1.0, 0.0, 1.0),
        "ELLIPSE_FOCI_USE_AXIS_COLORS": True,
        "COL_OVERLAY_ELLIPSE_FOCI": (0.1, 0.1, 0.1, 1.0),
        "ARC_2PT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_2PT": (0.3, 0.3, 0.3, 0.5),
        "ARC_3PT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_3PT": (0.3, 0.3, 0.3, 0.5),
        "CIRCLE_2PT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_C2PT": (0.3, 0.3, 0.3, 0.5),
        "CIRCLE_3PT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_C3PT": (0.3, 0.3, 0.3, 0.5),
        "ELLIPSE_RADIUS_USE_AXIS_COLORS": True,
        "COL_OVERLAY_ELLIPSE_RADIUS": (0.1, 0.1, 0.1, 1.0),
        "ELLIPSE_ENDPOINTS_USE_AXIS_COLORS": True,
        "COL_OVERLAY_ELLIPSE_ENDPOINTS": (0.1, 0.1, 0.1, 1.0),
        "ELLIPSE_CORNERS_COLOR": (0.0, 1.0, 0.0, 1.0),
        "POLYGON_CENTER_CORNER_USE_AXIS_COLORS": True,
        "COL_OVERLAY_POLYGON_CENTER_CORNER": (0.1, 0.1, 0.1, 1.0),
        "POLYGON_CENTER_TANGENT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_POLYGON_CENTER_TANGENT": (0.1, 0.1, 0.1, 1.0),
        "POLYGON_CORNER_CORNER_USE_AXIS_COLORS": True,
        "COL_OVERLAY_POLYGON_CORNER_CORNER": (0.1, 0.1, 0.1, 1.0),
        "POLYGON_EDGE_USE_AXIS_COLORS": True,
        "COL_OVERLAY_POLYGON_EDGE": (0.1, 0.1, 0.1, 1.0),
        "RECTANGLE_CENTER_CORNER_USE_AXIS_COLORS": True,
        "COL_OVERLAY_RECTANGLE_CENTER_CORNER": (0.1, 0.1, 0.1, 1.0),
        "RECTANGLE_CORNER_CORNER_USE_AXIS_COLORS": True,
        "COL_OVERLAY_RECTANGLE_CORNER_CORNER": (0.1, 0.1, 0.1, 1.0),
        "RECTANGLE_3PT_USE_AXIS_COLORS": True,
        "COL_OVERLAY_RECTANGLE_3PT": (0.1, 0.1, 0.1, 1.0)
    }

    system_prefs = ctx.preferences.system
    prefs["UI_SCALE"] = system_prefs.ui_scale

    vp = gpu.state.viewport_get()
    prefs["VIEWPORT_SIZE"] = (float(vp[2]), float(vp[3]))

    pkg = __package__ if __package__ else "radCAD"
    if '.' in pkg: pkg = pkg.split('.')[0] 

    try:
        addon_prefs = ctx.preferences.addons[pkg].preferences
        prefs["USE_AXIS_COLORS"] = addon_prefs.use_axis_colors
        prefs["AXIS_DIM"] = getattr(addon_prefs, "axis_color_dim", 1.0)
        prefs["LINE_PERP_SHOW_CATMULL"] = getattr(addon_prefs, "line_perp_show_catmull", True)
        prefs["LINE_PERP_COL_CATMULL"] = getattr(addon_prefs, "line_perp_col_catmull", (0.0, 0.8, 1.0, 0.5))
        prefs["LINE_PERP_WIDTH_CATMULL"] = getattr(addon_prefs, "line_perp_width_catmull", 2.0)
        prefs["LINE_PERP2_SHOW_CATMULL"] = getattr(addon_prefs, "line_perp2_show_catmull", True)
        prefs["LINE_PERP2_COL_CATMULL"] = getattr(addon_prefs, "line_perp2_col_catmull", (0.0, 0.8, 1.0, 0.5))
        prefs["LINE_PERP2_WIDTH_CATMULL"] = getattr(addon_prefs, "line_perp2_width_catmull", 2.0)
        
        prefs["LINE_TANGENT_SHOW_CATMULL"] = getattr(addon_prefs, "line_tangent_show_catmull", True)
        prefs["LINE_TANGENT_COL_CATMULL"] = getattr(addon_prefs, "line_tangent_col_catmull", (0.0, 0.8, 1.0, 0.5))
        prefs["LINE_TANGENT_WIDTH_CATMULL"] = getattr(addon_prefs, "line_tangent_width_catmull", 2.0)

        prefs["LINE_TAN_TAN_SHOW_CATMULL"] = getattr(addon_prefs, "line_tan_tan_show_catmull", True)
        prefs["LINE_TAN_TAN_COL_CATMULL"] = getattr(addon_prefs, "line_tan_tan_col_catmull", (0.0, 0.8, 1.0, 0.5))
        prefs["LINE_TAN_TAN_WIDTH_CATMULL"] = getattr(addon_prefs, "line_tan_tan_width_catmull", 2.0)

        prefs["CIRCLE_TAN3_SHOW_CURVES"] = getattr(addon_prefs, "circle_tan3_show_curves", True)
        prefs["CIRCLE_TAN3_COL_CURVES"] = tuple(getattr(addon_prefs, "circle_tan3_col_curves", (0.0, 0.8, 1.0, 0.5)))
        prefs["CIRCLE_TAN3_WIDTH_CURVES"] = getattr(addon_prefs, "circle_tan3_width_curves", 2.0)

        prefs["CIRCLE_TAN3_SHOW_TANGENT"] = getattr(addon_prefs, "circle_tan3_show_tangent", True)
        prefs["CIRCLE_TAN3_COL_TANGENT"] = tuple(getattr(addon_prefs, "circle_tan3_col_tangent", (0.0, 0.8, 1.0, 0.5)))
        prefs["CIRCLE_TAN3_COL_PREVIEW"] = tuple(getattr(addon_prefs, "circle_tan3_col_preview", (0.0, 0.0, 0.0, 1.0)))
        prefs["CIRCLE_TAN3_WIDTH_TANGENT"] = getattr(addon_prefs, "circle_tan3_width_tangent", 2.0)

        prefs["CIRCLE_TAN2_SHOW_CURVES"] = getattr(addon_prefs, "circle_tan2_show_curves", True)
        prefs["CIRCLE_TAN2_COL_CURVES"] = tuple(getattr(addon_prefs, "circle_tan2_col_curves", (0.0, 0.8, 1.0, 0.5)))
        prefs["CIRCLE_TAN2_COL_PREVIEW"] = tuple(getattr(addon_prefs, "circle_tan2_col_preview", (0.0, 0.0, 0.0, 1.0)))
        prefs["CIRCLE_TAN2_WIDTH_CURVES"] = getattr(addon_prefs, "circle_tan2_width_curves", 2.0)

        prefs["CIRCLE_TAN2_SHOW_TANGENT"] = getattr(addon_prefs, "circle_tan2_show_tangent", True)
        prefs["CIRCLE_TAN2_COL_TANGENT"] = tuple(getattr(addon_prefs, "circle_tan2_col_tangent", (0.0, 0.8, 1.0, 0.5)))
        prefs["CIRCLE_TAN2_WIDTH_TANGENT"] = getattr(addon_prefs, "circle_tan2_width_tangent", 2.0)
        
        prefs["ELLIPSE_FOCI_COL_FOCI"] = tuple(getattr(addon_prefs, "ellipse_foci_col_foci_lines", (0.0, 1.0, 0.0, 1.0)))
        prefs["ELLIPSE_FOCI_USE_AXIS_COLORS"] = getattr(addon_prefs, "ellipse_foci_use_axis_colors", True)
        prefs["COL_OVERLAY_ELLIPSE_FOCI"] = getattr(addon_prefs, "color_ellipse_foci_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["LIFT_COMPASS"] = addon_prefs.lift_compass
        prefs["LIFT_ARC"] = addon_prefs.lift_arc
        prefs["LIFT_PERSP"] = addon_prefs.lift_perspective
        prefs["LINE_COL"] = addon_prefs.snap_line_color
        prefs["COL_START"] = addon_prefs.color_arc_start
        prefs["COL_END"] = addon_prefs.color_arc_end
        prefs["ARC_2PT_USE_AXIS_COLORS"] = addon_prefs.arc_2pt_use_axis_colors
        prefs["COL_OVERLAY_2PT"] = addon_prefs.color_arc_2pt_overlay
        
        prefs["SNAP_MARKER_SIZE_3PT"] = addon_prefs.snap_marker_size_3pt
        prefs["SNAP_MARKER_COL_3PT"] = addon_prefs.snap_marker_color_3pt
        prefs["SNAP_LINE_COL_3PT"] = addon_prefs.snap_line_color_3pt
        prefs["ARC_3PT_USE_AXIS_COLORS"] = addon_prefs.arc_3pt_use_axis_colors
        prefs["COL_OVERLAY_3PT"] = addon_prefs.color_arc_3pt_overlay

        prefs["SNAP_MARKER_SIZE_C2PT"] = addon_prefs.snap_marker_size_c2pt
        prefs["SNAP_MARKER_COL_C2PT"] = addon_prefs.snap_marker_color_c2pt
        prefs["SNAP_LINE_COL_C2PT"] = addon_prefs.snap_line_color_c2pt
        prefs["CIRCLE_2PT_USE_AXIS_COLORS"] = addon_prefs.circle_2pt_use_axis_colors
        prefs["COL_OVERLAY_C2PT"] = addon_prefs.color_circle_2pt_overlay

        prefs["SNAP_MARKER_SIZE_C3PT"] = addon_prefs.snap_marker_size_c3pt
        prefs["SNAP_MARKER_COL_C3PT"] = addon_prefs.snap_marker_color_c3pt
        prefs["SNAP_LINE_COL_C3PT"] = addon_prefs.snap_line_color_c3pt
        prefs["CIRCLE_3PT_USE_AXIS_COLORS"] = addon_prefs.circle_3pt_use_axis_colors
        prefs["COL_OVERLAY_C3PT"] = addon_prefs.color_circle_3pt_overlay

        prefs["ELLIPSE_RADIUS_USE_AXIS_COLORS"] = getattr(addon_prefs, "ellipse_radius_use_axis_colors", True)
        prefs["COL_OVERLAY_ELLIPSE_RADIUS"] = getattr(addon_prefs, "color_ellipse_radius_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["ELLIPSE_ENDPOINTS_USE_AXIS_COLORS"] = getattr(addon_prefs, "ellipse_endpoints_use_axis_colors", True)
        prefs["COL_OVERLAY_ELLIPSE_ENDPOINTS"] = getattr(addon_prefs, "color_ellipse_endpoints_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["ELLIPSE_CORNERS_COLOR"] = getattr(addon_prefs, "ellipse_corners_color", (0.0, 1.0, 0.0, 1.0))

        prefs["POLYGON_CENTER_CORNER_USE_AXIS_COLORS"] = getattr(addon_prefs, "polygon_center_corner_use_axis_colors", True)
        prefs["COL_OVERLAY_POLYGON_CENTER_CORNER"] = getattr(addon_prefs, "color_polygon_center_corner_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["POLYGON_CENTER_TANGENT_USE_AXIS_COLORS"] = getattr(addon_prefs, "polygon_center_tangent_use_axis_colors", True)
        prefs["COL_OVERLAY_POLYGON_CENTER_TANGENT"] = getattr(addon_prefs, "color_polygon_center_tangent_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["POLYGON_CORNER_CORNER_USE_AXIS_COLORS"] = getattr(addon_prefs, "polygon_corner_corner_use_axis_colors", True)
        prefs["COL_OVERLAY_POLYGON_CORNER_CORNER"] = getattr(addon_prefs, "color_polygon_corner_corner_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["POLYGON_EDGE_USE_AXIS_COLORS"] = getattr(addon_prefs, "polygon_edge_use_axis_colors", True)
        prefs["COL_OVERLAY_POLYGON_EDGE"] = getattr(addon_prefs, "color_polygon_edge_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["RECTANGLE_CENTER_CORNER_USE_AXIS_COLORS"] = getattr(addon_prefs, "rectangle_center_corner_use_axis_colors", True)
        prefs["COL_OVERLAY_RECTANGLE_CENTER_CORNER"] = getattr(addon_prefs, "color_rectangle_center_corner_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["RECTANGLE_CORNER_CORNER_USE_AXIS_COLORS"] = getattr(addon_prefs, "rectangle_corner_corner_use_axis_colors", True)
        prefs["COL_OVERLAY_RECTANGLE_CORNER_CORNER"] = getattr(addon_prefs, "color_rectangle_corner_corner_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["RECTANGLE_3PT_USE_AXIS_COLORS"] = getattr(addon_prefs, "rectangle_3pt_use_axis_colors", True)
        prefs["COL_OVERLAY_RECTANGLE_3PT"] = getattr(addon_prefs, "color_rectangle_3pt_overlay", (0.1, 0.1, 0.1, 1.0))

        prefs["PREVIEW_VERTEX_SIZE"] = addon_prefs.preview_vertex_size
        
        # --- NEW: Points by Arc Settings ---
        prefs["POINTS_BY_ARC_COL1"] = addon_prefs.color_points_by_arc_1
        prefs["POINTS_BY_ARC_COL2"] = addon_prefs.color_points_by_arc_2
        prefs["POINTS_BY_ARC_CROSS_SIZE"] = addon_prefs.points_by_arc_crosshair_size
        prefs["POINTS_BY_ARC_SQUARE_SIZE"] = addon_prefs.points_by_arc_square_size
    except (KeyError, AttributeError):
        pass
        
    return prefs

# =========================================================================
#  PRIMITIVE DRAWERS
# =========================================================================

def setup_polyline_shader(sh, color, width, settings):
    """Configures the complex uniforms required by the Polyline shader."""
    sh.bind()
    sh.uniform_float("color", color)
    try:
        sh.uniform_float("viewportSize", settings["VIEWPORT_SIZE"])
        # FORCE 1.0 MINIMUM
        final_width = max(1.0, width * settings["UI_SCALE"])
        sh.uniform_float("lineWidth", final_width)
        sh.uniform_float("miterLimit", 1.0) 
    except Exception:
        pass

def draw_compass_geometry(ctx, shaders, center, Xp, Yp, rotation_radians, size_px, angle_inc, color, settings):
    """Draws the compass using the Polyline shader."""
    if center is None or Xp is None or Yp is None: return
    
    radius = world_radius_for_pixel_size(ctx, center, Xp, Yp, size_px)
    if radius <= 0: radius = 0.2
    
    outerR = radius; innerR = outerR * (80.0 / 120.0)
    tickLen = outerR * (10.0 / 120.0); crossLen = outerR * (10.0 / 120.0)
    safe_inc = max(angle_inc, 0.1); tickCount = int(360.0 / safe_inc)
    segs_circle = 72
    cosr, sinr = math.cos(rotation_radians), math.sin(rotation_radians)
    def R(ax, ay): return (ax * cosr - ay * sinr, ax * sinr + ay * cosr)
    
    # 1. Circle
    circle_segs = []
    for i in range(segs_circle):
        a1 = (i / segs_circle) * 2.0 * math.pi
        a2 = ((i + 1) / segs_circle) * 2.0 * math.pi
        ax1, ay1 = math.cos(a1) * outerR, math.sin(a1) * outerR
        ax2, ay2 = math.cos(a2) * outerR, math.sin(a2) * outerR
        rx1, ry1 = R(ax1, ay1); rx2, ry2 = R(ax2, ay2)
        p1 = center + Xp * rx1 + Yp * ry1
        p2 = center + Xp * rx2 + Yp * ry2
        circle_segs.extend([p1, p2])
        
    # 2. Ticks
    tick_segs = []
    for i in range(tickCount):
        a = (i / tickCount) * 2.0 * math.pi
        ax1, ay1 = math.cos(a) * outerR, math.sin(a) * outerR
        ax2, ay2 = math.cos(a) * (outerR - tickLen), math.sin(a) * (outerR - tickLen)
        rx1, ry1 = R(ax1, ay1); rx2, ry2 = R(ax2, ay2)
        tick_segs.extend([center + Xp * rx1 + Yp * ry1, center + Xp * rx2 + Yp * ry2])
        
    # 3. Arcs (RESTORED FROM STEAL THIS)
    arc_segs = []
    def add_arc_segs(a_start, a_end):
        steps = 48
        ax_start, ay_start = math.cos(a_start) * innerR, math.sin(a_start) * innerR
        rx_start, ry_start = R(ax_start, ay_start)
        p_start = center + Xp * rx_start + Yp * ry_start
        ax_end, ay_end = math.cos(a_end) * innerR, math.sin(a_end) * innerR
        rx_end, ry_end = R(ax_end, ay_end)
        p_end = center + Xp * rx_end + Yp * ry_end
        
        for i in range(steps):
            t1 = i / steps
            t2 = (i + 1) / steps
            ang1 = a_start + (a_end - a_start) * t1
            ang2 = a_start + (a_end - a_start) * t2
            ax1, ay1 = math.cos(ang1) * innerR, math.sin(ang1) * innerR
            ax2, ay2 = math.cos(ang2) * innerR, math.sin(ang2) * innerR
            rx1, ry1 = R(ax1, ay1); rx2, ry2 = R(ax2, ay2)
            p1 = center + Xp * rx1 + Yp * ry1
            p2 = center + Xp * rx2 + Yp * ry2
            arc_segs.extend([p1, p2])
        arc_segs.extend([p_start, p_end])
            
    add_arc_segs(math.radians(200), math.radians(340))
    add_arc_segs(math.radians(20), math.radians(160))
    
    # 4. Cross
    cross_segs = [
        center + Xp * (-crossLen), center + Xp * (crossLen), 
        center + Yp * (-crossLen), center + Yp * (crossLen)
    ]

    # Bias
    lift = settings["LIFT_COMPASS"]
    persp = settings["LIFT_PERSP"]
    circle_segs = apply_view_bias(circle_segs, ctx, lift_mult=lift, persp_percent=persp)
    tick_segs = apply_view_bias(tick_segs, ctx, lift_mult=lift, persp_percent=persp)
    arc_segs = apply_view_bias(arc_segs, ctx, lift_mult=lift, persp_percent=persp)
    cross_segs = apply_view_bias(cross_segs, ctx, lift_mult=lift, persp_percent=persp)

    # Draw
    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, 1.0, settings)
    
    if circle_segs: batch_for_shader(sh, 'LINES', {"pos": circle_segs}).draw(sh)
    if tick_segs: batch_for_shader(sh, 'LINES', {"pos": tick_segs}).draw(sh)
    if arc_segs: batch_for_shader(sh, 'LINES', {"pos": arc_segs}).draw(sh)
    if cross_segs: batch_for_shader(sh, 'LINES', {"pos": cross_segs}).draw(sh)

def draw_line(ctx, shaders, p1, p2, color, settings):
    """Draws a single high-quality line."""
    if p1 is None or p2 is None: return
    pts = apply_view_bias([p1, p2], ctx, lift_mult=settings["LIFT_ARC"], persp_percent=settings["LIFT_PERSP"])
    sh = shaders["POLYLINE"]
    
    # --- TEAL KILLER (Fixes the Perpendicular Light) ---
    # Catches Cyan/Teal (0, 1, 1) and forces it to Gold (1.0, 0.8, 0.0)
    final_color = color
    if len(color) >= 3 and color[0] < 0.1 and color[1] > 0.9 and color[2] > 0.9:
        final_color = (1.0, 0.8, 0.0, 1.0) # Gold
        
    setup_polyline_shader(sh, final_color, 1.0, settings)
    batch_for_shader(sh, 'LINES', {"pos": pts}).draw(sh)

def draw_polyline(ctx, shaders, points, color, settings, custom_lift=None, custom_width=1.0):
    if not points or len(points) < 2: return
    segments = []
    for i in range(len(points) - 1):
        segments.append(points[i])
        segments.append(points[i+1])
    
    lift = custom_lift if custom_lift is not None else settings["LIFT_ARC"]
    pts = apply_view_bias(segments, ctx, lift_mult=lift, persp_percent=settings["LIFT_PERSP"])
    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, custom_width, settings)
    batch_for_shader(sh, 'LINES', {"pos": pts}).draw(sh)

def get_round_point_shader():
    # Force fallback to avoid Blender 4.2+ crash
    return None

def draw_crosshair(ctx, shaders, points, color, size, settings, Xp, Yp, custom_lift=None):
    """Draws large crosshairs using the Polyline shader."""
    if not points: return
    lift = custom_lift if custom_lift is not None else settings.get("LIFT_ARC", 20.0)
    pts = apply_view_bias(points, ctx, lift_mult=lift, persp_percent=settings.get("LIFT_PERSP", 0.2))
    offset = world_radius_for_pixel_size(ctx, points[0], Xp, Yp, size)
    if offset <= 0: offset = 0.05
    segs = []
    for p in pts:
        segs.append(p - Xp * offset); segs.append(p + Xp * offset)
        segs.append(p - Yp * offset); segs.append(p + Yp * offset)
    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, 1.0, settings)
    batch_for_shader(sh, 'LINES', {"pos": segs}).draw(sh)

from bpy_extras import view3d_utils

def draw_points(ctx, shaders, points, color, size, settings, Xp=None, Yp=None, custom_lift=None):
    """Draws points as small 3D crosses using the POLYLINE shader (works in POST_VIEW/EEVEE-Next)."""
    if not points: return

    lift = custom_lift if custom_lift is not None else settings.get("LIFT_ARC", 20.0)
    pts = apply_view_bias(points, ctx, lift_mult=lift, persp_percent=settings.get("LIFT_PERSP", 0.2))

    # Use plane axes if provided, otherwise fall back to world axes
    if Xp is None: Xp = Vector((1, 0, 0))
    if Yp is None: Yp = Vector((0, 1, 0))
    Zp = Xp.cross(Yp).normalized()

    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, 2.0, settings)

    segs = []
    for p in pts:
        offset = world_radius_for_pixel_size(ctx, p, Xp, Yp, size / 2.0)
        if offset <= 0: offset = 0.01
        # Draw a small 3-axis cross
        segs += [p - Xp * offset, p + Xp * offset,
                 p - Yp * offset, p + Yp * offset,
                 p - Zp * offset, p + Zp * offset]

    batch_for_shader(sh, 'LINES', {"pos": segs}).draw(sh)
# =========================================================================
#  TOOL SPECIALISTS
# =========================================================================

def draw_preview_1point(ctx, shaders, prefs):
    # --- FIX: Ensure compass follows mouse even if plane is locked before pivot ---
    center = state["pivot"]
    if center is None:
        center = state.get("snap_point") or state.get("current") or state.get("last_surface_hit")
    
    Xc, Yc, Zc = state["Xp"], state["Yp"], state["Zp"]
    orient_normal = Zc if Zc else (state["locked_normal"] if (state["locked"] and state["locked_normal"]) else state["last_surface_normal"])
    
    if Xc is None or Yc is None:
        if orient_normal: Xc, Yc, Zc = orthonormal_basis_from_normal(orient_normal)
        
    # --- FIX: Default to BLACK if not aligned ---
    compass_col = (0.0, 0.0, 0.0, 1.0) # Black default
    
    if orient_normal:
        compass_col = get_axis_aligned_color(orient_normal, compass_col)
    
    c_size = state.get("compass_size", 125)
    a_inc = state.get("angle_increment", 15.0)
    
    if center and Xc and Yc:
        draw_compass_geometry(ctx, shaders, center, Xc, Yc, state["compass_rot"], c_size, a_inc, compass_col, prefs)

    pv = state["pivot"]
    if not pv: return

    # USE UNIFORM SIZE FROM PREFS (Default 5)
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)

    if state["stage"] == 1 and state["current"] is not None:
        draw_line(ctx, shaders, pv, state["current"], prefs["COL_START"], prefs)
        
        if state.get("tool_mode") == "CIRCLE_1POINT": # Logic kept for regular 1POINT arcs
            pass
    
    elif state["stage"] == 2:
        if state["start"]:
            draw_line(ctx, shaders, pv, state["start"], prefs["COL_START"], prefs)
        
        pts = state.get("preview_pts", [])
        if pts:
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)
            draw_line(ctx, shaders, pv, pts[-1], prefs["COL_END"], prefs)


def draw_preview_2point(ctx, shaders, prefs):
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)

    # --- STAGE 0: Initial Cursor Dot ---
    if state["stage"] == 0:
        target = state.get("snap_point") if state.get("snap_point") else state.get("current")
        if target:
            draw_points(ctx, shaders, [target], (0,0,0,1), pt_size, prefs)
    
    pv = state["pivot"]
    if not pv: return

    # --- STAGE 1: Dragging Chord (or Diameter) ---
    if state["stage"] == 1 and state["current"] is not None:
        diff = state["current"] - pv
        # Handle Circle 2 Point separately
        if state.get("tool_mode") == "CIRCLE_2POINT":
            draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
            draw_points(ctx, shaders, [state["current"]], (0,0,0,1), pt_size, prefs)
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_C2PT"], prefs, "CIRCLE_2PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pv, state["current"], col, prefs)
            pts = state.get("preview_pts", [])
            if pts:
                draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
                draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)
        else:
            # Arc 2Point: Draw endpoint dots
            if pv: draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
            if state["current"]: draw_points(ctx, shaders, [state["current"]], (0,0,0,1), pt_size, prefs)
            # Default to Custom Overlay Color (Defaults to Grey)
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_2PT"], prefs, "ARC_2PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pv, state["current"], col, prefs)

    # --- STAGE 2: Dragging Height (Arcs Only) ---
    elif state["stage"] == 2:
        p1 = state.get("p1")
        p2 = state.get("p2")
        
        if p1: draw_points(ctx, shaders, [p1], (0,0,0,1), pt_size, prefs)
        if p2: draw_points(ctx, shaders, [p2], (0,0,0,1), pt_size, prefs)
        
        if p1 and p2:
            chord_vec = p2 - p1
            # Default to Custom Overlay Color
            col_c = get_axis_aligned_color(chord_vec, prefs["COL_OVERLAY_2PT"], prefs, "ARC_2PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, p1, p2, col_c, prefs)
            
        if state["start"] is not None:
            peak = state["start"]
            height_vec = peak - pv
            # Default to Custom Overlay Color
            col_h = get_axis_aligned_color(height_vec, prefs["COL_OVERLAY_2PT"], prefs, "ARC_2PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pv, peak, col_h, prefs)
            
        pts = state.get("preview_pts", [])
        if pts:
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)


def draw_preview_3point(ctx, shaders, prefs):
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)
    if state["stage"] == 0:
        target = state.get("snap_point") if state.get("snap_point") else state.get("current")
        if target:
            draw_points(ctx, shaders, [target], (0,0,0,1), pt_size, prefs)

    pv = state["pivot"] 
    if not pv: return

    if state["stage"] == 1 and state["current"] is not None:
        draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
        diff = state["current"] - pv
        if state.get("tool_mode") == "CIRCLE_3POINT":
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_C3PT"], prefs, "CIRCLE_3PT_USE_AXIS_COLORS")
        else:
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_3PT"], prefs, "ARC_3PT_USE_AXIS_COLORS")
        draw_line(ctx, shaders, pv, state["current"], col, prefs)
        
    elif state["stage"] == 2:
        p1 = state.get("p1")
        p2 = state.get("p2")
        p3 = state.get("current")
        
        if p1: draw_points(ctx, shaders, [p1], (0,0,0,1), pt_size, prefs)
        if p2: draw_points(ctx, shaders, [p2], (0,0,0,1), pt_size, prefs)
        
        if p1 and p2:
            # Color chord by axis
            chord_vec = p2 - p1
            if state.get("tool_mode") == "CIRCLE_3POINT":
                col_c = get_axis_aligned_color(chord_vec, prefs["COL_OVERLAY_C3PT"], prefs, "CIRCLE_3PT_USE_AXIS_COLORS")
            else:
                col_c = get_axis_aligned_color(chord_vec, prefs["COL_OVERLAY_3PT"], prefs, "ARC_3PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, p1, p2, col_c, prefs)
            
        if p2 and p3:
            # Color curvature segment by axis
            curv_vec = p3 - p2
            if state.get("tool_mode") == "CIRCLE_3POINT":
                col_v = get_axis_aligned_color(curv_vec, prefs["COL_OVERLAY_C3PT"], prefs, "CIRCLE_3PT_USE_AXIS_COLORS")
            else:
                col_v = get_axis_aligned_color(curv_vec, prefs["COL_OVERLAY_3PT"], prefs, "ARC_3PT_USE_AXIS_COLORS")
            draw_line(ctx, shaders, p2, p3, col_v, prefs)
            
        pts = state.get("preview_pts", [])
        if pts:
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)

def draw_preview_ellipse(ctx, shaders, prefs):
    # Reuse standard compass drawing
    center = state["pivot"]
    
    mode = state.get("tool_mode")
    
    # Special Handling for Corners mode: Center is computed, not pivot
    if mode == "ELLIPSE_CORNERS" and state["stage"] == 1 and state["current"] is not None:
        # Re-calc center for display
        d_vec = state["current"] - state["pivot"]
        if state["Xp"] and state["Yp"]:
            width = d_vec.dot(state["Xp"])
            height = d_vec.dot(state["Yp"])
            center = state["pivot"] + (state["Xp"] * (width * 0.5)) + (state["Yp"] * (height * 0.5))
    
    if not center:
        # If in Stage 0, draw compass at surface hit
        center = state.get("last_surface_hit")
        
    pv = state["pivot"]
    if not pv: return
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)

    # --- ELLIPSE CORNERS DRAWING ---
    if mode == "ELLIPSE_CORNERS" and state["stage"] == 1:
        # Draw Bounding Box logic
        cur = state["current"]
        if cur and state["Xp"] and state["Yp"]:
            d_vec = cur - pv
            w = d_vec.dot(state["Xp"])
            h = d_vec.dot(state["Yp"])

            p1 = pv
            p2 = pv + state["Xp"] * w
            p3 = pv + state["Xp"] * w + state["Yp"] * h
            p4 = pv + state["Yp"] * h

            # Draw Box (no axis colors for corners)
            col_box = prefs.get("ELLIPSE_CORNERS_COLOR", (0.0, 1.0, 0.0, 1.0))
            draw_line(ctx, shaders, p1, p2, col_box, prefs)
            draw_line(ctx, shaders, p2, p3, col_box, prefs)
            draw_line(ctx, shaders, p3, p4, col_box, prefs)
            draw_line(ctx, shaders, p4, p1, col_box, prefs)

            # Draw Ellipse
            pts = state.get("preview_pts", [])
            if pts:
                 draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
                 draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)
        return

    # --- STANDARD ELLIPSE DRAWING (Radius/Endpoints/Foci) ---
    if state["stage"] == 1 and state["current"] is not None:
        draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
        if mode == "ELLIPSE_FOCI":
             draw_points(ctx, shaders, [state["current"]], (0,0,0,1), pt_size, prefs)

        pts = state.get("preview_pts", [])
        if mode == "ELLIPSE_RADIUS" and len(pts) >= 2:
            # Draw both halves of the symmetric diameter
            col = get_axis_aligned_color(state["Xp"], prefs["COL_OVERLAY_ELLIPSE_RADIUS"], prefs, "ELLIPSE_RADIUS_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pts[0], pts[1], col, prefs)
        elif mode == "ELLIPSE_ENDPOINTS":
            diff = state["current"] - pv
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_ELLIPSE_ENDPOINTS"], prefs, "ELLIPSE_ENDPOINTS_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pv, state["current"], col, prefs)
        elif mode == "ELLIPSE_FOCI":
            diff = state["current"] - pv
            col = get_axis_aligned_color(diff, prefs["COL_OVERLAY_ELLIPSE_FOCI"], prefs, "ELLIPSE_FOCI_USE_AXIS_COLORS")
            draw_line(ctx, shaders, pv, state["current"], col, prefs)
        else:
            # ELLIPSE_CORNERS doesn't have axis colors
            diff = state["current"] - pv
            col = prefs.get("ELLIPSE_CORNERS_COLOR", (0.0, 1.0, 0.0, 1.0))
            draw_line(ctx, shaders, pv, state["current"], col, prefs)
        
    elif state["stage"] == 2:
        draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
        
        # Draw Foci specifically
        if mode == "ELLIPSE_FOCI" and "f1" in state and "f2" in state:
             draw_points(ctx, shaders, [state["f1"]], (0,0,0,1), pt_size, prefs)
             draw_points(ctx, shaders, [state["f2"]], (0,0,0,1), pt_size, prefs)
             if state["current"] is not None:
                  f_col = prefs.get("ELLIPSE_FOCI_COL_FOCI", (0.0, 1.0, 0.0, 1.0))
                  draw_line(ctx, shaders, state["f1"], state["current"], f_col, prefs)
                  draw_line(ctx, shaders, state["f2"], state["current"], f_col, prefs)
        
        # Draw Major Axis (Ghosted/Reference)
        if mode == "ELLIPSE_RADIUS" and "Xp" in state and "rx" in state:
            center = pv
            end_major = center + (state["Xp"] * state["rx"])
            start_major = center - (state["Xp"] * state["rx"])
            col_m = get_axis_aligned_color(state["Xp"], prefs["COL_OVERLAY_ELLIPSE_RADIUS"], prefs, "ELLIPSE_RADIUS_USE_AXIS_COLORS")
            draw_line(ctx, shaders, start_major, end_major, col_m, prefs)

        # Draw Full Diameter for Endpoint Mode
        if mode == "ELLIPSE_ENDPOINTS" and "p1" in state and "p2" in state:
            if state["p1"] and state["p2"]:
                diff = state["p2"] - state["p1"]
                col_d = get_axis_aligned_color(diff, prefs["COL_OVERLAY_ELLIPSE_ENDPOINTS"], prefs, "ELLIPSE_ENDPOINTS_USE_AXIS_COLORS")
                draw_line(ctx, shaders, state["p1"], state["p2"], col_d, prefs)

        # Draw Minor Axis line (to cursor)
        if state["current"] is not None:
             # Only draw the center-to-cursor line for radius/endpoint modes (they have axis colors)
             if mode == "ELLIPSE_RADIUS":
                  center = pv
                  diff_h = state["current"] - center
                  col_h = get_axis_aligned_color(diff_h, prefs["COL_OVERLAY_ELLIPSE_RADIUS"], prefs, "ELLIPSE_RADIUS_USE_AXIS_COLORS")
                  draw_line(ctx, shaders, center, state["current"], col_h, prefs)
             elif mode == "ELLIPSE_ENDPOINTS":
                  center = (state["p1"] + state["p2"]) * 0.5
                  diff_h = state["current"] - center
                  col_h = get_axis_aligned_color(diff_h, prefs["COL_OVERLAY_ELLIPSE_ENDPOINTS"], prefs, "ELLIPSE_ENDPOINTS_USE_AXIS_COLORS")
                  draw_line(ctx, shaders, center, state["current"], col_h, prefs)

        # Draw Ellipse Polyline
        pts = state.get("preview_pts", [])
        if pts:
             draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
             draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)

def draw_preview_polygon(ctx, shaders, prefs):
    pv = state["pivot"]
    if not pv: return
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)
    tool_mode = state.get("tool_mode", "")
    
    if state["stage"] == 1 and state["current"] is not None:
        draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
        
        pts = state.get("preview_pts", [])
        if tool_mode == "ELLIPSE_RADIUS" and len(pts) >= 2:
            # Draw both halves of the symmetric diameter
            draw_line(ctx, shaders, pv, pts[0], prefs["COL_START"], prefs)
            draw_line(ctx, shaders, pv, pts[1], prefs["COL_START"], prefs)
        else:
            diff = state["current"] - pv
            is_3pt_rect = tool_mode == "RECTANGLE_3_POINTS"
            is_other_rect = tool_mode in ["RECTANGLE_CENTER_CORNER", "RECTANGLE_CORNER_CORNER"]

            # Map tool_mode to preference keys
            tool_pref_map = {
                "POLYGON_CENTER_CORNER": ("POLYGON_CENTER_CORNER_USE_AXIS_COLORS", "COL_OVERLAY_POLYGON_CENTER_CORNER"),
                "POLYGON_CENTER_TANGENT": ("POLYGON_CENTER_TANGENT_USE_AXIS_COLORS", "COL_OVERLAY_POLYGON_CENTER_TANGENT"),
                "POLYGON_CORNER_CORNER": ("POLYGON_CORNER_CORNER_USE_AXIS_COLORS", "COL_OVERLAY_POLYGON_CORNER_CORNER"),
                "POLYGON_EDGE": ("POLYGON_EDGE_USE_AXIS_COLORS", "COL_OVERLAY_POLYGON_EDGE"),
                "RECTANGLE_CENTER_CORNER": ("RECTANGLE_CENTER_CORNER_USE_AXIS_COLORS", "COL_OVERLAY_RECTANGLE_CENTER_CORNER"),
                "RECTANGLE_CORNER_CORNER": ("RECTANGLE_CORNER_CORNER_USE_AXIS_COLORS", "COL_OVERLAY_RECTANGLE_CORNER_CORNER"),
                "RECTANGLE_3_POINTS": ("RECTANGLE_3PT_USE_AXIS_COLORS", "COL_OVERLAY_RECTANGLE_3PT"),
            }

            if tool_mode in tool_pref_map:
                toggle_key, color_key = tool_pref_map[tool_mode]
                col = get_axis_aligned_color(diff, prefs[color_key], prefs, toggle_key)
            else:
                col = get_axis_aligned_color(diff, (0.5, 0.5, 0.5, 1.0))

            is_aligned = (col != (0.5, 0.5, 0.5, 1.0))

            # Draw line for polygons and 3-point rectangle (if aligned)
            if not is_other_rect:
                if not is_3pt_rect or is_aligned:
                    draw_line(ctx, shaders, pv, state["current"], col, prefs)
            
        draw_points(ctx, shaders, [state["current"]], (0,0,0,1), pt_size, prefs)
        
        if pts and tool_mode != "ELLIPSE_RADIUS":
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)
    
    # RECTANGLE 3 POINT SUPPORT
    if state["tool_mode"] == "RECTANGLE_3_POINTS" and state["stage"] == 2:
        pts = state.get("preview_pts", [])
        if pts:
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)


def draw_preview_circle_3point(ctx, shaders, prefs):
    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)
    
    # --- STAGE 0: Initial Cursor Dot ---
    if state["stage"] == 0:
        target = state.get("snap_point") if state.get("snap_point") else state.get("current")
        if target:
            draw_points(ctx, shaders, [target], (0,0,0,1), pt_size, prefs)

    pv = state.get("pivot")
    if not pv: return

    # --- STAGE 1: Dragging p2 ---
    if state["stage"] == 1 and state["current"] is not None:
        draw_points(ctx, shaders, [pv], (0,0,0,1), pt_size, prefs)
        draw_points(ctx, shaders, [state["current"]], (0,0,0,1), pt_size, prefs)
        
        diff = state["current"] - pv
        col = get_axis_aligned_color(diff, (0.5, 0.5, 0.5, 0.5))
        draw_line(ctx, shaders, pv, state["current"], col, prefs)
        
        # --- REMOVED: Black polyline in Stage 1 was muddying the axis line ---
        pass

    # --- STAGE 2: Dragging p3 ---
    elif state["stage"] == 2:
        p1 = state.get("p1")
        p2 = state.get("p2")
        p3 = state.get("current")
        
        if p1: draw_points(ctx, shaders, [p1], (0,0,0,1), pt_size, prefs)
        if p2: draw_points(ctx, shaders, [p2], (0,0,0,1), pt_size, prefs)
        if p3: draw_points(ctx, shaders, [p3], (0,0,0,1), pt_size, prefs)
        
        if p1 and p2:
            # Color chord by axis
            chord_vec = p2 - p1
            col_c = get_axis_aligned_color(chord_vec, (0.5, 0.5, 0.5, 0.5))
            draw_line(ctx, shaders, p1, p2, col_c, prefs)
            
        if p2 and p3:
            # Color curvature segment by axis
            curv_vec = p3 - p2
            col_v = get_axis_aligned_color(curv_vec, (0.5, 0.5, 0.5, 0.5))
            draw_line(ctx, shaders, p2, p3, col_v, prefs)
            
        pts = state.get("preview_pts", [])
        if pts:
            draw_polyline(ctx, shaders, pts, (0,0,0,1), prefs)
            draw_points(ctx, shaders, pts, (0,0,0,1), pt_size, prefs)

def draw_preview_tan_tan(ctx, shaders, prefs):
    if not prefs.get("CIRCLE_TAN2_SHOW_TANGENT", True): return

    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)
    t_col = prefs.get("CIRCLE_TAN2_COL_TANGENT", (0.0, 0.8, 1.0, 0.5))
    p_col = prefs.get("CIRCLE_TAN2_COL_PREVIEW", (0.0, 0.0, 0.0, 1.0))
    t_width = prefs.get("CIRCLE_TAN2_WIDTH_TANGENT", 2.0)

    # 1. Smooth Math Circle (Catmull) - Uses Preference Color (Blue)
    v_pts = state.get("visual_pts", [])
    if v_pts:
        draw_polyline(ctx, shaders, v_pts, t_col, prefs, custom_width=t_width)

    # 2. Mesh Geometry (Uses Preference Color, Default: Orange)
    p_pts = state.get("preview_pts", [])
    if p_pts:
        draw_polyline(ctx, shaders, p_pts, p_col, prefs, custom_width=1.0, custom_lift=prefs["LIFT_ARC"] + 2.0)
        draw_points(ctx, shaders, p_pts, p_col, pt_size, prefs, custom_lift=prefs["LIFT_ARC"] + 5.0)        
    # 3. Tangency Viz
    viz_tan = state.get("viz_tangent_line")
    if viz_tan and len(viz_tan) == 2:
        draw_line(ctx, shaders, viz_tan[0], viz_tan[1], (1, 0.8, 0, 1), prefs)
        
    viz_diam = state.get("viz_diameter_line")
    if viz_diam and len(viz_diam) == 2:
        draw_line(ctx, shaders, viz_diam[0], viz_diam[1], (1, 0.8, 0, 1), prefs)

def draw_preview_tan_tan_tan(ctx, shaders, prefs):
    if not prefs.get("CIRCLE_TAN3_SHOW_TANGENT", True): return

    pt_size = prefs.get("PREVIEW_VERTEX_SIZE", 5)
    t_col = prefs.get("CIRCLE_TAN3_COL_TANGENT", (0.0, 0.8, 1.0, 0.5))
    p_col = prefs.get("CIRCLE_TAN3_COL_PREVIEW", (0.0, 0.0, 0.0, 1.0))
    t_width = prefs.get("CIRCLE_TAN3_WIDTH_TANGENT", 2.0)

    # 1. Smooth Math Circle (Catmull) - Uses Preference Color (Blue)
    v_pts = state.get("visual_pts", [])
    if v_pts:
        draw_polyline(ctx, shaders, v_pts, t_col, prefs, custom_width=t_width)

    # 2. Mesh Geometry (Uses Preference Color, Default: Orange)
    p_pts = state.get("preview_pts", [])
    if p_pts:
        draw_polyline(ctx, shaders, p_pts, p_col, prefs, custom_width=1.0, custom_lift=prefs["LIFT_ARC"] + 2.0)
        # Match 2-curve behavior: draw points at every mesh vertex
        draw_points(ctx, shaders, p_pts, p_col, pt_size, prefs, custom_lift=prefs["LIFT_ARC"] + 5.0)

    # 3. Tangency Viz
    viz_tan = state.get("viz_tangent_line")
    if viz_tan and len(viz_tan) == 2:
        draw_line(ctx, shaders, viz_tan[0], viz_tan[1], (1, 0.8, 0, 1), prefs)
        
    viz_diam = state.get("viz_diameter_line")
    if viz_diam and len(viz_diam) == 2:
        draw_line(ctx, shaders, viz_diam[0], viz_diam[1], (1, 0.8, 0, 1), prefs)

def draw_cb_3d():
    if not state["active"]: return
    try:
        ctx = bpy.context
        is_xray = False
        if ctx.space_data and ctx.space_data.type == 'VIEW_3D':
            shading = ctx.space_data.shading
            if shading.type == 'WIREFRAME' or shading.show_xray:
                is_xray = True
        
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set('NONE' if is_xray else 'LESS_EQUAL')
        
        settings = get_render_settings(ctx)
        shaders = get_shaders()
        
        # --- NEW: DRAW GREY CATMULL OUTLINES FOR INPUT SELECTION ---
        catmull_outlines = state.get("catmull_spline_previews", [])
        if catmull_outlines:
            mode = state.get("tool_mode", "1POINT")
            
            # Default for selection outlines
            draw_col = (0.5, 0.5, 0.5, 0.7)
            do_draw = True
            
            # Override for specific tool
            if mode == "LINE_PERP_FROM_CURVE":
                do_draw = settings.get("LINE_PERP_SHOW_CATMULL", True)
                draw_col = settings.get("LINE_PERP_COL_CATMULL", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("LINE_PERP_WIDTH_CATMULL", 2.0)
            elif mode == "LINE_PERP_TO_TWO_CURVES":
                do_draw = settings.get("LINE_PERP2_SHOW_CATMULL", True)
                draw_col = settings.get("LINE_PERP2_COL_CATMULL", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("LINE_PERP2_WIDTH_CATMULL", 2.0)
            elif mode == "LINE_TANGENT_FROM_CURVE":
                do_draw = False
                draw_col = settings.get("LINE_TANGENT_COL_CATMULL", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("LINE_TANGENT_WIDTH_CATMULL", 2.0)
            elif mode == "LINE_TAN_TAN":
                do_draw = settings.get("LINE_TAN_TAN_SHOW_CATMULL", True)
                draw_col = settings.get("LINE_TAN_TAN_COL_CATMULL", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("LINE_TAN_TAN_WIDTH_CATMULL", 2.0)
            elif mode == "CIRCLE_TAN_TAN_TAN":
                do_draw = settings.get("CIRCLE_TAN3_SHOW_CURVES", True)
                draw_col = settings.get("CIRCLE_TAN3_COL_CURVES", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("CIRCLE_TAN3_WIDTH_CURVES", 2.0)
            elif mode == "CIRCLE_TAN_TAN":
                do_draw = settings.get("CIRCLE_TAN2_SHOW_CURVES", True)
                draw_col = settings.get("CIRCLE_TAN2_COL_CURVES", (0.0, 0.8, 1.0, 0.5))
                draw_width = settings.get("CIRCLE_TAN2_WIDTH_CURVES", 2.0)
            else:
                draw_width = 2.0

            if do_draw:
                for c_pts in catmull_outlines:
                    draw_polyline(ctx, shaders, c_pts, draw_col, settings, custom_lift=settings["LIFT_ARC"] - 5.0, custom_width=draw_width)
        
        # --- REMOVED SPLINE OVERLAYS TO PRESERVE ORANGE SELECTION ---

        mode = state.get("tool_mode", "1POINT")
        
        if mode == "POINT_BY_ARCS":
            center = state["pivot"] if state["pivot"] else state["last_surface_hit"]
            Xc, Yc = state["Xp"], state["Yp"]
            if center and Xc and Yc:
                draw_compass_geometry(ctx, shaders, center, Xc, Yc, state["compass_rot"], 125, 15.0, (0,0,0,1), settings)
            
            # --- COLORS FROM PREFS ---
            addon_prefs = ctx.preferences.addons["radCAD"].preferences
            arc1_col = addon_prefs.color_points_by_arc_1
            arc2_col = addon_prefs.color_points_by_arc_2
            start_col = addon_prefs.color_points_by_arc_start
            end_col = addon_prefs.color_points_by_arc_end
            cross_sz = addon_prefs.points_by_arc_crosshair_size
            square_sz = addon_prefs.points_by_arc_square_size

            # 1. Arc 1 (Always Arc 1 Color)
            if state.get("arc1_pts"):
                draw_polyline(ctx, shaders, state["arc1_pts"], arc1_col, settings)
            
            # 2. Active Preview Arc (Map to Arc 1 or Arc 2 based on Stage)
            if state.get("preview_pts"):
                if state["stage"] <= 2:
                    # Still drawing Arc 1
                    draw_polyline(ctx, shaders, state["preview_pts"], arc1_col, settings)
                else:
                    # Now drawing Arc 2
                    draw_polyline(ctx, shaders, state["preview_pts"], arc2_col, settings)
            
            # --- INTERSECTION MARKERS (Stage-Dependent) ---
            if state.get("intersection_pts"):
                # Size from Prefs
                if state["stage"] == 4:
                    draw_crosshair(ctx, shaders, state.get("intersection_pts"), (0, 0, 0, 1), cross_sz, settings, Xc, Yc, custom_lift=settings.get("LIFT_ARC", 20.0) + 50.0)
                else:
                    draw_points(ctx, shaders, state.get("intersection_pts"), (0, 0, 0, 1), square_sz, settings, custom_lift=settings.get("LIFT_ARC", 20.0) + 50.0)
            
            # Standard radius/angle guide lines (Yellow/Gold)
            pv = state.get("pivot")
            if pv:
                # Stage 1 & 4: Radius line follows mouse (snapped to circle)
                if state["stage"] in [1, 4] and state["current"]:
                    draw_line(ctx, shaders, pv, state["current"], start_col, settings)
                
                # Stage 2 & 5: Angle lines tethered to Arc Endpoints
                elif state["stage"] in [2, 5]:
                    if state.get("start"):
                        draw_line(ctx, shaders, pv, state["start"], start_col, settings)
                    pts = state.get("preview_pts")
                    if pts:
                        draw_line(ctx, shaders, pv, pts[-1], end_col, settings)
                
        elif mode == "POINT_CENTER":
            # Draw the Catmull-Rom curve over the selected polygon
            catmull_pts = state.get("catmull_center_pts", [])
            if catmull_pts:
                draw_polyline(ctx, shaders, catmull_pts, (0, 0, 0, 1), settings)
            # Draw the computed center vertex preview
            center_pts = state.get("intersection_pts", [])
            if center_pts:
                draw_points(ctx, shaders, center_pts, (0, 0, 0, 1), settings.get("PREVIEW_VERTEX_SIZE", 5), settings, custom_lift=settings.get("LIFT_ARC", 20.0) + 50.0)

        # UPDATED: Added all Line Curve Tools
        elif mode in ["LINE_POLY", "CURVE_INTERPOLATE", "CURVE_FREEHAND", "LINE_PERP_FROM_CURVE", "LINE_PERP_TO_TWO_CURVES", "LINE_TANGENT_FROM_CURVE", "LINE_TAN_TAN"]:
            
            pts = state.get("preview_pts", [])
            if pts:
                # DEFAULT COLOR
                base_color = (0, 0, 0, 1) # Black
                
                # If LINE_POLY, handle segment coloring for axis snaps
                if mode == "LINE_POLY" and len(pts) >= 2:
                    # Draw fixed segments (all except last segment)
                    if len(pts) > 2:
                        draw_polyline(ctx, shaders, pts[:-1], base_color, settings)
                    
                    # Draw active segment (last 2 points)
                    active_seg = pts[-2:]
                    active_col = base_color
                    
                    axis_vec = state.get("current_axis_vector")
                    if axis_vec:
                        # --- FIX: Only Line tools respect this toggle ---
                        active_col = get_axis_aligned_color(axis_vec, base_color, settings)
                    
                    draw_polyline(ctx, shaders, active_seg, active_col, settings)
                else:
                    # Fallback for other tools or single point
                    draw_polyline(ctx, shaders, pts, base_color, settings)

                # Dots
                draw_points(ctx, shaders, pts, base_color, settings.get("PREVIEW_VERTEX_SIZE", 5), settings) 

            # RESTORED: Hover Dot (But now Black Size 4, not Yellow Size 8)
            # EXCLUDE POINT_BY_ARCS from generic dot logic
            if mode != "POINT_BY_ARCS":
                if state.get("stage", 0) == 0:
                    target = state.get("snap_point") if state.get("snap_point") else state.get("current")
                    if target: draw_points(ctx, shaders, [target], (0.0, 0.0, 0.0, 1.0), settings.get("PREVIEW_VERTEX_SIZE", 5), settings)
                if state.get("current"):
                    draw_points(ctx, shaders, [state["current"]], (0.0, 0.0, 0.0, 1.0), settings.get("PREVIEW_VERTEX_SIZE", 5), settings)

        elif mode == "CIRCLE_TAN_TAN_TAN":
            draw_preview_tan_tan_tan(ctx, shaders, settings)
            
        elif mode == "CIRCLE_TAN_TAN":
            draw_preview_tan_tan(ctx, shaders, settings)

        elif mode == "1POINT":
            draw_preview_1point(ctx, shaders, settings)
            
        elif mode == "2POINT" or mode == "CIRCLE_2POINT":
            draw_preview_2point(ctx, shaders, settings)
            
        elif mode == "3POINT":
            draw_preview_3point(ctx, shaders, settings)
            
        elif mode == "CIRCLE_3POINT": 
            draw_preview_circle_3point(ctx, shaders, settings)
            
        elif mode in ["ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS", "ELLIPSE_FOCI", "ELLIPSE_CORNERS"]:
            draw_preview_ellipse(ctx, shaders, settings)
            
        elif mode in ["POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE", "RECTANGLE_CENTER_CORNER", "RECTANGLE_CORNER_CORNER", "RECTANGLE_3_POINTS"]: 
            draw_preview_polygon(ctx, shaders, settings)

        gpu.state.depth_test_set('NONE')
        gpu.state.blend_set("NONE")
    except Exception as e:
        print(f"DRAW ERROR: {e}")