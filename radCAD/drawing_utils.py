import math
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector
import bpy

from .modal_state import state
from .plane_utils import world_radius_for_pixel_size

# =========================================================================
#  SHADER MANAGEMENT
# =========================================================================

def get_shaders():
    """Returns a dict containing the two necessary shaders."""
    shaders = {}
    try: shaders["POLYLINE"] = gpu.shader.from_builtin('POLYLINE_UNIFORM_COLOR')
    except:
        try: shaders["POLYLINE"] = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
        except: shaders["POLYLINE"] = gpu.shader.from_builtin('UNIFORM_COLOR')
    
    try: shaders["UNIFORM"] = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
    except: shaders["UNIFORM"] = gpu.shader.from_builtin('UNIFORM_COLOR')
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

def get_axis_aligned_color(vec, default_col, settings=None):
    if vec.length_squared < 1e-6: return default_col

    # --- FIX: Respect the "Use Axis Colors" toggle ---
    if settings and not settings.get("USE_AXIS_COLORS", True):
        return default_col

    dim = settings.get("AXIS_DIM", 1.0) if settings else 1.0

    v_norm = vec.normalized()
    tol = 0.999999
    if abs(v_norm.dot(Vector((1, 0, 0)))) > tol: return (0.85 * dim, 0.00 * dim, 0.00 * dim, 1.00) # Red
    if abs(v_norm.dot(Vector((0, 1, 0)))) > tol: return (0.00 * dim, 0.60 * dim, 0.00 * dim, 1.00) # Green
    if abs(v_norm.dot(Vector((0, 0, 1)))) > tol: return (0.149 * dim, 0.376 * dim, 1.0 * dim, 1.0) # Blue
    return default_col

def get_render_settings(ctx):
    """Fetches colors, sizes, and crucial scaling factors."""
    prefs = {
        "LIFT_COMPASS": 4.0, "LIFT_ARC": 20.0, "LIFT_PERSP": 0.2,
        "LINE_COL": (1.0, 1.0, 0.0, 0.7),
        "COL_START": (0.8, 0.8, 0.2, 1.0), "COL_END": (0.2, 0.8, 0.2, 1.0),
        "COL_CHORD": (0.2, 0.8, 0.2, 1.0), "COL_HEIGHT": (0.2, 0.8, 0.2, 1.0),
        "UI_SCALE": 1.0, "VIEWPORT_SIZE": (100.0, 100.0),
        "USE_AXIS_COLORS": True,
        "AXIS_DIM": 1.0
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
        prefs["LIFT_COMPASS"] = addon_prefs.lift_compass
        prefs["LIFT_ARC"] = addon_prefs.lift_arc
        prefs["LIFT_PERSP"] = addon_prefs.lift_perspective
        prefs["LINE_COL"] = addon_prefs.snap_line_color
        prefs["COL_START"] = addon_prefs.color_arc_start
        prefs["COL_END"] = addon_prefs.color_arc_end
        prefs["COL_CHORD"] = addon_prefs.color_arc_2pt_chord
        prefs["COL_HEIGHT"] = addon_prefs.color_arc_2pt_height
    except: pass
    return prefs

# =========================================================================
#  PRIMITIVE DRAWERS
# =========================================================================

def setup_polyline_shader(sh, color, width, settings):
    sh.bind()
    sh.uniform_float("color", color)
    try:
        sh.uniform_float("viewportSize", settings["VIEWPORT_SIZE"])
        final_width = max(1.0, width * settings["UI_SCALE"])
        sh.uniform_float("lineWidth", final_width)
        sh.uniform_float("miterLimit", 1.0) 
    except: pass

def draw_compass_geometry(ctx, shaders, center, Xp, Yp, rotation_radians, size_px, angle_inc, color, settings):
    """The 'Correct' Compass logic with Protractor Arcs."""
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
        circle_segs.extend([center + Xp * rx1 + Yp * ry1, center + Xp * rx2 + Yp * ry2])
        
    # 2. Ticks
    tick_segs = []
    for i in range(tickCount):
        a = (i / tickCount) * 2.0 * math.pi
        ax1, ay1 = math.cos(a) * outerR, math.sin(a) * outerR
        ax2, ay2 = math.cos(a) * (outerR - tickLen), math.sin(a) * (outerR - tickLen)
        rx1, ry1 = R(ax1, ay1); rx2, ry2 = R(ax2, ay2)
        tick_segs.extend([center + Xp * rx1 + Yp * ry1, center + Xp * rx2 + Yp * ry2])
        
    # 3. Arcs (The fancy protractor look)
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
    cross_segs = [center + Xp * (-crossLen), center + Xp * (crossLen), center + Yp * (-crossLen), center + Yp * (crossLen)]

    lift = settings["LIFT_COMPASS"]
    persp = settings["LIFT_PERSP"]
    circle_segs = apply_view_bias(circle_segs, ctx, lift_mult=lift, persp_percent=persp)
    tick_segs = apply_view_bias(tick_segs, ctx, lift_mult=lift, persp_percent=persp)
    arc_segs = apply_view_bias(arc_segs, ctx, lift_mult=lift, persp_percent=persp)
    cross_segs = apply_view_bias(cross_segs, ctx, lift_mult=lift, persp_percent=persp)

    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, 1.0, settings)
    if circle_segs: batch_for_shader(sh, 'LINES', {"pos": circle_segs}).draw(sh)
    if tick_segs: batch_for_shader(sh, 'LINES', {"pos": tick_segs}).draw(sh)
    if arc_segs: batch_for_shader(sh, 'LINES', {"pos": arc_segs}).draw(sh)
    if cross_segs: batch_for_shader(sh, 'LINES', {"pos": cross_segs}).draw(sh)

def draw_line(ctx, shaders, p1, p2, color, settings):
    if p1 is None or p2 is None: return
    pts = apply_view_bias([p1, p2], ctx, lift_mult=settings["LIFT_ARC"], persp_percent=settings["LIFT_PERSP"])
    sh = shaders["POLYLINE"]
    
    # --- TEAL KILLER (Fixes the Perpendicular Light) ---
    final_color = color
    if len(color) >= 3 and color[0] < 0.1 and color[1] > 0.9 and color[2] > 0.9:
        final_color = (1.0, 0.8, 0.0, 1.0) # Gold
        
    setup_polyline_shader(sh, final_color, 1.0, settings)
    batch_for_shader(sh, 'LINES', {"pos": pts}).draw(sh)

def draw_polyline(ctx, shaders, points, color, settings):
    if not points or len(points) < 2: return
    segments = []
    for i in range(len(points) - 1):
        segments.append(points[i])
        segments.append(points[i+1])
    pts = apply_view_bias(segments, ctx, lift_mult=settings["LIFT_ARC"], persp_percent=settings["LIFT_PERSP"])
    sh = shaders["POLYLINE"]
    setup_polyline_shader(sh, color, 1.0, settings)
    batch_for_shader(sh, 'LINES', {"pos": pts}).draw(sh)

def get_round_point_shader():
    # Force fallback to avoid Blender 4.2+ crash on custom shaders
    return None

def draw_points(ctx, shaders, points, color, size, settings):
    if not points: return
    pts = apply_view_bias(points, ctx, lift_mult=settings["LIFT_ARC"], persp_percent=settings["LIFT_PERSP"])
    sh = get_round_point_shader()
    
    if sh:
        sh.bind()
        sh.uniform_float("color", color)
        final_size = max(1.0, size * settings["UI_SCALE"])
        sh.uniform_float("size", final_size)
        batch_for_shader(sh, 'POINTS', {"pos": pts}).draw(sh)
    else:
        sh = shaders["UNIFORM"]
        sh.bind()
        sh.uniform_float("color", color)
        final_size = max(1.0, size * settings["UI_SCALE"])
        gpu.state.point_size_set(int(final_size))
        batch_for_shader(sh, 'POINTS', {"pos": pts}).draw(sh)