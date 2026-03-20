# radCAD/hud_overlay.py
import bpy
import blf
import gpu
import math
from gpu_extras.batch import batch_for_shader
from bpy_extras.view3d_utils import location_3d_to_region_2d
from .modal_state import state, style
from .units_utils import format_length

# --- FONT SIZES ---
SIZE_KEY = 12
SIZE_LABEL = 10 

# --- SAFE SHADER LOADER (2D) ---
def get_2d_shader():
    try:
        return gpu.shader.from_builtin('2D_UNIFORM_COLOR')
    except Exception:
        return gpu.shader.from_builtin('UNIFORM_COLOR')

# --- HELPER: MEASURE MIXED TEXT ---
def get_mixed_text_metrics(font_id, text):
    """
    Splits text into Key (Size 12) and Label (Size 10).
    Returns (key_str, label_str, width_key, width_label, text_height)
    """
    if ":" in text:
        parts = text.split(":", 1)
        key_str = parts[0] + ":"
        label_str = parts[1]
    elif "\u2220" in text: # Angle Symbol
        key_str = "\u2220"
        label_str = text.replace("\u2220", "")
    else:
        key_str = text
        label_str = ""
        
    # Measure Key
    blf.size(font_id, SIZE_KEY)
    w_key = blf.dimensions(font_id, key_str)[0]
    h_key = blf.dimensions(font_id, "Hg")[1] # Use generic height
    
    # Measure Label
    blf.size(font_id, SIZE_LABEL)
    w_lbl = blf.dimensions(font_id, label_str)[0]
    h_lbl = blf.dimensions(font_id, "Hg")[1]
    
    # Use the max height of the Key font (12) to determine vertical center
    total_h = max(h_key, h_lbl)
    
    return key_str, label_str, w_key, w_lbl, total_h

# --- GENERIC BOX (Upper Overlay & Measurements) ---
def draw_ui_box_generic(x, y, text, active=False, bg_color_override=None):
    font_id = 0
    pad = style["bg_padding"]
    
    # 1. Measure
    key_str, label_str, w_key, w_lbl, txt_h = get_mixed_text_metrics(font_id, text)
    
    total_w = w_key + w_lbl
    box_w = total_w + (pad * 2)
    box_h = txt_h + (pad * 2) 
    
    x1, y1 = x, y
    x2, y2 = x + box_w, y - box_h 
    
    verts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    
    # SAFE SHADER
    sh = get_2d_shader()
    gpu.state.blend_set("ALPHA")
    sh.bind()
    
    bg_col = bg_color_override if bg_color_override else (style["bg_color_active"] if active else style["bg_color"])
    sh.uniform_float("color", bg_col)
    
    batch_for_shader(sh, 'TRI_FAN', {"pos": verts}).draw(sh)
    
    # 2. Draw Text (Vertically Centered)
    blf.color(font_id, *style["font_color"])
    
    # Calculate Y to center text: Middle of box - 1/3 of text height
    center_y = y - (box_h / 2)
    text_y = center_y - (txt_h / 3.0)
    
    # Draw Key (Size 12)
    blf.size(font_id, SIZE_KEY)
    blf.position(font_id, x + pad, text_y, 0)
    blf.draw(font_id, key_str)
    
    # Draw Label (Size 10)
    blf.size(font_id, SIZE_LABEL)
    blf.position(font_id, x + pad + w_key, text_y, 0)
    blf.draw(font_id, label_str)
    
    gpu.state.blend_set("NONE")
    return box_h

# --- BUTTON DRAWER (Bottom Bar) ---
def draw_ui_button(x, y, text, is_active, hitbox_id):
    font_id = 0
    pad_x = 10
    pad_y = 8 # Increased slightly for easier clicking
    
    # 1. Measure
    key_str, label_str, w_key, w_lbl, txt_h = get_mixed_text_metrics(font_id, text)
    
    total_text_w = w_key + w_lbl
    
    # Consistent height based on font size + padding
    box_w = total_text_w + (pad_x * 2)
    box_h = txt_h + (pad_y * 2) + 4
    
    x1, y1 = x, y
    x2, y2 = x + box_w, y + box_h
    
    if hitbox_id:
        state["ui_hitboxes"][hitbox_id] = (x1, x2, y1, y2)
    
    # === ROUNDED RECT GENERATION ===
    rad = 5  # Corner Radius
    segs = 6 # Smoothness
    verts = []
    
    # Helper to append arc vertices
    def add_arc(cx, cy, start_ang, end_ang):
        for i in range(segs + 1):
            a = start_ang + (end_ang - start_ang) * (i / segs)
            verts.append((cx + rad * math.cos(a), cy + rad * math.sin(a)))

    # Order: Top-Right, Top-Left, Bottom-Left, Bottom-Right
    add_arc(x2 - rad, y2 - rad, 0, math.pi / 2)           # TR
    add_arc(x1 + rad, y2 - rad, math.pi / 2, math.pi)     # TL
    add_arc(x1 + rad, y1 + rad, math.pi, 1.5 * math.pi)   # BL
    add_arc(x2 - rad, y1 + rad, 1.5 * math.pi, 2 * math.pi) # BR
    
    # === DRAW BACKGROUND ===
    bg_col = style["bg_color_active"] if is_active else style["bg_color"]
    sh = get_2d_shader()
    gpu.state.blend_set("ALPHA")
    sh.bind()
    sh.uniform_float("color", bg_col)
    
    batch_for_shader(sh, 'TRI_FAN', {"pos": verts}).draw(sh)
    
    # === BUTTON STYLING (Outline Only) ===
    gpu.state.line_width_set(1.0)
    sh.uniform_float("color", (0.0, 0.0, 0.0, 0.5))
    batch_for_shader(sh, 'LINE_LOOP', {"pos": verts}).draw(sh)
    # ---------------------------------------

    # 2. Draw Text (CENTERED VERTICALLY & HORIZONTALLY)
    blf.color(font_id, *style["font_color"])
    
    # Horizontal Center
    box_center_x = x1 + (box_w / 2)
    start_tx = box_center_x - (total_text_w / 2)
    
    # Vertical Center
    box_center_y = y1 + (box_h / 2)
    text_y = box_center_y - (txt_h / 3.0) 
    
    # Draw Key (Size 12)
    blf.size(font_id, SIZE_KEY)
    blf.position(font_id, start_tx, text_y, 0)
    blf.draw(font_id, key_str)
    
    # Draw Label (Size 10)
    blf.size(font_id, SIZE_LABEL)
    blf.position(font_id, start_tx + w_key, text_y, 0)
    blf.draw(font_id, label_str)
    
    gpu.state.blend_set("NONE")
    return box_w

    # --- TOP RIGHT HOTKEYS ---
def draw_hotkeys_panel():
    if not state.get("show_hotkeys", True): return
    if state.get("tool_mode") in ["LINE_TANGENT_FROM_CURVE", "CURVE_FREEHAND"]: return

    perp_state = "ON" if state.get("is_perpendicular") else "OFF"
    perp_col = None 

    tool_mode = state.get("tool_mode", "1POINT")
    lines = []
    
    # --- ONLY SHOW PERPENDICULAR ONCE ARC DRAWING STARTS ---
    show_perp = False
    if tool_mode in ["2POINT", "3POINT"] and state["stage"] == 2:
        show_perp = True
    elif tool_mode in ["CIRCLE_2POINT", "CIRCLE_3POINT"] and state["stage"] == 1:
        show_perp = True
    elif tool_mode == "1POINT" and state["stage"] >= 1:
        show_perp = True
    elif tool_mode in ["ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS", "ELLIPSE_FOCI", "POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE", "RECTANGLE_3_POINTS"] and state["stage"] >= 1:
        show_perp = True
    elif tool_mode == "ELLIPSE_CORNERS" and state["stage"] == 1:
        show_perp = True

    if show_perp:
        lines.append((f"P: Perpendicular ({perp_state})", perp_col))

    # --- INSERT SPACER IF RADIUS/DISTANCE IS ABOUT TO BE SHOWN ---
    if state["stage"] >= 1:
        lines.append((None, None))
        if state.get("tool_mode") in ["2POINT", "3POINT", "CIRCLE_2POINT", "CIRCLE_3POINT"]:
            # --- FIX: Only show Diameter hint in Stage 1 ---
            if state["stage"] == 1:
                if state["tool_mode"] != "3POINT":
                    lines.append(("D: Set Diameter", None))
                lines.append(("Alt: Bypass Axis Snap", None))
        elif state.get("tool_mode") == "ELLIPSE_ENDPOINTS" and state["stage"] == 1:
            lines.append(("D: Set Diameter", None))
        elif state.get("tool_mode") == "LINE_POLY":            lines.append(("L: Set Length", None)) # --- NEW: Line Length Hint ---
        elif state.get("tool_mode") == "ELLIPSE_FOCI":
            lines.append(("F: Set Foci", None))
            
            # --- KEEP FOCI TOGGLE ---
            keep_state = "ON" if state.get("keep_foci") else "OFF"
            lines.append((f"K: Keep Foci ({keep_state})", None))
        elif state.get("tool_mode") == "ELLIPSE_RADIUS":
            lines.append(("D: Set Diameter", None))
            if state["stage"] == 2:
                lines.append(("R: Set Radius", None))
        elif state.get("tool_mode") == "ELLIPSE_ENDPOINTS":
            lines.append(("D: Set Diameter", None))
            if state["stage"] == 2:
                lines.append(("R: Set Radius", None))
        
        elif state.get("tool_mode") == "POLYGON_CENTER_CORNER":
            lines.append(("R: Set Radius", None))
            lines.append(("Segments:", None))
            lines.append(("Alt: Bypass Axis Snap", None))
        elif state.get("tool_mode") == "POLYGON_CENTER_TANGENT":
            lines.append(("A: Set Apothem", None))
            lines.append(("Segments:", None))
            lines.append(("Alt: Bypass Axis Snap", None))
        elif state.get("tool_mode") in ["POLYGON_CORNER_CORNER", "POLYGON_EDGE"]:
            lines.append(("L: Edge Length", None))
            lines.append(("Segments:", None))
            lines.append(("Alt: Bypass Axis Snap", None))
        elif state.get("tool_mode") == "CURVE_INTERPOLATE":
            lines.append(("Backspace: Remove Point", None))
            lines.append(("Wheel: Segments", None))
            lines.append(("Segments:", None))
        elif state.get("tool_mode") == "CURVE_FREEHAND":
            lines.append(("Click & Drag to Draw", None))
            lines.append(("Wheel: Segments", None))
            lines.append(("Segments:", None))
        elif state.get("tool_mode") != "ELLIPSE_CORNERS":
            lines.append(("R: Set Radius", None))
    
    if state["stage"] == 2:
        if state.get("tool_mode") == "2POINT":
            lines.append(("H: Set Sagitta Height", None))
            # === NEW: Stage 2 Alt Hint (Bypass 180 Snap) ===
            lines.append(("Alt: Bypass 180\u00B0 Snap", None))
        elif tool_mode not in ["3POINT", "CIRCLE_3POINT", "CIRCLE_TAN_TAN_TAN", "LINE_POLY", "ELLIPSE_FOCI", "ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS"]:
            lines.append(("A: Set Angle", None))
            
        lines.append(("S: Set Segments", None))
        lines.append(("Scroll: +/- Segs", None))

    ctx = bpy.context
    width = ctx.region.width
    height = ctx.region.height
    
    off_x = state.get("hotkeys_offset_x", 1000)
    off_y = state.get("hotkeys_offset_y", 20)
    
    px = off_x
    current_y = height - off_y
    
    for text, col in lines:
        # Check for Spacer
        if text is None:
            current_y -= 10 # Add 10px vertical gap
            continue
            
        h = draw_ui_box_generic(px, current_y, text, bg_color_override=col)
        current_y -= (h + 2)

# --- BOTTOM SNAPPING BAR ---
def draw_bottom_bar():
    ctx = bpy.context
    width = ctx.region.width
    
    # FIXED: Increased Margin to avoid Header conflict
    margin_bottom = 60 
    button_spacing = 8
    
    buttons = [
        ("F1: Vert", state.get("snap_verts", False), "snap_verts"),
        ("F2: Edge", state.get("snap_edges", False), "snap_edges"),
        ("F3: Edge Center", state.get("snap_edge_center", False), "snap_edge_center"),
        ("F4: Face Center", state.get("snap_face_center", False), "snap_face_center"),
        ("C: Compass", state.get("use_angle_snap", True), "toggle_angle"),
        ("W: Weld", state.get("auto_weld", True), "weld_btn"),
    ]
    
    # --- FIXED: Only show Weld for Tan Tan tool ---
    if state.get("tool_mode") == "LINE_TAN_TAN":
        buttons = [("W: Weld", state.get("auto_weld", True), "weld_btn")]
    
    # === NEW: "Next Solution" Button ===
    if state.get("choosing_solution"):
         # Pink color to indicate action
         buttons.append(("Tab: Next Solution", True, "next_sol"))
    
    font_id = 0
    total_w = 0
    pad_x = 10
    
    # Calculate Total Width for Screen Centering
    for label, _, _ in buttons:
        _, _, w_k, w_l, _ = get_mixed_text_metrics(font_id, label)
        total_w += (w_k + w_l) + (pad_x * 2) + button_spacing
    total_w -= button_spacing
    
    start_x = (width - total_w) / 2
    curr_x = start_x
    curr_y = margin_bottom
    
    # --- SNAP LABEL ---
    label_size = 16 
    blf.size(font_id, label_size)
    snap_label = "Snap"
    snap_w = blf.dimensions(font_id, snap_label)[0]
    
    snap_x = start_x + (total_w / 2) - (snap_w / 2)
    
    blf.color(font_id, *style["font_color"])
    blf.position(font_id, snap_x, margin_bottom + 38, 0)
    blf.draw(font_id, snap_label)
    
    # Draw Buttons
    for label, active, hit_id in buttons:
        w = draw_ui_button(curr_x, curr_y, label, active, hit_id)
        curr_x += w + button_spacing

def get_display_str(label, raw_str, is_active):
    if not is_active: return f"{label} {raw_str}"
    idx = state["cursor_index"]
    idx = max(0, min(idx, len(raw_str)))
    s_with_cursor = raw_str[:idx] + "|" + raw_str[idx:]
    return f"{label} {s_with_cursor}"

def draw_hud_2d():
    if not state["active"]: return
    try:
        # --- NEW 2D SNAP MARKER (CUSTOMIZABLE) ---
        if state.get("snap_point") and state.get("tool_mode") != "LINE_TAN_TAN":
            ctx = bpy.context
            reg, rv3d = ctx.region, ctx.region_data
            
            # --- DEFAULT FALLBACKS ---
            snap_type = 'X'
            snap_sz = 6
            snap_col = (1.0, 0.8, 0.0, 1.0)
            
            pkg = __package__ if __package__ else "radCAD"
            if '.' in pkg: pkg = pkg.split('.')[0] 
            
            try:
                prefs = bpy.context.preferences.addons[pkg].preferences
                
                # --- CHECK TOOL MODE FOR CORRECT PREFS ---
                mode = state.get("tool_mode", "1POINT")
                
                if mode == "2POINT":
                    snap_sz = getattr(prefs, "snap_marker_size_2pt", 6)
                    snap_col = getattr(prefs, "snap_marker_color_2pt", (1.0, 0.8, 0.0, 1.0))
                    snap_type = getattr(prefs, "snap_marker_type_2pt", 'X')
                elif mode == "3POINT":
                    snap_sz = getattr(prefs, "snap_marker_size_3pt", 6)
                    snap_col = getattr(prefs, "snap_marker_color_3pt", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X' # Hardcoded like 2pt
                elif mode == "CIRCLE_2POINT":
                    snap_sz = getattr(prefs, "snap_marker_size_c2pt", 6)
                    snap_col = getattr(prefs, "snap_marker_color_c2pt", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                elif mode == "CIRCLE_3POINT":
                    snap_sz = getattr(prefs, "snap_marker_size_c3pt", 6)
                    snap_col = getattr(prefs, "snap_marker_color_c3pt", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                elif mode == "ELLIPSE_RADIUS":
                    snap_sz = getattr(prefs, "snap_marker_size", 6)
                    snap_col = getattr(prefs, "snap_marker_color", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                elif mode == "ELLIPSE_ENDPOINTS":
                    snap_sz = getattr(prefs, "snap_marker_size", 6)
                    snap_col = getattr(prefs, "snap_marker_color", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                elif mode == "ELLIPSE_CORNERS":
                    snap_sz = getattr(prefs, "snap_marker_size", 6)
                    snap_col = getattr(prefs, "snap_marker_color", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                elif mode == "ELLIPSE_FOCI":
                    snap_sz = getattr(prefs, "snap_marker_size", 6)
                    snap_col = getattr(prefs, "snap_marker_color", (1.0, 0.8, 0.0, 1.0))
                    snap_type = 'X'
                else:
                    snap_sz = getattr(prefs, "snap_marker_size", 6)
                    snap_col = getattr(prefs, "snap_marker_color", (1.0, 0.8, 0.0, 1.0))
                    snap_type = getattr(prefs, "snap_marker_type", 'X')
                    
            except Exception:
                pass

            p2d = location_3d_to_region_2d(reg, rv3d, state["snap_point"])
            
            if p2d:
                cx, cy = p2d
                sh = get_2d_shader()
                gpu.state.blend_set("ALPHA")
                sh.bind()
                sh.uniform_float("color", snap_col)
                gpu.state.line_width_set(2.0)
                
                # --- DRAW BASED ON TYPE ---
                if snap_type == 'CIRCLE':
                    circle_pts = []
                    steps = 16
                    for i in range(steps):
                        a = (i / steps) * 2.0 * math.pi
                        circle_pts.append((cx + snap_sz * math.cos(a), cy + snap_sz * math.sin(a)))
                    batch_for_shader(sh, 'LINE_LOOP', {"pos": circle_pts}).draw(sh)

                elif snap_type == 'DOT':
                    dot_pts = [(cx, cy)]
                    steps = 12
                    for i in range(steps + 1):
                        a = (i / steps) * 2.0 * math.pi
                        dot_pts.append((cx + snap_sz * math.cos(a), cy + snap_sz * math.sin(a)))
                    batch_for_shader(sh, 'TRI_FAN', {"pos": dot_pts}).draw(sh)
                    
                else: # Default 'X'
                    x_coords = [
                        (cx - snap_sz, cy - snap_sz), (cx + snap_sz, cy + snap_sz),
                        (cx - snap_sz, cy + snap_sz), (cx + snap_sz, cy - snap_sz)
                    ]
                    batch_for_shader(sh, 'LINES', {"pos": x_coords}).draw(sh)

                gpu.state.blend_set("NONE")

        # 1. Preview Points (Arc Fill or Phantom Circles)
        if state.get("preview_pts"):
            ctx = bpy.context
            reg, rv3d = ctx.region, ctx.region_data
            sh = get_2d_shader()
            gpu.state.blend_set("ALPHA")
            
            # --- Standard Arc Fill (TRI_FAN) ---
            if state.get("tool_mode") not in ("POINT_BY_ARCS", "POINT_CENTER"):
                base_pt = style.get("point_px", 5)
                final_pt = max(1, base_pt - 1) if bpy.app.version >= (5, 0, 0) else base_pt
                r = final_pt / 2.0
                col = (0.0, 0.0, 0.0, 1.0)
                steps = 16
                for p in state["preview_pts"]:
                    p2d = location_3d_to_region_2d(reg, rv3d, p)
                    if not p2d: continue
                    cx, cy = p2d
                    verts = [(cx, cy)]
                    for i in range(steps + 1):
                        a = (i / steps) * 2.0 * math.pi
                        verts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
                    batch = batch_for_shader(sh, "TRI_FAN", {"pos": verts})
                    sh.bind()
                    sh.uniform_float("color", col)
                    batch.draw(sh)
            
            gpu.state.blend_set("NONE")
        
        # 2. Draw Menus
        draw_hotkeys_panel()
        if state.get("tool_mode") != "CURVE_FREEHAND":
            draw_bottom_bar()
        
        # 3. Measurements
        if state["show_measure"]:
            ctx = bpy.context
            reg, rv3d = ctx.region, ctx.region_data
            
            px, py = 0, 0
            if state["input_mode"] is not None and state["input_screen_pos"]:
                px, py = state["input_screen_pos"]
            elif state.get("current") is not None:
                # Prefer anchoring to the mouse (current) for stability
                curr_2d = location_3d_to_region_2d(reg, rv3d, state["current"])
                if curr_2d:
                    px = curr_2d.x + state.get("overlay_offset_x", 75)
                    py = curr_2d.y + state.get("overlay_offset_y", 0)
                else:
                    # Fallback to pivot if mouse projection fails
                    pivot_2d = location_3d_to_region_2d(reg, rv3d, state["pivot"]) if state["pivot"] is not None else None
                    if pivot_2d:
                        px = pivot_2d.x + state.get("overlay_offset_x", 75)
                        py = pivot_2d.y + state.get("overlay_offset_y", 0)
                    else: return
            elif state["pivot"] is not None:
                pivot_2d = location_3d_to_region_2d(reg, rv3d, state["pivot"])
                if pivot_2d:
                    px = pivot_2d.x + state.get("overlay_offset_x", 75)
                    py = pivot_2d.y + state.get("overlay_offset_y", 0)
            else:
                 return

            current_y = py
            tool_mode = state.get("tool_mode", "1POINT")

            # --- STAGE 1 RECAP (Always show above input in Stage 2) ---
            if state["stage"] == 2:
                if tool_mode == "ELLIPSE_FOCI":
                    f1, f2 = state.get("f1") or state["pivot"], state.get("f2") or state["current"]
                    dist_f = (f2 - f1).length if (f1 and f2) else 0.0
                    r_txt_f = "F: " + format_length(dist_f)
                    h_f = draw_ui_box_generic(px, current_y, r_txt_f)
                    current_y -= (h_f + 4)
                elif tool_mode == "ELLIPSE_ENDPOINTS":
                    p1, p2 = state.get("p1") or state["pivot"], state.get("p2") or state["current"]
                    dist_d = (p2 - p1).length if (p1 and p2) else 0.0
                    r_txt_d = "D: " + format_length(dist_d)
                    h_d = draw_ui_box_generic(px, current_y, r_txt_d)
                    current_y -= (h_d + 4)
                elif tool_mode == "ELLIPSE_RADIUS":
                    dist_d = state.get("rx", 0.0) * 2.0
                    r_txt_d = "D: " + format_length(dist_d)
                    h_d = draw_ui_box_generic(px, current_y, r_txt_d)
                    current_y -= (h_d + 4)

            # --- D/R/L/F LABEL LOGIC ---
            if state["input_mode"] == 'RADIUS': 
                if tool_mode == "ELLIPSE_CORNERS":
                    pass
                else:
                    label = "R:"
                    if tool_mode in ["2POINT", "3POINT", "CIRCLE_2POINT", "CIRCLE_3POINT"]:
                        if state["stage"] == 1: label = "" if tool_mode == "3POINT" else "D:"
                        elif tool_mode == "2POINT": label = "S:" # Sagitta for 2pt Stage 2
                        else: label = "R:" # Default for 3pt Stage 2 or CIRCLE_2POINT Stage 2 is Radius
                    elif tool_mode == "ELLIPSE_ENDPOINTS" and state["stage"] == 1:
                        label = "D:"
                    elif tool_mode == "LINE_POLY": label = "" # --- REMOVED 'L' for Line Tool
                    elif tool_mode == "ELLIPSE_FOCI": 
                        label = "F:" if state["stage"] == 1 else ""
                    elif tool_mode == "ELLIPSE_RADIUS":
                        label = "D:" if state["stage"] == 1 else "R:"
                    elif tool_mode == "POLYGON_CENTER_CORNER":
                        label = "R:"
                    elif tool_mode == "POLYGON_CENTER_TANGENT":
                        label = "A:"
                    elif tool_mode in ["POLYGON_CORNER_CORNER", "POLYGON_EDGE"]:
                        label = "L:"
                    
                    if state["input_mode"] == 'SEGMENTS':
                        label = "Segments:" if tool_mode in ["POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE"] else "S:"
                    
                    r_txt = get_display_str(label, state['input_string'], True)
                    h1 = draw_ui_box_generic(px, current_y, r_txt, active=True)
                    current_y -= (h1 + 4)
            else:
                # Tan-Tan-Tan Radius Display
                if state.get("choosing_solution") and state.get("tan_solutions"):
                    idx = state["solution_index"] % len(state["tan_solutions"])
                    r_val = state["tan_solutions"][idx][1]
                    r_txt = "R: " + format_length(r_val)
                    h1 = draw_ui_box_generic(px, current_y, r_txt)
                    current_y -= (h1 + 4)
                elif tool_mode == "ELLIPSE_FOCI":
                    # For Stage 1 we still need the Foci label
                    if state["stage"] == 1:
                        f1, f2 = state.get("f1") or state["pivot"], state.get("f2") or state["current"]
                        dist_f = (f2 - f1).length if (f1 and f2) else 0.0
                        r_txt_f = "F: " + format_length(dist_f)
                        h1 = draw_ui_box_generic(px, current_y, r_txt_f)
                        current_y -= (h1 + 4)
                elif tool_mode == "ELLIPSE_ENDPOINTS":
                    if state["stage"] == 1:
                        # For Stage 1 we still need the Diameter label
                        p1, p2 = state.get("p1") or state["pivot"], state.get("p2") or state["current"]
                        dist_d = (p2 - p1).length if (p1 and p2) else 0.0
                        r_txt_d = "D: " + format_length(dist_d)
                        h1 = draw_ui_box_generic(px, current_y, r_txt_d)
                        current_y -= (h1 + 4)
                    elif state["stage"] == 2:
                        label_r = "R: "
                        r_val = state.get("ry", 0.0)
                        r_txt_r = label_r + format_length(r_val)
                        h2 = draw_ui_box_generic(px, current_y, r_txt_r)
                        current_y -= (h2 + 4)
                elif tool_mode == "ELLIPSE_RADIUS":
                    if state["stage"] == 1:
                        # Stage 1: Only show Diameter (R hasn't happened yet)
                        dist_d = state.get("rx", 0.0) * 2.0
                        r_txt_d = "D: " + format_length(dist_d)
                        h_d = draw_ui_box_generic(px, current_y, r_txt_d)
                        current_y -= (h_d + 4)
                    elif state["stage"] == 2 and not state.get("input_mode"):
                        # Stage 2: Active Minor Radius (R)
                        # (D is shown in the recap section above)
                        r_val_y = state.get("ry", 0.0)
                        r_txt_r = "R: " + format_length(r_val_y)
                        h2 = draw_ui_box_generic(px, current_y, r_txt_r)
                        current_y -= (h2 + 4)
                elif tool_mode in ["POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE"]:
                    # Label based on tool
                    label = "R: "
                    if tool_mode == "POLYGON_CENTER_TANGENT": label = "A: "
                    elif tool_mode in ["POLYGON_CORNER_CORNER", "POLYGON_EDGE"]: label = "L: "
                    
                    # Size (Radius/Apothem/Length)
                    r_val = state.get("radius", 0.0)
                    r_txt = label + format_length(r_val)
                    h1 = draw_ui_box_generic(px, current_y, r_txt)
                    current_y -= (h1 + 4)
                    
                    # Segments
                    s_val = state.get("segments", 6)
                    s_txt = f"Segments: {s_val}"
                    h2 = draw_ui_box_generic(px, current_y, s_txt)
                    current_y -= (h2 + 4)
                elif tool_mode in ["CURVE_INTERPOLATE", "CURVE_FREEHAND"]:
                    # No radius label, but show Segments
                    is_input_s = (state["input_mode"] == 'SEGMENTS')
                    if is_input_s: s_txt = get_display_str("Segments:", state['input_string'], True)
                    else: s_txt = f"Segments: {state['segments']}"
                    h_s = draw_ui_box_generic(px, current_y, s_txt, active=is_input_s)
                    current_y -= (h_s + 4)
                    if tool_mode == "CURVE_FREEHAND":
                        is_input_m = (state["input_mode"] == 'MIN_DIST')
                        if is_input_m: m_txt = get_display_str("Min Dist:", state['input_string'], True)
                        else: m_txt = f"Min Dist: {int(state.get('min_dist', 0.05) * 100)}"
                        draw_ui_box_generic(px, current_y, m_txt, active=is_input_m)
                elif tool_mode == "CIRCLE_TAN_TAN":
                    # Live radius display for Tan-Tan
                    target = state.get("input_target", "RADIUS")
                    r_val = state.get("radius", 0.0)
                    if target == 'DIAMETER':
                        r_txt = "D: " + format_length(r_val * 2.0)
                    else:
                        r_txt = "R: " + format_length(r_val)
                    h1 = draw_ui_box_generic(px, current_y, r_txt)
                    current_y -= (h1 + 4)
                elif tool_mode == "ELLIPSE_CORNERS":
                    pass
                elif tool_mode in ["2POINT", "3POINT", "CIRCLE_2POINT", "CIRCLE_3POINT"]:
                    if state["stage"] == 1:
                         label = "" if tool_mode == "3POINT" else "D: "
                         # Chord Length / Diameter
                         d_val = (state["current"] - state["pivot"]).length if (state["current"] and state["pivot"]) else 0.0
                         r_txt = label + format_length(d_val)
                         h1 = draw_ui_box_generic(px, current_y, r_txt)
                         current_y -= (h1 + 4)
                    elif state["stage"] == 2 and tool_mode == "2POINT":
                         label = "H: "
                         # Height
                         h_val = (state["start"] - state["pivot"]).length if (state["start"] and state["pivot"]) else 0.0
                         r_txt = label + format_length(h_val)
                         h1 = draw_ui_box_generic(px, current_y, r_txt)
                         current_y -= (h1 + 4)
                    elif state["stage"] == 2 and tool_mode in ["3POINT", "CIRCLE_2POINT", "CIRCLE_3POINT"]:
                         # USER REQUEST: Don't show Diameter in Stage 2
                         # Radius is shown below in the general 'else' if stage == 2
                         pass
                    else:
                         r_txt = "D: 0"
                         h1 = draw_ui_box_generic(px, current_y, r_txt)
                         current_y -= (h1 + 4)
                elif tool_mode == "LINE_TANGENT_FROM_CURVE":
                    pass
                else:
                    label = "R: "
                    if tool_mode == "LINE_POLY": label = "" # --- REMOVED 'L' for Line Tool
                    
                    r_val = state["radius"] if state["stage"] == 2 else ((state["current"] - state["pivot"]).length if (state["current"] and state["pivot"]) else 0.0)
                    r_txt = label + format_length(r_val)
                    h1 = draw_ui_box_generic(px, current_y, r_txt)
                    current_y -= (h1 + 4)
            
            if state["stage"] == 2:
                # --- HIDE ANGLE IF 2POINT, 3POINT, LINE_POLY, ELLIPSE_FOCI OR ELLIPSE_RADIUS ---
                if tool_mode not in ["2POINT", "3POINT", "CIRCLE_3POINT", "CIRCLE_TAN_TAN_TAN", "LINE_POLY", "ELLIPSE_FOCI", "ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS", "CURVE_FREEHAND"]:
                    is_input_a = (state["input_mode"] == 'ANGLE')
                    if is_input_a: a_txt = get_display_str("\u2220", state['input_string'], True)
                    else:
                        if state.get("use_radians"): a_txt = f"\u2220 {-state['accum_angle']:.3f} rad"
                        else: a_txt = f"\u2220 {-math.degrees(state['accum_angle']):.1f}\u00B0"
                    h2 = draw_ui_box_generic(px, current_y, a_txt, active=is_input_a)
                    current_y -= (h2 + 4)
                
                # --- HIDE SEGMENTS FOR LINE_POLY AND CURVE_FREEHAND ---
                if tool_mode not in ["LINE_POLY", "CURVE_FREEHAND"]:
                    is_input_s = (state["input_mode"] == 'SEGMENTS')
                    if is_input_s: s_txt = get_display_str("Segments:", state['input_string'], True)
                    else: s_txt = f"Segments: {state['segments']}"
                    draw_ui_box_generic(px, current_y, s_txt, active=is_input_s)
    except Exception as e:
        print(f"HUD DRAW ERROR: {e}")
