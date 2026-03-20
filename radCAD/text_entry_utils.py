# radCAD/text_entry_utils.py

import bpy
import math
from mathutils import Vector, geometry
from .modal_state import state
from .units_utils import format_length
from .plane_utils import world_to_plane

def apply_input_value(ctx):
    val_str = state["input_string"]
    if not val_str:
        return

    try:
        # Check for segments first (must be int)
        if state["input_mode"] == 'SEGMENTS':
            val_meters = 0 # Dummy for flow
        else:
            val_meters = bpy.utils.units.to_value(val_str, 'LENGTH', 'METERS')
    except ValueError:
        # Fallback if typing just numbers without units
        try:
            val_meters = float(val_str)
        except ValueError:
            return

    tool_mode = state.get("tool_mode", "1POINT")

    # --- RADIUS / DISTANCE INPUT ---
    if state["input_mode"] == 'RADIUS':
        # === 1-POINT LOGIC ===
        if tool_mode == "1POINT":
            state["radius"] = abs(val_meters)
            pv = state["pivot"]
            if pv:
                if state["stage"] == 2:
                    # Already drawing — update radius but keep the angle
                    Xp, Yp = state.get("Xp"), state.get("Yp")
                    if Xp and Yp:
                        a0 = state["a0"]
                        new_start = pv + (Xp * math.cos(a0) + Yp * math.sin(a0)) * state["radius"]
                        state["start"] = new_start
                        state["current"] = new_start
                        state["a1"] = state["a0"] + state["accum_angle"]
                else:
                    # Stage 1 — advance to stage 2
                    d = state["current"] - pv
                    if d.length > 1e-9: d.normalize()
                    else: d = Vector((1,0,0))

                    state["start"] = pv + (d * state["radius"])
                    state["current"] = state["start"]
                    rvec2 = world_to_plane(state["start"] - pv, state["Xp"], state["Yp"])
                    a0 = math.atan2(rvec2.y, rvec2.x)
                    state["a0"] = a0; state["a1"] = a0; state["a_prev_raw"] = a0
                    state["accum_angle"] = 0.0
                    state["stage"] += 1

        # === 2-POINT LOGIC ===
        elif tool_mode == "2POINT" or tool_mode == "CIRCLE_2POINT":
            target = state.get("input_target", "RADIUS")
            dist = abs(val_meters)
            if target == 'DIAMETER' or tool_mode == "CIRCLE_2POINT":
                # For CIRCLE_2POINT, we assume the input is diameter
                dist = abs(val_meters)
            
            if state["stage"] == 1:
                pv = state["pivot"]
                d = state["current"] - pv
                if d.length > 1e-9: d.normalize()
                else: d = Vector((1,0,0))
                
                # Update p2 based on parsed distance
                state["p2"] = pv + (d * dist)
                state["current"] = state["p2"]
                state["p1"] = pv
                state["midpoint"] = (state["p1"] + state["p2"]) * 0.5
                
                # For Arcs, move to stage 2. For Circles, finish if tool supports it (or wait for RET)
                if tool_mode == "2POINT":
                    state["stage"] = 2
                else:
                    # CIRCLE_2POINT: update radius for preview
                    state["radius"] = dist * 0.5
            
            elif state["stage"] == 2 and tool_mode == "2POINT":
                mid = state.get("midpoint")
                curr_peak = state.get("start") or state.get("current")
                if curr_peak and mid:
                    d = curr_peak - mid
                    if d.length > 1e-9: d.normalize()
                    else: d = state.get("Zp") or Vector((0,0,1))
                    state["start"] = mid + (d * abs(val_meters))

        # === 3-POINT LOGIC ===
        elif tool_mode == "CIRCLE_3POINT":
            # For 3-point circle, radius input is ambiguous unless we are in the final stage
            if state["stage"] == 2:
                # We can't easily "solve" for a 3rd point given a radius without more constraints,
                # but we can at least store the radius and let the tool update if it can.
                state["radius"] = abs(val_meters)

        # === ELLIPSE TOOLS LOGIC ===
        elif tool_mode == "ELLIPSE_RADIUS":
            target = state.get("input_target", "RADIUS")
            if target == 'DIAMETER':
                diam = abs(val_meters)
                state["rx"] = diam * 0.5
                center = state["pivot"]
                Xp = state.get("Xp") or Vector((1,0,0))
                state["current"] = center + Xp * (diam * 0.5)
                if state["stage"] == 1: state["stage"] = 2
            else: # RADIUS
                ry = abs(val_meters)
                state["ry"] = ry
                center = state["pivot"]
                Yp = state.get("Yp") or Vector((0,1,0))
                state["current"] = center + (Yp * ry)
                if state["stage"] == 1: state["stage"] = 2

        elif tool_mode == "ELLIPSE_ENDPOINTS":
            target = state.get("input_target", "RADIUS")
            if target == 'DIAMETER':
                p1 = state["pivot"]
                d_vec = state["current"] - p1
                if d_vec.length > 1e-9: d_vec.normalize()
                else: d_vec = state.get("Xp") or Vector((1,0,0))
                diam = abs(val_meters)
                p2 = p1 + d_vec * diam
                state["p1"], state["p2"] = p1, p2
                state["rx"] = diam * 0.5
                state["current"] = p2
                if state["stage"] == 1: state["stage"] = 2
            else: # RADIUS
                ry = abs(val_meters)
                state["ry"] = ry
                p1, p2 = state.get("p1"), state.get("p2")
                if p1 and p2:
                    center = (p1 + p2) * 0.5
                    Xp = (p2 - p1).normalized()
                    Zp = state.get("Zp") or Vector((0,0,1))
                    Yp = Zp.cross(Xp).normalized()
                    state["current"] = center + (Yp * ry)
                if state["stage"] == 1: state["stage"] = 2

        elif tool_mode == "ELLIPSE_FOCI":
            if state["stage"] == 1:
                pv = state["pivot"]
                maj = state.get("Xp") or Vector((1,0,0))
                state["f1"] = pv
                state["f2"] = pv + (maj * abs(val_meters))
                state["stage"] = 2

        elif tool_mode in ["POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE"]:
            # Generic Radius / Distance storage
            if state["input_mode"] == 'RADIUS':
                state["radius"] = abs(val_meters)

    # --- ANGLE INPUT (1-POINT ONLY) ---
    elif state["input_mode"] == 'ANGLE':
        try:
            val = float(val_str)
            accum = state.get("accum_angle", 0.0)
            direction = 1.0 if abs(accum) < 1e-6 else (-1.0 if accum < 0 else 1.0)
            rad = math.radians(val) * direction
            state["accum_angle"] = rad
            state["a1"] = state["a0"] + rad
            state["a_prev_raw"] = math.atan2(math.sin(state["a1"]), math.cos(state["a1"]))
        except ValueError:
            pass
            
    # --- SEGMENTS INPUT ---
    elif state["input_mode"] == 'SEGMENTS':
        try:
            val = int(val_str)
            state["segments"] = max(3, min(1000, val))
        except ValueError:
            pass

    # --- MIN DIST INPUT ---
    elif state["input_mode"] == 'MIN_DIST':
        try:
            val = abs(float(val_str)) / 100.0
            if val > 0:
                state["min_dist"] = val
        except ValueError:
            pass

    # Update Preview Points immediately
    state["skip_mouse_update"] = True 
    state["input_mode"] = None
    state["input_screen_pos"] = None


def handle_text_input(ctx, ev):
    if state["input_mode"] is None:
        return False

    curr_str = state["input_string"]
    idx = state["cursor_index"]

    if ev.type == 'RET' or ev.type == 'NUMPAD_ENTER':
        apply_input_value(ctx)
        ctx.area.tag_redraw()
        return True
    elif ev.type == 'ESC':
        state["input_mode"] = None
        state["input_screen_pos"] = None
        ctx.area.tag_redraw()
        return True
    elif ev.type == 'LEFT_ARROW' and ev.value == 'PRESS':
        state["cursor_index"] = max(0, idx - 1)
        ctx.area.tag_redraw()
        return True
    elif ev.type == 'RIGHT_ARROW' and ev.value == 'PRESS':
        state["cursor_index"] = min(len(curr_str), idx + 1)
        ctx.area.tag_redraw()
        return True
    elif ev.type == 'BACKSPACE' and ev.value == 'PRESS':
        if idx > 0:
            state["input_string"] = curr_str[:idx-1] + curr_str[idx:]
            state["cursor_index"] = idx - 1
        ctx.area.tag_redraw()
        return True
    elif ev.type == 'DEL' and ev.value == 'PRESS':
        if idx < len(curr_str):
            state["input_string"] = curr_str[:idx] + curr_str[idx+1:]
        ctx.area.tag_redraw()
        return True
    
    # Generic text characters
    if ev.unicode:
        state["input_string"] = curr_str[:idx] + ev.unicode + curr_str[idx:]
        state["cursor_index"] = idx + 1
        ctx.area.tag_redraw()
        return True

    return False

def get_display_str(label, typed, show_cursor):
    if not show_cursor:
        return f"{label} {typed}"
    
    idx = state["cursor_index"]
    return f"{label} {typed[:idx]}|{typed[idx:]}"
