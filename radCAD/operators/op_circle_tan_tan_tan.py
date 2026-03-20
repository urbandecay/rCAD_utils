import bpy
import math
import random
from mathutils import Vector
from .circle_tools import CatmullRomSpline, get_selected_edge_chains
from ..orientation_utils import orthonormal_basis_from_normal
from ..modal_state import state
from ..modal_core import begin_modal, modal_arc_common

# =========================================================================
# --- HELPER: LINEAR ALGEBRA ---
# =========================================================================

def solve_linear_system(A, B):
    n = len(B)
    for i in range(n):
        max_row = i
        for k in range(i + 1, n):
            if abs(A[k][i]) > abs(A[max_row][i]):
                max_row = k
        A[i], A[max_row] = A[max_row], A[i]
        B[i], B[max_row] = B[max_row], B[i]
        
        if abs(A[i][i]) < 1e-9: return None
        
        for k in range(i + 1, n):
            f = A[k][i] / A[i][i]
            B[k] -= f * B[i]
            for j in range(i, n):
                A[k][j] -= f * A[i][j]
                
    X = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = sum(A[i][j] * X[j] for j in range(i + 1, n))
        X[i] = (B[i] - s) / A[i][i]
    return X

# =========================================================================
# --- HELPER: SPLINE MATH ---
# =========================================================================

def eval_spline_derivatives(points, t, is_closed):
    N = len(points)
    if is_closed:
        t_mod = ((t % N) + N) % N
        i0 = int(t_mod)
        local_t = t_mod - i0
        p0 = points[(i0 - 1 + N) % N]
        p1 = points[i0]
        p2 = points[(i0 + 1) % N]
        p3 = points[(i0 + 2) % N]
    else:
        max_t = N - 1.0001
        t_clamped = max(0.0, min(float(max_t), t))
        i0 = int(t_clamped)
        local_t = t_clamped - i0
        
        p0 = points[max(0, i0-1)]
        p1 = points[i0]
        p2 = points[min(N-1, i0+1)]
        p3 = points[min(N-1, i0+2)]
        
        if i0 == 0: p0 = p1 + (p1 - p2)
        if i0 >= N - 2: p3 = p2 + (p2 - p1)

    tt = local_t * local_t
    ttt = tt * local_t

    q0 = -0.5*ttt + tt - 0.5*local_t
    q1 =  1.5*ttt - 2.5*tt + 1.0
    q2 = -1.5*ttt + 2.0*tt + 0.5*local_t
    q3 =  0.5*ttt - 0.5*tt
    
    dq0 = -1.5*tt + 2.0*local_t - 0.5
    dq1 =  4.5*tt - 5.0*local_t
    dq2 = -4.5*tt + 4.0*local_t + 0.5
    dq3 =  1.5*tt - 1.0*local_t
    
    ddq0 = -3.0*local_t + 2.0
    ddq1 =  9.0*local_t - 5.0
    ddq2 = -9.0*local_t + 4.0
    ddq3 =  3.0*local_t - 1.0
    
    x = q0*p0.x + q1*p1.x + q2*p2.x + q3*p3.x
    y = q0*p0.y + q1*p1.y + q2*p2.y + q3*p3.y
    
    xp = dq0*p0.x + dq1*p1.x + dq2*p2.x + dq3*p3.x
    yp = dq0*p0.y + dq1*p1.y + dq2*p2.y + dq3*p3.y
    
    xpp = ddq0*p0.x + ddq1*p1.x + ddq2*p2.x + ddq3*p3.x
    ypp = ddq0*p0.y + ddq1*p1.y + ddq2*p2.y + ddq3*p3.y
    
    return (x, y), (xp, yp), (xpp, ypp)

# =========================================================================
# --- THE TOOL ---
# =========================================================================

class CircleTool_TanTanTan:
    def __init__(self, manager):
        self.manager = manager
        self.stage = 0
        self.pivot = None
        self.current = None  # <--- FIXED: Added back for modal_core
        self.segments = 64
        self.preview_pts = []
        self.splines = [] 
        self.spline_points_2d = [[], [], []] 
        self.spline_closed = [False, False, False]
        self.spline_max_t = [0.0, 0.0, 0.0]
        self.spline_centers = [Vector((0,0,0)), Vector((0,0,0)), Vector((0,0,0))]
        
        self.perm_index = 0
        self.permutations = [
            (1, 1, 1), (-1, -1, -1), (1, 1, -1), (1, -1, 1), 
            (-1, 1, 1), (-1, -1, 1), (-1, 1, -1), (1, -1, -1)
        ]
        
        self.scene_scale = 1.0
        self.Xp, self.Yp, self.Zp = Vector((1,0,0)), Vector((0,1,0)), Vector((0,0,1))
        
        state.update({
            "stage": 0,
            "preview_pts": [],
            "visual_pts": [], # Explicitly reset to kill old ghost data
            "catmull_spline_previews": [], # Reset
            "tan_solutions": [],
            "tan_solution_active": False,
            "viz_tangent_line": [],
            "viz_diameter_line": [],
        })
        
        obj = bpy.context.edit_object
        chains = get_selected_edge_chains(obj)
        
        if len(chains) == 3:
            self.splines = chains 
            
            # --- POPULATE CATMULL OVERLAYS ---
            catmull_previews = []
            for pts_raw, is_closed in chains:
                spline = CatmullRomSpline(pts_raw, is_closed=is_closed)
                if spline.segments:
                    # Sample the spline for smooth visualization
                    curve_pts = []
                    for seg in spline.segments:
                        # 4 samples per segment
                        curve_pts.extend([seg.eval(seg.t_start + t*seg.dt) for t in [0.0, 0.25, 0.5, 0.75]])
                    # Add very last point
                    curve_pts.append(spline.segments[-1].eval(spline.segments[-1].t_end))
                    catmull_previews.append(curve_pts)
            state["catmull_spline_previews"] = catmull_previews
            
            v1 = chains[0][0][1] - chains[0][0][0]
            v2 = chains[0][0][-1] - chains[0][0][0]
            cross = v1.cross(v2)
            if cross.length > 1e-4:
                self.Zp = cross.normalized()
                basis = orthonormal_basis_from_normal(self.Zp)
                self.Xp, self.Yp = basis[0], basis[1]
                self.pivot = chains[0][0][0]
            if not self.pivot: self.pivot = chains[0][0][0] 
            
            min_x, max_x = 1e9, -1e9
            
            for i, c in enumerate(chains):
                pts_raw, is_closed = c
                self.spline_closed[i] = is_closed
                
                local_pts = []
                center_accum = Vector((0,0,0))
                for p in pts_raw:
                    d = p - self.pivot
                    pt_2d = Vector((d.dot(self.Xp), d.dot(self.Yp), 0))
                    local_pts.append(pt_2d)
                    center_accum += pt_2d
                    min_x = min(min_x, pt_2d.x); max_x = max(max_x, pt_2d.x)
                
                self.spline_points_2d[i] = local_pts
                self.spline_centers[i] = center_accum / len(pts_raw)
                self.spline_max_t[i] = float(len(local_pts)) if is_closed else len(local_pts) - 1.0

            self.scene_scale = max(1.0, abs(max_x - min_x))

    def check_manifold(self, cx, cy, r, signs):
        bad_idx = -1
        fix_t = -1
        max_penetration = 0.0
        tol = self.scene_scale * 0.02 
        
        for i in range(3):
            if not self.spline_closed[i]: continue 
            
            pts = self.spline_points_2d[i]
            max_t = self.spline_max_t[i]
            steps = 60 
            
            for k in range(steps + 1):
                t = (k / steps) * max_t
                (px, py), _, _ = eval_spline_derivatives(pts, t, True)
                d = math.hypot(px - cx, py - cy)
                
                if signs[i] == 1: 
                    penetration = abs(r) - d
                    if penetration > tol:
                        if penetration > max_penetration:
                            max_penetration = penetration
                            bad_idx = i
                            fix_t = t
                            
                elif signs[i] == -1: 
                    pop_out = d - abs(r)
                    if pop_out > tol:
                        if pop_out > max_penetration:
                            max_penetration = pop_out
                            bad_idx = i
                            fix_t = t
                            
        return bad_idx, fix_t

    def solve_current(self):
        state["tan_solutions"] = [] 
        s = self.permutations[self.perm_index]
        
        base_cx = sum(c.x for c in self.spline_centers) / 3.0
        base_cy = sum(c.y for c in self.spline_centers) / 3.0
        cx, cy = base_cx, base_cy
        
        total_sign = sum(s)
        curr_r = (self.scene_scale * 0.25) if total_sign > -1 else (self.scene_scale * 5.0)
        
        curr_t = []
        for i in range(3):
            pts = self.spline_points_2d[i]
            best_t = 0.0
            best_dist_sq = float('inf') if s[i] == 1 else -float('inf')
            steps = 60
            max_t = self.spline_max_t[i]
            for k in range(steps + 1): 
                t = (k/steps) * max_t
                (px, py), _, _ = eval_spline_derivatives(pts, t, self.spline_closed[i])
                d2 = (px - cx)**2 + (py - cy)**2
                if s[i] == 1:
                    if d2 < best_dist_sq: best_dist_sq = d2; best_t = t
                else:
                    if d2 > best_dist_sq: best_dist_sq = d2; best_t = t
            curr_t.append(best_t)

        converged = False
        stuck_frames = 0
        
        for frame in range(100): 
            step_res = self.solve_step(cx, cy, curr_r, curr_t, 6)
            if not step_res: break 
            cx, cy, curr_r, curr_t, max_err = step_res
            
            if max_err < (self.scene_scale * 0.1) or converged:
                bad_idx, fix_t = self.check_manifold(cx, cy, curr_r, s)
                if bad_idx != -1:
                    curr_t[bad_idx] = fix_t
                    nudge = self.scene_scale * 0.1
                    cx += (random.random() - 0.5) * nudge
                    cy += (random.random() - 0.5) * nudge
                    stuck_frames = 0 
                elif max_err < 1e-3:  
                    converged = True
                    stuck_frames = max(0, stuck_frames - 1)
                    if stuck_frames == 0: break
                else:
                    stuck_frames += 1
            else:
                stuck_frames += 1
            if stuck_frames > 20: break 
            
        if converged:
            pt_3d = self.pivot + self.Xp*cx + self.Yp*cy
            state["tan_solutions"] = [(pt_3d, curr_r)]
            
            # Calculate the 3 tangency points in 3D
            tan_pts_3d = []
            for i in range(3):
                pts_2d = self.spline_points_2d[i]
                (gx, gy), _, _ = eval_spline_derivatives(pts_2d, curr_t[i], self.spline_closed[i])
                wp = self.pivot + self.Xp*gx + self.Yp*gy
                tan_pts_3d.append(wp)
            state["tan_points"] = tan_pts_3d
            
            self.current = pt_3d # Update position for UI

        self.refresh_preview()

    def solve_step(self, sx, sy, sr, st, iters):
        curr_x, curr_y, curr_r, curr_t = sx, sy, sr, list(st)
        last_max_err = 0.0
        
        for _ in range(iters):
            J = [] 
            F = [] 
            max_err = 0.0
            
            for i in range(3):
                pts = self.spline_points_2d[i]
                closed = self.spline_closed[i]
                max_t = self.spline_max_t[i]
                
                if not closed: curr_t[i] = max(0, min(max_t, curr_t[i]))
                
                (gx, gy), (gxp, gyp), (gxpp, gypp) = eval_spline_derivatives(pts, curr_t[i], closed)
                
                dx = curr_x - gx
                dy = curr_y - gy
                dist_sq = dx*dx + dy*dy
                
                eq_dist = dist_sq - curr_r**2
                eq_tan = dx*gxp + dy*gyp 
                
                max_err = max(max_err, abs(eq_dist), abs(eq_tan))
                
                row_d = [2*dx, 2*dy, -2*curr_r, 0, 0, 0]
                row_d[3+i] = -2 * (dx*gxp + dy*gyp) 
                
                row_t = [gxp, gyp, 0, 0, 0, 0]
                row_t[3+i] = (-gxp*gxp + dx*gxpp) + (-gyp*gyp + dy*gypp) 
                
                J.append(row_d)
                J.append(row_t)
                F.append(eq_dist)
                F.append(eq_tan)
            
            last_max_err = max_err
            B = [-v for v in F]
            delta = solve_linear_system(J, B)
            if not delta: return None 
            
            curr_x += delta[0] * 0.8
            curr_y += delta[1] * 0.8
            curr_r += delta[2] * 0.8
            curr_t[0] += delta[3] * 0.8
            curr_t[1] += delta[4] * 0.8
            curr_t[2] += delta[5] * 0.8

        return curr_x, curr_y, abs(curr_r), curr_t, last_max_err

    def update(self, context, event, snap_pt, snap_normal):
        # Update segments from global state (mouse wheel support)
        self.segments = state.get("segments", 64)
        
        # Refresh if the segment count changed from the mouse wheel
        if len(self.preview_pts) != self.segments + 1:
            self.refresh_preview()

    def refresh_preview(self):
        state["visual_pts"] = []
        state["preview_pts"] = []
        
        if not state["tan_solutions"]: 
            state["tan_solution_active"] = False
            return
            
        c, r = state["tan_solutions"][0]
        
        # 1. Circle Preview (Always smooth/high-res for visuals)
        state["visual_pts"] = [c + self.Xp*math.cos(a)*r + self.Yp*math.sin(a)*r for a in [i*math.pi*2/128 for i in range(129)]]
        
        # 2. Geometry Preview (Matches segment count for commitment)
        self.preview_pts = [c + self.Xp*math.cos(a)*r + self.Yp*math.sin(a)*r for a in [i*math.pi*2/self.segments for i in range(self.segments + 1)]]
        state["preview_pts"] = self.preview_pts
        state["tan_solution_active"] = True

    def handle_click(self, context, event, snap_pt, snap_normal, button_id=None):
        return 'FINISHED'
    
    def handle_input(self, context, event):
        if event.type == 'TAB' and event.value == 'PRESS':
            self.perm_index = (self.perm_index + 1) % len(self.permutations)
            self.solve_current()
            return True
        return False

class VIEW3D_OT_circle_tan_tan_tan(bpy.types.Operator):
    bl_idname = "view3d.radcad_circle_tan_tan_tan"
    bl_label = "Circle Tan-Tan-Tan"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING'}
    def modal(self, context, event): return modal_arc_common(self, context, event)
    def invoke(self, context, event):
        state["tool_mode"] = "CIRCLE_TAN_TAN_TAN"
        ret = begin_modal(self, context, event)
        if ret == {'RUNNING_MODAL'}:
            tool = self.manager.active_tool
            if len(tool.splines) == 3: tool.solve_current()
        return ret