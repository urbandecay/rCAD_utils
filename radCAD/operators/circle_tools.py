import math
import bpy
import bmesh
from mathutils import Vector, geometry
from bpy_extras import view3d_utils
from ..geometry_utils import arc_points_world, snap_angle_soft
from ..plane_utils import world_to_plane, plane_to_world
from ..inference_utils import get_axis_snapped_location
from .base_tool import SurfaceDrawTool 
from ..orientation_utils import orthonormal_basis_from_normal

# =========================================================================
# --- 1. SPLINE MATH ---
# =========================================================================

class CubicSegment:
    def __init__(self, p0, p1, p2, p3):
        t0 = 0.0
        t1 = self._get_t(t0, p0, p1)
        t2 = self._get_t(t1, p1, p2)
        t3 = self._get_t(t2, p2, p3)
        self.t_start = t1; self.t_end = t2; self.dt = t2 - t1
        if abs(self.dt) < 1e-6: self.d = p1; self.is_degenerate = True; return
        self.is_degenerate = False; self.p0, self.p1, self.p2, self.p3 = p0, p1, p2, p3; self.times = (t0, t1, t2, t3)
    def _get_t(self, t, p0, p1):
        dist_sq = (p1 - p0).length_squared
        return t + pow(dist_sq, 0.25)
    def eval(self, t_global):
        if self.is_degenerate: return self.d
        t0, t1, t2, t3 = self.times; t = t_global
        a1 = (t1-t)/(t1-t0)*self.p0 + (t-t0)/(t1-t0)*self.p1; a2 = (t2-t)/(t2-t1)*self.p1 + (t-t1)/(t2-t1)*self.p2; a3 = (t3-t)/(t3-t2)*self.p2 + (t-t2)/(t3-t2)*self.p3
        b1 = (t2-t)/(t2-t0)*a1 + (t-t0)/(t2-t0)*a2; b2 = (t3-t)/(t3-t1)*a2 + (t-t1)/(t3-t1)*a3
        return (t2-t)/(t2-t1)*b1 + (t-t1)/(t2-t1)*b2
    def tangent(self, t_global):
        if self.is_degenerate: return Vector((0,0,0))
        h = 0.0001; return (self.eval(t_global + h) - self.eval(t_global - h)).normalized()

class CatmullRomSpline:
    def __init__(self, points, is_closed=False):
        self.segments = []; self.is_closed = is_closed; self.total_length_approx = 0.0
        clean_pts = []
        if len(points) > 0:
            clean_pts.append(points[0])
            for p in points[1:]:
                if (p - clean_pts[-1]).length_squared > 1e-8: clean_pts.append(p)
        if len(clean_pts) < 2: return
        if is_closed: padded_pts = [clean_pts[-1]] + clean_pts + clean_pts[:2]
        else: start_ghost = clean_pts[0] + (clean_pts[0] - clean_pts[1]); end_ghost = clean_pts[-1] + (clean_pts[-1] - clean_pts[-2]); padded_pts = [start_ghost] + clean_pts + [end_ghost]
        for i in range(len(padded_pts) - 3):
            seg = CubicSegment(padded_pts[i], padded_pts[i+1], padded_pts[i+2], padded_pts[i+3])
            self.segments.append(seg); self.total_length_approx += (padded_pts[i+2] - padded_pts[i+1]).length
    def project(self, point, samples=10):
        if not self.segments: return point, Vector((1,0,0)), 0.0
        best_dist_sq = 1e9; best_seg_idx = -1; best_t = 0.0
        for idx, seg in enumerate(self.segments):
            steps = 5
            for i in range(steps):
                t = seg.t_start + (seg.dt * (i / (steps-1))); pt = seg.eval(t); d_sq = (point - pt).length_squared
                if d_sq < best_dist_sq: best_dist_sq = d_sq; best_seg_idx = idx; best_t = t
        if best_seg_idx == -1: return point, Vector((1,0,0)), 0.0
        seg = self.segments[best_seg_idx]
        low = max(seg.t_start, best_t - seg.dt * 0.2); high = min(seg.t_end, best_t + seg.dt * 0.2)
        for _ in range(8):
            m1 = low + (high - low) / 3; m2 = high - (high - low) / 3
            d1 = (seg.eval(m1) - point).length_squared; d2 = (seg.eval(m2) - point).length_squared
            if d1 < d2: high = m2
            else: low = m1
        final_t = (low + high) / 2; final_pos = seg.eval(final_t); final_tan = seg.tangent(final_t)
        return final_pos, final_tan, (final_pos - point).length

# =========================================================================
# --- 2. THE ITERATIVE SOLVER ---
# =========================================================================

def solve_medial_axis_point(seed, spline1, spline2, Zp, iterations=5):
    """
    Relaxation method:
    1. Find closest points P1, P2
    2. Project current center onto the perpendicular bisector of P1-P2
    3. Repeat.
    """
    curr = seed.copy()
    
    for _ in range(iterations):
        # 1. Get closest points
        p1, _, _ = spline1.project(curr)
        p2, _, _ = spline2.project(curr)
        
        # 2. Chord vector and distance
        diff = p2 - p1
        dist = diff.length
        
        if dist < 1e-6:
            # Curves define a 0-radius circle (intersection)
            return (p1 + p2) * 0.5, 0.0
            
        # 3. Midpoint
        mid = (p1 + p2) * 0.5
        
        # 4. Perpendicular Bisector Direction
        rail_dir = diff.cross(Zp).normalized()
        
        # 5. Project current guess onto that bisector line
        to_curr = curr - mid
        dot = to_curr.dot(rail_dir)
        
        curr = mid + rail_dir * dot
        
    # Final Radius (average of distances)
    p1, _, _ = spline1.project(curr)
    p2, _, _ = spline2.project(curr)
    r = ((curr - p1).length + (curr - p2).length) * 0.5
    
    return curr, r

# =========================================================================
# --- 3. HELPER UTILS ---
# =========================================================================

def get_selected_edge_chains(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    sel_edges = [e for e in bm.edges if e.select]
    if not sel_edges: return []
    adj = {}; chains = []; processed_verts = set(); mw = obj.matrix_world
    for e in sel_edges:
        v1, v2 = e.verts
        if v1 not in adj: adj[v1] = []
        if v2 not in adj: adj[v2] = []
        adj[v1].append(v2); adj[v2].append(v1)
    all_verts = sorted(list(adj.keys()), key=lambda v: v.index)
    for v in all_verts:
        if v in processed_verts: continue
        component = []; stack = [v]; processed_verts.add(v)
        while stack:
            curr = stack.pop(); component.append(curr)
            for n in adj[curr]:
                if n not in processed_verts: processed_verts.add(n); stack.append(n)
        endpoints = [vert for vert in component if len(adj[vert]) == 1]; ordered_verts = []
        if len(endpoints) == 2:
            curr = endpoints[0]; visited = {curr}; ordered_verts.append(curr)
            while True:
                found = False
                for n in adj[curr]:
                    if n not in visited: visited.add(n); ordered_verts.append(n); curr = n; found = True; break
                if not found: break
        elif not endpoints and component:
            curr = component[0]; visited = {curr}; ordered_verts.append(curr)
            while True:
                found = False
                for n in adj[curr]:
                    if n not in visited: visited.add(n); ordered_verts.append(n); curr = n; found = True; break
                if not found: break
        else: ordered_verts = component
        pts = [mw @ v.co for v in ordered_verts]
        if len(pts) > 1: chains.append((pts, not endpoints))
    return chains

# =========================================================================
# --- 4. TOOL CLASSES ---
# =========================================================================

class CircleTool_2Point(SurfaceDrawTool):
    def __init__(self, core): 
        super().__init__(core)
        self.mode="CIRCLE_2POINT"
        self.segments=self.state["segments"]
        self.radius=0.0
        self.current=None
        self.preview_pts=[]
        self.constraint_axis=None
        self.ref_normal = Vector((0,0,1)) 
        self.vertical_override_axis = None 

    def update(self, context, event, snap_point, snap_normal):
        # 0. Check for numerical input override
        if self.state.get("skip_mouse_update"):
            self.state["skip_mouse_update"] = False
            if self.state.get("current"):
                self.current = self.state["current"].copy()
            if self.state.get("radius"):
                self.radius = self.state["radius"]
            
            # Re-establish coordinate basis if needed
            if self.current and self.pivot:
                bridge = self.current - self.pivot
                if bridge.length_squared > 1e-8:
                    b_vec = bridge.normalized()
                    up = self.ref_normal
                    if not self.Zp: self.Zp = up.copy()
                    if not self.Xp: self.Xp = b_vec
                    if not self.Yp: self.Yp = self.Zp.cross(self.Xp).normalized()

            self.refresh_preview()
            return

        if self.stage==0: 
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return
            
        if self.stage==1:
            # 1. DIRECT PROJECT TO FLOOR PLANE FIRST
            coord = (event.mouse_region_x, event.mouse_region_y)
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            
            target = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.ref_normal)
            if target is None: target = snap_point 

            # 2. APPLY SNAPPING TO THE FLOOR POSITION
            if self.constraint_axis and not event.alt:
                diff = target - self.pivot
                proj = self.constraint_axis * diff.dot(self.constraint_axis)
                target = self.pivot + proj
            
            elif not self.state.get("geometry_snap", False) and not event.alt:
                strength_deg = self.state.get("snap_strength", 6.0)
                strength_deg = max(0.1, min(89.0, strength_deg))
                axis_thresh = math.cos(math.radians(strength_deg))

                inf_loc, _, _ = get_axis_snapped_location(
                    self.pivot, 
                    (event.mouse_region_x, event.mouse_region_y), 
                    context,
                    snap_threshold=axis_thresh
                )
                if inf_loc: target = inf_loc

            # 2. COORDINATE BASIS CALCULATION (Screen-Space Anchor Method)
            bridge = target - self.pivot
            if bridge.length_squared > 1e-8:
                b_vec = bridge.normalized()
                up = self.ref_normal
                is_perp_mode = self.state.get("is_perpendicular", False)
                
                # A. HYSTERESIS: Detect Verticality
                dot_v = abs(b_vec.dot(up))
                was_vertical = getattr(self, "_is_vert_last", False)
                threshold = 0.98 if was_vertical else 0.995 
                is_vertical = dot_v > threshold
                self._is_vert_last = is_vertical

                # B. STABILIZED BASIS VIA SCREEN ANGLE
                reg, rv3d = context.region, context.region_data
                p2d = view3d_utils.location_3d_to_region_2d(reg, rv3d, self.pivot)
                
                if p2d:
                    # Get 2D Screen Angle
                    m2d = Vector((event.mouse_region_x, event.mouse_region_y))
                    screen_dir = (m2d - p2d)
                    if screen_dir.length_squared > 1:
                        # Find the 3D vector that aligns with this screen direction
                        # on the floor plane.
                        view_vec = view3d_utils.region_2d_to_vector_3d(reg, rv3d, m2d)
                        
                        if is_perp_mode or is_vertical:
                            if is_vertical:
                                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                                if self.vertical_override_axis is None:
                                    view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                                    new_Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                                else:
                                    new_Zp = ax_x if self.vertical_override_axis == 'X' else ax_y
                            else:
                                # Normal case: Plane follows the diameter line
                                new_Zp = b_vec.cross(up).normalized()
                            
                            if new_Zp.length > 1e-4:
                                # Stabilize normal relative to view
                                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                                if new_Zp.dot(view_fwd) > 0: new_Zp = -new_Zp
                                
                                self.Zp = new_Zp.normalized()
                                self.Yp = up.normalized()
                                self.Xp = self.Yp.cross(self.Zp).normalized()
                        else:
                            self.Zp = up.copy()
                            self.Xp = b_vec
                            self.Yp = self.Zp.cross(self.Xp).normalized()

            self.current = target
            self.refresh_preview()

    def refresh_preview(self):
        if self.current and self.pivot:
            c = (self.pivot + self.current) * 0.5
            r = (self.current - self.pivot).length * 0.5
            self.radius = r
            self.segments = self.state["segments"]
            if self.Xp and self.Yp:
                self.preview_pts = arc_points_world(c, r, 0.0, 2*math.pi, self.segments, self.Xp, self.Yp) if r > 1e-6 else []
        
        # Sync to shared state
        self.state["preview_pts"] = self.preview_pts

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage==0: 
            self.pivot=snap_point
            self.state["locked"]=False
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage=1
            return 'NEXT_STAGE'
        if self.stage==1: return 'FINISHED'
        return None

    def handle_input(self, context, event):
        # 1. Check Parent Input (L Key)
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True

        # 2. Tool Specifics (P Key)
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None # Reset auto-behavior
            # Reset locking to the floor normal to allow easy rotation
            self.state["locked"] = True
            self.state["locked_normal"] = self.ref_normal
            return True

        # 3. Vertical Orientation Override (X/Y)
        if event.type in {'X', 'Y'} and event.value == 'PRESS':
            if self.pivot and self.current:
                bridge = self.current - self.pivot
                if bridge.length_squared > 1e-8:
                    # Check if we are currently vertical (either auto or via P mode)
                    is_vertical = abs(bridge.normalized().dot(self.ref_normal)) > 0.99
                    if is_vertical or self.state.get("is_perpendicular"):
                        self.vertical_override_axis = event.type
                        self.core.report({'INFO'}, f"Orientation Locked to {event.type}-Facing")
                        return True
        return False

class CircleTool_3Point(SurfaceDrawTool):
    def __init__(self, core): 
        super().__init__(core)
        self.mode="CIRCLE_3POINT"
        self.segments=self.state["segments"]
        self.p1=None
        self.p2=None
        self.current=None
        self.preview_pts=[]
        self.constraint_axis=None
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None

    def update(self, context, event, snap_point, snap_normal):
        if self.stage==0: 
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return
            
        if self.stage==1:
            # 1. DIRECT PROJECT TO FLOOR PLANE FIRST
            coord = (event.mouse_region_x, event.mouse_region_y)
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            
            target = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.ref_normal)
            if target is None: target = snap_point 

            # 2. APPLY SNAPPING TO THE FLOOR POSITION
            if self.constraint_axis and not event.alt:
                diff = target - self.pivot
                proj = self.constraint_axis * diff.dot(self.constraint_axis)
                target = self.pivot + proj
            
            elif not self.state.get("geometry_snap", False) and not event.alt:
                strength_deg = self.state.get("snap_strength", 6.0)
                strength_deg = max(0.1, min(89.0, strength_deg))
                axis_thresh = math.cos(math.radians(strength_deg))

                inf_loc, _, _ = get_axis_snapped_location(
                    self.pivot, 
                    (event.mouse_region_x, event.mouse_region_y), 
                    context,
                    snap_threshold=axis_thresh
                )
                if inf_loc: target = inf_loc

            # 2. COORDINATE BASIS CALCULATION (Screen-Space Anchor Method)
            bridge = target - self.pivot
            if bridge.length_squared > 1e-8:
                b_vec = bridge.normalized()
                up = self.ref_normal
                is_perp_mode = self.state.get("is_perpendicular", False)
                
                # A. HYSTERESIS: Detect Verticality
                dot_v = abs(b_vec.dot(up))
                was_vertical = getattr(self, "_is_vert_last_3pt", False)
                threshold = 0.98 if was_vertical else 0.995 
                is_vertical = dot_v > threshold
                self._is_vert_last_3pt = is_vertical

                # B. STABILIZED BASIS VIA SCREEN ANGLE
                reg, rv3d = context.region, context.region_data
                p2d = view3d_utils.location_3d_to_region_2d(reg, rv3d, self.pivot)
                
                if p2d:
                    m2d = Vector((event.mouse_region_x, event.mouse_region_y))
                    screen_dir = (m2d - p2d)
                    if screen_dir.length_squared > 1:
                        if is_perp_mode or is_vertical:
                            if is_vertical:
                                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                                if self.vertical_override_axis is None:
                                    view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                                    new_Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                                else:
                                    new_Zp = ax_x if self.vertical_override_axis == 'X' else ax_y
                            else:
                                new_Zp = b_vec.cross(up).normalized()
                            
                            if new_Zp.length > 1e-4:
                                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                                if new_Zp.dot(view_fwd) > 0: new_Zp = -new_Zp
                                
                                self.Zp = new_Zp.normalized()
                                self.Yp = up.normalized()
                                self.Xp = self.Yp.cross(self.Zp).normalized()
                        else:
                            self.Zp = up.copy()
                            self.Xp = b_vec
                            self.Yp = self.Zp.cross(self.Xp).normalized()

            self.current = target
            self.preview_pts = [self.pivot, target]
            return

        if self.stage==2:
            # --- FIX: Stay on established drawing plane (self.Zp) or stabilize based on chord ---
            plane_n = self.Zp if self.Zp else Vector((0,0,1))
            
            # --- SCREEN-SPACE ANCHOR STABILIZATION FOR 3-POINT ---
            chord_vec = self.p2 - self.p1
            if chord_vec.length > 1e-6:
                chord_dir = chord_vec.normalized()
                rv3d = context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))

                # Detect Verticality with Hysteresis
                dot_v = abs(chord_dir.dot(self.ref_normal))
                was_vertical = getattr(self, "_is_vert_last_3pt", False)
                threshold = 0.98 if was_vertical else 0.995 
                is_vertical = dot_v > threshold
                self._is_vert_last_3pt = is_vertical

                if is_vertical:
                    # Case A: Snap is vertical. Use View-Aligned or Manual Override.
                    ax_x, ax_y, _ = orthonormal_basis_from_normal(self.ref_normal)
                    if self.vertical_override_axis is None:
                        view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                        plane_n = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                    else:
                        plane_n = ax_x if self.vertical_override_axis == 'X' else ax_y
                else:
                    # Case B: Standard orientation. Stabilize normal to face camera.
                    p1, p2, p3 = self.p1, self.p2, snap_point
                    v1 = p2 - p1
                    v2 = p3 - p1
                    if v1.length > 1e-6 and v2.length > 1e-6 and abs(v1.normalized().dot(v2.normalized())) < 0.999:
                        plane_n = v1.cross(v2).normalized()
                    
                    if plane_n.dot(view_fwd) > 0: plane_n = -plane_n
                
                self.Zp = plane_n

            # Project mouse point onto the established plane
            self.current = snap_point
            p1, p2, p3 = self.p1, self.p2, snap_point
            
            d_p3 = p3 - p1
            d_plane = d_p3 - plane_n * d_p3.dot(plane_n)
            p3_proj = p1 + d_plane
            
            m1 = (p1 + p3_proj) / 2
            m2 = (p2 + p3_proj) / 2
            vec1 = p3_proj - p1
            vec2 = p2 - p3_proj
            
            if vec1.length < 1e-5 or vec2.length < 1e-5:
                self.preview_pts = [p1, p3_proj, p2]
                return

            n1 = plane_n.cross(vec1)
            n2 = plane_n.cross(vec2)
            
            isect = geometry.intersect_line_line(m1, m1 + n1, m2, m2 + n2)
            if isect is None:
                self.preview_pts = [p1, p3_proj, p2]
            else:
                c1, c2 = isect
                center = (c1 + c2) / 2
                radius = (p1 - center).length
                self.radius = radius
                
                X_arc = (p1 - center).normalized()
                Y_arc = plane_n.cross(X_arc).normalized()
                self.preview_pts = arc_points_world(center, radius, 0.0, 2*math.pi, self.state["segments"], X_arc, Y_arc)

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage==0: 
            self.pivot=snap_point
            self.p1=snap_point
            self.state["locked"]=False
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage=1
            return 'NEXT_STAGE'
        if self.stage==1:
            self.p2=self.current
            self.stage=2
            return 'NEXT_STAGE'
        if self.stage==2: return 'FINISHED'
        return None

    def handle_input(self, context, event):
        # 1. Check Parent Input (L Key)
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True

        # 2. Tool Specifics (P Key)
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"] = True
            self.state["locked_normal"] = self.ref_normal
            return True

        # 3. Vertical Orientation Override (X/Y)
        if event.type in {'X', 'Y'} and event.value == 'PRESS':
            if self.pivot and self.current:
                bridge = self.current - self.pivot
                if bridge.length_squared > 1e-8:
                    is_vertical = abs(bridge.normalized().dot(self.ref_normal)) > 0.99
                    if is_vertical or self.state.get("is_perpendicular"):
                        self.vertical_override_axis = event.type
                        self.core.report({'INFO'}, f"Orientation Locked to {event.type}-Facing")
                        return True
        return False

# --- RESTORED TAN TAN CLASS (No Rail Lag) ---
class CircleTool_TanTan(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "CIRCLE_TAN_TAN"
        self.segments = self.state["segments"]
        self.radius = 0.0
        self.current = None
        self.preview_pts = []
        
        self.spline_1 = None
        self.spline_2 = None
        
        ctx = bpy.context
        self.Zp = Vector((0,0,1)); self.Xp = Vector((1,0,0)); self.Yp = Vector((0,1,0))

        if ctx.edit_object:
            chains = get_selected_edge_chains(ctx.edit_object)
            
            if len(chains) >= 2:
                # --- AUTO-DETECT CURVES ---
                raw_pts1, closed1 = chains[0]
                raw_pts2, closed2 = chains[1]
                self.spline_1 = CatmullRomSpline(raw_pts1, is_closed=closed1)
                self.spline_2 = CatmullRomSpline(raw_pts2, is_closed=closed2)
                
                if self.spline_1.segments and self.spline_2.segments:
                    self.has_preselection = True
                    # Estimate Plane
                    if len(raw_pts1) > 2:
                        v1 = raw_pts1[1] - raw_pts1[0]; v2 = raw_pts1[-1] - raw_pts1[0]
                        cross = v1.cross(v2)
                        if cross.length > 1e-4:
                            self.Zp = cross.normalized()
                            vals = orthonormal_basis_from_normal(self.Zp)
                            if len(vals) == 3: self.Xp, self.Yp, _ = vals
                            else: self.Xp, self.Yp = vals
                    
                    self.state["locked"] = True
                    self.state["locked_normal"] = self.Zp
                    self.state.update({"Xp": self.Xp, "Yp": self.Yp})

                    # --- POPULATE CATMULL CURVE OVERLAYS ---
                    catmull_previews = []
                    for spline in [self.spline_1, self.spline_2]:
                        curve_pts = []
                        for seg in spline.segments:
                            curve_pts.extend([seg.eval(seg.t_start + t*seg.dt) for t in [0.0, 0.25, 0.5, 0.75]])
                        if spline.segments:
                            curve_pts.append(spline.segments[-1].eval(spline.segments[-1].t_end))
                        catmull_previews.append(curve_pts)
                    self.state["catmull_spline_previews"] = catmull_previews

                    # --- NO RAIL GENERATION (Speed Fix) ---
                    # Just smooth the curves for display, no heavy rail calculation
                    self.state["smooth_curve_1"] = [self.spline_1.segments[i].eval(self.spline_1.segments[i].t_start + t*self.spline_1.segments[i].dt) for i in range(len(self.spline_1.segments)) for t in [0.0, 0.33, 0.66]]
                    self.state["smooth_curve_2"] = [self.spline_2.segments[i].eval(self.spline_2.segments[i].t_start + t*self.spline_2.segments[i].dt) for i in range(len(self.spline_2.segments)) for t in [0.0, 0.33, 0.66]]
                    
                    self.stage = 1
                    # Start at the beginning of the curve (Instant!)
                    self.pivot = raw_pts1[0]
                    self.current = raw_pts1[0]
            
            elif len(chains) == 1:
                # Fallback for single curve
                raw_pts1, closed1 = chains[0]
                self.spline_1 = CatmullRomSpline(raw_pts1, is_closed=closed1)
                if self.spline_1.segments:
                    self.has_preselection = True
                    self.state["smooth_curve_1"] = [self.spline_1.segments[i].eval(self.spline_1.segments[i].t_start + t*self.spline_1.segments[i].dt) for i in range(len(self.spline_1.segments)) for t in [0.0, 0.33, 0.66]]
                    self.state["smooth_curve_2"] = []
                    self.pivot = raw_pts1[0]

    def update(self, context, event, snap_point, snap_normal):
        # Stage 0: Fallback
        if self.stage == 0:
            self.current = snap_point
            self.update_initial_plane(context, event, snap_point, snap_normal)
            return

        # Stage 1: Live Solver
        if self.stage == 1:
            self.segments = self.state.get("segments", 64)
            mouse_pos = snap_point
            if self.pivot is None: self.pivot = mouse_pos
                
            d = mouse_pos - self.pivot 
            d_plane = d - self.Zp * d.dot(self.Zp)
            mouse_on_plane = self.pivot + d_plane
            
            # Use Mouse as Seed for LIVE Solver
            solved_c, solved_r = solve_medial_axis_point(mouse_on_plane, self.spline_1, self.spline_2, self.Zp)
            
            self.current = solved_c
            self.radius = solved_r
            self.pivot = solved_c
            
            # Visualization of the tangency
            p1_v, _, _ = self.spline_1.project(self.current)
            p2_v, _, _ = self.spline_2.project(self.current)
            self.state["viz_tangent_line"] = (self.current, p1_v)
            self.state["viz_diameter_line"] = (self.current, p2_v)
            self.state["viz_opposite_dot"] = None

            if self.radius > 1e-5:
                self.preview_pts = arc_points_world(
                    self.current, self.radius, 0.0, 2 * math.pi, self.segments, self.Xp, self.Yp
                )
                self.state["visual_pts"] = arc_points_world(
                    self.current, self.radius, 0.0, 2 * math.pi, 128, self.Xp, self.Yp
                )
            else:
                self.preview_pts = []; self.state["visual_pts"] = []

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.state["locked"] = False
            self.pivot = snap_point
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage == 1:
            return 'FINISHED'
        return None