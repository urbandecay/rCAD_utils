import math
import bmesh
from mathutils import Vector, Matrix
from ..geometry_utils import snap_angle_soft, unwrap, arc_points_world
from ..plane_utils import world_to_plane, plane_to_world
from ..orientation_utils import orthonormal_basis_from_normal
from .base_tool import SurfaceDrawTool, CAD_BaseTool

def intersect_circles_3d(c1, r1, c2, r2, Xp, Yp):
    """
    Finds intersection points of two circles on the same plane (defined by Xp, Yp).
    Returns a list of 3D Vectors.
    """
    # Project c2 onto c1's local plane basis
    d_vec = c2 - c1
    dx = d_vec.dot(Xp)
    dy = d_vec.dot(Yp)
    d = math.sqrt(dx*dx + dy*dy)
    
    # Check for no intersection or containment
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        return []
    
    a = (r1**2 - r2**2 + d**2) / (2*d)
    h_sq = r1**2 - a**2
    h = math.sqrt(max(0, h_sq))
    
    # P2 is the point on the line connecting centers
    p2_x = dx * a / d
    p2_y = dy * a / d
    
    # Offsets for intersection points
    ox = -dy * h / d
    oy = dx * h / d
    
    pts = []
    
    # Intersection 1
    i1_local_x = p2_x + ox
    i1_local_y = p2_y + oy
    pts.append(c1 + Xp * i1_local_x + Yp * i1_local_y)
    
    # Intersection 2 (if distinct)
    if h > 1e-6:
        i2_local_x = p2_x - ox
        i2_local_y = p2_y - oy
        pts.append(c1 + Xp * i2_local_x + Yp * i2_local_y)
        
    return pts

def is_angle_in_arc(pt, center, Xp, Yp, a0, a1):
    """Checks if a point lies within the directed angular sweep from a0 to a1, with very high tolerance."""
    d = a1 - a0
    # 1. Full Circle Check
    if abs(d) >= 2 * math.pi - 0.05:
        return True
        
    # 2. Project point to local 2D angle
    v = pt - center
    lx, ly = v.dot(Xp), v.dot(Yp)
    ang = math.atan2(ly, lx)
    
    # 3. Get positive forward distance from a0 to ang
    diff = ang - a0
    diff_norm = diff % (2 * math.pi)
    
    # 4. HIGH TOLERANCE (approx 10 degrees)
    tol = 0.17 
    
    # 5. Check against sweep direction
    if d >= 0:
        # Forward sweep: valid if diff_norm is in [0, d] with tolerance
        return diff_norm <= (d + tol) or diff_norm >= (2 * math.pi - tol)
    else:
        # Backward sweep
        back_dist = (2 * math.pi - diff_norm) % (2 * math.pi)
        return back_dist <= (abs(d) + tol) or back_dist >= (2 * math.pi - tol)

class PointTool_ByArcs(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "POINT_BY_ARCS"
        
        # Arc 1 Data
        self.arc1_done = False
        self.c1 = None
        self.r1 = 0.0
        self.a0_1 = 0.0
        self.a1_1 = 0.0
        self.endpoints_1 = [] # To store Arc 1 Start/End for snapping
        
        # Current Arc (Shared)
        self.start = None
        self.current = None
        self.radius = 0.0
        self.a0 = 0.0
        self.a1 = 0.0
        self.a_prev_raw = 0.0
        self.accum_angle = 0.0
        self.compass_rot = 0.0
        
        # Display
        self.segments = 128 # Fixed High res for smooth curves
        self.preview_pts = []
        self.intersection_pts = [] 
        self.state["arc1_pts"] = []
        self.state["intersection_pts"] = []
        self.constraint_axis = None

    def handle_input(self, context, event):
        # 1. Check Parent Input first (L Key)
        if self.handle_plane_lock_input(context, event):
            return True

        # 2. Perpendicular Lock (P Key)
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            
            bridge = None
            if self.stage in [1, 2] and self.pivot:
                if self.stage == 1 and self.current: bridge = self.current - self.pivot
                elif self.stage == 2 and self.start: bridge = self.start - self.pivot
            elif self.stage in [4, 5] and self.pivot:
                if self.stage == 4 and self.current: bridge = self.current - self.pivot
                elif self.stage == 5 and self.start: bridge = self.start - self.pivot
            
            if bridge and bridge.length_squared > 1e-6:
                self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
                
                if self.state["is_perpendicular"]:
                    b_vec = bridge.normalized()
                    floor_n = self.state.get("locked_normal") or Vector((0,0,1))
                    new_Zp = b_vec.cross(floor_n).normalized()
                    new_Yp = floor_n 
                    new_Xp = new_Yp.cross(new_Zp).normalized() 
                    
                    self.Xp, self.Yp, self.Zp = new_Xp, new_Yp, new_Zp
                    self.state["locked"] = True
                    self.state["locked_normal"] = new_Zp
                else:
                    self.state["locked"] = False
                    self.state["locked_normal"] = None

                # --- FIX: Re-map existing arcs to the new plane ---
                if self.stage > 0:
                    # 1. Re-map Arc 1 (if started/done)
                    if self.start: # Arc 1 or Arc 2 is active
                        if self.stage <= 2: # Arc 1 is active
                            pv = self.pivot
                            d_start = self.start - pv
                            d_plane = d_start - self.Zp * d_start.dot(self.Zp)
                            self.start = pv + d_plane
                            
                            rvec2 = world_to_plane(self.start - pv, self.Xp, self.Yp)
                            self.radius = rvec2.length
                            self.a0 = math.atan2(rvec2.y, rvec2.x)
                            self.a1 = self.a0 + self.accum_angle
                            self.a_prev_raw = self.a0
                        
                        else: # Arc 2 is active
                            # Arc 1 is already 'done' and stored in self.c1, self.r1, etc.
                            # We must re-project its center and endpoints
                            if self.c1:
                                # (For PointsByArcs, usually Arc 1 and 2 share the same plane)
                                d_c1 = self.c1 - self.pivot # Pivot here is C2
                                d_plane = d_c1 - self.Zp * d_c1.dot(self.Zp)
                                self.c1 = self.pivot + d_plane
                                
                                # Re-map Arc 2 (active)
                                pv = self.pivot
                                d_start = self.start - pv
                                d_plane = d_start - self.Zp * d_start.dot(self.Zp)
                                self.start = pv + d_plane
                                
                                rvec2 = world_to_plane(self.start - pv, self.Xp, self.Yp)
                                self.radius = rvec2.length
                                self.a0 = math.atan2(rvec2.y, rvec2.x)
                                self.a1 = self.a0 + self.accum_angle
                                self.a_prev_raw = self.a0
                return True

        # 3. Axis Locking (X/Y/Z) - Locks Plane Normal (like Arc 1-Point)
        if event.type in {'X', 'Y', 'Z'} and event.value == 'PRESS':
            # --- FIX: Only allow X/Y/Z plane locking BEFORE drawing starts (Stage 0) ---
            if self.stage > 0: return False

            axes = {'X': Vector((1, 0, 0)), 'Y': Vector((0, 1, 0)), 'Z': Vector((0, 0, 1))}
            new_n = axes[event.type]
            
            current_locked = self.state.get("locked")
            current_normal = self.state.get("locked_normal")
            
            is_same_axis = False
            if current_locked and current_normal:
                if abs(current_normal.dot(new_n)) > 0.99:
                    is_same_axis = True

            if is_same_axis:
                self.state["locked"] = False
                self.state["locked_normal"] = None
                self.state["locked_plane_point"] = None
                self.core.report({'INFO'}, f"Unlocked {event.type}-Plane")
            else:
                self.Zp = new_n
                self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
                self.state["locked"] = True
                self.state["locked_normal"] = self.Zp
                
                # --- FIX: Set plane point to where the compass is RIGHT NOW ---
                target_point = self.current if self.current else Vector((0,0,0))
                self.state["locked_plane_point"] = target_point
                
                self.core.report({'INFO'}, f"Locked to {event.type}-Plane")
            return True

        return False

    def update(self, context, event, snap_point, snap_normal):
        self.current = snap_point
        
        # --- PROJECT ONTO LOCKED PLANE IF APPLICABLE ---
        if self.state.get("locked") and self.Zp and self.pivot:
            d = snap_point - self.pivot
            d_plane = d - self.Zp * d.dot(self.Zp)
            self.current = self.pivot + d_plane
        
        # --- STAGE 0: PIVOT 1 (First Arc) ---
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            return

        # --- STAGE 1: RADIUS 1 ---
        if self.stage == 1:
            pv = self.pivot
            d_vec = self.current - pv
            d2 = world_to_plane(d_vec, self.Xp, self.Yp)
            self.radius = d2.length
            
            raw_angle = math.atan2(d2.y, d2.x)
            
            if self.state.get("geometry_snap", False): final_angle = raw_angle
            else:
                strength = self.state.get("snap_strength", 6.0)
                if self.state.get("use_angle_snap", True):
                    final_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)
                else: final_angle = raw_angle
            
            self.compass_rot = final_angle
            snapped_vec2 = Vector((math.cos(final_angle), math.sin(final_angle))) * self.radius
            self.current = pv + plane_to_world(snapped_vec2, self.Xp, self.Yp)
            return

        # --- STAGE 2: ANGLE 1 ---
        if self.stage == 2:
            pv = self.pivot
            d_vec = self.current - pv
            d2 = world_to_plane(d_vec, self.Xp, self.Yp)
            raw_angle = math.atan2(d2.y, d2.x)

            strength = self.state.get("snap_strength", 6.0)
            if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                raw_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)

            accum, a_prev = unwrap(self.a_prev_raw, raw_angle, self.accum_angle)
            self.a_prev_raw = a_prev
            self.accum_angle = accum
            self.a1 = self.a0 + accum

            self.preview_pts = arc_points_world(
                pv, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
            )
            return

        # --- STAGE 3: PIVOT 2 (Wait for click) ---
        if self.stage == 3:
            # self.current is already projected onto locked plane at top of update
            return

        # --- STAGE 4: RADIUS 2 ---
        if self.stage == 4:
            pv = self.pivot # Now pivot is C2
            d_vec = self.current - pv
            d2 = world_to_plane(d_vec, self.Xp, self.Yp)
            self.radius = d2.length
            
            raw_angle = math.atan2(d2.y, d2.x)
            if self.state.get("geometry_snap", False): final_angle = raw_angle
            else:
                strength = self.state.get("snap_strength", 6.0)
                if self.state.get("use_angle_snap", True):
                    final_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)
                else: final_angle = raw_angle
                
            self.compass_rot = final_angle
            snapped_vec2 = Vector((math.cos(final_angle), math.sin(final_angle))) * self.radius
            self.current = pv + plane_to_world(snapped_vec2, self.Xp, self.Yp)
            
            # --- CALCULATE INTERSECTION FOR PREVIEW (FULL CIRCLE FOR ARC 2) ---
            self.intersection_pts = []
            if self.c1 is not None and self.r1 > 1e-6 and self.radius > 1e-6:
                candidates = intersect_circles_3d(self.c1, self.r1, pv, self.radius, self.Xp, self.Yp)
                valid_ints = []
                for pt in candidates:
                    # Check if point is on Arc 1
                    if is_angle_in_arc(pt, self.c1, self.Xp, self.Yp, self.a0_1, self.a1_1):
                        valid_ints.append(pt)
                self.intersection_pts = valid_ints
                self.state["intersection_pts"] = valid_ints
                
            return

        # --- STAGE 5: ANGLE 2 & SOLVE ---
        if self.stage == 5:
            pv = self.pivot # C2
            d_vec = self.current - pv
            d2 = world_to_plane(d_vec, self.Xp, self.Yp)
            raw_angle = math.atan2(d2.y, d2.x)

            strength = self.state.get("snap_strength", 6.0)
            if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                raw_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)

            accum, a_prev = unwrap(self.a_prev_raw, raw_angle, self.accum_angle)
            self.a_prev_raw = a_prev
            self.accum_angle = accum
            self.a1 = self.a0 + accum

            # Update Arc 2 Preview
            self.preview_pts = arc_points_world(
                pv, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
            )
            
            # --- CALCULATE INTERSECTION ---
            self.intersection_pts = []
            if self.c1 is not None and self.r1 > 1e-6:
                # Find circle-circle intersections
                candidates = intersect_circles_3d(self.c1, self.r1, pv, self.radius, self.Xp, self.Yp)
                
                valid_ints = []
                for pt in candidates:
                    # Check if point is on Arc 1
                    in_arc1 = is_angle_in_arc(pt, self.c1, self.Xp, self.Yp, self.a0_1, self.a1_1)
                    # Check if point is on Arc 2
                    in_arc2 = is_angle_in_arc(pt, pv, self.Xp, self.Yp, self.a0, self.a1)
                    
                    if in_arc1 and in_arc2:
                        valid_ints.append(pt)
                
                self.intersection_pts = valid_ints
                self.state["intersection_pts"] = valid_ints

    def refresh_preview(self):
        """Recalculates previews and intersections based on current internal variables."""
        if self.stage in [1, 2]:
            # Refresh Arc 1
            self.preview_pts = arc_points_world(
                self.pivot, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
            )
        elif self.stage in [4, 5]:
            # Refresh Arc 2
            self.preview_pts = arc_points_world(
                self.pivot, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
            )
            
            # Recalculate Intersections
            self.intersection_pts = []
            if self.c1 is not None and self.r1 > 1e-6 and self.radius > 1e-6:
                candidates = intersect_circles_3d(self.c1, self.r1, self.pivot, self.radius, self.Xp, self.Yp)
                valid_ints = []
                for pt in candidates:
                    in_arc1 = is_angle_in_arc(pt, self.c1, self.Xp, self.Yp, self.a0_1, self.a1_1)
                    if self.stage == 4: # Radius 2 (Arc 2 is a full circle for preview)
                        if in_arc1: valid_ints.append(pt)
                    else: # Angle 2
                        in_arc2 = is_angle_in_arc(pt, self.pivot, self.Xp, self.Yp, self.a0, self.a1)
                        if in_arc1 and in_arc2: valid_ints.append(pt)
                self.intersection_pts = valid_ints
                self.state["intersection_pts"] = valid_ints

        # Sync back to shared state
        self.state["preview_pts"] = self.preview_pts

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        # Stage 0: Commit Pivot 1
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"] = True
            self.state["locked_normal"] = self.Zp
            self.stage = 1
            return 'NEXT_STAGE'

        # Stage 1: Commit Radius 1 / Start 1
        if self.stage == 1:
            target = snap_point
            if self.Xp is not None and self.Zp is not None:
                d = target - self.pivot
                d_plane = d - self.Zp * d.dot(self.Zp)
                target = self.pivot + d_plane
            self.start = target
            
            # Init Angle
            rvec2 = world_to_plane(self.start - self.pivot, self.Xp, self.Yp)
            self.radius = rvec2.length
            if rvec2.length > 1e-9: rvec2.normalize()
            a0 = math.atan2(rvec2.y, rvec2.x)
            
            # Snap start angle
            if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                strength = self.state.get("snap_strength", 6.0)
                a0 = snap_angle_soft(a0, 15.0, strength)
                snapped_vec2 = Vector((math.cos(a0), math.sin(a0))) * self.radius
                self.start = self.pivot + plane_to_world(snapped_vec2, self.Xp, self.Yp)

            self.stage = 2
            self.a0 = a0; self.a1 = a0; self.a_prev_raw = a0; self.accum_angle = 0.0
            return 'NEXT_STAGE'

        # Stage 2: Finish Arc 1
        if self.stage == 2:
            # Save Arc 1 Data
            self.c1 = self.pivot
            self.r1 = self.radius
            self.a0_1 = self.a0
            self.a1_1 = self.a1
            self.state["arc1_pts"] = self.preview_pts # Persist for display
            
            # Store endpoints for snapping
            if self.preview_pts and len(self.preview_pts) >= 2:
                self.endpoints_1 = [self.preview_pts[0], self.preview_pts[-1]]
            
            self.preview_pts = [] # Clear for next
            
            self.stage = 3 # Ready for Pivot 2
            return 'NEXT_STAGE'

        # Stage 3: Commit Pivot 2
        if self.stage == 3:
            target = snap_point
            if self.state.get("locked") and self.Zp and self.c1:
                d = target - self.c1
                d_plane = d - self.Zp * d.dot(self.Zp)
                target = self.c1 + d_plane
            
            self.pivot = target
            # Keep plane locked from Arc 1
            self.stage = 4
            return 'NEXT_STAGE'

        # Stage 4: Commit Radius 2 / Start 2
        if self.stage == 4:
            target = snap_point
            if self.Xp is not None and self.Zp is not None:
                d = target - self.pivot
                d_plane = d - self.Zp * d.dot(self.Zp)
                target = self.pivot + d_plane
            self.start = target
            
            rvec2 = world_to_plane(self.start - self.pivot, self.Xp, self.Yp)
            self.radius = rvec2.length
            a0 = math.atan2(rvec2.y, rvec2.x)
            
            if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                strength = self.state.get("snap_strength", 6.0)
                a0 = snap_angle_soft(a0, 15.0, strength)
                snapped_vec2 = Vector((math.cos(a0), math.sin(a0))) * self.radius
                self.start = self.pivot + plane_to_world(snapped_vec2, self.Xp, self.Yp)

            self.stage = 5
            self.a0 = a0; self.a1 = a0; self.a_prev_raw = a0; self.accum_angle = 0.0
            return 'NEXT_STAGE'

        # Stage 5: Finish
        if self.stage == 5:
            return 'FINISHED'

        return None


def fit_circle_to_points_3d(world_pts):
    """Best-fit circle through 3D points that roughly lie on a plane.
    Returns (center_3d, radius) or (None, 0.0) on failure."""
    if len(world_pts) < 3:
        return None, 0.0

    n = len(world_pts)
    centroid = sum(world_pts, Vector()) / n

    # Estimate plane normal via averaged cross products of consecutive edges
    normal = Vector((0.0, 0.0, 0.0))
    for i in range(n):
        a = world_pts[i] - centroid
        b = world_pts[(i + 1) % n] - centroid
        normal += a.cross(b)

    if normal.length_squared < 1e-12:
        return None, 0.0
    normal.normalize()

    # Build orthonormal plane basis
    temp = Vector((1.0, 0.0, 0.0)) if abs(normal.x) < 0.9 else Vector((0.0, 1.0, 0.0))
    Xp = normal.cross(temp).normalized()
    Yp = normal.cross(Xp).normalized()

    # Project points to 2D plane
    pts2d = []
    for wp in world_pts:
        v = wp - centroid
        pts2d.append((v.dot(Xp), v.dot(Yp)))

    # Algebraic least squares circle fit: x^2 + y^2 = a*x + b*y + c
    # Normal equations: (A^T A) [a, b, c]^T = A^T rhs
    s_xx = s_xy = s_yy = s_x = s_y = s_n = 0.0
    s_xr = s_yr = s_r = 0.0
    for (x, y) in pts2d:
        r2 = x * x + y * y
        s_xx += x * x;  s_xy += x * y;  s_yy += y * y
        s_x  += x;      s_y  += y;      s_n  += 1.0
        s_xr += x * r2; s_yr += y * r2; s_r  += r2

    M = Matrix(((s_xx, s_xy, s_x),
                (s_xy, s_yy, s_y),
                (s_x,  s_y,  s_n)))
    rhs = Vector((s_xr, s_yr, s_r))

    try:
        sol = M.inverted() @ rhs
    except Exception:
        return None, 0.0

    cx2d = sol[0] / 2.0
    cy2d = sol[1] / 2.0
    r = math.sqrt(max(0.0, sol[2] + cx2d * cx2d + cy2d * cy2d))
    center_3d = centroid + Xp * cx2d + Yp * cy2d
    return center_3d, r, Xp, Yp


class PointTool_Center(CAD_BaseTool):
    """Fits a best-fit circle to the selected mesh polygon and places a vertex at the center."""

    def __init__(self, core):
        super().__init__(core)
        self.stage = 0
        # Required by sync_tool_from_state
        self.pivot = None
        self.current = None
        self.Xp = None
        self.Yp = None
        self.Zp = None
        self.intersection_pts = []
        self.catmull_preview = []
        self.state["catmull_center_pts"] = []
        self._compute()

    def _compute(self):
        import bpy
        from .circle_tools import get_selected_edge_chains, CatmullRomSpline

        obj = bpy.context.edit_object
        if not obj:
            return

        chains = get_selected_edge_chains(obj)
        if chains:
            raw_pts, is_closed = chains[0]
        else:
            bm = bmesh.from_edit_mesh(obj.data)
            mw = obj.matrix_world
            raw_pts = [mw @ v.co for v in bm.verts if v.select]
            is_closed = False

        if len(raw_pts) < 3:
            return

        center, radius, Xp, Yp = fit_circle_to_points_3d(raw_pts)
        if center is None:
            return

        # Store on self so sync_tool_from_state doesn't wipe it
        self.intersection_pts = [center]

        # Check if all verts lie on the same circle (perfect polygon/arc)
        is_perfect = False
        if radius > 1e-6:
            dists = [(wp - center).length for wp in raw_pts]
            avg = sum(dists) / len(dists)
            max_dev = max(abs(d - avg) for d in dists)
            is_perfect = (max_dev / avg) < 0.001  # 0.1% tolerance

        if is_perfect:
            # Generate smooth arc using arc_points_world (battle-tested in this codebase)
            n_steps = max(128, 16 * len(raw_pts))
            angles = sorted(math.atan2((wp - center).dot(Yp), (wp - center).dot(Xp)) for wp in raw_pts)
            n = len(angles)
            # Check if wrap-around gap matches the per-vertex gap → closed polygon
            gaps = [angles[i+1] - angles[i] for i in range(n - 1)]
            wrap_gap = (2 * math.pi) - (angles[-1] - angles[0])
            avg_gap = 2 * math.pi / n
            is_closed_polygon = wrap_gap < avg_gap * 1.5
            if is_closed or is_closed_polygon:
                preview = arc_points_world(center, radius, 0.0, 2 * math.pi, n_steps, Xp, Yp)
            else:
                preview = arc_points_world(center, radius, angles[0], angles[-1], n_steps, Xp, Yp)
        else:
            # Irregular shape — use Catmull-Rom
            spline = CatmullRomSpline(raw_pts, is_closed=is_closed)
            preview = []
            steps = 24
            for seg in spline.segments:
                for i in range(steps + 1):
                    t = seg.t_start + (i / steps) * seg.dt
                    preview.append(seg.eval(t))

        self.catmull_preview = preview
        self.state["catmull_center_pts"] = preview

    def handle_input(self, context, event):
        return False

    def update(self, context, event, snap_point, snap_normal):
        # Keep catmull preview in state (not auto-synced by sync_tool_from_state)
        self.state["catmull_center_pts"] = self.catmull_preview

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        return 'FINISHED'