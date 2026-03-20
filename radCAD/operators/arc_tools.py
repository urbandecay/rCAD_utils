import math
from mathutils import Vector, geometry
from bpy_extras import view3d_utils
from ..geometry_utils import snap_angle_soft, unwrap, arc_points_world
from ..plane_utils import world_to_plane, plane_to_world
from ..inference_utils import get_axis_snapped_location
from ..orientation_utils import orthonormal_basis_from_normal
from .base_tool import SurfaceDrawTool 

class ArcTool_Common(SurfaceDrawTool):
    def __init__(self, core, mode="1POINT"):
        super().__init__(core)
        self.mode = mode
        
        # Tool Specifics (Pivot, Xp, Yp, Zp are handled by Parent)
        self.start = None
        self.p1 = None
        self.p2 = None
        self.midpoint = None  
        self.current = None
        self.radius = 0.0
        
        self.a0 = 0.0
        self.a1 = 0.0
        self.a_prev_raw = 0.0
        self.accum_angle = 0.0
        
        self.preview_pts = []
        self.compass_rot = 0.0
        self.segments = self.state["segments"]
        self.constraint_axis = None

        # --- VERTICAL HANDLING ---
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None # None means 'Auto'

    def update(self, context, event, snap_point, snap_normal):
        # 0. Check for numerical input override
        if self.state.get("skip_mouse_update"):
            self.state["skip_mouse_update"] = False
            
            # Sync internal state from shared state (populated by text_entry_utils)
            if self.state.get("p2"): self.p2 = self.state["p2"].copy()
            if self.state.get("current"): self.current = self.state["current"].copy()
            if self.state.get("radius"): self.radius = self.state["radius"]
            if self.state.get("start"): self.start = self.state["start"].copy()
            if self.state.get("stage"): self.stage = self.state["stage"]
            
            # Re-establish coordinate basis if needed
            if self.pivot and self.current:
                bridge = self.current - self.pivot
                if bridge.length_squared > 1e-8:
                    b_vec = bridge.normalized()
                    up = self.ref_normal
                    if not self.Zp: self.Zp = up.copy()
                    if not self.Xp: self.Xp = b_vec
                    if not self.Yp: self.Yp = self.Zp.cross(self.Xp).normalized()
            
            self.refresh_preview()
            return

        self.current = snap_point
        
        # --- STAGE 0: THE PREP CHEF ---
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        # --- STAGE 1 & 2: THE SPECIALIST ---
        
        # =========================================================
        # 1-POINT ARC LOGIC
        # =========================================================
        if self.mode == "1POINT":
            pv = self.pivot 
            
            if self.stage == 1:
                # Dragging Radius
                d = snap_point - pv
                # Project onto the drawing plane (flatten it)
                d_plane = d - self.Zp * d.dot(self.Zp)
                d2 = world_to_plane(d_plane, self.Xp, self.Yp)
                length = d2.length
                raw_angle = math.atan2(d2.y, d2.x)
                
                # --- OVERLAY LOGIC: ROTATING COMPASS ---
                if self.state.get("geometry_snap", False): 
                    final_angle = raw_angle
                else:
                    strength = self.state.get("snap_strength", 6.0)
                    if self.state.get("use_angle_snap", True): 
                        final_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)
                    else: 
                        final_angle = raw_angle
                    
                self.compass_rot = final_angle 
                snapped_vec2 = Vector((math.cos(final_angle), math.sin(final_angle))) * length
                self.current = pv + plane_to_world(snapped_vec2, self.Xp, self.Yp)
                self.radius = length
                return

            if self.stage == 2 and self.start is not None:
                # Dragging Angle
                d_vec = snap_point - pv
                d2 = world_to_plane(d_vec, self.Xp, self.Yp)
                raw_angle = math.atan2(d2.y, d2.x)

                strength = self.state.get("snap_strength", 6.0)
                if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                    raw_angle = snap_angle_soft(raw_angle, self.state.get("angle_increment", 15.0), strength)

                accum, a_prev = unwrap(self.a_prev_raw, raw_angle, self.accum_angle)
                limit = 2 * math.pi
                if abs(accum) > limit:
                    diff = raw_angle - self.a0
                    if math.cos(diff) > 0.8:
                        phase = (accum + math.pi) % (2 * math.pi) - math.pi
                        base_lap = math.copysign(limit, accum)
                        accum = base_lap + phase
                
                self.a_prev_raw = a_prev
                self.accum_angle = accum
                self.a1 = self.a0 + accum

                self.segments = self.state["segments"]
                # --- OVERLAY LOGIC: ARC PREVIEW ---
                self.preview_pts = arc_points_world(
                    pv, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
                )
                return

        # =========================================================
        # 2-POINT ARC LOGIC
        # =========================================================
        elif self.mode == "2POINT":
            if self.stage == 1:
                target = snap_point
                
                # --- FIX: Alt bypasses Axis Snapping in Stage 1 ---
                if self.constraint_axis and not event.alt:
                    if self.state.get("geometry_snap", False):
                        diff = target - self.pivot
                        proj = self.constraint_axis * diff.dot(self.constraint_axis)
                        target = self.pivot + proj
                    else:
                        region = context.region
                        rv3d = context.region_data
                        coord = (event.mouse_region_x, event.mouse_region_y)
                        ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
                        ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
                        res = geometry.intersect_line_line(
                            ray_origin, ray_origin + ray_vector, 
                            self.pivot, self.pivot + self.constraint_axis
                        )
                        if res: target = res[1]
                
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

                self.current = target
                return

            if self.stage == 2:
                # Height Drag logic
                mid = self.midpoint
                chord_vec = self.p2 - self.p1
                chord_len = chord_vec.length
                
                if chord_len < 1e-6:
                    self.preview_pts = []
                    return

                plane_n = self.Zp if self.Zp else Vector((0,0,1))
                chord_dir = chord_vec.normalized()
                
                # --- 2-POINT ARC PLANE LOGIC (Proven Stability Port) ---
                rv3d = context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                
                is_perp_mode = self.state.get("is_perpendicular", False)
                
                # A. Detect Verticality with Hysteresis
                dot_v = abs(chord_dir.dot(self.ref_normal))
                was_vertical = getattr(self, "_is_vert_last", False)
                threshold = 0.98 if was_vertical else 0.995 
                is_vertical = dot_v > threshold
                self._is_vert_last = is_vertical

                if is_vertical:
                    # Case A: Snap is vertical. Use View-Aligned or Manual Override.
                    ax_x, ax_y, _ = orthonormal_basis_from_normal(self.ref_normal)
                    if self.vertical_override_axis is None:
                        plane_n = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                    else:
                        plane_n = ax_x if self.vertical_override_axis == 'X' else ax_y
                elif is_perp_mode:
                    # Case B: Perpendicular mode is ON. Use the plane already stored in self.Zp.
                    plane_n = self.Zp if self.Zp else self.ref_normal
                    # Stabilize normal to face camera
                    if plane_n.dot(view_fwd) > 0: plane_n = -plane_n
                else:
                    # Case C: DEFAULT. Stay FLAT on the floor.
                    plane_n = self.ref_normal
                
                self.Zp = plane_n
                        
                perp_dir = plane_n.cross(chord_dir).normalized()
                target_pt = snap_point
                
                if not self.state.get("geometry_snap", False) and context.region and context.region_data:
                    coord = (event.mouse_region_x, event.mouse_region_y)
                    ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                    ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                    hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, mid, plane_n)
                    if hit: target_pt = hit

                mouse_vec = target_pt - mid
                height = mouse_vec.dot(perp_dir)

                if not self.state.get("geometry_snap", False) and not event.alt:
                    target_h = chord_len / 2.0
                    snap_val = self.state.get("snap_strength", 6.0)
                    tolerance = chord_len * (snap_val / 100.0)
                    if abs(abs(height) - target_h) < tolerance:
                        height = math.copysign(target_h, height)
                
                peak_pt = mid + (perp_dir * height)
                self.pivot = mid 
                self.start = peak_pt
                
                if abs(height) < 0.0001:
                    self.preview_pts = [self.p1, self.p2]
                else:
                    radius = (abs(height) / 2.0) + ((chord_len**2) / (8.0 * abs(height)))
                    self.radius = radius
                    to_center_dir = -perp_dir if height > 0 else perp_dir
                    center = peak_pt + to_center_dir * radius
                    
                    X_arc = chord_dir
                    Y_arc = perp_dir 
                    
                    def to_local(v): return Vector((v.dot(X_arc), v.dot(Y_arc)))
                    v1_2d = to_local(self.p1 - center)
                    v2_2d = to_local(self.p2 - center)
                    vp_2d = to_local(peak_pt - center)
                    
                    ang1 = math.atan2(v1_2d.y, v1_2d.x)
                    ang2 = math.atan2(v2_2d.y, v2_2d.x)
                    ang_mid = math.atan2(vp_2d.y, vp_2d.x)
                    
                    ang_mid_u, _ = unwrap(ang1, ang_mid, ang1)
                    ang2_u, _ = unwrap(ang_mid, ang2, ang_mid_u)
                    
                    self.a0 = ang1
                    self.a1 = ang2_u
                    self.accum_angle = ang2_u - ang1
                    self.segments = self.state["segments"]
                    self.preview_pts = arc_points_world(center, radius, ang1, ang2_u, self.segments, X_arc, Y_arc)

        # =========================================================
        # 3-POINT ARC LOGIC
        # =========================================================
        elif self.mode == "3POINT":
            if self.stage == 1:
                target = snap_point
                
                # --- FIX: Project Stage 1 point onto the locked plane ---
                if self.Zp and self.pivot:
                    d = snap_point - self.pivot
                    d_plane = d - self.Zp * d.dot(self.Zp)
                    target = self.pivot + d_plane

                if self.constraint_axis and not event.alt:
                    if self.state.get("geometry_snap", False):
                        diff = target - self.pivot
                        proj = self.constraint_axis * diff.dot(self.constraint_axis)
                        target = self.pivot + proj
                    else:
                        region = context.region
                        rv3d = context.region_data
                        coord = (event.mouse_region_x, event.mouse_region_y)
                        ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
                        ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
                        res = geometry.intersect_line_line(ray_origin, ray_origin + ray_vector, self.pivot, self.pivot + self.constraint_axis)
                        if res: target = res[1]
                
                elif not self.state.get("geometry_snap", False) and not event.alt:
                    # AXIS INFERENCE SNAPPING
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
                
                self.current = target
                self.preview_pts = [self.p1, self.current]
                return

            elif self.stage == 2:
                # --- FIX: Stay on established drawing plane (self.Zp) ---
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
                            plane_n = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                        else:
                            plane_n = ax_x if self.vertical_override_axis == 'X' else ax_y
                    else:
                        # Case B: Standard orientation. Stabilize normal to face camera.
                        if plane_n.dot(view_fwd) > 0: plane_n = -plane_n
                    
                    self.Zp = plane_n

                # Project mouse point onto the plane defined by p1 and plane_n
                d_p3 = snap_point - self.p1
                d_plane = d_p3 - plane_n * d_p3.dot(plane_n)
                p3_proj = self.p1 + d_plane
                
                # Update current to the projected point for consistent HUD feedback
                self.current = p3_proj
                p1, p2 = self.p1, self.p2
                
                v1 = p2 - p1
                v2 = p3_proj - p1
                
                m1 = (p1 + p3_proj) / 2
                m2 = (p2 + p3_proj) / 2
                vec1 = p3_proj - p1
                vec2 = p2 - p3_proj
                
                if vec1.length < 1e-5 or vec2.length < 1e-5:
                    self.preview_pts = [p1, p3_proj, p2]
                    return

                n1 = plane_n.cross(vec1)
                n2 = plane_n.cross(vec2)
                
                # --- FIX: Check if intersection exists before unpacking ---
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
                    def to_local(v): return Vector((v.dot(X_arc), v.dot(Y_arc)))

                    v1_2d = to_local(p1 - center)
                    v2_2d = to_local(p2 - center)
                    v3_2d = to_local(p3_proj - center)
                    
                    ang1 = math.atan2(v1_2d.y, v1_2d.x)
                    ang2 = math.atan2(v2_2d.y, v2_2d.x)
                    ang3 = math.atan2(v3_2d.y, v3_2d.x)
                    
                    # --- ROBUST 3-POINT ARC WINDING ---
                    # Find the arc segment from ang1 to ang2 that contains ang3
                    two_pi = 2 * math.pi
                    da3 = (ang3 - ang1) % two_pi
                    da2 = (ang2 - ang1) % two_pi
                    
                    if da3 < da2:
                        # Path P1 -> P3 -> P2 is CCW
                        self.a0 = ang1
                        self.a1 = ang1 + da2
                    else:
                        # Path P1 -> P3 -> P2 is CW
                        self.a0 = ang1
                        self.a1 = ang1 - (two_pi - da2)

                    self.accum_angle = self.a1 - self.a0
                    self.segments = self.state["segments"]
                    self.preview_pts = arc_points_world(center, radius, self.a0, self.a1, self.segments, X_arc, Y_arc)
                    self.pivot = center
                    self.start = p1

    def refresh_preview(self):
        """Recalculates preview points based on current internal variables."""
        if self.mode == "1POINT":
            if self.stage == 2:
                self.preview_pts = arc_points_world(
                    self.pivot, self.radius, self.a0, self.a1, self.segments, self.Xp, self.Yp
                )
        elif self.mode == "2POINT":
            if self.stage == 2 and self.midpoint and self.start:
                # Re-calculate radius from current start (peak) position
                mid = self.midpoint
                chord_vec = self.p2 - self.p1
                chord_len = chord_vec.length
                
                if chord_len > 1e-6:
                    chord_dir = chord_vec.normalized()
                    plane_n = self.Zp if self.Zp else Vector((0,0,1))
                    perp_dir = plane_n.cross(chord_dir).normalized()
                    
                    mouse_vec = self.start - mid
                    height = mouse_vec.dot(perp_dir)
                    
                    if abs(height) < 0.0001:
                        self.preview_pts = [self.p1, self.p2]
                    else:
                        radius = (abs(height) / 2.0) + ((chord_len**2) / (8.0 * abs(height)))
                        self.radius = radius
                        to_center_dir = -perp_dir if height > 0 else perp_dir
                        center = self.start + to_center_dir * radius
                        
                        X_arc = chord_dir
                        Y_arc = perp_dir 
                        
                        def to_local(v): return Vector((v.dot(X_arc), v.dot(Y_arc)))
                        v1_2d = to_local(self.p1 - center)
                        v2_2d = to_local(self.p2 - center)
                        vp_2d = to_local(self.start - center)
                        
                        ang1 = math.atan2(v1_2d.y, v1_2d.x)
                        ang2 = math.atan2(v2_2d.y, v2_2d.x)
                        ang_mid = math.atan2(vp_2d.y, vp_2d.x)
                        
                        ang_mid_u, _ = unwrap(ang1, ang_mid, ang1)
                        ang2_u, _ = unwrap(ang_mid, ang2, ang_mid_u)
                        
                        self.a0 = ang1
                        self.a1 = ang2_u
                        self.accum_angle = ang2_u - ang1
                        self.segments = self.state["segments"]
                        self.preview_pts = arc_points_world(center, radius, ang1, ang2_u, self.segments, X_arc, Y_arc)

        # Sync to shared state
        self.state["preview_pts"] = self.preview_pts

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        # --- STAGE 0 CLICK (Common for all modes) ---
        if self.stage == 0:
            # Commit Pivot (Parent)
            self.pivot = snap_point
            self.p1 = snap_point 
            self.start = snap_point 
            
            # Common lock logic for 1-point and 3-point tools
            if self.mode in ["1POINT", "3POINT"]:
                self.state["locked"] = True
                self.state["locked_normal"] = self.Zp
            else:
                # 2-Point: Just record normal, don't hard lock yet
                self.state["locked"] = False
            
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage = 1
            return 'NEXT_STAGE'

        # --- STAGE 1 CLICK ---
        elif self.stage == 1 and self.mode == "1POINT":
            src = snap_point
            if self.Xp is not None and self.Zp is not None and self.pivot is not None:
                d = src - self.pivot
                d_plane = d - self.Zp * d.dot(self.Zp)
                self.start = self.pivot + d_plane
            else: 
                self.start = src
                
            self.stage = 2
            
            # Init Angle Drag
            rvec2 = world_to_plane(self.start - self.pivot, self.Xp, self.Yp)
            self.radius = rvec2.length
            if rvec2.length > 1e-9: rvec2.normalize()
            a0 = math.atan2(rvec2.y, rvec2.x)
            
            if self.state.get("use_angle_snap", True) and not self.state.get("geometry_snap", False):
                strength = self.state.get("snap_strength", 6.0)
                snap_angle_val = self.state.get("angle_increment", 15.0)
                a0 = snap_angle_soft(a0, snap_angle_val, strength)
                snapped_vec2 = Vector((math.cos(a0), math.sin(a0))) * self.radius
                self.start = self.pivot + plane_to_world(snapped_vec2, self.Xp, self.Yp)

            self.a0 = a0; self.a1 = a0; self.a_prev_raw = a0; self.accum_angle = 0.0
            self.preview_pts = arc_points_world(self.pivot, self.radius, a0, a0, self.segments, self.Xp, self.Yp)
            return 'NEXT_STAGE'

        # --- 2-POINT CLICKS ---
        elif self.stage == 1 and self.mode == "2POINT":
            # (Logic copied from previous implementation, mostly unchanged)
            target = snap_point
            if self.constraint_axis:
                if self.state.get("geometry_snap", False):
                    diff = target - self.pivot
                    proj = self.constraint_axis * diff.dot(self.constraint_axis)
                    target = self.pivot + proj
                else:
                    # Ray intersect
                    region = context.region
                    rv3d = context.region_data
                    coord = (event.mouse_region_x, event.mouse_region_y)
                    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
                    ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
                    res = geometry.intersect_line_line(ray_origin, ray_origin + ray_vector, self.pivot, self.pivot + self.constraint_axis)
                    if res: target = res[1]
            elif not self.state.get("geometry_snap", False):
                strength_deg = self.state.get("snap_strength", 6.0)
                strength_deg = max(0.1, min(89.0, strength_deg))
                axis_thresh = math.cos(math.radians(strength_deg))
                inf_loc, _, _ = get_axis_snapped_location(self.pivot, (event.mouse_region_x, event.mouse_region_y), context, snap_threshold=axis_thresh)
                if inf_loc: target = inf_loc

            self.p2 = target
            self.midpoint = (self.p1 + self.p2) * 0.5
            self.constraint_axis = None
            self.state["constraint_axis"] = None
            
            # Detect Vertical Chord for Plane Reset
            chord_vec = self.p2 - self.p1
            if chord_vec.length > 1e-6:
                chord_dir = chord_vec.normalized()
                if abs(chord_dir.dot(self.ref_normal)) > 0.99:
                    # Intelligent Vertical Flip
                    rv3d = context.region_data
                    view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    ax_x, ax_y, _ = orthonormal_basis_from_normal(self.ref_normal)
                    self.Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                    self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
                    self.state["locked_normal"] = self.Zp
                    self.state["locked"] = True
            self.stage = 2
            return 'NEXT_STAGE'

        elif self.stage == 1 and self.mode == "3POINT":
            # --- FIX: Finalize chord point (p2) on plane ---
            target = snap_point
            if self.Zp and self.pivot:
                d = target - self.pivot
                d_plane = d - self.Zp * d.dot(self.Zp)
                target = self.pivot + d_plane

            if self.constraint_axis:
                if self.state.get("geometry_snap", False):
                    diff = target - self.pivot
                    proj = self.constraint_axis * diff.dot(self.constraint_axis)
                    target = self.pivot + proj
                else:
                    region = context.region
                    rv3d = context.region_data
                    coord = (event.mouse_region_x, event.mouse_region_y)
                    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
                    ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
                    res = geometry.intersect_line_line(ray_origin, ray_origin + ray_vector, self.pivot, self.pivot + self.constraint_axis)
                    if res: target = res[1]
            elif not self.state.get("geometry_snap", False):
                # AXIS INFERENCE SNAPPING ON CLICK
                strength_deg = self.state.get("snap_strength", 6.0)
                strength_deg = max(0.1, min(89.0, strength_deg))
                axis_thresh = math.cos(math.radians(strength_deg))
                inf_loc, _, _ = get_axis_snapped_location(self.pivot, (event.mouse_region_x, event.mouse_region_y), context, snap_threshold=axis_thresh)
                if inf_loc: target = inf_loc
            
            self.p2 = target
            self.midpoint = (self.p1 + self.p2) * 0.5
            self.constraint_axis = None
            self.state["constraint_axis"] = None
            self.stage = 2
            return 'NEXT_STAGE'

        elif self.stage == 2:
            return 'FINISHED'
            
        return None

    def handle_input(self, context, event):
        # 1. Check Parent Input first (L Key)
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True

        # 2. Tool Specifics (P Key)
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            bridge = None
            
            if self.mode == "1POINT":
                if self.stage == 1 and self.current: bridge = self.current - self.pivot
                elif self.stage == 2 and self.start: bridge = self.start - self.pivot
            elif self.mode in ["2POINT", "3POINT"]:
                if self.stage == 1 and self.current: bridge = self.current - self.p1
                elif self.stage == 2 and self.p2: bridge = self.p2 - self.p1
            
            if bridge and bridge.length_squared > 1e-6:
                # Vertical check logic...
                b_vec = bridge.normalized()
                floor_n = self.ref_normal
                is_vertical = abs(b_vec.dot(floor_n)) > 0.99
                
                if is_vertical and self.mode in ["2POINT", "3POINT"] and self.stage == 2:
                    # Switch to manual toggle for vertical state
                    if self.vertical_override_axis == 'X':
                        self.vertical_override_axis = 'Y'
                    else:
                        self.vertical_override_axis = 'X'
                    
                    rv3d = context.region_data
                    ax_x, ax_y, _ = orthonormal_basis_from_normal(self.ref_normal)
                    self.Zp = ax_x if self.vertical_override_axis == 'X' else ax_y
                    self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
                    self.state["locked_normal"] = self.Zp
                    self.state["locked"] = True
                    self.core.report({'INFO'}, f"Vertical Orientation: {self.vertical_override_axis}")
                    return True

                self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
                
                if self.state["is_perpendicular"]:
                    b_vec = bridge.normalized()
                    floor_n = self.ref_normal
                    new_Zp = b_vec.cross(floor_n).normalized()
                    new_Yp = floor_n 
                    new_Xp = new_Yp.cross(new_Zp).normalized() 
                    self.Xp, self.Yp, self.Zp = new_Xp, new_Yp, new_Zp
                    self.state["locked_normal"] = new_Zp
                else:
                    # Reset to generic
                    floor_n = self.ref_normal
                    self.Xp, self.Yp, self.Zp = orthonormal_basis_from_normal(floor_n)
                    self.state["locked_normal"] = floor_n

                # --- FIX: Re-tether 1-Point Arc after Plane Toggle ---
                if self.mode == "1POINT" and self.stage > 0:
                    region = context.region
                    rv3d = context.region_data
                    coord = (event.mouse_region_x, event.mouse_region_y)
                    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
                    ray_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
                    
                    hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.Zp)
                    if hit:
                        if self.stage == 1:
                            # Stage 1: Radius Drag. Line should point at mouse.
                            self.current = hit
                            rvec2 = world_to_plane(self.current - self.pivot, self.Xp, self.Yp)
                            self.radius = rvec2.length
                        elif self.stage == 2:
                            # Stage 2: Angle Drag. Sweep should point at mouse.
                            # First, project the OLD 'start' point to the new plane to keep radius/a0 stable
                            d_start = self.start - self.pivot
                            d_plane = d_start - self.Zp * d_start.dot(self.Zp)
                            self.start = self.pivot + d_plane
                            
                            rvec_start = world_to_plane(self.start - self.pivot, self.Xp, self.Yp)
                            self.radius = rvec_start.length
                            self.a0 = math.atan2(rvec_start.y, rvec_start.x)
                            
                            # Now, find the angle of the mouse on the new plane
                            rvec_mouse = world_to_plane(hit - self.pivot, self.Xp, self.Yp)
                            target_angle = math.atan2(rvec_mouse.y, rvec_mouse.x)
                            
                            # Update accumulation so the sweep matches the mouse
                            diff, a_prev = unwrap(self.a0, target_angle, 0.0)
                            self.accum_angle = diff
                            self.a1 = self.a0 + diff
                            self.a_prev_raw = a_prev
                        
                        self.refresh_preview()
                return True

        # 3. Axis Locking (X/Y/Z)
        if event.type in {'X', 'Y', 'Z'} and event.value == 'PRESS':
            # --- FIX: Only allow X/Y/Z plane locking BEFORE drawing starts (Stage 0) ---
            if self.stage > 0: 
                # --- NEW: Allow X/Y toggle for vertical chords during drawing ---
                if event.type in {'X', 'Y'}:
                    if self.p1 and self.p2:
                        chord_vec = self.p2 - self.p1
                        if chord_vec.length > 1e-6:
                            if abs(chord_vec.normalized().dot(self.ref_normal)) > 0.99:
                                self.vertical_override_axis = event.type
                                rv3d = context.region_data
                                ax_x, ax_y, _ = orthonormal_basis_from_normal(self.ref_normal)
                                self.Zp = ax_x if self.vertical_override_axis == 'X' else ax_y
                                self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
                                self.state["locked_normal"] = self.Zp
                                self.state["locked"] = True
                                self.core.report({'INFO'}, f"Locked to {event.type}-Facing Orientation")
                                return True
                return False

            axes = {'X': Vector((1, 0, 0)), 'Y': Vector((0, 1, 0)), 'Z': Vector((0, 0, 1))}
            
            # FOR 1-POINT: Switch to Locking the PLANE Normal (X/Y/Z)
            if self.mode == "1POINT":
                new_n = axes[event.type]
                
                # Check if already locked to this exact normal (Toggle Logic)
                current_locked = self.state.get("locked")
                current_normal = self.state.get("locked_normal")
                
                is_same_axis = False
                if current_locked and current_normal:
                    if abs(current_normal.dot(new_n)) > 0.99:
                        is_same_axis = True

                if is_same_axis:
                    # UNLOCK
                    self.state["locked"] = False
                    self.state["locked_normal"] = None
                    self.state["locked_plane_point"] = None
                    self.core.report({'INFO'}, f"Unlocked {event.type}-Plane")
                else:
                    # LOCK
                    self.Zp = new_n
                    self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)
                    self.state["locked"] = True
                    self.state["locked_normal"] = self.Zp
                    
                    # --- FIX: Set plane point to where the compass is RIGHT NOW ---
                    target_point = self.current if self.current else Vector((0,0,0))
                    self.state["locked_plane_point"] = target_point
                    
                    # --- FIX: Re-map existing geometry to the new plane ---
                    if self.pivot:
                        if self.start:
                            # Project start point onto new plane
                            d_start = self.start - self.pivot
                            d_plane = d_start - self.Zp * d_start.dot(self.Zp)
                            self.start = self.pivot + d_plane
                            
                            # Recalculate local basis angles
                            rvec2 = world_to_plane(self.start - self.pivot, self.Xp, self.Yp)
                            self.radius = rvec2.length
                            self.a0 = math.atan2(rvec2.y, rvec2.x)
                            self.a1 = self.a0 + self.accum_angle
                            self.a_prev_raw = self.a0
                    
                    # Clear linear constraint if any, to avoid confusion
                    self.constraint_axis = None
                    self.state["constraint_axis"] = None
                    
                    self.core.report({'INFO'}, f"Locked to {event.type}-Plane")
                return True

            # FOR 2/3-POINT: Keep existing Linear Axis Constraint logic
            new_axis = axes[event.type]
            if self.constraint_axis == new_axis:
                self.constraint_axis = None
                self.state["constraint_axis"] = None
            else:
                self.constraint_axis = new_axis
                self.state["constraint_axis"] = new_axis
            return True

        return False

class ArcTool_1Point(ArcTool_Common):
    def __init__(self, core): super().__init__(core, "1POINT")

class ArcTool_2Point(ArcTool_Common):
    def __init__(self, core): super().__init__(core, "2POINT")

class ArcTool_3Point(ArcTool_Common):
    def __init__(self, core): super().__init__(core, "3POINT")