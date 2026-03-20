import math
from mathutils import Vector, geometry
from ..inference_utils import get_axis_snapped_location
from .base_tool import SurfaceDrawTool 

def safe_lerp(p_a, p_b, t, t_a, t_b):
    """Safely interpolates between p_a and p_b."""
    if abs(t_b - t_a) < 1e-6:
        return p_a
    return (t_b - t) / (t_b - t_a) * p_a + (t - t_a) / (t_b - t_a) * p_b

def evaluate_catmull_rom(p0, p1, p2, p3, t_normalized):
    """Directly evaluates a Catmull-Rom point for a single segment p1-p2."""
    alpha = 0.5
    def get_t(t, pa, pb): return t + pow((pb - pa).length_squared, alpha * 0.5)
    
    t0 = 0.0
    t1 = get_t(t0, p0, p1)
    t2 = get_t(t1, p1, p2)
    t3 = get_t(t2, p2, p3)
    
    t = t1 + (t2 - t1) * t_normalized
    A1 = safe_lerp(p0, p1, t, t0, t1)
    A2 = safe_lerp(p1, p2, t, t1, t2)
    A3 = safe_lerp(p2, p3, t, t2, t3)
    B1 = safe_lerp(A1, A2, t, t0, t2)
    B2 = safe_lerp(A2, A3, t, t1, t3)
    return safe_lerp(B1, B2, t, t1, t2)

def solve_catmull_rom_chain(points, num_segments=16, cached_dists=None):
    """
    Highly efficient uniform curve solver.
    Directly evaluates requested segments using chord-length distances.
    """
    if len(points) < 2: return points
    if len(points) == 2:
        return [points[0].lerp(points[1], i/num_segments) for i in range(num_segments + 1)]

    # 1. Calculate cumulative straight-line distances between control points
    # Use cached version if available (O(1) vs O(N))
    if cached_dists and len(cached_dists) == len(points):
        dists = cached_dists
        total_len = dists[-1]
    else:
        dists = [0.0]
        total_len = 0.0
        for i in range(len(points) - 1):
            total_len += (points[i+1] - points[i]).length
            dists.append(total_len)
    
    if total_len < 1e-6: return [points[0], points[-1]]

    # 2. Add extrapolated ghost boundary points for Catmull-Rom.
    # Duplicate endpoints collapse centripetal parameterization (t0==t1==0),
    # causing point bunching at start/end. Reflected ghosts fix that.
    ghost_start = points[0] + (points[0] - points[1])
    ghost_end   = points[-1] + (points[-1] - points[-2])
    chain = [ghost_start] + points + [ghost_end]
    
    # 3. Sample exactly 'num_segments' points along the curve
    final_pts = []
    # Optimization: pre-calculate step
    step = total_len / num_segments
    
    idx = 0
    for i in range(num_segments):
        target_d = step * i
        
        # Find which segment this distance falls into
        # Optimization: start search from previous index (O(1) average case)
        while idx < len(dists) - 2 and dists[idx+1] < target_d:
            idx += 1
            
        # Local 't' within the segment [idx, idx+1]
        d0, d1 = dists[idx], dists[idx+1]
        local_t = (target_d - d0) / (d1 - d0) if (d1 - d0) > 1e-8 else 0.0
        
        # Evaluate spline using P[idx-1], P[idx], P[idx+1], P[idx+2]
        # (Offset by 1 because of the boundary points added to 'chain')
        p0, p1, p2, p3 = chain[idx:idx+4]
        final_pts.append(evaluate_catmull_rom(p0, p1, p2, p3, local_t))
        
    final_pts.append(points[-1])
    return final_pts

class CurveTool_Interpolate(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "CURVE_INTERPOLATE"
        self.chains = [[]]        # each Ctrl+click starts a new chain = sharp kink
        self.chain_dists = [[0.0]]
        self.current = None
        self.constraint_axis = None

    @property
    def control_points(self):
        return self.chains[-1]

    @property
    def dists(self):
        return self.chain_dists[-1]

    def _build_all_preview(self, extra_pt=None, num_segs=12):
        """Stitch all chains into one preview, distributing segments by chord length."""
        pts_list  = [chain[:] for chain in self.chains]
        dists_list = [d[:]    for d in self.chain_dists]

        # Tack the live mouse point onto the last chain — but only if it's
        # meaningfully far from the last anchor. Right after a click, self.current
        # is still on the clicked point; adding it as a near-duplicate collapses
        # the ghost-end extrapolation and bunches verts.
        if extra_pt is not None and pts_list[-1]:
            seg_len = (extra_pt - pts_list[-1][-1]).length
            if seg_len > 1e-4:
                dists_list[-1] = dists_list[-1] + [dists_list[-1][-1] + seg_len]
                pts_list[-1]   = pts_list[-1]   + [extra_pt]

        total_len = sum(d[-1] for d in dists_list if d)
        if total_len < 1e-6:
            return [p for chain in pts_list for p in chain]

        all_pts = []
        for i, (pts, dists) in enumerate(zip(pts_list, dists_list)):
            if len(pts) < 2:
                if pts and not all_pts:
                    all_pts.append(pts[0])
                continue
            chain_segs = max(2, round(num_segs * dists[-1] / total_len))
            solved = solve_catmull_rom_chain(pts, num_segments=chain_segs, cached_dists=dists)
            # Skip the first point of every chain after the first — it's the kink point
            # which is already the last point of the previous chain
            all_pts.extend(solved if not all_pts else solved[1:])

        return all_pts

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            self.current = snap_point
            self.preview_pts = []
            return

        if self.stage >= 1:
            ref_point = self.chains[-1][-1] if self.chains[-1] else self.pivot
            target = snap_point

            if self.constraint_axis:
                if self.state.get("geometry_snap", False):
                    diff = target - ref_point
                    proj = self.constraint_axis * diff.dot(self.constraint_axis)
                    target = ref_point + proj
                else:
                    from bpy_extras import view3d_utils
                    ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, (event.mouse_region_x, event.mouse_region_y))
                    ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, (event.mouse_region_x, event.mouse_region_y))
                    res = geometry.intersect_line_line(ray_origin, ray_origin + ray_vector, ref_point, ref_point + self.constraint_axis)
                    if res: target = res[1]
            elif not self.state.get("geometry_snap", False):
                axis_thresh = math.cos(math.radians(self.state.get("snap_strength", 6.0)))
                inf_loc, _, _ = get_axis_snapped_location(ref_point, (event.mouse_region_x, event.mouse_region_y), context, snap_threshold=axis_thresh)
                if inf_loc: target = inf_loc

            self.current = target
            num_segs = self.state.get("segments", 12)
            self.preview_pts = self._build_all_preview(extra_pt=self.current, num_segs=num_segs)

    def refresh_preview(self):
        if self.stage >= 1 and self.current:
            num_segs = self.state.get("segments", 12)
            self.preview_pts = self._build_all_preview(extra_pt=self.current, num_segs=num_segs)

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.chains = [[snap_point]]
            self.chain_dists = [[0.0]]
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage >= 1:
            if (self.current - self.chains[-1][-1]).length < 1e-5: return None
            # Commit current point to active chain
            seg_len = (self.current - self.chains[-1][-1]).length
            self.chain_dists[-1].append(self.chain_dists[-1][-1] + seg_len)
            self.chains[-1].append(self.current.copy())

            if event.ctrl:
                # Kink! Seal this chain and start a fresh one from the same point
                self.chains.append([self.current.copy()])
                self.chain_dists.append([0.0])

            self.constraint_axis = self.state["constraint_axis"] = None
            self.pivot = self.current
            return 'NEXT_STAGE'

        return None

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event): return True
        if event.type in {'X', 'Y', 'Z'} and event.value == 'PRESS':
            axes = {'X': Vector((1, 0, 0)), 'Y': Vector((0, 1, 0)), 'Z': Vector((0, 0, 1))}
            self.constraint_axis = self.state["constraint_axis"] = None if self.constraint_axis == axes[event.type] else axes[event.type]
            return True
        if event.type == 'BACK_SPACE' and event.value == 'PRESS':
            chain = self.chains[-1]
            dists = self.chain_dists[-1]
            if len(chain) > 1:
                # Normal undo: pop last point
                chain.pop()
                dists.pop()
                self.pivot = chain[-1]
                self.refresh_preview()
                return True
            elif len(self.chains) > 1:
                # Undo the kink: ditch the fresh chain, pop kink point from previous chain
                self.chains.pop()
                self.chain_dists.pop()
                if len(self.chains[-1]) > 1:
                    self.chains[-1].pop()
                    self.chain_dists[-1].pop()
                self.pivot = self.chains[-1][-1]
                self.refresh_preview()
                return True
        return False

class CurveTool_Freehand(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "CURVE_FREEHAND"
        self.points = []
        self.current = None
        self.last_collect_pos = None
        self.is_drawing = False
        self.min_dist = 0.05 # Optimal for direct solving
        self.dists = [0.0]

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            self.current = snap_point
            self.preview_pts = []
            return

        if self.stage == 1 and self.is_drawing:
            target = snap_point
            if self.Zp and self.pivot:
                from bpy_extras import view3d_utils
                coord = (event.mouse_region_x, event.mouse_region_y)
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.Zp)
                if hit: target = hit
            
            self.current = target
            
            if self.last_collect_pos is None or (target - self.last_collect_pos).length > self.min_dist:
                # Update distance cache
                if self.points:
                    seg_len = (target - self.points[-1]).length
                    self.dists.append(self.dists[-1] + seg_len)
                
                self.points.append(target.copy())
                self.last_collect_pos = target.copy()
            
            # DIRECT SOLVE: Perfectly smooth, perfectly fast
            num_segs = self.state.get("segments", 12)
            self.preview_pts = solve_catmull_rom_chain(self.points, num_segments=num_segs, cached_dists=self.dists)
            self.state["preview_pts"] = self.preview_pts

        if self.stage == 2:
            self.current = snap_point

    def refresh_preview(self):
        if len(self.points) > 1:
            num_segs = self.state.get("segments", 12)
            self.preview_pts = solve_catmull_rom_chain(self.points, num_segments=num_segs, cached_dists=self.dists)
            self.state["preview_pts"] = self.preview_pts

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if event.value != 'PRESS': return None
        
        # Safety: If snap_point is None (skipped for performance), use the calculated current position
        target = snap_point if snap_point is not None else self.current
        if target is None: return None

        if self.stage == 0:
            self.pivot = target
            self.points = [target.copy()]
            self.dists = [0.0]
            self.last_collect_pos = target.copy()
            self.is_drawing = True
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            self.stage = 1
            return 'NEXT_STAGE'
        if self.stage == 1:
            self.is_drawing = False
            if (target - self.points[-1]).length > 1e-5:
                seg_len = (target - self.points[-1]).length
                self.dists.append(self.dists[-1] + seg_len)
                self.points.append(target.copy())
            self.refresh_preview()
            self.stage = 2
            return 'NEXT_STAGE'
        if self.stage == 2: return 'FINISHED'
        return None

    def handle_input(self, context, event):
        return super().handle_plane_lock_input(context, event)
