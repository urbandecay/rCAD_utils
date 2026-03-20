import bpy
import bmesh
import math
from mathutils import Vector
from ..modal_state import state
from ..modal_core import begin_modal, modal_arc_common
from ..orientation_utils import orthonormal_basis_from_normal
from .circle_tools import get_selected_edge_chains, CatmullRomSpline, solve_medial_axis_point

class CircleTool_TanTan:
    def __init__(self, manager):
        self.manager = manager
        self.stage = 0
        self.pivot = None
        self.current = None
        self.segments = 32
        self.preview_pts = []
        self.splines = []
        self.spline_samples = [[], []]
        self.last_mouse = None  # NEW: Performance Tracker
        self.Xp, self.Yp, self.Zp = Vector((1,0,0)), Vector((0,1,0)), Vector((0,0,1))
        
        # STATE: Rail is gone, Lines/Dots remain
        state.update({
            "stage": 0,
            "preview_pts": [],
            "visual_pts": [],
            "tan_solutions": [],
            "solution_index": 0,
            "choosing_solution": False,
            "tan_solution_active": False,
            "radius": 1.0,
            "tangent_rail": [],     # EMPTY
            "viz_tangent_line": [], # KEEP
            "viz_diameter_line": [],# KEEP
            "viz_opposite_dot": []  # KEEP
        })
        
        # Initialize containers for the specific splines
        self.spline_1 = None
        self.spline_2 = None
        
        ctx = bpy.context
        
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
                    
                    state["locked_normal"] = self.Zp
                    state["locked"] = True
                    state.update({"Xp": self.Xp, "Yp": self.Yp})

                    # --- POPULATE CATMULL CURVE OVERLAYS ---
                    catmull_previews = []
                    for spline in [self.spline_1, self.spline_2]:
                        curve_pts = []
                        for seg in spline.segments:
                            # Sample each segment
                            curve_pts.extend([seg.eval(seg.t_start + t*seg.dt) for t in [0.0, 0.25, 0.5, 0.75]])
                        # Add very last point
                        if spline.segments:
                            curve_pts.append(spline.segments[-1].eval(spline.segments[-1].t_end))
                        catmull_previews.append(curve_pts)
                    state["catmull_spline_previews"] = catmull_previews

                    # --- NO RAIL GENERATION ---
                    # Replaced heavy loop with simple visual curve smoothing
                    self.state["smooth_curve_1"] = [self.spline_1.segments[i].eval(self.spline_1.segments[i].t_start + t*self.spline_1.segments[i].dt) for i in range(len(self.spline_1.segments)) for t in [0.0, 0.33, 0.66]]
                    self.state["smooth_curve_2"] = [self.spline_2.segments[i].eval(self.spline_2.segments[i].t_start + t*self.spline_2.segments[i].dt) for i in range(len(self.spline_2.segments)) for t in [0.0, 0.33, 0.66]]

                    self.stage = 1
                    
                    # Simple default start point (Instantly sets pivot to start of curve 1)
                    self.pivot = raw_pts1[0]
                    self.current = raw_pts1[0]
                    state["pivot"] = self.pivot
            
            elif len(chains) == 1:
                # Fallback for single curve
                raw_pts1, closed1 = chains[0]
                self.spline_1 = CatmullRomSpline(raw_pts1, is_closed=closed1)
                if self.spline_1.segments:
                    self.has_preselection = True
                    self.state["smooth_curve_1"] = [self.spline_1.segments[i].eval(self.spline_1.segments[i].t_start + t*self.spline_1.segments[i].dt) for i in range(len(self.spline_1.segments)) for t in [0.0, 0.33, 0.66]]
                    self.state["smooth_curve_2"] = []
                    self.pivot = raw_pts1[0]
                    state["pivot"] = self.pivot

    def update(self, context, event, snap_pt, snap_normal):
        # Update segments from global state (mouse wheel support)
        new_segs = state.get("segments", 32)
        
        # --- PERFORMANCE CHECK: Skip Solve if Mouse hasn't moved & Segments haven't changed ---
        mouse_moved = (self.last_mouse is None or (snap_pt - self.last_mouse).length > 1e-4)
        segs_changed = (self.segments != new_segs)
        
        if not mouse_moved and not segs_changed:
            return # Skip all work
            
        self.segments = new_segs
        
        # Stage 0: Fallback
        if self.stage == 0:
            self.current = snap_pt
            self.last_mouse = snap_pt.copy()
            return

        # Stage 1: Live Solver
        if self.stage == 1:
            # Early exit if splines aren't loaded
            if not self.spline_1 or not self.spline_2:
                return

            if mouse_moved:
                self.last_mouse = snap_pt.copy()
                mouse_pos = snap_pt
                if self.pivot is None: self.pivot = mouse_pos

                d = mouse_pos - self.pivot
                d_plane = d - self.Zp * d.dot(self.Zp)
                mouse_on_plane = self.pivot + d_plane

                # 1. Run Solver (The heavy part) - Only on mouse move
                solved_c, solved_r = solve_medial_axis_point(mouse_on_plane, self.spline_1, self.spline_2, self.Zp)

                self.current = solved_c
                self.radius = solved_r
                self.pivot = solved_c

                # 2. Visualization of the tangency
                p1_v, _, _ = self.spline_1.project(self.current)
                p2_v, _, _ = self.spline_2.project(self.current)
                state["viz_tangent_line"] = (self.current, p1_v)
                state["viz_diameter_line"] = (self.current, p2_v)
                state["viz_opposite_dot"] = None

                # 3. Update Visual Smooth Circle (The high-res blue one) - Only on mouse move
                if self.radius > 1e-5:
                    state["visual_pts"] = [self.current + self.Xp*math.cos(a)*self.radius + self.Yp*math.sin(a)*self.radius for a in [i*math.pi*2/128 for i in range(129)]]
                else:
                    state["visual_pts"] = []

            # 4. Update Mesh Preview (The black one) - On mouse move OR segment change
            if self.radius > 1e-5:
                self.preview_pts = [self.current + self.Xp*math.cos(a)*self.radius + self.Yp*math.sin(a)*self.radius for a in [i*math.pi*2/self.segments for i in range(self.segments+1)]]
                state["preview_pts"] = self.preview_pts
                state["tan_solution_active"] = True
            else:
                self.preview_pts = []
                state["preview_pts"] = []
                state["tan_solution_active"] = False

    def handle_click(self, context, event, snap_pt, snap_normal, button_id=None):
        if self.stage == 0:
            state["locked"] = False
            self.pivot = snap_pt
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage == 1:
            return 'FINISHED'
        return None

    def handle_input(self, context, event):
        return False

class VIEW3D_OT_circle_tan_tan(bpy.types.Operator):
    bl_idname = "view3d.radcad_circle_tan_tan"
    bl_label = "Circle Tan Tan"
    bl_options = {'REGISTER', 'UNDO', 'BLOCKING'}

    def invoke(self, ctx, ev):
        state["tool_mode"] = "CIRCLE_TAN_TAN"
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)