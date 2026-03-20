import math
import bpy
from mathutils import Vector, geometry
from .base_tool import SurfaceDrawTool
from ..plane_utils import plane_to_world
from ..orientation_utils import orthonormal_basis_from_normal
from ..inference_utils import get_axis_snapped_location
from bpy_extras import view3d_utils

def ellipse_points_world(center, rx, ry, segments, Xp, Yp):
    """
    Generates points for a full ellipse in 3D space.
    """
    pts = []
    if segments < 3: return pts
    
    # Pre-calculate angle step
    step = (2 * math.pi) / segments
    
    for i in range(segments + 1):
        angle = i * step
        # Ellipse parametric equation
        vx = math.cos(angle) * rx
        vy = math.sin(angle) * ry
        
        # Map to 3D World Space
        pt = center + (Xp * vx) + (Yp * vy)
        pts.append(pt)
        
    return pts

class EllipseTool_FromRadius(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "ELLIPSE_RADIUS"
        self.segments = self.state["segments"]
        self.rx = 0.0 
        self.ry = 0.0 
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None
        self.major_axis = Vector((1,0,0))

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        coord = (event.mouse_region_x, event.mouse_region_y)
        if self.stage == 1:
            # CIRCLE LOGIC: Direct Project to Floor
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            target = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.ref_normal)
            if target is None: target = snap_point 

            # Snapping
            strength_deg = self.state.get("snap_strength", 6.0)
            axis_thresh = math.cos(math.radians(strength_deg))
            inf_loc, _, _ = get_axis_snapped_location(self.pivot, coord, context, snap_threshold=axis_thresh)
            if inf_loc and not event.alt: target = inf_loc

            # CIRCLE LOGIC: Screen-Space Anchor Basis
            bridge = target - self.pivot
            if bridge.length_squared > 1e-8:
                b_vec = bridge.normalized()
                self.Xp = b_vec # Update Xp immediately for preview
                up = self.ref_normal
                is_perp = self.state.get("is_perpendicular", False)
                is_vertical = abs(b_vec.dot(up)) > (0.98 if getattr(self, "_is_vert_last", False) else 0.995)
                self._is_vert_last = is_vertical

                reg, rv3d = context.region, context.region_data
                p2d = view3d_utils.location_3d_to_region_2d(reg, rv3d, self.pivot)
                
                if p2d:
                    m2d = Vector(coord)
                    if (m2d - p2d).length_squared > 1:
                        if is_perp or is_vertical:
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
                
                self.major_axis = self.Xp.copy()

            self.rx = (target - self.pivot).length
            self.current = target
            # Display full diameter symmetrically from the center (pivot)
            self.preview_pts = [self.pivot - (self.Xp * self.rx), self.pivot + (self.Xp * self.rx)]

        if self.stage == 2:
            center = self.pivot
            target_pt = snap_point
            if not self.state.get("geometry_snap", False):
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, center, self.Zp)
                if hit: target_pt = hit

            d = target_pt - center
            dist_y = d.dot(self.Yp)
            self.ry = abs(dist_y)
            self.current = center + (self.Yp * dist_y)
            self.segments = self.state["segments"]
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"], self.state["locked_normal"] = True, self.ref_normal
            if self.stage == 2: self._update_basis_locked()
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            if self.stage == 2: self._update_basis_locked()
            return True
        return False

    def _update_basis_locked(self):
        up = self.ref_normal
        is_perp = self.state.get("is_perpendicular")
        b_vec = self.major_axis
        is_vertical = abs(b_vec.dot(up)) > (0.98 if getattr(self, "_is_vert_last", False) else 0.995)
        self._is_vert_last = is_vertical

        if is_perp or is_vertical:
            if is_vertical:
                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                if self.vertical_override_axis == 'X': new_Zp = ax_x
                elif self.vertical_override_axis == 'Y': new_Zp = ax_y
                else:
                    rv3d = bpy.context.region_data
                    view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    new_Zp = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                self.Zp = new_Zp
                self.Yp = up.normalized()
                self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Xp = b_vec
                self.Zp = self.Xp.cross(up).normalized()
                rv3d = bpy.context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                if self.Zp.dot(view_fwd) > 0: self.Zp = -self.Zp
                self.Yp = self.Zp.cross(self.Xp).normalized()
        else:
            self.Zp = up.copy()
            self.Xp = b_vec
            self.Yp = self.Zp.cross(self.Xp).normalized()
        self.state["locked_normal"] = self.Zp

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage = 1
            return 'NEXT_STAGE'
        if self.stage == 1:
            if self.rx < 1e-6: return None
            self.stage = 2
            return 'NEXT_STAGE'
        if self.stage == 2: return 'FINISHED'
        return None

    def refresh_preview(self):
        if self.stage == 1:
            self.stage = 2 
            self.current = self.pivot + (self.Xp * self.rx)
            # Symmetric diameter preview
            self.preview_pts = [self.pivot - (self.Xp * self.rx), self.pivot + (self.Xp * self.rx)]
        elif self.stage >= 2:
            center = self.pivot
            # Update current to reflect the minor radius point
            self.current = center + (self.Yp * self.ry)
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

class EllipseTool_FociPoint(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "ELLIPSE_FOCI"
        self.segments = self.state["segments"]
        self.f1 = None 
        self.f2 = None
        self.rx = 0.0 
        self.ry = 0.0 
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None
        self.major_axis = Vector((1,0,0))

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        coord = (event.mouse_region_x, event.mouse_region_y)
        if self.stage == 1:
            self.f1 = self.pivot
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            target = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.f1, self.ref_normal)
            if target is None: target = snap_point 

            strength_deg = self.state.get("snap_strength", 6.0)
            axis_thresh = math.cos(math.radians(strength_deg))
            inf_loc, _, _ = get_axis_snapped_location(self.f1, coord, context, snap_threshold=axis_thresh)
            if inf_loc and not event.alt: target = inf_loc

            bridge = target - self.f1
            if bridge.length_squared > 1e-8:
                b_vec = bridge.normalized()
                up = self.ref_normal
                is_perp = self.state.get("is_perpendicular", False)
                is_vertical = abs(b_vec.dot(up)) > (0.98 if getattr(self, "_is_vert_last", False) else 0.995)
                self._is_vert_last = is_vertical

                reg, rv3d = context.region, context.region_data
                p2d = view3d_utils.location_3d_to_region_2d(reg, rv3d, self.f1)
                
                if p2d:
                    if is_perp or is_vertical:
                        if is_vertical:
                            self.Xp = b_vec
                            ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                            view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                            self.Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                            if self.Zp.dot(view_dir) > 0: self.Zp = -self.Zp
                            self.Yp = self.Zp.cross(self.Xp).normalized()
                        else:
                            self.Xp = b_vec
                            self.Zp = self.Xp.cross(up).normalized()
                            view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                            if self.Zp.dot(view_dir) > 0: self.Zp = -self.Zp
                            self.Yp = self.Zp.cross(self.Xp).normalized()
                    else:
                        self.Zp = up.copy()
                        self.Xp = b_vec
                        self.Yp = self.Zp.cross(self.Xp).normalized()
                
                self.major_axis = self.Xp.copy()

            self.current = target
            self.preview_pts = [self.f1, self.current]

        if self.stage == 2:
            self.f1 = self.pivot
            self.state["f1"], self.state["f2"] = self.f1, self.f2

            target_pt = snap_point
            if not self.state.get("geometry_snap", False):
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.f1, self.Zp)
                if hit: target_pt = hit

            d_raw = target_pt - self.f1
            d_plane = d_raw - self.Zp * d_raw.dot(self.Zp)
            P = self.f1 + d_plane
            
            center = (self.f1 + self.f2) * 0.5
            c = (self.f2 - self.f1).length * 0.5
            dist_sum = (P - self.f1).length + (P - self.f2).length
            a = dist_sum * 0.5
            if a < c + 1e-6: a = c + 1e-6
            b = math.sqrt(a**2 - c**2)
            self.rx, self.ry = a, b
            
            rel = P - center
            lx = rel.dot(self.Xp); ly = rel.dot(self.Yp)
            angle = math.atan2(ly, lx)
            P_ellipse = center + (self.Xp * (self.rx * math.cos(angle))) + (self.Yp * (self.ry * math.sin(angle)))
            self.current = P_ellipse
            self.state["current"] = P_ellipse 
            
            self.segments = self.state["segments"]
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"], self.state["locked_normal"] = True, self.ref_normal
            if self.stage == 2: self._update_basis_locked()
            return True
        if event.type == 'K' and event.value == 'PRESS':
            self.state["keep_foci"] = not self.state.get("keep_foci", False)
            status = "ON" if self.state["keep_foci"] else "OFF"
            self.core.report({'INFO'}, f"Keep Foci: {status}")
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            if self.stage == 2: self._update_basis_locked()
            return True
        return False

    def _update_basis_locked(self):
        up = self.ref_normal
        is_perp = self.state.get("is_perpendicular")
        b_vec = self.major_axis
        is_vertical = abs(b_vec.dot(up)) > 0.995

        if is_perp or is_vertical:
            if is_vertical:
                self.Xp = b_vec
                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                if self.vertical_override_axis == 'X': new_Zp = ax_x
                elif self.vertical_override_axis == 'Y': new_Zp = ax_y
                else:
                    rv3d = bpy.context.region_data
                    view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    new_Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                self.Zp = new_Zp
                self.Yp = up.normalized()
                self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Xp = b_vec
                self.Zp = self.Xp.cross(up).normalized()
                rv3d = bpy.context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                if self.Zp.dot(view_fwd) > 0: self.Zp = -self.Zp
                self.Yp = self.Zp.cross(self.Xp).normalized()
        else:
            self.Zp = up.copy()
            self.Xp = b_vec
            self.Yp = self.Zp.cross(self.Xp).normalized()
        self.state["locked_normal"] = self.Zp

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage = 1
            return 'NEXT_STAGE'
        if self.stage == 1:
            self.f2 = self.current if self.current else snap_point
            if (self.f2 - self.pivot).length < 1e-6: return None
            self.stage = 2
            return 'NEXT_STAGE'
        if self.stage == 2: return 'FINISHED'
        return None

    def refresh_preview(self):
        if self.stage == 1:
            self.f1 = self.pivot
            self.f2 = self.current if self.current else self.f1
            self.stage = 2 
            self.preview_pts = [self.f1, self.f2]
        elif self.stage >= 2:
            if self.f1 is None or self.f2 is None or self.ry is None:
                self.preview_pts = []
                return
            
            center = (self.f1 + self.f2) * 0.5
            c = (self.f2 - self.f1).length * 0.5
            
            # ry is already set from input. We need rx (a).
            # b^2 = a^2 - c^2  => a = sqrt(b^2 + c^2)
            b = self.ry
            a = math.sqrt(b**2 + c**2)
            self.rx = a
            
            # Update current to reflect the minor radius point
            self.current = center + (self.Yp * b)
            
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

class EllipseTool_FromEndpoints(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "ELLIPSE_ENDPOINTS"
        self.segments = self.state["segments"]
        self.p1 = None 
        self.p2 = None
        self.rx = 0.0 
        self.ry = 0.0 
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None
        self.major_axis = Vector((1,0,0))

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        coord = (event.mouse_region_x, event.mouse_region_y)
        if self.stage == 1:
            self.p1 = self.pivot
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            target = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.p1, self.ref_normal)
            if target is None: target = snap_point 

            strength_deg = self.state.get("snap_strength", 6.0)
            axis_thresh = math.cos(math.radians(strength_deg))
            inf_loc, _, _ = get_axis_snapped_location(self.p1, coord, context, snap_threshold=axis_thresh)
            if inf_loc and not event.alt: target = inf_loc

            bridge = target - self.p1
            if bridge.length_squared > 1e-8:
                b_vec = bridge.normalized()
                up = self.ref_normal
                is_perp = self.state.get("is_perpendicular", False)
                is_vertical = abs(b_vec.dot(up)) > (0.98 if getattr(self, "_is_vert_last", False) else 0.995)
                self._is_vert_last = is_vertical

                reg, rv3d = context.region, context.region_data
                p2d = view3d_utils.location_3d_to_region_2d(reg, rv3d, self.p1)
                
                if p2d:
                    if is_perp or is_vertical:
                        if is_vertical:
                            self.Xp = b_vec
                            ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                            view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                            self.Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                            if self.Zp.dot(view_dir) > 0: self.Zp = -self.Zp
                            self.Yp = self.Zp.cross(self.Xp).normalized()
                        else:
                            self.Xp = b_vec
                            self.Zp = self.Xp.cross(up).normalized()
                            view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                            if self.Zp.dot(view_dir) > 0: self.Zp = -self.Zp
                            self.Yp = self.Zp.cross(self.Xp).normalized()
                    else:
                        self.Zp = up.copy()
                        self.Xp = b_vec
                        self.Yp = self.Zp.cross(self.Xp).normalized()
                
                self.major_axis = self.Xp.copy()

            self.rx = (target - self.p1).length * 0.5
            self.current = target
            self.preview_pts = [self.p1, self.current]

        if self.stage == 2:
            center = (self.p1 + self.p2) * 0.5
            target_pt = snap_point
            if not self.state.get("geometry_snap", False):
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, center, self.Zp)
                if hit: target_pt = hit

            d_raw = target_pt - center
            d_plane = d_raw - self.Zp * d_raw.dot(self.Zp)
            dist_y = d_plane.dot(self.Yp)
            self.ry = abs(dist_y)
            self.current = center + (self.Yp * dist_y)
            self.segments = self.state["segments"]
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"], self.state["locked_normal"] = True, self.ref_normal
            if self.stage == 2: self._update_basis_locked()
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            if self.stage == 2: self._update_basis_locked()
            return True
        return False

    def _update_basis_locked(self):
        up = self.ref_normal
        is_perp = self.state.get("is_perpendicular")
        b_vec = self.major_axis
        is_vertical = abs(b_vec.dot(up)) > 0.995

        if is_perp or is_vertical:
            if is_vertical:
                self.Xp = b_vec
                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                if self.vertical_override_axis == 'X': new_Zp = ax_x
                elif self.vertical_override_axis == 'Y': new_Zp = ax_y
                else:
                    rv3d = bpy.context.region_data
                    view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    new_Zp = ax_x if abs(view_dir.dot(ax_x)) > abs(view_dir.dot(ax_y)) else ax_y
                self.Zp = new_Zp
                self.Yp = up.normalized()
                self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Xp = b_vec
                self.Zp = self.Xp.cross(up).normalized()
                rv3d = bpy.context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                if self.Zp.dot(view_fwd) > 0: self.Zp = -self.Zp
                self.Yp = self.Zp.cross(self.Xp).normalized()
        else:
            self.Zp = up.copy()
            self.Xp = b_vec
            self.Yp = self.Zp.cross(self.Xp).normalized()
        self.state["locked_normal"] = self.Zp

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage = 1
            return 'NEXT_STAGE'
        if self.stage == 1:
            self.p2 = self.current if self.current else snap_point
            if (self.p2 - self.pivot).length < 1e-6: return None
            self.stage = 2
            return 'NEXT_STAGE'
        if self.stage == 2: return 'FINISHED'
        return None

    def refresh_preview(self):
        if self.stage == 1:
            self.p1 = self.pivot
            self.p2 = self.current if self.current else self.p1
            self.stage = 2 
            self.preview_pts = [self.p1, self.p2]
        elif self.stage >= 2:
            if self.p1 is None or self.p2 is None or self.ry is None:
                self.preview_pts = []
                return
            
            center = (self.p1 + self.p2) * 0.5
            # Update current to reflect the minor radius point
            self.current = center + (self.Yp * self.ry)
            
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

class EllipseTool_FromCorners(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "ELLIPSE_CORNERS"
        self.segments = self.state["segments"]
        self.rx = 0.0
        self.ry = 0.0
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None
        self.initial_Xp = None
        self.initial_Yp = None

    def update(self, context, event, snap_point, snap_normal):
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return
        
        coord = (event.mouse_region_x, event.mouse_region_y)
        if self.stage == 1:
            # --- CUSTOM PROJECT TO PLANE ---
            target = snap_point
            if not self.state.get("geometry_snap", False):
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.Zp)
                if hit: target = hit

            # BASIS logic
            up = self.ref_normal
            is_perp = self.state.get("is_perpendicular", False)
            bridge = target - self.pivot
            is_vertical = abs(bridge.normalized().dot(up)) > (0.98 if getattr(self, "_is_vert_last", False) else 0.995) if bridge.length_squared > 1e-8 else False
            self._is_vert_last = is_vertical

            if is_perp or is_vertical:
                if is_vertical:
                    ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                    if self.vertical_override_axis is None:
                        rv3d = context.region_data
                        view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                        new_Zp = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                    else:
                        new_Zp = ax_x if self.vertical_override_axis == 'X' else ax_y
                else:
                    rv3d = context.region_data
                    view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    if abs(view_fwd.dot(self.initial_Xp)) > abs(view_fwd.dot(self.initial_Yp)):
                        new_Zp = self.initial_Xp.copy()
                    else:
                        new_Zp = self.initial_Yp.copy()
                
                if new_Zp.length > 1e-4:
                    rv3d = context.region_data
                    view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                    if new_Zp.dot(view_fwd) > 0: new_Zp = -new_Zp
                    self.Zp = new_Zp.normalized()
                    self.Yp = up.normalized()
                    self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Zp = up.copy()
                self.Xp, self.Yp = self.initial_Xp.copy(), self.initial_Yp.copy()

            self.state["locked_normal"] = self.Zp

            # Re-project to final Zp
            if not self.state.get("geometry_snap", False):
                ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
                ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, self.pivot, self.Zp)
                if hit: target = hit

            self.current = target
            d_vec = self.current - self.pivot
            width, height = d_vec.dot(self.Xp), d_vec.dot(self.Yp)
            self.rx, self.ry = abs(width) * 0.5, abs(height) * 0.5
            center = self.pivot + (self.Xp * (width * 0.5)) + (self.Yp * (height * 0.5))
            self.segments = self.state["segments"]
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"] = True
            self.state["locked_normal"] = self.ref_normal
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            return True
        return False

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            up = Vector((0,0,1))
            if abs(self.Zp.dot(up)) > 0.99:
                self.initial_Xp, self.initial_Yp = Vector((1, 0, 0)), Vector((0, 1, 0))
                self.initial_Yp = self.Zp.cross(self.initial_Xp).normalized()
                self.initial_Xp = self.initial_Yp.cross(self.Zp).normalized()
            else:
                self.initial_Xp, self.initial_Yp = self.Xp.copy(), self.Yp.copy()
            self.state["locked"], self.state["locked_normal"] = True, self.Zp
            if self.Zp: self.ref_normal = self.Zp.copy()
            self.stage = 1
            return 'NEXT_STAGE'
        if self.stage == 1: return 'FINISHED'
        return None

    def refresh_preview(self):
        if self.stage >= 1:
            center = self.pivot + (self.Xp * self.rx) + (self.Yp * self.ry)
            self.preview_pts = ellipse_points_world(center, self.rx, self.ry, self.segments, self.Xp, self.Yp)
