import math
from mathutils import Vector
from .base_tool import SurfaceDrawTool

class RectangleTool_CenterCorner(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "RECTANGLE_CENTER_CORNER"
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None

    def update(self, context, event, snap_point, snap_normal):
        # Stage 0: Set Center / Plane
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        # Stage 1: Drag to Corner
        if self.stage == 1:
            center = self.pivot
            coord = (event.mouse_region_x, event.mouse_region_y)

            from bpy_extras import view3d_utils
            from mathutils import geometry
            from ..orientation_utils import orthonormal_basis_from_normal

            # 1. ORIENTATION LOGIC FIRST — so Zp is correct before we resolve target
            up = self.ref_normal
            is_perp = self.state.get("is_perpendicular", False)

            if is_perp:
                # Flip to view-aligned vertical plane
                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                rv3d = context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                new_Zp = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                if new_Zp.dot(view_fwd) > 0: new_Zp = -new_Zp
                self.Zp = new_Zp.normalized()
                self.Yp = up.normalized()
                self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Zp = up.copy()
                self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)

            # 2. Resolve target using the updated Zp — intersects the correct plane
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            if is_perp:
                # In perp mode, always raycast to the vertical plane — snap_point is a floor
                # point with z=0 which would collapse height to zero
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, center, self.Zp)
                target = hit if hit else snap_point
            else:
                if snap_point and (snap_point - center).length > 1e-6:
                    target = snap_point
                else:
                    hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, center, self.Zp)
                    target = hit if hit else snap_point

            # NOTE: No axis snapping — snapping to a world axis zeros out dx or dy, collapsing to a line.

            # 3. Project target onto plane
            if self.Zp:
                offset = (target - center).dot(self.Zp)
                target = target - self.Zp * offset

            self.current = target
            d_vec = target - center

            # 4. Calculate dimensions on our basis
            dx = d_vec.dot(self.Xp)
            dy = d_vec.dot(self.Yp)
            self.rx, self.ry = dx, dy # For refresh/HUD
            
            # 5. Define 4 corners centered on pivot
            p1 = center + (self.Xp * dx) + (self.Yp * dy)
            p2 = center - (self.Xp * dx) + (self.Yp * dy)
            p3 = center - (self.Xp * dx) - (self.Yp * dy)
            p4 = center + (self.Xp * dx) - (self.Yp * dy)
            
            self.preview_pts = [p1, p2, p3, p4, p1]

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"], self.state["locked_normal"] = True, self.ref_normal
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            return True
        return False

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"] = True
            self.state["locked_normal"] = self.Zp
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage == 1:
            return 'FINISHED'
        return None

class RectangleTool_CornerCorner(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "RECTANGLE_CORNER_CORNER"
        self.current = None
        self.preview_pts = []
        self.ref_normal = Vector((0,0,1))
        self.vertical_override_axis = None

    def update(self, context, event, snap_point, snap_normal):
        # Stage 0: Set First Corner / Plane
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        # Stage 1: Drag to Second Corner
        if self.stage == 1:
            c1 = self.pivot
            coord = (event.mouse_region_x, event.mouse_region_y)

            from bpy_extras import view3d_utils
            from mathutils import geometry
            from ..orientation_utils import orthonormal_basis_from_normal

            # 1. ORIENTATION LOGIC FIRST — so Zp is correct before we resolve target
            up = self.ref_normal
            is_perp = self.state.get("is_perpendicular", False)

            if is_perp:
                # Flip to view-aligned vertical plane
                ax_x, ax_y, _ = orthonormal_basis_from_normal(up)
                rv3d = context.region_data
                view_fwd = rv3d.view_matrix.inverted().to_3x3() @ Vector((0,0,-1))
                new_Zp = ax_x if abs(view_fwd.dot(ax_x)) > abs(view_fwd.dot(ax_y)) else ax_y
                if new_Zp.dot(view_fwd) > 0: new_Zp = -new_Zp
                self.Zp = new_Zp.normalized()
                self.Yp = up.normalized()
                self.Xp = self.Yp.cross(self.Zp).normalized()
            else:
                self.Zp = up.copy()
                self.Xp, self.Yp, _ = orthonormal_basis_from_normal(self.Zp)

            # 2. Resolve target using the updated Zp — intersects the correct plane
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            if is_perp:
                # In perp mode, always raycast to the vertical plane — snap_point is a floor
                # point with z=0 which would collapse height to zero
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, c1, self.Zp)
                target = hit if hit else snap_point
            else:
                if snap_point and (snap_point - c1).length > 1e-6:
                    target = snap_point
                else:
                    hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, c1, self.Zp)
                    target = hit if hit else snap_point

            # NOTE: No axis snapping — snapping to a world axis zeros out dx or dy, collapsing to a line.

            # 3. Project target onto plane
            if self.Zp:
                offset = (target - c1).dot(self.Zp)
                target = target - self.Zp * offset

            self.current = target
            d_vec = target - c1

            # 4. Calculate dimensions on our basis
            dx = d_vec.dot(self.Xp)
            dy = d_vec.dot(self.Yp)
            self.rx, self.ry = dx, dy
            
            # 5. Define 4 corners starting from anchor c1
            p1 = c1
            p2 = c1 + (self.Xp * dx)
            p3 = c1 + (self.Xp * dx) + (self.Yp * dy)
            p4 = c1 + (self.Yp * dy)
            
            self.preview_pts = [p1, p2, p3, p4, p1]

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.vertical_override_axis = None 
            self.state["locked"], self.state["locked_normal"] = True, self.ref_normal
            return True
        if event.type in {'X', 'Y'} and event.value == 'PRESS' and self.stage > 0:
            self.vertical_override_axis = event.type
            return True
        return False

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"] = True
            self.state["locked_normal"] = self.Zp
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage == 1:
            return 'FINISHED'
        return None

class RectangleTool_3Point(SurfaceDrawTool):
    def __init__(self, core):
        super().__init__(core)
        self.mode = "RECTANGLE_3_POINTS"
        self.current = None
        self.preview_pts = []
        self.p2 = None
        self.ref_normal = Vector((0,0,1))

    def update(self, context, event, snap_point, snap_normal):
        # Stage 0: Set P1 (Start of Edge)
        if self.stage == 0:
            self.update_initial_plane(context, event, snap_point, snap_normal)
            if self.Zp: self.ref_normal = self.Zp.copy()
            return

        # Common snapping setup
        from ..inference_utils import get_axis_snapped_location
        strength_deg = self.state.get("snap_strength", 6.0)
        axis_thresh = math.cos(math.radians(strength_deg))
        coord = (event.mouse_region_x, event.mouse_region_y)

        # Stage 1: Set P2 (End of Edge / Rotation)
        if self.stage == 1:
            p1 = self.pivot

            # Reject snap_point if it snapped back to the anchor (collapses the edge/rectangle)
            from bpy_extras import view3d_utils
            from mathutils import geometry as geo
            ray_o = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_v = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            if snap_point and (snap_point - p1).length > 1e-6:
                target = snap_point
            else:
                hit = geo.intersect_line_plane(ray_o, ray_o + ray_v, p1, self.ref_normal)
                target = hit if hit else snap_point

            # Axis Snapping (Ignore normal-axis snaps like Z)
            inf_loc, inf_vec, _ = get_axis_snapped_location(p1, coord, context, snap_threshold=axis_thresh)
            if inf_loc and not event.alt:
                if inf_vec and abs(inf_vec.dot(self.ref_normal)) < 0.9:
                    target = inf_loc

            self.current = target
            # Just draw the edge being defined
            self.preview_pts = [p1, target]

        # Stage 2: Set Width/Height
        if self.stage == 2:
            p1 = self.pivot
            p2 = self.p2
            
            # 1. Edge Vector and Orientation
            edge_vec = p2 - p1
            if edge_vec.length < 1e-6: return

            is_perp = self.state.get("is_perpendicular", False)
            if is_perp:
                self.Xp = edge_vec.normalized()
                self.Yp = self.ref_normal.normalized()
                self.Zp = self.Xp.cross(self.Yp).normalized()
            else:
                self.Zp = self.ref_normal.normalized()
                self.Xp = edge_vec.normalized()
                self.Yp = self.Zp.cross(self.Xp).normalized()

            # 2. STABLE TETHERING
            from bpy_extras import view3d_utils
            from mathutils import geometry
            ray_origin = view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, coord)
            ray_vector = view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, coord)
            
            res = geometry.intersect_line_line(p2, p2 + self.Yp, ray_origin, ray_origin + ray_vector)
            
            if res:
                p_axis, p_ray = res
                height = (p_axis - p2).dot(self.Yp)
            else:
                hit = geometry.intersect_line_plane(ray_origin, ray_origin + ray_vector, p2, self.Zp)
                height = (hit - p2).dot(self.Yp) if hit else 0.0

            # 3. Apply Snapping to Height (if aligned to axis and NOT perpendicular and NOT normal-axis)
            if not is_perp:
                inf_loc, inf_vec, _ = get_axis_snapped_location(p2, coord, context, snap_threshold=axis_thresh)
                if inf_loc and not event.alt:
                    if inf_vec and abs(inf_vec.dot(self.Zp)) < 0.9:
                        # Project snapped point onto plane, then extract height
                        offset = (inf_loc - p2).dot(self.Zp)
                        snapped_on_plane = inf_loc - self.Zp * offset
                        height = (snapped_on_plane - p2).dot(self.Yp)

            height_vec = self.Yp * height
            
            # 4. Calculate corners
            c1, c2 = p1, p2
            c3 = p2 + height_vec
            c4 = p1 + height_vec
            
            self.current = c3 # Tether indicator line to the corner
            self.preview_pts = [c1, c2, c3, c4, c1]

    def handle_input(self, context, event):
        if super().handle_plane_lock_input(context, event):
            if self.Zp: self.ref_normal = self.Zp.copy()
            return True
        if event.type == 'P' and event.value == 'PRESS':
            if self.stage == 0: return False
            self.state["is_perpendicular"] = not self.state.get("is_perpendicular", False)
            self.state["locked"] = True
            self.state["locked_normal"] = self.ref_normal
            return True
        return False

    def handle_click(self, context, event, snap_point, snap_normal, button_id=None):
        if self.stage == 0:
            self.pivot = snap_point
            self.state["locked"] = True
            self.state["locked_normal"] = self.Zp
            self.stage = 1
            return 'NEXT_STAGE'

        if self.stage == 1:
            # Prevent zero-length edge
            if self.current is None or (self.current - self.pivot).length < 1e-6: return None 
            self.p2 = self.current
            self.stage = 2
            return 'NEXT_STAGE'

        if self.stage == 2:
            return 'FINISHED'
        return None