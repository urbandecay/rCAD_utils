import math
import time
import bpy
import bmesh
from mathutils import Vector, Matrix
from mathutils.geometry import intersect_line_plane
from bpy_extras.view3d_utils import region_2d_to_origin_3d, region_2d_to_vector_3d, location_3d_to_region_2d

from .modal_state import state, reset_state_from_context
from .orientation_utils import orthonormal_basis_from_normal
from .plane_utils import world_to_plane, plane_to_world, project_mouse_to_ground, raycast_under_mouse

# --- PERSISTENT REGISTRY FIX ---
# We store the registry in driver_namespace so it survives script reloads.
# This ensures "Clear Stuck Overlays" can always find handles from previous sessions.
if "radcad_draw_handler_registry" not in bpy.app.driver_namespace:
    bpy.app.driver_namespace["radcad_draw_handler_registry"] = {}

_DRAW_HANDLER_REGISTRY = bpy.app.driver_namespace["radcad_draw_handler_registry"]
# -------------------------------

class DrawManager:
    @staticmethod
    def add_handler(source_id, draw_func, args, region_type='WINDOW', draw_event='POST_VIEW'):
        if source_id in _DRAW_HANDLER_REGISTRY:
            DrawManager.remove_handler(source_id)
        try:
            handle = bpy.types.SpaceView3D.draw_handler_add(draw_func, args, region_type, draw_event)
            _DRAW_HANDLER_REGISTRY[source_id] = (handle, region_type)
        except Exception as e:
            print(f"[DrawManager] Failed to register {source_id}: {e}")

    @staticmethod
    def remove_handler(source_id):
        if source_id not in _DRAW_HANDLER_REGISTRY:
            return
        handle, region_type = _DRAW_HANDLER_REGISTRY[source_id]
        try:
            bpy.types.SpaceView3D.draw_handler_remove(handle, region_type)
        except: pass
        del _DRAW_HANDLER_REGISTRY[source_id]

    @staticmethod
    def clear_all():
        for source_id in list(_DRAW_HANDLER_REGISTRY.keys()):
            DrawManager.remove_handler(source_id)

def is_event_over_ui(context, event):
    if context.area.type != 'VIEW_3D': return False
    for region in context.area.regions:
        if region.type == 'WINDOW': continue
        if (region.x <= event.mouse_x <= region.x + region.width) and \
           (region.y <= event.mouse_y <= region.y + region.height):
            return True
    return False

def is_number_input(ev):
    valid_keys = {
        'ZERO', 'ONE', 'TWO', 'THREE', 'FOUR', 'FIVE', 'SIX', 'SEVEN', 'EIGHT', 'NINE',
        'PERIOD', 'MINUS',
        'NUMPAD_0', 'NUMPAD_1', 'NUMPAD_2', 'NUMPAD_3', 'NUMPAD_4',
        'NUMPAD_5', 'NUMPAD_6', 'NUMPAD_7', 'NUMPAD_8', 'NUMPAD_9',
        'NUMPAD_PERIOD', 'NUMPAD_MINUS'
    }
    return ev.type in valid_keys and ev.value == 'PRESS'

def apply_custom_orbit(context, pivot, dx, dy):
    rv3d = context.region_data
    if not rv3d: return
    speed = 0.01
    view_mat = rv3d.view_matrix
    cam_mat = view_mat.inverted()
    trans_to = Matrix.Translation(pivot)
    trans_from = Matrix.Translation(-pivot)
    angle_z = -dx * speed
    rot_z = Matrix.Rotation(angle_z, 4, 'Z')
    orbit_z = trans_to @ rot_z @ trans_from
    cam_mat = orbit_z @ cam_mat
    cam_fwd = -cam_mat.col[2].xyz 
    world_up = Vector((0, 0, 1))
    if abs(cam_fwd.dot(world_up)) > 0.99: flat_right = cam_mat.col[0].xyz
    else: flat_right = cam_fwd.cross(world_up).normalized()
    angle_x = dy * speed 
    rot_x = Matrix.Rotation(angle_x, 4, flat_right)
    orbit_x = trans_to @ rot_x @ trans_from
    cam_mat = orbit_x @ cam_mat
    rv3d.view_matrix = cam_mat.inverted()

class ModalManager:
    def __init__(self, ctx, operator=None):
        self.operator = operator
        self.state = state
        self.active_tool = None
        self.region = ctx.region
        self.rv3d = ctx.region_data
        self.is_navigating = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        if ctx.area.type != 'VIEW_3D' or (ctx.region and ctx.region.type != 'WINDOW'):
            for area in ctx.screen.areas:
                if area.type == 'VIEW_3D':
                    for region in area.regions:
                        if region.type == 'WINDOW':
                            self.region = region
                            self.rv3d = area.spaces.active.region_3d
                            break
                    if self.region: break

        t_mode = state.get("tool_mode", "1POINT")
        
        if t_mode == "1POINT": 
            from .operators import arc_tools
            self.active_tool = arc_tools.ArcTool_1Point(self)
        elif t_mode == "2POINT": 
            from .operators import arc_tools
            self.active_tool = arc_tools.ArcTool_2Point(self)
        elif t_mode == "3POINT": 
            from .operators import arc_tools
            self.active_tool = arc_tools.ArcTool_3Point(self)
            
        elif t_mode == "CIRCLE_2POINT": 
            from .operators import circle_tools
            self.active_tool = circle_tools.CircleTool_2Point(self)
        elif t_mode == "CIRCLE_3POINT": 
            from .operators import circle_tools
            self.active_tool = circle_tools.CircleTool_3Point(self)
            
        elif t_mode == "CIRCLE_TAN_TAN": 
            from .operators import circle_tools
            self.active_tool = circle_tools.CircleTool_TanTan(self)

        elif t_mode == "CIRCLE_TAN_TAN_TAN":
            from .operators import op_circle_tan_tan_tan
            self.active_tool = op_circle_tan_tan_tan.CircleTool_TanTanTan(self)
            
        elif t_mode == "ELLIPSE_RADIUS": 
            from .operators import ellipse_tools
            self.active_tool = ellipse_tools.EllipseTool_FromRadius(self)
        elif t_mode == "ELLIPSE_ENDPOINTS": 
            from .operators import ellipse_tools
            self.active_tool = ellipse_tools.EllipseTool_FromEndpoints(self)
        elif t_mode == "ELLIPSE_FOCI":
            from .operators import ellipse_tools
            self.active_tool = ellipse_tools.EllipseTool_FociPoint(self)
        elif t_mode == "ELLIPSE_CORNERS": 
            from .operators import ellipse_tools
            self.active_tool = ellipse_tools.EllipseTool_FromCorners(self)
            
        elif t_mode == "POLYGON_CENTER_CORNER": 
            from .operators import polygon_tools
            self.active_tool = polygon_tools.PolygonTool_CenterCorner(self)
        elif t_mode == "POLYGON_CENTER_TANGENT": 
            from .operators import polygon_tools
            self.active_tool = polygon_tools.PolygonTool_CenterTangent(self)
        elif t_mode == "POLYGON_CORNER_CORNER": 
            from .operators import polygon_tools
            self.active_tool = polygon_tools.PolygonTool_CornerCorner(self)
        elif t_mode == "POLYGON_EDGE": 
            from .operators import polygon_tools
            self.active_tool = polygon_tools.PolygonTool_Edge(self)

        elif t_mode == "RECTANGLE_CENTER_CORNER":
            from .operators import rectangle_tools
            self.active_tool = rectangle_tools.RectangleTool_CenterCorner(self)
        elif t_mode == "RECTANGLE_CORNER_CORNER":
            from .operators import rectangle_tools
            self.active_tool = rectangle_tools.RectangleTool_CornerCorner(self)
        elif t_mode == "RECTANGLE_3_POINTS":
            from .operators import rectangle_tools
            self.active_tool = rectangle_tools.RectangleTool_3Point(self)
            
        elif t_mode == "LINE_POLY": 
            from .operators import line_tools
            self.active_tool = line_tools.LineTool_Poly(self)
        
        elif t_mode == "LINE_PERP_FROM_CURVE":
            from .operators import line_tools
            self.active_tool = line_tools.LineTool_PerpFromCurve(self)

        elif t_mode == "LINE_TAN_TAN": 
            from .operators import line_tools
            self.active_tool = line_tools.LineTool_TanTan(self)

        elif t_mode == "LINE_PERP_TO_TWO_CURVES": 
            from .operators import line_tools
            self.active_tool = line_tools.LineTool_PerpToTwoCurves(self)

        elif t_mode == "LINE_TANGENT_FROM_CURVE": 
            from .operators import line_tools
            self.active_tool = line_tools.LineTool_TangentFromCurve(self)

        elif t_mode == "CURVE_INTERPOLATE": 
            from .operators import curve_tools
            self.active_tool = curve_tools.CurveTool_Interpolate(self)
        elif t_mode == "CURVE_FREEHAND": 
            from .operators import curve_tools
            self.active_tool = curve_tools.CurveTool_Freehand(self)
            
        elif t_mode == "POINT_BY_ARCS":
            from .operators import point_tools
            self.active_tool = point_tools.PointTool_ByArcs(self)

        elif t_mode == "POINT_CENTER":
            from .operators import point_tools
            self.active_tool = point_tools.PointTool_Center(self)
            
        else: 
            from .operators import arc_tools
            self.active_tool = arc_tools.ArcTool_1Point(self)

    def report(self, type_set, message):
        """Pass report messages to the operator if available."""
        if self.operator:
            self.operator.report(type_set, message)
        else:
            print(f"radCAD Report {type_set}: {message}")

    def get_snap_data(self, ctx, x, y):
        snapped_pos = None
        
        reg, rv3d = self.region, self.rv3d
        if not reg or not rv3d: return Vector((0,0,0)), Vector((0,0,1))
        snap_radius = self.state.get("snap_strength", 6.0) * 2.0

        # --- OPTIMIZATION: Skip expensive mesh snapping for Freehand tool ---
        if state.get("tool_mode") != "CURVE_FREEHAND":
            try:
                from .snapping_utils import snap_to_mesh_components
            except ImportError:
                def snap_to_mesh_components(**kwargs): return None

            snapped_pos = snap_to_mesh_components(
                ctx, ctx.edit_object, x, y, max_px=snap_radius,
                do_verts=state.get("snap_verts", True),
                do_edges=state.get("snap_edges", True),
                do_edge_center=state.get("snap_edge_center", True),
                do_faces=False, 
                do_face_center=state.get("snap_face_center", True)
            )

            # --- PREVIEW SNAPPING (SELF-SNAP) ---
            self_snap_targets = []
            if state.get("tool_mode") == "LINE_POLY":
                preview_pts = state.get("preview_pts", [])
                if len(preview_pts) > 1: self_snap_targets = preview_pts[:-1]
            elif state.get("tool_mode") == "CURVE_INTERPOLATE":
                # Snap to the full smooth curve preview (vertices and edges)
                preview_pts = state.get("preview_pts", [])
                if len(preview_pts) > 1: self_snap_targets = preview_pts[:-1]
            elif state.get("tool_mode") == "POINT_BY_ARCS":
                self_snap_targets = getattr(self.active_tool, "endpoints_1", [])

            if self_snap_targets:
                best_self_pt = None
                best_self_dist = float('inf')
                limit_sq = snap_radius * snap_radius
                mvec = Vector((x, y))

                for pt in self_snap_targets:
                    p2d = location_3d_to_region_2d(reg, rv3d, pt)
                    if p2d:
                        d2 = (mvec - p2d).length_squared
                        if d2 < limit_sq and d2 < best_self_dist:
                            best_self_dist = d2
                            best_self_pt = pt

                # For CURVE_INTERPOLATE, snap to edges/edge-centers if enabled
                if state.get("tool_mode") == "CURVE_INTERPOLATE":
                    preview_pts = state.get("preview_pts", [])
                    for i in range(len(preview_pts) - 2):
                        p0, p1 = preview_pts[i], preview_pts[i+1]
                        p0_2d = location_3d_to_region_2d(reg, rv3d, p0)
                        p1_2d = location_3d_to_region_2d(reg, rv3d, p1)
                        if p0_2d and p1_2d:
                            edge_2d = p1_2d - p0_2d
                            edge_len_sq = edge_2d.length_squared
                            if edge_len_sq > 1e-8:
                                # Snap to edge center if that button is on
                                if state.get("snap_edge_center", False):
                                    center_2d = p0_2d + edge_2d * 0.5
                                    d2 = (mvec - center_2d).length_squared
                                    if d2 < limit_sq and d2 < best_self_dist:
                                        best_self_dist = d2
                                        best_self_pt = (p0 + p1) * 0.5
                                # Snap to closest point on edge if that button is on
                                elif state.get("snap_edges", True):
                                    t = max(0, min(1, (mvec - p0_2d).dot(edge_2d) / edge_len_sq))
                                    closest_2d = p0_2d + edge_2d * t
                                    d2 = (mvec - closest_2d).length_squared
                                    if d2 < limit_sq and d2 < best_self_dist:
                                        best_self_dist = d2
                                        best_self_pt = p0 + (p1 - p0) * t

                if best_self_pt:
                    use_self = True
                    if snapped_pos:
                        p2d_mesh = location_3d_to_region_2d(reg, rv3d, snapped_pos)
                        if p2d_mesh:
                            dist_mesh = (mvec - p2d_mesh).length_squared
                            if dist_mesh <= best_self_dist:
                                use_self = False
                    if use_self:
                        snapped_pos = best_self_pt

        # --- FALLBACK TO SURFACE/PLANE (STILL ACTIVE FOR FREEHAND) ---
        state["snap_point"] = None 
        if snapped_pos is not None:
            state["geometry_snap"] = True
            state["snap_point"] = snapped_pos
            state["last_surface_hit"] = snapped_pos
            locked_normal = state.get("locked_normal")
            if locked_normal and state.get("locked"):
                return snapped_pos, locked_normal
            _, nrm, _ = raycast_under_mouse(ctx, x, y)
            return snapped_pos, nrm if nrm else Vector((0,0,1))
        
        is_locked = state.get("locked")
        locked_normal = state.get("locked_normal")
        if is_locked and locked_normal:
            l_point = state.get("pivot") or state.get("locked_plane_point") or Vector((0,0,0))
            ray_origin = region_2d_to_origin_3d(reg, rv3d, (x,y))
            ray_vector = region_2d_to_vector_3d(reg, rv3d, (x,y))
            hit_plane = intersect_line_plane(ray_origin, ray_origin + ray_vector * 10000, l_point, locked_normal)
            if hit_plane:
                state["geometry_snap"] = False
                state["last_surface_hit"] = hit_plane
                state["last_surface_normal"] = locked_normal
                return hit_plane, locked_normal

        view_vec = region_2d_to_vector_3d(reg, rv3d, (x,y))
        ray_origin = region_2d_to_origin_3d(reg, rv3d, (x,y))
        depsgraph = ctx.evaluated_depsgraph_get()
        hit, loc, norm, _, _, _ = ctx.scene.ray_cast(depsgraph, ray_origin, view_vec)
        if hit:
            state["geometry_snap"] = False
            state["last_surface_hit"] = loc
            state["last_surface_normal"] = norm
            return loc, norm

        # --- FALLBACK: VOID DRAWING (Smart Ortho Alignment) ---
        plane_normal = Vector((0, 0, 1))
        if rv3d.view_perspective == 'ORTHO':
            view_dir = rv3d.view_matrix.inverted().to_3x3() @ Vector((0, 0, -1))
            x_align = abs(view_dir.dot(Vector((1, 0, 0))))
            y_align = abs(view_dir.dot(Vector((0, 1, 0))))
            z_align = abs(view_dir.dot(Vector((0, 0, 1))))
            limit = 0.99
            if x_align > limit: plane_normal = Vector((1, 0, 0))
            elif y_align > limit: plane_normal = Vector((0, 1, 0))
            elif z_align > limit: plane_normal = Vector((0, 0, 1))
            else: plane_normal = -view_dir

        denom = view_vec.dot(plane_normal)
        if abs(denom) > 1e-6:
            t = (Vector((0,0,0)) - ray_origin).dot(plane_normal) / denom
            gpos = ray_origin + view_vec * t
        else:
            gpos = Vector((0,0,0))
            
        state["geometry_snap"] = False
        state["last_surface_hit"] = gpos
        state["last_surface_normal"] = plane_normal
        return gpos, plane_normal

    def on_move(self, context, event):
        if self.active_tool:
            # --- FIX: One-frame bypass for numerical input ---
            if state.get("skip_mouse_update"):
                state["skip_mouse_update"] = False
                self.sync_tool_to_state()
                # Force the tool to recalculate its points from the new state
                if hasattr(self.active_tool, "refresh_preview"):
                    self.active_tool.refresh_preview()
                self.sync_tool_from_state()
                context.area.tag_redraw()
                return

            snap_pt, snap_n = self.get_snap_data(context, event.mouse_region_x, event.mouse_region_y)
            self.active_tool.update(context, event, snap_pt, snap_n)
            self.sync_tool_from_state()
            context.area.tag_redraw()

    def sync_tool_from_state(self):
        """Copies tool properties into the shared 'state'."""
        t = self.active_tool
        state["stage"] = t.stage
        state["pivot"] = t.pivot
        state["current"] = t.current
        state["start"] = getattr(t, "start", None)
        state["p1"] = getattr(t, "p1", None)
        state["p2"] = getattr(t, "p2", None)
        state["f1"] = getattr(t, "f1", None)
        state["f2"] = getattr(t, "f2", None)
        state["midpoint"] = getattr(t, "midpoint", None)
        state["radius"] = getattr(t, "radius", 0.0)
        state["compass_rot"] = getattr(t, "compass_rot", 0.0)
        state["a0"] = getattr(t, "a0", 0.0)
        state["a1"] = getattr(t, "a1", 0.0)
        state["accum_angle"] = getattr(t, "accum_angle", 0.0)
        state["a_prev_raw"] = getattr(t, "a_prev_raw", 0.0)
        state["segments"] = getattr(t, "segments", 32)
        state["min_dist"] = getattr(t, "min_dist", 0.05)
        state["rx"] = getattr(t, "rx", 0.0)
        state["ry"] = getattr(t, "ry", 0.0)
        state["preview_pts"] = getattr(t, "preview_pts", [])
        state["intersection_pts"] = getattr(t, "intersection_pts", [])
        state["spline_geom"] = getattr(t, "spline_geom", [])
        state["Xp"] = t.Xp
        state["Yp"] = t.Yp
        state["Zp"] = t.Zp

    def sync_tool_to_state(self):
        """Copies shared 'state' values back into the tool instance."""
        t = self.active_tool
        if "radius" in state: t.radius = state["radius"]
        if "stage" in state: t.stage = state["stage"]
        if "start" in state: t.start = state["start"]
        if "p1" in state: t.p1 = state["p1"]
        if "p2" in state: t.p2 = state["p2"]
        if "f1" in state: t.f1 = state["f1"]
        if "f2" in state: t.f2 = state["f2"]
        if "rx" in state: t.rx = state["rx"]
        if "ry" in state: t.ry = state["ry"]
        if "midpoint" in state: t.midpoint = state["midpoint"]
        if "current" in state: t.current = state["current"]
        if "segments" in state: t.segments = state["segments"]
        if "min_dist" in state: t.min_dist = state["min_dist"]
        if "a0" in state: t.a0 = state["a0"]
        if "a1" in state: t.a1 = state["a1"]
        if "accum_angle" in state: t.accum_angle = state["accum_angle"]
        if "a_prev_raw" in state: t.a_prev_raw = state["a_prev_raw"]

def get_or_create_grey_material():
    mat_name = "radCAD_Grey"
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        bsdf = nodes.get("Principled BSDF")
        if bsdf:
            # Use name-based access for Blender 4.0+ compatibility
            if "Base Color" in bsdf.inputs: bsdf.inputs["Base Color"].default_value = (0.5, 0.5, 0.5, 1)
            if "Roughness" in bsdf.inputs: bsdf.inputs["Roughness"].default_value = 1.0
    return mat

def get_or_create_black_material():
    mat_name = "radCAD_Black"
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        bsdf = nodes.get("Principled BSDF")
        if bsdf:
            if "Base Color" in bsdf.inputs: bsdf.inputs["Base Color"].default_value = (0, 0, 0, 1)
            if "Roughness" in bsdf.inputs: bsdf.inputs["Roughness"].default_value = 1.0
    return mat

def commit_arc_to_mesh(ctx):
    from . import arc_weld_manager

    # Determine tool name for the new object
    tool_mode = state.get("tool_mode", "CAD_Object")
    obj_name = tool_mode.replace("_", " ").title()

    obj = ctx.edit_object
    bm = bmesh.from_edit_mesh(obj.data)
    imw = obj.matrix_world.inverted()
    
    if state["tool_mode"] in ("POINT_BY_ARCS", "POINT_CENTER"):
        int_pts = state.get("intersection_pts", [])
        if not int_pts: return
        bpy.ops.mesh.select_all(action='DESELECT')
        for wp in int_pts:
            v = bm.verts.new(imw @ wp)
            v.select = True
        bm.select_history.clear()
        bm.verts.ensure_lookup_table()
        bmesh.update_edit_mesh(obj.data)
        return

    pts = state["preview_pts"]
    if not pts: return
    is_closed = abs(state["accum_angle"]) >= (2 * math.pi - 0.001)
    
    # Continuous tools that always have a "floating" mouse point at the end
    continuous_tools = ["LINE_POLY", "CURVE_INTERPOLATE"]
    
    # Shape tools that should be closed automatically
    shape_tools = ["CIRCLE_2POINT", "CIRCLE_3POINT", "CIRCLE_TAN_TAN", "CIRCLE_TAN_TAN_TAN", 
                   "ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS", "ELLIPSE_CORNERS", 
                   "POLYGON_CENTER_CORNER", "POLYGON_CENTER_TANGENT", "POLYGON_CORNER_CORNER", "POLYGON_EDGE", 
                   "RECTANGLE_CENTER_CORNER", "RECTANGLE_CORNER_CORNER", "RECTANGLE_3_POINTS"]

    # Line-to-Curve tools that should commit BOTH points without closing
    complete_line_tools = ["LINE_PERP_FROM_CURVE", "LINE_PERP_TO_TWO_CURVES", "LINE_TANGENT_FROM_CURVE", "LINE_TAN_TAN"]

    if state["tool_mode"] in shape_tools:
        is_closed = True
    
    # If it's a fixed line tool, we want to create exactly what's in preview_pts (usually 2 pts)
    elif state["tool_mode"] in complete_line_tools:
        is_closed = False
        # Do NOT discard any points for these tools
    
    elif state["tool_mode"] in continuous_tools:
        is_closed = False
        # Discard the last point if it is the "floating" mouse point.
        if len(pts) > 1 and not state.get("input_string"):
            pts = pts[:-1]

    bpy.ops.mesh.select_all(action='DESELECT')
    created_verts = []
    points_to_create = pts if not is_closed else pts[:-1] 
    
    for wp in points_to_create:
        v = bm.verts.new(imw @ wp)
        v.select = True 
        created_verts.append(v)
    created_edges = []
    for i in range(len(created_verts) - 1):
        v1, v2 = created_verts[i], created_verts[i+1]
        try: e = bm.edges.new((v1, v2))
        except ValueError: e = bm.edges.get((v1, v2))
        if e: 
            e.select = True
            created_edges.append(e)
    if is_closed and len(created_verts) > 2:
        v_last = created_verts[-1]
        v_first = created_verts[0]
        try: e = bm.edges.new((v_last, v_first))
        except ValueError: e = bm.edges.get((v_last, v_first))
        if e: 
            e.select = True
            created_edges.append(e)
    bm.select_history.clear()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    
    if state.get("auto_weld", True): 
        arc_weld_manager.run(ctx, created_verts, created_edges)
        
    bpy.ops.mesh.select_all(action='DESELECT')
    bmesh.update_edit_mesh(obj.data)

def begin_modal(self, ctx, ev):
    from .tool_previews import draw_cb_3d
    from .hud_overlay import draw_hud_2d 

    if ctx.area.type != 'VIEW_3D' or ctx.mode != 'EDIT_MESH':
        self.report({'WARNING'}, "Run in Edit Mode on a mesh")
        return {'CANCELLED'}
        
    # --- CURSOR FIX: FORCE 'DEFAULT' ARROW ---
    ctx.window.cursor_modal_set('DEFAULT')
    
    DrawManager.clear_all()
    reset_state_from_context(ctx)
    new_tool_id = f"{state['tool_mode']}_{time.time()}"
    self.tool_instance_id = new_tool_id
    ctx.scene.active_cad_tool_id = new_tool_id
    # Pass self (the operator) so ModalManager can report messages
    self.manager = ModalManager(ctx, self)
    self.manager.on_move(ctx, ev)
    
    DrawManager.add_handler('MAIN_3D', draw_cb_3d, (), 'WINDOW', 'POST_VIEW')
    DrawManager.add_handler('HUD_2D', draw_hud_2d, (), 'WINDOW', 'POST_PIXEL')
    
    ctx.window_manager.modal_handler_add(self)
    ctx.area.tag_redraw()
    return {'RUNNING_MODAL'}

def finish_modal(self, ctx):
    current_id = getattr(ctx.scene, "active_cad_tool_id", "")
    if current_id == self.tool_instance_id:
        # --- RESTORE CURSOR ---
        ctx.window.cursor_modal_restore()
        DrawManager.clear_all()
        state["active"] = False
        ctx.scene.active_cad_tool_id = ""
    ctx.area.tag_redraw()

def modal_arc_common(self, ctx, ev):
    from .text_entry_utils import handle_text_input
    
    current_id = getattr(ctx.scene, "active_cad_tool_id", "")
    if current_id != self.tool_instance_id: return {'CANCELLED'}

    if ev.type in {'LEFTMOUSE', 'RIGHTMOUSE', 'MOUSEMOVE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'MIDDLEMOUSE'}:
        reg = self.manager.region
        is_outside_viewport = False
        if reg:
            mx, my = ev.mouse_region_x, ev.mouse_region_y
            rw, rh = reg.width, reg.height
            if not (0 <= mx <= rw and 0 <= my <= rh): is_outside_viewport = True
        is_over_ui = is_event_over_ui(ctx, ev)
        
        # --- CURSOR FIX: Consume Moves over UI ---
        # Consuming MOUSEMOVE events prevents Blender from seeing the cursor on the 
        # UI border and swapping it to the 'resize' double-arrow icon.
        if is_outside_viewport or is_over_ui: 
            if ev.type == 'MOUSEMOVE':
                return {'RUNNING_MODAL'}
            else:
                return {'PASS_THROUGH'}

    # --- PRIORITY: Handle Text Input First ---
    if state["input_mode"] is not None:
        consumed = handle_text_input(ctx, ev)
        if consumed: 
             # If input finished (e.g. HIT ENTER), sync state back to tool
             if state["input_mode"] is None:
                 if self.manager.active_tool:
                     # --- FIX: Trigger one-frame bypass ---
                     state["skip_mouse_update"] = True
                     # Force immediate update with fresh coordinates
                     self.manager.on_move(ctx, ev)
                     
             return {'RUNNING_MODAL'}

    # --- Commit / Finish ---
    if (ev.type in {'SPACE', 'RET', 'NUMPAD_ENTER'} and ev.value == 'PRESS') or (ev.type == 'RIGHTMOUSE' and ev.value == 'PRESS'):
        if self.manager.active_tool:
            if state["tool_mode"] == "LINE_POLY":
                # If we have a keyboard value active, commit it as a click first
                if state.get("input_string"):
                    mx, my = ev.mouse_region_x, ev.mouse_region_y
                    snap_pt, snap_n = self.manager.get_snap_data(ctx, mx, my)
                    self.manager.active_tool.handle_click(ctx, ev, snap_pt, snap_n)
                    self.manager.on_move(ctx, ev)
            
            elif state["tool_mode"] == "CURVE_INTERPOLATE":
                # Build final curve including current mouse pos (snapped target),
                # then append a dummy copy of the last point so commit_arc_to_mesh's
                # strip-one-from-end eats the dummy instead of the real last edge.
                tool = self.manager.active_tool
                num_segs = state.get("segments", 12)
                if hasattr(tool, "_build_all_preview"):
                    pts = tool._build_all_preview(extra_pt=tool.current, num_segs=num_segs)
                    if pts:
                        state["preview_pts"] = pts + [pts[-1]]
                elif hasattr(tool, "control_points"):
                    from .operators.curve_tools import solve_catmull_rom_chain
                    state["preview_pts"] = solve_catmull_rom_chain(tool.control_points, num_segments=num_segs)

        commit_arc_to_mesh(ctx)
        finish_modal(self, ctx)
        return {'FINISHED'}

    if ev.type == 'ESC':
        finish_modal(self, ctx)
        return {'CANCELLED'}

    if ev.type == 'MIDDLEMOUSE':
        if state.get("pivot") is None: return {'PASS_THROUGH'}
        if ev.shift or ev.ctrl or ev.alt: return {'PASS_THROUGH'}
        if ev.value == 'PRESS':
            self.manager.is_navigating = True
            self.manager.last_mouse_x = ev.mouse_x
            self.manager.last_mouse_y = ev.mouse_y
            return {'RUNNING_MODAL'}
        elif ev.value == 'RELEASE':
            self.manager.is_navigating = False
            return {'RUNNING_MODAL'}

    if self.manager.is_navigating and ev.type == 'MOUSEMOVE':
        if state.get("pivot"):
            dx = ev.mouse_x - self.manager.last_mouse_x
            dy = ev.mouse_y - self.manager.last_mouse_y
            apply_custom_orbit(ctx, state["pivot"], dx, dy)
            self.manager.last_mouse_x = ev.mouse_x
            self.manager.last_mouse_y = ev.mouse_y
            return {'RUNNING_MODAL'}
        else: return {'PASS_THROUGH'}

    if ev.type in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}:
        if ev.ctrl:
            delta = 1 if ev.type == 'WHEELUPMOUSE' else -1
            bpy.ops.view3d.zoom('INVOKE_DEFAULT', delta=delta, use_cursor_init=True)
            return {'RUNNING_MODAL'} 

    if ev.type == 'WHEELUPMOUSE':
        if state.get("tool_mode") not in ["POINT_BY_ARCS", "LINE_POLY"]:
            step = 2 if state.get("tool_mode") == "POLYGON_EDGE" else 1
            state["segments"] = min(256, state["segments"] + step)
            if self.manager.active_tool: 
                self.manager.active_tool.segments = state["segments"]
                if hasattr(self.manager.active_tool, "refresh_preview"):
                    self.manager.active_tool.refresh_preview()
                self.manager.on_move(ctx, ev)
        ctx.area.tag_redraw()
        return {'RUNNING_MODAL'}
        
    if ev.type == 'WHEELDOWNMOUSE':
        if state.get("tool_mode") not in ["POINT_BY_ARCS", "LINE_POLY"]:
            step = 2 if state.get("tool_mode") == "POLYGON_EDGE" else 1
            state["segments"] = max(1 if "CURVE" in state.get("tool_mode", "") else 3, state["segments"] - step)
            if self.manager.active_tool: 
                self.manager.active_tool.segments = state["segments"]
                if hasattr(self.manager.active_tool, "refresh_preview"):
                    self.manager.active_tool.refresh_preview()
                self.manager.on_move(ctx, ev)
        ctx.area.tag_redraw()
        return {'RUNNING_MODAL'}

    if ev.type == 'MOUSEMOVE':
        self.manager.on_move(ctx, ev)
        # Force redraw while drawing freehand to keep it smooth
        if state.get("tool_mode") == "CURVE_FREEHAND" and getattr(self.manager.active_tool, "is_drawing", False):
            ctx.area.tag_redraw()
        return {'RUNNING_MODAL'}

    if ev.value == 'PRESS':
        if self.manager.active_tool:
            if self.manager.active_tool.handle_input(ctx, ev):
                self.manager.on_move(ctx, ev)
                return {'RUNNING_MODAL'}
        
        if ev.type == 'F1': state["snap_verts"] = not state.get("snap_verts", True); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
        if ev.type == 'F2': state["snap_edges"] = not state.get("snap_edges", False); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
        if ev.type == 'F3': state["snap_edge_center"] = not state.get("snap_edge_center", False); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
        if ev.type == 'F4': state["snap_face_center"] = not state.get("snap_face_center", False); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}

        if ev.type == 'C': state["use_angle_snap"] = not state.get("use_angle_snap", True); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
        if ev.type == 'W': state["auto_weld"] = not state.get("auto_weld", True); ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
        
        if ev.type == 'L':
            if state.get("locked"):
                state["locked"] = False
                state["locked_normal"] = None
                self.report({'INFO'}, "Unlocked")
            else:
                n = state.get("last_surface_normal")
                if n:
                    state["locked"] = True
                    state["locked_normal"] = n
                    self.report({'INFO'}, "Locked to Normal")
                else:
                    self.report({'WARNING'}, "No Normal to Lock To")
            ctx.area.tag_redraw()
            return {'RUNNING_MODAL'}

        target_mode = None
        tool_mode = state.get("tool_mode", "1POINT")
        
        if ev.type == 'S': target_mode = 'SEGMENTS'
        elif ev.type == 'M' and tool_mode == "CURVE_FREEHAND": target_mode = 'MIN_DIST'
        elif ev.type == 'R' and tool_mode != "ELLIPSE_CORNERS":
            if tool_mode != "ELLIPSE_FOCI" or state["stage"] == 1:
                target_mode = 'RADIUS'; state["input_target"] = 'RADIUS'
        elif ev.type == 'D' and tool_mode in ["2POINT", "CIRCLE_2POINT", "ELLIPSE_ENDPOINTS", "ELLIPSE_RADIUS"]: target_mode = 'RADIUS'; state["input_target"] = 'DIAMETER'
        elif ev.type == 'H' and tool_mode == "2POINT" and state["stage"] == 2: target_mode = 'RADIUS'; state["input_target"] = 'SAGITTA'
        elif ev.type == 'A' and tool_mode == "POLYGON_CENTER_TANGENT": target_mode = 'RADIUS'; state["input_target"] = 'RADIUS'
        elif ev.type == 'L' and tool_mode in ["POLYGON_CORNER_CORNER", "POLYGON_EDGE", "LINE_POLY"]: target_mode = 'RADIUS'; state["input_target"] = 'RADIUS'
        elif ev.type == 'A' and state["stage"] == 2 and tool_mode not in ["2POINT", "CIRCLE_TAN_TAN_TAN", "LINE_POLY", "ELLIPSE_CORNERS", "ELLIPSE_ENDPOINTS", "ELLIPSE_FOCI"]:
            target_mode = 'ANGLE'        
        if is_number_input(ev): 
            # --- FIX: Context-aware number typing ---
            is_angle_stage = False
            if tool_mode == "1POINT" and state["stage"] == 2:
                is_angle_stage = True
            elif tool_mode == "POINT_BY_ARCS" and state["stage"] in [2, 5]:
                is_angle_stage = True
                
            if is_angle_stage:
                target_mode = 'ANGLE'
            elif tool_mode == "CURVE_FREEHAND":
                target_mode = 'MIN_DIST'
            elif tool_mode != "ELLIPSE_CORNERS":
                if tool_mode != "ELLIPSE_FOCI" or state["stage"] == 1:
                    target_mode = 'RADIUS' # Covers 2POINT Sagitta automatically as it's in Stage 2 but not an angle stage
                    if tool_mode == "2POINT" and state["stage"] == 2: state["input_target"] = 'SAGITTA'
                    elif tool_mode in ["CIRCLE_2POINT", "ELLIPSE_RADIUS", "ELLIPSE_ENDPOINTS"] and state["stage"] == 1: state["input_target"] = 'DIAMETER'
                    else: state["input_target"] = 'RADIUS'
            
        if target_mode:
            if self.manager.region and self.manager.rv3d and state["pivot"]:
                p2d = location_3d_to_region_2d(self.manager.region, self.manager.rv3d, state["pivot"])
                if p2d:
                    if state["input_mode"] is None: state["input_string"] = ""; state["cursor_index"] = 0
                    state["input_screen_pos"] = (p2d.x + 25, p2d.y + 10)
                    state["input_mode"] = target_mode
                    if is_number_input(ev): handle_text_input(ctx, ev)
                    ctx.area.tag_redraw()
                    return {'RUNNING_MODAL'}

        if ev.type == 'LEFTMOUSE' or ev.type in {'RET', 'NUMPAD_ENTER'}:
            mx, my = ev.mouse_region_x, ev.mouse_region_y
            clicked_ui_id = None
            for k, v in state["ui_hitboxes"].items():
                xmin, xmax, ymin, ymax = v
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    clicked_ui_id = k
                    if k == "snap_verts": state["snap_verts"] = not state.get("snap_verts", False)
                    elif k == "snap_edges": state["snap_edges"] = not state.get("snap_edges", False)
                    elif k == "snap_edge_center": state["snap_edge_center"] = not state.get("snap_edge_center", False)
                    elif k == "snap_face_center": state["snap_face_center"] = not state.get("snap_face_center", False)
                    elif k == "toggle_angle": state["use_angle_snap"] = not state.get("use_angle_snap", True)
                    elif k == "weld_btn": state["auto_weld"] = not state.get("auto_weld", True)
                    ctx.area.tag_redraw(); return {'RUNNING_MODAL'}
            
            if self.manager.active_tool:
                 snap_pt, snap_n = self.manager.get_snap_data(ctx, mx, my)
                 result = self.manager.active_tool.handle_click(ctx, ev, snap_pt, snap_n, button_id=clicked_ui_id)
                 state["stage"] = self.manager.active_tool.stage
                 if result == 'FINISHED':
                     commit_arc_to_mesh(ctx)
                     finish_modal(self, ctx)
                     return {'FINISHED'}
                 elif result == 'NEXT_STAGE':
                     self.manager.on_move(ctx, ev)
                     ctx.area.tag_redraw()
                     return {'RUNNING_MODAL'}

    elif ev.type == 'RIGHTMOUSE':
        finish_modal(self, ctx)
        return {'CANCELLED'}

    return {'RUNNING_MODAL'}

class VIEW3D_OT_radcad_modal(bpy.types.Operator):
    bl_idname = "view3d.radcad_modal"
    bl_label = "CAD Drawing Modal"
    bl_options = {'REGISTER', 'UNDO'}

    def modal(self, context, event):
        return modal_arc_common(self, context, event)

    def invoke(self, context, event):
        return begin_modal(self, context, event)

    def cancel(self, context):
        finish_modal(self, context)