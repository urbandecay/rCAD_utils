import bpy
import bmesh
import math
import numpy as np
from mathutils import Vector, Matrix
from bpy.props import BoolProperty, FloatProperty
import traceback

# Scene properties keys/names
SCENE_PROFILE_RAW_LOCALS = "profile_raw_locals"
SCENE_PROFILE_ANCHOR_INDEX = "profile_anchor_index"
SCENE_PROFILE_EDGES = "profile_edges"
SCENE_PROFILE_FACES = "profile_faces"
SCENE_PROFILE_SOURCE_EDGES = "profile_source_edge_indices"
SCENE_PROFILE_SOURCE_VERTS = "profile_source_vert_indices"
SCENE_PROFILE_ANCHOR_EDGE_VERTS = "profile_anchor_edge_verts"

SCENE_PROFILE_ROT_WIDTH = "profile_rot_width"
SCENE_PROFILE_ROT_HEIGHT = "profile_rot_height"

SCENE_PROPERTY_KEYS_TO_DELETE = [
    SCENE_PROFILE_RAW_LOCALS,
    SCENE_PROFILE_ANCHOR_INDEX,
    SCENE_PROFILE_EDGES,
    SCENE_PROFILE_FACES,
    SCENE_PROFILE_SOURCE_EDGES,
    SCENE_PROFILE_SOURCE_VERTS,
    SCENE_PROFILE_ANCHOR_EDGE_VERTS,
]


def calculate_pca_normal_from_coords(coords_list):
    if len(coords_list) < 3:
        return None
    coords_np = np.array([list(co) for co in coords_list])
    center = np.mean(coords_np, axis=0)
    covariance_matrix = np.cov(coords_np - center, rowvar=False)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        normal = Vector(eigenvectors[:, np.argmin(eigenvalues)])
        return normal.normalized() if normal.length > 1e-9 else None
    except Exception as e:
        print(f"PCA Error: {e}")
        return None


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def orient_profile_data(raw_coords, anchor_index, edge_indices_list):
    status_msgs = []
    num_verts = len(raw_coords)
    if num_verts < 2:
        return False, ["Need >= 2 vertices."], None, None

    if not (0 <= anchor_index < num_verts):
        msg = f"Invalid anchor index: {anchor_index} for {num_verts} verts"
        return False, [msg], None, None

    working_coords = [co.copy() for co in raw_coords]
    pivot_coord = working_coords[anchor_index].copy()
    coords_for_pca = [co - pivot_coord for co in working_coords]

    current_normal = (
        calculate_pca_normal_from_coords(coords_for_pca)
        if len(coords_for_pca) >= 3
        else None
    )
    flatten_matrix = Matrix.Identity(4)
    if current_normal is None:
        status_msgs.append("PCA failed; Projected Y.")
    elif current_normal.length < 1e-9:
        status_msgs.append("Zero normal; Projected Y.")
    else:
        target_normal = Vector((0.0, 1.0, 0.0))
        if abs(current_normal.normalized().dot(target_normal)) < 0.99999:
            rot_diff = current_normal.rotation_difference(target_normal)
            flatten_matrix = rot_diff.to_matrix().to_4x4()
            status_msgs.append("Flattened (PCA).")
        else:
            status_msgs.append("Already flat.")

    coords_after_flatten = [flatten_matrix @ (co - pivot_coord) for co in working_coords]
    coords_after_flatten = [Vector((c.x, 0.0, c.z)) for c in coords_after_flatten]
    working_coords = [co + pivot_coord for co in coords_after_flatten]
    pivot_coord = working_coords[anchor_index]

    neighbor_indices = {
        idx2 if idx1 == anchor_index else idx1
        for idx1, idx2 in edge_indices_list
        if idx1 == anchor_index or idx2 == anchor_index
    }

    align_neighbor_coord = None
    if neighbor_indices:
        first_neighbor_index = sorted(neighbor_indices)[0]
        if 0 <= first_neighbor_index < len(working_coords):
            align_neighbor_coord = working_coords[first_neighbor_index]

    if align_neighbor_coord:
        edge_vec = align_neighbor_coord - pivot_coord
        proj_xz = Vector((edge_vec.x, edge_vec.z))
        correction = 0.0
        if proj_xz.length >= 1e-6:
            current_angle = math.atan2(proj_xz.y, proj_xz.x)
            angle_to_up = math.pi / 2 - current_angle
            angle_to_down = -math.pi / 2 - current_angle
            correction = (
                angle_to_up
                if abs(normalize_angle(angle_to_up))
                <= abs(normalize_angle(angle_to_down))
                else angle_to_down
            )

        if abs(correction) > 1e-6:
            R_corr = Matrix.Rotation(-correction, 4, 'Y')
            to_pivot = Matrix.Translation(-pivot_coord)
            from_pivot = Matrix.Translation(pivot_coord)
            corr_matrix = from_pivot @ R_corr @ to_pivot
            working_coords = [corr_matrix @ co for co in working_coords]
            pivot_coord = working_coords[anchor_index]
            status_msgs.append("Aligned edge.")
        else:
            status_msgs.append("Edge already aligned.")
    else:
        status_msgs.append("No neighbor found; skipped edge align.")

    oriented_anchor_global = working_coords[anchor_index]
    final_relative_coords = [co - oriented_anchor_global for co in working_coords]
    status_msgs.append("Oriented data to place-ready.")
    return True, status_msgs, final_relative_coords, Vector((0.0, 0.0, 0.0))


class OBJECT_OT_store_profile_info_edit(bpy.types.Operator):
    bl_idname = "object.store_profile_info_edit"
    bl_label = "Store Profile Shape"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        scene = context.scene

        if not obj or obj.type != 'MESH' or context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Need active Mesh in Edit Mode.")
            return {'CANCELLED'}

        for key in SCENE_PROPERTY_KEYS_TO_DELETE:
            if key in scene:
                del scene[key]

        bm = bmesh.from_edit_mesh(obj.data)
        sel_verts = {v for v in bm.verts if v.select and v.is_valid}
        if not sel_verts:
            self.report({'ERROR'}, "Select profile vertices!")
            return {'CANCELLED'}

        sel_list = sorted(list(sel_verts), key=lambda v: v.index)
        v_map = {v: i for i, v in enumerate(sel_list)}

        active_elem = bm.select_history.active
        active_vert = None

        if isinstance(active_elem, bmesh.types.BMEdge) and all(v in sel_verts for v in active_elem.verts):
            v1, v2 = active_elem.verts
            active_vert = v1
            scene[SCENE_PROFILE_ANCHOR_EDGE_VERTS] = [v_map[v1], v_map[v2]]
        elif isinstance(active_elem, bmesh.types.BMVert) and active_elem in sel_verts:
            active_vert = active_elem
        else:
            active_vert = next(
                (elem for elem in reversed(bm.select_history)
                 if isinstance(elem, bmesh.types.BMVert) and elem in sel_verts),
                next(iter(sel_verts), None)
            )

        if not active_vert:
            self.report({'ERROR'}, "Cannot determine anchor vertex!")
            return {'CANCELLED'}

        anchor_co = active_vert.co.copy()
        raw_coords = [v.co - anchor_co for v in sel_list]
        anchor_idx = v_map[active_vert]

        sel_edges = {
            e for e in bm.edges if e.select and e.is_valid and all(v in sel_verts for v in e.verts)
        }
        edges = [
            sorted([v_map[v1], v_map[v2]])
            for e in sel_edges
            for v1, v2 in [e.verts]
            if v1 in v_map and v2 in v_map
        ]
        sel_faces = {
            f for f in bm.faces if f.select and f.is_valid and all(v in sel_verts for v in f.verts)
        }
        faces = [[v_map[v] for v in f.verts if v in v_map] for f in sel_faces]

        scene[SCENE_PROFILE_RAW_LOCALS] = [[c.x, c.y, c.z] for c in raw_coords]
        scene[SCENE_PROFILE_ANCHOR_INDEX] = anchor_idx
        scene[SCENE_PROFILE_EDGES] = [list(vals) for vals in {tuple(p) for p in edges}]
        scene[SCENE_PROFILE_FACES] = faces
        scene[SCENE_PROFILE_SOURCE_EDGES] = list({e.index for e in sel_edges})
        scene[SCENE_PROFILE_SOURCE_VERTS] = list({v.index for v in sel_verts})
        return {'FINISHED'}


class OBJECT_OT_profile_rotate_axis(bpy.types.Operator):
    bl_idname = "object.profile_rotate_axis"
    bl_label = "Rotate Profile Axis"
    bl_options = {'INTERNAL'}

    axis: BoolProperty(name="Height Axis", default=False)  # False=width, True=height
    delta: FloatProperty(name="Delta", default=15.0)

    def execute(self, context):
        scene = context.scene
        key = SCENE_PROFILE_ROT_HEIGHT if self.axis else SCENE_PROFILE_ROT_WIDTH
        scene[key] = (scene.get(key, 0.0) + self.delta) % 360.0
        bpy.ops.object.place_profile_on_edges_edit('EXEC_DEFAULT')
        return {'FINISHED'}


class OBJECT_OT_place_profile_on_edges_edit(bpy.types.Operator):
    bl_idname = "object.place_profile_on_edges_edit"
    bl_label = "Orient, Place & Delete"
    bl_options = {'REGISTER', 'UNDO'}

    flip_width: FloatProperty(name="Width Rotation", default=0.0)
    flip_height: FloatProperty(name="Height Rotation", default=0.0)
    use_alt_anchor: BoolProperty(name="Use Anchor B", default=False)

    def draw(self, context):
        layout = self.layout
        rot_w = context.scene.get(SCENE_PROFILE_ROT_WIDTH, 0.0)
        rot_h = context.scene.get(SCENE_PROFILE_ROT_HEIGHT, 0.0)

        row = layout.row(align=True)
        row.label(text=f"Width: {rot_w:.0f}°")
        op = row.operator("object.profile_rotate_axis", text="◄")
        op.axis = False
        op.delta = -15.0
        op = row.operator("object.profile_rotate_axis", text="►")
        op.axis = False
        op.delta = 15.0

        row = layout.row(align=True)
        row.label(text=f"Height: {rot_h:.0f}°")
        op = row.operator("object.profile_rotate_axis", text="◄")
        op.axis = True
        op.delta = -15.0
        op = row.operator("object.profile_rotate_axis", text="►")
        op.axis = True
        op.delta = 15.0

        if SCENE_PROFILE_ANCHOR_EDGE_VERTS in context.scene:
            button_text = "Anchor: Point B" if self.use_alt_anchor else "Anchor: Point A"
            layout.prop(self, "use_alt_anchor", text=button_text, toggle=True)

    def execute(self, context):
        obj = context.active_object
        scene = context.scene
        mesh = obj.data

        try:
            raw_coords = [Vector(s) for s in scene[SCENE_PROFILE_RAW_LOCALS]]
            default_anchor_idx = int(scene[SCENE_PROFILE_ANCHOR_INDEX])
            anchor_edge_verts = scene.get(SCENE_PROFILE_ANCHOR_EDGE_VERTS)
            edges = scene.get(SCENE_PROFILE_EDGES, [])
        except Exception as e:
            self.report(
                {'ERROR'},
                f"Stored data invalid: {e}. You MUST use 'Store Profile Shape' first."
            )
            return {'CANCELLED'}

        anchor_idx = default_anchor_idx
        if self.use_alt_anchor and anchor_edge_verts:
            option_a, option_b = anchor_edge_verts
            anchor_idx = option_b if default_anchor_idx == option_a else option_a

        ok, status_msgs, oriented_coords, oriented_anchor = orient_profile_data(
            raw_coords, anchor_idx, edges
        )
        if not ok:
            self.report({'ERROR'}, f"Data orientation failed for anchor {anchor_idx}.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(mesh)
        bm.verts.ensure_lookup_table()

        calc_start_vert = calc_middle_vert = calc_end_vert = None
        if scene.profile_path_mode:
            selected_edges = [e for e in bm.edges if e.select and e.is_valid]
            selected_verts = [v for v in bm.verts if v.select and v.is_valid]
            active_elem = bm.select_history.active

            if isinstance(active_elem, bmesh.types.BMVert):
                if len(selected_verts) != 3:
                    self.report({'ERROR'}, "Vertex Mode needs 3 selected vertices.")
                    return {'CANCELLED'}
                calc_start_vert = active_elem
                other_verts = [v for v in selected_verts if v != calc_start_vert]
                middle = next(
                    (edge.other_vert(calc_start_vert)
                     for edge in calc_start_vert.link_edges
                     if edge.other_vert(calc_start_vert) in other_verts),
                    None
                )
                if not middle:
                    self.report({'ERROR'}, "Active vert not connected to other selected verts.")
                    return {'CANCELLED'}
                calc_middle_vert = middle
                calc_end_vert = next(v for v in other_verts if v != calc_middle_vert)
            elif isinstance(active_elem, bmesh.types.BMEdge):
                if len(selected_edges) != 2:
                    self.report({'ERROR'}, "Edge Mode needs 2 selected edges.")
                    return {'CANCELLED'}
                active_edge = active_elem
                helper_edge = next(e for e in selected_edges if e != active_edge)
                common_verts = set(active_edge.verts).intersection(helper_edge.verts)
                if len(common_verts) != 1:
                    self.report({'ERROR'}, "Edges must share one vertex.")
                    return {'CANCELLED'}
                calc_middle_vert = common_verts.pop()
                calc_start_vert = active_edge.other_vert(calc_middle_vert)
                calc_end_vert = helper_edge.other_vert(calc_middle_vert)
            else:
                self.report({'ERROR'}, "Path Mode needs an active vert or edge.")
                return {'CANCELLED'}

        placements = []
        if calc_start_vert and calc_middle_vert and calc_end_vert:
            P = calc_start_vert.co.copy()
            d = (calc_middle_vert.co - calc_start_vert.co).normalized()
            helper_vec = (calc_end_vert.co - calc_middle_vert.co).normalized()
            world_up = Vector((0, 0, 1))
            if abs(helper_vec.dot(d)) > 0.999:
                up = (world_up - d * d.dot(world_up)).normalized()
            else:
                up = (helper_vec - d * helper_vec.dot(d)).normalized()
            if up.length < 1e-6:
                up = (world_up - d * d.dot(world_up)).normalized()
            right = up.cross(d).normalized()
            R = Matrix((right, d, up)).transposed()
            T = P - (R @ oriented_anchor)
            placements.append({'R': R, 'T': T})
        elif not scene.profile_path_mode:
            targets = [
                e for e in bm.edges
                if e.select and e.is_valid and e.index not in scene.get(SCENE_PROFILE_SOURCE_EDGES, [])
            ]
            if not targets:
                self.report({'ERROR'}, "No valid target edges selected.")
                return {'CANCELLED'}
            for edge in targets:
                v1, v2 = edge.verts
                P = v1.co.copy()
                d = (v2.co - v1.co).normalized()
                if edge.link_faces:
                    face_normal = edge.link_faces[0].normal.normalized()
                else:
                    world_up = Vector((0, 0, 1))
                    face_normal = world_up if abs(d.dot(world_up)) < 0.999 else Vector((0, 1, 0))
                up = d.cross(face_normal).normalized()
                if up.length < 1e-6:
                    world_up = Vector((0, 0, 1))
                    face_normal = world_up if abs(d.dot(world_up)) < 0.999 else Vector((0, 1, 0))
                    up = d.cross(face_normal).normalized()
                right = up.cross(d).normalized()
                R = Matrix((right, d, up)).transposed()
                T = P - (R @ oriented_anchor)
                placements.append({'R': R, 'T': T})

        if not placements:
            self.report({'ERROR'}, "Could not calculate any valid placements.")
            return {'CANCELLED'}

        rot_w = Matrix.Rotation(math.radians(scene.get(SCENE_PROFILE_ROT_WIDTH, 0.0)), 3, 'Y')
        rot_h = Matrix.Rotation(math.radians(scene.get(SCENE_PROFILE_ROT_HEIGHT, 0.0)), 3, 'X')
        coords = [rot_h @ rot_w @ v for v in oriented_coords]

        for placement in placements:
            R, T = placement['R'], placement['T']
            instance_verts = [bm.verts.new(T + R @ v) for v in coords]

            for i1, i2 in scene.get(SCENE_PROFILE_EDGES, []):
                if 0 <= i1 < len(instance_verts) and 0 <= i2 < len(instance_verts):
                    try:
                        bm.edges.new((instance_verts[i1], instance_verts[i2]))
                    except ValueError:
                        pass

            for f_idx in scene.get(SCENE_PROFILE_FACES, []):
                vs = [instance_verts[i] for i in f_idx if 0 <= i < len(instance_verts)]
                if len(vs) >= 3:
                    try:
                        bm.faces.new(vs)
                    except ValueError:
                        pass

        src_v_set = set(scene.get(SCENE_PROFILE_SOURCE_VERTS, []))
        if src_v_set:
            bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.index in src_v_set and v.is_valid], context='VERTS')

        bmesh.update_edit_mesh(mesh, destructive=True)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.scene[SCENE_PROFILE_ROT_WIDTH] = 0.0
        context.scene[SCENE_PROFILE_ROT_HEIGHT] = 0.0
        return self.execute(context)
