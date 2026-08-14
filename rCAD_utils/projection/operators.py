import bmesh
import bpy
from mathutils.bvhtree import BVHTree

from .storage import SourceEntry, projection_source


_RAY_EPSILON = 1.0e-5
_MAX_RAY_DISTANCE = 1.0e30


def _edit_mesh_objects(context):
    objects = getattr(context, "objects_in_mode_unique_data", None)
    if objects is None:
        objects = [
            obj for obj in context.view_layer.objects
            if obj.type == 'MESH' and obj.mode == 'EDIT'
        ]
    return [obj for obj in objects if obj.type == 'MESH' and obj.mode == 'EDIT']


def _prepare_bmesh(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.index_update()
    bm.faces.index_update()
    return bm


def _deselect_bmesh(bm):
    for face in bm.faces:
        face.select_set(False)
    for edge in bm.edges:
        edge.select_set(False)
    for vert in bm.verts:
        vert.select_set(False)
    bm.select_history.clear()


def _stored_entries_are_valid():
    if not projection_source.has_source():
        return False, "No source geometry stored. Select geometry and press Store first."

    for entry in projection_source.entries:
        try:
            obj = entry.obj
            if obj.type != 'MESH' or obj.data != entry.mesh:
                return False, "Stored source geometry is no longer available. Store it again."
            if obj.mode != 'EDIT':
                return False, "Stored source objects must remain in Edit Mode."

            bm = _prepare_bmesh(obj)
            current_counts = (len(bm.verts), len(bm.edges), len(bm.faces))
            stored_counts = (entry.vertex_count, entry.edge_count, entry.face_count)
            if current_counts != stored_counts:
                return False, "Source topology changed after Store. Store the source geometry again."
            if any(index < 0 or index >= len(bm.verts) for index in entry.vertex_indices):
                return False, "Stored source vertices are no longer valid. Store them again."
        except (ReferenceError, RuntimeError):
            return False, "Stored source geometry is no longer available. Store it again."

    return True, ""


def _selected_target_faces(context):
    selected = {}
    for obj in _edit_mesh_objects(context):
        bm = _prepare_bmesh(obj)
        face_indices = {
            face.index for face in bm.faces
            if face.select and not face.hide
        }

        if not face_indices:
            selected_edges = [edge for edge in bm.edges if edge.select and not edge.hide]
            if selected_edges:
                face_indices.update(
                    face.index
                    for edge in selected_edges
                    for face in edge.link_faces
                    if not face.hide
                )
            else:
                selected_verts = [vert for vert in bm.verts if vert.select and not vert.hide]
                face_indices.update(
                    face.index
                    for vert in selected_verts
                    for face in vert.link_faces
                    if not face.hide
                )

        if face_indices:
            selected[obj] = tuple(sorted(face_indices))
    return selected


def _append_bmesh_faces(vertices, polygons, obj, face_indices=None):
    bm = _prepare_bmesh(obj)
    matrix = obj.matrix_world
    vertex_map = {}

    if face_indices is None:
        faces = (face for face in bm.faces if not face.hide)
    else:
        faces = (
            bm.faces[index] for index in face_indices
            if 0 <= index < len(bm.faces)
        )

    for face in faces:
        if not face.is_valid or face.hide:
            continue
        polygon = []
        for vert in face.verts:
            mapped_index = vertex_map.get(vert.index)
            if mapped_index is None:
                mapped_index = len(vertices)
                vertex_map[vert.index] = mapped_index
                vertices.append(matrix @ vert.co)
            polygon.append(mapped_index)
        if len(polygon) >= 3:
            polygons.append(polygon)


def _append_evaluated_mesh(vertices, polygons, obj, depsgraph):
    evaluated_obj = obj.evaluated_get(depsgraph)
    mesh = evaluated_obj.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    if mesh is None:
        return

    try:
        offset = len(vertices)
        matrix = evaluated_obj.matrix_world
        vertices.extend(matrix @ vert.co for vert in mesh.vertices)
        polygons.extend(
            [offset + index for index in polygon.vertices]
            for polygon in mesh.polygons
            if len(polygon.vertices) >= 3
        )
    finally:
        evaluated_obj.to_mesh_clear()


def _object_is_visible(context, obj):
    if obj.hide_viewport or obj.hide_get():
        return False
    try:
        return obj.visible_get(view_layer=context.view_layer, viewport=context.space_data)
    except TypeError:
        return obj.visible_get(view_layer=context.view_layer)


def _build_target_tree(context, selected_faces):
    vertices = []
    polygons = []

    if selected_faces:
        for obj, face_indices in selected_faces.items():
            _append_bmesh_faces(vertices, polygons, obj, face_indices)
    else:
        depsgraph = context.evaluated_depsgraph_get()
        for obj in context.view_layer.objects:
            if obj.type != 'MESH' or not _object_is_visible(context, obj):
                continue
            if obj.mode == 'EDIT':
                _append_bmesh_faces(vertices, polygons, obj)
            else:
                _append_evaluated_mesh(vertices, polygons, obj, depsgraph)

    if not polygons:
        return None
    return BVHTree.FromPolygons(vertices, polygons, all_triangles=False, epsilon=0.0)


def _build_selected_target_surfaces(selected_faces):
    """Build one ray-cast surface per explicitly selected edit-mode face.

    Keeping each face separate lets explicit projection follow that face's
    normal, matching the plane-intersection behavior of the legacy tool.
    """
    surfaces = []
    for obj, face_indices in selected_faces.items():
        bm = _prepare_bmesh(obj)
        bm.normal_update()
        matrix = obj.matrix_world
        normal_matrix = matrix.to_3x3().inverted_safe().transposed()

        for index in face_indices:
            if index < 0 or index >= len(bm.faces):
                continue
            face = bm.faces[index]
            if not face.is_valid or face.hide or len(face.verts) < 3:
                continue

            vertices = [matrix @ vert.co for vert in face.verts]
            normal = normal_matrix @ face.normal
            if normal.length_squared == 0.0:
                continue
            normal.normalize()

            tree = BVHTree.FromPolygons(
                vertices,
                [list(range(len(vertices)))],
                all_triangles=False,
                epsilon=0.0,
            )
            surfaces.append((tree, normal))

    return surfaces


def _view_projection(context):
    space = context.space_data
    region_3d = getattr(space, "region_3d", None) if space else None
    if region_3d is None:
        return None

    view_inverse = region_3d.view_matrix.inverted_safe()
    into_view = -view_inverse.col[2].to_3d()
    if into_view.length_squared == 0.0:
        return None

    return region_3d.is_perspective, view_inverse.translation.copy(), into_view.normalized()


def _closest_view_line_hit(target_tree, source_world, direction):
    closest_location = None
    closest_distance_squared = float('inf')

    for ray_direction in (direction, -direction):
        ray_origin = source_world + ray_direction * _RAY_EPSILON
        location, _normal, _face_index, _distance = target_tree.ray_cast(
            ray_origin,
            ray_direction,
            _MAX_RAY_DISTANCE,
        )
        if location is None:
            continue

        distance_squared = (location - source_world).length_squared
        if distance_squared < closest_distance_squared:
            closest_location = location.copy()
            closest_distance_squared = distance_squared

    return closest_location


def _closest_target_normal_hit(target_surfaces, source_world):
    closest_location = None
    closest_distance_squared = float('inf')

    for target_tree, normal in target_surfaces:
        for direction in (normal, -normal):
            ray_origin = source_world + direction * _RAY_EPSILON
            location, _normal, _face_index, _distance = target_tree.ray_cast(
                ray_origin,
                direction,
                _MAX_RAY_DISTANCE,
            )
            if location is None:
                continue

            distance_squared = (location - source_world).length_squared
            if distance_squared < closest_distance_squared:
                closest_location = location.copy()
                closest_distance_squared = distance_squared

    return closest_location


def _restore_source_selection(context):
    edit_objects = _edit_mesh_objects(context)
    for obj in edit_objects:
        _deselect_bmesh(_prepare_bmesh(obj))

    for entry in projection_source.entries:
        bm = _prepare_bmesh(entry.obj)
        for index in entry.vertex_indices:
            bm.verts[index].select_set(True)
        for index in entry.edge_indices:
            if 0 <= index < len(bm.edges):
                bm.edges[index].select_set(True)
        for index in entry.face_indices:
            if 0 <= index < len(bm.faces):
                bm.faces[index].select_set(True)

    for obj in edit_objects:
        bm = _prepare_bmesh(obj)
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)


class RCAD_OT_StoreProjectionSource(bpy.types.Operator):
    bl_idname = "mesh.rcad_store_projection_source"
    bl_label = "Store"
    bl_description = "Store selected edit-mode geometry as the projection source"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and bool(_edit_mesh_objects(context))

    def execute(self, context):
        entries = []
        edit_objects = _edit_mesh_objects(context)

        for obj in edit_objects:
            bm = _prepare_bmesh(obj)
            vertex_indices = tuple(vert.index for vert in bm.verts if vert.select)
            if not vertex_indices:
                continue

            entries.append(SourceEntry(
                obj=obj,
                mesh=obj.data,
                vertex_indices=vertex_indices,
                edge_indices=tuple(edge.index for edge in bm.edges if edge.select),
                face_indices=tuple(face.index for face in bm.faces if face.select),
                vertex_count=len(bm.verts),
                edge_count=len(bm.edges),
                face_count=len(bm.faces),
            ))

        if not entries:
            self.report({'WARNING'}, "Select source vertices, edges, or faces in Edit Mode.")
            return {'CANCELLED'}

        projection_source.store(entries)
        for obj in edit_objects:
            bm = _prepare_bmesh(obj)
            _deselect_bmesh(bm)
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

        self.report(
            {'INFO'},
            f"Stored {projection_source.vertex_count} source vertices. Select target geometry or press Project in Orthographic view.",
        )
        return {'FINISHED'}


class RCAD_OT_ProjectStoredGeometry(bpy.types.Operator):
    bl_idname = "mesh.rcad_project_stored_geometry"
    bl_label = "Project"
    bl_description = "Project stored geometry onto selected target geometry, or through the view onto visible meshes in Orthographic view"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and projection_source.has_source()

    def execute(self, context):
        valid, message = _stored_entries_are_valid()
        if not valid:
            self.report({'WARNING'}, message)
            return {'CANCELLED'}

        view_projection = _view_projection(context)
        if view_projection is None:
            self.report({'ERROR'}, "Projection must be run from a 3D View.")
            return {'CANCELLED'}

        is_perspective, view_origin, ortho_direction = view_projection
        selected_faces = _selected_target_faces(context)
        if is_perspective and not selected_faces:
            self.report({'WARNING'}, "Perspective projection requires selected target geometry.")
            return {'CANCELLED'}

        target_tree = _build_target_tree(context, selected_faces)
        if target_tree is None:
            target_description = "selected target geometry" if selected_faces else "visible mesh faces"
            self.report({'WARNING'}, f"No {target_description} are available for projection.")
            return {'CANCELLED'}
        target_surfaces = _build_selected_target_surfaces(selected_faces)

        hits = []
        missed = 0
        for entry in projection_source.entries:
            bm = _prepare_bmesh(entry.obj)
            matrix = entry.obj.matrix_world
            for index in entry.vertex_indices:
                source_world = matrix @ bm.verts[index].co
                if is_perspective:
                    direction = source_world - view_origin
                    if direction.length_squared == 0.0:
                        missed += 1
                        continue
                    direction.normalize()
                else:
                    direction = ortho_direction

                if selected_faces:
                    location = _closest_target_normal_hit(target_surfaces, source_world)
                else:
                    location = _closest_view_line_hit(target_tree, source_world, direction)

                if location is None and selected_faces:
                    nearest_location, _normal, _face_index, _distance = target_tree.find_nearest(
                        source_world
                    )
                    if nearest_location is not None:
                        location = nearest_location.copy()
                if location is None:
                    missed += 1
                    continue
                hits.append((entry, index, location))

        if not hits:
            self.report({'WARNING'}, "No stored vertices intersected a target face along the view line.")
            return {'CANCELLED'}

        inverse_matrices = {
            id(entry): entry.obj.matrix_world.inverted_safe()
            for entry in projection_source.entries
        }
        changed_objects = set()
        for entry, index, location in hits:
            bm = _prepare_bmesh(entry.obj)
            bm.verts[index].co = inverse_matrices[id(entry)] @ location
            changed_objects.add(entry.obj)

        for obj in changed_objects:
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

        _restore_source_selection(context)

        if missed:
            self.report({'WARNING'}, f"Projected {len(hits)} vertices; {missed} had no target hit.")
        else:
            self.report({'INFO'}, f"Projected {len(hits)} vertices.")
        return {'FINISHED'}


classes = (
    RCAD_OT_StoreProjectionSource,
    RCAD_OT_ProjectStoredGeometry,
)
