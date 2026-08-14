"""Blender operators for the explicit source/target projection workflow."""

import hashlib
import math
import struct

import bmesh
import bpy
from mathutils import Vector

from .engine import ProjectionError, build_surface_components, project_points_coherently
from .storage import SourceEntry, TargetEntry, projection_state


_MATRIX_EPSILON = 1.0e-12
_MOVE_EPSILON_SQUARED = 1.0e-20


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


def _canonical_face(indices):
    values = tuple(indices)
    if not values:
        return values
    rotations = []
    for sequence in (values, tuple(reversed(values))):
        minimum = min(sequence)
        rotations.extend(
            sequence[offset:] + sequence[:offset]
            for offset, value in enumerate(sequence)
            if value == minimum
        )
    return min(rotations)


def _topology_signature(obj):
    """Create a compact fingerprint that detects index-invalidating edits."""
    if obj.mode == 'EDIT':
        bm = _prepare_bmesh(obj)
        vertex_count = len(bm.verts)
        edges = [tuple(sorted((edge.verts[0].index, edge.verts[1].index))) for edge in bm.edges]
        faces = [_canonical_face(tuple(vert.index for vert in face.verts)) for face in bm.faces]
    else:
        mesh = obj.data
        vertex_count = len(mesh.vertices)
        edges = [tuple(sorted(edge.vertices)) for edge in mesh.edges]
        faces = [_canonical_face(tuple(face.vertices)) for face in mesh.polygons]

    digest = hashlib.blake2b(digest_size=16)
    digest.update(struct.pack("<QQQ", vertex_count, len(edges), len(faces)))
    for first, second in sorted(edges):
        digest.update(struct.pack("<QQ", first, second))
    for face in sorted(faces):
        digest.update(struct.pack("<Q", len(face)))
        for index in face:
            digest.update(struct.pack("<Q", index))
    return vertex_count, len(edges), len(faces), digest.digest()


def _deselect_bmesh(bm):
    for face in bm.faces:
        face.select_set(False)
    for edge in bm.edges:
        edge.select_set(False)
    for vert in bm.verts:
        vert.select_set(False)
    bm.select_history.clear()


def _selection_history(bm):
    history = []
    for element in bm.select_history:
        if not element.is_valid or not element.select:
            continue
        if isinstance(element, bmesh.types.BMVert):
            kind = 'VERT'
        elif isinstance(element, bmesh.types.BMEdge):
            kind = 'EDGE'
        elif isinstance(element, bmesh.types.BMFace):
            kind = 'FACE'
        else:
            continue
        history.append((kind, element.index))
    return tuple(history)


def _validate_object_reference(obj, mesh):
    try:
        return obj is not None and obj.type == 'MESH' and obj.data == mesh
    except (ReferenceError, RuntimeError):
        return False


def _validate_source(context):
    if not projection_state.has_source():
        return False, "No source is stored. Select source vertices and press Store Source."

    current_edit_ids = {id(obj) for obj in _edit_mesh_objects(context)}
    for entry in projection_state.source_entries:
        if not _validate_object_reference(entry.obj, entry.mesh):
            return False, "A stored source object is no longer available. Store the source again."
        if entry.obj.mode != 'EDIT':
            return False, "Return the stored source object(s) to Edit Mode before projecting."
        if id(entry.obj) not in current_edit_ids:
            return False, "All stored source objects must be in the current Edit Mode session."
        if _topology_signature(entry.obj) != entry.topology_signature:
            return False, "Source topology changed after it was stored. Store the source again."
        bm = _prepare_bmesh(entry.obj)
        if any(index < 0 or index >= len(bm.verts) for index in entry.vertex_indices):
            return False, "Stored source vertex indices are no longer valid. Store the source again."
        determinant = entry.obj.matrix_world.determinant()
        if not math.isfinite(determinant) or abs(determinant) <= _MATRIX_EPSILON:
            return False, f'Source object "{entry.obj.name}" has a singular world transform.'
    return True, ""


def _validate_target():
    target = projection_state.target
    if target is None:
        return False, "No target is stored. Select a target object or target faces and press Store Target."
    if not _validate_object_reference(target.obj, target.mesh):
        return False, "The stored target object is no longer available. Store the target again."
    if _topology_signature(target.obj) != target.topology_signature:
        return False, "Target topology changed after it was stored. Store the target again."
    if target.face_indices is not None:
        face_count = (
            len(_prepare_bmesh(target.obj).faces)
            if target.obj.mode == 'EDIT'
            else len(target.obj.data.polygons)
        )
        if any(index < 0 or index >= face_count for index in target.face_indices):
            return False, "Stored target face indices are no longer valid. Store the target again."
    return True, ""


def _base_mesh_polygon_soup(obj, face_indices=None):
    matrix = obj.matrix_world
    if obj.mode == 'EDIT':
        bm = _prepare_bmesh(obj)
        vertices = [matrix @ vert.co for vert in bm.verts]
        faces = bm.faces if face_indices is None else (bm.faces[index] for index in face_indices)
        polygons = [tuple(vert.index for vert in face.verts) for face in faces]
    else:
        mesh = obj.data
        vertices = [matrix @ vert.co for vert in mesh.vertices]
        faces = mesh.polygons if face_indices is None else (mesh.polygons[index] for index in face_indices)
        polygons = [tuple(face.vertices) for face in faces]
    return vertices, polygons


def _evaluated_polygon_soup(context, obj):
    depsgraph = context.evaluated_depsgraph_get()
    evaluated_obj = obj.evaluated_get(depsgraph)
    evaluated_mesh = evaluated_obj.to_mesh(
        preserve_all_data_layers=False,
        depsgraph=depsgraph,
    )
    if evaluated_mesh is None:
        raise ProjectionError("The stored target object has no evaluated mesh.")
    try:
        matrix = evaluated_obj.matrix_world
        vertices = [matrix @ vertex.co for vertex in evaluated_mesh.vertices]
        polygons = [tuple(polygon.vertices) for polygon in evaluated_mesh.polygons]
        return vertices, polygons
    finally:
        evaluated_obj.to_mesh_clear()


def _append_polygon_soup(vertices, polygons, new_vertices, new_polygons):
    offset = len(vertices)
    vertices.extend(new_vertices)
    polygons.extend(
        tuple(offset + index for index in polygon)
        for polygon in new_polygons
    )


def _object_is_visible(context, obj):
    if obj.hide_viewport or obj.hide_get():
        return False
    try:
        return obj.visible_get(
            view_layer=context.view_layer,
            viewport=context.space_data,
        )
    except TypeError:
        return obj.visible_get(view_layer=context.view_layer)


def _visible_target_polygon_soup(context):
    source_ids = {id(entry.obj) for entry in projection_state.source_entries}
    vertices = []
    polygons = []
    for obj in context.view_layer.objects:
        if obj.type != 'MESH' or id(obj) in source_ids or not _object_is_visible(context, obj):
            continue
        if obj.mode == 'EDIT':
            bm = _prepare_bmesh(obj)
            matrix = obj.matrix_world
            new_vertices = [matrix @ vert.co for vert in bm.verts]
            new_polygons = [
                tuple(vert.index for vert in face.verts)
                for face in bm.faces
                if face.is_valid and not face.hide and len(face.verts) >= 3
            ]
        else:
            new_vertices, new_polygons = _evaluated_polygon_soup(context, obj)
        _append_polygon_soup(vertices, polygons, new_vertices, new_polygons)
    if not polygons:
        raise ProjectionError("No visible mesh target is available for view projection.")
    return vertices, polygons


def _orthographic_view_direction(context):
    space = context.space_data
    region_3d = getattr(space, "region_3d", None) if space else None
    if region_3d is None:
        raise ProjectionError("View projection must be run from a 3D View.")
    if region_3d.is_perspective:
        raise ProjectionError("View projection requires an Orthographic view.")
    direction = -region_3d.view_matrix.inverted_safe().col[2].to_3d()
    if direction.length <= _MATRIX_EPSILON:
        raise ProjectionError("The Orthographic view has no valid projection direction.")
    return direction.normalized()


def _selected_target_faces(context):
    selections = []
    for obj in _edit_mesh_objects(context):
        bm = _prepare_bmesh(obj)
        face_indices = tuple(face.index for face in bm.faces if face.select and not face.hide)
        if face_indices:
            selections.append((obj, face_indices))
    return selections


def _selected_faces_polygon_soup(selections):
    vertices = []
    polygons = []
    for obj, face_indices in selections:
        bm = _prepare_bmesh(obj)
        used_indices = sorted({
            vert.index
            for index in face_indices
            for vert in bm.faces[index].verts
        })
        offset = len(vertices)
        remap = {index: offset + local_index for local_index, index in enumerate(used_indices)}
        vertices.extend(obj.matrix_world @ bm.verts[index].co for index in used_indices)
        polygons.extend(
            tuple(remap[vert.index] for vert in bm.faces[index].verts)
            for index in face_indices
        )
    return vertices, polygons


def _target_polygon_soup(context, target):
    if isinstance(target, list):
        return (*_selected_faces_polygon_soup(target), True)
    if target.face_indices is not None:
        # Explicit stored geometry means the exact edit-cage faces.
        return (*_base_mesh_polygon_soup(target.obj, target.face_indices), True)
    if target.obj.mode == 'EDIT':
        return (*_base_mesh_polygon_soup(target.obj), False)
    # A whole-object target includes its current modifier result and ignores visibility.
    return (*_evaluated_polygon_soup(context, target.obj), False)


def _source_world_points():
    records = []
    locations = []
    for entry in projection_state.source_entries:
        bm = _prepare_bmesh(entry.obj)
        matrix = entry.obj.matrix_world
        for index in entry.vertex_indices:
            records.append((entry, index))
            locations.append(matrix @ bm.verts[index].co)
    return records, locations


def _projection_direction(context, source_locations, components, target_is_explicit_faces, target_vertices):
    """Choose a shared direction from target geometry, never from the view."""
    if getattr(context.scene, "rcad_projection_direction", 'NORMAL') == 'HORIZONTAL':
        source_centroid = sum(source_locations, Vector((0.0, 0.0, 0.0))) / len(source_locations)
        target_centroid = sum(target_vertices, Vector((0.0, 0.0, 0.0))) / len(target_vertices)
        horizontal = target_centroid - source_centroid
        horizontal.z = 0.0
        if horizontal.length > _MATRIX_EPSILON:
            return horizontal.normalized()
        raise ProjectionError(
            "Horizontal projection needs a horizontal separation between source and target."
        )
    if getattr(context.scene, "rcad_projection_direction", 'NORMAL') == 'VERTICAL':
        return Vector((0.0, 0.0, 1.0))

    if target_is_explicit_faces:
        normal_sum = Vector((0.0, 0.0, 0.0))
        for component in components:
            normal_sum += component.reference_normal * component.normal_weight
        if normal_sum.length > _MATRIX_EPSILON:
            return normal_sum.normalized()

    centroid = Vector((0.0, 0.0, 0.0))
    for location in source_locations:
        centroid += location
    centroid /= len(source_locations)

    closest = None
    for component_index, component in enumerate(components):
        location, normal, _face_index, distance = component.tree.find_nearest(centroid)
        if location is None or distance is None or not math.isfinite(distance):
            continue
        candidate = (distance, component_index, location, normal)
        if closest is None or candidate[:2] < closest[:2]:
            closest = candidate

    if closest is not None:
        _distance, component_index, location, normal = closest
        if normal is not None and normal.length > _MATRIX_EPSILON:
            return normal.normalized()
        toward_surface = location - centroid
        if toward_surface.length > _MATRIX_EPSILON:
            return toward_surface.normalized()
        return components[component_index].reference_normal

    raise ProjectionError("The target does not provide a valid projection direction.")


def _restore_source_selection(context):
    edit_objects = _edit_mesh_objects(context)
    edit_object_ids = {id(obj) for obj in edit_objects}
    if any(id(entry.obj) not in edit_object_ids for entry in projection_state.source_entries):
        raise ProjectionError("All stored source objects must be in the current Edit Mode session.")

    for obj in edit_objects:
        _deselect_bmesh(_prepare_bmesh(obj))

    for entry in projection_state.source_entries:
        bm = _prepare_bmesh(entry.obj)
        for index in entry.vertex_indices:
            bm.verts[index].select_set(True)
        for index in entry.edge_indices:
            if 0 <= index < len(bm.edges):
                bm.edges[index].select_set(True)
        for index in entry.face_indices:
            if 0 <= index < len(bm.faces):
                bm.faces[index].select_set(True)

        bm.select_history.clear()
        for kind, index in entry.selection_history:
            sequence = {
                'VERT': bm.verts,
                'EDGE': bm.edges,
                'FACE': bm.faces,
            }.get(kind)
            if sequence is not None and 0 <= index < len(sequence):
                element = sequence[index]
                if element.select:
                    bm.select_history.add(element)

    active = projection_state.source_active_object
    if active is not None and id(active) in edit_object_ids:
        context.view_layer.objects.active = active

    for obj in edit_objects:
        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)


def _return_to_source_edit_mode(context):
    """After storing any target, make the source ready for Project."""
    source_objects = [entry.obj for entry in projection_state.source_entries]
    if not source_objects or any(obj.name not in context.view_layer.objects for obj in source_objects):
        return False
    try:
        current_edit_ids = {id(obj) for obj in _edit_mesh_objects(context)}
        if context.mode == 'EDIT_MESH' and all(id(obj) in current_edit_ids for obj in source_objects):
            active = projection_state.source_active_object
            if active in source_objects:
                context.view_layer.objects.active = active
            return True
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for obj in context.selected_objects:
            obj.select_set(False)
        for obj in source_objects:
            obj.select_set(True)
        active = projection_state.source_active_object
        if active not in source_objects:
            active = source_objects[0]
        context.view_layer.objects.active = active
        bpy.ops.object.mode_set(mode='EDIT')
        return all(obj.mode == 'EDIT' for obj in source_objects)
    except (RuntimeError, ReferenceError):
        return False


class RCAD_OT_StoreProjectionSource(bpy.types.Operator):
    bl_idname = "mesh.rcad_store_projection_source"
    bl_label = "Store Source"
    bl_description = "Store the selected edit-mode vertices as the projection source"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and bool(_edit_mesh_objects(context))

    def execute(self, context):
        entries = []
        edit_objects = _edit_mesh_objects(context)
        for obj in edit_objects:
            bm = _prepare_bmesh(obj)
            vertex_indices = tuple(vertex.index for vertex in bm.verts if vertex.select)
            if not vertex_indices:
                continue
            entries.append(SourceEntry(
                obj=obj,
                mesh=obj.data,
                vertex_indices=vertex_indices,
                edge_indices=tuple(edge.index for edge in bm.edges if edge.select),
                face_indices=tuple(face.index for face in bm.faces if face.select),
                selection_history=_selection_history(bm),
                topology_signature=_topology_signature(obj),
            ))

        if not entries:
            self.report({'WARNING'}, "Select at least one source vertex in Edit Mode.")
            return {'CANCELLED'}

        active = context.view_layer.objects.active
        if active not in [entry.obj for entry in entries]:
            active = entries[0].obj
        projection_state.store_source(entries, active_object=active)
        projection_state.clear_target()

        for obj in edit_objects:
            _deselect_bmesh(_prepare_bmesh(obj))
            bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

        self.report(
            {'INFO'},
            f"Stored {projection_state.vertex_count} source vertices. Select target faces, then press Project.",
        )
        return {'FINISHED'}


class RCAD_OT_StoreProjectionTarget(bpy.types.Operator):
    bl_idname = "mesh.rcad_store_projection_target"
    bl_label = "Store Target"
    bl_description = "Store selected Edit Mode faces, or one selected Object Mode mesh, as the target"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return projection_state.has_source() and context.mode in {'EDIT_MESH', 'OBJECT'}

    def execute(self, context):
        if not projection_state.has_source():
            self.report({'WARNING'}, "Store source vertices first.")
            return {'CANCELLED'}

        if context.mode == 'EDIT_MESH':
            selections = []
            for obj in _edit_mesh_objects(context):
                bm = _prepare_bmesh(obj)
                indices = tuple(face.index for face in bm.faces if face.select and not face.hide)
                if indices:
                    selections.append((obj, indices))
            if not selections:
                self.report({'WARNING'}, "Select one or more target faces in Edit Mode.")
                return {'CANCELLED'}
            if len(selections) != 1:
                self.report({'WARNING'}, "Target faces must belong to exactly one mesh object.")
                return {'CANCELLED'}

            obj, face_indices = selections[0]
            projection_state.store_target(TargetEntry(
                obj=obj,
                mesh=obj.data,
                face_indices=face_indices,
                topology_signature=_topology_signature(obj),
            ))
            for edit_obj in _edit_mesh_objects(context):
                _deselect_bmesh(_prepare_bmesh(edit_obj))
                bmesh.update_edit_mesh(edit_obj.data, loop_triangles=False, destructive=False)
            returned = _return_to_source_edit_mode(context)
            suffix = " Source is ready in Edit Mode." if returned else " Return the source to Edit Mode, then Project."
            self.report(
                {'INFO'},
                f'Stored {len(face_indices)} target faces from "{obj.name}".{suffix}',
            )
            return {'FINISHED'}

        selected_objects = list(context.selected_objects)
        if len(selected_objects) != 1 or selected_objects[0].type != 'MESH':
            self.report({'WARNING'}, "Select exactly one mesh object as the target.")
            return {'CANCELLED'}
        obj = selected_objects[0]
        source_objects = [entry.obj for entry in projection_state.source_entries]
        if obj in source_objects:
            self.report(
                {'WARNING'},
                "A complete target object cannot also be a source object; select its target faces in Edit Mode instead.",
            )
            return {'CANCELLED'}

        projection_state.store_target(TargetEntry(
            obj=obj,
            mesh=obj.data,
            face_indices=None,
            topology_signature=_topology_signature(obj),
        ))
        returned = _return_to_source_edit_mode(context)
        suffix = " Returned to the source in Edit Mode." if returned else " Return the source to Edit Mode, then Project."
        self.report({'INFO'}, f'Stored target object "{obj.name}".{suffix}')
        return {'FINISHED'}


class RCAD_OT_ProjectStoredGeometry(bpy.types.Operator):
    bl_idname = "mesh.rcad_project_stored_geometry"
    bl_label = "Project"
    bl_description = "Project stored source vertices onto the currently selected target faces"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (
            context.mode == 'EDIT_MESH'
            and projection_state.has_source()
        )

    def execute(self, context):
        valid, message = _validate_source(context)
        if not valid:
            self.report({'WARNING'}, message)
            return {'CANCELLED'}
        target = _selected_target_faces(context)
        view_projection = not target and not projection_state.has_target()
        if not target and not view_projection:
            valid, message = _validate_target()
            if not valid:
                self.report({'WARNING'}, message)
                return {'CANCELLED'}
            target = projection_state.target

        try:
            if view_projection:
                target_vertices, target_polygons = _visible_target_polygon_soup(context)
                target_is_explicit_faces = False
            else:
                target_vertices, target_polygons, target_is_explicit_faces = _target_polygon_soup(
                    context,
                    target,
                )
            components = build_surface_components(target_vertices, target_polygons)
            records, source_locations = _source_world_points()
            direction = (
                _orthographic_view_direction(context)
                if view_projection
                else _projection_direction(
                    context,
                    source_locations,
                    components,
                    target_is_explicit_faces,
                    target_vertices,
                )
            )
            result = project_points_coherently(source_locations, components, direction)

            inverse_matrices = {
                id(entry): entry.obj.matrix_world.inverted()
                for entry in projection_state.source_entries
            }
            bmeshes = {
                id(entry): _prepare_bmesh(entry.obj)
                for entry in projection_state.source_entries
            }
            originals = [
                (entry, vertex_index, bmeshes[id(entry)].verts[vertex_index].co.copy())
                for entry, vertex_index in records
            ]
            try:
                moved = 0
                for (entry, vertex_index), old_location, new_location in zip(
                    records,
                    source_locations,
                    result.locations,
                ):
                    if (new_location - old_location).length_squared > _MOVE_EPSILON_SQUARED:
                        moved += 1
                    bmeshes[id(entry)].verts[vertex_index].co = (
                        inverse_matrices[id(entry)] @ new_location
                    )

                for entry in projection_state.source_entries:
                    bmesh.update_edit_mesh(
                        entry.obj.data,
                        loop_triangles=False,
                        destructive=False,
                    )
                _restore_source_selection(context)
            except Exception:
                # Projection is atomic even if Blender rejects an update late.
                for entry, vertex_index, coordinate in originals:
                    try:
                        bmeshes[id(entry)].verts[vertex_index].co = coordinate
                    except (ReferenceError, RuntimeError, IndexError):
                        pass
                for entry in projection_state.source_entries:
                    try:
                        bmesh.update_edit_mesh(
                            entry.obj.data,
                            loop_triangles=False,
                            destructive=False,
                        )
                    except (ReferenceError, RuntimeError):
                        pass
                raise
        except ProjectionError as exc:
            self.report({'WARNING'}, str(exc))
            return {'CANCELLED'}
        except (ReferenceError, RuntimeError, ValueError) as exc:
            self.report({'ERROR'}, f"Projection failed without changing the source: {exc}")
            return {'CANCELLED'}

        component_note = ""
        if len(components) > 1 and result.component_index >= 0:
            component_note = f" using target surface {result.component_index + 1} of {len(components)}"
        elif len(components) > 1 and result.component_index == -1:
            component_note = " across disconnected target surfaces"
        if moved:
            self.report(
                {'INFO'},
                f"Projected {len(records)} vertices{component_note}; {moved} moved.",
            )
        else:
            self.report({'INFO'}, f"All {len(records)} source vertices were already on the target.")
        return {'FINISHED'}


class RCAD_OT_ClearProjectionState(bpy.types.Operator):
    bl_idname = "mesh.rcad_clear_projection_state"
    bl_label = "Clear"
    bl_description = "Forget the stored projection source and target"
    bl_options = {'REGISTER'}

    def execute(self, _context):
        projection_state.clear()
        self.report({'INFO'}, "Cleared stored projection source and target.")
        return {'FINISHED'}


classes = (
    RCAD_OT_StoreProjectionSource,
    RCAD_OT_StoreProjectionTarget,
    RCAD_OT_ProjectStoredGeometry,
    RCAD_OT_ClearProjectionState,
)
