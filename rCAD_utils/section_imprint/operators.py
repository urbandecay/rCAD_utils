"""Edit-mode operators for generating independent section imprints."""

import bmesh
import bpy
from bpy.props import BoolProperty, FloatProperty
from mathutils import Vector

from .geometry import (
    DEFAULT_TOLERANCE,
    add_section_face,
    bisect_section_faces,
    copy_face_group,
    dominant_material_index,
    face_groups,
    selected_faces,
    tag_face_groups,
)
from .storage import section_plane_storage


_EPSILON = 1.0e-10
_RESULT_COLLECTION_NAME = "rCAD Section Imprints"
_RESULT_TAG = "rcad_section_imprint"


def _active_edit_mesh(context):
    obj = getattr(context, "edit_object", None)
    if obj is None:
        obj = getattr(context, "active_object", None)
    if obj is None or obj.type != 'MESH' or obj.mode != 'EDIT':
        return None
    return obj


def _average(points):
    if not points:
        return Vector()
    return sum((Vector(point) for point in points), Vector()) / len(points)


def _world_normal(obj, local_normal):
    normal = obj.matrix_world.to_3x3().inverted_safe().transposed() @ local_normal
    if normal.length <= _EPSILON:
        return Vector()
    return normal.normalized()


def _first_plane_normal(points, tolerance):
    """Find a non-degenerate normal from a point selection."""

    if len(points) < 3:
        return Vector()
    first = points[0]
    for second_index in range(1, len(points) - 1):
        edge_a = points[second_index] - first
        if edge_a.length <= tolerance:
            continue
        for third in points[second_index + 1:]:
            edge_b = third - first
            normal = edge_a.cross(edge_b)
            if normal.length > tolerance * tolerance:
                return normal.normalized()
    return Vector()


def _plane_basis(points, normal, tolerance):
    """Build a deterministic right-handed basis in the stored plane."""

    for first_index, first in enumerate(points[:-1]):
        for second in points[first_index + 1:]:
            candidate = second - first
            candidate -= normal * candidate.dot(normal)
            if candidate.length <= tolerance:
                continue
            axis_u = candidate.normalized()
            axis_v = normal.cross(axis_u)
            if axis_v.length > tolerance:
                return axis_u, axis_v.normalized()

    reference = Vector((1.0, 0.0, 0.0))
    if abs(reference.dot(normal)) > 0.9:
        reference = Vector((0.0, 1.0, 0.0))
    axis_u = reference - normal * reference.dot(normal)
    if axis_u.length <= tolerance:
        reference = Vector((0.0, 0.0, 1.0))
        axis_u = reference - normal * reference.dot(normal)
    if axis_u.length <= tolerance:
        return Vector(), Vector()
    axis_u.normalize()
    axis_v = normal.cross(axis_u)
    if axis_v.length <= tolerance:
        return Vector(), Vector()
    return axis_u, axis_v.normalized()


def _plane_from_points(obj, local_points, local_face_normal, tolerance):
    """Build a world-space section plane from one object's local geometry."""

    if len(local_points) < 3:
        return None

    world_points = [obj.matrix_world @ point for point in local_points]
    origin = _average(world_points)
    if local_face_normal is not None and local_face_normal.length > _EPSILON:
        normal = _world_normal(obj, local_face_normal.normalized())
    else:
        normal = _first_plane_normal(world_points, tolerance)
    if normal.length <= _EPSILON:
        return None

    max_distance = max(
        abs((point - origin).dot(normal))
        for point in world_points
    )
    if max_distance > tolerance:
        return None

    axis_u, axis_v = _plane_basis(world_points, normal, tolerance)
    if axis_u.length <= _EPSILON or axis_v.length <= _EPSILON:
        return None
    return {
        "origin": origin,
        "normal": normal,
        "axis_u": axis_u,
        "axis_v": axis_v,
        "source_object_name": obj.name,
    }


def _selected_plane_points(bm):
    """Use the active selected face, or selected vertices, as the plane."""

    selected = [
        face
        for face in bm.faces
        if face.is_valid and not face.hide and face.select and len(face.verts) >= 3
    ]
    if selected:
        active = bm.select_history.active
        if not isinstance(active, bmesh.types.BMFace) or active not in selected:
            active = selected[0]
        return [vert.co.copy() for vert in active.verts], active.normal.copy()

    return [
        vert.co.copy()
        for vert in bm.verts
        if vert.is_valid and not vert.hide and vert.select
    ], None


def _planar_object_plane(obj, bm, tolerance):
    """Return a plane when an Edit Mode object is entirely planar."""

    visible_faces = [
        face
        for face in bm.faces
        if face.is_valid and not face.hide and len(face.verts) >= 3
    ]
    if not visible_faces:
        return None

    # A normal one-face cutter does not need to be selected in Face Select
    # mode.  Selecting the plane object is enough.
    if len(visible_faces) == 1:
        face = visible_faces[0]
        return _plane_from_points(
            obj,
            [vert.co.copy() for vert in face.verts],
            face.normal.copy(),
            tolerance,
        )

    points = []
    seen = set()
    for face in visible_faces:
        for vert in face.verts:
            if vert in seen:
                continue
            seen.add(vert)
            points.append(vert.co.copy())
    return _plane_from_points(obj, points, None, tolerance)


def _selected_geometry_plane(obj, bm, tolerance):
    """Return a plane from selected planar geometry in an Edit Mode object."""

    selected_faces = [
        face
        for face in bm.faces
        if face.is_valid and not face.hide and face.select
        and len(face.verts) >= 3
    ]
    if selected_faces:
        points = []
        seen = set()
        for face in selected_faces:
            for vert in face.verts:
                if vert in seen:
                    continue
                seen.add(vert)
                points.append(vert.co.copy())
        plane = _plane_from_points(obj, points, None, tolerance)
        if plane is not None:
            return plane

    selected_vertices = [
        vert.co.copy()
        for vert in bm.verts
        if vert.is_valid and not vert.hide and vert.select
    ]
    if len(selected_vertices) >= 3:
        return _plane_from_points(obj, selected_vertices, None, tolerance)
    return None


def _edit_mesh_objects(context):
    """Return all unique mesh objects in the current Edit Mode session."""

    objects = []
    seen = set()
    # ``objects_in_mode_unique_data`` can contain only one object when several
    # Edit Mode objects share a mesh datablock.  The section operation needs
    # every object so the cutter does not hide the actual targets.
    for attribute in ("objects_in_mode", "objects_in_mode_unique_data"):
        values = getattr(context, attribute, None)
        if values is None:
            continue
        for obj in values:
            if (
                obj.type == 'MESH'
                and obj.mode == 'EDIT'
                and obj not in seen
            ):
                objects.append(obj)
                seen.add(obj)

    # Keep the fallback explicit for Blender contexts that do not expose
    # either objects-in-mode collection from a sidebar operator.
    for obj in getattr(context, "selected_objects", ()):
        if obj.type == 'MESH' and obj.mode == 'EDIT' and obj not in seen:
            objects.append(obj)
            seen.add(obj)
    for obj in context.view_layer.objects:
        if obj.type == 'MESH' and obj.mode == 'EDIT' and obj not in seen:
            objects.append(obj)
            seen.add(obj)

    if not objects:
        active = _active_edit_mesh(context)
        if active is not None:
            objects = [active]
    return objects


def _selected_plane_object(context, tolerance):
    """Find the dedicated planar cutter in the current Edit Mode session."""

    selected = []
    for obj in _edit_mesh_objects(context):
        bm = bmesh.from_edit_mesh(obj.data)
        bm.normal_update()
        bm.faces.index_update()
        faces = [f for f in bm.faces if not f.hide and
                 (f.select or all(v.select for v in f.verts))]
        if not faces:
            continue
        verts = {v for f in faces for v in f.verts}
        plane = _plane_from_points(obj, [v.co.copy() for v in verts], None, tolerance)
        if plane is None:
            # Multi-object Edit Mode commonly enters with every target face
            # selected. A nonplanar target is simply not a cutter candidate;
            # keep looking for the selected planar cutter object.
            continue
        plane["cutter_face_indices"] = {f.index for f in faces}
        plane["embedded_cutter"] = len(faces) < len(bm.faces)
        selected.append((obj, plane))
    if len(selected) == 1:
        return selected[0]
    if len(selected) > 1:
        raise ValueError("Select one planar cutter; targets are found automatically.")

    candidates = []
    for obj in _edit_mesh_objects(context):
        bm = bmesh.from_edit_mesh(obj.data)
        plane = _planar_object_plane(obj, bm, tolerance)
        if plane is None:
            plane = _selected_geometry_plane(obj, bm, tolerance)
        if plane is not None:
            candidates.append((obj, plane))

    # Targets must be in Edit Mode, but a separately selected cutter may still
    # be in Object Mode.  Accept that cutter as part of the same one-button
    # Edit Mode workflow.
    known_objects = {obj for obj, _plane in candidates}
    for obj in getattr(context, "selected_objects", ()):
        if obj in known_objects or obj.type != 'MESH' or obj.mode == 'EDIT':
            continue
        bm = bmesh.new()
        try:
            bm.from_mesh(obj.data)
            plane = _planar_object_plane(obj, bm, tolerance)
        finally:
            bm.free()
        if plane is not None:
            candidates.append((obj, plane))

    if not candidates:
        return None, None

    active = getattr(context, "active_object", None)
    active_candidate = next(
        ((obj, plane) for obj, plane in candidates if obj == active),
        None,
    )
    if active_candidate is not None:
        return active_candidate
    if len(candidates) == 1:
        return candidates[0]
    raise ValueError(
        "More than one planar mesh is in Edit Mode; make the cutter active."
    )


def _plane_intersects_bounds(obj, plane, tolerance):
    """Reject targets whose world-space bounds are entirely off the plane."""

    try:
        distances = [
            ((obj.matrix_world @ Vector(corner)) - plane["origin"]).dot(
                plane["normal"]
            )
            for corner in obj.bound_box
        ]
    except (AttributeError, TypeError, ValueError):
        return True
    if not distances:
        return True
    return min(distances) <= tolerance and max(distances) >= -tolerance


def _ensure_result_collection(context, source_object):
    collection = bpy.data.collections.get(_RESULT_COLLECTION_NAME)
    if collection is None:
        collection = bpy.data.collections.new(_RESULT_COLLECTION_NAME)

    parent = next(iter(getattr(source_object, "users_collection", ())), None)
    if parent is None:
        parent = context.scene.collection
    if collection.name not in {child.name for child in parent.children}:
        try:
            parent.children.link(collection)
        except RuntimeError:
            pass
    return collection


def _unique_object_name(base_name):
    name = base_name
    index = 1
    while bpy.data.objects.get(name) is not None:
        name = f"{base_name}.{index:03d}"
        index += 1
    return name


def _material_key(material):
    if material is None:
        return None
    try:
        return material.as_pointer()
    except AttributeError:
        return id(material)


def _output_material_index(output_mesh, source_object, source_index, cache):
    try:
        material = source_object.data.materials[int(source_index)]
    except (IndexError, TypeError, ValueError):
        material = None
    if material is None:
        return 0

    key = _material_key(material)
    if key not in cache:
        cache[key] = len(output_mesh.materials)
        output_mesh.materials.append(material)
    return cache[key]


def _append_output_to_edit_object(output_mesh, destination):
    """Append world-space section faces to the active Edit Mode mesh."""

    if destination.mode != 'EDIT':
        raise RuntimeError("The section destination must remain in Edit Mode.")

    edit_bm = bmesh.from_edit_mesh(destination.data)
    inverse = destination.matrix_world.inverted_safe()
    material_indices = {}
    for index, material in enumerate(output_mesh.materials):
        if material is None:
            material_indices[index] = 0
            continue
        destination_index = next(
            (
                slot
                for slot, existing in enumerate(destination.data.materials)
                if existing == material
            ),
            None,
        )
        if destination_index is None:
            destination.data.materials.append(material)
            destination_index = len(destination.data.materials) - 1
        material_indices[index] = destination_index

    added = 0
    for polygon in output_mesh.polygons:
        vertices = [
            edit_bm.verts.new(tuple(inverse @ output_mesh.vertices[index].co))
            for index in polygon.vertices
        ]
        if len(vertices) < 3:
            continue
        try:
            face = edit_bm.faces.new(vertices)
        except ValueError:
            # A repeated section request may meet an existing face. Keep the
            # existing mesh intact and continue with the other section faces.
            continue
        face.material_index = material_indices.get(polygon.material_index, 0)
        face.select_set(True)
        added += 1

    edit_bm.normal_update()
    bmesh.update_edit_mesh(destination.data, loop_triangles=True, destructive=True)
    return added


def _target_objects(context, cutter_object=None):
    """Return all mesh targets except the dedicated cutter object."""

    objects = _edit_mesh_objects(context)
    if cutter_object is not None:
        return [obj for obj in objects if obj != cutter_object]

    # Keep the stored-plane workflow usable for existing files and scripts.
    plane_name = section_plane_storage.source_object_name
    if plane_name:
        filtered = [obj for obj in objects if obj.name != plane_name]
        if filtered:
            return filtered
    return objects


def _section_plane_from_selection(obj, bm, tolerance):
    local_points, local_face_normal = _selected_plane_points(bm)
    if len(local_points) < 3:
        raise ValueError(
            "Select a planar face or at least three vertices for the section plane."
        )

    plane = _plane_from_points(
        obj,
        local_points,
        local_face_normal,
        tolerance,
    )
    if plane is None:
        raise ValueError(
            "The selected plane is not planar or does not have a usable normal."
        )
    return (
        plane["origin"],
        plane["normal"],
        plane["axis_u"],
        plane["axis_v"],
    )


class MESH_OT_rcad_store_section_plane(bpy.types.Operator):
    """Store a selected planar face or vertex selection as a section plane."""

    bl_idname = "mesh.rcad_store_section_plane"
    bl_label = "Store Section Plane"
    bl_description = "Store the active selected face or vertex plane"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return _active_edit_mesh(context) is not None

    def execute(self, context):
        obj = _active_edit_mesh(context)
        tolerance = max(
            float(getattr(context.scene, "rcad_section_tolerance", DEFAULT_TOLERANCE)),
            1.0e-8,
        )
        bm = bmesh.from_edit_mesh(obj.data)
        try:
            origin, normal, axis_u, axis_v = _section_plane_from_selection(
                obj,
                bm,
                tolerance,
            )
        except ValueError as exc:
            self.report({'WARNING'}, str(exc))
            section_plane_storage.clear()
            return {'CANCELLED'}

        section_plane_storage.store(
            origin,
            normal,
            axis_u,
            axis_v,
            source_object_name=obj.name,
        )
        self.report({'INFO'}, "Section plane stored.")
        return {'FINISHED'}


class MESH_OT_rcad_clear_section_plane(bpy.types.Operator):
    """Clear the stored section plane."""

    bl_idname = "mesh.rcad_clear_section_plane"
    bl_label = "Clear Plane"
    bl_description = "Clear the stored section plane"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        section_plane_storage.clear()
        self.report({'INFO'}, "Stored section plane cleared.")
        return {'FINISHED'}


class MESH_OT_rcad_section_imprint(bpy.types.Operator):
    """Create independent material-aware faces where meshes meet a plane."""

    bl_idname = "mesh.rcad_section_imprint"
    bl_label = "Create Section Imprint"
    bl_description = (
        "Create separate section faces from the active planar cutter without "
        "changing the source meshes or using a Boolean"
    )
    bl_options = {'REGISTER', 'UNDO'}

    tolerance: FloatProperty(
        name="Tolerance",
        description="World-space tolerance used by the section operation",
        default=DEFAULT_TOLERANCE,
        min=1.0e-8,
        max=1.0,
    )
    keep_cutter: BoolProperty(
        name="Keep Cutter",
        description="Keep the selected plane object after creating the section",
        default=False,
    )

    @classmethod
    def poll(cls, context):
        return _active_edit_mesh(context) is not None

    def execute(self, context):
        tolerance = max(float(self.tolerance), 1.0e-8)
        try:
            cutter_object, plane = _selected_plane_object(context, tolerance)
        except ValueError as exc:
            self.report({'WARNING'}, str(exc))
            return {'CANCELLED'}
        if plane is None and section_plane_storage.has_plane():
            plane = section_plane_storage.snapshot()
            stored_source = bpy.data.objects.get(
                plane.get("source_object_name", "")
            )
            if (
                stored_source is not None
                and stored_source in _edit_mesh_objects(context)
                and len(stored_source.data.polygons) == 1
            ):
                cutter_object = stored_source
        if plane is None:
            self.report(
                {'WARNING'},
                "Select one separate planar mesh as the section cutter.",
            )
            return {'CANCELLED'}

        embedded_cutter = bool(plane.get("embedded_cutter"))
        destination = _active_edit_mesh(context)
        if destination is not None and destination == cutter_object and not embedded_cutter:
            destination = next(
                (
                    obj for obj in _edit_mesh_objects(context)
                    if obj != cutter_object
                ),
                None,
            )
        if destination is None:
            self.report(
                {'WARNING'},
                "The active Edit Mode object is the section cutter; enter Edit Mode on the model.",
            )
            return {'CANCELLED'}

        targets = [destination]
        if not _plane_intersects_bounds(destination, plane, tolerance):
            self.report(
                {'WARNING'},
                "The section plane does not intersect the active object.",
            )
            return {'CANCELLED'}

        output_mesh = bpy.data.meshes.new("RCAD_Section_Imprint_Mesh")
        output_bm = bmesh.new()
        material_cache = {}
        section_count = 0
        group_count = 0
        failed_count = 0
        no_section_count = 0

        try:
            for obj in targets:
                if obj.mode == 'EDIT':
                    analysis_bm = bmesh.from_edit_mesh(obj.data).copy()
                else:
                    analysis_bm = bmesh.new()
                    analysis_bm.from_mesh(obj.data)
                try:
                    if obj == cutter_object and embedded_cutter:
                        analysis_bm.faces.index_update()
                        cutter_faces = [f for f in analysis_bm.faces
                                        if f.index in plane["cutter_face_indices"]]
                        bmesh.ops.delete(analysis_bm, geom=cutter_faces, context='FACES_ONLY')
                    candidates = selected_faces(
                        analysis_bm,
                        selection_only=False,
                    )
                    if not candidates:
                        failed_count += 1
                        continue

                    groups = face_groups(analysis_bm, candidates)
                    if not groups:
                        failed_count += 1
                        continue
                    _group_layer, groups = tag_face_groups(analysis_bm, groups)

                    for group_index, group in enumerate(groups):
                        work_bm = None
                        try:
                            work_bm = copy_face_group(
                                analysis_bm,
                                group_index,
                            )
                            section_faces = bisect_section_faces(
                                work_bm,
                                obj.matrix_world,
                                plane["origin"],
                                plane["normal"],
                                tolerance=tolerance,
                                fallback_material=dominant_material_index(group),
                            )
                            if not section_faces:
                                no_section_count += 1
                                group_count += 1
                                continue

                            vertex_cache = {}
                            for section_face in section_faces:
                                output_material = _output_material_index(
                                    output_mesh,
                                    obj,
                                    section_face.material_index,
                                    material_cache,
                                )
                                if add_section_face(
                                    output_bm,
                                    section_face,
                                    plane["normal"],
                                    output_material,
                                    group_count,
                                    vertex_cache,
                                    tolerance=tolerance,
                                ):
                                    section_count += 1
                            group_count += 1
                        except Exception as exc:
                            failed_count += 1
                            self.report(
                                {'WARNING'},
                                f"{obj.name}: section group failed ({exc}).",
                            )
                        finally:
                            if work_bm is not None:
                                work_bm.free()
                finally:
                    analysis_bm.free()

            if not section_count:
                bpy.data.meshes.remove(output_mesh)
                output_mesh = None
                self.report(
                    {'WARNING'},
                    "The plane did not produce any closed section faces.",
                )
                return {'CANCELLED'}

            output_bm.normal_update()
            output_bm.to_mesh(output_mesh)
        except Exception as exc:
            if output_mesh is not None:
                bpy.data.meshes.remove(output_mesh)
                output_mesh = None
            self.report({'ERROR'}, f"Section imprint failed: {exc}")
            return {'CANCELLED'}
        finally:
            output_bm.free()

        try:
            added_count = _append_output_to_edit_object(output_mesh, destination)
        finally:
            if output_mesh is not None:
                bpy.data.meshes.remove(output_mesh)
                output_mesh = None
        if not added_count:
            self.report(
                {'WARNING'},
                "The section was already present or could not be added to the active object.",
            )
            return {'CANCELLED'}

        if not self.keep_cutter and cutter_object is not None:
            if embedded_cutter and cutter_object == destination:
                edit_bm = bmesh.from_edit_mesh(destination.data)
                edit_bm.faces.index_update()
                cutter_faces = [
                    face
                    for face in edit_bm.faces
                    if face.index in plane.get("cutter_face_indices", set())
                ]
                if cutter_faces:
                    cutter_edges = {
                        edge
                        for face in cutter_faces
                        for edge in face.edges
                    }
                    cutter_verts = {
                        vert
                        for face in cutter_faces
                        for vert in face.verts
                    }
                    standalone = all(
                        all(linked_face in cutter_faces
                            for linked_face in vert.link_faces)
                        and all(linked_edge in cutter_edges
                                for linked_edge in vert.link_edges)
                        for vert in cutter_verts
                    )
                    if standalone:
                        # Remove the construction plane completely. Deleting
                        # only its face leaves the hollow outline visible.
                        bmesh.ops.delete(
                            edit_bm,
                            geom=list(cutter_verts),
                            context='VERTS',
                        )
                    else:
                        # If the selected face shares model topology, hide
                        # only its unshared construction edges and vertices.
                        for face in cutter_faces:
                            face.hide_set(True)
                        for edge in cutter_edges:
                            if all(linked_face in cutter_faces
                                   for linked_face in edge.link_faces):
                                edge.hide_set(True)
                        for vert in cutter_verts:
                            if (
                                all(linked_face in cutter_faces
                                    for linked_face in vert.link_faces)
                                and all(linked_edge in cutter_edges
                                         for linked_edge in vert.link_edges)
                            ):
                                vert.hide_set(True)
                    edit_bm.normal_update()
                    bmesh.update_edit_mesh(
                        destination.data,
                        loop_triangles=True,
                        destructive=True,
                    )
            elif cutter_object != destination:
                # Keep the cutter available for a later operation without
                # leaving it visible in the viewport.
                cutter_object.hide_set(True)

        section_plane_storage.clear()

        message = (
            f"Created {added_count} section face(s) in {destination.name}."
        )
        if no_section_count or failed_count:
            message += (
                f" Skipped {no_section_count} non-intersecting part(s) and "
                f"{failed_count} failure(s)."
            )
        self.report({'INFO'}, message)
        return {'FINISHED'}


class MESH_OT_rcad_clear_section_imprints(bpy.types.Operator):
    """Remove only section objects generated by this tool."""

    bl_idname = "mesh.rcad_clear_section_imprints"
    bl_label = "Clear Results"
    bl_description = "Remove generated rCAD section imprint objects"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        results = [
            obj
            for obj in bpy.data.objects
            if obj.get(_RESULT_TAG, False)
        ]
        removed = 0
        for obj in results:
            mesh = obj.data if obj.type == 'MESH' else None
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
            removed += 1
        self.report({'INFO'}, f"Removed {removed} section imprint object(s).")
        return {'FINISHED'}


classes = (
    MESH_OT_rcad_store_section_plane,
    MESH_OT_rcad_clear_section_plane,
    MESH_OT_rcad_section_imprint,
    MESH_OT_rcad_clear_section_imprints,
)


def register():
    bpy.types.Scene.rcad_section_tolerance = FloatProperty(
        name="Section tolerance",
        description="World-space tolerance used by the section operation",
        default=DEFAULT_TOLERANCE,
        min=1.0e-8,
        max=1.0,
    )
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    section_plane_storage.clear()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    if hasattr(bpy.types.Scene, "rcad_section_tolerance"):
        del bpy.types.Scene.rcad_section_tolerance
