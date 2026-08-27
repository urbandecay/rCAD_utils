import bpy
import bmesh

from .extrude import (
    cap_buf,
    check_lukap,
    get_active_element_and_its_indices,
)
from .eap_adapter import extrude_faces_to_cutter
from .cool_bool import subtract_selected_islands


_last_keep_profile = False


def _get_keep_profile(operator):
    return _last_keep_profile


def _set_keep_profile(operator, value):
    global _last_keep_profile
    _last_keep_profile = bool(value)


def _active_mesh_in_edit_mode(context):
    object_active = context.active_object
    return (
        object_active is not None
        and object_active.type == 'MESH'
        and context.mode == 'EDIT_MESH'
    )


class OT_CarveAlongPath_Store_Path(bpy.types.Operator):
    bl_label = "Store"
    bl_idname = "mesh.cap_store_path"
    bl_options = {'REGISTER'}

    def execute(self, context):
        if not _active_mesh_in_edit_mode(context):
            self.report({'WARNING'}, "Select a mesh and enter Edit Mode before storing a path.")
            return {'CANCELLED'}

        object_active = context.active_object
        bm = bmesh.from_edit_mesh(object_active.data)
        check_lukap(bm)

        selected_faces = [face for face in bm.faces if face.select]
        selected_edges = [edge for edge in bm.edges if edge.select]
        face_select_mode = context.tool_settings.mesh_select_mode[2]
        stored_edges = []
        start_vertex = None

        if selected_faces and face_select_mode:
            path_face = selected_faces[0]
            start_vertex = path_face.verts[0].index
            stored_edges = [[vertex.index for vertex in edge.verts] for edge in path_face.edges]
        elif selected_edges:
            active_index, active_kind = get_active_element_and_its_indices(bm)
            if active_kind == 'E' and 0 <= active_index < len(bm.edges):
                active_edge = bm.edges[active_index]
                if active_edge.select:
                    start_vertex = active_edge.verts[0].index
            elif active_kind == 'V' and 0 <= active_index < len(bm.verts):
                active_vertex = bm.verts[active_index]
                if active_vertex.select:
                    start_vertex = active_vertex.index

            stored_edges = [[vertex.index for vertex in edge.verts] for edge in selected_edges]
            if start_vertex is None and stored_edges:
                start_vertex = stored_edges[0][0]

        if start_vertex is None or len(stored_edges) == 0:
            self.report({'WARNING'}, "Select a path edge chain or path face first.")
            return {'CANCELLED'}

        cap_buf.list_ek[:] = stored_edges
        cap_buf.list_sp[:] = [start_vertex]
        cap_buf.mesh_name = object_active.data.name

        path_vertices = cap_buf.path_vertex_indices
        for face in selected_faces:
            face.select_set(False)
        for edge in selected_edges:
            edge.select_set(False)
        for vertex in bm.verts:
            if vertex.index in path_vertices:
                vertex.select_set(False)
        bmesh.update_edit_mesh(object_active.data)

        self.report({'INFO'}, "Carve path stored.")
        return {'FINISHED'}


def _capture_profile(bm):
    profile_vertices = [vertex for vertex in bm.verts if vertex.select]
    if not profile_vertices:
        return None, None, None, None, None

    profile_indices = [vertex.index for vertex in profile_vertices]
    index_lookup = {index: offset for offset, index in enumerate(profile_indices)}
    profile_coordinates = [vertex.co.copy() for vertex in profile_vertices]

    profile_edges = []
    seen_edges = set()
    for edge in bm.edges:
        first = edge.verts[0].index
        second = edge.verts[1].index
        if first not in index_lookup or second not in index_lookup:
            continue
        edge_key = frozenset((first, second))
        if edge_key in seen_edges:
            continue
        seen_edges.add(edge_key)
        profile_edges.append((index_lookup[first], index_lookup[second]))

    profile_faces = []
    profile_face_indices = set()
    for face in bm.faces:
        face_indices = [vertex.index for vertex in face.verts]
        # A face profile can be selected in vertex or edge mode, where
        # Blender does not mark the face itself selected.  The selected
        # vertices still describe the profile, so preserve the face too.
        if all(index in index_lookup for index in face_indices):
            profile_faces.append([index_lookup[index] for index in face_indices])
            profile_face_indices.add(face.index)

    return profile_indices, profile_coordinates, profile_edges, profile_faces, profile_face_indices


def _find_source_cleanup_vertices(bm, profile_indices, profile_face_indices, path_edges):
    """Find isolated profile/path islands that can be removed after capture.

    Extrude Along Path leaves its source geometry in the active mesh.  A carve
    consumes that source as the Boolean cutter, so remove it only when the
    selected profile/path is a self-contained island.  Geometry that is part
    of the target solid is deliberately preserved.
    """
    source_vertices = set(profile_indices)
    source_vertices.update(index for edge in path_edges for index in edge)
    if not source_vertices:
        return set()

    profile_edge_keys = {
        frozenset((edge.verts[0].index, edge.verts[1].index))
        for edge in bm.edges
        if edge.verts[0].index in profile_indices
        and edge.verts[1].index in profile_indices
    }
    path_edge_keys = {frozenset(edge) for edge in path_edges}
    allowed_edge_keys = profile_edge_keys | path_edge_keys

    cleanup_vertices = set()
    unvisited = set(source_vertices)
    while unvisited:
        start_index = unvisited.pop()
        component = {start_index}
        pending = [bm.verts[start_index]]
        while pending:
            vertex = pending.pop()
            for edge in vertex.link_edges:
                other = edge.other_vert(vertex)
                if other.index in source_vertices and other.index not in component:
                    component.add(other.index)
                    unvisited.discard(other.index)
                    pending.append(other)

        component_verts = [bm.verts[index] for index in component]
        component_edges = {
            edge
            for vertex in component_verts
            for edge in vertex.link_edges
        }
        component_faces = {
            face
            for edge in component_edges
            for face in edge.link_faces
        }

        is_self_contained = component.issubset(source_vertices)
        is_self_contained &= all(
            face.index in profile_face_indices for face in component_faces
        )
        is_self_contained &= all(
            frozenset((edge.verts[0].index, edge.verts[1].index)) in allowed_edge_keys
            for edge in component_edges
        )
        if is_self_contained:
            cleanup_vertices.update(component)

    return cleanup_vertices


def _remove_source_geometry(target, cleanup_vertices):
    if not cleanup_vertices:
        return

    cleanup_bm = bmesh.new()
    try:
        cleanup_bm.from_mesh(target.data)
        cleanup_bm.verts.ensure_lookup_table()
        cleanup_bm.edges.ensure_lookup_table()
        cleanup_bm.faces.ensure_lookup_table()

        valid_vertices = {
            cleanup_bm.verts[index]
            for index in cleanup_vertices
            if 0 <= index < len(cleanup_bm.verts)
        }
        if not valid_vertices:
            return

        faces_to_delete = [
            face for face in cleanup_bm.faces
            if all(vertex in valid_vertices for vertex in face.verts)
        ]
        # Do not consume an object that contains no remaining target faces.
        if faces_to_delete and len(faces_to_delete) == len(cleanup_bm.faces):
            return
        if faces_to_delete:
            bmesh.ops.delete(cleanup_bm, geom=faces_to_delete, context='FACES')

        edges_to_delete = [
            edge for edge in cleanup_bm.edges
            if all(vertex in valid_vertices for vertex in edge.verts)
        ]
        if edges_to_delete:
            bmesh.ops.delete(cleanup_bm, geom=edges_to_delete, context='EDGES')

        verts_to_delete = [
            vertex for vertex in cleanup_bm.verts
            if vertex in valid_vertices and not vertex.link_edges
        ]
        if verts_to_delete:
            bmesh.ops.delete(cleanup_bm, geom=verts_to_delete, context='VERTS')

        cleanup_bm.to_mesh(target.data)
        target.data.update()
    finally:
        cleanup_bm.free()


def _make_mesh_object(name, mesh, collection):
    object_new = bpy.data.objects.new(name, mesh)
    collection.objects.link(object_new)
    return object_new


def _make_profile_object(target, profile_coordinates, profile_edges, profile_faces):
    """Snapshot the selected profile so it can be restored after the carve."""
    profile_mesh = bpy.data.meshes.new(f"{target.name}_CarveProfileMesh")
    profile_mesh.from_pydata(
        profile_coordinates,
        profile_edges,
        profile_faces,
    )
    profile_mesh.update()
    profile_object = _make_mesh_object(
        f"{target.name}_CarveProfile",
        profile_mesh,
        target.users_collection[0] if target.users_collection else bpy.context.collection,
    )
    profile_object.matrix_world = target.matrix_world.copy()
    return profile_object


def _append_kept_profile(target, profile_object):
    """Append the preserved source profile to the carved target mesh."""
    if profile_object is None:
        return

    target_bm = bmesh.new()
    profile_bm = bmesh.new()
    try:
        target_bm.from_mesh(target.data)
        profile_bm.from_mesh(profile_object.data)
        profile_bm.verts.ensure_lookup_table()
        profile_bm.edges.ensure_lookup_table()
        profile_bm.faces.ensure_lookup_table()

        profile_to_target = (
            target.matrix_world.inverted_safe() @ profile_object.matrix_world
        )
        vertex_map = {
            vertex: target_bm.verts.new(profile_to_target @ vertex.co)
            for vertex in profile_bm.verts
        }

        for edge in profile_bm.edges:
            try:
                target_bm.edges.new([vertex_map[v] for v in edge.verts])
            except ValueError:
                pass

        for face in profile_bm.faces:
            try:
                target_bm.faces.new([vertex_map[v] for v in face.verts])
            except ValueError:
                pass

        target_bm.to_mesh(target.data)
        target.data.update()
    finally:
        profile_bm.free()
        target_bm.free()


def _remove_temporary_object(object_mesh):
    if object_mesh is None:
        return
    mesh_data = object_mesh.data if object_mesh.type == 'MESH' else None
    if object_mesh.name in bpy.data.objects:
        bpy.data.objects.remove(object_mesh, do_unlink=True)
    if mesh_data is not None and mesh_data.users == 0 and mesh_data.name in bpy.data.meshes:
        bpy.data.meshes.remove(mesh_data)


def _deselect_all_objects(context):
    for object_scene in context.view_layer.objects:
        object_scene.select_set(False)


def _join_cutter_for_cool_bool(context, target, cutter):
    """Join EAP's result and make its island the active Cool Bool island."""
    marker = bpy.data.materials.new("__rcad_carve_cutter_marker__")
    placeholder = None

    # A target without material slots would otherwise share slot zero with the
    # marker after Object > Join.  Give it a temporary slot so the cutter can
    # be identified unambiguously after the join.
    if len(target.data.materials) == 0:
        placeholder = bpy.data.materials.new("__rcad_carve_target_slot__")
        target.data.materials.append(placeholder)

    cutter.data.materials.append(marker)
    marker_slot = len(cutter.data.materials) - 1
    for polygon in cutter.data.polygons:
        polygon.material_index = marker_slot

    _deselect_all_objects(context)
    target.select_set(True)
    cutter.select_set(True)
    context.view_layer.objects.active = target
    result = bpy.ops.object.join()
    if 'FINISHED' not in result:
        _remove_temporary_materials([marker, placeholder])
        raise RuntimeError("Could not join the EAP cutter to the target")

    marker_slot = next(
        (
            index
            for index, material in enumerate(target.data.materials)
            if material == marker
        ),
        None,
    )
    if marker_slot is None:
        _remove_temporary_materials([marker, placeholder])
        raise RuntimeError("Could not identify the joined EAP cutter island")

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bm = bmesh.from_edit_mesh(target.data)
    bm.faces.ensure_lookup_table()
    cutter_faces = [
        face for face in bm.faces
        if face.material_index == marker_slot
    ]
    if not cutter_faces:
        _remove_temporary_materials([marker, placeholder])
        raise RuntimeError("The EAP cutter has no faces to subtract")

    # Cool Bool uses the active BMesh element to decide which selected island
    # is the cutter.  Preserve the manual "cutter first, target second"
    # workflow explicitly instead of relying on Blender's join history.
    bm.select_history.clear()
    bm.select_history.add(cutter_faces[0])
    bmesh.update_edit_mesh(target.data)
    return marker, placeholder


def _remove_temporary_materials(materials):
    """Remove the private join markers from any objects left by Cool Bool."""
    if not materials:
        return

    if bpy.context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    for object_scene in list(bpy.data.objects):
        if object_scene.type != 'MESH':
            continue
        slots = object_scene.data.materials
        for index in range(len(slots) - 1, -1, -1):
            if slots[index] in materials:
                slots.pop(index=index)

    # Cool Bool's temporary separation can leave orphan mesh datablocks with
    # copied material slots.  Clear those slots as well so the private marker
    # materials do not accumulate across repeated carves.
    for mesh_data in list(bpy.data.meshes):
        slots = mesh_data.materials
        for index in range(len(slots) - 1, -1, -1):
            if slots[index] in materials:
                slots.pop(index=index)

    for material in materials:
        if material is not None and material.users == 0:
            bpy.data.materials.remove(material)


class OT_CarveAlongPath_Carve(bpy.types.Operator):
    bl_label = "Carve"
    bl_idname = "mesh.cap_carve"
    bl_options = {'REGISTER', 'UNDO'}

    invert_cutter: bpy.props.BoolProperty(
        name="Invert",
        default=False,
        description="Reverse the swept cutter so Difference keeps the opposite side",
    )
    keep_profile: bpy.props.BoolProperty(
        name="Keep Profile",
        default=False,
        get=_get_keep_profile,
        set=_set_keep_profile,
        description="Keep the selected profile used to create the carve",
    )

    def draw(self, context):
        row = self.layout.row(align=True)
        row.prop(self, "invert_cutter")
        row.prop(self, "keep_profile")

    def execute(self, context):
        target = context.active_object
        keep_profile = bool(self.keep_profile)
        if not _active_mesh_in_edit_mode(context):
            self.report({'ERROR'}, "Select a mesh in Edit Mode before carving.")
            return {'CANCELLED'}
        if cap_buf.mesh_name and target.data.name != cap_buf.mesh_name:
            self.report({'ERROR'}, "The stored carve path belongs to a different mesh.")
            return {'CANCELLED'}
        if not cap_buf.list_ek or not cap_buf.list_sp:
            self.report({'ERROR'}, "No path edges stored. Store a path first.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(target.data)
        check_lukap(bm)
        captured = _capture_profile(bm)
        (
            profile_indices,
            profile_coordinates,
            profile_edges,
            profile_faces,
            profile_face_indices,
        ) = captured
        if not profile_indices:
            self.report({'ERROR'}, "Select a face profile before carving.")
            return {'CANCELLED'}
        if not profile_edges:
            self.report({'ERROR'}, "The selected profile has no connected edges to sweep.")
            return {'CANCELLED'}

        if any(index >= len(bm.verts) for index in cap_buf.path_vertex_indices):
            self.report({'ERROR'}, "The stored path no longer matches this mesh.")
            return {'CANCELLED'}

        cleanup_vertices = _find_source_cleanup_vertices(
            bm,
            profile_indices,
            profile_face_indices,
            cap_buf.list_ek,
        )

        path_edges = cap_buf.list_ek[:]
        path_start = cap_buf.list_sp[0]
        solver = getattr(context.scene, "carve_along_path_solver", 'EXACT')
        cutter = None
        source_object = None
        profile_object = None
        temporary_materials = []
        wm = context.window_manager
        wm.progress_begin(0, 100)

        try:
            if keep_profile and (profile_faces or profile_edges):
                profile_object = _make_profile_object(
                    target,
                    profile_coordinates,
                    profile_edges,
                    profile_faces,
                )

            # Make the exact source mesh EAP expects, with only the selected
            # profile selected.  EAP then owns the complete face extrusion
            # and cutter separation on this temporary object.
            bmesh.update_edit_mesh(target.data)
            source_mesh = bpy.data.meshes.new(f"{target.name}_CarveSourceMesh")
            source_object = _make_mesh_object(
                f"{target.name}_CarveSource",
                source_mesh,
                target.users_collection[0] if target.users_collection else context.collection,
            )

            source_object.matrix_world = target.matrix_world.copy()
            source_bm = bm.copy()
            check_lukap(source_bm)
            for vertex in source_bm.verts:
                vertex.select_set(False)
            for edge in source_bm.edges:
                edge.select_set(False)
            for face in source_bm.faces:
                face.select_set(False)
            for index in profile_indices:
                source_bm.verts[index].select_set(True)
            for index in profile_face_indices:
                if 0 <= index < len(source_bm.faces):
                    source_bm.faces[index].select_set(True)
            source_bm.to_mesh(source_mesh)
            source_bm.free()
            source_mesh.update()

            _deselect_all_objects(context)
            source_object.select_set(True)
            context.view_layer.objects.active = source_object
            bpy.ops.object.mode_set(mode='EDIT')

            cutter, error_message = extrude_faces_to_cutter(
                context,
                source_object,
                path_edges,
                path_start,
            )
            if cutter is None:
                raise RuntimeError(error_message or "EAP did not create a cutter")

            bpy.ops.object.mode_set(mode='OBJECT')
            _remove_temporary_object(source_object)
            source_object = None
            cutter.matrix_world = target.matrix_world.copy()

            _deselect_all_objects(context)
            target.select_set(True)
            context.view_layer.objects.active = target
            _remove_source_geometry(target, cleanup_vertices)
            # The operation order is deliberate: EAP has already produced
            # the cutter above.  Now perform the same joined-island setup that
            # Cool Bool expects, with the cutter as the active island, and run
            # the copied Cool Bool Difference implementation.
            marker, placeholder = _join_cutter_for_cool_bool(
                context,
                target,
                cutter,
            )
            temporary_materials = [marker, placeholder]
            cutter = None
            subtract_selected_islands(
                context,
                invert_cutter=self.invert_cutter,
                solver_mode=solver,
            )

            _remove_temporary_materials(temporary_materials)
            temporary_materials = []
            _append_kept_profile(target, profile_object)
            _remove_temporary_object(profile_object)
            profile_object = None

            _deselect_all_objects(context)
            target.select_set(True)
            context.view_layer.objects.active = target
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='DESELECT')
            self.report({'INFO'}, "Carve Along Path finished.")
            return {'FINISHED'}
        except Exception as error:
            self.report({'ERROR'}, f"Carve Along Path failed: {error}")
            try:
                if context.mode != 'EDIT_MESH':
                    _deselect_all_objects(context)
                    target.select_set(True)
                    context.view_layer.objects.active = target
                    bpy.ops.object.mode_set(mode='EDIT')
            except Exception:
                pass
            return {'CANCELLED'}
        finally:
            if source_object is not None and source_object.name in bpy.data.objects:
                if context.mode != 'OBJECT':
                    try:
                        bpy.ops.object.mode_set(mode='OBJECT')
                    except Exception:
                        pass
                _remove_temporary_object(source_object)
            if cutter is not None and cutter.name in bpy.data.objects:
                _remove_temporary_object(cutter)
            if profile_object is not None and profile_object.name in bpy.data.objects:
                _remove_temporary_object(profile_object)
            if temporary_materials:
                try:
                    _remove_temporary_materials(temporary_materials)
                except Exception:
                    pass
            wm.progress_end()
