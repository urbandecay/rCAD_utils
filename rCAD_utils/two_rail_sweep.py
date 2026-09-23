"""Two-rail mesh sweep operators."""

import json
from bisect import bisect_right
from collections import defaultdict
from itertools import permutations, product

import bpy
import bmesh
from mathutils import Vector


def _edge_components(edges):
    """Return connected edge components from a BMesh edge collection."""
    adjacency = defaultdict(list)
    for edge in edges:
        for vert in edge.verts:
            adjacency[vert].append(edge)

    unseen = set(edges)
    components = []
    while unseen:
        seed = next(iter(unseen))
        component = set()
        stack = [seed.verts[0]]
        visited_verts = set()
        while stack:
            vert = stack.pop()
            if vert in visited_verts:
                continue
            visited_verts.add(vert)
            for edge in adjacency[vert]:
                if edge in unseen:
                    unseen.remove(edge)
                    component.add(edge)
                other = edge.other_vert(vert)
                if other not in visited_verts:
                    stack.append(other)
        components.append(component)
    return components


def _ordered_open_chain(edges):
    """Order one connected, non-branching, open edge chain."""
    adjacency = defaultdict(list)
    vertices = set()
    for edge in edges:
        a, b = edge.verts
        adjacency[a].append(b)
        adjacency[b].append(a)
        vertices.update((a, b))

    endpoints = [vert for vert in vertices if len(adjacency[vert]) == 1]
    if len(endpoints) != 2 or any(len(adjacency[vert]) > 2 for vert in vertices):
        raise ValueError("Each rail and the profile must be a single open, unbranched edge chain.")

    start = min(endpoints, key=lambda vert: vert.index)
    ordered = [start]
    previous = None
    current = start
    while True:
        next_verts = [vert for vert in adjacency[current] if vert != previous]
        if not next_verts:
            break
        if len(next_verts) != 1:
            raise ValueError("The selected edges branch and cannot be used as a chain.")
        following = next_verts[0]
        if following in ordered:
            raise ValueError("The selected edge chain contains a loop.")
        ordered.append(following)
        previous, current = current, following

    if len(ordered) != len(vertices) or len(ordered) != len(edges) + 1:
        raise ValueError("The selected edges are disconnected or do not form one open chain.")
    return ordered


def _selected_edges_or_vertex_chain(bm):
    selected_edges = [edge for edge in bm.edges if edge.select and edge.is_valid]
    if selected_edges:
        return selected_edges

    selected_verts = {vert for vert in bm.verts if vert.select and vert.is_valid}
    if not selected_verts:
        return []
    return [
        edge for edge in bm.edges
        if edge.verts[0] in selected_verts and edge.verts[1] in selected_verts
    ]


def _get_stored_rail_vertices(scene, bm):
    """Resolve the stored vertex chains and ensure their edges still exist."""
    raw = scene.rcad_two_rail_sweep_paths
    try:
        index_paths = json.loads(raw)
    except (TypeError, ValueError):
        raise ValueError("Store the two rails first.")

    if not isinstance(index_paths, list) or len(index_paths) != 2:
        raise ValueError("Stored rail data is invalid. Store the two rails again.")

    bm.verts.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.ensure_lookup_table()
    edge_pairs = {
        frozenset((edge.verts[0].index, edge.verts[1].index))
        for edge in bm.edges if edge.is_valid
    }

    paths = []
    for indices in index_paths:
        if not isinstance(indices, list) or len(indices) < 2:
            raise ValueError("Stored rail data is invalid. Store the two rails again.")
        if any(not isinstance(index, int) or index < 0 or index >= len(bm.verts) for index in indices):
            raise ValueError("Mesh topology changed after storing the rails. Store them again.")
        if any(frozenset((a, b)) not in edge_pairs for a, b in zip(indices, indices[1:])):
            raise ValueError("Mesh topology changed after storing the rails. Store them again.")
        paths.append([bm.verts[index].co.copy() for index in indices])
    return paths


def _prepare_path(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + (b - a).length)
    total = lengths[-1]
    if total <= 1e-10:
        raise ValueError("A stored rail has no measurable length.")
    return points, lengths, total


def _point_at(path, distance_fraction):
    points, lengths, total = path
    distance = max(0.0, min(1.0, distance_fraction)) * total
    if distance <= 1e-12:
        return points[0].copy()
    if distance >= total - 1e-12:
        return points[-1].copy()

    index = min(max(bisect_right(lengths, distance) - 1, 0), len(lengths) - 2)
    while index < len(lengths) - 1:
        start, end = lengths[index], lengths[index + 1]
        segment_length = end - start
        if segment_length > 1e-12:
            factor = max(0.0, min(1.0, (distance - start) / segment_length))
            return points[index].lerp(points[index + 1], factor)
        index += 1
    return points[-1].copy()


def _tangent_at(path, distance_fraction):
    delta = 1e-4
    start = max(0.0, distance_fraction - delta)
    end = min(1.0, distance_fraction + delta)
    tangent = _point_at(path, end) - _point_at(path, start)
    if tangent.length <= 1e-10:
        points, lengths, _total = path
        segment_index = 0
        target = distance_fraction * lengths[-1]
        for index in range(len(lengths) - 1):
            if lengths[index + 1] >= target:
                segment_index = index
                break
        tangent = points[segment_index + 1] - points[segment_index]
    if tangent.length <= 1e-10:
        return Vector((0.0, 0.0, 1.0))
    return tangent.normalized()


def _unique_stations(*paths):
    values = [0.0, 1.0]
    for _points, lengths, total in paths:
        values.extend(length / total for length in lengths)
    values.sort()
    stations = []
    for value in values:
        if not stations or value - stations[-1] > 1e-8:
            stations.append(value)
    stations[0] = 0.0
    stations[-1] = 1.0
    return stations


def _orient_rails_to_profile(rails, profile_start, profile_end):
    best = None
    for rail_order in permutations(range(2)):
        for reverse_first, reverse_second in product((False, True), repeat=2):
            first = list(rails[rail_order[0]])
            second = list(rails[rail_order[1]])
            if reverse_first:
                first.reverse()
            if reverse_second:
                second.reverse()
            cost = (first[0] - profile_start).length + (second[0] - profile_end).length
            if best is None or cost < best[0]:
                best = (cost, first, second)
    return best[1], best[2]


def _projected_unit(vector, axis, fallback=None):
    projected = vector - axis * vector.dot(axis)
    if projected.length > 1e-8:
        return projected.normalized()
    if fallback is not None:
        projected = fallback - axis * fallback.dot(axis)
        if projected.length > 1e-8:
            return projected.normalized()
    basis = Vector((1.0, 0.0, 0.0))
    if abs(axis.dot(basis)) > 0.9:
        basis = Vector((0.0, 1.0, 0.0))
    return (basis - axis * basis.dot(axis)).normalized()


def _profile_positions(profile_coords, path_a, path_b, stations, scale_height=False):
    profile_start = profile_coords[0]
    profile_end = profile_coords[-1]
    source_axis = profile_end - profile_start
    profile_width = source_axis.length
    if profile_width <= 1e-10:
        raise ValueError("The profile endpoints must be different vertices.")
    source_axis.normalize()

    source_progress = _tangent_at(path_a, 0.0) + _tangent_at(path_b, 0.0)
    source_z = _projected_unit(source_progress, source_axis)
    source_y = source_z.cross(source_axis).normalized()
    source_z = source_axis.cross(source_y).normalized()

    output = []
    previous_z = None
    for station in stations:
        point_a = _point_at(path_a, station)
        point_b = _point_at(path_b, station)
        span = point_b - point_a
        span_length = span.length
        if span_length <= 1e-8:
            raise ValueError("The rails meet or cross at a sweep station.")
        target_x = span / span_length
        progress = _tangent_at(path_a, station) + _tangent_at(path_b, station)
        fallback_z = previous_z if previous_z is not None else source_z
        target_z = _projected_unit(progress, target_x, fallback_z)
        if previous_z is not None and target_z.dot(previous_z) < 0.0:
            target_z.negate()
        target_y = target_z.cross(target_x).normalized()
        target_z = target_x.cross(target_y).normalized()
        previous_z = target_z

        width_scale = span_length / profile_width
        layer = []
        for coordinate in profile_coords:
            relative = coordinate - profile_start
            x = relative.dot(source_axis) * width_scale
            y = relative.dot(source_y) * (width_scale if scale_height else 1.0)
            z = relative.dot(source_z)
            layer.append(point_a + target_x * x + target_y * y + target_z * z)
        output.append(layer)
    return output


class MESH_OT_StoreTwoRailSweepRails(bpy.types.Operator):
    bl_idname = "mesh.store_two_rail_sweep_rails"
    bl_label = "Store Rails"
    bl_description = "Store two selected open edge chains as the sweep rails"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.active_object is not None and context.active_object.type == 'MESH'

    def execute(self, context):
        obj = context.active_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.verts.index_update()

        selected_edges = _selected_edges_or_vertex_chain(bm)
        components = _edge_components(selected_edges)
        if len(components) != 2:
            self.report({'ERROR'}, "Select exactly two separate open edge chains for the rails.")
            return {'CANCELLED'}

        try:
            ordered_chains = [_ordered_open_chain(component) for component in components]
        except ValueError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        paths = [[vert.index for vert in chain] for chain in ordered_chains]
        context.scene.rcad_two_rail_sweep_object = obj
        context.scene.rcad_two_rail_sweep_paths = json.dumps(paths)

        # Clear the rail selection so a new profile can be selected immediately.
        for face in bm.faces:
            face.select_set(False)
        for edge in bm.edges:
            edge.select_set(False)
        for vert in bm.verts:
            vert.select_set(False)
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

        self.report({'INFO'}, "Two rails stored. Select an open profile chain and press Sweep.")
        return {'FINISHED'}


class MESH_OT_TwoRailSweep(bpy.types.Operator):
    bl_idname = "mesh.two_rail_sweep"
    bl_label = "Sweep"
    bl_description = "Sweep the selected open profile chain between the stored rails"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.active_object is not None and context.active_object.type == 'MESH'

    def execute(self, context):
        obj = context.active_object
        scene = context.scene
        if scene.rcad_two_rail_sweep_object != obj:
            self.report({'ERROR'}, "The stored rails belong to a different mesh. Store rails on this mesh first.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.verts.index_update()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        try:
            rails = _get_stored_rail_vertices(scene, bm)

            selected_faces = [face for face in bm.faces if face.select and face.is_valid]
            if selected_faces:
                raise ValueError("Select an open edge chain as the profile, not faces.")

            profile_edges = _selected_edges_or_vertex_chain(bm)
            if not profile_edges:
                raise ValueError("Select an open edge chain for the profile.")
            profile_components = _edge_components(profile_edges)
            if len(profile_components) != 1:
                raise ValueError("Select one connected open edge chain as the profile.")
            profile_verts = _ordered_open_chain(profile_components[0])

            selected_verts = {vert for vert in bm.verts if vert.select and vert.is_valid}
            if selected_verts and not selected_verts.issubset(set(profile_verts)):
                raise ValueError("The profile selection includes vertices outside its edge chain.")

            original_coords = [vert.co.copy() for vert in profile_verts]
            profile_coords = [coord.copy() for coord in original_coords]
            path_a_points, path_b_points = _orient_rails_to_profile(
                rails,
                profile_coords[0],
                profile_coords[-1],
            )
            path_a = _prepare_path(path_a_points)
            path_b = _prepare_path(path_b_points)
            stations = _unique_stations(path_a, path_b)
            positions = _profile_positions(
                profile_coords,
                path_a,
                path_b,
                stations,
                scale_height=scene.rcad_two_rail_sweep_scale_height,
            )
        except (ValueError, TypeError, IndexError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        new_verts = []
        new_faces = []
        try:
            for vert, coordinate in zip(profile_verts, positions[0]):
                vert.co = coordinate

            layers = [profile_verts]
            for layer_positions in positions[1:]:
                layer = []
                for coordinate in layer_positions:
                    vert = bm.verts.new(coordinate)
                    new_verts.append(vert)
                    layer.append(vert)
                layers.append(layer)

            for layer_index in range(len(layers) - 1):
                current_layer = layers[layer_index]
                next_layer = layers[layer_index + 1]
                for profile_index in range(len(profile_verts) - 1):
                    face = bm.faces.new((
                        current_layer[profile_index],
                        current_layer[profile_index + 1],
                        next_layer[profile_index + 1],
                        next_layer[profile_index],
                    ))
                    new_faces.append(face)
        except Exception as exc:
            for face in reversed(new_faces):
                if face.is_valid:
                    bm.faces.remove(face)
            for vert in reversed(new_verts):
                if vert.is_valid:
                    bm.verts.remove(vert)
            for vert, coordinate in zip(profile_verts, original_coords):
                if vert.is_valid:
                    vert.co = coordinate
            bmesh.update_edit_mesh(obj.data, loop_triangles=True, destructive=True)
            self.report({'ERROR'}, f"Could not create the swept faces: {exc}")
            return {'CANCELLED'}

        for face in bm.faces:
            face.select_set(False)
        for edge in bm.edges:
            edge.select_set(False)
        for vert in bm.verts:
            vert.select_set(False)
        for face in new_faces:
            face.select_set(True)
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data, loop_triangles=True, destructive=True)

        self.report({'INFO'}, f"Two-rail sweep created {len(new_faces)} faces.")
        return {'FINISHED'}
