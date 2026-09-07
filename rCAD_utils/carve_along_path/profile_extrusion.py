"""Prepare the complete profile before invoking the unchanged EAP sweep."""

from mathutils.geometry import closest_point_on_tri


def find_profile_vertices_on_edges(bm, profile_indices, matrix_world):
    """Return interior profile vertices that lie on another mesh edge.

    A profile point that lands on a target edge creates a zero-width corner
    for the sweep.  Treat that point as a redundant point in the profile.  A
    profile endpoint is kept because endpoints are used to determine the
    normal extrusion direction.
    """
    bm.verts.ensure_lookup_table()
    profile = {
        bm.verts[index]
        for index in profile_indices
        if 0 <= index < len(bm.verts)
    }
    if not profile:
        return set()

    profile_edge_keys = {
        frozenset((edge.verts[0].index, edge.verts[1].index))
        for edge in bm.edges
        if edge.verts[0] in profile and edge.verts[1] in profile
    }
    profile_neighbors = {vertex: [] for vertex in profile}
    for edge in bm.edges:
        first, second = edge.verts
        if first in profile and second in profile:
            profile_neighbors[first].append(second)
            profile_neighbors[second].append(first)

    # Only remove a point in the middle of the profile.  The two endpoints
    # must remain so endpoint extrusion can still find their outgoing edge.
    interior = [
        vertex for vertex, neighbors in profile_neighbors.items()
        if len(neighbors) == 2
    ]
    if not interior:
        return set()

    world_points = {
        vertex: matrix_world @ vertex.co
        for vertex in profile
    }
    mesh_edges = []
    for edge in bm.edges:
        key = frozenset((edge.verts[0].index, edge.verts[1].index))
        if key in profile_edge_keys:
            continue
        first, second = edge.verts
        first_world = matrix_world @ first.co
        second_world = matrix_world @ second.co
        edge_vector = second_world - first_world
        length_squared = edge_vector.length_squared
        if length_squared <= 1e-20:
            continue
        # Use the edge length as the scale so this works for both the default
        # cube and objects that have been scaled in the scene.
        tolerance = max(edge_vector.length * 1e-5, 1e-7)
        mesh_edges.append((first_world, edge_vector, length_squared, tolerance))

    removed = set()
    for vertex in interior:
        point = world_points[vertex]
        for first_world, edge_vector, length_squared, tolerance in mesh_edges:
            parameter = (point - first_world).dot(edge_vector) / length_squared
            # A profile point that is itself a target corner is part of the
            # target topology.  Only collapse points in the interior of an
            # edge, which is the redundant seam point this cleanup targets.
            if parameter <= 1e-5 or parameter >= 1.0 - 1e-5:
                continue
            closest = first_world + edge_vector * parameter
            if (point - closest).length <= tolerance:
                removed.add(vertex.index)
                break
    return removed


def profile_bridge_pairs(bm, profile_indices, removed_indices):
    """Find the retained neighbors that should be joined across removals."""
    profile_set = set(profile_indices)
    removed = profile_set.intersection(removed_indices)
    if not removed:
        return set()

    adjacency = {index: set() for index in profile_set}
    for edge in bm.edges:
        first, second = edge.verts[0].index, edge.verts[1].index
        if first in profile_set and second in profile_set:
            adjacency[first].add(second)
            adjacency[second].add(first)

    resolved = set()
    for start in removed:
        retained_neighbors = set()
        pending = [(start, None)]
        visited = {start}
        while pending:
            current, previous = pending.pop()
            for neighbor in adjacency[current]:
                if neighbor == previous or neighbor in visited:
                    continue
                visited.add(neighbor)
                if neighbor in removed:
                    pending.append((neighbor, current))
                else:
                    retained_neighbors.add(neighbor)
        if len(retained_neighbors) == 2:
            first, second = sorted(retained_neighbors)
            resolved.add((first, second))
    return resolved


def connect_profile_across_removed_vertices(bm, profile_indices, removed_indices):
    """Add profile edges that skip points removed from the sweep profile."""
    bridges = profile_bridge_pairs(bm, profile_indices, removed_indices)
    if not bridges:
        return

    bm.verts.ensure_lookup_table()
    existing = {
        frozenset((edge.verts[0].index, edge.verts[1].index))
        for edge in bm.edges
    }
    for first, second in bridges:
        key = frozenset((first, second))
        if key not in existing:
            bm.edges.new((bm.verts[first], bm.verts[second]))
            existing.add(key)
    bm.edges.index_update()


def extrude_profile_endpoints(bm, profile_indices, matrix_world):
    """Keep the original profile and add one edge beyond each touching face.

    Return the complete profile vertex list, including the new endpoints.
    Face normals and extrusion distances are evaluated in world space.
    """
    bm.normal_update()
    bm.verts.ensure_lookup_table()
    profile = {bm.verts[index] for index in profile_indices}
    neighbors = {vertex: [] for vertex in profile}
    for edge in bm.edges:
        a, b = edge.verts
        if a in profile and b in profile:
            neighbors[a].append(b)
            neighbors[b].append(a)
    endpoints = [v for v in profile if len(neighbors[v]) == 1]
    if not endpoints:
        return list(profile_indices)

    inverse = matrix_world.inverted()
    normal_matrix = matrix_world.to_3x3().inverted().transposed()
    triangles = []
    for loops in bm.calc_loop_triangles():
        face = loops[0].face
        if all(v in profile for v in face.verts):
            continue
        coordinates = [matrix_world @ loop.vert.co for loop in loops]
        size = max((a - b).length for a in coordinates for b in coordinates)
        if size == 0:
            continue
        normal = (normal_matrix @ face.normal).normalized()
        triangles.append((coordinates, normal, size))

    additions = []
    for endpoint in endpoints:
        point = matrix_world @ endpoint.co
        outgoing = (point - matrix_world @ neighbors[endpoint][0].co).normalized()
        candidates = []
        for coordinates, normal, size in triangles:
            closest = closest_point_on_tri(point, *coordinates)
            gap = (closest - point).length
            tolerance = max(size * 1e-5, 1e-7)
            if gap <= tolerance and outgoing.dot(normal) > 1e-6:
                candidates.append((outgoing.dot(normal), normal, size, coordinates[0]))
        if not candidates:
            continue
        _, normal, size, face_point = max(candidates, key=lambda item: item[0])
        signed_distance = (point - face_point).dot(normal)
        # Create a new endpoint; never move the original vertex.
        coordinate = inverse @ (point + normal * (size * .01 - signed_distance))
        vertex = bm.verts.new(coordinate)
        bm.edges.new((endpoint, vertex))
        additions.append(vertex)

    bm.verts.index_update()
    bm.edges.index_update()
    return list(profile_indices) + [vertex.index for vertex in additions]


def select_complete_profile(bm, profile_indices):
    """Select the original curve AND its extrusion edges in every select mode."""
    selected = set(profile_indices)
    for vertex in bm.verts:
        vertex.select = vertex.index in selected
    for edge in bm.edges:
        edge.select = all(vertex.index in selected for vertex in edge.verts)
    for face in bm.faces:
        face.select = all(vertex.index in selected for vertex in face.verts)
    bm.select_history.clear()
