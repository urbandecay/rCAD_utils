"""Prepare the complete profile before invoking the unchanged EAP sweep."""

from mathutils.geometry import closest_point_on_tri


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
