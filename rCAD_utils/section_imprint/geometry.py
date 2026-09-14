"""BMesh geometry helpers for the edit-mode section imprint tool.

The section result is built from Blender's own mesh-editing operations.  The
source edit mesh is copied first, then the copy is processed with the same
operation sequence used by Blender's Edit Mode Bisect tool:

    bisect_plane -> triangle_fill -> face_attribute_fill

Only the newly created planar fill faces are copied into the result mesh.
"""

from collections import defaultdict, deque
from dataclasses import dataclass
from math import isfinite

import bmesh
from mathutils import Vector


DEFAULT_TOLERANCE = 1.0e-5
_GROUP_LAYER_NAME = "__rcad_section_group"
_PART_LAYER_NAME = "rcad_section_part"
_AREA_EPSILON = 1.0e-12


@dataclass
class SectionFace:
    """One generated section face in world coordinates."""

    points: tuple
    material_index: int


def _visible_faces(bm):
    return [
        face
        for face in bm.faces
        if face.is_valid and not face.hide and len(face.verts) >= 3
    ]


def _face_neighbours(face, visible):
    """Return manifold neighbours of a face in the visible face set."""

    neighbours = []
    for edge in face.edges:
        if len(edge.link_faces) != 2:
            continue
        for other in edge.link_faces:
            if other is not face and other in visible and other.is_valid:
                neighbours.append(other)
    return neighbours


def _selection_seeds(bm, visible):
    selected_faces = [face for face in visible if face.select]
    if selected_faces:
        return selected_faces

    selected_vertices = {
        vert
        for vert in bm.verts
        if vert.is_valid and not vert.hide and vert.select
    }
    if selected_vertices:
        fully_selected = [
            face
            for face in visible
            if all(vert in selected_vertices for vert in face.verts)
        ]
        if fully_selected:
            return fully_selected

    selected_edges = {
        edge
        for edge in bm.edges
        if edge.is_valid and not edge.hide and edge.select
    }
    if selected_edges:
        return [
            face
            for face in visible
            if any(edge in selected_edges for edge in face.edges)
        ]

    return []


def selected_faces(bm, selection_only=True):
    """Return the visible components selected for the section operation.

    Selecting one face is treated as selecting its connected manifold part.
    This makes the tool practical for closed solids: the user can select a
    face on a part instead of having to select every face of that part.
    Non-manifold edges remain part boundaries.
    """

    visible = _visible_faces(bm)
    if not selection_only:
        return visible

    seeds = _selection_seeds(bm, visible)
    if not seeds:
        return []

    visible_set = set(visible)
    result = []
    seen = set()
    queue = deque(seeds)
    while queue:
        face = queue.popleft()
        if face in seen or face not in visible_set or not face.is_valid:
            continue
        seen.add(face)
        result.append(face)
        for neighbour in _face_neighbours(face, visible_set):
            if neighbour not in seen:
                queue.append(neighbour)
    return result


def face_groups(bm, faces):
    """Split faces into manifold-connected parts."""

    faces = [face for face in faces if face.is_valid]
    if not faces:
        return []

    wanted = set(faces)
    adjacency = {face: set() for face in faces}
    for edge in bm.edges:
        linked = [
            face
            for face in edge.link_faces
            if face in wanted and face.is_valid
        ]
        if len(linked) < 2 or len(edge.link_faces) != 2:
            continue
        for left_index, left in enumerate(linked[:-1]):
            for right in linked[left_index + 1:]:
                adjacency[left].add(right)
                adjacency[right].add(left)

    order = {face: index for index, face in enumerate(faces)}
    unseen = set(faces)
    groups = []
    while unseen:
        seed = min(unseen, key=lambda face: order[face])
        unseen.remove(seed)
        queue = deque([seed])
        group = []
        while queue:
            face = queue.popleft()
            group.append(face)
            for neighbour in sorted(adjacency[face], key=lambda item: order[item]):
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    queue.append(neighbour)
        groups.append(group)
    return groups


def tag_face_groups(bm, groups, layer_name=_GROUP_LAYER_NAME):
    """Tag each face with the index of its source part.

    Creating a new BMesh custom-data layer can invalidate Python face
    wrappers.  Face indices let us refresh the groups after the layer is
    created, so callers never retain stale BMFace references.
    """

    bm.faces.ensure_lookup_table()
    group_indices = [
        [face.index for face in group if face.is_valid]
        for group in groups
    ]
    layer = bm.faces.layers.int.get(layer_name)
    if layer is None:
        layer = bm.faces.layers.int.new(layer_name)

    for face in bm.faces:
        face[layer] = -1
    bm.faces.ensure_lookup_table()
    refreshed_groups = []
    for group_index, indices in enumerate(group_indices):
        refreshed_group = []
        for face_index in indices:
            face = bm.faces[face_index]
            face[layer] = group_index
            refreshed_group.append(face)
        refreshed_groups.append(refreshed_group)
    return layer, refreshed_groups


def copy_face_group(bm, group_index, layer_name=_GROUP_LAYER_NAME):
    """Return a copied BMesh containing one tagged source part."""

    work = bm.copy()
    layer = work.faces.layers.int.get(layer_name)
    if layer is None:
        work.free()
        raise RuntimeError("Section group data was lost while copying the mesh.")

    delete_faces = [
        face
        for face in work.faces
        if face.is_valid and face[layer] != group_index
    ]
    if delete_faces:
        bmesh.ops.delete(work, geom=delete_faces, context='FACES')

    loose_edges = [
        edge
        for edge in work.edges
        if edge.is_valid and not edge.link_faces
    ]
    if loose_edges:
        bmesh.ops.delete(work, geom=loose_edges, context='EDGES')

    loose_verts = [
        vert
        for vert in work.verts
        if vert.is_valid and not vert.link_edges
    ]
    if loose_verts:
        bmesh.ops.delete(work, geom=loose_verts, context='VERTS')
    return work


def dominant_material_index(faces):
    """Choose a stable fallback material for a source part."""

    weights = defaultdict(float)
    for face in faces:
        try:
            area = max(float(face.calc_area()), 1.0)
        except Exception:
            area = 1.0
        weights[int(face.material_index)] += area
    if not weights:
        return 0
    return max(weights, key=lambda index: (weights[index], -index))


def _local_plane_tolerance(matrix_world, tolerance):
    """Convert a world tolerance to a conservative local-space tolerance."""

    matrix = matrix_world.to_3x3()
    scales = [matrix.col[index].length for index in range(3)]
    scale = max(scales) if scales else 1.0
    if not isfinite(scale) or scale <= 1.0e-12:
        scale = 1.0
    return max(float(tolerance) / scale, 1.0e-10)


def _vertex_key(coordinate, tolerance):
    scale = max(float(tolerance), 1.0e-10)
    return tuple(
        int(round(float(component) / scale))
        for component in coordinate
    )


def _face_signature(face, tolerance):
    return tuple(
        sorted(_vertex_key(vert.co, tolerance) for vert in face.verts)
    )


def _face_on_plane(face, plane_co, plane_no, tolerance):
    return all(
        abs((vert.co - plane_co).dot(plane_no)) <= tolerance * 2.0
        for vert in face.verts
    )


def _adjacent_material_index(face, fallback):
    """Choose a material from faces surrounding a newly filled face."""

    counts = defaultdict(float)
    for edge in face.edges:
        for neighbour in edge.link_faces:
            if neighbour is face or not neighbour.is_valid:
                continue
            try:
                weight = max(float(neighbour.calc_area()), 1.0e-12)
            except Exception:
                weight = 1.0
            counts[int(neighbour.material_index)] += weight
    if not counts:
        return int(fallback)
    return max(counts, key=lambda index: (counts[index], -index))


def bisect_section_faces(
    work_bm,
    matrix_world,
    plane_origin_world,
    plane_normal_world,
    tolerance=DEFAULT_TOLERANCE,
    fallback_material=0,
):
    """Bisect a copied part and return only its generated cap faces.

    This deliberately uses the same BMesh operations as Blender's Edit Mode
    Bisect tool.  Both clear flags stay disabled because the source part is
    only an analysis copy; the result mesh receives the filled section faces.
    """

    matrix = matrix_world.copy()
    inverse = matrix.inverted_safe()
    plane_co_local = inverse @ plane_origin_world
    plane_no_local = (
        inverse.to_3x3().transposed() @ plane_normal_world
    )
    if plane_no_local.length <= 1.0e-12:
        return []
    plane_no_local.normalize()

    local_tolerance = _local_plane_tolerance(matrix, tolerance)
    all_geometry = (
        list(work_bm.verts)
        + list(work_bm.edges)
        + list(work_bm.faces)
    )
    bisect_result = bmesh.ops.bisect_plane(
        work_bm,
        geom=all_geometry,
        dist=local_tolerance,
        plane_co=plane_co_local,
        plane_no=plane_no_local,
        use_snap_center=False,
        clear_inner=False,
        clear_outer=False,
    )

    cut_edges = [
        element
        for element in bisect_result.get("geom_cut", ())
        if isinstance(element, bmesh.types.BMEdge) and element.is_valid
    ]
    if not cut_edges:
        return []

    existing_signatures = {
        _face_signature(face, local_tolerance)
        for face in work_bm.faces
        if face.is_valid
    }

    fill_result = bmesh.ops.triangle_fill(
        work_bm,
        edges=cut_edges,
        use_beauty=False,
        use_dissolve=True,
        normal=plane_no_local,
    )
    candidate_faces = [
        element
        for element in fill_result.get("geom", ())
        if isinstance(element, bmesh.types.BMFace) and element.is_valid
    ]
    if not candidate_faces:
        return []

    candidate_signatures = {
        _face_signature(face, local_tolerance)
        for face in candidate_faces
    }
    fallback_by_signature = {
        _face_signature(face, local_tolerance): _adjacent_material_index(
            face,
            fallback_material,
        )
        for face in candidate_faces
    }

    # Blender's attribute-fill operation may replace a newly-created face
    # while copying its face/loop custom data.  Therefore the original face
    # references are only used as input; the result is found again by its
    # geometry signature below.
    attribute_fill_ok = True
    try:
        bmesh.ops.face_attribute_fill(
            work_bm,
            faces=candidate_faces,
            use_normals=True,
            use_data=True,
        )
    except (AttributeError, RuntimeError, ValueError):
        # Older Blender builds may not expose this helper.  Material fallback
        # still keeps the section result usable.
        attribute_fill_ok = False
        pass

    section_faces = []
    seen_signatures = set()
    for face in work_bm.faces:
        if not face.is_valid:
            continue
        signature = _face_signature(face, local_tolerance)
        if signature not in candidate_signatures:
            continue
        if signature in existing_signatures or signature in seen_signatures:
            continue
        if not _face_on_plane(
            face,
            plane_co_local,
            plane_no_local,
            local_tolerance,
        ):
            continue
        if len(face.verts) < 3 or face.calc_area() <= _AREA_EPSILON:
            continue

        points = tuple(
            (matrix @ vert.co).copy()
            for vert in face.verts
        )
        if attribute_fill_ok:
            material_index = int(face.material_index)
        else:
            material_index = fallback_by_signature.get(
                signature,
                fallback_material,
            )
        section_faces.append(SectionFace(points, material_index))
        seen_signatures.add(signature)

    return section_faces


def _oriented_points(points, normal):
    """Return points wound toward the supplied world-space normal."""

    points = tuple(Vector(point) for point in points)
    if len(points) < 3:
        return points

    accumulated = Vector()
    for index, point in enumerate(points):
        previous = points[index - 1]
        accumulated.x += (previous.y - point.y) * (previous.z + point.z)
        accumulated.y += (previous.z - point.z) * (previous.x + point.x)
        accumulated.z += (previous.x - point.x) * (previous.y + point.y)
    if accumulated.dot(normal) < 0.0:
        return tuple(reversed(points))
    return points


def add_section_face(
    output_bm,
    section_face,
    plane_normal,
    material_index,
    part_index,
    vertex_cache,
    tolerance=DEFAULT_TOLERANCE,
):
    """Copy one section face into the independent result BMesh."""

    part_layer = output_bm.faces.layers.int.get(_PART_LAYER_NAME)
    if part_layer is None:
        part_layer = output_bm.faces.layers.int.new(_PART_LAYER_NAME)

    points = _oriented_points(section_face.points, plane_normal)
    if len(points) < 3:
        return False

    vertices = []
    for point in points:
        key = _vertex_key(point, tolerance)
        vert = vertex_cache.get(key)
        if vert is None or not vert.is_valid:
            vert = output_bm.verts.new(tuple(point))
            vertex_cache[key] = vert
        vertices.append(vert)

    compact_vertices = []
    for vert in vertices:
        if not compact_vertices or compact_vertices[-1] is not vert:
            compact_vertices.append(vert)
    if len(compact_vertices) > 1 and compact_vertices[0] is compact_vertices[-1]:
        compact_vertices.pop()
    if len(compact_vertices) < 3:
        return False

    try:
        face = output_bm.faces.new(compact_vertices)
    except ValueError:
        return False

    face.material_index = int(material_index)
    face[part_layer] = int(part_index)
    return True
