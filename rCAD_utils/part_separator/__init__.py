"""Recover and separate welded mesh parts.

The important distinction here is between a *loose part* and a *manifold
shell*.  Blender's built-in Separate by Loose Parts follows vertex
connectivity.  That means a single accidental vertex weld makes two otherwise
independent boards one loose part.

This module uses two increasingly reliable sources of part identity:

* ``rcad_part_id`` — a face attribute written before risky editing.  This is
  the lossless path and survives ordinary vertex welding.
* manifold face shells — when no IDs are present, faces are connected only
  across edges with exactly two incident faces.  A vertex-only weld or a
  non-manifold weld seam therefore does not join two shells.

The second path is deliberately generic.  It does not assume that the part is
box-shaped, but it also does not pretend that a Boolean union can be reversed
without provenance.  See the operator descriptions for that boundary.
"""

from collections import defaultdict, deque
from math import isfinite

import bmesh
import bpy
from bpy.props import EnumProperty


PART_ATTRIBUTE = "rcad_part_id"
SOURCE_FACE_LAYER = "__rcad_source_face_index"


def _ensure_face_indices(bm):
    """Give the current BMesh faces stable indices for this operation."""
    bm.faces.ensure_lookup_table()
    for index, face in enumerate(bm.faces):
        face.index = index


def _face_groups_from_edges(bm, face_indices, manifold_only=True):
    """Return deterministic face groups from the BMesh topology.

    With ``manifold_only`` enabled, an edge is a bridge only when it has
    exactly two incident faces in the source mesh.  A welded seam generally
    has four (or more) incident faces, while an accidental vertex weld has no
    shared edge at all.  Both cases remain separate here.
    """
    _ensure_face_indices(bm)
    wanted = {int(index) for index in face_indices}
    faces = [face for face in bm.faces if face.index in wanted and not face.hide]
    if not faces:
        return []

    adjacency = {face: set() for face in faces}
    wanted_faces = set(faces)

    for edge in bm.edges:
        linked = [face for face in edge.link_faces if face in wanted_faces]
        if len(linked) < 2:
            continue
        if manifold_only and len(edge.link_faces) != 2:
            continue
        for left_index, left in enumerate(linked[:-1]):
            for right in linked[left_index + 1:]:
                adjacency[left].add(right)
                adjacency[right].add(left)

    groups = []
    unseen = set(faces)
    while unseen:
        seed = min(unseen, key=lambda face: face.index)
        unseen.remove(seed)
        queue = deque([seed])
        group = []
        while queue:
            face = queue.popleft()
            group.append(face.index)
            for neighbor in adjacency[face]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        groups.append(sorted(group))

    groups.sort(key=lambda group: group[0])
    return groups


def _group_face_indices(group):
    """Accept plain indices or ``(index, flip_normal)`` assignments."""
    return [_assignment_face_index(item) for item in group]


def _assignment_face_index(item):
    if isinstance(item, (tuple, list)):
        return int(item[0])
    return int(item)


def _ensure_source_face_layer(bm):
    """Store source polygon indices in a layer that survives BMesh copies."""
    layer = bm.faces.layers.int.get(SOURCE_FACE_LAYER)
    if layer is None:
        layer = bm.faces.layers.int.new(SOURCE_FACE_LAYER)
    bm.faces.ensure_lookup_table()
    for index, face in enumerate(bm.faces):
        face.index = index
        face[layer] = index
    return layer


def _face_info(bm, face_indices):
    """Return a small geometric summary for a face group."""
    faces = [bm.faces[index] for index in face_indices]
    if not faces:
        return None

    center = faces[0].calc_center_median()
    normal = faces[0].normal.copy()
    area = faces[0].calc_area()
    for face in faces[1:]:
        face_center = face.calc_center_median()
        face_area = face.calc_area()
        center += face_center
        area += face_area
        if normal.dot(face.normal) < 0.0:
            normal -= face.normal
        else:
            normal += face.normal
    center /= float(len(faces))
    if normal.length_squared > 1.0e-20:
        normal.normalize()

    planar = all(abs(abs(normal.dot(face.normal)) - 1.0) <= 1.0e-4 for face in faces)
    return {
        "faces": faces,
        "center": center,
        "normal": normal,
        "area": float(area),
        "planar": planar,
    }


def _recover_welded_box_groups(bm, face_indices, return_details=False):
    """Recover box shells when a weld removed coincident cap faces.

    ``weld_verts`` can remove one of two coincident coplanar faces.  The
    remaining cap then appears as a one-face group between two sets of side
    faces.  We attach that cap to both neighboring shells and reverse its
    winding for the shell on the opposite side.  This is what turns the
    four-board stack in the common 2x4 failure case back into four closed
    boards.

    The result is a list of face assignments.  An assignment is
    ``(source_face_index, flip_normal)``.  A source face may intentionally be
    present in two output groups: the missing coincident cap must be recreated.
    If the topology does not resemble a capped solid, the ordinary manifold
    groups are returned unchanged.
    """
    base_groups = _face_groups_from_edges(bm, face_indices, manifold_only=True)
    if len(base_groups) <= 1:
        if return_details:
            return base_groups, base_groups, set(), {}
        return base_groups

    _ensure_face_indices(bm)
    face_to_group = {
        face_index: group_index
        for group_index, group in enumerate(base_groups)
        for face_index in group
    }
    infos = [_face_info(bm, group) for group in base_groups]
    if any(info is None for info in infos):
        if return_details:
            return base_groups, base_groups, set(), face_to_group
        return base_groups

    # Map small planar groups to their neighboring non-planar groups through
    # non-manifold edges.  A valid box cap normally touches all four side
    # edges, which makes it much more reliable than using face count alone.
    candidates = defaultdict(lambda: defaultdict(int))
    for edge in bm.edges:
        linked_groups = {face_to_group.get(face.index) for face in edge.link_faces}
        linked_groups.discard(None)
        if len(edge.link_faces) <= 2 or len(linked_groups) < 2:
            continue
        for face in edge.link_faces:
            cap_group = face_to_group.get(face.index)
            if cap_group is None:
                continue
            for other_group in linked_groups:
                if other_group != cap_group:
                    candidates[cap_group][other_group] += 1

    assignments = [[(index, False) for index in group] for group in base_groups]
    attached_caps = set()

    for cap_group, neighbors in candidates.items():
        cap_info = infos[cap_group]
        cap_faces = base_groups[cap_group]

        # A cap may be triangulated, but it should still be a small planar
        # patch.  Larger planar groups are likely an ordinary surface region.
        if not cap_info["planar"] or len(cap_faces) > 6:
            continue

        scored = []
        for other_group, contact_count in neighbors.items():
            other_info = infos[other_group]
            if other_info["planar"] or len(base_groups[other_group]) < 3:
                continue
            signed_distance = cap_info["normal"].dot(
                cap_info["center"] - other_info["center"]
            )
            if not isfinite(signed_distance):
                continue
            scored.append((other_group, contact_count, signed_distance))

        if not scored:
            continue

        # Keep the strongest neighbor on each side of the cap plane.  In the
        # stacked-board case this picks one shell below and one above.  At a
        # T/L junction, edge-contact count rejects incidental nearby shells.
        selected = []
        for side in (1, -1):
            side_candidates = [item for item in scored if item[2] * side > 1.0e-7]
            if not side_candidates:
                continue
            side_candidates.sort(
                key=lambda item: (item[1], -abs(item[2])),
                reverse=True,
            )
            selected.append(side_candidates[0])

        if not selected:
            continue

        for other_group, _contact_count, signed_distance in selected:
            flip = signed_distance < 0.0
            for face_index in cap_faces:
                assignments[other_group].append((face_index, flip))
        attached_caps.add(cap_group)

    result = [
        group for index, group in enumerate(assignments)
        if index not in attached_caps
    ]
    result.sort(key=lambda group: min(item[0] for item in group))
    if not result:
        result = base_groups
    if return_details:
        return result, base_groups, attached_caps, face_to_group
    return result


def _canonical_axis(vector):
    axis = vector.normalized()
    for component in axis:
        if abs(component) <= 1.0e-12:
            continue
        if component < 0.0:
            axis.negate()
        break
    return axis


def _candidate_extrusion_axes(bm, face_indices):
    """Return likely extrusion axes, longest/most-repeated first."""
    wanted = {bm.faces[index] for index in face_indices}
    edges = [
        edge for edge in bm.edges
        if edge.calc_length() > 1.0e-10 and any(face in wanted for face in edge.link_faces)
    ]
    clusters = []
    for edge in edges:
        direction = _canonical_axis(edge.verts[1].co - edge.verts[0].co)
        length = edge.calc_length()
        cluster = next(
            (item for item in clusters if abs(item["axis"].dot(direction)) >= 0.9995),
            None,
        )
        if cluster is None:
            clusters.append({"axis": direction, "weighted": direction * length, "length": length})
            continue
        if cluster["axis"].dot(direction) < 0.0:
            direction.negate()
        cluster["weighted"] += direction * length
        cluster["length"] += length
        cluster["axis"] = cluster["weighted"].normalized()

    clusters.sort(key=lambda item: item["length"], reverse=True)
    return [item["axis"] for item in clusters[:8]]


def _edge_between_verts(left, right):
    return any(edge.other_vert(left) is right for edge in left.link_edges)


def _rectangle_from_cycle(cycle, axis, tolerance):
    points = [vert.co for vert in cycle]
    vectors = [points[(index + 1) % 4] - points[index] for index in range(4)]
    lengths = [vector.length for vector in vectors]
    if any(length <= tolerance for length in lengths):
        return None

    for index in range(4):
        left = vectors[index]
        right = vectors[(index + 1) % 4]
        if abs(left.dot(right)) > 1.0e-3 * lengths[index] * lengths[(index + 1) % 4]:
            return None

    length_tolerance = max(tolerance * 5.0, max(lengths) * 1.0e-3)
    if abs(lengths[0] - lengths[2]) > length_tolerance:
        return None
    if abs(lengths[1] - lengths[3]) > length_tolerance:
        return None

    normal = vectors[0].cross(vectors[1])
    if normal.length_squared <= tolerance * tolerance:
        return None
    normal.normalize()
    if abs(normal.dot(axis)) < 0.999:
        return None

    ordered = list(cycle)
    if normal.dot(axis) < 0.0:
        ordered = [cycle[0], cycle[3], cycle[2], cycle[1]]

    profile = tuple(sorted((
        (lengths[0] + lengths[2]) * 0.5,
        (lengths[1] + lengths[3]) * 0.5,
    )))
    return ordered, profile


def _four_edge_rectangles(adjacency, axis, tolerance):
    rectangles = []
    seen = set()
    for first in adjacency:
        for second in adjacency[first]:
            for third in adjacency.get(second, ()):
                if third is first:
                    continue
                for fourth in adjacency.get(third, ()):
                    if fourth is first or fourth is second:
                        continue
                    if first not in adjacency.get(fourth, ()):
                        continue
                    cycle = [first, second, third, fourth]
                    result = _rectangle_from_cycle(cycle, axis, tolerance)
                    if result is None:
                        continue
                    ordered, profile = result
                    key = frozenset(vert.index for vert in ordered)
                    if key in seen:
                        continue
                    seen.add(key)
                    rectangles.append((ordered, profile))
    return rectangles


def _profile_matches(left, right, tolerance):
    return all(
        abs(a - b) <= max(tolerance * 5.0, max(a, b) * 1.0e-3)
        for a, b in zip(left, right)
    )


def _prism_cells_for_axis(bm, face_indices, axis):
    """Read rectangular cells from both ends of one extruded manifold solid."""
    wanted_faces = {bm.faces[index] for index in face_indices}
    wanted_verts = {
        vert for face in wanted_faces for vert in face.verts
    }
    if len(wanted_verts) < 8:
        return []

    projections = {vert: axis.dot(vert.co) for vert in wanted_verts}
    t_min = min(projections.values())
    t_max = max(projections.values())
    span = t_max - t_min
    if span <= 1.0e-8:
        return []

    coordinates = [vert.co for vert in wanted_verts]
    diagonal = max((point - coordinates[0]).length for point in coordinates)
    tolerance = max(1.0e-6, diagonal * 1.0e-5)
    min_verts = {vert for vert, value in projections.items() if abs(value - t_min) <= tolerance}
    max_verts = {vert for vert, value in projections.items() if abs(value - t_max) <= tolerance}
    if len(min_verts) < 4 or len(max_verts) < 4:
        return []

    adjacency = {vert: set() for vert in min_verts}
    for edge in bm.edges:
        left, right = edge.verts
        if left not in min_verts or right not in min_verts:
            continue
        if not any(face in wanted_faces for face in edge.link_faces):
            continue
        adjacency[left].add(right)
        adjacency[right].add(left)

    rectangles = _four_edge_rectangles(adjacency, axis, tolerance)
    if len(rectangles) < 2:
        return []

    max_perpendicular = {
        vert: vert.co - axis * projections[vert]
        for vert in max_verts
    }
    cells = []
    for min_cycle, profile in rectangles:
        max_cycle = []
        used = set()
        valid = True
        for min_vert in min_cycle:
            min_perpendicular = min_vert.co - axis * projections[min_vert]
            candidates = sorted(
                (
                    ((perpendicular - min_perpendicular).length, vert)
                    for vert, perpendicular in max_perpendicular.items()
                    if vert not in used
                ),
                key=lambda item: item[0],
            )
            if not candidates or candidates[0][0] > tolerance * 5.0:
                valid = False
                break
            max_vert = candidates[0][1]
            if not _edge_between_verts(min_vert, max_vert):
                valid = False
                break
            used.add(max_vert)
            max_cycle.append(max_vert)

        if not valid:
            continue
        if any(
            not _edge_between_verts(max_cycle[index], max_cycle[(index + 1) % 4])
            for index in range(4)
        ):
            continue

        cells.append({
            "min": [vert.co.copy() for vert in min_cycle],
            "max": [vert.co.copy() for vert in max_cycle],
            "profile": profile,
        })

    if len(cells) < 2:
        return []

    # A real stack uses one repeated cross-section.  Keep the largest matching
    # profile family so unrelated end-cap subdivisions are not turned into
    # false boards.
    profile_families = []
    for cell in cells:
        family = next(
            (item for item in profile_families if _profile_matches(item[0]["profile"], cell["profile"], tolerance)),
            None,
        )
        if family is None:
            profile_families.append([cell])
        else:
            family.append(cell)
    cells = max(profile_families, key=len)
    if len(cells) < 2:
        return []

    def cell_center_key(cell):
        center = cell["min"][0].copy()
        center.zero()
        for point in cell["min"]:
            center += point
        center /= 4.0
        return tuple(round(component, 8) for component in center)

    cells.sort(key=cell_center_key)
    return cells


def _recognize_fused_prism_cells(bm, face_indices):
    """Recognize repeated rectangular prisms inside one manifold island."""
    _ensure_face_indices(bm)
    bm.verts.ensure_lookup_table()
    for index, vert in enumerate(bm.verts):
        vert.index = index

    best = []
    for axis in _candidate_extrusion_axes(bm, face_indices):
        cells = _prism_cells_for_axis(bm, face_indices, axis)
        if len(cells) > len(best):
            best = cells
    return best


def _tagged_groups_with_recovered_caps(bm, mesh, face_indices):
    """Use IDs for ownership and topology for missing coincident caps."""
    tagged = _read_part_attribute(mesh, face_indices)
    if tagged is None:
        return None

    recovered, _base_groups, attached_caps, face_to_group = _recover_welded_box_groups(
        bm,
        face_indices,
        return_details=True,
    )
    if not attached_caps:
        return tagged

    tag_by_face = {
        face_index: part_id
        for part_id, group in enumerate(tagged)
        for face_index in group
    }
    parts = defaultdict(list)

    # ``recovered`` has one output group for each non-cap base group.  Its
    # owner is determined from the original IDs on that base group; attached
    # cap faces are deliberately not allowed to vote because the surviving
    # duplicate may carry the neighboring board's ID.
    for assignment_group in recovered:
        base_face_indices = [
            item
            for item in assignment_group
            if face_to_group.get(_assignment_face_index(item)) not in attached_caps
        ]
        votes = [tag_by_face.get(index) for index in _group_face_indices(base_face_indices)]
        votes = [vote for vote in votes if vote is not None]
        if not votes:
            votes = [tag_by_face.get(index) for index in _group_face_indices(assignment_group)]
            votes = [vote for vote in votes if vote is not None]
        if not votes:
            continue

        counts = defaultdict(int)
        for vote in votes:
            counts[vote] += 1
        owner = min(counts, key=lambda value: (-counts[value], value))
        parts[owner].extend(assignment_group)

    if not parts:
        return tagged

    result = list(parts.values())
    result.sort(key=lambda group: min(item[0] for item in group))
    return result


def _read_part_attribute(mesh, face_indices):
    """Read valid face-domain part IDs, or return ``None`` when unavailable."""
    attribute = mesh.attributes.get(PART_ATTRIBUTE)
    if attribute is None:
        return None
    if attribute.domain != 'FACE' or attribute.data_type != 'INT':
        return None

    values = {}
    try:
        for index in face_indices:
            values[int(index)] = int(attribute.data[index].value)
    except (IndexError, TypeError, AttributeError):
        return None

    if not values:
        return None

    groups_by_id = defaultdict(list)
    for face_index, part_id in values.items():
        if part_id < 0:
            return None
        groups_by_id[part_id].append(face_index)

    groups = [sorted(group) for group in groups_by_id.values()]
    groups.sort(key=lambda group: group[0])
    return groups


def _write_part_attribute(mesh, groups):
    """Assign compact, deterministic IDs to the supplied face groups."""
    attribute = mesh.attributes.get(PART_ATTRIBUTE)
    if attribute is not None and (attribute.domain != 'FACE' or attribute.data_type != 'INT'):
        mesh.attributes.remove(attribute)
        attribute = None
    if attribute is None:
        attribute = mesh.attributes.new(
            name=PART_ATTRIBUTE,
            type='INT',
            domain='FACE',
        )

    for part_id, group in enumerate(groups):
        for face_index in _group_face_indices(group):
            if 0 <= face_index < len(attribute.data):
                attribute.data[face_index].value = part_id

    mesh[PART_ATTRIBUTE + "_source"] = "rcad_part_separator"


def _groups_for_strategy(bm, mesh, face_indices, strategy):
    if strategy in {'AUTO', 'TAGGED'}:
        tagged = _tagged_groups_with_recovered_caps(bm, mesh, face_indices)
        if tagged is not None:
            return tagged, "stored face IDs" if all(
                not isinstance(item, (tuple, list)) for group in tagged for item in group
            ) else "stored IDs plus recovered caps"
        if strategy == 'TAGGED':
            return [], "no valid rcad_part_id face attribute"

    if strategy == 'EDGE_CONNECTED':
        return _face_groups_from_edges(bm, face_indices, manifold_only=False), "edge-connected face islands"

    recovered = _recover_welded_box_groups(bm, face_indices)
    return recovered, "welded box shells" if len(recovered) > 1 else "manifold face shells"


def _copy_part_mesh(source_bm, source_mesh, face_indices, part_id, mesh_name):
    """Create a new mesh containing exactly one face group.

    Copying a BMesh rather than rebuilding with ``from_pydata`` keeps loop
    custom data such as UVs and face smoothing flags when Blender exposes it
    through BMesh.
    """
    keep = set(_group_face_indices(face_indices))
    flip_indices = {
        int(item[0])
        for item in face_indices
        if isinstance(item, (tuple, list)) and len(item) > 1 and bool(item[1])
    }
    _ensure_source_face_layer(source_bm)
    part_bm = source_bm.copy()
    part_layer = part_bm.faces.layers.int.get(SOURCE_FACE_LAYER)
    if part_layer is None:
        # This should not occur in supported Blender versions, but keeping a
        # local layer makes the fallback deterministic if BMesh drops custom
        # layers during a future copy implementation.
        part_layer = part_bm.faces.layers.int.new(SOURCE_FACE_LAYER)
        for face in part_bm.faces:
            face[part_layer] = face.index

    delete_faces = [face for face in part_bm.faces if int(face[part_layer]) not in keep]
    if delete_faces:
        bmesh.ops.delete(part_bm, geom=delete_faces, context='FACES')

    loose_edges = [edge for edge in part_bm.edges if not edge.link_faces]
    if loose_edges:
        bmesh.ops.delete(part_bm, geom=loose_edges, context='EDGES')
    loose_verts = [vert for vert in part_bm.verts if not vert.link_edges]
    if loose_verts:
        bmesh.ops.delete(part_bm, geom=loose_verts, context='VERTS')

    for face in part_bm.faces:
        if int(face[part_layer]) in flip_indices:
            face.normal_flip()

    part_bm.normal_update()
    try:
        part_bm.faces.layers.int.remove(part_layer)
    except Exception:
        pass

    part_mesh = bpy.data.meshes.new(mesh_name)
    part_bm.to_mesh(part_mesh)
    part_bm.free()

    for material in source_mesh.materials:
        part_mesh.materials.append(material)
    part_mesh.update()

    # The new object is tagged as well, so a later join/edit/weld cycle has a
    # provenance path even after this repair operation.
    part_attribute = part_mesh.attributes.get(PART_ATTRIBUTE)
    if part_attribute is not None and (
        part_attribute.domain != 'FACE' or part_attribute.data_type != 'INT'
    ):
        part_mesh.attributes.remove(part_attribute)
        part_attribute = None
    if part_attribute is None:
        part_attribute = part_mesh.attributes.new(
            name=PART_ATTRIBUTE,
            type='INT',
            domain='FACE',
        )
    for item in part_attribute.data:
        item.value = int(part_id)
    part_mesh[PART_ATTRIBUTE + "_source"] = "rcad_part_separator"
    return part_mesh


def _selection_for_edit_object(context, obj, selected_only=False):
    """Return the complete mesh or the edge-connected selected regions.

    Detection needs every face of a fused board assembly, even if the user
    started from one selected vertex, edge, or face.  In selection-only mode
    the selection is therefore expanded to complete edge-connected regions.
    Disconnected, unselected regions in the same object are left untouched.
    """
    bm = bmesh.from_edit_mesh(obj.data)
    _ensure_face_indices(bm)
    if not bm.faces:
        return None, "The active mesh has no faces to separate."

    if not selected_only:
        return list(range(len(bm.faces))), None

    seeds = {
        face for face in bm.faces
        if not face.hide and (
            face.select
            or any(edge.select for edge in face.edges)
            or any(vertex.select for vertex in face.verts)
        )
    }
    if not seeds:
        return None, "Select at least one mesh element to separate."

    selected_regions = set(seeds)
    queue = deque(seeds)
    while queue:
        face = queue.popleft()
        for edge in face.edges:
            for neighbor in edge.link_faces:
                if neighbor not in selected_regions:
                    selected_regions.add(neighbor)
                    queue.append(neighbor)

    return sorted(face.index for face in selected_regions), None


def _targets_from_context(context, selected_only=False):
    """Return ``[(object, face_indices)]`` and an optional error message."""
    if context.mode == 'EDIT_MESH':
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            return [], "Active edit object must be a mesh."
        face_indices, error = _selection_for_edit_object(
            context,
            obj,
            selected_only=selected_only,
        )
        if error:
            return [], error
        return [(obj, face_indices)], None

    targets = [obj for obj in context.selected_objects if obj.type == 'MESH']
    if not targets:
        return [], "Select at least one mesh object."
    return [(obj, list(range(len(obj.data.polygons)))) for obj in targets], None


def _link_like_source(context, source_obj, new_obj):
    collections = list(source_obj.users_collection)
    if not collections:
        collections = [context.collection]
    for collection in collections:
        collection.objects.link(new_obj)


def _join_part_objects(context, source_obj, part_objects):
    """Join generated objects without welding their mesh vertices."""
    for selected in context.selected_objects:
        selected.select_set(False)
    for part_obj in part_objects:
        part_obj.select_set(True)
    context.view_layer.objects.active = source_obj
    bpy.ops.object.join()


def _replace_with_parts(context, obj, groups):
    """Keep one object and make one disconnected island per face group."""
    source_mesh = obj.data
    source_bm = bmesh.new()
    source_bm.from_mesh(source_mesh)
    _ensure_face_indices(source_bm)

    if len(groups) <= 1:
        single_group = groups[0] if groups else list(range(len(source_mesh.polygons)))
        _write_part_attribute(source_mesh, [_group_face_indices(single_group)])
        source_bm.free()
        return [obj], 1, False

    base_name = obj.name
    part_objects = []
    for part_index, group in enumerate(groups):
        part_mesh = _copy_part_mesh(
            source_bm,
            source_mesh,
            group,
            part_index,
            f"{base_name}_part_{part_index + 1:03d}",
        )

        if part_index == 0:
            obj.data = part_mesh
            obj.name = base_name
            part_obj = obj
        else:
            part_obj = obj.copy()
            part_obj.data = part_mesh
            part_obj.name = f"{base_name}_part_{part_index + 1:03d}"
            _link_like_source(context, obj, part_obj)
        part_objects.append(part_obj)

    source_bm.free()
    _join_part_objects(context, obj, part_objects)
    return [obj], len(part_objects), True


def _prism_mesh(source_mesh, cell, part_id, mesh_name):
    coordinates = [tuple(point) for point in cell["min"] + cell["max"]]
    faces = [
        (3, 2, 1, 0),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    mesh = bpy.data.meshes.new(mesh_name)
    mesh.from_pydata(coordinates, [], faces)
    for material in source_mesh.materials:
        mesh.materials.append(material)
    mesh.update(calc_edges=True)

    attribute = mesh.attributes.new(
        name=PART_ATTRIBUTE,
        type='INT',
        domain='FACE',
    )
    for item in attribute.data:
        item.value = int(part_id)
    mesh[PART_ATTRIBUTE + "_source"] = "rcad_fused_prism_recognition"
    return mesh


def _replace_with_prism_cells(context, obj, cells):
    """Rebuild closed prisms as disconnected islands inside one object."""
    source_mesh = obj.data
    base_name = obj.name
    part_objects = []
    for part_index, cell in enumerate(cells):
        part_mesh = _prism_mesh(
            source_mesh,
            cell,
            part_index,
            f"{base_name}_part_{part_index + 1:03d}",
        )
        if part_index == 0:
            obj.data = part_mesh
            obj.name = base_name
            part_obj = obj
        else:
            part_obj = obj.copy()
            part_obj.data = part_mesh
            part_obj.name = f"{base_name}_part_{part_index + 1:03d}"
            _link_like_source(context, obj, part_obj)
        part_objects.append(part_obj)
    _join_part_objects(context, obj, part_objects)
    return [obj], len(part_objects), len(part_objects) > 1


def _remove_part_attribute(mesh):
    """Keep an untouched remainder from inheriting generated ownership IDs."""
    attribute = mesh.attributes.get(PART_ATTRIBUTE)
    if attribute is not None:
        mesh.attributes.remove(attribute)
    mesh.pop(PART_ATTRIBUTE + "_source", None)


def _replace_selected_regions(context, obj, selected_face_indices, part_specs):
    """Rebuild selected regions and preserve all other geometry in one object."""
    if not part_specs:
        return [obj], 0, False

    source_mesh = obj.data
    source_bm = bmesh.new()
    source_bm.from_mesh(source_mesh)
    _ensure_face_indices(source_bm)

    selected_faces = {int(index) for index in selected_face_indices}
    remainder_faces = sorted(set(range(len(source_mesh.polygons))) - selected_faces)
    base_name = obj.name
    output_objects = []

    if remainder_faces:
        remainder_mesh = _copy_part_mesh(
            source_bm,
            source_mesh,
            remainder_faces,
            -1,
            f"{base_name}_unselected",
        )
        _remove_part_attribute(remainder_mesh)
        obj.data = remainder_mesh
        output_objects.append(obj)

    for part_index, (kind, payload) in enumerate(part_specs):
        mesh_name = f"{base_name}_selected_{part_index + 1:03d}"
        if kind == 'CELL':
            part_mesh = _prism_mesh(
                source_mesh,
                payload,
                part_index,
                mesh_name,
            )
        else:
            part_mesh = _copy_part_mesh(
                source_bm,
                source_mesh,
                payload,
                part_index,
                mesh_name,
            )

        if not output_objects:
            obj.data = part_mesh
            output_objects.append(obj)
            continue

        part_obj = obj.copy()
        part_obj.data = part_mesh
        part_obj.name = mesh_name
        _link_like_source(context, obj, part_obj)
        output_objects.append(part_obj)

    source_bm.free()
    if len(output_objects) > 1:
        _join_part_objects(context, obj, output_objects)
    return [obj], len(part_specs), True


class MESH_OT_rcad_mark_part_ids(bpy.types.Operator):
    """Store a face-level identity for each currently recoverable part."""

    bl_idname = "mesh.rcad_mark_part_ids"
    bl_label = "Mark Recoverable Parts"
    bl_description = (
        "Store a face-level part ID before editing or welding. Later separation "
        "can use the ID even when vertices are shared"
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        targets, error = _targets_from_context(context)
        if error:
            self.report({'ERROR'}, error)
            return {'CANCELLED'}

        was_edit = context.mode == 'EDIT_MESH'
        if was_edit:
            bpy.ops.object.mode_set(mode='OBJECT')

        part_count = 0
        for obj, face_indices in targets:
            bm = bmesh.new()
            bm.from_mesh(obj.data)
            groups = _recover_welded_box_groups(bm, face_indices)
            bm.free()
            if not groups:
                continue
            _write_part_attribute(obj.data, groups)
            part_count += len(groups)

        if was_edit:
            context.view_layer.objects.active = targets[0][0]
            targets[0][0].select_set(True)
            bpy.ops.object.mode_set(mode='EDIT')

        self.report({'INFO'}, f"Marked {part_count} recoverable part(s) with {PART_ATTRIBUTE}.")
        return {'FINISHED'}


class MESH_OT_rcad_separate_parts(bpy.types.Operator):
    """Separate stored or topologically recoverable mesh parts."""

    bl_idname = "mesh.rcad_separate_parts"
    bl_label = "Separate 2x4 Islands"
    bl_description = (
        "Separate face-tagged parts, welded shells, and fully fused rectangular "
        "2x4 cells recognized from their end cross-section seams; Edit Mode "
        "changes only selected mesh regions"
    )
    bl_options = {'REGISTER', 'UNDO'}

    strategy: EnumProperty(
        name="Detection",
        description="How part boundaries are found",
        items=[
            ('AUTO', "Auto (IDs, then weld seams)", "Use stored IDs when available, otherwise detect manifold shells"),
            ('TAGGED', "Stored IDs only", "Use only the rcad_part_id face attribute"),
            ('MANIFOLD', "Weld seams", "Ignore vertex-only and non-manifold face connections"),
            ('EDGE_CONNECTED', "Edge-connected", "Use ordinary face islands; useful for open surface meshes"),
        ],
        default='AUTO',
    )

    def execute(self, context):
        was_edit = context.mode == 'EDIT_MESH'
        targets, error = _targets_from_context(
            context,
            selected_only=was_edit,
        )
        if error:
            self.report({'ERROR'}, error)
            return {'CANCELLED'}

        if was_edit:
            bpy.ops.object.mode_set(mode='OBJECT')

        all_new_objects = []
        total_parts = 0
        unresolved_objects = 0
        detection_labels = set()

        for obj, face_indices in targets:
            # The object may be invalid if a prior target was a linked data
            # user; skip safely rather than corrupting the remaining targets.
            if obj.type != 'MESH' or obj.data is None:
                continue
            bm = bmesh.new()
            bm.from_mesh(obj.data)
            all_face_indices = set(range(len(obj.data.polygons)))
            selected_face_indices = {int(index) for index in face_indices}

            # Object Mode and a complete Edit Mode selection use the original
            # separator path verbatim.  This keeps the proven full-object
            # behavior unchanged.
            if not was_edit or selected_face_indices == all_face_indices:
                groups, label = _groups_for_strategy(
                    bm,
                    obj.data,
                    face_indices,
                    self.strategy,
                )
                prism_cells = []
                if self.strategy in {'AUTO', 'MANIFOLD'} and len(groups) == 1:
                    prism_cells = _recognize_fused_prism_cells(bm, face_indices)
                bm.free()
                if not groups:
                    self.report({'WARNING'}, f"{obj.name}: no parts detected ({label}).")
                    continue

                if len(prism_cells) > 1:
                    parts, part_count, _did_split = _replace_with_prism_cells(
                        context,
                        obj,
                        prism_cells,
                    )
                    label = "fused rectangular-prism cells"
                else:
                    parts, part_count, _did_split = _replace_with_parts(
                        context,
                        obj,
                        groups,
                    )

                if len(groups) == 1 and len(prism_cells) <= 1 and self.strategy != 'TAGGED':
                    unresolved_objects += 1
                all_new_objects.extend(parts)
                total_parts += part_count
                detection_labels.add(label)
                continue

            # A partial Edit Mode selection is already expanded to complete
            # edge-connected regions.  Run the same detector on each selected
            # region, then rebuild only those faces and preserve the rest.
            selected_regions = _face_groups_from_edges(
                bm,
                face_indices,
                manifold_only=False,
            )
            part_specs = []
            object_labels = set()
            object_unresolved = False
            last_failure_label = "no selected regions"

            for region in selected_regions:
                groups, label = _groups_for_strategy(
                    bm,
                    obj.data,
                    region,
                    self.strategy,
                )
                last_failure_label = label
                if not groups:
                    continue

                prism_cells = []
                if self.strategy in {'AUTO', 'MANIFOLD'} and len(groups) == 1:
                    prism_cells = _recognize_fused_prism_cells(bm, region)

                if len(prism_cells) > 1:
                    part_specs.extend(('CELL', cell) for cell in prism_cells)
                    object_labels.add("selected fused rectangular-prism cells")
                else:
                    part_specs.extend(('FACES', group) for group in groups)
                    object_labels.add(f"selected {label}")
                    if len(groups) == 1 and self.strategy != 'TAGGED':
                        object_unresolved = True

            bm.free()
            if not part_specs:
                self.report(
                    {'WARNING'},
                    f"{obj.name}: no selected parts detected ({last_failure_label}).",
                )
                continue

            parts, part_count, _did_split = _replace_selected_regions(
                context,
                obj,
                face_indices,
                part_specs,
            )
            if object_unresolved:
                unresolved_objects += 1
            all_new_objects.extend(parts)
            total_parts += part_count
            detection_labels.update(object_labels)

        if not all_new_objects:
            if was_edit and context.mode == 'OBJECT':
                context.view_layer.objects.active = targets[0][0]
                targets[0][0].select_set(True)
                bpy.ops.object.mode_set(mode='EDIT')
            self.report({'ERROR'}, "No mesh parts were recovered.")
            return {'CANCELLED'}

        # Make the resulting objects easy to continue working with.
        for obj in context.selected_objects:
            obj.select_set(False)
        for obj in all_new_objects:
            obj.select_set(True)
        context.view_layer.objects.active = all_new_objects[0]

        if was_edit:
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')

        label = ", ".join(sorted(detection_labels))
        self.report(
            {'INFO'},
            f"Recovered {total_parts} island(s) in {len(all_new_objects)} object(s) using {label}.",
        )
        if unresolved_objects:
            self.report(
                {'WARNING'},
                "No recoverable boundary found on "
                f"{unresolved_objects} object(s); a seam may have been erased.",
            )
        return {'FINISHED'}


classes = (
    MESH_OT_rcad_mark_part_ids,
    MESH_OT_rcad_separate_parts,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
