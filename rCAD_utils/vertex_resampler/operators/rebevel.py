"""Rebuild an existing all-edge bevel with a new segment count.

This intentionally starts with the topology Blender's bevel operator produces
for a closed manifold whose edges were beveled together. Recovering that base
solid lets Blender rebuild both the edge profiles and its parity-sensitive
corner VMesh instead of trying to patch the corner grid a row at a time.
"""

import bmesh
from mathutils import Matrix, Vector, geometry


_EPSILON = 1.0e-7
_SUPPORT_AREA_RATIO = 1.5


class ReBevelError(RuntimeError):
    pass


def _report(report, level, message):
    if report is not None:
        report(level, message)


def _face_neighbors(face):
    neighbors = set()
    for edge in face.edges:
        for other in edge.link_faces:
            if other is not face:
                neighbors.add(other)
    return neighbors


def _selected_seed_faces(bm):
    # Users naturally select the visible edge/vertex network of the corner,
    # rather than complete faces. Treat every face touched by that selection as
    # a seed so either vertex-select or edge-select mode works.
    return {
        face
        for face in bm.faces
        if face.select
        or any(edge.select for edge in face.edges)
        or any(vert.select for vert in face.verts)
    }


def _face_component(seed_face):
    component = set()
    stack = [seed_face]
    while stack:
        face = stack.pop()
        if face in component:
            continue
        component.add(face)
        stack.extend(
            neighbor
            for neighbor in _face_neighbors(face)
            if neighbor not in component
        )
    return component


def _other_face(edge, face):
    others = [candidate for candidate in edge.link_faces if candidate is not face]
    if len(others) != 1:
        raise ReBevelError("Re-Bevel needs a closed manifold surface.")
    return others[0]


def _support_faces(component):
    supports = set()
    for face in component:
        neighbors = _face_neighbors(face)
        if not neighbors:
            continue
        largest_neighbor = max(neighbor.calc_area() for neighbor in neighbors)
        if face.calc_area() >= largest_neighbor * _SUPPORT_AREA_RATIO:
            supports.add(face)

    if len(supports) < 4:
        raise ReBevelError(
            "Could not identify the original flat faces. Select the corner patch "
            "and its connecting bevel strips, as shown in the example."
        )
    return supports


def _opposite_quad_edge(face, edge):
    loops = list(face.loops)
    if len(loops) != 4:
        raise ReBevelError(
            "The bevel edge strips must be made from Blender-style quads."
        )
    for index, loop in enumerate(loops):
        if loop.edge is edge:
            return loops[(index + 2) % 4].edge
    raise ReBevelError("The bevel strip topology is inconsistent.")


def _trace_strip(start_support, start_edge, supports, component):
    current_edge = start_edge
    current_face = _other_face(current_edge, start_support)
    if current_face in supports:
        raise ReBevelError(
            "The connected surface contains an unbeveled edge; this first version "
            "supports closed solids whose edges were all beveled together."
        )

    faces = []
    rails = [start_edge]
    visited = set()
    while current_face not in supports:
        if current_face not in component or current_face in visited:
            raise ReBevelError("A bevel strip did not terminate on a flat support face.")
        visited.add(current_face)
        faces.append(current_face)
        current_edge = _opposite_quad_edge(current_face, current_edge)
        rails.append(current_edge)
        current_face = _other_face(current_edge, current_face)

    return {
        'support_a': start_support,
        'support_b': current_face,
        'faces': tuple(faces),
        'rails': tuple(rails),
    }


def _build_strips(supports, component):
    strips_by_key = {}
    strip_by_boundary = {}

    for support in sorted(supports, key=lambda face: face.index):
        for edge in support.edges:
            traced = _trace_strip(support, edge, supports, component)
            key = frozenset(traced['faces'])
            if not key:
                raise ReBevelError("Found an empty bevel strip.")
            strip = strips_by_key.get(key)
            if strip is None:
                strip = traced
                strip['index'] = len(strips_by_key)
                strips_by_key[key] = strip
            strip_by_boundary[(support, edge)] = strip

    strips = list(strips_by_key.values())
    if len(strips) < 6:
        raise ReBevelError("Too few connected bevel strips were found.")
    return strips, strip_by_boundary


def _face_components(faces):
    face_set = set(faces)
    components = []
    visited = set()
    for seed in sorted(face_set, key=lambda face: face.index):
        if seed in visited:
            continue
        component = set()
        stack = [seed]
        visited.add(seed)
        while stack:
            face = stack.pop()
            component.add(face)
            for neighbor in _face_neighbors(face):
                if neighbor in face_set and neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components


def _build_corners(component, supports, strips):
    strip_faces = {
        face
        for strip in strips
        for face in strip['faces']
    }
    leftovers = component - supports - strip_faces
    if not leftovers:
        raise ReBevelError("No bevel corner patches were found.")

    corner_components = _face_components(leftovers)
    corner_by_face = {
        face: corner_index
        for corner_index, faces in enumerate(corner_components)
        for face in faces
    }

    for strip in strips:
        endpoint_corners = set()
        for index, face in enumerate(strip['faces']):
            rail_edges = {strip['rails'][index], strip['rails'][index + 1]}
            for edge in face.edges:
                if edge in rail_edges:
                    continue
                neighbor = _other_face(edge, face)
                corner_index = corner_by_face.get(neighbor)
                if corner_index is None:
                    raise ReBevelError(
                        "A bevel strip endpoint is not attached to a corner patch."
                    )
                endpoint_corners.add(corner_index)
        if len(endpoint_corners) != 2:
            raise ReBevelError(
                "Each bevel strip must connect exactly two corner patches."
            )
        strip['corners'] = tuple(sorted(endpoint_corners))

    incident_strips = {index: set() for index in range(len(corner_components))}
    for strip in strips:
        for corner_index in strip['corners']:
            incident_strips[corner_index].add(strip['index'])

    if any(len(indices) < 3 for indices in incident_strips.values()):
        raise ReBevelError(
            "A detected corner has fewer than three beveled edges."
        )

    return corner_components, corner_by_face, incident_strips


def _corner_supports(corner_index, strips, supports):
    result = set()
    for strip in strips:
        if corner_index in strip['corners']:
            result.add(strip['support_a'])
            result.add(strip['support_b'])
    return result.intersection(supports)


def _intersect_support_planes(faces):
    ata = Matrix(((0.0, 0.0, 0.0),) * 3)
    atb = Vector((0.0, 0.0, 0.0))
    for face in faces:
        normal = face.normal.normalized()
        distance = normal.dot(face.verts[0].co)
        for row in range(3):
            atb[row] += normal[row] * distance
            for column in range(3):
                ata[row][column] += normal[row] * normal[column]

    if abs(ata.determinant()) <= _EPSILON:
        raise ReBevelError("The original corner planes do not have a unique intersection.")
    return ata.inverted() @ atb


def _base_corner_coordinates(corner_components, strips, supports):
    coordinates = []
    for corner_index in range(len(corner_components)):
        incident_supports = _corner_supports(corner_index, strips, supports)
        if len(incident_supports) < 3:
            raise ReBevelError(
                "A bevel corner is not bounded by at least three flat faces."
            )
        coordinates.append(_intersect_support_planes(incident_supports))
    return coordinates


def _ordered_face_corners(support, strip_by_boundary):
    ordered = []
    for loop in support.loops:
        current_strip = strip_by_boundary.get((support, loop.edge))
        previous_strip = strip_by_boundary.get((support, loop.link_loop_prev.edge))
        if current_strip is None or previous_strip is None:
            raise ReBevelError("A flat face boundary is missing a bevel strip.")
        common = set(current_strip['corners']).intersection(previous_strip['corners'])
        if len(common) != 1:
            raise ReBevelError("Could not recover the original face corner order.")
        ordered.append(common.pop())

    if len(ordered) < 3 or len(set(ordered)) != len(ordered):
        raise ReBevelError("A recovered original face is degenerate.")
    return ordered


def _point_line_distance(point, line_a, line_b):
    direction = line_b - line_a
    length = direction.length
    if length <= _EPSILON:
        raise ReBevelError("A recovered original edge has zero length.")
    return (point - line_a).cross(direction).length / length


def _infer_offset(strips, corner_coords):
    offsets = []
    for strip in strips:
        corner_a, corner_b = strip['corners']
        line_a = corner_coords[corner_a]
        line_b = corner_coords[corner_b]
        for rail in (strip['rails'][0], strip['rails'][-1]):
            midpoint = (rail.verts[0].co + rail.verts[1].co) * 0.5
            offsets.append(_point_line_distance(midpoint, line_a, line_b))

    offset = sum(offsets) / len(offsets)
    if offset <= _EPSILON:
        raise ReBevelError("The existing bevel width is too small to recover.")

    tolerance = max(offset * 0.08, 1.0e-5)
    if any(abs(value - offset) > tolerance for value in offsets):
        raise ReBevelError(
            "The connected bevel does not have a uniform offset. Weighted or uneven "
            "bevels are not supported yet."
        )
    return offset


def _solve_superellipse_r(x_value, y_value):
    x_value = min(max(x_value, 1.0e-6), 1.0 - 1.0e-6)
    y_value = min(max(y_value, 1.0e-6), 1.0 - 1.0e-6)
    low = 0.02
    high = 1000.0
    for _iteration in range(64):
        middle = (low + high) * 0.5
        value = (x_value ** middle) + (y_value ** middle)
        if value > 1.0:
            low = middle
        else:
            high = middle
    return (low + high) * 0.5


def _infer_profile(strips, corner_coords):
    samples = []
    for strip in strips:
        if len(strip['faces']) <= 1:
            continue
        corner_a, corner_b = strip['corners']
        line_a = corner_coords[corner_a]
        line_b = corner_coords[corner_b]
        start = sum((vert.co for vert in strip['rails'][0].verts), Vector()) * 0.5
        end = sum((vert.co for vert in strip['rails'][-1].verts), Vector()) * 0.5
        intersections = geometry.intersect_line_line(line_a, line_b, start, end)
        if intersections is None:
            continue
        middle = intersections[0]
        start_axis = start - middle
        end_axis = end - middle
        aa = start_axis.dot(start_axis)
        bb = end_axis.dot(end_axis)
        ab = start_axis.dot(end_axis)
        determinant = (aa * bb) - (ab * ab)
        if determinant <= _EPSILON:
            continue

        for rail in strip['rails'][1:-1]:
            point = sum((vert.co for vert in rail.verts), Vector()) * 0.5
            delta = point - middle
            da = delta.dot(start_axis)
            db = delta.dot(end_axis)
            start_factor = ((da * bb) - (db * ab)) / determinant
            end_factor = ((db * aa) - (da * ab)) / determinant
            # Blender's unit-square profile map places the original hard edge
            # at (1, 1). The coefficients measured from that point are the
            # complements of the superellipse coordinates.
            x_value = 1.0 - start_factor
            y_value = 1.0 - end_factor
            if not (0.0 < x_value < 1.0 and 0.0 < y_value < 1.0):
                continue
            exponent = _solve_superellipse_r(x_value, y_value)
            profile = 2.0 ** (-2.0 / exponent)
            samples.append(profile)

    if not samples:
        return 0.5
    samples.sort()
    profile = samples[len(samples) // 2]
    if abs(profile - 0.5) <= 0.03:
        return 0.5
    return min(max(profile, 0.01), 0.99)


def _validate_selection(selected_faces, strips, corner_components):
    strip_faces = {face for strip in strips for face in strip['faces']}
    selected_strip_count = sum(
        1 for strip in strips if selected_faces.intersection(strip['faces'])
    )
    selected_corner_count = sum(
        1 for corner in corner_components if selected_faces.intersection(corner)
    )
    if not selected_faces.intersection(strip_faces) or selected_strip_count < 2:
        raise ReBevelError(
            "Select the connected bevel strips meeting at a corner."
        )
    if selected_corner_count == 0:
        raise ReBevelError("The selection must include a bevel corner patch.")

    def corner_score(corner_faces):
        corner_edges = {edge for face in corner_faces for edge in face.edges}
        corner_verts = {vert for face in corner_faces for vert in face.verts}
        return (
            sum(face.select for face in corner_faces) * 4
            + sum(edge.select for edge in corner_edges) * 2
            + sum(vert.select for vert in corner_verts)
        )

    return max(
        range(len(corner_components)),
        key=lambda index: (corner_score(corner_components[index]), -index),
    )


def _analyze(bm):
    bm.normal_update()
    selected_faces = _selected_seed_faces(bm)
    if not selected_faces:
        raise ReBevelError(
            "Select the bevel corner patch and its connecting strips first."
        )

    component = _face_component(min(selected_faces, key=lambda face: face.index))
    if not selected_faces.issubset(component):
        raise ReBevelError("Re-Bevel handles one connected beveled solid at a time.")
    if any(len(edge.link_faces) != 2 for face in component for edge in face.edges):
        raise ReBevelError("Re-Bevel needs a closed manifold solid.")

    supports = _support_faces(component)
    strips, strip_by_boundary = _build_strips(supports, component)
    corner_components, _corner_by_face, _incident = _build_corners(
        component,
        supports,
        strips,
    )
    selected_corner_index = _validate_selection(
        selected_faces,
        strips,
        corner_components,
    )

    segment_counts = {len(strip['faces']) for strip in strips}
    if len(segment_counts) != 1:
        raise ReBevelError(
            "All connected bevel edges must currently use the same segment count."
        )
    segment_count = segment_counts.pop()

    corner_coords = _base_corner_coordinates(corner_components, strips, supports)
    support_corner_orders = {
        support: _ordered_face_corners(support, strip_by_boundary)
        for support in supports
    }
    offset = _infer_offset(strips, corner_coords)
    profile = _infer_profile(strips, corner_coords)

    return {
        'component': component,
        'supports': supports,
        'strips': strips,
        'corner_coords': corner_coords,
        'support_corner_orders': support_corner_orders,
        'segment_count': segment_count,
        'offset': offset,
        'profile': profile,
        'selected_corner_co': corner_coords[selected_corner_index].copy(),
    }


def _probe_rebuild(data, target_segments):
    probe = bmesh.new()
    try:
        verts = [probe.verts.new(co) for co in data['corner_coords']]
        for order in data['support_corner_orders'].values():
            probe.faces.new([verts[index] for index in order])

        # A mesh can be closed/manifold while one or more faces are wound in
        # the opposite direction. Passing that hard-edge cage to bevel makes
        # those edges offset to opposite sides, producing crossing corner
        # polygons and coplanar faces (the characteristic viewport moire).
        # Recalculate the recovered cage as one closed shell before beveling.
        bmesh.ops.recalc_face_normals(probe, faces=list(probe.faces))
        probe.normal_update()
        edges = list(probe.edges)
        if len(edges) != len(data['strips']):
            raise ReBevelError("The recovered hard-edge graph is incomplete.")
        if target_segments == 0:
            if any(len(edge.link_faces) != 2 for edge in edges):
                raise ReBevelError(
                    "The recovered sharp-corner solid is not closed and manifold."
                )
            if any(face.calc_area() <= _EPSILON for face in probe.faces):
                raise ReBevelError(
                    "The recovered sharp-corner solid contains a collapsed face."
                )
            return
        result = bmesh.ops.bevel(
            probe,
            geom=edges,
            offset=data['offset'],
            segments=target_segments,
            profile=data['profile'],
            affect='EDGES',
            clamp_overlap=False,
            loop_slide=True,
            material=-1,
            miter_outer='SHARP',
            miter_inner='SHARP',
            vmesh_method='ADJ',
        )
        result_faces = [face for face in result.get('faces', []) if face.is_valid]
        if not result_faces:
            raise ReBevelError("Blender's bevel probe did not return any faces.")
        for vert in probe.verts:
            vert.select = True
        rebuilt = _analyze(probe)
        if rebuilt['segment_count'] != target_segments:
            raise ReBevelError("The rebuilt bevel did not pass its topology check.")
    except ReBevelError:
        raise
    except Exception as exc:
        raise ReBevelError(f"Blender could not rebuild this bevel: {exc}") from exc
    finally:
        probe.free()


def _rebuild(bm, data, target_segments):
    corner_coords = [coordinate.copy() for coordinate in data['corner_coords']]
    edge_specs = {
        frozenset(strip['corners']): (
            strip['rails'][0].seam,
            strip['rails'][0].smooth,
        )
        for strip in data['strips']
    }
    component_faces = list(data['component'])
    component_verts = {
        vert
        for face in component_faces
        for vert in face.verts
    }

    corner_verts = []
    base_faces = []
    try:
        corner_verts = [bm.verts.new(co) for co in corner_coords]
        for support, order in data['support_corner_orders'].items():
            face = bm.faces.new([corner_verts[index] for index in order])
            face.material_index = support.material_index
            face.smooth = support.smooth
            face.copy_from_face_interp(support, True)
            base_faces.append(face)
    except Exception as exc:
        staged_verts = [vert for vert in corner_verts if vert.is_valid]
        if staged_verts:
            bmesh.ops.delete(bm, geom=staged_verts, context='VERTS')
        raise ReBevelError(
            f"Could not stage the recovered support faces: {exc}"
        ) from exc

    # Normalize local winding for the same reason as the probe. This operates
    # only on the newly staged closed cage; the analyzed source shell is left
    # untouched until the replacement is known to be constructible.
    bmesh.ops.recalc_face_normals(bm, faces=base_faces)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    base_edges = list({edge for face in base_faces for edge in face.edges})
    if len(base_edges) != len(data['strips']):
        bmesh.ops.delete(bm, geom=corner_verts, context='VERTS')
        raise ReBevelError("The recovered hard-edge graph is incomplete.")

    corner_index = {vert: index for index, vert in enumerate(corner_verts)}
    for edge in base_edges:
        key = frozenset(corner_index[vert] for vert in edge.verts)
        seam, smooth = edge_specs.get(key, (False, True))
        edge.seam = seam
        edge.smooth = smooth

    # Remove only the analyzed shell. Building and interpolating the recovered
    # support faces first lets Blender carry UV and other loop data onto them.
    bmesh.ops.delete(bm, geom=component_faces, context='FACES_ONLY')
    orphaned_component_verts = [
        vert
        for vert in component_verts
        if vert.is_valid and not vert.link_faces
    ]
    if orphaned_component_verts:
        bmesh.ops.delete(bm, geom=orphaned_component_verts, context='VERTS')

    if target_segments == 0:
        for vert in bm.verts:
            vert.select = False
        for edge in bm.edges:
            edge.select = False
        for face in bm.faces:
            face.select = False

        selected_corner_index = min(
            range(len(corner_verts)),
            key=lambda index: (
                corner_verts[index].co - data['selected_corner_co']
            ).length_squared,
        )
        selected_corner = corner_verts[selected_corner_index]
        selected_corner.select = True
        selected_edges = [
            edge for edge in base_edges
            if selected_corner in edge.verts
        ]
        for edge in selected_edges:
            edge.select = True
        if 'FACE' in bm.select_mode:
            for face in base_faces:
                if selected_corner in face.verts:
                    face.select = True
        bm.select_flush_mode()
        bm.normal_update()
        return

    result = bmesh.ops.bevel(
        bm,
        geom=base_edges,
        offset=data['offset'],
        segments=target_segments,
        profile=data['profile'],
        affect='EDGES',
        clamp_overlap=False,
        loop_slide=True,
        material=-1,
        miter_outer='SHARP',
        miter_inner='SHARP',
        vmesh_method='ADJ',
    )

    for vert in bm.verts:
        vert.select = False
    for edge in bm.edges:
        edge.select = False
    for face in bm.faces:
        face.select = False

    operator_faces = [face for face in result.get('faces', []) if face.is_valid]
    if not operator_faces:
        raise ReBevelError("Blender rebuilt the bevel without returning any faces.")
    rebuilt_component = _face_component(operator_faces[0])
    rebuilt_supports = _support_faces(rebuilt_component)
    rebuilt_strips, _strip_by_boundary = _build_strips(
        rebuilt_supports,
        rebuilt_component,
    )
    rebuilt_corners, _corner_by_face, _incident = _build_corners(
        rebuilt_component,
        rebuilt_supports,
        rebuilt_strips,
    )
    rebuilt_corner_coords = _base_corner_coordinates(
        rebuilt_corners,
        rebuilt_strips,
        rebuilt_supports,
    )
    selected_corner_index = min(
        range(len(rebuilt_corners)),
        key=lambda index: (
            rebuilt_corner_coords[index] - data['selected_corner_co']
        ).length_squared,
    )
    selected_strips = [
        strip
        for strip in rebuilt_strips
        if selected_corner_index in strip['corners']
    ]
    selected_faces = set(rebuilt_corners[selected_corner_index])
    for strip in selected_strips:
        selected_faces.update(strip['faces'])
    selected_edges = {edge for face in selected_faces for edge in face.edges}
    selected_verts = {vert for edge in selected_edges for vert in edge.verts}

    for vert in selected_verts:
        vert.select = True
    for edge in selected_edges:
        edge.select = True
    if 'FACE' in bm.select_mode:
        for face in selected_faces:
            face.select = True
    bm.select_flush_mode()

    bm.normal_update()


def execute(bm, obj, direction, report=None):
    try:
        data = _analyze(bm)
        target_segments = data['segment_count'] + direction
        if target_segments < 0:
            _report(report, {'WARNING'}, "Can't remove any more bevel segments.")
            return {'CANCELLED'}
        if direction == 0:
            return {'CANCELLED'}

        _probe_rebuild(data, target_segments)
        _rebuild(bm, data, target_segments)
        # The rebuild replaces the face topology, including the corner VMesh
        # and support n-gons. Recalculate Blender's edit-mesh tessellation now;
        # otherwise the viewport can keep drawing stale loop triangles until a
        # later face edit (such as Flip or Delete) happens to refresh the cache.
        bmesh.update_edit_mesh(obj.data, loop_triangles=True, destructive=True)
        if target_segments == 0:
            message = (
                "Re-Bevel finished: chamfer removed and sharp corners restored "
                f"(segments {data['segment_count']} -> 0)."
            )
        else:
            message = (
                "Re-Bevel finished: "
                f"segments {data['segment_count']} -> {target_segments}, "
                f"offset {data['offset']:.4g}, profile {data['profile']:.3f}."
            )
        _report(report, {'INFO'}, message)
        return {'FINISHED'}
    except ReBevelError as exc:
        _report(report, {'WARNING'}, str(exc))
        return {'CANCELLED'}
