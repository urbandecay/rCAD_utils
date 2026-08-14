"""Viewport-independent surface projection primitives.

This module knows nothing about Blender modes or selections.  It accepts a
world-space polygon soup, partitions it by edge connectivity, and projects a
set of points onto one component selected for the set as a whole.
"""

from dataclasses import dataclass
import math

from mathutils import Vector
from mathutils.bvhtree import BVHTree


_AREA_EPSILON = 1.0e-18
_RAY_EPSILON = 1.0e-7
_MAX_RAY_DISTANCE = 1.0e30


class ProjectionError(ValueError):
    """Raised when a complete, coherent projection cannot be produced."""


@dataclass(frozen=True)
class SurfaceComponent:
    tree: object
    face_indices: tuple
    reference_normal: Vector
    normal_weight: float


@dataclass(frozen=True)
class ProjectionResult:
    locations: tuple
    component_index: int
    component_indices: tuple
    squared_distance: float


def _is_finite_vector(value):
    return all(math.isfinite(component) for component in value)


def _polygon_has_area(vertices, polygon):
    origin = vertices[polygon[0]]
    doubled_area = 0.0
    for offset in range(1, len(polygon) - 1):
        edge_a = vertices[polygon[offset]] - origin
        edge_b = vertices[polygon[offset + 1]] - origin
        doubled_area += edge_a.cross(edge_b).length
    return doubled_area > _AREA_EPSILON


def build_surface_components(vertices, polygons):
    """Return BVHs for the non-degenerate edge-connected face components.

    Invalid individual polygons are ignored.  If no usable face remains, the
    whole surface is invalid and :class:`ProjectionError` is raised.
    """
    world_vertices = tuple(Vector(vertex).to_3d() for vertex in vertices)
    if not world_vertices:
        raise ProjectionError("The stored target has no vertices.")
    if any(not _is_finite_vector(vertex) for vertex in world_vertices):
        raise ProjectionError("The stored target contains non-finite coordinates.")

    valid_faces = []
    for original_index, raw_polygon in enumerate(polygons):
        polygon = tuple(int(index) for index in raw_polygon)
        if len(polygon) < 3 or len(set(polygon)) < 3:
            continue
        if any(index < 0 or index >= len(world_vertices) for index in polygon):
            continue
        if not _polygon_has_area(world_vertices, polygon):
            continue
        valid_faces.append((original_index, polygon))

    if not valid_faces:
        raise ProjectionError("The stored target has no non-degenerate faces.")

    edge_faces = {}
    for valid_index, (_original_index, polygon) in enumerate(valid_faces):
        for offset, first in enumerate(polygon):
            second = polygon[(offset + 1) % len(polygon)]
            edge = (first, second) if first < second else (second, first)
            edge_faces.setdefault(edge, []).append(valid_index)

    neighbours = [set() for _face in valid_faces]
    for attached_faces in edge_faces.values():
        for face_index in attached_faces:
            neighbours[face_index].update(attached_faces)

    component_face_sets = []
    unseen = set(range(len(valid_faces)))
    while unseen:
        seed = min(unseen)
        stack = [seed]
        unseen.remove(seed)
        component = []
        while stack:
            face_index = stack.pop()
            component.append(face_index)
            linked = neighbours[face_index] & unseen
            unseen.difference_update(linked)
            stack.extend(sorted(linked, reverse=True))
        component_face_sets.append(tuple(sorted(component)))

    components = []
    for face_set in component_face_sets:
        used_vertex_indices = sorted({
            vertex_index
            for face_index in face_set
            for vertex_index in valid_faces[face_index][1]
        })
        remap = {
            original_index: compact_index
            for compact_index, original_index in enumerate(used_vertex_indices)
        }
        component_vertices = [world_vertices[index] for index in used_vertex_indices]
        component_polygons = [
            [remap[index] for index in valid_faces[face_index][1]]
            for face_index in face_set
        ]
        normal_sum = Vector((0.0, 0.0, 0.0))
        normal_weight = 0.0
        for face_index in face_set:
            polygon = valid_faces[face_index][1]
            origin = world_vertices[polygon[0]]
            face_normal = Vector((0.0, 0.0, 0.0))
            for offset in range(1, len(polygon) - 1):
                face_normal += (
                    world_vertices[polygon[offset]] - origin
                ).cross(world_vertices[polygon[offset + 1]] - origin)
            normal_sum += face_normal
            normal_weight += face_normal.length
        if normal_sum.length <= _AREA_EPSILON:
            # Opposing faces can cancel in the aggregate.  A usable first-face
            # normal still gives deterministic two-sided ray projection.
            polygon = valid_faces[face_set[0]][1]
            origin = world_vertices[polygon[0]]
            normal_sum = (
                world_vertices[polygon[1]] - origin
            ).cross(world_vertices[polygon[2]] - origin)
        reference_normal = normal_sum.normalized()
        try:
            tree = BVHTree.FromPolygons(
                component_vertices,
                component_polygons,
                all_triangles=False,
                epsilon=0.0,
            )
        except (RuntimeError, ValueError) as exc:
            raise ProjectionError("The stored target surface could not be indexed.") from exc
        components.append(SurfaceComponent(
            tree=tree,
            face_indices=tuple(valid_faces[index][0] for index in face_set),
            reference_normal=reference_normal,
            normal_weight=max(normal_weight, _AREA_EPSILON),
        ))

    return tuple(components)


def _closest_ray_hit(tree, point, direction):
    closest_location = None
    closest_distance_squared = float('inf')
    for sign in (1.0, -1.0):
        ray_direction = direction * sign
        ray_origin = point + ray_direction * _RAY_EPSILON
        location, _normal, _face_index, _distance = tree.ray_cast(
            ray_origin,
            ray_direction,
            _MAX_RAY_DISTANCE,
        )
        if location is None:
            continue
        distance_squared = (location - point).length_squared
        if distance_squared < closest_distance_squared:
            closest_location = location.copy()
            closest_distance_squared = distance_squared
    return closest_location, closest_distance_squared


def project_points_coherently(points, components, direction):
    """Project every point along one shared direction onto one target component.

    The direction is deliberately supplied by the caller from target geometry;
    it must never come from the viewport.  A target component is scored by the
    sum of squared ray travel distances for the complete source selection.
    This preserves the source's form on a planar target and prevents vertices
    from scattering across unrelated target islands.
    """
    source_points = tuple(Vector(point).to_3d() for point in points)
    if not source_points:
        raise ProjectionError("No source vertices were supplied.")
    if any(not _is_finite_vector(point) for point in source_points):
        raise ProjectionError("The stored source contains non-finite coordinates.")
    if not components:
        raise ProjectionError("The stored target has no usable surface components.")
    projection_direction = Vector(direction).to_3d()
    if not _is_finite_vector(projection_direction) or projection_direction.length <= _AREA_EPSILON:
        raise ProjectionError("The target does not provide a valid projection direction.")
    projection_direction.normalize()

    best = None
    for component_index, component in enumerate(components):
        locations = []
        squared_distance = 0.0
        complete = True
        for point in source_points:
            location, distance_squared = _closest_ray_hit(
                component.tree,
                point,
                projection_direction,
            )
            if location is None or not math.isfinite(distance_squared):
                complete = False
                break
            location = location.copy()
            if not _is_finite_vector(location):
                complete = False
                break
            locations.append(location)
            squared_distance += distance_squared

        if not complete:
            continue

        score = (squared_distance, component_index)
        if best is None or score < best[0]:
            best = (
                score,
                ProjectionResult(
                    locations=tuple(locations),
                    component_index=component_index,
                    component_indices=tuple(component_index for _point in source_points),
                    squared_distance=squared_distance,
                ),
            )

    if best is None:
        locations = []
        component_indices = []
        squared_distance = 0.0
        for point in source_points:
            closest = None
            for component_index, component in enumerate(components):
                location, distance_squared = _closest_ray_hit(
                    component.tree,
                    point,
                    projection_direction,
                )
                if location is None or not math.isfinite(distance_squared):
                    continue
                candidate = (distance_squared, component_index, location)
                if closest is None or candidate[:2] < closest[:2]:
                    closest = candidate
            if closest is None:
                raise ProjectionError(
                    "The shared projection direction did not intersect the target for every source vertex."
                )
            distance_squared, component_index, location = closest
            squared_distance += distance_squared
            locations.append(location)
            component_indices.append(component_index)
        return ProjectionResult(
            locations=tuple(locations),
            component_index=(
                component_indices[0]
                if len(set(component_indices)) == 1
                else -1
            ),
            component_indices=tuple(component_indices),
            squared_distance=squared_distance,
        )
    return best[1]
