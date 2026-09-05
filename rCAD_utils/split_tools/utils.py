import bmesh
from mathutils import Vector


EPS = 1e-12
PARAM_EPS = 1e-8


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def closest_point_on_segment(a: Vector, b: Vector, point: Vector):
    direction = b - a
    length_squared = direction.length_squared
    if length_squared <= EPS:
        return a.copy(), 0.0
    factor = clamp((point - a).dot(direction) / length_squared)
    return a + direction * factor, factor


def closest_points_on_segments(p0: Vector, p1: Vector, q0: Vector, q1: Vector):
    """Return closest points and parameters on two 3D line segments."""
    d1 = p1 - p0
    d2 = q1 - q0
    between = p0 - q0

    a = d1.dot(d1)
    e = d2.dot(d2)
    f = d2.dot(between)

    if a <= EPS and e <= EPS:
        return p0.copy(), q0.copy(), 0.0, 0.0

    if a <= EPS:
        s = 0.0
        t = clamp(f / e if e > EPS else 0.0)
    else:
        c = d1.dot(between)
        if e <= EPS:
            t = 0.0
            s = clamp(-c / a)
        else:
            b = d1.dot(d2)
            denominator = a * e - b * b
            if abs(denominator) > EPS * a * e:
                s = clamp((b * f - c * e) / denominator)
            else:
                s = 0.0

            t_numerator = b * s + f
            if t_numerator <= 0.0:
                t = 0.0
                s = clamp(-c / a)
            elif t_numerator >= e:
                t = 1.0
                s = clamp((b - c) / a)
            else:
                t = t_numerator / e

    point_a = p0 + d1 * s
    point_b = q0 + d2 * t
    return point_a, point_b, float(s), float(t)


def aabb_for_segment(p0: Vector, p1: Vector, padding: float):
    padding = max(0.0, float(padding))
    return (
        min(p0.x, p1.x) - padding,
        max(p0.x, p1.x) + padding,
        min(p0.y, p1.y) - padding,
        max(p0.y, p1.y) + padding,
        min(p0.z, p1.z) - padding,
        max(p0.z, p1.z) + padding,
    )


def aabb_overlaps(first, second):
    return (
        first[0] <= second[1] and first[1] >= second[0] and
        first[2] <= second[3] and first[3] >= second[2] and
        first[4] <= second[5] and first[5] >= second[4]
    )


def edge_intersection(p0: Vector, p1: Vector, q0: Vector, q1: Vector, tolerance: float):
    """Return (target parameter, world point) for a non-collinear hit."""
    direction_a = p1 - p0
    direction_b = q1 - q0
    if direction_a.length_squared <= EPS or direction_b.length_squared <= EPS:
        return None

    # Collinear/parallel overlaps are not a single T or X crossing.
    cross = direction_a.cross(direction_b)
    if cross.length_squared <= EPS * direction_a.length_squared * direction_b.length_squared:
        return None

    point_a, point_b, target_factor, _cutter_factor = closest_points_on_segments(
        p0, p1, q0, q1
    )
    tolerance = max(0.0, float(tolerance))
    if (point_a - point_b).length > max(tolerance, EPS):
        return None

    # The new vertex belongs to the selected target edge, so use the closest
    # point on that edge when the tolerance accepts a near miss.
    return float(target_factor), point_a


def edge_between(first_vert, second_vert):
    for edge in first_vert.link_edges:
        if edge.is_valid and second_vert in edge.verts:
            return edge
    return None


def split_edge_at_cuts(bm, edge, cuts):
    """Split one edge at sorted-independent (parameter, local-coordinate) cuts."""
    if not edge or not edge.is_valid or not cuts:
        return 0, []

    base_vert = edge.verts[0]
    current_edge = edge
    current_max = 1.0
    split_count = 0
    new_verts = []

    # Descending order keeps the current edge as the segment from the original
    # first endpoint to the next cut, so the parameters remain stable.
    for original_factor, local_point in sorted(cuts, key=lambda item: item[0], reverse=True):
        original_factor = float(original_factor)
        if original_factor <= PARAM_EPS or original_factor >= current_max - PARAM_EPS:
            continue
        if not current_edge or not current_edge.is_valid:
            break

        try:
            old_right_vert = current_edge.other_vert(base_vert)
        except Exception:
            break

        relative_factor = clamp(original_factor / current_max, PARAM_EPS, 1.0 - PARAM_EPS)
        try:
            result = bmesh.utils.edge_split(current_edge, base_vert, relative_factor)
        except Exception:
            break

        new_vert = next(
            (item for item in result if isinstance(item, bmesh.types.BMVert)),
            None,
        ) if isinstance(result, tuple) else None
        if not new_vert or not new_vert.is_valid:
            break

        new_vert.co = Vector(local_point)
        left_edge = edge_between(base_vert, new_vert)
        right_edge = edge_between(new_vert, old_right_vert)

        if left_edge and left_edge.is_valid:
            left_edge.select_set(True)
        if right_edge and right_edge.is_valid:
            right_edge.select_set(True)

        if not left_edge or not left_edge.is_valid:
            break

        current_edge = left_edge
        current_max = original_factor
        split_count += 1
        new_verts.append(new_vert)

    return split_count, new_verts
