"""Rephase selected faceted loops onto selected common tangent lines.

Tangify keeps the selected topology intact.  Each selected open edge chain is
treated as a tangent guide between two selected closed loops.  The guide's
nearby common tangent is solved on the same Catmull-Rom interpolation used by
Kissing mode, then the existing loop vertices are redistributed with the
tangent points as mandatory anchors.
"""

import math

import bmesh

from ..math_engine import CatmullRomSpline
from .detection_utils import get_selected_islands
from .kissing import _closed_target_coords, _deduplicate_anchors


_LENGTH_EPSILON = 1.0e-10
_STRAIGHT_RELATIVE_TOLERANCE = 0.02
_COARSE_STEPS = 16
_REFINE_STEPS = 48


def _report(report, level, message):
    if report is not None:
        report(level, message)


def _is_straight_open_chain(island):
    if island['closed'] or len(island['verts']) < 2:
        return False

    verts = island['verts']
    start = verts[0].co
    chord = verts[-1].co - start
    chord_length = chord.length
    if chord_length <= _LENGTH_EPSILON:
        return False

    direction = chord / chord_length
    tolerance = max(1.0e-6, chord_length * _STRAIGHT_RELATIVE_TOLERANCE)
    for vert in verts[1:-1]:
        offset = vert.co - start
        perpendicular = offset - direction * offset.dot(direction)
        if perpendicular.length > tolerance:
            return False
    return True


def _parameter_distance(spline, first, second):
    distance = abs(first - second)
    if spline.is_closed:
        period = float(len(spline.segments))
        distance %= period
        return min(distance, period - distance)
    return distance


def _clamp_local_parameter(spline, value, seed, window):
    if spline.is_closed:
        period = float(len(spline.segments))
        delta = (value - seed + period * 0.5) % period - period * 0.5
        value = seed + delta
    value = max(seed - window, min(seed + window, value))
    if spline.is_closed:
        return value % len(spline.segments)
    return max(0.0, min(float(len(spline.segments)), value))


def _tangent_alignment_error(spline_a, spline_b, t_a, t_b):
    point_a = spline_a.eval_global(t_a)
    point_b = spline_b.eval_global(t_b)
    chord = point_b - point_a
    if chord.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON:
        return float('inf'), point_a, point_b

    direction = chord.normalized()
    tangent_a = spline_a.tangent_global(t_a)
    tangent_b = spline_b.tangent_global(t_b)
    if (
        tangent_a.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON
        or tangent_b.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON
    ):
        return float('inf'), point_a, point_b

    dot_a = max(-1.0, min(1.0, tangent_a.dot(direction)))
    dot_b = max(-1.0, min(1.0, tangent_b.dot(direction)))
    error = (1.0 - dot_a * dot_a) + (1.0 - dot_b * dot_b)
    return error, point_a, point_b


def _guided_score(
    spline_a,
    spline_b,
    t_a,
    t_b,
    seed_a,
    seed_b,
    window_a,
    window_b,
    guide_direction,
):
    error, point_a, point_b = _tangent_alignment_error(
        spline_a,
        spline_b,
        t_a,
        t_b,
    )
    if not math.isfinite(error):
        return error

    chord = (point_b - point_a).normalized()
    direction_dot = max(-1.0, min(1.0, chord.dot(guide_direction)))
    direction_error = 1.0 - direction_dot
    drift_a = _parameter_distance(spline_a, t_a, seed_a) / window_a
    drift_b = _parameter_distance(spline_b, t_b, seed_b) / window_b
    drift_error = drift_a * drift_a + drift_b * drift_b

    # The line and endpoint projections pick the intended tangent branch.  The
    # alignment term remains dominant, so a slightly approximate input guide
    # still converges to a true common tangent.
    return error + 0.02 * direction_error + 0.002 * drift_error


def _common_tangent(spline_a, spline_b, seed_a, seed_b, guide_direction):
    count_a = float(len(spline_a.segments))
    count_b = float(len(spline_b.segments))
    window_a = min(count_a * 0.25, max(1.5, count_a * 0.10))
    window_b = min(count_b * 0.25, max(1.5, count_b * 0.10))

    best_t_a = seed_a
    best_t_b = seed_b
    best_score = float('inf')
    for index_a in range(_COARSE_STEPS + 1):
        raw_a = seed_a - window_a + 2.0 * window_a * index_a / _COARSE_STEPS
        t_a = _clamp_local_parameter(spline_a, raw_a, seed_a, window_a)
        for index_b in range(_COARSE_STEPS + 1):
            raw_b = seed_b - window_b + 2.0 * window_b * index_b / _COARSE_STEPS
            t_b = _clamp_local_parameter(spline_b, raw_b, seed_b, window_b)
            score = _guided_score(
                spline_a,
                spline_b,
                t_a,
                t_b,
                seed_a,
                seed_b,
                window_a,
                window_b,
                guide_direction,
            )
            if score < best_score:
                best_score = score
                best_t_a = t_a
                best_t_b = t_b

    step_a = 2.0 * window_a / _COARSE_STEPS
    step_b = 2.0 * window_b / _COARSE_STEPS
    for _iteration in range(_REFINE_STEPS):
        candidate_t_a = best_t_a
        candidate_t_b = best_t_b
        candidate_score = best_score
        for offset_a in (-step_a, 0.0, step_a):
            t_a = _clamp_local_parameter(
                spline_a,
                best_t_a + offset_a,
                seed_a,
                window_a,
            )
            for offset_b in (-step_b, 0.0, step_b):
                t_b = _clamp_local_parameter(
                    spline_b,
                    best_t_b + offset_b,
                    seed_b,
                    window_b,
                )
                score = _guided_score(
                    spline_a,
                    spline_b,
                    t_a,
                    t_b,
                    seed_a,
                    seed_b,
                    window_a,
                    window_b,
                    guide_direction,
                )
                if score < candidate_score:
                    candidate_score = score
                    candidate_t_a = t_a
                    candidate_t_b = t_b

        moved = (
            _parameter_distance(spline_a, candidate_t_a, best_t_a) > 1.0e-12
            or _parameter_distance(spline_b, candidate_t_b, best_t_b) > 1.0e-12
        )
        best_t_a = candidate_t_a
        best_t_b = candidate_t_b
        best_score = candidate_score
        if not moved:
            step_a *= 0.5
            step_b *= 0.5
            if max(step_a, step_b) <= 1.0e-8:
                break

    # Remove the small branch-selection penalties for the final convergence.
    pure_score, _point_a, _point_b = _tangent_alignment_error(
        spline_a,
        spline_b,
        best_t_a,
        best_t_b,
    )
    for _iteration in range(_REFINE_STEPS):
        candidate_t_a = best_t_a
        candidate_t_b = best_t_b
        candidate_score = pure_score
        for offset_a in (-step_a, 0.0, step_a):
            t_a = _clamp_local_parameter(
                spline_a,
                best_t_a + offset_a,
                seed_a,
                window_a,
            )
            for offset_b in (-step_b, 0.0, step_b):
                t_b = _clamp_local_parameter(
                    spline_b,
                    best_t_b + offset_b,
                    seed_b,
                    window_b,
                )
                score, _point_a, _point_b = _tangent_alignment_error(
                    spline_a,
                    spline_b,
                    t_a,
                    t_b,
                )
                if score < candidate_score:
                    candidate_score = score
                    candidate_t_a = t_a
                    candidate_t_b = t_b

        moved = (
            _parameter_distance(spline_a, candidate_t_a, best_t_a) > 1.0e-12
            or _parameter_distance(spline_b, candidate_t_b, best_t_b) > 1.0e-12
        )
        best_t_a = candidate_t_a
        best_t_b = candidate_t_b
        pure_score = candidate_score
        if not moved:
            step_a *= 0.5
            step_b *= 0.5
            if max(step_a, step_b) <= 1.0e-10:
                break

    error, point_a, point_b = _tangent_alignment_error(
        spline_a,
        spline_b,
        best_t_a,
        best_t_b,
    )
    if not math.isfinite(error):
        return None
    return {
        't_a': best_t_a,
        't_b': best_t_b,
        'point_a': point_a,
        'point_b': point_b,
        'error': error,
    }


def _build_curves(islands):
    curves = []
    for island in islands:
        if not island['closed'] or len(island['verts']) < 3:
            continue
        spline = CatmullRomSpline(
            [vert.co.copy() for vert in island['verts']],
            is_closed=True,
        )
        if spline.segments:
            curves.append({
                'island': island,
                'spline': spline,
                'anchors': [],
            })
    return curves


def _match_guide(guide, curves):
    start = guide['verts'][0].co
    end = guide['verts'][-1].co
    best = None
    for index_a, curve_a in enumerate(curves):
        t_a, _point_a, distance_a = curve_a['spline'].project(start)
        for index_b, curve_b in enumerate(curves):
            if index_a == index_b:
                continue
            t_b, _point_b, distance_b = curve_b['spline'].project(end)
            score = distance_a + distance_b
            if best is None or score < best['score']:
                best = {
                    'index_a': index_a,
                    'index_b': index_b,
                    't_a': t_a,
                    't_b': t_b,
                    'distance_a': distance_a,
                    'distance_b': distance_b,
                    'score': score,
                }
    return best


def _guide_target_coords(guide, start, end):
    verts = guide['verts']
    if len(verts) == 2:
        return [start.copy(), end.copy()]

    lengths = [0.0]
    for first, second in zip(verts, verts[1:]):
        lengths.append(lengths[-1] + (second.co - first.co).length)
    total_length = lengths[-1]
    if total_length <= _LENGTH_EPSILON:
        return [
            start.lerp(end, index / (len(verts) - 1))
            for index in range(len(verts))
        ]
    return [start.lerp(end, length / total_length) for length in lengths]


def execute(bm, obj, report=None):
    islands = get_selected_islands(bm)
    curves = _build_curves(islands)
    guides = [island for island in islands if _is_straight_open_chain(island)]

    if len(curves) < 2:
        _report(
            report,
            {'WARNING'},
            "Tangify needs at least two selected closed edge curves.",
        )
        return {'CANCELLED'}
    if not guides:
        _report(
            report,
            {'WARNING'},
            "Tangify needs at least one selected open tangent line.",
        )
        return {'CANCELLED'}

    prepared_guides = []
    for guide_number, guide in enumerate(guides, start=1):
        match = _match_guide(guide, curves)
        if match is None:
            _report(
                report,
                {'WARNING'},
                f"Tangify could not pair line {guide_number} with two curves.",
            )
            return {'CANCELLED'}

        start = guide['verts'][0].co
        end = guide['verts'][-1].co
        direction = end - start
        if direction.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON:
            _report(
                report,
                {'WARNING'},
                f"Tangify line {guide_number} has no usable length.",
            )
            return {'CANCELLED'}
        direction.normalize()

        curve_a = curves[match['index_a']]
        curve_b = curves[match['index_b']]
        solution = _common_tangent(
            curve_a['spline'],
            curve_b['spline'],
            match['t_a'],
            match['t_b'],
            direction,
        )
        if solution is None:
            _report(
                report,
                {'WARNING'},
                f"Tangify could not solve line {guide_number}.",
            )
            return {'CANCELLED'}

        curve_a['anchors'].append({
            't': solution['t_a'],
            'co': solution['point_a'].copy(),
            'touching': True,
        })
        curve_b['anchors'].append({
            't': solution['t_b'],
            'co': solution['point_b'].copy(),
            'touching': True,
        })
        prepared_guides.append((guide, solution))

    prepared_curves = []
    for curve in curves:
        if not curve['anchors']:
            continue
        anchors = _deduplicate_anchors(curve['anchors'], curve['spline'])
        target_count = len(curve['island']['verts'])
        if len(anchors) > target_count:
            _report(
                report,
                {'WARNING'},
                "Tangify found more tangent contacts than available curve vertices.",
            )
            return {'CANCELLED'}
        target_coords = _closed_target_coords(
            curve['spline'],
            anchors,
            target_count,
        )
        if len(target_coords) != target_count:
            _report(
                report,
                {'ERROR'},
                "Tangify generated an invalid curve sample count.",
            )
            return {'CANCELLED'}
        prepared_curves.append((curve['island']['verts'], target_coords))

    # All validation and solving happens before this point so a failed solve
    # never leaves the Edit Mesh half modified.
    for verts, target_coords in prepared_curves:
        for vert, coordinate in zip(verts, target_coords):
            vert.co = coordinate
            vert.select = True

    for guide, solution in prepared_guides:
        target_coords = _guide_target_coords(
            guide,
            solution['point_a'],
            solution['point_b'],
        )
        for vert, coordinate in zip(guide['verts'], target_coords):
            vert.co = coordinate
            vert.select = True

    bmesh.update_edit_mesh(obj.data)
    _report(
        report,
        {'INFO'},
        f"Tangified {len(prepared_guides)} line"
        f"{'s' if len(prepared_guides) != 1 else ''} across "
        f"{len(prepared_curves)} curve"
        f"{'s' if len(prepared_curves) != 1 else ''}; vertex counts unchanged.",
    )
    return {'FINISHED'}
