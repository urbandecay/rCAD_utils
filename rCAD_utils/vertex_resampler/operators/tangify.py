"""Rephase selected faceted loops onto selected contacts and tangent lines.

Tangify keeps the selected topology intact.  Each selected open edge chain is
treated as a tangent guide between two selected closed loops.  The guide's
nearby common tangent is solved on the same Catmull-Rom interpolation used by
Kissing mode, then the existing loop vertices are redistributed with the
tangent points as mandatory anchors.  When no guides are selected, the nearest
closed-loop relationships are used as shared contact anchors instead.
"""

import math

import bmesh

from ..math_engine import CatmullRomSpline
from .detection_utils import get_selected_islands
from .kissing import (
    _closed_target_coords,
    _closest_spline_pair,
    _deduplicate_anchors,
    _minimum_spanning_pairs,
)


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


def _contact_alignment_error(spline_a, spline_b, t_a, t_b):
    point_a = spline_a.eval_global(t_a)
    point_b = spline_b.eval_global(t_b)
    tangent_a = spline_a.tangent_global(t_a)
    tangent_b = spline_b.tangent_global(t_b)
    if (
        tangent_a.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON
        or tangent_b.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON
    ):
        return float('inf'), point_a, point_b
    dot = max(-1.0, min(1.0, tangent_a.dot(tangent_b)))
    return 1.0 - dot * dot, point_a, point_b


def _spline_scale(spline):
    samples = [
        spline.eval_global(index)
        for index in range(len(spline.segments))
    ]
    if not samples:
        return 1.0
    minimum = samples[0].copy()
    maximum = samples[0].copy()
    for point in samples[1:]:
        minimum.x = min(minimum.x, point.x)
        minimum.y = min(minimum.y, point.y)
        minimum.z = min(minimum.z, point.z)
        maximum.x = max(maximum.x, point.x)
        maximum.y = max(maximum.y, point.y)
        maximum.z = max(maximum.z, point.z)
    return max((maximum - minimum).length, 1.0e-6)


def _contact_score(
    spline_a,
    spline_b,
    t_a,
    t_b,
    seed_a,
    seed_b,
    window_a,
    window_b,
    scale,
):
    alignment, point_a, point_b = _contact_alignment_error(
        spline_a,
        spline_b,
        t_a,
        t_b,
    )
    if not math.isfinite(alignment):
        return float('inf')
    distance_error = (point_a - point_b).length / scale
    drift_a = _parameter_distance(spline_a, t_a, seed_a) / window_a
    drift_b = _parameter_distance(spline_b, t_b, seed_b) / window_b
    # Tangency is the important part of a contact.  The distance term keeps
    # the search at the closest intended relationship when the faceted source
    # curves overlap or leave a tiny gap.
    return (
        alignment
        + distance_error * distance_error
        + 0.001 * (drift_a * drift_a + drift_b * drift_b)
    )


def _tangent_contact(spline_a, spline_b):
    seed = _closest_spline_pair(spline_a, spline_b)
    if seed is None:
        return None

    count_a = float(len(spline_a.segments))
    count_b = float(len(spline_b.segments))
    window_a = min(count_a * 0.12, max(0.75, count_a * 0.04))
    window_b = min(count_b * 0.12, max(0.75, count_b * 0.04))
    scale = max(_spline_scale(spline_a), _spline_scale(spline_b))
    best_t_a = seed['t_a']
    best_t_b = seed['t_b']
    best_score = float('inf')

    coarse_steps = 12
    for index_a in range(coarse_steps + 1):
        raw_a = seed['t_a'] - window_a + 2.0 * window_a * index_a / coarse_steps
        t_a = _clamp_local_parameter(spline_a, raw_a, seed['t_a'], window_a)
        for index_b in range(coarse_steps + 1):
            raw_b = seed['t_b'] - window_b + 2.0 * window_b * index_b / coarse_steps
            t_b = _clamp_local_parameter(spline_b, raw_b, seed['t_b'], window_b)
            score = _contact_score(
                spline_a,
                spline_b,
                t_a,
                t_b,
                seed['t_a'],
                seed['t_b'],
                window_a,
                window_b,
                scale,
            )
            if score < best_score:
                best_score = score
                best_t_a = t_a
                best_t_b = t_b

    step_a = 2.0 * window_a / coarse_steps
    step_b = 2.0 * window_b / coarse_steps
    for _iteration in range(48):
        candidate_t_a = best_t_a
        candidate_t_b = best_t_b
        candidate_score = best_score
        for offset_a in (-step_a, 0.0, step_a):
            t_a = _clamp_local_parameter(
                spline_a,
                best_t_a + offset_a,
                seed['t_a'],
                window_a,
            )
            for offset_b in (-step_b, 0.0, step_b):
                t_b = _clamp_local_parameter(
                    spline_b,
                    best_t_b + offset_b,
                    seed['t_b'],
                    window_b,
                )
                score = _contact_score(
                    spline_a,
                    spline_b,
                    t_a,
                    t_b,
                    seed['t_a'],
                    seed['t_b'],
                    window_a,
                    window_b,
                    scale,
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
            if max(step_a, step_b) <= 1.0e-9:
                break

    _alignment, point_a, point_b = _contact_alignment_error(
        spline_a,
        spline_b,
        best_t_a,
        best_t_b,
    )
    if not math.isfinite(_alignment):
        return None
    shared_point = (point_a + point_b) * 0.5
    return {
        't_a': best_t_a,
        't_b': best_t_b,
        'point_a': shared_point,
        'point_b': shared_point.copy(),
        'distance': (point_a - point_b).length,
        'alignment': _alignment,
    }


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
    matches = []
    for endpoint_index in (0, -1):
        point = guide['verts'][endpoint_index].co
        candidates = []
        for curve_index, curve in enumerate(curves):
            parameter, _point, distance = curve['spline'].project(point)
            candidates.append((distance, curve_index, parameter))
        if not candidates:
            return None
        matches.append(min(candidates, key=lambda item: item[0]))

    start_match, end_match = matches
    if start_match[1] == end_match[1]:
        return None

    # A line is considered connected when both endpoints are near separate
    # interpolated curves.  This prevents a free endpoint from being treated
    # as a second curve merely because it is the closest available geometry.
    start_tolerance = max(
        1.0e-4,
        _spline_scale(curves[start_match[1]]['spline']) * 0.12,
    )
    end_tolerance = max(
        1.0e-4,
        _spline_scale(curves[end_match[1]]['spline']) * 0.12,
    )
    if start_match[0] > start_tolerance or end_match[0] > end_tolerance:
        return None
    return {
        'index_a': start_match[1],
        'index_b': end_match[1],
        't_a': start_match[2],
        't_b': end_match[2],
        'distance_a': start_match[0],
        'distance_b': end_match[0],
        'score': start_match[0] + end_match[0],
    }


def _match_single_guide(guide, curves):
    best = None
    for endpoint_index in (0, -1):
        point = guide['verts'][endpoint_index].co
        for curve_index, curve in enumerate(curves):
            parameter, _point, distance = curve['spline'].project(point)
            candidate = {
                'endpoint_index': endpoint_index,
                'curve_index': curve_index,
                't': parameter,
                'distance': distance,
                'point': point.copy(),
            }
            if best is None or distance < best['distance']:
                best = candidate
    if best is None:
        return None
    tolerance = max(
        1.0e-4,
        _spline_scale(curves[best['curve_index']]['spline']) * 0.12,
    )
    return best if best['distance'] <= tolerance else None


def _single_curve_solution(spline, seed, direction):
    seed_tangent = spline.tangent_global(seed)
    if seed_tangent.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON:
        return None
    seed_dot = abs(seed_tangent.dot(direction))
    relation = 'TANGENT' if seed_dot >= 0.70710678 else 'NORMAL'
    count = float(len(spline.segments))
    window = min(count * 0.20, max(1.0, count * 0.08))

    def score(parameter):
        tangent = spline.tangent_global(parameter)
        if tangent.length_squared <= _LENGTH_EPSILON * _LENGTH_EPSILON:
            return float('inf')
        dot = max(-1.0, min(1.0, abs(tangent.dot(direction))))
        alignment = (1.0 - dot * dot) if relation == 'TANGENT' else dot * dot
        drift = _parameter_distance(spline, parameter, seed) / window
        return alignment + 0.001 * drift * drift

    best_t = seed
    best_score = float('inf')
    coarse_steps = 16
    for index in range(coarse_steps + 1):
        raw = seed - window + 2.0 * window * index / coarse_steps
        parameter = _clamp_local_parameter(spline, raw, seed, window)
        candidate_score = score(parameter)
        if candidate_score < best_score:
            best_t = parameter
            best_score = candidate_score

    step = 2.0 * window / coarse_steps
    for _iteration in range(48):
        candidate_t = best_t
        candidate_score = best_score
        for offset in (-step, 0.0, step):
            parameter = _clamp_local_parameter(
                spline,
                best_t + offset,
                seed,
                window,
            )
            candidate = score(parameter)
            if candidate < candidate_score:
                candidate_t = parameter
                candidate_score = candidate
        moved = _parameter_distance(spline, candidate_t, best_t) > 1.0e-12
        best_t = candidate_t
        best_score = candidate_score
        if not moved:
            step *= 0.5
            if step <= 1.0e-9:
                break

    point = spline.eval_global(best_t)
    return {
        't': best_t,
        'point': point,
        'relation': relation,
        'alignment_error': best_score,
    }


def _prepare_curve_targets(curves, report=None):
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
                "Tangify found more contacts than available curve vertices.",
            )
            return None
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
            return None
        prepared_curves.append((curve['island']['verts'], target_coords))
    return prepared_curves


def execute(bm, obj, report=None):
    islands = get_selected_islands(bm)
    curves = _build_curves(islands)
    guides = [island for island in islands if _is_straight_open_chain(island)]

    if not curves:
        _report(
            report,
            {'WARNING'},
            "Tangify needs at least one selected closed edge curve.",
        )
        return {'CANCELLED'}
    if not guides and len(curves) < 2:
        _report(
            report,
            {'WARNING'},
            "Tangify contact mode needs at least two selected closed curves.",
        )
        return {'CANCELLED'}

    prepared_guides = []
    contact_count = 0
    if not guides:
        if len(curves) != len(islands):
            _report(
                report,
                {'WARNING'},
                "Tangify only supports closed curves in contact-only mode.",
            )
            return {'CANCELLED'}

        splines = [curve['spline'] for curve in curves]
        chosen_pairs = _minimum_spanning_pairs(splines)
        if len(chosen_pairs) != len(curves) - 1:
            _report(
                report,
                {'WARNING'},
                "Tangify could not establish all curve contact relationships.",
            )
            return {'CANCELLED'}
        for index_a, index_b, solution in chosen_pairs:
            curve_a = curves[index_a]
            curve_b = curves[index_b]
            solution = _tangent_contact(
                curve_a['spline'],
                curve_b['spline'],
            )
            if solution is None:
                _report(
                    report,
                    {'WARNING'},
                    "Tangify could not solve a Catmull-Rom curve contact.",
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
        contact_count = len(chosen_pairs)
    else:
        for guide_number, guide in enumerate(guides, start=1):
            match = _match_guide(guide, curves)

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

            if match is not None:
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
                    # The guide endpoint is fixed geometry.  Use its spline
                    # projection as the sample parameter so the anchor point
                    # and the arc-length distribution share the same place.
                    't': match['t_a'],
                    'co': start.copy(),
                    'touching': True,
                })
                curve_b['anchors'].append({
                    't': match['t_b'],
                    'co': end.copy(),
                    'touching': True,
                })
                prepared_guides.append((guide, solution))
                continue

            single_match = _match_single_guide(guide, curves)
            if single_match is None:
                _report(
                    report,
                    {'WARNING'},
                    f"Tangify could not find a curve contact for line {guide_number}.",
                )
                return {'CANCELLED'}

            curve = curves[single_match['curve_index']]
            solution = _single_curve_solution(
                curve['spline'],
                single_match['t'],
                direction,
            )
            if solution is None:
                _report(
                    report,
                    {'WARNING'},
                    f"Tangify could not solve line {guide_number}.",
                )
                return {'CANCELLED'}
            curve['anchors'].append({
                # Keep the stationary line endpoint on the Catmull-Rom
                # curve.  The solver selects the relation branch; the
                # projection is the authoritative fixed contact parameter.
                't': single_match['t'],
                'co': (
                    start.copy()
                    if single_match['endpoint_index'] == 0
                    else end.copy()
                ),
                'touching': True,
            })
            prepared_guides.append((guide, solution))

    prepared_curves = _prepare_curve_targets(curves, report=report)
    if prepared_curves is None:
        return {'CANCELLED'}

    # All validation and solving happens before this point so a failed solve
    # never leaves the Edit Mesh half modified.
    for verts, target_coords in prepared_curves:
        for vert, coordinate in zip(verts, target_coords):
            vert.co = coordinate
            vert.select = True

    bmesh.update_edit_mesh(obj.data)
    if guides:
        message = (
            f"Tangified {len(prepared_guides)} line"
            f"{'s' if len(prepared_guides) != 1 else ''} across "
            f"{len(prepared_curves)} curve"
            f"{'s' if len(prepared_curves) != 1 else ''}"
        )
    else:
        message = (
            f"Tangified {contact_count} curve contact"
            f"{'s' if contact_count != 1 else ''} across "
            f"{len(prepared_curves)} curves"
        )
    _report(report, {'INFO'}, message + "; vertex counts unchanged.")
    return {'FINISHED'}
