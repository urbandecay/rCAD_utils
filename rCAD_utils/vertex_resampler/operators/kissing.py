"""Contact-aware Catmull-Rom resampling for selected edge curves.

Each selected edge island remains separate topology.  The closest positions
between related interpolated curves become mandatory sample anchors, so every
resample keeps a vertex at both sides of each contact.  Curves which already
touch receive two distinct vertices at the exact same coordinate.
"""

import math

import bmesh
from mathutils import kdtree

from ..math_engine import CatmullRomSpline
from .detection_utils import get_selected_islands


_TOUCH_EPSILON = 1e-4
_PARAM_EPSILON = 1e-5
_LENGTH_EPSILON = 1e-10


def _report(report, level, message):
    if report is not None:
        report(level, message)


def _sample_parameters(spline, samples_per_segment=10):
    num_segs = len(spline.segments)
    if num_segs == 0:
        return []

    sample_count = max(16, num_segs * samples_per_segment)
    if spline.is_closed:
        return [
            (index / sample_count) * num_segs
            for index in range(sample_count)
        ]
    return [
        (index / sample_count) * num_segs
        for index in range(sample_count + 1)
    ]


def _closest_spline_pair(spline_a, spline_b):
    """Find a stable minimum-distance parameter pair on two splines."""
    params_a = _sample_parameters(spline_a)
    params_b = _sample_parameters(spline_b)
    if not params_a or not params_b:
        return None

    points_b = [spline_b.eval_global(t) for t in params_b]
    tree = kdtree.KDTree(len(points_b))
    for index, point in enumerate(points_b):
        tree.insert(point, index)
    tree.balance()

    best_t_a = params_a[0]
    best_t_b = params_b[0]
    best_dist = float('inf')
    for t_a in params_a:
        point_a = spline_a.eval_global(t_a)
        _co, index_b, distance = tree.find(point_a)
        if distance < best_dist:
            best_dist = distance
            best_t_a = t_a
            best_t_b = params_b[index_b]

    # Alternating spline projection refines the coarse global seed.
    point_b = spline_b.eval_global(best_t_b)
    for _ in range(12):
        next_t_a, point_a, _distance_a = spline_a.project(point_b)
        next_t_b, next_point_b, _distance_b = spline_b.project(point_a)
        delta = abs(next_t_a - best_t_a) + abs(next_t_b - best_t_b)
        best_t_a = next_t_a
        best_t_b = next_t_b
        point_b = next_point_b
        if delta <= 1e-9:
            break

    point_a = spline_a.eval_global(best_t_a)
    point_b = spline_b.eval_global(best_t_b)
    return {
        't_a': best_t_a,
        't_b': best_t_b,
        'point_a': point_a,
        'point_b': point_b,
        'distance': (point_a - point_b).length,
    }


def _minimum_spanning_pairs(splines):
    """Choose the nearest N-1 relationships without requiring parent metadata."""
    candidates = []
    for index_a in range(len(splines)):
        for index_b in range(index_a + 1, len(splines)):
            solution = _closest_spline_pair(
                splines[index_a],
                splines[index_b],
            )
            if solution is not None:
                candidates.append((
                    solution['distance'],
                    index_a,
                    index_b,
                    solution,
                ))

    parents = list(range(len(splines)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(index_a, index_b):
        root_a = find(index_a)
        root_b = find(index_b)
        if root_a == root_b:
            return False
        parents[root_b] = root_a
        return True

    chosen = []
    for _distance, index_a, index_b, solution in sorted(
        candidates,
        key=lambda item: (item[0], item[1], item[2]),
    ):
        if union(index_a, index_b):
            chosen.append((index_a, index_b, solution))
            if len(chosen) == len(splines) - 1:
                break
    return chosen


def _parameter_distance(a, b, period=None):
    distance = abs(a - b)
    if period is not None and period > 0.0:
        distance = min(distance, period - distance)
    return distance


def _deduplicate_anchors(anchors, spline):
    if not anchors:
        return []

    period = float(len(spline.segments)) if spline.is_closed else None
    ordered = sorted(anchors, key=lambda anchor: anchor['t'])
    unique = []
    for anchor in ordered:
        candidate = dict(anchor)
        if period is not None:
            candidate['t'] %= period

        match = None
        for existing in unique:
            if _parameter_distance(
                candidate['t'],
                existing['t'],
                period,
            ) <= _PARAM_EPSILON:
                match = existing
                break

        if match is None:
            unique.append(candidate)
        elif candidate.get('touching') and not match.get('touching'):
            match['co'] = candidate['co']
            match['touching'] = True

    return sorted(unique, key=lambda anchor: anchor['t'])


def _interval_lut(spline, t_start, t_end):
    span = max(0.0, t_end - t_start)
    steps = max(24, int(math.ceil(span * 24.0)))
    params = []
    lengths = [0.0]
    previous = None

    for index in range(steps + 1):
        factor = index / steps
        t = t_start + span * factor
        point = spline.eval_global(t)
        params.append(t)
        if previous is not None:
            lengths.append(lengths[-1] + (point - previous).length)
        previous = point

    return params, lengths


def _parameter_at_length(params, lengths, target_length):
    if not params:
        return 0.0
    if target_length <= 0.0:
        return params[0]
    if target_length >= lengths[-1]:
        return params[-1]

    low = 0
    high = len(lengths) - 1
    while low + 1 < high:
        middle = (low + high) // 2
        if lengths[middle] < target_length:
            low = middle
        else:
            high = middle

    segment_length = lengths[high] - lengths[low]
    if segment_length <= _LENGTH_EPSILON:
        return params[low]
    factor = (target_length - lengths[low]) / segment_length
    return params[low] + (params[high] - params[low]) * factor


def _allocate_segment_counts(lengths, total_segments):
    interval_count = len(lengths)
    if interval_count == 0:
        return []

    total_segments = max(interval_count, int(total_segments))
    counts = [1] * interval_count
    remaining = total_segments - interval_count
    if remaining == 0:
        return counts

    total_length = sum(lengths)
    if total_length <= _LENGTH_EPSILON:
        for index in range(remaining):
            counts[index % interval_count] += 1
        return counts

    raw_extras = [
        remaining * (length / total_length)
        for length in lengths
    ]
    floor_extras = [int(math.floor(value)) for value in raw_extras]
    for index, extra in enumerate(floor_extras):
        counts[index] += extra

    leftovers = remaining - sum(floor_extras)
    ranked = sorted(
        range(interval_count),
        key=lambda index: (
            raw_extras[index] - floor_extras[index],
            lengths[index],
            -index,
        ),
        reverse=True,
    )
    for index in ranked[:leftovers]:
        counts[index] += 1
    return counts


def _sample_interval(spline, t_start, t_end, segment_count):
    params, lengths = _interval_lut(spline, t_start, t_end)
    total_length = lengths[-1] if lengths else 0.0
    coords = []

    for index in range(segment_count + 1):
        if index == 0:
            t = t_start
        elif index == segment_count:
            t = t_end
        elif total_length <= _LENGTH_EPSILON:
            factor = index / segment_count
            t = t_start + (t_end - t_start) * factor
        else:
            target_length = total_length * (index / segment_count)
            t = _parameter_at_length(params, lengths, target_length)
        coords.append(spline.eval_global(t))
    return coords, total_length


def _closed_target_coords(spline, anchors, target_count):
    num_segs = float(len(spline.segments))
    if not anchors:
        anchors = [{
            't': 0.0,
            'co': spline.eval_global(0.0),
            'touching': False,
        }]

    interval_data = []
    for index, anchor in enumerate(anchors):
        next_anchor = anchors[(index + 1) % len(anchors)]
        t_start = anchor['t']
        t_end = next_anchor['t']
        if index == len(anchors) - 1 or t_end <= t_start:
            t_end += num_segs
        _coords, length = _sample_interval(spline, t_start, t_end, 1)
        interval_data.append((anchor, next_anchor, t_start, t_end, length))

    counts = _allocate_segment_counts(
        [item[4] for item in interval_data],
        target_count,
    )
    coords = []
    for segment_count, item in zip(counts, interval_data):
        anchor, _next_anchor, t_start, t_end, _length = item
        section, _section_length = _sample_interval(
            spline,
            t_start,
            t_end,
            segment_count,
        )
        section[0] = anchor['co'].copy()
        coords.extend(section[:-1])
    return coords


def _open_target_coords(spline, anchors, target_count):
    num_segs = float(len(spline.segments))
    boundary_anchors = [
        {
            't': 0.0,
            'co': spline.eval_global(0.0),
            'touching': False,
        },
        *anchors,
        {
            't': num_segs,
            'co': spline.eval_global(num_segs),
            'touching': False,
        },
    ]
    boundary_anchors = _deduplicate_anchors(boundary_anchors, spline)

    interval_data = []
    for anchor, next_anchor in zip(
        boundary_anchors,
        boundary_anchors[1:],
    ):
        _coords, length = _sample_interval(
            spline,
            anchor['t'],
            next_anchor['t'],
            1,
        )
        interval_data.append((anchor, next_anchor, length))

    counts = _allocate_segment_counts(
        [item[2] for item in interval_data],
        target_count - 1,
    )
    coords = []
    for index, (segment_count, item) in enumerate(zip(counts, interval_data)):
        anchor, next_anchor, _length = item
        section, _section_length = _sample_interval(
            spline,
            anchor['t'],
            next_anchor['t'],
            segment_count,
        )
        section[0] = anchor['co'].copy()
        section[-1] = next_anchor['co'].copy()
        if index:
            section = section[1:]
        coords.extend(section)
    return coords


def _resize_curve_topology(bm, verts, target_coords, closed):
    """Resize one edge island without ever joining it to another island."""
    target_count = len(target_coords)
    if target_count < 2 or len(verts) < 2:
        return False

    while len(verts) < target_count:
        edge_count = len(verts) if closed else len(verts) - 1
        best_index = -1
        best_length = -1.0
        for index in range(edge_count):
            next_index = (index + 1) % len(verts)
            edge = bm.edges.get((verts[index], verts[next_index]))
            if edge is None:
                edge = bm.edges.get((verts[next_index], verts[index]))
            if edge is not None and edge.calc_length() > best_length:
                best_index = index
                best_length = edge.calc_length()

        if best_index < 0:
            return False

        next_index = (best_index + 1) % len(verts)
        first_vert = verts[best_index]
        second_vert = verts[next_index]
        edge = bm.edges.get((first_vert, second_vert))
        if edge is None:
            edge = bm.edges.get((second_vert, first_vert))
        if edge is None:
            return False

        result = bmesh.utils.edge_split(edge, first_vert, 0.5)
        new_vert = (
            result[0]
            if isinstance(result[0], bmesh.types.BMVert)
            else result[1]
        )
        verts.insert(best_index + 1, new_vert)

    while len(verts) > target_count:
        if closed:
            if len(verts) <= 3:
                return False
            kill_index = 1
        else:
            if len(verts) <= 2:
                return False
            kill_index = 1

        kill_vert = verts.pop(kill_index)
        if kill_vert.is_valid:
            bmesh.ops.dissolve_verts(bm, verts=[kill_vert])

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()

    if len(verts) != target_count:
        return False

    for vert, coordinate in zip(verts, target_coords):
        if not vert.is_valid:
            return False
        vert.co = coordinate
        vert.select = True

    edge_count = len(verts) if closed else len(verts) - 1
    for index in range(edge_count):
        next_index = (index + 1) % len(verts)
        edge = bm.edges.get((verts[index], verts[next_index]))
        if edge is None:
            edge = bm.edges.get((verts[next_index], verts[index]))
        if edge is not None:
            edge.select = True
    return True


def execute(bm, obj, direction, report=None):
    islands = get_selected_islands(bm)
    if len(islands) < 2:
        _report(
            report,
            {'WARNING'},
            "Kissing needs at least two selected edge curves.",
        )
        return {'CANCELLED'}

    splines = []
    for island in islands:
        points = [vert.co.copy() for vert in island['verts']]
        spline = CatmullRomSpline(points, island['closed'])
        if not spline.segments:
            _report(
                report,
                {'WARNING'},
                "Kissing could not interpolate one of the selected curves.",
            )
            return {'CANCELLED'}
        splines.append(spline)

    chosen_pairs = _minimum_spanning_pairs(splines)
    if len(chosen_pairs) != len(islands) - 1:
        _report(
            report,
            {'WARNING'},
            "Kissing could not establish contact relationships.",
        )
        return {'CANCELLED'}

    anchors_by_island = [[] for _island in islands]
    touching_pairs = 0
    for index_a, index_b, solution in chosen_pairs:
        touching = solution['distance'] <= _TOUCH_EPSILON
        point_a = solution['point_a']
        point_b = solution['point_b']
        if touching:
            shared_point = (point_a + point_b) * 0.5
            point_a = shared_point
            point_b = shared_point
            touching_pairs += 1

        anchors_by_island[index_a].append({
            't': solution['t_a'],
            'co': point_a.copy(),
            'touching': touching,
        })
        anchors_by_island[index_b].append({
            't': solution['t_b'],
            'co': point_b.copy(),
            'touching': touching,
        })

    prepared = []
    for island, spline, raw_anchors in zip(
        islands,
        splines,
        anchors_by_island,
    ):
        anchors = _deduplicate_anchors(raw_anchors, spline)
        current_count = len(island['verts'])

        if island['closed']:
            minimum_count = max(3, len(anchors))
        else:
            minimum_count = max(2, len(anchors) + 2)

        target_count = max(minimum_count, current_count + direction)
        if direction < 0 and current_count <= minimum_count:
            _report(
                report,
                {'WARNING'},
                "Kissing cannot remove another vertex without losing an anchor.",
            )
            return {'CANCELLED'}

        if island['closed']:
            target_coords = _closed_target_coords(
                spline,
                anchors,
                target_count,
            )
        else:
            target_coords = _open_target_coords(
                spline,
                anchors,
                target_count,
            )

        if len(target_coords) != target_count:
            _report(
                report,
                {'ERROR'},
                "Kissing generated an invalid resample count.",
            )
            return {'CANCELLED'}
        prepared.append((island, target_coords))

    for island, target_coords in prepared:
        if not _resize_curve_topology(
            bm,
            island['verts'],
            target_coords,
            island['closed'],
        ):
            _report(
                report,
                {'ERROR'},
                "Kissing could not safely update the selected topology.",
            )
            return {'CANCELLED'}

    bmesh.update_edit_mesh(obj.data)
    _report(
        report,
        {'INFO'},
        "Kissing resampled "
        f"{len(islands)} curves with {len(chosen_pairs)} paired anchors "
        f"({touching_pairs} overlapping).",
    )
    return {'FINISHED'}
