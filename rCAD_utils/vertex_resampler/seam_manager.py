# seam_manager.py — Seam home persistence, drift detection, and migration.
# Seam homes are stored per-object as a flat float list in obj["rcad_seam_origins"].
# Each seam vert's ORIGINAL position is preserved across presses so drift
# accumulates against the true origin, not last-press position.

import bmesh
from mathutils import Vector, kdtree

from .debug import debug_log, vert_ref


_VECTOR_TOLERANCE = 1.0e-9


def _ring_vert_positions(ring_info):
    return {
        vert: index
        for index, vert in enumerate(ring_info.verts)
        if vert.is_valid
    }


def _ring_step_distance(ring_info, index_a, index_b):
    delta = abs(index_a - index_b)
    if ring_info.is_closed:
        count = len(ring_info.verts)
        return min(delta, count - delta)
    return delta


def _min_ring_spacing(ring_info, vert, other_verts, positions=None):
    if not other_verts:
        return len(ring_info.verts)

    if positions is None:
        positions = _ring_vert_positions(ring_info)

    index = positions.get(vert)
    if index is None:
        return -1

    valid_other_indices = [
        positions[other]
        for other in other_verts
        if other in positions
    ]
    if not valid_other_indices:
        return len(ring_info.verts)

    return min(
        _ring_step_distance(ring_info, index, other_index)
        for other_index in valid_other_indices
    )


def _seam_clearance_steps(ring_info):
    if not ring_info.is_closed:
        return 1
    return 2 if len(ring_info.verts) >= 8 else 1


def _safe_normalized(vector):
    if vector.length <= _VECTOR_TOLERANCE:
        return None
    return vector.normalized()


def _adjacent_ring_edge_dirs(ring_info, vert, positions):
    index = positions.get(vert)
    if index is None:
        return []

    loop = ring_info.verts
    count = len(loop)
    if count < 2:
        return []

    neighbor_indices = []
    if ring_info.is_closed:
        neighbor_indices.extend([
            (index - 1) % count,
            (index + 1) % count,
        ])
    else:
        if index > 0:
            neighbor_indices.append(index - 1)
        if index < count - 1:
            neighbor_indices.append(index + 1)

    directions = []
    for neighbor_index in neighbor_indices:
        neighbor = loop[neighbor_index]
        if not neighbor.is_valid:
            continue
        direction = _safe_normalized(neighbor.co - vert.co)
        if direction is not None:
            directions.append(direction)
    return directions


def _seam_perpendicular_error(ring_info, vert, non_ring_vert, positions):
    if non_ring_vert is None or not non_ring_vert.is_valid:
        return float('inf')

    seam_direction = _safe_normalized(non_ring_vert.co - vert.co)
    if seam_direction is None:
        return float('inf')

    ring_dirs = _adjacent_ring_edge_dirs(ring_info, vert, positions)
    if not ring_dirs:
        return float('inf')

    return sum(abs(seam_direction.dot(direction)) for direction in ring_dirs) / len(ring_dirs)


def _local_tangent_dirs(ring_info, vert, positions, sample_steps=2):
    index = positions.get(vert)
    if index is None:
        return []

    loop = ring_info.verts
    count = len(loop)
    if count < 3:
        return []

    directions = []
    for step in range(1, sample_steps + 1):
        if ring_info.is_closed:
            prev_vert = loop[(index - step) % count]
            next_vert = loop[(index + step) % count]
        else:
            prev_index = index - step
            next_index = index + step
            if prev_index < 0 or next_index >= count:
                continue
            prev_vert = loop[prev_index]
            next_vert = loop[next_index]

        if not (prev_vert.is_valid and next_vert.is_valid):
            continue
        direction = _safe_normalized(next_vert.co - prev_vert.co)
        if direction is not None:
            directions.append(direction)

    if directions:
        return directions

    return _adjacent_ring_edge_dirs(ring_info, vert, positions)


def _local_perpendicular_score(ring_info, vert, non_ring_vert, positions, sample_steps=2):
    if non_ring_vert is None or not non_ring_vert.is_valid:
        return (float('inf'), float('inf'))

    seam_direction = _safe_normalized(non_ring_vert.co - vert.co)
    if seam_direction is None:
        return (float('inf'), float('inf'))

    tangent_dirs = _local_tangent_dirs(
        ring_info,
        vert,
        positions,
        sample_steps=sample_steps,
    )
    if not tangent_dirs:
        return (float('inf'), float('inf'))

    tangent_error = (
        sum(abs(seam_direction.dot(direction)) for direction in tangent_dirs)
        / len(tangent_dirs)
    )

    direct_dirs = _adjacent_ring_edge_dirs(ring_info, vert, positions)
    direct_error = (
        sum(abs(seam_direction.dot(direction)) for direction in direct_dirs)
        / len(direct_dirs)
        if direct_dirs else tangent_error
    )

    return (tangent_error, direct_error)


def _local_ring_candidates(ring_info, center_vert, positions, search_steps=2):
    center_index = positions.get(center_vert)
    if center_index is None:
        return []

    loop = ring_info.verts
    count = len(loop)
    candidates = []
    seen = set()

    for offset in range(-search_steps, search_steps + 1):
        if ring_info.is_closed:
            index = (center_index + offset) % count
        else:
            index = center_index + offset
            if index < 0 or index >= count:
                continue

        vert = loop[index]
        if not vert.is_valid or vert in seen:
            continue
        seen.add(vert)
        candidates.append(vert)

    return candidates


def _seam_target_score(
    ring_info,
    vert,
    home,
    other_targets,
    clearance_steps,
    positions,
    non_ring_vert=None,
):
    min_spacing = _min_ring_spacing(ring_info, vert, other_targets, positions)
    home_dist = (vert.co - home).length
    perpendicular_error = _seam_perpendicular_error(
        ring_info,
        vert,
        non_ring_vert,
        positions,
    )
    is_clear = min_spacing >= clearance_steps
    return (
        0 if is_clear else 1,
        perpendicular_error,
        home_dist,
        -min_spacing,
        vert.index,
    ), min_spacing, perpendicular_error


def _outside_edges_for_vert(vert, all_ring_verts):
    return [
        edge for edge in vert.link_edges
        if edge.is_valid and edge.other_vert(vert) not in all_ring_verts
    ]


def _primary_outside_edge_for_vert(vert, all_ring_verts):
    seam_edges = _outside_edges_for_vert(vert, all_ring_verts)
    if not seam_edges:
        return None
    return max(seam_edges, key=lambda edge: edge.calc_length())


def _occupied_ring_seam_verts(ring_info, all_ring_verts):
    occupied = set()
    for vert in ring_info.verts:
        if vert.is_valid and _outside_edges_for_vert(vert, all_ring_verts):
            occupied.add(vert)
    return occupied


def _nearest_ring_index_for_point(ring_info, point):
    best_index = None
    best_dist = None
    for index, vert in enumerate(ring_info.verts):
        if not vert.is_valid:
            continue
        dist = (vert.co - point).length
        candidate = (dist, index)
        if best_dist is None or candidate < best_dist:
            best_dist = candidate
            best_index = index
    return best_index


def _home_indices_for_seams(ring_info, seam_homes):
    home_indices = {}
    for seam_vert, home in seam_homes.items():
        if seam_vert not in ring_info.verts or not seam_vert.is_valid:
            continue
        home_index = _nearest_ring_index_for_point(ring_info, home)
        if home_index is not None:
            home_indices[seam_vert] = home_index
    return home_indices


def _candidate_in_home_sector(ring_info, candidate_vert, seam_vert, home_indices, positions):
    if not ring_info.is_closed or len(home_indices) <= 1:
        return True

    candidate_index = positions.get(candidate_vert)
    own_home_index = home_indices.get(seam_vert)
    if candidate_index is None or own_home_index is None:
        return True

    own_key = (
        _ring_step_distance(ring_info, candidate_index, own_home_index),
        own_home_index,
        seam_vert.index,
    )
    best_key = own_key
    best_seam = seam_vert

    for other_seam, other_home_index in home_indices.items():
        key = (
            _ring_step_distance(ring_info, candidate_index, other_home_index),
            other_home_index,
            other_seam.index,
        )
        if key < best_key:
            best_key = key
            best_seam = other_seam

    return best_seam == seam_vert


def _ring_move_steps(ring_info, from_vert, to_vert, positions):
    from_index = positions.get(from_vert)
    to_index = positions.get(to_vert)
    if from_index is None or to_index is None:
        return None
    return _ring_step_distance(ring_info, from_index, to_index)


def _migration_horizon_steps(ring_info):
    if not ring_info.is_closed:
        return 1
    return 2


def load_seam_homes(obj):
    """Load persistent seam home positions from object custom property."""
    stored_flat = list(obj.get("rcad_seam_origins", []))
    homes = []
    if stored_flat and len(stored_flat) % 3 == 0:
        for i in range(0, len(stored_flat), 3):
            homes.append(Vector((stored_flat[i], stored_flat[i + 1], stored_flat[i + 2])))
    return homes


def save_seam_homes(obj, homes):
    """Save seam home positions to object custom property."""
    flat = []
    for hp in homes:
        flat.extend([hp.x, hp.y, hp.z])
    obj["rcad_seam_origins"] = flat


def match_seam_homes(ring_group, stored_homes, avg_edge_len):
    """Match current seam verts to stored home positions via KDTree.

    First press: stores current seam positions as homes.
    Later presses: matches to stored homes by proximity (3x avg edge len).
    Unmatched seam verts get their current position as a new home.

    Returns dict mapping BMVert -> home Vector.
    """
    seam_homes = {}
    seam_verts = [
        v
        for ring_info in ring_group.rings
        for v in sorted(ring_info.seam_verts, key=lambda vert: vert.index)
        if v.is_valid
    ]

    if stored_homes:
        max_match_dist = avg_edge_len * 3.0
        used_home_indices = set()
        remaining_verts = list(seam_verts)

        while remaining_verts and stored_homes:
            kd = kdtree.KDTree(len(stored_homes))
            for idx, hp in enumerate(stored_homes):
                kd.insert(hp, idx)
            kd.balance()

            best_match = None
            for v in remaining_verts:
                for home_co, home_idx, dist in kd.find_n(v.co, len(stored_homes)):
                    if home_idx in used_home_indices:
                        continue
                    if dist >= max_match_dist:
                        break
                    candidate = (dist, v.index, home_idx, v)
                    if best_match is None or candidate < best_match:
                        best_match = candidate
                    break

            if best_match is None:
                break

            _dist, _vert_index, home_idx, vert = best_match
            seam_homes[vert] = stored_homes[home_idx]
            used_home_indices.add(home_idx)
            remaining_verts.remove(vert)

        for v in remaining_verts:
            home = v.co.copy()
            seam_homes[v] = home
            stored_homes.append(home)
    else:
        for v in seam_verts:
            home = v.co.copy()
            seam_homes[v] = home
            stored_homes.append(home)

    return seam_homes


def _migrate_seam_vert(bm, old_sv, new_sv, all_ring_verts):
    """Migrate seam edges from old_sv to new_sv.
    Draws a new edge (new_sv -> non-ring vert) then dissolves the old one.
    Do NOT use face deletion — just draw + dissolve.
    """
    old_edge = _primary_outside_edge_for_vert(old_sv, all_ring_verts)
    if old_edge is None:
        return False

    non_ring_vert = old_edge.other_vert(old_sv)
    old_len = old_edge.calc_length()
    new_len = (new_sv.co - non_ring_vert.co).length

    occupied_targets = [
        edge for edge in _outside_edges_for_vert(new_sv, all_ring_verts)
        if edge.other_vert(new_sv) != non_ring_vert
    ]
    if occupied_targets:
        return False

    if new_len > max(old_len * 2.0, old_len + 1e-6):
        return False

    if not bm.edges.get([new_sv, non_ring_vert]):
        bmesh.ops.connect_verts(bm, verts=[new_sv, non_ring_vert])
    if old_edge.is_valid:
        bmesh.ops.dissolve_edges(bm, edges=[old_edge],
                                 use_verts=False, use_face_split=False)
    return True


def realign_seams_to_ring_normals(bm, ring_group, search_steps=2, sample_steps=2):
    """Move seam edges to the local ring vertex closest to a 90-degree hit.

    The candidate window is the current seam vertex plus `search_steps` verts
    on both sides. Each candidate is scored against tangents sampled from one
    and two verts on either side, which keeps face-hole connector edges from
    landing diagonally after the ring is resampled.
    """
    any_migrated = False

    for ring_info in ring_group.rings:
        positions = _ring_vert_positions(ring_info)
        reserved_targets = set()

        for seam_vert in list(ring_info.seam_verts):
            if not seam_vert.is_valid:
                continue
            primary_edge = _primary_outside_edge_for_vert(
                seam_vert,
                ring_group.all_ring_verts,
            )
            if primary_edge is None:
                continue

            non_ring_vert = primary_edge.other_vert(seam_vert)
            old_len = primary_edge.calc_length()
            occupied_targets = _occupied_ring_seam_verts(
                ring_info,
                ring_group.all_ring_verts,
            )
            blocked_targets = {
                vert for vert in occupied_targets
                if vert.is_valid and vert != seam_vert
            }
            blocked_targets.update(
                vert for vert in reserved_targets
                if vert.is_valid and vert != seam_vert
            )

            current_perpendicular_score = _local_perpendicular_score(
                ring_info,
                seam_vert,
                non_ring_vert,
                positions,
                sample_steps=sample_steps,
            )
            current_score = (
                current_perpendicular_score[0],
                current_perpendicular_score[1],
                0,
                seam_vert.index,
            )
            best = seam_vert
            best_score = current_score

            for candidate in _local_ring_candidates(
                ring_info,
                seam_vert,
                positions,
                search_steps=search_steps,
            ):
                if candidate in blocked_targets:
                    continue
                if candidate != seam_vert:
                    new_len = (candidate.co - non_ring_vert.co).length
                    if new_len > max(old_len * 2.0, old_len + 1e-6):
                        continue

                move_steps = _ring_move_steps(
                    ring_info,
                    seam_vert,
                    candidate,
                    positions,
                )
                candidate_perpendicular_score = _local_perpendicular_score(
                    ring_info,
                    candidate,
                    non_ring_vert,
                    positions,
                    sample_steps=sample_steps,
                )
                candidate_score = (
                    candidate_perpendicular_score[0],
                    candidate_perpendicular_score[1],
                    move_steps if move_steps is not None else float('inf'),
                    candidate.index,
                )
                if candidate_score < best_score:
                    best = candidate
                    best_score = candidate_score

            if best is seam_vert:
                debug_log(
                    "seam_realign",
                    "Kept seam at current local perpendicular target.",
                    seam=vert_ref(seam_vert),
                    score=tuple(
                        round(item, 6) if isinstance(item, float) else item
                        for item in current_score
                    ),
                    search_steps=search_steps,
                    sample_steps=sample_steps,
                )
                reserved_targets.add(seam_vert)
                continue

            debug_log(
                "seam_realign",
                "Migrating seam to the most perpendicular local ring target.",
                old_seam=vert_ref(seam_vert),
                new_seam=vert_ref(best),
                current_score=tuple(
                    round(item, 6) if isinstance(item, float) else item
                    for item in current_score
                ),
                new_score=tuple(
                    round(item, 6) if isinstance(item, float) else item
                    for item in best_score
                ),
                search_steps=search_steps,
                sample_steps=sample_steps,
                blocked_targets=[
                    vert_ref(v)
                    for v in sorted(blocked_targets, key=lambda vert: vert.index)
                ],
            )
            migrated = _migrate_seam_vert(
                bm,
                seam_vert,
                best,
                ring_group.all_ring_verts,
            )
            if migrated:
                ring_info.seam_verts.discard(seam_vert)
                ring_info.seam_verts.add(best)
                reserved_targets.add(best)
                any_migrated = True
            else:
                debug_log(
                    "seam_realign",
                    "Rejected perpendicular seam target during edge migration.",
                    old_seam=vert_ref(seam_vert),
                    rejected_target=vert_ref(best),
                )
                reserved_targets.add(seam_vert)

    if any_migrated:
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.normal_update()

    return any_migrated


def migrate_drifted_seams(bm, ring_group, seam_homes, threshold):
    """Check all seam verts for drift and migrate if needed.

    If a seam vert drifted more than `threshold` from its original home,
    find the nearest ring vert to the home and migrate the seam there.
    Seams that started at different positions never merge — each is
    tracked independently via its own home position.
    """
    for ring_info in ring_group.rings:
        positions = _ring_vert_positions(ring_info)
        clearance_steps = _seam_clearance_steps(ring_info)
        horizon_steps = _migration_horizon_steps(ring_info)
        home_indices = _home_indices_for_seams(ring_info, seam_homes)
        reserved_targets = set()
        for seam_vert in list(ring_info.seam_verts):
            if not seam_vert.is_valid or seam_vert not in seam_homes:
                continue
            primary_edge = _primary_outside_edge_for_vert(
                seam_vert,
                ring_group.all_ring_verts,
            )
            if primary_edge is None:
                continue
            non_ring_vert = primary_edge.other_vert(seam_vert)
            old_len = primary_edge.calc_length()
            home = seam_homes[seam_vert]
            drift = (seam_vert.co - home).length
            occupied_targets = _occupied_ring_seam_verts(ring_info, ring_group.all_ring_verts)
            blocked_targets = {
                vert for vert in ring_info.seam_verts
                if vert.is_valid and vert != seam_vert
            }
            blocked_targets.update(
                vert for vert in occupied_targets
                if vert.is_valid and vert != seam_vert
            )
            blocked_targets.update(
                vert for vert in reserved_targets
                if vert.is_valid and vert != seam_vert
            )

            current_score, current_spacing, current_perpendicular_error = _seam_target_score(
                ring_info,
                seam_vert,
                home,
                blocked_targets,
                clearance_steps,
                positions,
                non_ring_vert,
            )
            is_crowded = bool(blocked_targets) and current_spacing < clearance_steps

            best = None
            best_score = None
            best_spacing = None
            best_perpendicular_error = None

            for v in ring_info.verts:
                if not v.is_valid or v in blocked_targets:
                    continue
                move_steps = _ring_move_steps(ring_info, seam_vert, v, positions)
                if move_steps is not None and move_steps > horizon_steps:
                    continue
                if v != seam_vert:
                    new_len = (v.co - non_ring_vert.co).length
                    if new_len > max(old_len * 2.0, old_len + 1e-6):
                        continue
                if not _candidate_in_home_sector(
                    ring_info,
                    v,
                    seam_vert,
                    home_indices,
                    positions,
                ):
                    continue
                candidate_score, candidate_spacing, candidate_perpendicular_error = _seam_target_score(
                    ring_info,
                    v,
                    home,
                    blocked_targets,
                    clearance_steps,
                    positions,
                    non_ring_vert,
                )
                if best_score is None or candidate_score < best_score:
                    best_score = candidate_score
                    best_spacing = candidate_spacing
                    best_perpendicular_error = candidate_perpendicular_error
                    best = v

            if best is not None and best != seam_vert and best_score < current_score:
                debug_log(
                    "seam_migrate",
                    "Migrating seam to a more normal local ring target.",
                    old_seam=vert_ref(seam_vert),
                    new_seam=vert_ref(best),
                    drift=round(drift, 6),
                    current_spacing=current_spacing,
                    new_spacing=best_spacing,
                    current_perpendicular_error=round(current_perpendicular_error, 6),
                    new_perpendicular_error=round(best_perpendicular_error, 6),
                    clearance_steps=clearance_steps,
                    horizon_steps=horizon_steps,
                    home_index=home_indices.get(seam_vert),
                    occupied_targets=[vert_ref(v) for v in sorted(occupied_targets, key=lambda vert: vert.index)],
                    blocked_targets=[vert_ref(v) for v in sorted(blocked_targets, key=lambda vert: vert.index)],
                )
                migrated = _migrate_seam_vert(bm, seam_vert, best, ring_group.all_ring_verts)
                if migrated:
                    ring_info.seam_verts.discard(seam_vert)
                    ring_info.seam_verts.add(best)
                    seam_homes[best] = home
                    seam_homes.pop(seam_vert, None)
                    reserved_targets.add(best)
                else:
                    debug_log(
                        "seam_migrate",
                        "Rejected seam target because it was occupied or would create an overlong support edge.",
                        old_seam=vert_ref(seam_vert),
                        rejected_target=vert_ref(best),
                    )
                    reserved_targets.add(seam_vert)
            else:
                debug_log(
                    "seam_migrate",
                    "Kept seam in place because no better local normal target was available.",
                    seam=vert_ref(seam_vert),
                    drift=round(drift, 6),
                    crowded=is_crowded,
                    current_spacing=current_spacing,
                    current_perpendicular_error=round(current_perpendicular_error, 6),
                    clearance_steps=clearance_steps,
                    horizon_steps=horizon_steps,
                    home_index=home_indices.get(seam_vert),
                    best_candidate=vert_ref(best),
                    best_score=best_score,
                    best_perpendicular_error=(
                        round(best_perpendicular_error, 6)
                        if best_perpendicular_error is not None
                        else None
                    ),
                    occupied_targets=[vert_ref(v) for v in sorted(occupied_targets, key=lambda vert: vert.index)],
                    blocked_targets=[vert_ref(v) for v in sorted(blocked_targets, key=lambda vert: vert.index)],
                )
                reserved_targets.add(seam_vert)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
