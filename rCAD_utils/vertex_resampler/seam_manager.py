# seam_manager.py — Seam home persistence, drift detection, and migration.
# Seam homes are stored per-object as a flat float list in obj["rcad_seam_origins"].
# Each seam vert's ORIGINAL position is preserved across presses so drift
# accumulates against the true origin, not last-press position.

import bmesh
from mathutils import Vector, kdtree


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


def load_face_seam_anchors(obj):
    """Load persistent face-hole seam anchors from object custom property."""
    stored_flat = list(obj.get("rcad_face_seam_anchors", []))
    anchors = []
    if stored_flat and len(stored_flat) % 6 == 0:
        for i in range(0, len(stored_flat), 6):
            ring_home = Vector((stored_flat[i], stored_flat[i + 1], stored_flat[i + 2]))
            outer_home = Vector((stored_flat[i + 3], stored_flat[i + 4], stored_flat[i + 5]))
            anchors.append((ring_home, outer_home))
    return anchors


def save_face_seam_anchors(obj, anchors):
    """Save persistent face-hole seam anchors to object custom property."""
    flat = []
    for ring_home, outer_home in anchors:
        flat.extend([ring_home.x, ring_home.y, ring_home.z])
        flat.extend([outer_home.x, outer_home.y, outer_home.z])
    obj["rcad_face_seam_anchors"] = flat


def match_seam_homes(ring_group, stored_homes, avg_edge_len):
    """Match current seam verts to stored home positions via KDTree.

    First press: stores current seam positions as homes.
    Later presses: matches to stored homes by proximity (3x avg edge len).
    Unmatched seam verts get their current position as a new home.

    Returns dict mapping BMVert -> home Vector.
    """
    seam_homes = {}

    if stored_homes:
        kd = kdtree.KDTree(len(stored_homes))
        for idx, hp in enumerate(stored_homes):
            kd.insert(hp, idx)
        kd.balance()

        for ring_info in ring_group.rings:
            for v in ring_info.seam_verts:
                if not v.is_valid:
                    continue
                _co, _idx, _dist = kd.find(v.co)
                if _dist < avg_edge_len * 3.0:
                    seam_homes[v] = stored_homes[_idx]
                else:
                    home = v.co.copy()
                    seam_homes[v] = home
                    stored_homes.append(home)
    else:
        for ring_info in ring_group.rings:
            for v in ring_info.seam_verts:
                if v.is_valid:
                    home = v.co.copy()
                    seam_homes[v] = home
                    stored_homes.append(home)

    return seam_homes


def _create_edge_robust(bm, v1, v2):
    """Create an edge between v1 and v2 using the most reliable method.
    Tries face_split first, then connect_verts, then raw edge creation.
    Returns True if an edge exists between v1 and v2 after this call."""
    if bm.edges.get([v1, v2]):
        return True

    # Try face_split on the largest shared face — most reliable
    shared_face = None
    max_verts = 0
    for f in v1.link_faces:
        if v2 in f.verts and len(f.verts) > max_verts:
            max_verts = len(f.verts)
            shared_face = f

    if shared_face and len(shared_face.verts) >= 4:
        try:
            bmesh.utils.face_split(shared_face, v1, v2)
        except Exception:
            pass

    if bm.edges.get([v1, v2]):
        return True

    # Fall back to connect_verts
    try:
        bmesh.ops.connect_verts(bm, verts=[v1, v2])
    except Exception:
        pass

    if bm.edges.get([v1, v2]):
        return True

    # Last resort: raw edge creation — always works
    try:
        bm.edges.new([v1, v2])
    except Exception:
        pass

    return bm.edges.get([v1, v2]) is not None


def _primary_seam_edge(seam_vert, all_ring_verts):
    seam_edges = [
        edge for edge in seam_vert.link_edges
        if edge.other_vert(seam_vert) not in all_ring_verts
    ]
    if not seam_edges:
        return None
    return max(seam_edges, key=lambda edge: edge.calc_length())


def match_face_seam_anchors(ring_group, stored_anchors, avg_edge_len):
    """Match current face-hole seams to persistent endpoint anchors."""
    seam_anchors = {}
    remaining = list(stored_anchors)

    for ring_info in ring_group.rings:
        for seam_vert in ring_info.seam_verts:
            if not seam_vert.is_valid:
                continue

            seam_edge = _primary_seam_edge(seam_vert, ring_group.all_ring_verts)
            if seam_edge is None:
                continue
            outer_vert = seam_edge.other_vert(seam_vert)

            best_index = -1
            best_score = float('inf')
            for index, (ring_home, outer_home) in enumerate(remaining):
                score = (
                    (outer_vert.co - outer_home).length +
                    (seam_vert.co - ring_home).length
                )
                if score < best_score:
                    best_score = score
                    best_index = index

            if best_index >= 0 and best_score < avg_edge_len * 6.0:
                seam_anchors[seam_vert] = remaining.pop(best_index)
            else:
                anchor = (seam_vert.co.copy(), outer_vert.co.copy())
                seam_anchors[seam_vert] = anchor
                stored_anchors.append(anchor)

    return seam_anchors


def _migrate_seam_vert(bm, old_sv, new_sv, all_ring_verts):
    """Migrate seam edges from old_sv to new_sv.
    Draws a new edge (new_sv -> non-ring vert) then dissolves the old one.
    Returns True if new_sv has a non-ring edge after migration.
    Do NOT use face deletion — just draw + dissolve.
    """
    seam_edges = [e for e in old_sv.link_edges
                  if e.other_vert(old_sv) not in all_ring_verts]
    if not seam_edges:
        return False

    old_edge = max(seam_edges, key=lambda edge: edge.calc_length())
    non_ring_vert = old_edge.other_vert(old_sv)

    created = _create_edge_robust(bm, new_sv, non_ring_vert)

    if created:
        if old_edge.is_valid:
            bmesh.ops.dissolve_edges(bm, edges=[old_edge],
                                     use_verts=False, use_face_split=False)
        return True

    return False


def _migrate_seam_vert_to_target(bm, old_sv, new_sv, target_non_ring_vert, all_ring_verts):
    """Migrate a seam edge to a chosen ring vert and chosen outside endpoint.
    Returns True if new_sv has a non-ring edge after migration."""
    if not (target_non_ring_vert and target_non_ring_vert.is_valid):
        return False

    seam_edges = [
        edge for edge in old_sv.link_edges
        if edge.other_vert(old_sv) not in all_ring_verts
    ]
    if not seam_edges:
        return False

    old_edge = max(seam_edges, key=lambda edge: edge.calc_length())
    created = _create_edge_robust(bm, new_sv, target_non_ring_vert)

    if created:
        if old_edge.is_valid:
            bmesh.ops.dissolve_edges(
                bm,
                edges=[old_edge],
                use_verts=False,
                use_face_split=False,
            )
        return True

    return False


def _ensure_seam_edge(bm, seam_vert, all_ring_verts):
    """Guarantee seam_vert has a non-ring edge. If missing, draw one to the
    nearest non-ring vert reachable via shared faces.
    Returns True if an edge exists after this call."""
    if _primary_seam_edge(seam_vert, all_ring_verts) is not None:
        return True

    # Candidates: non-ring verts sharing a face with seam_vert
    seen = set()
    candidates = []
    for face in seam_vert.link_faces:
        for v in face.verts:
            if v.is_valid and v not in all_ring_verts and v != seam_vert and v not in seen:
                seen.add(v)
                candidates.append(v)

    if not candidates:
        return False

    target = min(candidates, key=lambda v: (v.co - seam_vert.co).length)
    _create_edge_robust(bm, seam_vert, target)

    return _primary_seam_edge(seam_vert, all_ring_verts) is not None


def ensure_all_seam_edges(bm, ring_group):
    """Final safety net — guarantee every tracked seam vert has an edge.
    Call this as the very last step before bmesh.update_edit_mesh."""
    for ring_info in ring_group.rings:
        for sv in list(ring_info.seam_verts):
            if sv.is_valid:
                _ensure_seam_edge(bm, sv, ring_group.all_ring_verts)


def migrate_drifted_seams(bm, ring_group, seam_homes, threshold):
    """Check all seam verts for drift and migrate if needed.

    If a seam vert drifted more than `threshold` from its original home,
    find the nearest ring vert to the home and migrate the seam there.
    Seams that started at different positions never merge — each is
    tracked independently via its own home position.
    """
    for ring_info in ring_group.rings:
        for seam_vert in list(ring_info.seam_verts):
            if not seam_vert.is_valid or seam_vert not in seam_homes:
                continue
            home = seam_homes[seam_vert]
            drift = (seam_vert.co - home).length
            if drift <= threshold:
                continue

            best = None
            best_dist = float('inf')
            for v in ring_info.verts:
                if v.is_valid and v != seam_vert:
                    d = (v.co - home).length
                    if d < best_dist:
                        best_dist = d
                        best = v
            if best:
                migrated = _migrate_seam_vert(bm, seam_vert, best, ring_group.all_ring_verts)
                if migrated:
                    ring_info.seam_verts.discard(seam_vert)
                    ring_info.seam_verts.add(best)

    # Verification pass — guarantee every tracked seam still has an edge.
    # If anything slipped through, _ensure_seam_edge recreates it.
    for ring_info in ring_group.rings:
        for sv in list(ring_info.seam_verts):
            if sv.is_valid:
                _ensure_seam_edge(bm, sv, ring_group.all_ring_verts)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()


def migrate_drifted_face_seams(bm, ring_group, seam_anchors, threshold):
    """Reattach face-hole seams when the ring-side endpoint drifts too far."""
    non_ring_verts = [
        vert for vert in bm.verts
        if vert.is_valid and vert not in ring_group.all_ring_verts
    ]
    kd = None
    if non_ring_verts:
        kd = kdtree.KDTree(len(non_ring_verts))
        for index, vert in enumerate(non_ring_verts):
            kd.insert(vert.co, index)
        kd.balance()

    for ring_info in ring_group.rings:
        for seam_vert in list(ring_info.seam_verts):
            if not seam_vert.is_valid or seam_vert not in seam_anchors:
                continue

            ring_home, outer_home = seam_anchors[seam_vert]
            seam_edge = _primary_seam_edge(seam_vert, ring_group.all_ring_verts)

            if seam_edge is None:
                # Seam edge was lost — recreate it now
                _ensure_seam_edge(bm, seam_vert, ring_group.all_ring_verts)
                continue

            outer_vert = seam_edge.other_vert(seam_vert)
            ring_drift = (seam_vert.co - ring_home).length
            outer_drift = (outer_vert.co - outer_home).length
            if ring_drift <= threshold and outer_drift <= threshold:
                continue

            best_ring_vert = None
            best_ring_dist = float('inf')
            for vert in ring_info.verts:
                if not vert.is_valid or vert == seam_vert:
                    continue
                dist = (vert.co - ring_home).length
                if dist < best_ring_dist:
                    best_ring_dist = dist
                    best_ring_vert = vert

            target_outer_vert = outer_vert
            if kd is not None:
                _co, outer_index, _dist = kd.find(outer_home)
                candidate = non_ring_verts[outer_index]
                if candidate.is_valid:
                    target_outer_vert = candidate

            if best_ring_vert is not None:
                migrated = _migrate_seam_vert_to_target(
                    bm,
                    seam_vert,
                    best_ring_vert,
                    target_outer_vert,
                    ring_group.all_ring_verts,
                )
                if migrated:
                    ring_info.seam_verts.discard(seam_vert)
                    ring_info.seam_verts.add(best_ring_vert)

    # Verification pass — guarantee every tracked seam still has an edge.
    for ring_info in ring_group.rings:
        for sv in list(ring_info.seam_verts):
            if sv.is_valid:
                _ensure_seam_edge(bm, sv, ring_group.all_ring_verts)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
