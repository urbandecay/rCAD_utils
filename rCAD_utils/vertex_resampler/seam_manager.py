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


def _migrate_seam_vert(bm, old_sv, new_sv, all_ring_verts):
    """Migrate seam edges from old_sv to new_sv.
    Draws a new edge (new_sv -> non-ring vert) then dissolves the old one.
    Do NOT use face deletion — just draw + dissolve.
    """
    seam_edges = [e for e in old_sv.link_edges
                  if e.other_vert(old_sv) not in all_ring_verts]
    if not seam_edges:
        return

    for old_edge in seam_edges:
        non_ring_vert = old_edge.other_vert(old_sv)
        if not bm.edges.get([new_sv, non_ring_vert]):
            bmesh.ops.connect_verts(bm, verts=[new_sv, non_ring_vert])
        if old_edge.is_valid:
            bmesh.ops.dissolve_edges(bm, edges=[old_edge],
                                     use_verts=False, use_face_split=False)


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
                _migrate_seam_vert(bm, seam_vert, best, ring_group.all_ring_verts)
                ring_info.seam_verts.discard(seam_vert)
                ring_info.seam_verts.add(best)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
