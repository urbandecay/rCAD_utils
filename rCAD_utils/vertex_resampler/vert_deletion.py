# vert_deletion.py — Smart vert selection for deletion.
# Picks non-seam verts so seam anchors are never destroyed.
# Works with arbitrary seam counts — just needs a RingGroup with
# seam_verts identified per ring.

import bmesh


def find_safe_deletion_index(ring_group):
    """Find an index that's safe to delete across ALL rings.

    A safe index is one where NO ring has a seam vert at that position.
    Starts searching at index 1 to match historical loop[1] behavior —
    for the common no-seam or 2-seam case this gives identical results.

    Returns the index, or -1 if every position has a seam in some ring.
    """
    n = ring_group.vert_count
    if n == 0:
        return -1

    # Start at 1 for backwards compat with the old loop[1] deletion
    for offset in range(n):
        i = (1 + offset) % n
        safe = True
        for ring_info in ring_group.rings:
            if ring_info.verts[i] in ring_info.seam_verts:
                safe = False
                break
        if safe:
            return i
    return -1


def delete_at_index(bm, ring_group, index):
    """Delete the vert at `index` from every ring.

    Dissolves the verts, updates lookup tables, pops the killed verts
    from each ring's vert list, and returns neighbor pairs for topology repair.
    """
    n = ring_group.vert_count
    v_kills = []
    neighbor_pairs = []

    for ring_info in ring_group.rings:
        verts = ring_info.verts
        v_kills.append(verts[index])

        if ring_info.is_closed:
            prev_v = verts[(index - 1) % n]
            next_v = verts[(index + 1) % n]
        else:
            prev_v = verts[max(0, index - 1)]
            next_v = verts[min(n - 1, index + 1)]
        neighbor_pairs.append((prev_v, next_v))

    valid_kills = [v for v in v_kills if v.is_valid]
    if valid_kills:
        bmesh.ops.dissolve_verts(bm, verts=valid_kills)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    # Remove killed verts from tracking
    ring_group.all_ring_verts -= set(v_kills)
    for ring_info in ring_group.rings:
        ring_info.verts.pop(index)

    return neighbor_pairs
