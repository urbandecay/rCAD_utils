# vert_insertion.py — Smart vert insertion for adding.
# Splits ring edges at a chosen index across all rings.
# Mirror of vert_deletion.py for the add operation.

import bmesh


def find_safe_insertion_index(ring_group):
    """Find an edge index safe to split across ALL rings.

    Returns index i — the edge between verts[i] and verts[(i+1) % n]
    will be split. Any position is geometrically safe for insertion;
    starts at 1 to match the deletion convention.

    Returns -1 if the ring is too small to split.
    """
    if ring_group.vert_count < 3:
        return -1
    return 1


def insert_at_index(bm, ring_group, index):
    """Insert a new vert between index and (index+1)%n on every ring.

    Splits the ring edge via edge_split, inserts the new vert into
    each ring's vert list, updates tracking, and returns consecutive
    new-vert pairs for shaft face repair.
    """
    n = ring_group.vert_count
    is_closed = ring_group.is_closed
    next_idx = (index + 1) % n if is_closed else min(index + 1, n - 1)

    new_verts = []

    for ring_info in ring_group.rings:
        verts = ring_info.verts
        v1 = verts[index]
        v2 = verts[next_idx]

        e = bm.edges.get([v1, v2])
        if e is None:
            return []

        result = bmesh.utils.edge_split(e, v1, 0.5)
        new_v = (
            result[0]
            if isinstance(result[0], bmesh.types.BMVert)
            else result[1]
        )
        new_verts.append(new_v)

        # Insert into the ring's vert list right after index
        verts.insert(index + 1, new_v)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    # Update ring group tracking
    ring_group.all_ring_verts.update(new_verts)

    # Return consecutive pairs for shaft face repair
    repair_pairs = []
    for i in range(len(new_verts) - 1):
        repair_pairs.append((new_verts[i], new_verts[i + 1]))

    return repair_pairs
