# vert_deletion.py — Smart vert selection for deletion.
# Picks non-seam verts so seam anchors are never destroyed.
# Works with arbitrary seam counts — just needs a RingGroup with
# seam_verts identified per ring.

import bmesh

from .debug import debug_log, pair_ref, vert_ref


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

    if ring_group.is_closed:
        # Start at 1 for backwards compat with the old loop[1] deletion
        candidate_indices = ((1 + offset) % n for offset in range(n))
    else:
        candidate_indices = range(1, n - 1)

    for i in candidate_indices:
        safe = True
        blocked_by = []
        for ring_info in ring_group.rings:
            if ring_info.verts[i] in ring_info.seam_verts:
                safe = False
                blocked_by.append(vert_ref(ring_info.verts[i]))
                break
        if safe:
            debug_log(
                "delete_pick",
                "Selected safe deletion index.",
                index=i,
                candidate_verts=[vert_ref(ring_info.verts[i]) for ring_info in ring_group.rings],
            )
            return i
        debug_log(
            "delete_pick",
            "Rejected deletion index because it hits a seam vert.",
            index=i,
            blocked_by=blocked_by,
        )
    debug_log(
        "delete_pick",
        "No safe deletion index found.",
        vert_count=ring_group.vert_count,
    )
    return -1


def delete_at_index(bm, ring_group, index):
    """Delete the vert at `index` from every ring.

    Dissolves the verts, updates lookup tables, pops the killed verts
    from each ring's vert list, and returns neighbor pairs for topology repair.
    """
    n = ring_group.vert_count
    v_kills = []
    neighbor_pairs = []
    repair_contexts = []

    for ring_info in ring_group.rings:
        verts = ring_info.verts
        v_kill = verts[index]
        v_kills.append(v_kill)

        if ring_info.is_closed:
            prev_v = verts[(index - 1) % n]
            next_v = verts[(index + 1) % n]
        else:
            prev_v = verts[max(0, index - 1)]
            next_v = verts[min(n - 1, index + 1)]
        neighbor_pairs.append((prev_v, next_v))

        debug_log(
            "delete_step",
            "Prepared ring vert for dissolve.",
            index=index,
            kill_vert=vert_ref(v_kill),
            prev_vert=vert_ref(prev_v),
            next_vert=vert_ref(next_v),
            kill_faces=[face.index for face in v_kill.link_faces if face.is_valid],
        )

    for ring_index, (ring_info, v_kill, pair) in enumerate(zip(ring_group.rings, v_kills, neighbor_pairs)):
        prev_v, next_v = pair
        other_kills = {other for other in v_kills if other is not v_kill}
        shell_candidates = [
            face for face in v_kill.link_faces
            if face.is_valid and all(other not in face.verts for other in other_kills)
        ]
        shell_face = max(shell_candidates, key=lambda face: len(face.verts), default=None)

        context = {
            "kind": "delete_pair",
            "ring_index": ring_index,
            "a": prev_v,
            "b": next_v,
            "kill_vert": v_kill,
            "shell_face": shell_face,
            "shell_hint_verts": [
                vert for vert in (shell_face.verts if shell_face is not None else [])
                if vert.is_valid and vert is not v_kill
            ],
            "all_pairs": neighbor_pairs,
        }
        repair_contexts.append(context)

        debug_log(
            "delete_step",
            "Captured local repair context before dissolve.",
            ring_index=ring_index,
            pair=pair_ref(prev_v, next_v),
            shell_face=shell_face.index if shell_face is not None else None,
            shell_hint_verts=[vert_ref(vert) for vert in context["shell_hint_verts"]],
        )

    valid_kills = [v for v in v_kills if v.is_valid]
    if valid_kills:
        debug_log(
            "delete_step",
            "Running dissolve_verts.",
            kill_verts=[vert_ref(v) for v in valid_kills],
            neighbor_pairs=[pair_ref(a, b) for a, b in neighbor_pairs],
        )
        bmesh.ops.dissolve_verts(bm, verts=valid_kills)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()

    debug_log(
        "delete_step",
        "Finished dissolve_verts.",
        surviving_pairs=[
            {
                "pair": pair_ref(a, b),
                "edge_exists": bm.edges.get([a, b]) is not None if a.is_valid and b.is_valid else False,
                "shared_faces": [
                    face.index
                    for face in a.link_faces
                    if a.is_valid and b.is_valid and face.is_valid and b in face.verts
                ] if a.is_valid and b.is_valid else [],
            }
            for a, b in neighbor_pairs
        ],
    )

    # Remove killed verts from tracking
    ring_group.all_ring_verts -= set(v_kills)
    for ring_info in ring_group.rings:
        ring_info.verts.pop(index)

    return repair_contexts
