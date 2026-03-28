# topology_repair.py — Post-dissolve topology repair.
# Restores proper face structure after vert deletion.
# No knowledge of rings or seams — just takes neighbor pairs.

import bmesh


def repair_after_dissolve(bm, neighbor_pairs):
    """Repair topology after dissolve_verts.

    When dissolve_verts removes a ring vert from a hole-in-mesh cylinder,
    it merges the shaft quad + mesh face into one big n-gon — the ring edge
    A-B ends up buried inside it. Splitting the n-gon at (A, B) restores
    the shaft quad and mesh face as separate faces.

    Works for any deletion position — just needs the two neighbor verts
    that were on either side of the deleted vert.
    """
    for a, b in neighbor_pairs:
        if not (a.is_valid and b.is_valid):
            continue
        if bm.edges.get([a, b]) is not None:
            continue  # edge already exists, no repair needed

        # Find the LARGEST face containing both a and b — that's the
        # merged n-gon, not a shaft quad we'd accidentally split.
        merged_face = None
        max_verts = 0
        for f in a.link_faces:
            if b in f.verts and len(f.verts) > max_verts:
                max_verts = len(f.verts)
                merged_face = f

        if merged_face is not None and len(merged_face.verts) > 4:
            bmesh.utils.face_split(merged_face, a, b)
        elif merged_face is None:
            bm.edges.new([a, b])

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
