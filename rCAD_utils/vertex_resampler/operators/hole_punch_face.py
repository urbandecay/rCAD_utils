# hole_punch_face.py — Detect punched holes cut into a flat face.

from ..ring_analyzer import analyze_rings
from .detection_utils import get_selected_islands


def _selected_loop_degree(vert, ring_set):
    return sum(
        1
        for edge in vert.link_edges
        if edge.select and edge.other_vert(vert) in ring_set
    )


def detect(bm, report=None):
    del report

    groups = []
    covered_verts = set()

    for island in get_selected_islands(bm):
        if not island['closed']:
            continue

        ring = island['verts']
        if len(ring) < 4:
            continue

        ring_set = set(ring)
        if any(_selected_loop_degree(vert, ring_set) != 2 for vert in ring):
            continue

        ring_group = analyze_rings([ring], is_closed=True)
        seam_verts = ring_group.rings[0].seam_verts
        if not seam_verts:
            continue

        groups.append({
            'rings': ([ring], True),
            # Keep one boundary anchor active while resampling so the face-hole
            # path follows the same edge more like the solid-hole path.
            'use_seams': True,
            'migrate_seams': True,
            'max_seams': 2,
            # Face holes need a tighter redraw window so the seam snaps back
            # before it visibly walks along the border.
            'seam_drift_factor': 0.2,
            # Judge drift from the seam location at the start of this press.
            # Reusing old stored homes is much less stable on flat face holes.
            'persist_seam_homes': False,
            # Keep the loop indexed from the seam so removing verts doesn't
            # rotate the seam around the hole.
            'anchor_to_seams': True,
            # Persist both ends of the seam edge so face holes can redraw the
            # seam back to the same boundary area across repeated presses.
            'use_endpoint_seam_anchors': True,
        })
        covered_verts.update(ring)

    if not groups:
        return None

    selected_verts = {vert for vert in bm.verts if vert.select}
    if covered_verts != selected_verts:
        return None

    return {
        'groups': groups,
        'invalid_components': 0,
    }
