# hole_punch_face.py — Detect punched holes cut into a flat face.

from ..ring_analyzer import analyze_rings
from .detection_utils import get_selected_islands


def _selected_loop_degree(vert, ring_set):
    return sum(
        1
        for edge in vert.link_edges
        if edge.select and edge.other_vert(vert) in ring_set
    )


def detect(bm):
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

        use_seams = len(seam_verts) < len(ring)

        groups.append({
            'rings': ([ring], True),
            'use_seams': use_seams,
            'migrate_seams': True,
            'max_seams': 1,
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
