# ring_analyzer.py — Ring topology analysis for vertex resampler.
# Identifies ring structure, seam verts, and geometry type.
# Geometry types (cylinder, tee, corner) are just labels — all downstream
# logic (deletion, repair, seam mgmt) works the same regardless of type.


class RingInfo:
    """Single ring with ordered verts and identified seam positions."""
    __slots__ = ('verts', 'seam_verts', 'is_closed')

    def __init__(self, verts, seam_verts, is_closed=True):
        self.verts = verts
        self.seam_verts = seam_verts  # set of BMVerts
        self.is_closed = is_closed


class RingGroup:
    """Group of related rings (cylinder pair, tee junction, etc).

    Adding a new geometry type = new ring detection function that returns
    a RingGroup with the right alignment and geometry_type label.
    The rest of the pipeline (deletion, repair, seams) doesn't change.
    """
    __slots__ = ('rings', 'geometry_type', 'all_ring_verts')

    def __init__(self, rings, geometry_type='cylinder'):
        self.rings = rings
        self.geometry_type = geometry_type
        self.all_ring_verts = set()
        for r in rings:
            self.all_ring_verts.update(r.verts)

    @property
    def is_closed(self):
        return self.rings[0].is_closed if self.rings else True

    @property
    def loops(self):
        return [r.verts for r in self.rings]

    @property
    def vert_count(self):
        return len(self.rings[0].verts) if self.rings else 0


def analyze_rings(loops, is_closed=True):
    """Build a RingGroup from aligned loops.

    Detects seam verts per ring — any ring vert with at least one edge
    going to a non-ring vert is a seam anchor. Works for any number of
    seams (2 for cylinders, 3+ for tees/corners).
    """
    all_ring_verts = set()
    for loop in loops:
        all_ring_verts.update(loop)

    rings = []
    for loop in loops:
        seam_verts = set()
        for v in loop:
            for e in v.link_edges:
                if e.other_vert(v) not in all_ring_verts:
                    seam_verts.add(v)
                    break
        rings.append(RingInfo(loop, seam_verts, is_closed))

    return RingGroup(rings)
