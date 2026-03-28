# hole_in_mesh.py — Resample cylinders punched through flat mesh.

from .closed_loop import _execute_bridged_logic


def _fully_selected_faces(edge, sel_set):
    return sum(1 for f in edge.link_faces if all(v in sel_set for v in f.verts))


def get_rings_from_selected(bm):
    sel_verts = [v for v in bm.verts if v.select]
    if len(sel_verts) < 4:
        return None
    sel_set = set(sel_verts)

    ring_adj = {v: [] for v in sel_verts}
    for v in sel_verts:
        for e in v.link_edges:
            other = e.other_vert(v)
            if other in sel_set and _fully_selected_faces(e, sel_set) == 1:
                ring_adj[v].append(other)

    visited = set()
    rings = []
    for v in sel_verts:
        if v in visited or not ring_adj[v]:
            continue
        comp = []
        stack = [v]
        visited.add(v)
        while stack:
            curr = stack.pop()
            comp.append(curr)
            for nb in ring_adj[curr]:
                if nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
        if len(comp) >= 3:
            rings.append(comp)

    if len(rings) < 2 or len(rings) % 2 != 0:
        return None

    ring_sets = [set(r) for r in rings]
    paired = [False] * len(rings)
    cylinder_pairs = []

    for i in range(len(rings)):
        if paired[i]:
            continue
        partner = None
        for j in range(i + 1, len(rings)):
            if paired[j]:
                continue
            for v in rings[i]:
                for e in v.link_edges:
                    if _fully_selected_faces(e, sel_set) == 2:
                        if e.other_vert(v) in ring_sets[j]:
                            partner = j
                            break
                if partner is not None:
                    break
            if partner is not None:
                break

        if partner is None:
            return None

        ring0_set = ring_sets[i]
        ring0_ordered = [rings[i][0]]
        vis = {rings[i][0]}
        curr = rings[i][0]
        for _ in range(len(rings[i]) - 1):
            for nb in ring_adj[curr]:
                if nb in ring0_set and nb not in vis:
                    ring0_ordered.append(nb)
                    vis.add(nb)
                    curr = nb
                    break

        ring1_set = ring_sets[partner]
        ring1_aligned = []
        for v0 in ring0_ordered:
            for e in v0.link_edges:
                if _fully_selected_faces(e, sel_set) == 2:
                    other = e.other_vert(v0)
                    if other in ring1_set:
                        ring1_aligned.append(other)
                        break

        if len(ring1_aligned) != len(ring0_ordered):
            return None

        cylinder_pairs.append(([ring0_ordered, ring1_aligned], True))
        paired[i] = True
        paired[partner] = True

    return cylinder_pairs if cylinder_pairs else None


def detect(bm):
    rings_list = get_rings_from_selected(bm)
    return rings_list if rings_list else None


def execute(bm, obj, direction, report=None):
    data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for rings_data in data:
        _execute_bridged_logic(bm, obj, rings_data, direction, report=report)
    return {'FINISHED'}
