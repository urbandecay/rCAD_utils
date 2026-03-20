# Lightweight sweep to select candidate edges per vertex in 2D (XY).
# Pure Python logic. The operator will perform geometric checks.

from bisect import bisect_left

VERTICAL_U_EPS = 1e-9


def edge_left_right_by_uv(a2, b2):
    (ua, va) = a2
    (ub, vb) = b2
    if ua < ub: return 0, 1
    if ua > ub: return 1, 0
    if va <= vb: return 0, 1
    return 1, 0


def y_at_x_on_segment(u, a2, b2):
    (ua, va) = a2
    (ub, vb) = b2
    du = (ub - ua)
    if abs(du) < 1e-20:
        return None
    t = (u - ua) / du
    return va + t * (vb - va)


def sweep_candidates_for_vertices(edges2d, verts2d):
    """
    edges2d: list of dicts:
      {
        'idx': int,
        'a2': (u,v), 'b2': (u,v),
        'vertical': bool,
        'umin': float, 'umax': float,
        'vmin': float, 'vmax': float,
      }
    verts2d: list of dicts:
      {
        'idx': int, 'uv': (u,v),
      }
    Returns:
      cand_map: dict v_idx -> [edge_idx,...]
    """
    events_by_u = {}
    def add_event(u, kind, payload):
        d = events_by_u.setdefault(u, {'start': [], 'verts': [], 'end': []})
        d[kind].append(payload)

    vertical_pool = []
    for ed in edges2d:
        add_event(ed['umin'], 'start', ed)
        add_event(ed['umax'], 'end', ed)
        if ed['vertical']:
            vertical_pool.append(ed)

    for vd in verts2d:
        add_event(vd['uv'][0], 'verts', vd)

    u_keys = sorted(events_by_u.keys())
    active = set()
    cand_map = {}

    for u in u_keys:
        bucket = events_by_u[u]

        # Add starts
        for ed in bucket['start']:
            active.add(ed['idx'])

        # Build status of non-vertical
        status = []
        for ed in list(active):
            edinfo = next((e for e in edges2d if e['idx'] == ed), None)
            if not edinfo:
                active.discard(ed)
                continue
            if edinfo['vertical']:
                continue
            if u < edinfo['umin'] - 1e-20 or u > edinfo['umax'] + 1e-20:
                continue
            vpos = y_at_x_on_segment(u, edinfo['a2'], edinfo['b2'])
            if vpos is None:
                continue
            status.append((vpos, ed))
        status.sort(key=lambda t: t[0])
        status_v = [t[0] for t in status]

        # Process vertices at u
        for vd in bucket['verts']:
            v_idx = vd['idx']
            u_v, w_v = vd['uv']
            cands = []
            if status:
                idx = bisect_left(status_v, w_v)
                for delta in (-2, -1, 0, 1, 2):
                    j = idx + delta
                    if 0 <= j < len(status):
                        cands.append(status[j][1])

            # Vertical aligned edges at same u
            for ed in vertical_pool:
                if abs(ed['a2'][0] - u_v) <= VERTICAL_U_EPS:
                    if (w_v >= ed['vmin'] - 1e-12) and (w_v <= ed['vmax'] + 1e-12):
                        cands.append(ed['idx'])

            # Dedup
            seen = set()
            uniq = []
            for eidx in cands:
                if eidx not in seen:
                    uniq.append(eidx)
                    seen.add(eidx)
            cand_map[v_idx] = uniq

        # Remove ends
        for ed in bucket['end']:
            if ed['idx'] in active:
                active.remove(ed['idx'])

    return cand_map
