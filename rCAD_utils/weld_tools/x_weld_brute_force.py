from mathutils import Vector
from .utils import aabb_overlap_3d, round_key_3d, closest_points_on_segments

# Geometry tolerances
EPS = 1e-12
T_EPS = 1e-9        # strict interior check (avoid endpoints)
P_ROUND_3D = 9      # key precision for grouping identical hits
PARALLEL_COS_3D = 0.9995  # treat near-parallel segments as non-X


def _length_sq(v: Vector) -> float:
    return v.x * v.x + v.y * v.y + v.z * v.z


def find_brute_force_x_intersections(segments, merge_radius, plane=None):
    """
    3D X-crossing finder (plane-agnostic).

    segments: list of dicts:
      {
        'sid': int,
        'p0w': Vector (world),
        'p1w': Vector (world),
      }
    merge_radius: float, used for broad-phase AABB slack and X proximity threshold
    plane: kept for API compatibility; ignored (stats["plane"] = "3D")

    Returns:
      per_seg_cuts: dict sid -> list of (t_param, key3d)
      all_keys: set of 3D rounded keys
      stats: dict of counters (includes plane="3D")
      key_world: dict key3d -> Vector (world) representative intersection position
    """
    n = len(segments)
    per_seg_cuts = {s['sid']: [] for s in segments}
    per_seg_seen_keys = {s['sid']: set() for s in segments}
    all_keys = set()
    key_world_sum = {}
    key_world_count = {}

    stats = {
        "pairs_total": n * (n - 1) // 2,
        "pairs_aabb_skipped": 0,
        "pairs_tested": 0,
        "hits": 0,
        "plane": "3D",
        "t_filtered": 0,  # candidate X suppressed as T via endpoint proximity
    }

    slack = max(merge_radius, 0.0)
    r2 = float(max(merge_radius, 0.0)) ** 2

    for i in range(n - 1):
        si = segments[i]
        p1a = si['p0w']; p1b = si['p1w']

        # Degenerate segment guard
        if (p1a - p1b).length_squared <= EPS:
            continue

        min_i = Vector((min(p1a.x, p1b.x), min(p1a.y, p1b.y), min(p1a.z, p1b.z)))
        max_i = Vector((max(p1a.x, p1b.x), max(p1a.y, p1b.y), max(p1a.z, p1b.z)))

        for j in range(i + 1, n):
            sj = segments[j]
            p2a = sj['p0w']; p2b = sj['p1w']

            if (p2a - p2b).length_squared <= EPS:
                continue

            min_j = Vector((min(p2a.x, p2b.x), min(p2a.y, p2b.y), min(p2a.z, p2b.z)))
            max_j = Vector((max(p2a.x, p2b.x), max(p2a.y, p2b.y), max(p2a.z, p2b.z)))

            # Broad-phase AABB overlap with slack
            if not aabb_overlap_3d(min_i, max_i, min_j, max_j, slack=slack):
                stats["pairs_aabb_skipped"] += 1
                continue

            stats["pairs_tested"] += 1

            # Reject near-parallel pairs to avoid false X on side-by-side lines
            u = p1b - p1a
            v = p2b - p2a
            Lu2 = _length_sq(u)
            Lv2 = _length_sq(v)
            if Lu2 <= EPS or Lv2 <= EPS:
                continue
            du = u / (Lu2 ** 0.5)
            dv = v / (Lv2 ** 0.5)
            if abs(du.dot(dv)) >= PARALLEL_COS_3D:
                continue

            # Exact 3D closest points and params on each segment
            c1, c2, s, t = closest_points_on_segments(p1a, p1b, p2a, p2b)
            d2 = (c1 - c2).length_squared

            # Must be within merge radius and strictly interior on both segments
            if d2 > r2 + 1e-24:
                continue
            if (s <= T_EPS) or (s >= 1.0 - T_EPS) or (t <= T_EPS) or (t >= 1.0 - T_EPS):
                continue

            # Candidate X position (midpoint of closest points)
            p_world = (c1 + c2) * 0.5

            # T-junction filter: if within radius of any endpoint, treat as T, not X
            if merge_radius > 0.0:
                near_endpoint = False
                for ep in (p1a, p1b, p2a, p2b):
                    if (p_world - ep).length_squared <= r2:
                        near_endpoint = True
                        break
                if near_endpoint:
                    stats["t_filtered"] += 1
                    continue

            # Group by rounded key to merge numerically identical hits
            key = round_key_3d(p_world, P_ROUND_3D)

            # Record params once per segment per key
            if key not in per_seg_seen_keys[si['sid']]:
                per_seg_seen_keys[si['sid']].add(key)
                per_seg_cuts[si['sid']].append((float(s), key))
            if key not in per_seg_seen_keys[sj['sid']]:
                per_seg_seen_keys[sj['sid']].add(key)
                per_seg_cuts[sj['sid']].append((float(t), key))
            all_keys.add(key)

            # Accumulate robust representative point per key
            if key not in key_world_sum:
                key_world_sum[key] = Vector(p_world)
                key_world_count[key] = 1
            else:
                key_world_sum[key] += p_world
                key_world_count[key] += 1

            stats["hits"] += 1

    key_world = {k: (key_world_sum[k] / float(max(1, key_world_count[k]))) for k in all_keys}
    return per_seg_cuts, all_keys, stats, key_world