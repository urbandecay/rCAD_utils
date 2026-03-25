import bpy
import bmesh
from math import isfinite
from mathutils import Vector
from bpy_extras import view3d_utils

# --- Deferred mesh update mechanism ---
# When True, operators skip bmesh.update_edit_mesh() calls.
# The orchestrator (execute_weld) sets this and does ONE update at the end.
_defer_mesh_update = False

def set_defer_mesh_update(state: bool):
    global _defer_mesh_update
    _defer_mesh_update = state

def deferred_update_edit_mesh(mesh, **kwargs):
    """Call instead of bmesh.update_edit_mesh(). Skips if deferred."""
    if _defer_mesh_update:
        return
    bmesh.update_edit_mesh(mesh, **kwargs)

# Geometry tolerances
EPS = 1e-12
LJUNC_T_EPS = 1e-6
COLINEAR_COS_2D = 0.9995  # ~1.8 deg

# Welder tag layer (General Contractor system)
WELDER_TAG_LAYER = "sf_welder_tag"

# Toggle GC debug
GC_DEBUG = True
def _gc_log(msg):
    if GC_DEBUG:
        try:
            print(f"[Super Fuse][GC] {msg}")
        except Exception:
            pass

# Tag codes
TAG_NONE = 0          # Default, not involved
TAG_L_WELD = 1        # Surviving anchor of L-weld cluster
TAG_T_ANCHOR = 2      # Stationary anchor in T (endpoint or new split)
TAG_T_MOVER = 3       # Moved vertex in T
TAG_X_WELD = 4        # Central vertex created at X crossing
TAG_DOT_WELD = 5      # Final anchor vertex from verts->edges merge

def ensure_welder_tag_layer(bm: bmesh.types.BMesh):
    layer = bm.verts.layers.int.get(WELDER_TAG_LAYER)
    if layer is None:
        layer = bm.verts.layers.int.new(WELDER_TAG_LAYER)
        _gc_log(f"Created welder tag layer: {WELDER_TAG_LAYER}")
    return layer

def tag_involved_verts(bm: bmesh.types.BMesh, verts_to_tag, tag_code: int):
    if not verts_to_tag:
        _gc_log("tag_involved_verts: empty verts_to_tag; nothing to tag.")
        return
    layer = ensure_welder_tag_layer(bm)
    code = int(tag_code)
    count = 0
    for v in verts_to_tag:
        if v and getattr(v, "is_valid", False):
            try:
                v[layer] = code
                count += 1
            except Exception:
                pass
    _gc_log(f"tag_involved_verts: wrote tag={code} to {count} verts")

def clear_tags_on_verts(bm: bmesh.types.BMesh, verts):
    if not verts:
        _gc_log("clear_tags_on_verts: empty input; nothing cleared.")
        return
    layer = ensure_welder_tag_layer(bm)
    count = 0
    for v in verts:
        if v and getattr(v, "is_valid", False):
            try:
                v[layer] = int(TAG_NONE)
                count += 1
            except Exception:
                pass
    _gc_log(f"clear_tags_on_verts: cleared to TAG_NONE on {count} verts")

def read_vert_tag(bm: bmesh.types.BMesh, v):
    if not (v and getattr(v, "is_valid", False)):
        return TAG_NONE
    layer = bm.verts.layers.int.get(WELDER_TAG_LAYER)
    if layer is None:
        return TAG_NONE
    try:
        return int(v[layer])
    except Exception:
        return TAG_NONE


# 3D geometry helpers
def closest_point_on_segment(a: Vector, b: Vector, p: Vector):
    ab = b - a
    ab_len2 = ab.length_squared
    if ab_len2 <= EPS:
        return a.copy(), 0.0
    t = (p - a).dot(ab) / ab_len2
    if t < 0.0: t = 0.0
    elif t > 1.0: t = 1.0
    c = a + t * ab
    return c, float(t)


def closest_points_on_segments(p1, q1, p2, q2):
    u = q1 - p1
    v = q2 - p2
    w0 = p1 - p2
    a = u.dot(u)
    b = u.dot(v)
    c = v.dot(v)
    d = u.dot(w0)
    e = v.dot(w0)
    denom = a * c - b * b

    if a <= EPS and c <= EPS:
        s = t = 0.0
        return p1, p2, s, t
    if a <= EPS:
        s = 0.0
        t = max(0.0, min(1.0, e / c if c > EPS else 0.0))
    elif c <= EPS:
        t = 0.0
        s = max(0.0, min(1.0, -d / a if a > EPS else 0.0))
    else:
        if abs(denom) > 1e-18:
            s = (b * e - c * d) / denom
            t = (a * e - b * d) / denom
        else:
            s = 0.0
            t = (b * s - e) / (c if c > EPS else 1.0)
        s = max(0.0, min(1.0, s))
        t = max(0.0, min(1.0, t))
        if s == 0.0 or s == 1.0:
            t = max(0.0, min(1.0, (b * s - e) / (c if c > EPS else 1.0)))
        if t == 0.0 or t == 1.0:
            s = max(0.0, min(1.0, (b * t + d) / (a if a > EPS else 1.0)))
    c1 = p1 + u * s
    c2 = p2 + v * t
    return c1, c2, float(s), float(t)


def aabb_overlap_3d(a_min, a_max, b_min, b_max, slack=0.0):
    return (
        (a_min.x - slack <= b_max.x and a_max.x + slack >= b_min.x) and
        (a_min.y - slack <= b_max.y and a_max.y + slack >= b_min.y) and
        (a_min.z - slack <= b_max.z and a_max.z + slack >= b_min.z)
    )


# 2D helpers
def proj2d(v: Vector, plane: str = 'XY'):
    if plane == 'XY': return (v.x, v.y)
    if plane == 'YZ': return (v.y, v.z)
    if plane == 'ZX': return (v.z, v.x)
    return (v.x, v.y)


def edge_proj2d(e, plane: str = 'XY'):
    a3 = e.verts[0].co
    b3 = e.verts[1].co
    a2 = proj2d(a3, plane)
    b2 = proj2d(b3, plane)
    return a2, b2, a3, b3


def edge_left_right_by_uv(a2, b2):
    (ua, va) = a2
    (ub, vb) = b2
    if ua < ub: return 0, 1
    if ua > ub: return 1, 0
    if va <= vb: return 0, 1
    return 1, 0


def project_point_to_segment_3d(p: Vector, a: Vector, b: Vector):
    ab = b - a
    ab_len2 = ab.length_squared
    if ab_len2 <= EPS:
        return 0.0, a.copy(), (p - a).length
    t = (p - a).dot(ab) / ab_len2
    if t < 0.0: t = 0.0
    elif t > 1.0: t = 1.0
    pro = a + t * ab
    return float(t), pro, (p - pro).length


def dir2d_edge_from_vertex(e, v_from, plane: str = 'XY'):
    v0, v1 = e.verts
    other = v1 if v0 == v_from else v0
    a2 = proj2d(v_from.co, plane)
    b2 = proj2d(other.co, plane)
    dv = Vector((b2[0] - a2[0], b2[1] - a2[1]))
    L = dv.length
    if L < 1e-20:
        return None
    return dv / L


def is_L_endpoint_junction(e, endpoint_vert, plane: str = 'XY', colinear_cos=COLINEAR_COS_2D):
    if not endpoint_vert or not endpoint_vert.is_valid:
        return False
    d_base = dir2d_edge_from_vertex(e, endpoint_vert, plane)
    if d_base is None:
        return False
    for e2 in endpoint_vert.link_edges:
        if e2 == e or not e2.is_valid:
            continue
        d2 = dir2d_edge_from_vertex(e2, endpoint_vert, plane)
        if d2 is None:
            continue
        if abs(d_base.dot(d2)) < float(colinear_cos):
            return True
    return False


def is_geometric_L_at_point(point3: Vector, e, endpoint_vert, plane: str,
                            neighbor_edges, pos_eps: float, colinear_cos=COLINEAR_COS_2D):
    d_base = dir2d_edge_from_vertex(e, endpoint_vert, plane)
    if d_base is None:
        return False
    p = point3
    for e2 in neighbor_edges:
        if not e2 or not e2.is_valid or e2 == e:
            continue
        vA, vB = e2.verts
        dA = (vA.co - p).length
        dB = (vB.co - p).length
        near = None
        far = None
        if dA <= pos_eps and dB <= pos_eps:
            continue
        elif dA <= pos_eps:
            near, far = vA, vB
        elif dB <= pos_eps:
            near, far = vB, vA
        else:
            continue

        a2 = proj2d(near.co, plane)
        b2 = proj2d(far.co, plane)
        dv = Vector((b2[0] - a2[0], b2[1] - a2[1]))
        L = dv.length
        if L < 1e-20:
            continue
        d2 = dv / L
        if abs(d_base.dot(d2)) < float(colinear_cos):
            return True
    return False


# Blender view helpers
def get_active_window_region_and_rv3d(context):
    area = context.area
    if not area or area.type != 'VIEW_3D':
        return None, None
    region_window = None
    for r in area.regions:
        if r.type == 'WINDOW':
            region_window = r
            break
    rv3d = context.space_data.region_3d if context.space_data and context.space_data.type == 'VIEW_3D' else None
    return region_window, rv3d


def project_world_to_region2d(region, rv3d, world_co):
    if region is None or rv3d is None:
        return None
    p = view3d_utils.location_3d_to_region_2d(region, rv3d, world_co)
    if p is None:
        return None
    return Vector((float(p.x), float(p.y)))


# BMesh helpers
def edge_between(a, b):
    for e2 in a.link_edges:
        if b in e2.verts:
            return e2
    return None


def safe_edge_split_pair(bm, edge, split_from_vert, fac):
    # Robustly return (BMVert, BMEdge) if possible
    fac = float(fac)
    res = bmesh.utils.edge_split(edge, split_from_vert, fac)
    if isinstance(res, tuple) and len(res) == 2:
        a, b = res
        if isinstance(a, bmesh.types.BMVert) and isinstance(b, bmesh.types.BMEdge):
            return a, b
        if isinstance(a, bmesh.types.BMEdge) and isinstance(b, bmesh.types.BMVert):
            return b, a
    if isinstance(res, bmesh.types.BMVert):
        v = res
        for e2 in v.link_edges:
            if (edge.verts[0] in e2.verts) or (edge.verts[1] in e2.verts):
                return v, e2
        if v.link_edges:
            return v, v.link_edges[0]
        return v, None
    if isinstance(res, bmesh.types.BMEdge):
        for vv in res.verts:
            if vv not in edge.verts:
                return vv, res
        # As a fallback, try to deduce a new vertex attached to res
        for e2 in res.verts[0].link_edges:
            if e2 is res:
                continue
            for vv in e2.verts:
                if vv not in edge.verts:
                    return vv, res
        return None, res

    # Last resort: subdivide
    ret = bmesh.ops.subdivide_edges(
        bm,
        edges=[edge],
        cuts=1,
        use_grid_fill=False,
        edge_percents={edge: fac}
    )
    new_candidates = [ele for ele in ret.get("geom_split", []) if isinstance(ele, bmesh.types.BMVert)]
    v = new_candidates[-1] if new_candidates else None
    # Guess edge after split
    if v:
        e_after = edge_between(v, edge.verts[1])
        return v, e_after
    return None, None


def safe_edge_split_vert_only(bm, edge, split_from_vert, fac):
    v, _ = safe_edge_split_pair(bm, edge, split_from_vert, fac)
    return v


def round_key_3d(p: Vector, decimals=6):
    return (round(float(p.x), decimals), round(float(p.y), decimals), round(float(p.z), decimals))