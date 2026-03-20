import bmesh
import math
from mathutils import Vector, geometry
from mathutils.kdtree import KDTree

EPS = 1e-6
DEBUG_WELD = False  # <--- Shhh. Quiet mode.

def dbg(msg):
    if DEBUG_WELD:
        print(f"[WeldUtils] {msg}")

# --- GEOMETRY HELPERS ---

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
    # Returns c1, c2, s, t
    res = geometry.intersect_line_line(p1, q1, p2, q2)
    if res is None:
        return p1, p2, 0.0, 0.0
    
    c1, c2 = res
    
    vec1 = q1 - p1
    len1 = vec1.length_squared
    s = (c1 - p1).dot(vec1) / len1 if len1 > EPS else 0.0
    
    vec2 = q2 - p2
    len2 = vec2.length_squared
    t = (c2 - p2).dot(vec2) / len2 if len2 > EPS else 0.0
    
    s = max(0.0, min(1.0, s))
    t = max(0.0, min(1.0, t))
    
    c1 = p1 + s * vec1
    c2 = p2 + t * vec2
    
    return c1, c2, s, t

def safe_edge_split_vert_only(bm, edge, split_from_vert, fac):
    fac = float(fac)
    try:
        res = bmesh.utils.edge_split(edge, split_from_vert, fac)
        if isinstance(res, tuple) and len(res) == 2:
            a, b = res
            if isinstance(a, bmesh.types.BMVert): 
                a.select = True
                return a
            if isinstance(b, bmesh.types.BMVert): 
                b.select = True
                return b
    except Exception:
        pass
    return None

# --- SEARCH HELPERS ---

def find_nearby_geometry(bm, arc_verts, radius, mw):
    arc_vert_set = set(arc_verts)
    
    # 1. Verts (KDTree)
    bg_verts = [v for v in bm.verts if v not in arc_vert_set and not v.hide]
    target_verts = []
    
    if bg_verts:
        kd = KDTree(len(bg_verts))
        for i, v in enumerate(bg_verts):
            kd.insert(mw @ v.co, i)
        kd.balance()
        
        found_v_idxs = set()
        for av in arc_verts:
            if not av.is_valid: continue
            p_w = mw @ av.co
            for (co, index, dist) in kd.find_range(p_w, radius * 2.0):
                found_v_idxs.add(index)
        target_verts = [bg_verts[i] for i in found_v_idxs]

    # 2. Edges (AABB)
    min_v = Vector((float('inf'),)*3)
    max_v = Vector((float('-inf'),)*3)
    
    valid_arc = False
    for av in arc_verts:
        if av.is_valid:
            valid_arc = True
            p = mw @ av.co
            min_v.x = min(min_v.x, p.x); min_v.y = min(min_v.y, p.y); min_v.z = min(min_v.z, p.z)
            max_v.x = max(max_v.x, p.x); max_v.y = max(max_v.y, p.y); max_v.z = max(max_v.z, p.z)
            
    target_edges = set()
    if valid_arc:
        margin = max(radius * 2.0, 0.01) 
        min_v -= Vector((margin, margin, margin))
        max_v += Vector((margin, margin, margin))
        
        for e in bm.edges:
            if e.hide: continue
            if e.verts[0] in arc_vert_set and e.verts[1] in arc_vert_set: continue
            
            p1 = mw @ e.verts[0].co
            p2 = mw @ e.verts[1].co
            
            e_min_x = min(p1.x, p2.x); e_max_x = max(p1.x, p2.x)
            if e_max_x < min_v.x or e_min_x > max_v.x: continue
            e_min_y = min(p1.y, p2.y); e_max_y = max(p1.y, p2.y)
            if e_max_y < min_v.y or e_min_y > max_v.y: continue
            e_min_z = min(p1.z, p2.z); e_max_z = max(p1.z, p2.z)
            if e_max_z < min_v.z or e_min_z > max_v.z: continue
            
            target_edges.add(e)
            
    return list(set(target_verts)), list(target_edges)


def perform_heavy_weld(bm, arc_verts, target_geom, radius, mw):
    moves = 0
    tgt_verts, tgt_edges = target_geom
    mw_inv = mw.inverted()
    
    for av in arc_verts:
        if not av.is_valid: continue
        p_w = mw @ av.co
        
        best_dist = radius
        best_pos = None
        
        # Check Verts
        for tv in tgt_verts:
            d = ((mw @ tv.co) - p_w).length
            if d <= best_dist:
                best_dist = d
                best_pos = tv.co
                
        if best_pos:
            av.co = best_pos
            moves += 1
            continue
            
        # Check Edges (Snap to line)
        for te in tgt_edges:
            a_w = mw @ te.verts[0].co
            b_w = mw @ te.verts[1].co
            c_w, t = closest_point_on_segment(a_w, b_w, p_w)
            d = (p_w - c_w).length
            
            if d <= best_dist:
                if 0.01 < t < 0.99:
                    best_dist = d
                    av.co = mw_inv @ c_w
                    moves += 1
                
    return moves

def perform_x_weld(bm, arc_edges, target_edges, radius, mw):
    cuts = 0
    r2 = radius * radius
    mw_inv = mw.inverted()
    
    arc_edges_safe = [e for e in arc_edges if e.is_valid]
    
    for i, ae in enumerate(arc_edges_safe):
        if not ae.is_valid: continue
        p1a = mw @ ae.verts[0].co
        p1b = mw @ ae.verts[1].co
        
        for j, te in enumerate(target_edges):
            if not te.is_valid: continue
            if len(set(ae.verts) & set(te.verts)) > 0: continue
            
            p2a = mw @ te.verts[0].co
            p2b = mw @ te.verts[1].co
            
            c1, c2, s, t = closest_points_on_segments(p1a, p1b, p2a, p2b)
            dist_sq = (c1 - c2).length_squared
            
            if dist_sq < r2:
                # LOGIC UPDATE: Handle T-Junctions (Arc End on Target Middle)
                is_arc_internal = (EPS < s < (1.0 - EPS))
                is_tgt_internal = (EPS < t < (1.0 - EPS))

                if is_tgt_internal:
                    # 1. Split Target Edge
                    v_tgt = safe_edge_split_vert_only(bm, te, te.verts[0], t)
                    if not v_tgt: continue
                    
                    # Calculate world position
                    intersect_w = (c1 + c2) * 0.5
                    intersect_l = mw_inv @ intersect_w
                    v_tgt.co = intersect_l

                    # 2. Handle Arc Edge
                    if is_arc_internal:
                        # X-Crossing: Split Arc Edge too
                        v_arc = safe_edge_split_vert_only(bm, ae, ae.verts[0], s)
                        if v_arc: v_arc.co = intersect_l
                    else:
                        # T-Junction: Arc Edge ends here.
                        # Move the existing endpoint to the cut.
                        vert_idx = 0 if s < 0.5 else 1
                        v_arc_existing = ae.verts[vert_idx]
                        v_arc_existing.co = intersect_l
                    
                    cuts += 1
                    # --- FIX: Removed 'break' to allow the other end of the edge to weld too ---
                        
    return cuts

def perform_self_x_weld(bm, edges, radius, mw):
    """Detects and cuts segments within the SAME drawing that cross each other."""
    cuts = 0
    r2 = radius * radius
    mw_inv = mw.inverted()

    # Use a snapshot of the list to be safe during splits
    edges_list = [e for e in edges if e.is_valid]

    for i in range(len(edges_list)):
        e1 = edges_list[i]
        if not e1.is_valid: continue
        p1a = mw @ e1.verts[0].co
        p1b = mw @ e1.verts[1].co

        for j in range(i + 1, len(edges_list)):
            e2 = edges_list[j]
            if not e2.is_valid: continue

            # Skip connected edges (neighbors)
            if set(e1.verts) & set(e2.verts): continue

            p2a = mw @ e2.verts[0].co
            p2b = mw @ e2.verts[1].co

            c1, c2, s, t = closest_points_on_segments(p1a, p1b, p2a, p2b)
            dist_sq = (c1 - c2).length_squared

            if dist_sq < r2:
                # Intersection point in world space
                intersect_w = (c1 + c2) * 0.5
                intersect_l = mw_inv @ intersect_w

                # Split e2 (Target)
                if 0.001 < t < 0.999:
                    v2 = safe_edge_split_vert_only(bm, e2, e2.verts[0], t)
                    if v2: v2.co = intersect_l

                # Split e1 (Source)
                if 0.001 < s < 0.999:
                    v1 = safe_edge_split_vert_only(bm, e1, e1.verts[0], s)
                    if v1: v1.co = intersect_l

                cuts += 1

    return cuts
