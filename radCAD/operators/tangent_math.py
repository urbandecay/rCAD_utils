import math
from mathutils import Vector

def fit_circle_3pt(p1, p2, p3):
    """
    Returns (Center, Radius) for a circle defined by 3 points.
    Used by both tools to convert curved edges into circle data.
    """
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(D) < 1e-9: return None, None
    Ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / D
    Uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / D
    c = Vector((Ux, Uy, 0))
    return c, (c - p1).length

def _solve_core(c1, c2, c3, s1, s2, s3):
    """Internal algebraic solver."""
    try:
        x1, y1, r1 = c1[0], c1[1], c1[2] * s1
        x2, y2, r2 = c2[0], c2[1], c2[2] * s2
        x3, y3, r3 = c3[0], c3[1], c3[2] * s3

        v11, v12 = 2*x2 - 2*x1, 2*y2 - 2*y1
        v13 = x1**2 - x2**2 + y1**2 - y2**2 - r1**2 + r2**2
        v14, v21, v22 = 2*r2 - 2*r1, 2*x3 - 2*x2, 2*y3 - 2*y2
        v23 = x2**2 - x3**2 + y2**2 - y3**2 - r2**2 + r3**2
        v24 = 2*r3 - 2*r2
        
        # Check for singularities
        if abs(v11) < 1e-7 and abs(v12) < 1e-7: return None

        # Gaussian elimination
        if abs(v11) > 1e-9:
            w12, w13, w14 = v12/v11, v13/v11, v14/v11
            w22 = v22 - v21*w12
            w23, w24 = v23 - v21*w13, v24 - v21*w14
            if abs(w22) < 1e-7: return None
            P, Q = -w23/w22, w24/w22
            M, N = -w12*P - w13, w14 - w12*Q
        else:
            # Fallback if v11 is near 0 but v12 is stable
            if abs(v12) < 1e-9: return None
            # v12*y = v14*r - v13  =>  y = (v14/v12)*r - (v13/v12)
            P_val = (v14 - 2*r1)/v12  # Simplification, usually safer to Rotate
            return None 

        a = N**2 + Q**2 - 1
        b = 2*M*N - 2*N*x1 + 2*P*Q - 2*Q*y1 + 2*r1
        c_val = x1**2 + M**2 - 2*M*x1 + P**2 + y1**2 - 2*P*y1 - r1**2
        
        D = b**2 - 4*a*c_val
        if D < 0: return None
        rs = (-b - math.sqrt(D)) / (2*a)
        if rs < 0: rs = (-b + math.sqrt(D)) / (2*a)
        return Vector((M + N*rs, P + Q*rs, 0)), abs(rs)
    except: return None

def solve_apollonius_robust(c1, c2, c3, s1, s2, s3):
    """Standard solve with a 90-degree rotation fallback for singularities."""
    res = _solve_core(c1, c2, c3, s1, s2, s3)
    if res: return res
    def rot(c): return (-c[1], c[0], c[2])
    res_rot = _solve_core(rot(c1), rot(c2), rot(c3), s1, s2, s3)
    if res_rot: return Vector((res_rot[0].y, -res_rot[0].x, 0)), res_rot[1]
    return None