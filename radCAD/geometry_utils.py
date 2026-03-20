import math
from mathutils import Vector
from .plane_utils import plane_to_world

# --- THE MISSING MATH ENGINE ---
class RS_Math_Solver:
    """
    A precision math class translated from LibreCAD's C++ logic.
    Used to find exact intersections for tangency.
    """
    
    TOLERANCE = 1e-12

    @staticmethod
    def quadratic_solver(a, b, c):
        """Solves ax^2 + bx + c = 0"""
        if abs(a) < RS_Math_Solver.TOLERANCE:
            if abs(b) < RS_Math_Solver.TOLERANCE:
                return []
            return [-c / b]
        
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return []
        
        sqrt_d = math.sqrt(discriminant)
        return [(-b + sqrt_d) / (2*a), (-b - sqrt_d) / (2*a)]

    @staticmethod
    def cubic_solver(coefficients):
        """
        Solves x^3 + a*x^2 + b*x + c = 0
        Translated from RS_Math::cubicSolver
        """
        a, b, c = coefficients
        # Tschirnhaus transformation to depressed cubic: t^3 + pt + q = 0
        shift = a / 3.0
        p = b - (a * a / 3.0)
        q = (2.0 * a**3 / 27.0) - (a * b / 3.0) + c
        
        discriminant = (q**2 / 4.0) + (p**3 / 27.0)
        
        roots = []
        if discriminant > 0:
            sqrt_d = math.sqrt(discriminant)
            u = abs(-q/2.0 + sqrt_d)**(1/3.0)
            if (-q/2.0 + sqrt_d) < 0: u = -u
            v = abs(-q/2.0 - sqrt_d)**(1/3.0)
            if (-q/2.0 - sqrt_d) < 0: v = -v
            roots.append(u + v - shift)
        elif discriminant == 0:
            if p == 0:
                roots.append(-shift)
            else:
                roots.append(3.0 * q / p - shift)
                roots.append(-1.5 * q / p - shift)
        else:
            # Three real roots case
            r = math.sqrt(-(p**3 / 27.0))
            phi = math.acos(-q / (2.0 * r))
            r_val = 2.0 * abs(p/3.0)**0.5
            roots.append(r_val * math.cos(phi/3.0) - shift)
            roots.append(r_val * math.cos((phi + 2*math.pi)/3.0) - shift)
            roots.append(r_val * math.cos((phi + 4*math.pi)/3.0) - shift)
            
        return roots

    @staticmethod
    def quartic_solver(ce):
        """
        Solves x^4 + ce[0]*x^3 + ce[1]*x^2 + ce[2]*x + ce[3] = 0
        Translated from RS_Math::quarticSolver (The Holy Grail)
        """
        # This uses the Ferrari's Method logic
        a, b, c, d = ce
        
        shift = a / 4.0
        p = b - 3.0 * a**2 / 8.0
        q = c - a * b / 2.0 + a**3 / 8.0
        r = d - a * c / 4.0 + a**2 * b / 16.0 - 3.0 * a**4 / 256.0
        
        # Solving the resolvent cubic
        # y^3 + 2py^2 + (p^2 - 4r)y - q^2 = 0
        cubic_roots = RS_Math_Solver.cubic_solver([2.0*p, p**2 - 4.0*r, -q**2])
        
        # Pick a non-negative root
        y = 0
        for root in cubic_roots:
            if root >= 0:
                y = root
                break
        
        sqrt_y = math.sqrt(y)
        
        # Convert back to two quadratics
        res = []
        if sqrt_y > RS_Math_Solver.TOLERANCE:
            q_part = q / (2.0 * sqrt_y)
            res += RS_Math_Solver.quadratic_solver(1.0, sqrt_y, (p + y)/2.0 + q_part)
            res += RS_Math_Solver.quadratic_solver(1.0, -sqrt_y, (p + y)/2.0 - q_part)
        else:
            # Case where q is zero
            res += RS_Math_Solver.quadratic_solver(1.0, 0, p/2.0 + math.sqrt(max(0, p**2/4.0 - r)))
            res += RS_Math_Solver.quadratic_solver(1.0, 0, p/2.0 - math.sqrt(max(0, p**2/4.0 - r)))

        return [x - shift for x in res]

# --- EXISTING UTILS ---

def snap_angle_soft(raw_angle_rad, snap_step_deg, strength_deg):
    """Soft 15° snapping. If strength_deg is 0, no snapping."""
    if strength_deg <= 0.0 or snap_step_deg <= 0.0:
        return raw_angle_rad

    raw_deg = math.degrees(raw_angle_rad)
    nearest = round(raw_deg / snap_step_deg) * snap_step_deg
    if abs(raw_deg - nearest) <= strength_deg:
        return math.radians(nearest)
    return raw_angle_rad

def unwrap(prev_raw, new_raw, accum):
    step = new_raw - prev_raw
    if step > math.pi:
        step -= 2 * math.pi
    elif step < -math.pi:
        step += 2 * math.pi
    return accum + step, new_raw

def arc_points_world(center, r, a0, a1, segs, Xp=None, Yp=None):
    """
    Generates a list of 3D points for an arc.
    Requires Xp, Yp basis vectors to orient the arc in 3D.
    """
    pts = []
    
    # If no basis, assume XY plane
    if Xp is None: Xp = Vector((1, 0, 0))
    if Yp is None: Yp = Vector((0, 1, 0))
        
    d = a1 - a0
    d = max(min(d, 2 * math.pi), -2 * math.pi)
    if abs(abs(d) - 2 * math.pi) < 1e-9:
        d = math.copysign(2 * math.pi - 1e-6, d)
        
    for i in range(segs + 1):
        a = a0 + d * (i / segs)
        p2 = Vector((r * math.cos(a), r * math.sin(a)))
        pts.append(center + plane_to_world(p2, Xp, Yp))
    return pts