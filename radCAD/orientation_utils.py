# orientation_utils.py
from mathutils import Vector


def orthonormal_basis_from_normal(n):
    if n is None:
        return None, None, None
    n = n.normalized()
    ref = Vector((1, 0, 0)) if abs(n.dot(Vector((1, 0, 0)))) < 0.99 else Vector((0, 1, 0))
    Yp = n.cross(ref).normalized()
    Xp = Yp.cross(n).normalized()
    return Xp, Yp, n

