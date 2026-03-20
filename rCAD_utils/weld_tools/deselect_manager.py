# deselect_manager.py

import bmesh
from mathutils import Vector
from mathutils.kdtree import KDTree

# Simple manager to record and apply vertex deselection by proximity in WORLD space.
# Operators add world-space positions for their welded results with a radius, tagged by kind: 'X', 'T', 'Dot', 'L'.
# The manager will find current mesh verts within that radius and deselect them.

# Global active session (used by the combined executor). If None, operators use ephemeral sessions.
_ACTIVE_SESSION = None


class DeselectSession:
    def __init__(self):
        # records: list of tuples (kind: str, pos_world: Vector, radius: float)
        self.records = []

    def is_empty(self):
        return not self.records

    def add_world(self, kind: str, pos_world, radius: float):
        if pos_world is None:
            return
        try:
            p = Vector((float(pos_world.x), float(pos_world.y), float(pos_world.z)))
        except Exception:
            # Allow tuples
            try:
                x, y, z = pos_world
                p = Vector((float(x), float(y), float(z)))
            except Exception:
                return
        r = float(max(0.0, radius))
        self.records.append((str(kind), p, r))

    def add_from_verts(self, kind: str, verts, obj, radius: float):
        if not obj or not verts:
            return
        mw = getattr(obj, "matrix_world", None)
        if mw is None:
            return
        for v in verts:
            if v and getattr(v, "is_valid", False):
                self.add_world(kind, mw @ v.co, radius)

    def apply_on_object(self, obj):
        """
        Apply deselection to 'obj' EditMesh verts by proximity to stored world points.
        Returns a dict kind->count deselected.
        """
        if not obj or obj.type != 'MESH' or not self.records:
            return {}

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        mw = obj.matrix_world

        # Build a KDTree of current, visible vertices in WORLD space
        pool = [v for v in bm.verts if v and v.is_valid and not v.hide]
        if not pool:
            return {}

        kd = KDTree(len(pool))
        for i, v in enumerate(pool):
            kd.insert(mw @ v.co, i)
        kd.balance()

        # Collect matches by kind
        matched_by_kind = {}
        for kind, p_w, r in self.records:
            r_eff = float(r) + 1.0e-12  # tiny slack
            for _co, idx, _d in kd.find_range(p_w, r_eff):
                v = pool[idx]
                # Deselect; we tally by kind
                v.select = False
                matched_by_kind[kind] = matched_by_kind.get(kind, 0) + 1

        # Do not update the mesh here; let the caller/operator handle update timing.
        return matched_by_kind


# ---- Global session helpers (used by the combined executor) ----

def start_global_session():
    global _ACTIVE_SESSION
    _ACTIVE_SESSION = DeselectSession()
    return _ACTIVE_SESSION


def get_active_session():
    return _ACTIVE_SESSION


def end_global_session(context):
    """
    Apply and clear the global session if present.
    """
    global _ACTIVE_SESSION
    sess = _ACTIVE_SESSION
    _ACTIVE_SESSION = None
    if not sess or sess.is_empty():
        return

    obj = context.edit_object if context and getattr(context, "edit_object", None) else None
    if not obj or obj.type != 'MESH':
        return

    # Apply deselection and update edit mesh
    sess.apply_on_object(obj)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)


# ---- Operator-friendly helpers ----

def get_or_create_session():
    """
    Returns (session, owned_local):
      - If a global session exists (executor), returns it with owned_local=False.
      - Otherwise, creates a local session and returns owned_local=True.
    """
    sess = get_active_session()
    if sess is not None:
        return sess, False
    return DeselectSession(), True


def commit_if_owned(context, session: DeselectSession, owned_local: bool):
    """
    If the session is locally owned (no global executor), apply deselection now and discard.
    If a global session is active, do nothing here; the executor will apply once at the end.
    """
    if not session or not owned_local or session.is_empty():
        return

    obj = context.edit_object if context and getattr(context, "edit_object", None) else None
    if not obj or obj.type != 'MESH':
        return

    session.apply_on_object(obj)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
