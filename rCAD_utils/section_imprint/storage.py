"""Stored section-plane state for the edit-mode section imprint tool."""

from mathutils import Vector


class SectionPlaneStorage:
    """Keep one world-space plane independent from the current edit object.

    The plane is deliberately stored as a point, a normal, and a right-handed
    in-plane basis.  Keeping the basis here makes the generated section mesh
    deterministic even when the target object has a rotated or non-uniformly
    scaled transform.
    """

    _instance = None

    def __init__(self):
        self.clear()

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def store(self, origin, normal, axis_u, axis_v, source_object_name=""):
        self.origin = Vector(origin).to_3d()
        self.normal = Vector(normal).to_3d().normalized()
        self.axis_u = Vector(axis_u).to_3d().normalized()
        self.axis_v = Vector(axis_v).to_3d().normalized()
        self.source_object_name = str(source_object_name or "")
        self.plane_status = "Stored"

    def clear(self):
        self.origin = None
        self.normal = None
        self.axis_u = None
        self.axis_v = None
        self.source_object_name = ""
        self.plane_status = "None"

    def has_plane(self):
        return (
            self.origin is not None
            and self.normal is not None
            and self.axis_u is not None
            and self.axis_v is not None
        )

    def snapshot(self):
        if not self.has_plane():
            return None
        return {
            "origin": self.origin.copy(),
            "normal": self.normal.copy(),
            "axis_u": self.axis_u.copy(),
            "axis_v": self.axis_v.copy(),
            "source_object_name": self.source_object_name,
        }


section_plane_storage = SectionPlaneStorage.get_instance()
