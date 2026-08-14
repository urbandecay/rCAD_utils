"""In-memory selection state for the projection workflow.

The state deliberately contains Blender data-block references and topology
snapshots rather than copies of mesh coordinates.  Source vertices therefore
remain live between Store and Project, while a topology edit invalidates the
stored indices instead of silently projecting the wrong geometry.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SourceEntry:
    obj: object
    mesh: object
    vertex_indices: tuple
    edge_indices: tuple
    face_indices: tuple
    selection_history: tuple
    topology_signature: tuple


@dataclass(frozen=True)
class TargetEntry:
    obj: object
    mesh: object
    # None means the complete object.  A tuple means an explicit face subset.
    face_indices: object
    topology_signature: tuple

    @property
    def is_face_selection(self):
        return self.face_indices is not None


class ProjectionStorage:
    def __init__(self):
        self.source_entries = []
        self.source_active_object = None
        self.target = None

    def clear(self):
        self.clear_source()
        self.clear_target()

    def clear_source(self):
        self.source_entries.clear()
        self.source_active_object = None

    def clear_target(self):
        self.target = None

    def store_source(self, entries, active_object=None):
        self.source_entries = list(entries)
        self.source_active_object = active_object

    def store_target(self, entry):
        self.target = entry

    @property
    def vertex_count(self):
        return sum(len(entry.vertex_indices) for entry in self.source_entries)

    @property
    def source_object_count(self):
        return len(self.source_entries)

    @property
    def target_face_count(self):
        if self.target is None or self.target.face_indices is None:
            return 0
        return len(self.target.face_indices)

    def has_source(self):
        return bool(self.source_entries and self.vertex_count)

    def has_target(self):
        return self.target is not None


projection_state = ProjectionStorage()
