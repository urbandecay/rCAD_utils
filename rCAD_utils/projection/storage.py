from dataclasses import dataclass


@dataclass
class SourceEntry:
    obj: object
    mesh: object
    vertex_indices: tuple
    edge_indices: tuple
    face_indices: tuple
    vertex_count: int
    edge_count: int
    face_count: int


class ProjectionSourceStorage:
    def __init__(self):
        self.entries = []

    def clear(self):
        self.entries.clear()

    def store(self, entries):
        self.entries = list(entries)

    @property
    def vertex_count(self):
        return sum(len(entry.vertex_indices) for entry in self.entries)

    def has_source(self):
        return bool(self.entries and self.vertex_count)


projection_source = ProjectionSourceStorage()
