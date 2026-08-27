"""Carve Along Path state shared by its path and carve operators.

The actual sweep deliberately lives in :mod:`eap_adapter`: it invokes the
normal Faces branch from Extrude Along Path so both tools use the same path
ordering and profile-placement behavior.
"""

import bmesh


class CarvePathBuffer:
    def __init__(self):
        self.list_ek = []
        self.list_sp = []
        self.mesh_name = None

    @property
    def path_vertex_indices(self):
        return {index for edge in self.list_ek for index in edge}


cap_buf = CarvePathBuffer()


def check_lukap(bm):
    """Keep lookup tables and element indices ready for stored paths."""
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.index_update()
    bm.faces.index_update()


def get_active_element_and_its_indices(bm):
    for elem in reversed(bm.select_history):
        if isinstance(elem, bmesh.types.BMVert):
            return elem.index, 'V'
        if isinstance(elem, bmesh.types.BMEdge):
            return elem.index, 'E'
        if isinstance(elem, bmesh.types.BMFace):
            return elem.index, 'F'
    return None, None

