# utils.py — Shared utilities for 1D tools (ported from 1D_Lite)

import bpy
import bmesh
import mathutils
from mathutils import Vector

maloe = 1e-2


def edit_mode_out():
    bpy.ops.object.mode_set(mode='OBJECT')


def edit_mode_in():
    bpy.ops.object.mode_set(mode='EDIT')


def check_lukap(bm):
    if hasattr(bm.verts, "ensure_lookup_table"):
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()


def find_index_of_selected_vertices(mesh):
    selected_verts = [vertex.index for vertex in mesh.vertices if vertex.select]
    if len(selected_verts) < 1:
        return None
    return selected_verts


def find_extreme_select_verts(mesh, list_of_vertices):
    vertices_at_extremes = []
    edges = mesh.edges
    for vertex in list_of_vertices:
        connecting_edges = [edge for edge in edges if vertex in edge.vertices[:] and edge.select]
        if len(connecting_edges) == 1:
            vertices_at_extremes.append(vertex)
    return vertices_at_extremes


def find_all_connected_verts(meshdata, active_vert, visited_verts=None):
    if visited_verts is None:
        visited_verts = []
    connected_verts = [active_vert]
    visited_verts.append(active_vert)
    neighbors = find_connected_verts(meshdata, active_vert, visited_verts)
    for neighbor_vert in neighbors:
        connected_neighbors = find_all_connected_verts(meshdata, neighbor_vert, visited_verts)
        connected_verts += connected_neighbors
    return connected_verts


def find_connected_verts(meshdata, starting_vert, visited_verts):
    edges = meshdata.edges
    connecting_edges = []
    for edge in edges:
        if starting_vert in edge.vertices:
            connecting_edges.append(edge)
    if len(connecting_edges) == 0:
        return []
    connected_verts = []
    for edge in connecting_edges:
        shared_verts = set(edge.vertices[:])
        shared_verts.remove(starting_vert)
        neighbor_vert = shared_verts.pop()
        if not (neighbor_vert in visited_verts) and meshdata.vertices[neighbor_vert].select:
            connected_verts.append(neighbor_vert)
    return connected_verts


def find_connected_verts_bm(bm, found_index, not_list):
    connecting_edges = bm.verts[found_index].link_edges
    if len(connecting_edges) == 0:
        return []
    connected_verts = []
    for edge in connecting_edges:
        cvert = set((edge.verts[0].index, edge.verts[1].index))
        cvert.remove(found_index)
        vert = cvert.pop()
        if not (vert in not_list) and bm.verts[vert].select:
            connected_verts.append(vert)
            not_list.append(vert)
    return connected_verts


def bm_vert_active_get(bm):
    for elem in reversed(bm.select_history):
        if isinstance(elem, (bmesh.types.BMVert, bmesh.types.BMEdge, bmesh.types.BMFace)):
            return elem.index, str(elem)[3:4]
    return None, None


def getNormalPlane(vecs, mat):
    if len(vecs) < 3:
        return None
    out_ = []
    vec_c = mathutils.Vector((0, 0, 0))
    for v in vecs:
        vec = v @ mat
        out_.append(vec)
        vec_c += vec
    vec_c = vec_c / len(vecs)
    v = out_[1] - out_[0]
    w = out_[2] - out_[0]
    A = v.y * w.z - v.z * w.y
    B = -v.x * w.z + v.z * w.x
    C = v.x * w.y - v.y * w.x
    norm = mathutils.Vector((A, B, C)).normalized()
    return norm


def get_active_edge(bm):
    for elem in reversed(bm.select_history):
        if isinstance(elem, bmesh.types.BMEdge):
            return [elem.verts[0].index, elem.verts[1].index]
        elif isinstance(elem, bmesh.types.BMVert):
            # In vert mode: find a selected edge connected to the active vert
            for e in elem.link_edges:
                if e.select:
                    return [e.verts[0].index, e.verts[1].index]
    return None
