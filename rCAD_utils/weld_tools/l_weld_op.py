import bpy
import bmesh
from mathutils import Vector
from mathutils.kdtree import KDTree

from .utils import EPS, deferred_update_edit_mesh
from .deselect_manager import get_or_create_session, commit_if_owned


def _collect_selected_edge_endpoints(bm):
    sel_edges = [e for e in bm.edges if e.select and not e.hide]
    endpoint_verts = set()
    for e in sel_edges:
        v0, v1 = e.verts
        endpoint_verts.add(v0)
        endpoint_verts.add(v1)
    return list(endpoint_verts), sel_edges


class _DSU:
    __slots__ = ("p", "r")
    def __init__(self, n):
        self.p = list(range(n))
        self.r = [0] * n
    def find(self, x):
        p = self.p
        while p[x] != x:
            p[x] = p[p[x]]
            x = p[x]
        return x
    def union(self, a, b):
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return False
        if self.r[ra] < self.r[rb]:
            self.p[ra] = rb
        elif self.r[ra] > self.r[rb]:
            self.p[rb] = ra
        else:
            self.p[rb] = ra
            self.r[ra] += 1
        return True


def _clusters_by_radius_world(verts, mw, radius):
    m = len(verts)
    if m < 2:
        return []

    pts = [mw @ v.co for v in verts]
    kd = KDTree(m)
    for i, p in enumerate(pts):
        kd.insert(p, i)
    kd.balance()

    dsu = _DSU(m)
    r = float(max(0.0, radius))
    for i, p in enumerate(pts):
        for co, j, d in kd.find_range(p, r):
            if j > i:
                dsu.union(i, j)

    buckets = {}
    for i in range(m):
        root = dsu.find(i)
        buckets.setdefault(root, []).append(i)

    clusters = []
    for idxs in buckets.values():
        if len(idxs) >= 2:
            clusters.append([verts[i] for i in idxs])
    return clusters


class MESH_OT_super_fuse_l(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_l"
    bl_label = "Super Fuse: L"
    bl_options = {'REGISTER', 'UNDO'}

    def _scrub_select_history(self, bm):
        try:
            for elem in list(bm.select_history):
                if (elem is None) or (not getattr(elem, "is_valid", False)):
                    bm.select_history.remove(elem)
        except Exception:
            pass

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Edit Mode required.")
            return {'CANCELLED'}
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a Mesh.")
            return {'CANCELLED'}

        props = getattr(context.scene, "super_fuse", None)
        radius = float(getattr(props, "search_radius", 1e-4)) if props else 1e-4
        radius = max(0.0, radius)

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        endpoint_verts, sel_edges = _collect_selected_edge_endpoints(bm)
        if not sel_edges:
            self.report({'ERROR'}, "Select at least one edge.")
            return {'CANCELLED'}
        if len(endpoint_verts) < 2:
            self.report({'INFO'}, "Not enough endpoints to weld.")
            return {'CANCELLED'}

        mw = obj.matrix_world.copy()
        mw_inv = mw.inverted_safe()

        clusters = _clusters_by_radius_world(endpoint_verts, mw, radius)
        if not clusters:
            self.report({'INFO'}, "No coincident endpoints found within radius.")
            return {'CANCELLED'}

        targetmap = {}
        groups_merged = 0
        anchors_moved = 0

        # Prepare deselection session (record cluster centers)
        session, owned_local = get_or_create_session()

        for cl in clusters:
            valid = [v for v in cl if v and v.is_valid]
            if len(valid) < 2:
                continue

            sum_w = Vector((0.0, 0.0, 0.0))
            for v in valid:
                sum_w += (mw @ v.co)
            center_w = sum_w / float(len(valid))

            anchor = valid[0]
            if not anchor.is_valid:
                continue
            anchor.co = mw_inv @ center_w
            anchors_moved += 1

            # Record this cluster's center for deselection
            session.add_world('L', center_w, radius)

            for v in valid[1:]:
                if v is not anchor:
                    targetmap[v] = anchor

            groups_merged += 1

        if not targetmap:
            self._scrub_select_history(bm)
            # Apply deselection if we own it; otherwise global executor will do it later.
            commit_if_owned(context, session, owned_local)
            deferred_update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
            self.report({'INFO'}, "No endpoint merges performed.")
            return {'CANCELLED'}

        try:
            bmesh.ops.weld_verts(bm, targetmap=targetmap)
        except Exception as ex:
            self.report({'ERROR'}, f"Weld failed: {ex}")
            return {'CANCELLED'}

        self._scrub_select_history(bm)
        # Apply deselection if local; otherwise executor will apply at the end.
        commit_if_owned(context, session, owned_local)

        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
        self.report({'INFO'}, f"L: groups={groups_merged}, welded={len(targetmap)} endpoints, moved_anchors={anchors_moved}.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_l,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)