import bpy
import bmesh

class MESH_OT_super_fuse_unweld(bpy.types.Operator):
    bl_idname = "mesh.super_fuse_unweld"
    bl_label = "Super Fuse: Unweld"
    bl_description = "Force-split selected geometry (V/Y-style): duplicates shared verts/edges along selection boundaries and cleans endpoint overlaps"
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

        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        sel_verts = [v for v in bm.verts if v.select and not v.hide]
        sel_edges = [e for e in bm.edges if e.select and not e.hide]
        sel_faces = [f for f in bm.faces if f.select and not f.hide]

        if not sel_verts and not sel_edges and not sel_faces:
            self.report({'INFO'}, "Nothing selected to unweld.")
            return {'CANCELLED'}

        split_edges_set = set()
        split_edges_set.update(sel_edges)
        for v in sel_verts:
            for e in v.link_edges:
                if not e.hide:
                    split_edges_set.add(e)
        for f in sel_faces:
            for e in f.edges:
                if not e.hide:
                    split_edges_set.add(e)

        if not split_edges_set:
            self.report({'INFO'}, "No edges found to split.")
            return {'CANCELLED'}

        deg = {}
        touched_verts = set()
        for e in split_edges_set:
            v0, v1 = e.verts
            touched_verts.add(v0); touched_verts.add(v1)
            deg[v0] = deg.get(v0, 0) + 1
            deg[v1] = deg.get(v1, 0) + 1
        endpoint_seeds = [v for v in touched_verts if deg.get(v, 0) <= 1 and not v.hide and v.is_valid]

        total_new_verts = 0
        new_verts_edge_split = []
        try:
            res = bmesh.ops.split_edges(bm, edges=list(split_edges_set))
            new_verts_edge_split = res.get("verts", []) or []
            total_new_verts = len(new_verts_edge_split)
        except Exception as ex:
            self.report({'ERROR'}, f"Edge split failed: {ex}")
            return {'CANCELLED'}

        total_split_geom = 0
        try:
            if sel_faces:
                res2 = bmesh.ops.split(
                    bm,
                    geom=sel_faces,
                    use_only_faces=True
                )
                g = res2.get("geom", []) or []
                total_split_geom = len(g)
        except Exception:
            pass

        ENDPOINT_MERGE_DIST = 1.0e-12
        endpoint_merges = 0

        def pos_key(v):
            c = v.co
            return (round(float(c.x), 12), round(float(c.y), 12), round(float(c.z), 12))

        if endpoint_seeds:
            pool = set()
            for e in split_edges_set:
                if e.is_valid:
                    pool.update([vv for vv in e.verts if vv and vv.is_valid])
            for v in new_verts_edge_split:
                if v and v.is_valid:
                    pool.add(v)

            seen_positions = set()
            tol2 = ENDPOINT_MERGE_DIST * ENDPOINT_MERGE_DIST

            for seed in endpoint_seeds:
                if not (seed and seed.is_valid):
                    continue
                k = pos_key(seed)
                if k in seen_positions:
                    continue
                seen_positions.add(k)
                p = seed.co
                cluster = [vv for vv in pool
                           if vv and vv.is_valid and (vv.co - p).length_squared <= tol2]
                if len(cluster) >= 2:
                    try:
                        ret_rd = bmesh.ops.remove_doubles(bm, verts=cluster, dist=ENDPOINT_MERGE_DIST)
                    except Exception:
                        ret_rd = {}
                    if isinstance(ret_rd, dict):
                        tm = ret_rd.get("targetmap") or ret_rd.get("vert_map") or ret_rd.get("merge_map")
                        if tm:
                            endpoint_merges += len(tm)

            endpoint_keys = set(pos_key(v) for v in endpoint_seeds if v and v.is_valid)
            stray = [v for v in bm.verts
                     if v and v.is_valid and not v.hide and not v.link_edges and pos_key(v) in endpoint_keys]
            if stray:
                try:
                    bmesh.ops.delete(bm, geom=stray, context='VERTS')
                except Exception:
                    pass

        # Clean only invalid history; do not touch visible selection
        self._scrub_select_history(bm)

        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
        self.report({'INFO'}, f"Unweld: new_verts={total_new_verts}, split_geom={total_split_geom}, endpoint_merges={endpoint_merges}.")
        return {'FINISHED'}


classes = (MESH_OT_super_fuse_unweld,)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)