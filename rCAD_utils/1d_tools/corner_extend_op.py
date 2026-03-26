# corner_extend_op.py — Corner Extend operator (ported from 1D_Lite)

import bpy
import bmesh
from mathutils.geometry import intersect_line_line
from .utils import check_lukap, get_active_edge


def corner_extend(self, active, to_active):
    obj = bpy.context.active_object
    me = obj.data

    bm = bmesh.new()
    bm.from_mesh(me)
    check_lukap(bm)

    loop1 = get_active_edge(bm)

    if not loop1:
        self.report({'ERROR'}, 'It should be an active edge')
        bm.free()
        return False

    sel_edges = [e for e in bm.edges if e.select and e.verts[0].index not in loop1 and e.verts[1].index not in loop1]
    loops = [[e.verts[0].index, e.verts[1].index] for e in sel_edges]

    if len(loops) == 0:
        self.report({'ERROR'}, 'Need additional selected edges')
        bm.free()
        return False

    bm.verts.ensure_lookup_table()

    for loop2 in loops:
        v1 = bm.verts[loop1[0]].co
        v2 = bm.verts[loop1[1]].co
        v3 = bm.verts[loop2[0]].co
        v4 = bm.verts[loop2[1]].co
        p_cross = intersect_line_line(v1, v2, v3, v4)

        if p_cross is None:
            self.report({'WARNING'}, 'Edges are parallel, no intersection')
            continue

        corner = (p_cross[0] + p_cross[1]) / 2

        i1 = 0 if (corner - v1).length <= (corner - v2).length else 1
        i2 = 0 if (corner - v3).length <= (corner - v4).length else 1

        va = bm.verts[loop1[i1]]
        vb = bm.verts[loop2[i2]]

        if active:
            va.co = corner.copy()
        elif to_active:
            vb.co = corner.copy()
        else:
            va.co = corner.copy()
            vb.co = corner.copy()
            bmesh.ops.pointmerge(bm, verts=[va, vb], merge_co=corner)
            bm.verts.ensure_lookup_table()

    bm.to_mesh(me)
    bm.free()
    return True


class RCAD_OT_CornerExtend(bpy.types.Operator):
    """Extend edges to their intersection point"""
    bl_idname = "rcad.corner_extend"
    bl_label = "Extend"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        config = bpy.context.scene.rcad_1d_props
        active = config.only_active_edge
        to_active = config.to_active_edge
        mode_orig = bpy.context.object.mode
        bpy.ops.object.mode_set(mode='OBJECT')
        corner_extend(self, active, to_active)
        bpy.ops.object.mode_set(mode=mode_orig)
        return {'FINISHED'}


classes = (RCAD_OT_CornerExtend,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
