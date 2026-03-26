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

    sel_edges = [e for e in bm.edges if e.select and e.verts[0].index not in loop1]
    loops = [[e.verts[0].index, e.verts[1].index] for e in sel_edges]

    if len(loops) == 0:
        self.report({'ERROR'}, 'Need additional selected edges')
        bm.free()
        return False

    for loop2 in loops:
        verts = me.vertices
        v1 = verts[loop1[0]].co
        v2 = verts[loop1[1]].co
        v3 = verts[loop2[0]].co
        v4 = verts[loop2[1]].co
        p_cross = intersect_line_line(v1, v2, v3, v4)

        l1 = (p_cross[0] - v1).length
        l2 = (p_cross[0] - v2).length
        i1 = 0
        if l2 < l1:
            i1 = 1

        l1 = (p_cross[1] - v3).length
        l2 = (p_cross[1] - v4).length
        i2 = 0
        if l2 < l1:
            i2 = 1

        if (active or to_active) and loop1:
            if me.vertices[loop1[i1]].index in loop1:
                if active:
                    me.vertices[loop1[i1]].co = p_cross[0]
                else:
                    me.vertices[loop2[i2]].co = p_cross[1]
            else:
                if active:
                    me.vertices[loop2[i2]].co = p_cross[1]
                else:
                    me.vertices[loop1[i1]].co = p_cross[0]
        else:
            me.vertices[loop1[i1]].co = p_cross[0]
            me.vertices[loop2[i2]].co = p_cross[1]

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
