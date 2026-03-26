# barc_op.py — Create B-Arc operator (ported from 1D_Lite)

import bpy
import bmesh
import mathutils
from mathutils import Vector
from math import sin, cos, pi
from .utils import (
    check_lukap, find_index_of_selected_vertices,
    find_extreme_select_verts, find_all_connected_verts,
    bm_vert_active_get, getNormalPlane, edit_mode_in, maloe,
)

# Module-level state for arc calculation
p_a_ = None
p_b_ = None
p_c_ = None
p_a = None
p_b = None
p_c = None
max_y = None
p_rad = None


def barc(self, first_time_moving_vertex):
    global p_a_, p_b_, p_c_, p_a, p_b, p_c, max_y, p_rad

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

    config = bpy.context.window_manager.rcad_1d_props

    obj = bpy.context.active_object
    me = obj.data

    verts = find_index_of_selected_vertices(me)
    cou_vs = len(verts) - 1

    if verts is not None and cou_vs > 0:
        extreme_vs = find_extreme_select_verts(me, verts)
        if len(extreme_vs) != 2:
            self.report({'ERROR'}, 'Single Loop only')
            return False

        sort_list = find_all_connected_verts(me, extreme_vs[0], [])
        if len(sort_list) != len(verts):
            self.report({'ERROR'}, 'Incoherent loop')
            return False

        bpy.ops.object.mode_set(mode='OBJECT')
        bm = bmesh.new()
        bm.from_mesh(me)
        check_lukap(bm)

        pa_idx = bm_vert_active_get(bm)[0]
        if pa_idx is None:
            self.report({'ERROR'}, 'Active vert is not detected')
            return False

        if first_time_moving_vertex:
            p_a_ = me.vertices[extreme_vs[0]].co.copy()
            p_b_ = me.vertices[pa_idx].co.copy()
            p_c_ = me.vertices[extreme_vs[1]].co.copy()

        normal_B = getNormalPlane([p_a_, p_b_, p_c_], mathutils.Matrix())
        normal_z = mathutils.Vector((0, 0, -1))
        mat_rot_norm = normal_B.rotation_difference(normal_z).to_matrix().to_4x4()

        if first_time_moving_vertex:
            p_a = mat_rot_norm @ p_a_
            p_b = mat_rot_norm @ p_b_
            p_c = mat_rot_norm @ p_c_

            p_rad = (p_a - p_c).length
            config.arc_size = ((p_b.y - p_c.y) / p_rad) * 10
            first_time_moving_vertex = False
        else:
            p_b.y = (config.arc_size * p_rad / 10) + p_c.y

        p_ab = (p_a + p_b) / 2
        p_bc = (p_b + p_c) / 2
        ab = p_b - p_a
        bc = p_c - p_b
        k_ab = -ab.y / (ab.x + 1e-7)
        k_bc = -bc.y / (bc.x + 1e-7)
        z = p_a.z
        ab_d = mathutils.Vector((k_ab, 1, 0)).normalized()
        bc_d = mathutils.Vector((k_bc, 1, 0)).normalized()
        p_d_ = mathutils.geometry.intersect_line_line(p_ab, p_ab + ab_d, p_bc, p_bc + bc_d)

        if p_d_ is None:
            if isinstance(self, bpy.types.Operator):
                self.report({'ERROR'}, 'Impossible to construct the arc radius')
            else:
                raise ValueError('Impossible to construct the arc radius')

        p_d = p_d_[0]
        ad = p_a - p_d

        radius = ad.length

        angle = ad.angle(p_c - p_d)
        section_angle = angle / (len(sort_list) - 1)
        vector_zero = mathutils.Vector((1, 0, 0))
        angle_zero = pi / 2 + ad.angle(vector_zero)

        test_x = sin(section_angle * (len(sort_list) - 1) + angle_zero) * radius + p_d.x
        test_y = cos(section_angle * (len(sort_list) - 1) + angle_zero) * radius + p_d.y
        test_by_x = abs(test_x - p_c.x) < maloe
        test_by_y = abs(test_y - p_c.y) < maloe
        if not test_by_x or not test_by_y:
            angle_zero = pi / 2 - ad.angle(vector_zero)
            test_x = sin(angle_zero) * radius + p_d.x
            test_y = cos(angle_zero) * radius + p_d.y
            test_by_x = abs(test_x - p_a.x) < maloe
            test_by_y = abs(test_y - p_a.y) < maloe
            if not test_by_x or not test_by_y:
                angle = 2 * pi - angle
                angle_zero = pi / 2 + ad.angle(vector_zero)
                section_angle = angle / (len(sort_list) - 1)
            else:
                test_x = sin(section_angle * (len(sort_list) - 1) + angle_zero) * radius + p_d.x
                test_y = cos(section_angle * (len(sort_list) - 1) + angle_zero) * radius + p_d.y
                test_by_x = abs(test_x - p_c.x) < maloe
                test_by_y = abs(test_y - p_c.y) < maloe
                if not test_by_x or not test_by_y:
                    angle = 2 * pi - angle
                    angle_zero = pi / 2 - ad.angle(vector_zero)
                    section_angle = angle / (len(sort_list) - 1)

        mat_rot_norm_inv = mat_rot_norm.inverted()
        for i, v_idx in enumerate(sort_list):
            x = sin(section_angle * i + angle_zero) * radius + p_d.x
            y = cos(section_angle * i + angle_zero) * radius + p_d.y
            me.vertices[v_idx].co = mat_rot_norm_inv @ Vector((x, y, z))

        edit_mode_in()


def update_moved_vertex(self, context):
    try:
        barc(self, first_time_moving_vertex=False)
    except (ValueError, Exception):
        pass
    finally:
        bpy.ops.object.mode_set(mode='EDIT')


class RCAD_OT_CreateBArc(bpy.types.Operator):
    """Create a Bezier arc through 3 selected vertices"""
    bl_label = "Create B-Arc"
    bl_idname = "rcad.create_b_arc"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        barc(self, first_time_moving_vertex=True)
        return {'FINISHED'}


classes = (RCAD_OT_CreateBArc,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
