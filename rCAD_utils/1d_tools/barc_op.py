# barc_op.py — Create B-Arc operator (ported from 1D_Lite)

import bpy
import bmesh
import mathutils
from mathutils import Vector
from math import sin, cos, pi, atan, atan2, tan, radians, degrees
from .utils import (
    check_lukap, find_index_of_selected_vertices,
    find_extreme_select_verts, find_all_connected_verts,
    bm_vert_active_get, getNormalPlane, edit_mode_in, maloe,
)

# Module-level state for arc calculation
p_a = None               # endpoint A in rotated (XY) space
p_c = None               # endpoint C in rotated (XY) space
arc_chord_mid = None     # midpoint of chord in rotated space
arc_perp_dir = None      # unit vector perpendicular to chord in XY plane
arc_chord_length = None  # chord length
arc_z = None             # Z value in rotated space (same for all arc verts)
mat_rot_norm_inv_ = None # cached inverse rotation matrix (world from rotated)
arc_sort_list = None     # cached vertex order — reused by slider so it always
arc_obj_name = None      #   updates the same arc regardless of current selection
_arc_busy = False        # re-entrancy guard: stops slider callback firing mid-create


def _sagitta_from_angle(angle_deg, chord_length):
    """Sagitta from signed arc angle (degrees) and chord length.
    0 = flat, ±180 = semicircle. Sign controls which side of chord."""
    if abs(angle_deg) < 1e-6 or chord_length < 1e-7:
        return 0.0
    return chord_length * tan(radians(abs(angle_deg)) / 4) / 2 * (1 if angle_deg > 0 else -1)


def _angle_from_sagitta(sagitta, chord_length):
    """Signed arc angle (degrees) from sagitta and chord length."""
    if chord_length < 1e-7 or abs(sagitta) < 1e-7:
        return 0.0
    ratio = min(abs(sagitta) / chord_length, 0.5)
    angle = degrees(4 * atan(2 * ratio))
    return angle if sagitta > 0 else -angle


def _apply_arc(me, config):
    """Write arc vertex positions using current slider value and cached arc state."""
    global p_a, p_c, arc_chord_mid, arc_perp_dir, arc_chord_length, arc_z
    global mat_rot_norm_inv_, arc_sort_list

    sagitta = _sagitta_from_angle(config.arc_size, arc_chord_length)

    # Flat case (within 0.5°) — interpolate linearly along the chord
    if abs(config.arc_size) < 0.5:
        n = len(arc_sort_list)
        for i, v_idx in enumerate(arc_sort_list):
            t = i / max(n - 1, 1)
            pos = p_a * (1 - t) + p_c * t
            me.vertices[v_idx].co = mat_rot_norm_inv_ @ pos
        return

    p_b = arc_chord_mid.copy()
    p_b += arc_perp_dir * sagitta
    p_b.z = arc_z

    # Circumcenter of p_a, p_b, p_c in XY plane
    p_ab = (p_a + p_b) / 2
    p_bc = (p_b + p_c) / 2
    ab = p_b - p_a
    bc = p_c - p_b
    k_ab = -ab.y / (ab.x + 1e-7)
    k_bc = -bc.y / (bc.x + 1e-7)
    z = arc_z
    ab_d = mathutils.Vector((k_ab, 1, 0)).normalized()
    bc_d = mathutils.Vector((k_bc, 1, 0)).normalized()
    p_d_ = mathutils.geometry.intersect_line_line(p_ab, p_ab + ab_d, p_bc, p_bc + bc_d)

    if p_d_ is None:
        raise ValueError('Impossible to construct the arc radius')

    p_d = p_d_[0]
    p_d.z = arc_z
    radius = (p_a - p_d).length

    # Use p_b's actual angle to decide which arc direction is correct
    def pt_angle(pt):
        return atan2(pt.x - p_d.x, pt.y - p_d.y)

    def norm_angle(a):
        a = a % (2 * pi)
        return a if a >= 0 else a + 2 * pi

    start_a = pt_angle(p_a)
    mid_a   = pt_angle(p_b)
    end_a   = pt_angle(p_c)

    ccw_span   = norm_angle(end_a - start_a)
    mid_offset = norm_angle(mid_a - start_a)

    if mid_offset <= ccw_span + 1e-6:
        section_angle = ccw_span / (len(arc_sort_list) - 1)
    else:
        section_angle = -(2 * pi - ccw_span) / (len(arc_sort_list) - 1)

    angle_zero = start_a

    for i, v_idx in enumerate(arc_sort_list):
        angle = section_angle * i + angle_zero
        x = sin(angle) * radius + p_d.x
        y = cos(angle) * radius + p_d.y
        me.vertices[v_idx].co = mat_rot_norm_inv_ @ Vector((x, y, z))


def barc(self, context):
    global p_a, p_c, arc_chord_mid, arc_perp_dir, arc_chord_length, arc_z
    global mat_rot_norm_inv_, arc_sort_list, arc_obj_name, _arc_busy

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

    config = bpy.context.window_manager.rcad_1d_props
    obj = bpy.context.active_object
    me = obj.data

    verts = find_index_of_selected_vertices(me)
    cou_vs = len(verts) - 1

    if not (verts is not None and cou_vs > 0):
        return False

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

    p_a_ = me.vertices[extreme_vs[0]].co.copy()
    p_b_ = me.vertices[pa_idx].co.copy()
    p_c_ = me.vertices[extreme_vs[1]].co.copy()

    # Rotate arc plane to XY
    normal_B = getNormalPlane([p_a_, p_b_, p_c_], mathutils.Matrix())
    normal_z = mathutils.Vector((0, 0, -1))
    mat_rot_norm = normal_B.rotation_difference(normal_z).to_matrix().to_4x4()
    mat_rot_norm_inv_ = mat_rot_norm.inverted()

    p_a = mat_rot_norm @ p_a_
    p_b_rot = mat_rot_norm @ p_b_
    p_c = mat_rot_norm @ p_c_

    arc_z = p_a.z
    arc_chord_length = (p_a - p_c).length
    arc_chord_mid = (p_a + p_c) / 2

    chord = p_c - p_a
    perp = Vector((-chord.y, chord.x, 0))
    arc_perp_dir = perp.normalized() if perp.length > 1e-7 else Vector((0, 1, 0))

    arc_sort_list = sort_list
    arc_obj_name = obj.name

    # Set slider to match current arc shape — guard prevents recursive slider callback
    sagitta = (p_b_rot - arc_chord_mid).dot(arc_perp_dir)
    _arc_busy = True
    config.arc_size = _angle_from_sagitta(sagitta, arc_chord_length)
    _arc_busy = False

    _apply_arc(me, config)
    edit_mode_in()
    return True


def update_moved_vertex(self, context):
    global _arc_busy, arc_sort_list, arc_obj_name
    if _arc_busy or arc_sort_list is None:
        return
    try:
        obj = bpy.context.active_object
        if obj is None or obj.name != arc_obj_name:
            return
        config = bpy.context.window_manager.rcad_1d_props
        bpy.ops.object.mode_set(mode='OBJECT')
        _apply_arc(obj.data, config)
    except Exception:
        pass
    finally:
        bpy.ops.object.mode_set(mode='EDIT')


class RCAD_OT_CreateBArc(bpy.types.Operator):
    """Create a Bezier arc through 3 selected vertices"""
    bl_label = "Create B-Arc"
    bl_idname = "rcad.create_b_arc"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        barc(self, context)
        return {'FINISHED'}


classes = (RCAD_OT_CreateBArc,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
