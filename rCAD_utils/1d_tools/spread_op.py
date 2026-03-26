# spread_op.py — Spread Loop operator (ported from 1D_Lite)

import bpy
import bmesh
from .utils import (
    check_lukap, find_index_of_selected_vertices,
    find_extreme_select_verts, find_all_connected_verts,
    bm_vert_active_get,
)

steps_smoose = 5


def main_spread(self, context, mode, influe):
    conf = bpy.context.window_manager.rcad_1d_props
    if not conf.shape_spline:
        mode = (mode[0], mode[1], mode[2], not mode[3])

    if conf.shape_spline and influe < 51:
        return main_spline(self, context, mode, influe / 50)
    elif conf.shape_spline and influe < 101:
        if not conf.spline_Bspline2 or main_spline(self, context, mode, (100 - influe) / 50):
            return main_B_spline_2(self, context, mode, (influe - 50) / 50)
        else:
            return False
    elif conf.shape_spline and influe < 151:
        if not conf.spline_Bspline2 or main_B_spline_2(self, context, mode, (150 - influe) / 50):
            return main_B_spline(self, context, mode, (influe - 100) / 50)
        else:
            return False
    elif conf.shape_spline and influe < 201:
        if not conf.spline_Bspline2 or main_B_spline(self, context, mode, (200 - influe) / 50):
            return main_Basier_mid(self, context, mode, (influe - 150) / 50)
        else:
            return False
    elif conf.shape_spline and influe > 200:
        if conf.spline_Bspline2:
            return main_Basier_mid(self, context, mode, (250 - influe) / 50)
        else:
            return False

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

    obj = bpy.context.active_object
    me = obj.data

    verts = find_index_of_selected_vertices(me)
    cou_vs = len(verts) - 1
    if verts is not None and cou_vs > 0:
        extreme_vs = find_extreme_select_verts(me, verts)

        if len(extreme_vs) != 2:
            self.report({'ERROR'}, 'Single Loop only')
            return False

        list_koeff = []

        if mode[0]:
            min_v = min([me.vertices[extreme_vs[0]].co.x, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.x, extreme_vs[1]])
            max_v = max([me.vertices[extreme_vs[0]].co.x, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.x, extreme_vs[1]])

            if (max_v[0] - min_v[0]) == 0:
                min_v = [me.vertices[extreme_vs[0]].co.x, extreme_vs[0]]
                max_v = [me.vertices[extreme_vs[1]].co.x, extreme_vs[1]]

            sort_list = find_all_connected_verts(me, min_v[1], [])

            if len(sort_list) != len(verts):
                self.report({'ERROR'}, 'Incoherent loop')
                return False

            step = []
            if mode[3]:
                list_length = []
                sum_length = 0.0
                x_sum = 0.0
                for sl in range(cou_vs):
                    subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
                    length = subb.length
                    sum_length += length
                    list_length.append(sum_length)
                    x_sum += subb.x

                for sl in range(cou_vs):
                    tmp = list_length[sl] / sum_length
                    list_koeff.append(tmp)
                    step.append(x_sum * tmp)
            else:
                diap = (max_v[0] - min_v[0]) / cou_vs
                for sl in range(cou_vs):
                    step.append((sl + 1) * diap)

            bpy.ops.object.mode_set(mode='OBJECT')
            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.x = me.vertices[sort_list[0]].co.x + step[idx]
            bpy.ops.object.mode_set(mode='EDIT')

        if mode[1]:
            min_v = min([me.vertices[extreme_vs[0]].co.y, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.y, extreme_vs[1]])
            max_v = max([me.vertices[extreme_vs[0]].co.y, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.y, extreme_vs[1]])

            if (max_v[0] - min_v[0]) == 0:
                min_v = [me.vertices[extreme_vs[0]].co.y, extreme_vs[0]]
                max_v = [me.vertices[extreme_vs[1]].co.y, extreme_vs[1]]

            sort_list = find_all_connected_verts(me, min_v[1], [])

            if len(sort_list) != len(verts):
                self.report({'ERROR'}, 'Incoherent loop')
                return False

            step = []
            if mode[3]:
                list_length = []
                sum_length = 0.0
                y_sum = 0.0
                if len(list_koeff) == 0:
                    for sl in range(cou_vs):
                        subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
                        length = subb.length
                        sum_length += length
                        list_length.append(sum_length)
                        y_sum += subb.y
                    for sl in range(cou_vs):
                        tmp = list_length[sl] / sum_length
                        list_koeff.append(tmp)
                        step.append(y_sum * tmp)
                else:
                    for sl in range(cou_vs):
                        subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
                        y_sum += subb.y
                        tmp = list_koeff[sl]
                        step.append(y_sum * tmp)
            else:
                diap = (max_v[0] - min_v[0]) / cou_vs
                for sl in range(cou_vs):
                    step.append((sl + 1) * diap)

            bpy.ops.object.mode_set(mode='OBJECT')
            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.y = me.vertices[sort_list[0]].co.y + step[idx]
            bpy.ops.object.mode_set(mode='EDIT')

        if mode[2]:
            min_v = min([me.vertices[extreme_vs[0]].co.z, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.z, extreme_vs[1]])
            max_v = max([me.vertices[extreme_vs[0]].co.z, extreme_vs[0]],
                        [me.vertices[extreme_vs[1]].co.z, extreme_vs[1]])

            if (max_v[0] - min_v[0]) == 0:
                min_v = [me.vertices[extreme_vs[0]].co.z, extreme_vs[0]]
                max_v = [me.vertices[extreme_vs[1]].co.z, extreme_vs[1]]

            sort_list = find_all_connected_verts(me, min_v[1], [])

            if len(sort_list) != len(verts):
                self.report({'ERROR'}, 'Incoherent loop')
                return False

            step = []
            if mode[3]:
                list_length = []
                sum_length = 0.0
                z_sum = 0.0
                if len(list_koeff) == 0:
                    for sl in range(cou_vs):
                        subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
                        length = subb.length
                        sum_length += length
                        list_length.append(sum_length)
                        z_sum += subb.z
                    for sl in range(cou_vs):
                        step.append(z_sum * list_length[sl] / sum_length)
                else:
                    for sl in range(cou_vs):
                        subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
                        z_sum += subb.z
                        tmp = list_koeff[sl]
                        step.append(z_sum * tmp)
            else:
                diap = (max_v[0] - min_v[0]) / cou_vs
                for sl in range(cou_vs):
                    step.append((sl + 1) * diap)

            bpy.ops.object.mode_set(mode='OBJECT')
            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.z = me.vertices[sort_list[0]].co.z + step[idx]
            bpy.ops.object.mode_set(mode='EDIT')

    return True


# ==========================================================================

def main_spline(self, context, mode, influe):
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

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

        list_length = []
        sum_length = 0.0
        for sl in range(cou_vs):
            subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
            sum_length += subb.length
            list_length.append(sum_length)

        list_koeff = []
        for sl in range(cou_vs):
            tmp = list_length[sl] / sum_length
            list_koeff.append(tmp)

        bpy.ops.object.mode_set(mode='OBJECT')
        bm = bmesh.new()
        bm.from_mesh(me)
        check_lukap(bm)

        pa_idx = bm_vert_active_get(bm)[0]

        if pa_idx is None:
            self.report({'ERROR'}, 'Active vert is not detected')
            return False

        pa_sort = sort_list.index(pa_idx)
        if pa_sort == 0:
            pa_sort = 1
        pa_perc = list_koeff[pa_sort - 1]
        p0_ = me.vertices[sort_list[0]].co
        p1_ = me.vertices[pa_idx].co
        p2_ = me.vertices[sort_list[-1]].co

        if mode[3]:
            l = len(list_koeff)
            d = 1 / l
            list_koeff = list(map(lambda n: d * n, list(range(1, l + 1))))

        if mode[0]:
            p0 = p0_.x
            p1 = p1_.x - p0
            p2 = p2_.x - p0

            if p1 == 0 or p1 == p2:
                new_vts = list(map(lambda t: p2 * t ** 2, list_koeff))
            else:
                b = (p1 - pa_perc ** 2 * p2) / (2 * pa_perc * (1 - pa_perc) + 1e-8)
                new_vts = list(map(lambda t: 2 * b * t * (1 - t) + p2 * t ** 2, list_koeff))

            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.x += (new_vts[idx] + p0 - me.vertices[sort_list[idx + 1]].co.x) * influe

        if mode[1]:
            p0 = p0_.y
            p1 = p1_.y - p0
            p2 = p2_.y - p0

            b = (p1 - pa_perc ** 2 * p2) / (2 * pa_perc * (1 - pa_perc) + 1e-8)
            new_vts = list(map(lambda t: 2 * b * t * (1 - t) + p2 * t ** 2, list_koeff))

            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.y += (new_vts[idx] + p0 - me.vertices[sort_list[idx + 1]].co.y) * influe

        if mode[2]:
            p0 = p0_.z
            p1 = p1_.z - p0
            p2 = p2_.z - p0

            b = (p1 - pa_perc ** 2 * p2) / (2 * pa_perc * (1 - pa_perc) + 1e-8)
            new_vts = list(map(lambda t: 2 * b * t * (1 - t) + p2 * t ** 2, list_koeff))

            for idx in range(cou_vs):
                me.vertices[sort_list[idx + 1]].co.z += (new_vts[idx] + p0 - me.vertices[sort_list[idx + 1]].co.z) * influe

        me.update()
        bm.free()
        bpy.ops.object.mode_set(mode='EDIT')

    return True


# ==========================================================================

def main_B_spline(self, context, mode, influe):
    global steps_smoose
    spread_x = mode[0]
    spread_y = mode[1]
    spread_z = mode[2]
    Uniform = mode[3]

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

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

        list_of_successive_summed_edge_lengths = []
        sum_length = 0.0
        for sl in range(cou_vs - 2):
            edge_segment = me.vertices[sort_list[sl + 2]].co - me.vertices[sort_list[sl + 1]].co
            sum_length += edge_segment.length
            list_of_successive_summed_edge_lengths.append(sum_length)

        proportions_of_path_segments = []
        for sl in range(cou_vs - 2):
            tmp = list_of_successive_summed_edge_lengths[sl] / sum_length
            proportions_of_path_segments.append(tmp)

        bpy.ops.object.mode_set(mode='OBJECT')
        bm = bmesh.new()
        bm.from_mesh(me)
        check_lukap(bm)

        active_vert = bm_vert_active_get(bm)[0]

        if active_vert is None:
            self.report({'ERROR'}, 'Active vert is not detected')
            return False

        index_of_active_vert = sort_list.index(active_vert)
        if index_of_active_vert < 2:
            index_of_active_vert = 2
        if index_of_active_vert > len(sort_list) - 3:
            index_of_active_vert = len(sort_list) - 3
        active_vert = sort_list[index_of_active_vert]
        initial_path_segment = proportions_of_path_segments[index_of_active_vert - 2]
        p0_ = me.vertices[sort_list[1]].co
        p1_ = me.vertices[active_vert].co
        p2_ = me.vertices[sort_list[-2]].co

        kn1_ = me.vertices[sort_list[0]].co
        kn2_ = me.vertices[sort_list[-1]].co
        nkn1_ = p1_ - kn1_ + p1_
        nkn2_ = p2_ - kn2_ + p2_

        if Uniform:
            l = len(proportions_of_path_segments)
            d = 1 / l
            proportions_of_path_segments = list(map(lambda n: d * n, list(range(1, l + 1))))

        def _do_axis(p0, p1_val, p2_val, knot_1, knot_2, axis_attr):
            t = initial_path_segment
            b = (p1_val - (4 * knot_1 * t * (1 - t) ** 3) - (4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4)) / (4 * t ** 2 * (1 - t) ** 2 + 1e-8)
            props = proportions_of_path_segments
            updated = list(map(lambda t: 4 * knot_1 * t * (1 - t) ** 3 + 4 * b * t ** 2 * (1 - t) ** 2 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4, props))

            if Uniform:
                for c in range(steps_smoose):
                    updated_ = [0] + updated
                    V = [vi for vi in updated_]
                    P = list(map(lambda x, y: abs(y - x), V[:-1], V[1:]))
                    L = sum(P)
                    lp = len(P)
                    d = L / lp
                    l_ = list(map(lambda y: d * y / L, list(range(1, lp + 1))))
                    l = list(map(lambda x: x / L, P))
                    tmp = 0
                    for i in range(lp):
                        tmp += l[i]
                        m = l_[i] / tmp
                        props[i] = m * props[i]
                    updated = list(map(lambda t: 4 * knot_1 * t * (1 - t) ** 3 + 4 * b * t ** 2 * (1 - t) ** 2 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4, props))

            for idx in range(cou_vs - 2):
                old = getattr(me.vertices[sort_list[idx + 2]].co, axis_attr)
                setattr(me.vertices[sort_list[idx + 2]].co, axis_attr,
                        old + (updated[idx] + p0 - old) * influe)

        if spread_x:
            _do_axis(p0_.x, p1_.x - p0_.x, p2_.x - p0_.x, nkn1_.x - p0_.x, nkn2_.x - p0_.x, 'x')
        if spread_y:
            _do_axis(p0_.y, p1_.y - p0_.y, p2_.y - p0_.y, nkn1_.y - p0_.y, nkn2_.y - p0_.y, 'y')
        if spread_z:
            _do_axis(p0_.z, p1_.z - p0_.z, p2_.z - p0_.z, nkn1_.z - p0_.z, nkn2_.z - p0_.z, 'z')

        me.update()
        bm.free()
        bpy.ops.object.mode_set(mode='EDIT')

    return True


# ==========================================================================

def main_B_spline_2(self, context, mode, influe):
    global steps_smoose
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

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

        list_length = []
        sum_length = 0.0
        for sl in range(cou_vs):
            subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
            sum_length += subb.length
            list_length.append(sum_length)

        list_koeff = []
        for sl in range(cou_vs):
            tmp = list_length[sl] / sum_length
            list_koeff.append(tmp)

        bpy.ops.object.mode_set(mode='OBJECT')
        bm = bmesh.new()
        bm.from_mesh(me)
        check_lukap(bm)

        pa_idx = bm_vert_active_get(bm)[0]

        if pa_idx is None:
            self.report({'ERROR'}, 'Active vert is not detected')
            return False

        list_koeff = [0] + list_koeff
        pa_sort = sort_list.index(pa_idx)
        if pa_sort == 0:
            pa_perc = 0
            kn1_i = sort_list[0]
            kn2_i = sort_list[pa_sort + 1]
        elif pa_sort == len(sort_list) - 1:
            pa_perc = 1.0
            kn1_i = sort_list[pa_sort - 1]
            kn2_i = sort_list[-1]
        else:
            kn1_i = sort_list[pa_sort - 1]
            kn2_i = sort_list[pa_sort + 1]
            pa_perc = list_koeff[pa_sort]

        kn1_ = me.vertices[kn1_i].co
        kn2_ = me.vertices[kn2_i].co
        p0_ = me.vertices[sort_list[0]].co
        p1_ = me.vertices[pa_idx].co
        p2_ = me.vertices[sort_list[-1]].co

        if mode[3]:
            l = len(list_koeff) - 1
            d = 1 / l
            list_koeff = list(map(lambda n: d * n, list(range(0, l + 1))))

        def _do_axis_b2(p0, p1_val, p2_val, knot_1, knot_2, axis_attr):
            t = pa_perc
            if knot_1 == 0 and p1_val != 0:
                b = (p1_val - (3 * knot_2 * t ** 2 * (1 - t) + p2_val * t ** 3)) / (3 * t * (1 - t) ** 2 + 1e-8)
                new_vts = list(map(lambda t: 3 * b * t * (1 - t) ** 2 + 3 * knot_2 * t ** 2 * (1 - t) + p2_val * t ** 3, list_koeff))
            elif p1_val == 0:
                new_vts = list(map(lambda t: 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff))
            elif knot_2 == p2_val and p1_val != p2_val:
                b = (p1_val - (3 * knot_1 * t * (1 - t) ** 2 + p2_val * t ** 3)) / (3 * t ** 2 * (1 - t) + 1e-8)
                new_vts = list(map(lambda t: 3 * knot_1 * t * (1 - t) ** 2 + 3 * b * t ** 2 * (1 - t) + p2_val * t ** 3, list_koeff))
            elif p1_val == p2_val:
                new_vts = list(map(lambda t: 2 * knot_1 * t * (1 - t) + p2_val * t ** 2, list_koeff))
            else:
                b = (p1_val - (4 * knot_1 * t * (1 - t) ** 3 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4)) / (4 * t ** 2 * (1 - t) ** 2 + 1e-8)
                new_vts = list(map(lambda t: 4 * knot_1 * t * (1 - t) ** 3 + 4 * b * t ** 2 * (1 - t) ** 2 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4, list_koeff))

            if mode[3]:
                for c in range(steps_smoose):
                    new_vts_ = new_vts
                    V = [vi for vi in new_vts_]
                    P = list(map(lambda x, y: abs(y - x), V[:-1], V[1:]))
                    L = sum(P)
                    lp = len(P)
                    d_val = L / lp
                    l_ = list(map(lambda y: d_val * y / L, list(range(1, lp + 1))))
                    l_list = list(map(lambda x: x / L, P))
                    tmp = 1e-8
                    for i in range(lp):
                        tmp += l_list[i]
                        m = l_[i] / tmp
                        list_koeff[i] = m * list_koeff[i]
                    # Recalculate with same branch logic
                    if knot_1 == 0 and p1_val != 0:
                        b2 = (p1_val - (3 * knot_2 * t ** 2 * (1 - t) + p2_val * t ** 3)) / (3 * t * (1 - t) ** 2 + 1e-8)
                        new_vts = list(map(lambda t: 3 * b2 * t * (1 - t) ** 2 + 3 * knot_2 * t ** 2 * (1 - t) + p2_val * t ** 3, list_koeff))
                    elif p1_val == 0:
                        new_vts = list(map(lambda t: 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff))
                    elif knot_2 == p2_val and p1_val != p2_val:
                        b2 = (p1_val - (3 * knot_1 * t * (1 - t) ** 2 + p2_val * t ** 3)) / (3 * t ** 2 * (1 - t) + 1e-8)
                        new_vts = list(map(lambda t: 3 * knot_1 * t * (1 - t) ** 2 + 3 * b2 * t ** 2 * (1 - t) + p2_val * t ** 3, list_koeff))
                    elif p1_val == p2_val:
                        new_vts = list(map(lambda t: 2 * knot_1 * t * (1 - t) + p2_val * t ** 2, list_koeff))
                    else:
                        b2 = (p1_val - (4 * knot_1 * t * (1 - t) ** 3 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4)) / (4 * t ** 2 * (1 - t) ** 2 + 1e-8)
                        new_vts = list(map(lambda t: 4 * knot_1 * t * (1 - t) ** 3 + 4 * b2 * t ** 2 * (1 - t) ** 2 + 4 * t ** 3 * (1 - t) * knot_2 + p2_val * t ** 4, list_koeff))

            for idx in range(cou_vs + 1):
                old = getattr(me.vertices[sort_list[idx]].co, axis_attr)
                setattr(me.vertices[sort_list[idx]].co, axis_attr,
                        old + (new_vts[idx] + p0 - old) * influe)

        if mode[0]:
            _do_axis_b2(p0_.x, p1_.x - p0_.x, p2_.x - p0_.x, kn1_.x - p0_.x, kn2_.x - p0_.x, 'x')
        if mode[1]:
            _do_axis_b2(p0_.y, p1_.y - p0_.y, p2_.y - p0_.y, kn1_.y - p0_.y, kn2_.y - p0_.y, 'y')
        if mode[2]:
            _do_axis_b2(p0_.z, p1_.z - p0_.z, p2_.z - p0_.z, kn1_.z - p0_.z, kn2_.z - p0_.z, 'z')

        me.update()
        bm.free()
        bpy.ops.object.mode_set(mode='EDIT')

    return True


# ==========================================================================

def main_Basier_mid(self, context, mode, influe):
    global steps_smoose
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')

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
            bm.free()
            self.report({'ERROR'}, 'Active vert is not detected')
            return False

        pa_sort = sort_list.index(pa_idx)

        list_length_a = []
        list_length_b = []
        sum_length_a = 0.0
        sum_length_b = 0.0
        for sl in range(pa_sort - 1):
            subb = me.vertices[sort_list[sl + 1]].co - me.vertices[sort_list[sl]].co
            sum_length_a += subb.length
            list_length_a.append(sum_length_a)
        for sl in range(cou_vs - pa_sort - 1):
            subb = me.vertices[sort_list[sl + 2 + pa_sort]].co - me.vertices[sort_list[sl + 1 + pa_sort]].co
            sum_length_b += subb.length
            list_length_b.append(sum_length_b)

        list_koeff_a = []
        list_koeff_b = []
        for sl in range(len(list_length_a)):
            list_koeff_a.append(list_length_a[sl] / sum_length_a)
        for sl in range(len(list_length_b)):
            list_koeff_b.append(list_length_b[sl] / sum_length_b)

        list_koeff_a = [0] + list_koeff_a
        list_koeff_b = [0] + list_koeff_b

        if pa_sort == 0:
            kn1_i = sort_list[0]
            kn2_i = sort_list[pa_sort + 1]
        elif pa_sort == len(sort_list) - 1:
            kn1_i = sort_list[pa_sort - 1]
            kn2_i = sort_list[-1]
        else:
            kn1_i = sort_list[pa_sort - 1]
            kn2_i = sort_list[pa_sort + 1]

        nkn1_ = me.vertices[kn1_i].co
        nkn2_ = me.vertices[kn2_i].co
        p0_ = me.vertices[sort_list[0]].co
        p1_ = me.vertices[pa_idx].co
        p2_ = me.vertices[sort_list[-1]].co
        kn1_ = nkn1_ - p1_ + nkn1_
        kn2_ = nkn2_ - p1_ + nkn2_

        if mode[3]:
            la = len(list_koeff_a) - 1
            lb = len(list_koeff_b) - 1
            da = 0 if la == 0 else 1 / la
            db = 0 if lb == 0 else 1 / lb
            list_koeff_a = list(map(lambda n: da * n, list(range(0, la + 1))))
            list_koeff_b = list(map(lambda n: db * n, list(range(0, lb + 1))))

        def _do_axis_mid(p0, p1_val, p2_val, knot_1, knot_2, pA, pB, nkn1, nkn2, axis_attr):
            if nkn1 == 0 or p1_val == 0:
                new_vts_a = []
                new_vts_b = list(map(lambda t: pB * (1 - t) ** 2 + 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff_b))
            elif nkn2 == p2_val or p1_val == p2_val:
                new_vts_a = list(map(lambda t: 2 * knot_1 * t * (1 - t) + pA * t ** 2, list_koeff_a))
                new_vts_b = []
            else:
                new_vts_a = list(map(lambda t: 2 * knot_1 * t * (1 - t) + pA * t ** 2, list_koeff_a))
                new_vts_b = list(map(lambda t: pB * (1 - t) ** 2 + 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff_b))

            if mode[3]:
                for c in range(steps_smoose):
                    # Smooth a
                    V = [vi for vi in new_vts_a]
                    P = list(map(lambda x, y: abs(y - x), V[:-1], V[1:]))
                    L = sum(P)
                    lp = len(P)
                    if lp > 0:
                        d_val = L / lp
                        l_ = list(map(lambda y: d_val * y / L, list(range(1, lp + 1))))
                        l_list = list(map(lambda x: x / L, P))
                        tmp = 1e-8
                        for i in range(lp):
                            tmp += l_list[i]
                            m = l_[i] / tmp
                            list_koeff_a[i] = m * list_koeff_a[i]
                        if nkn1 == 0 or p1_val == 0:
                            new_vts_a = []
                        elif nkn2 == p2_val or p1_val == p2_val:
                            new_vts_a = list(map(lambda t: 2 * knot_1 * t * (1 - t) + pA * t ** 2, list_koeff_a))
                        else:
                            new_vts_a = list(map(lambda t: 2 * knot_1 * t * (1 - t) + pA * t ** 2, list_koeff_a))

                    # Smooth b
                    V = [vi for vi in new_vts_b]
                    P = list(map(lambda x, y: abs(y - x), V[:-1], V[1:]))
                    L = sum(P)
                    lp = len(P)
                    if lp > 0:
                        d_val = L / lp
                        l_ = list(map(lambda y: d_val * y / L, list(range(1, lp + 1))))
                        l_list = list(map(lambda x: x / L, P))
                        tmp = 1e-8
                        for i in range(lp):
                            tmp += l_list[i]
                            m = l_[i] / tmp
                            list_koeff_b[i] = m * list_koeff_b[i]
                        if nkn1 == 0 or p1_val == 0:
                            new_vts_b = list(map(lambda t: pB * (1 - t) ** 2 + 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff_b))
                        elif nkn2 == p2_val or p1_val == p2_val:
                            new_vts_b = []
                        else:
                            new_vts_b = list(map(lambda t: pB * (1 - t) ** 2 + 2 * knot_2 * t * (1 - t) + p2_val * t ** 2, list_koeff_b))

            if new_vts_a:
                for idx in range(pa_sort):
                    old = getattr(me.vertices[sort_list[idx]].co, axis_attr)
                    setattr(me.vertices[sort_list[idx]].co, axis_attr,
                            old + (new_vts_a[idx] + p0 - old) * influe)
            if new_vts_b:
                for idx in range(cou_vs - pa_sort):
                    old = getattr(me.vertices[sort_list[idx + pa_sort + 1]].co, axis_attr)
                    setattr(me.vertices[sort_list[idx + pa_sort + 1]].co, axis_attr,
                            old + (new_vts_b[idx] + p0 - old) * influe)

        if mode[0]:
            _do_axis_mid(p0_.x, p1_.x - p0_.x, p2_.x - p0_.x, kn1_.x - p0_.x, kn2_.x - p0_.x,
                         nkn1_.x - p0_.x, nkn2_.x - p0_.x, nkn1_.x - p0_.x, nkn2_.x - p0_.x, 'x')
        if mode[1]:
            _do_axis_mid(p0_.y, p1_.y - p0_.y, p2_.y - p0_.y, kn1_.y - p0_.y, kn2_.y - p0_.y,
                         nkn1_.y - p0_.y, nkn2_.y - p0_.y, nkn1_.y - p0_.y, nkn2_.y - p0_.y, 'y')
        if mode[2]:
            _do_axis_mid(p0_.z, p1_.z - p0_.z, p2_.z - p0_.z, kn1_.z - p0_.z, kn2_.z - p0_.z,
                         nkn1_.z - p0_.z, nkn2_.z - p0_.z, nkn1_.z - p0_.z, nkn2_.z - p0_.z, 'z')

        me.update()
        bm.free()
        bpy.ops.object.mode_set(mode='EDIT')

    return True


# ==========================================================================

class RCAD_OT_SpreadLoop(bpy.types.Operator):
    """Spread vertices along a loop with optional spline shaping"""
    bl_label = "Spread Loop"
    bl_idname = "rcad.spread_loop"
    bl_options = {'REGISTER', 'UNDO'}

    def updateself(self, context):
        bpy.context.window_manager.rcad_1d_props.shape_inf = self.influence * 5

    influence: bpy.props.IntProperty(
        name="Shape",
        description="instance -> spline -> spline 2 -> Basier_mid -> Basier -> instance",
        default=0, min=0, max=50,
        update=updateself)

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(self, 'influence')

    def execute(self, context):
        ibc = bpy.context.window_manager.rcad_1d_props
        main_spread(self, context, (ibc.spread_x, ibc.spread_y, ibc.spread_z, ibc.uniform), self.influence * 5)
        return {'FINISHED'}


classes = (RCAD_OT_SpreadLoop,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
