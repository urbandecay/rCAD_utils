"""Background-Blender verification for Tangify.

Run from the rCAD_utils directory with:
    blender --background --factory-startup --python vertex_resampler/tangify_blender_verify.py
"""

import math
from pathlib import Path
import sys

import bmesh
import bpy


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from rCAD_utils import panel  # noqa: E402
from rCAD_utils.vertex_resampler import math_engine, operators, ui  # noqa: E402


def _circle_points(center_x, center_y, radius, count):
    return [
        (
            center_x + radius * math.cos(2.0 * math.pi * index / count),
            center_y + radius * math.sin(2.0 * math.pi * index / count),
            0.0,
        )
        for index in range(count)
    ]


def _loop_edges(start, count):
    return [
        (start + index, start + (index + 1) % count)
        for index in range(count)
    ]


def _smooth_tangent_dot(loop_coords, endpoint, line_direction):
    spline = math_engine.CatmullRomSpline(loop_coords, is_closed=True)
    parameter, _point, distance = spline.project(endpoint)
    assert distance <= 1.0e-5, f"Guide endpoint missed its loop by {distance}"
    return abs(spline.tangent_global(parameter).dot(line_direction))


def _anchor_index(coords, point):
    return min(
        range(len(coords)),
        key=lambda index: (coords[index] - point).length_squared,
    )


def _assert_even_arc_spacing(coords, anchor_indices):
    first, second = sorted(anchor_indices)
    paths = (
        list(range(first, second)),
        list(range(second, len(coords))) + list(range(0, first)),
    )
    for path in paths:
        lengths = [
            (coords[index] - coords[(index + 1) % len(coords)]).length
            for index in path
        ]
        assert lengths and max(lengths) / min(lengths) <= 1.12, lengths


def verify_two_loops_two_lines():
    count = 32
    left = _circle_points(-3.0, 0.0, 1.0, count)
    right = _circle_points(3.0, 0.0, 1.0, count)
    vertices = left + right
    edges = _loop_edges(0, count) + _loop_edges(count, count)

    top_left = count // 4
    top_right = count + count // 4
    bottom_left = 3 * count // 4
    bottom_right = count + 3 * count // 4
    guide_indices = []
    for left_index, right_index in (
        (top_left, top_right),
        (bottom_left, bottom_right),
    ):
        start = len(vertices)
        vertices.extend((vertices[left_index], vertices[right_index]))
        edges.append((start, start + 1))
        guide_indices.append((start, start + 1))

    mesh = bpy.data.meshes.new("TangifyVerifyMesh")
    mesh.from_pydata(vertices, edges, [])
    mesh.update()
    obj = bpy.data.objects.new("TangifyVerify", mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')

    bm = bmesh.from_edit_mesh(mesh)
    bm.verts.ensure_lookup_table()
    before_counts = (len(bm.verts), len(bm.edges), len(bm.faces))
    before_guides = [
        (bm.verts[start].co.copy(), bm.verts[end].co.copy())
        for start, end in guide_indices
    ]
    assert bpy.ops.rcad.tangify() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(mesh)
    bm.verts.ensure_lookup_table()
    after_counts = (len(bm.verts), len(bm.edges), len(bm.faces))
    assert after_counts == before_counts, (before_counts, after_counts)
    for (start, end), (before_start, before_end) in zip(
        guide_indices,
        before_guides,
    ):
        assert (bm.verts[start].co - before_start).length <= 1.0e-8
        assert (bm.verts[end].co - before_end).length <= 1.0e-8

    loop_coords = [
        [bm.verts[index].co.copy() for index in range(count)],
        [bm.verts[count + index].co.copy() for index in range(count)],
    ]
    for start_index, end_index in guide_indices:
        start = bm.verts[start_index].co.copy()
        end = bm.verts[end_index].co.copy()
        direction = (end - start).normalized()
        left_dot = _smooth_tangent_dot(loop_coords[0], start, direction)
        right_dot = _smooth_tangent_dot(loop_coords[1], end, direction)
        assert left_dot >= 0.999, left_dot
        assert right_dot >= 0.999, right_dot

    _assert_even_arc_spacing(
        loop_coords[0],
        [
            _anchor_index(loop_coords[0], before_guides[0][0]),
            _anchor_index(loop_coords[0], before_guides[1][0]),
        ],
    )
    _assert_even_arc_spacing(
        loop_coords[1],
        [
            _anchor_index(loop_coords[1], before_guides[0][1]),
            _anchor_index(loop_coords[1], before_guides[1][1]),
        ],
    )


def verify_four_loop_contacts():
    count = 32
    circles = [
        _circle_points(0.0, 0.0, 2.0, count),
        _circle_points(-2.8, 0.0, 0.8, count),
        _circle_points(2.8, 0.0, 0.8, count),
        _circle_points(0.0, -2.8, 0.8, count),
    ]
    vertices = [point for circle in circles for point in circle]
    edges = [
        edge
        for circle_index in range(len(circles))
        for edge in _loop_edges(circle_index * count, count)
    ]

    mesh = bpy.data.meshes.new("TangifyContactsVerifyMesh")
    mesh.from_pydata(vertices, edges, [])
    mesh.update()
    obj = bpy.data.objects.new("TangifyContactsVerify", mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')

    bm = bmesh.from_edit_mesh(mesh)
    before_counts = (len(bm.verts), len(bm.edges), len(bm.faces))
    assert bpy.ops.rcad.tangify() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(mesh)
    bm.verts.ensure_lookup_table()
    after_counts = (len(bm.verts), len(bm.edges), len(bm.faces))
    assert after_counts == before_counts, (before_counts, after_counts)
    loop_coords = [
        [bm.verts[offset + index].co.copy() for index in range(count)]
        for offset in range(0, len(vertices), count)
    ]

    # The minimum spanning contacts should connect the large central loop to
    # each of the three surrounding loops.
    for outer_index in (1, 2, 3):
        closest = min(
            (
                (first - second).length,
                first,
                second,
            )
            for first in loop_coords[0]
            for second in loop_coords[outer_index]
        )
        distance, point_a, point_b = closest
        assert distance <= 1.0e-6, distance
        tangent_a = math_engine.CatmullRomSpline(
            loop_coords[0], is_closed=True
        ).tangent_global(
            math_engine.CatmullRomSpline(loop_coords[0], is_closed=True).project(point_a)[0]
        )
        tangent_b = math_engine.CatmullRomSpline(
            loop_coords[outer_index], is_closed=True
        ).tangent_global(
            math_engine.CatmullRomSpline(loop_coords[outer_index], is_closed=True).project(point_b)[0]
        )
        assert abs(tangent_a.dot(tangent_b)) >= 0.999, (outer_index, tangent_a, tangent_b)


def verify_single_perpendicular_line():
    count = 32
    circle = _circle_points(0.0, 0.0, 1.0, count)
    vertices = circle + [(-3.0, 0.0, 0.0), (-1.0, 0.0, 0.0)]
    edges = _loop_edges(0, count) + [(count, count + 1)]

    mesh = bpy.data.meshes.new("TangifyNormalVerifyMesh")
    mesh.from_pydata(vertices, edges, [])
    mesh.update()
    obj = bpy.data.objects.new("TangifyNormalVerify", mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')

    bm = bmesh.from_edit_mesh(mesh)
    bm.verts.ensure_lookup_table()
    before_line = [bm.verts[count].co.copy(), bm.verts[count + 1].co.copy()]
    assert bpy.ops.rcad.tangify() == {'FINISHED'}
    bm = bmesh.from_edit_mesh(mesh)
    bm.verts.ensure_lookup_table()
    assert (bm.verts[count].co - before_line[0]).length <= 1.0e-8
    assert (bm.verts[count + 1].co - before_line[1]).length <= 1.0e-8
    contact = bm.verts[count + 1].co.copy()
    line_direction = (bm.verts[count + 1].co - bm.verts[count].co).normalized()
    loop_coords = [bm.verts[index].co.copy() for index in range(count)]
    spline = math_engine.CatmullRomSpline(loop_coords, is_closed=True)
    parameter, _point, distance = spline.project(contact)
    assert distance <= 1.0e-5, distance
    dot = abs(spline.tangent_global(parameter).dot(line_direction))
    assert dot <= 5.0e-3


if __name__ == "__main__":
    bpy.utils.register_class(panel.RCAD_PT_Main)
    operators.register()
    ui.register()
    try:
        verify_two_loops_two_lines()
        bpy.ops.object.mode_set(mode='OBJECT')
        verify_four_loop_contacts()
        bpy.ops.object.mode_set(mode='OBJECT')
        verify_single_perpendicular_line()
        print("Tangify Blender verification passed")
    finally:
        ui.unregister()
        operators.unregister()
        bpy.utils.unregister_class(panel.RCAD_PT_Main)
