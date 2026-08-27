"""Focused background-Blender checks for Edge Project & Knife direction math.

Run from the rCAD_utils directory with:
    blender --background --factory-startup --python edge_knife_project/blender_verify.py
"""

from pathlib import Path
import sys

import bmesh
import bpy
from mathutils import Vector


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from rCAD_utils.edge_knife_project import operators  # noqa: E402


TOLERANCE = 1.0e-6


def assert_direction(actual, expected, message):
    distance = (Vector(actual) - Vector(expected)).length
    assert distance <= TOLERANCE, f"{message}: {actual} != {expected}"


def make_box_with_cutter():
    mesh = bpy.data.meshes.new("EdgeKnifeDirectionMesh")
    obj = bpy.data.objects.new("EdgeKnifeDirection", mesh)
    bpy.context.scene.collection.objects.link(obj)

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=2.0)
    first = bm.verts.new((2.0, -0.5, 0.5))
    second = bm.verts.new((2.0, 0.5, 0.5))
    cutter = bm.edges.new((first, second))
    bm.edges.index_update()
    cutter_index = cutter.index
    bm.to_mesh(mesh)
    bm.free()
    return obj, cutter_index


def verify_nearest_surface_avoids_centroid_tilt():
    obj, cutter_index = make_box_with_cutter()
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    cutter = bm.edges[cutter_index]
    source_points = [obj.matrix_world @ vert.co for vert in cutter.verts]
    target_faces = list(bm.faces)
    target_points = operators._world_face_points(obj, target_faces)

    direction = operators._choose_view_direction(
        None,
        obj,
        source_points,
        target_faces,
        target_points,
    )
    assert_direction(direction, (-1.0, 0.0, 0.0), "box-side projection tilted")
    bm.free()


def verify_transformed_target_uses_world_space():
    obj, cutter_index = make_box_with_cutter()
    obj.rotation_euler = (0.0, 0.0, 0.35)
    obj.scale = (1.5, 0.75, 2.0)
    bpy.context.view_layer.update()

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    cutter = bm.edges[cutter_index]
    source_points = [obj.matrix_world @ vert.co for vert in cutter.verts]
    target_faces = list(bm.faces)
    target_points = operators._world_face_points(obj, target_faces)
    expected = obj.matrix_world.to_3x3() @ Vector((-1.0, 0.0, 0.0))
    expected.normalize()

    direction = operators._choose_view_direction(
        None,
        obj,
        source_points,
        target_faces,
        target_points,
    )
    assert_direction(direction, expected, "transformed target direction")
    bm.free()


def verify_preview_cut_line():
    obj, cutter_index = make_box_with_cutter()
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    cutter = bm.edges[cutter_index]
    cutter_segments = [
        (obj.matrix_world @ cutter.verts[0].co,
         obj.matrix_world @ cutter.verts[1].co),
    ]
    target_faces = list(bm.faces)
    direction = Vector((-1.0, 0.0, 0.0))
    preview = operators._preview_projected_segments(
        obj,
        target_faces,
        cutter_segments,
        direction,
        1.0e-6,
    )
    assert preview, "preview did not find the projected box-side cut"
    for point in preview:
        assert abs(abs(point.x) - 1.0) <= TOLERANCE, point
        assert abs(point.z - 0.5) <= TOLERANCE, point
    bm.free()


def verify_existing_straight_seam_is_split():
    mesh = bpy.data.meshes.new("ExistingSeamMesh")
    obj = bpy.data.objects.new("ExistingSeam", mesh)
    bpy.context.scene.collection.objects.link(obj)

    bm = bmesh.new()
    bottom_left = bm.verts.new((0.0, -1.0, -1.0))
    bottom_right = bm.verts.new((0.0, 1.0, -1.0))
    seam_right = bm.verts.new((0.0, 1.0, 0.5))
    seam_left = bm.verts.new((0.0, -1.0, 0.5))
    top_right = bm.verts.new((0.0, 1.0, 1.0))
    top_left = bm.verts.new((0.0, -1.0, 1.0))
    lower = bm.faces.new((bottom_left, bottom_right, seam_right, seam_left))
    upper = bm.faces.new((seam_left, seam_right, top_right, top_left))
    cutter_first = bm.verts.new((2.0, -1.0, 0.5))
    cutter_second = bm.verts.new((2.0, 1.0, 0.5))
    bm.edges.new((cutter_first, cutter_second))
    bm.to_mesh(mesh)
    bm.free()

    check = bmesh.new()
    check.from_mesh(mesh)
    check.faces.ensure_lookup_table()
    check.edges.ensure_lookup_table()
    target_faces = list(check.faces)[:2]
    lower, upper = target_faces
    seam_edge = next(
        edge
        for edge in check.edges
        if len(edge.link_faces) == 2
        and all(abs(vert.co.z - 0.5) <= TOLERANCE for vert in edge.verts)
    )
    cutter_edge = next(
        edge
        for edge in check.edges
        if all(abs(vert.co.x - 2.0) <= TOLERANCE for vert in edge.verts)
    )
    cutter_segments = [
        (obj.matrix_world @ cutter_edge.verts[0].co,
         obj.matrix_world @ cutter_edge.verts[1].co),
    ]
    seams = operators._projected_seam_edges(
        obj,
        check.edges,
        cutter_segments,
        Vector((-1.0, 0.0, 0.0)),
        1.0e-6,
    )
    assert seam_edge in seams, "existing straight seam was not detected"
    bmesh.ops.split_edges(check, edges=seams)
    assert not set(lower.verts) & set(upper.verts), "existing seam stayed connected"
    check.free()


def main():
    verify_nearest_surface_avoids_centroid_tilt()
    verify_transformed_target_uses_world_space()
    verify_preview_cut_line()
    verify_existing_straight_seam_is_split()
    print("EDGE_KNIFE_PROJECT_VERIFICATION_OK")


if __name__ == "__main__":
    main()
