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


def main():
    verify_nearest_surface_avoids_centroid_tilt()
    verify_transformed_target_uses_world_space()
    print("EDGE_KNIFE_PROJECT_VERIFICATION_OK")


if __name__ == "__main__":
    main()
