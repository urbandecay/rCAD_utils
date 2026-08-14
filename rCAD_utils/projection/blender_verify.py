"""Background-Blender verification for the rebuilt projection tool.

Run from the rCAD_utils directory with:
    blender --background --factory-startup --python projection/blender_verify.py
"""

import math
from pathlib import Path
import sys

import bmesh
import bpy
from bpy.props import BoolProperty, PointerProperty
from mathutils import Vector


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from rCAD_utils.projection.engine import (  # noqa: E402
    ProjectionError,
    build_surface_components,
    project_points_coherently,
)
from rCAD_utils.projection import operators  # noqa: E402
from rCAD_utils.projection.storage import projection_state  # noqa: E402
from rCAD_utils.weld_tools.ui import FuseGeometryProps  # noqa: E402
from rCAD_utils.weld_tools import face_weld_op, heavy_weld_op, x_weld_op  # noqa: E402


TOLERANCE = 1.0e-5


def assert_near(actual, expected, message=""):
    distance = (Vector(actual) - Vector(expected)).length
    assert distance <= TOLERANCE, f"{message} distance {distance}: {actual} != {expected}"


def expect_projection_error(callback):
    try:
        callback()
    except ProjectionError:
        return
    raise AssertionError("Expected ProjectionError")


def verify_flat_surface():
    vertices = [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)]
    components = build_surface_components(vertices, [(0, 1, 2, 3)])
    result = project_points_coherently(
        [(-1, 0.5, 3), (1, -0.5, -2)], components, Vector((0, 0, 1))
    )
    assert_near(result.locations[0], (-1, 0.5, 0), "flat point 0")
    assert_near(result.locations[1], (1, -0.5, 0), "flat point 1")


def verify_angled_surface():
    # z = x, with points offset along the plane normal.
    vertices = [(-2, -2, -2), (2, -2, 2), (2, 2, 2), (-2, 2, -2)]
    normal = Vector((-1, 0, 1)).normalized()
    expected = [Vector((-1, -0.5, -1)), Vector((1, 0.75, 1))]
    points = [point + normal * distance for point, distance in zip(expected, (1.25, -0.8))]
    result = project_points_coherently(
        points,
        build_surface_components(vertices, [(0, 1, 2, 3)]),
        normal,
    )
    for index, expected_point in enumerate(expected):
        assert_near(result.locations[index], expected_point, f"angled point {index}")


def verify_horizontal_surface():
    vertices = [(0, -2, -2), (0, 2, -2), (0, 2, 2), (0, -2, 2)]
    components = build_surface_components(vertices, [(0, 1, 2, 3)])
    result = project_points_coherently(
        [(3, -0.5, 1.25), (-2, 0.75, -1.5)],
        components,
        Vector((1, 0, 0)),
    )
    assert_near(result.locations[0], (0, -0.5, 1.25), "horizontal point 0")
    assert_near(result.locations[1], (0, 0.75, -1.5), "horizontal point 1")


def verify_vertical_surface():
    vertices = [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)]
    components = build_surface_components(vertices, [(0, 1, 2, 3)])
    result = project_points_coherently(
        [(-0.5, 1.25, 3), (0.75, -1.5, -2)],
        components,
        Vector((0, 0, 1)),
    )
    assert_near(result.locations[0], (-0.5, 1.25, 0), "vertical point 0")
    assert_near(result.locations[1], (0.75, -1.5, 0), "vertical point 1")


def verify_separated_surfaces_are_coherent():
    vertices = [
        (-3, -1, 0), (-1, -1, 0), (-1, 1, 0), (-3, 1, 0),
        (1, -1, 0), (3, -1, 0), (3, 1, 0), (1, 1, 0),
    ]
    components = build_surface_components(vertices, [(0, 1, 2, 3), (4, 5, 6, 7)])
    assert len(components) == 2
    # Each source is directly over a different island. The shared direction is
    # coherent even though the disconnected target requires two islands.
    result = project_points_coherently(
        [(-2, 0, 1), (2, 0, 1)], components, Vector((0, 0, 1))
    )
    assert result.component_index == -1
    assert_near(result.locations[0], (-2, 0, 0), "separated point 0")
    assert_near(result.locations[1], (2, 0, 0), "separated point 1")


def verify_multifaced_bevel_surface():
    # Three connected strips: lower flat, angled bevel, upper flat.
    vertices = [
        (-2, -1, 0), (-2, 1, 0),
        (-1, -1, 0), (-1, 1, 0),
        (1, -1, 1), (1, 1, 1),
        (2, -1, 1), (2, 1, 1),
    ]
    polygons = [(0, 2, 3, 1), (2, 4, 5, 3), (4, 6, 7, 5)]
    components = build_surface_components(vertices, polygons)
    assert len(components) == 1

    bevel_normal = Vector((-0.5, 0, 1)).normalized()
    expected = [Vector((-1.5, 0, 0)), Vector((0, 0, 0.5)), Vector((1.5, 0, 1))]
    points = [
        expected[0] + Vector((0, 0, 0.2)),
        expected[1] + bevel_normal * 0.2,
        expected[2] + Vector((0, 0, 0.2)),
    ]
    result = project_points_coherently(points, components, Vector((0, 0, 1)))
    expected_vertical = [
        expected[0],
        Vector((points[1].x, 0, 0.5 * points[1].x + 0.5)),
        expected[2],
    ]
    for index, expected_point in enumerate(expected_vertical):
        assert_near(result.locations[index], expected_point, f"bevel point {index}")


def verify_invalid_engine_inputs():
    expect_projection_error(lambda: build_surface_components([], []))
    expect_projection_error(
        lambda: build_surface_components([(0, 0, 0), (1, 0, 0), (2, 0, 0)], [(0, 1, 2)])
    )
    component = build_surface_components(
        [(-1, -1, 0), (1, -1, 0), (1, 1, 0), (-1, 1, 0)],
        [(0, 1, 2, 3)],
    )
    expect_projection_error(
        lambda: project_points_coherently([], component, Vector((0, 0, 1)))
    )
    expect_projection_error(
        lambda: project_points_coherently(
            [(math.nan, 0, 0)], component, Vector((0, 0, 1))
        )
    )


def clear_scene():
    if bpy.context.object is not None and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    projection_state.clear()


def mesh_object(name, vertices, faces):
    mesh = bpy.data.meshes.new(f"{name}Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def select_only(obj):
    for selected in list(bpy.context.selected_objects):
        selected.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def edit_vertex_locations(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    return [vertex.co.copy() for vertex in bm.verts]


def verify_visible_object_target_discovery():
    clear_scene()
    source = mesh_object("VisibleSource", [(0, 0, 2)], [])
    mesh_object(
        "VisibleObjectTarget",
        [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
        [(0, 1, 2, 3)],
    )
    select_only(source)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(source.data)
    bm.verts.ensure_lookup_table()
    bm.verts[0].select_set(True)
    bmesh.update_edit_mesh(source.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}

    vertices, polygons = operators._visible_target_polygon_soup(bpy.context)
    assert len(vertices) == 4
    assert polygons == [(0, 1, 2, 3)]


def verify_same_object_unselected_target_discovery():
    clear_scene()
    obj = mesh_object(
        "SharedSourceAndTarget",
        [(0, 0, 2), (-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
        [(1, 2, 3, 4)],
    )
    select_only(obj)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.verts[0].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}

    vertices, polygons = operators._visible_target_polygon_soup(bpy.context)
    assert len(vertices) == 4
    assert polygons == [(0, 1, 2, 3)]


def verify_operator_workflow_and_undo():
    clear_scene()
    source = mesh_object("Source", [(-1, -0.5, 2), (0, 0.5, 2), (1, -0.5, 2)], [])
    target = mesh_object(
        "Target",
        [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
        [(0, 1, 2, 3)],
    )

    select_only(source)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(source.data)
    bm.verts.ensure_lookup_table()
    for vertex in bm.verts:
        vertex.select_set(True)
    bm.select_history.add(bm.verts[1])
    bmesh.update_edit_mesh(source.data, loop_triangles=False, destructive=False)
    before = edit_vertex_locations(source)

    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}
    assert not any(vertex.select for vertex in bmesh.from_edit_mesh(source.data).verts)

    bpy.ops.object.mode_set(mode='OBJECT')
    select_only(target)
    assert bpy.ops.mesh.rcad_store_projection_target() == {'FINISHED'}
    assert bpy.context.mode == 'EDIT_MESH'
    assert bpy.context.view_layer.objects.active == source

    # Background Blender starts with Undo disabled until the first explicit push.
    assert bpy.ops.ed.undo_push(message="Projection verification base") == {'FINISHED'}
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'FINISHED'}
    projected = edit_vertex_locations(source)
    for index, location in enumerate(projected):
        assert_near(location, (before[index].x, before[index].y, 0), f"operator point {index}")

    bm = bmesh.from_edit_mesh(source.data)
    assert all(vertex.select for vertex in bm.verts), "Stored source selection was not restored"
    assert bm.select_history.active == bm.verts[1], "Active source element was not restored"

    # Background invocation does not perform the UI event loop's automatic
    # push for an operator carrying bl_options={'UNDO'}.
    assert 'UNDO' in operators.RCAD_OT_ProjectStoredGeometry.bl_options
    assert bpy.ops.ed.undo_push(message="Projection verification projected") == {'FINISHED'}
    assert bpy.ops.ed.undo() == {'FINISHED'}
    assert bpy.context.mode == 'EDIT_MESH'
    undone = edit_vertex_locations(source)
    for index, location in enumerate(undone):
        assert_near(location, before[index], f"undo point {index}")


def verify_explicit_face_target_workflow():
    clear_scene()
    obj = mesh_object(
        "CombinedSourceAndTarget",
        [
            (-1, 0, 2), (0, 0, 2), (1, 0, 2),
            (-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0),
        ],
        [(3, 4, 5, 6)],
    )
    select_only(obj)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    for index in (0, 1, 2):
        bm.verts[index].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.faces[0].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_target() == {'FINISHED'}
    assert projection_state.target.is_face_selection
    assert projection_state.target.face_indices == (0,)
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    for index in (0, 1, 2):
        assert_near(bm.verts[index].co, (index - 1, 0, 0), f"face target point {index}")
        assert bm.verts[index].select
    assert not bm.faces[0].select, "Target face remained selected instead of restoring source"


def verify_select_target_then_project_workflow():
    clear_scene()
    obj = mesh_object(
        "DirectSelection",
        [(-1, 0, 2), (0, 0, 2), (1, 0, 2),
         (-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
        [(3, 4, 5, 6)],
    )
    select_only(obj)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    for index in (0, 1, 2):
        bm.verts[index].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.faces[0].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    # No Store Target call: Project consumes the current target-face selection.
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'FINISHED'}
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    for index in (0, 1, 2):
        assert_near(bm.verts[index].co, (index - 1, 0, 0), f"direct target point {index}")


def verify_separate_face_target_returns_to_source():
    clear_scene()
    source = mesh_object("FaceSource", [(-0.5, 0, 1), (0.5, 0, 1)], [])
    target = mesh_object(
        "FaceTarget",
        [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
        [(0, 1, 2, 3)],
    )
    select_only(source)
    bpy.ops.object.mode_set(mode='EDIT')
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}
    bpy.ops.object.mode_set(mode='OBJECT')

    select_only(target)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(target.data)
    bm.faces.ensure_lookup_table()
    bm.faces[0].select_set(True)
    bmesh.update_edit_mesh(target.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_target() == {'FINISHED'}
    assert bpy.context.mode == 'EDIT_MESH'
    assert bpy.context.view_layer.objects.active == source
    assert source.mode == 'EDIT' and target.mode == 'OBJECT'
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'FINISHED'}
    for index, location in enumerate(edit_vertex_locations(source)):
        assert_near(location, ((index - 0.5), 0, 0), f"separate face point {index}")


def verify_topology_change_is_rejected():
    clear_scene()
    source = mesh_object("ChangingSource", [(0, 0, 2)], [])
    target = mesh_object(
        "StableTarget",
        [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)],
        [(0, 1, 2, 3)],
    )
    select_only(source)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(source.data)
    bm.verts.ensure_lookup_table()
    bm.verts[0].select_set(True)
    bmesh.update_edit_mesh(source.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}
    bpy.ops.object.mode_set(mode='OBJECT')
    select_only(target)
    assert bpy.ops.mesh.rcad_store_projection_target() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(source.data)
    bm.verts.new((1, 0, 2))
    bmesh.update_edit_mesh(source.data, loop_triangles=False, destructive=True)
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'CANCELLED'}
    bm = bmesh.from_edit_mesh(source.data)
    bm.verts.ensure_lookup_table()
    assert_near(bm.verts[0].co, (0, 0, 2), "rejected projection changed source")


def verify_projection_weld_sequence():
    """The projection owns the fixed Heavy -> X -> Square orchestration."""
    clear_scene()
    obj = mesh_object(
        "ProjectionWeld",
        [
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
            (-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0),
        ],
        [(0, 1, 2, 3), (4, 5, 6, 7)],
    )
    select_only(obj)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.faces[0].select_set(True)
    for vertex in bm.faces[0].verts:
        vertex.select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
    assert bpy.ops.mesh.rcad_store_projection_source() == {'FINISHED'}

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.faces[1].select_set(True)
    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)

    bpy.context.scene.rcad_projection_weld = True
    assert bpy.ops.mesh.rcad_project_stored_geometry() == {'FINISHED'}
    # The projection's final reselection must leave the result selected even
    # though every weld operator clears its own welded elements.
    bm = bmesh.from_edit_mesh(obj.data)
    assert any(vertex.select for vertex in bm.verts)


def main():
    registered = []
    try:
        for cls in operators.classes:
            bpy.utils.register_class(cls)
            registered.append(cls)
        bpy.types.Scene.rcad_projection_weld = BoolProperty(default=False)
        bpy.utils.register_class(FuseGeometryProps)
        bpy.types.Scene.super_fuse = PointerProperty(type=FuseGeometryProps)
        for module in (heavy_weld_op, x_weld_op, face_weld_op):
            module.register()
        verify_flat_surface()
        verify_angled_surface()
        verify_horizontal_surface()
        verify_vertical_surface()
        verify_separated_surfaces_are_coherent()
        verify_multifaced_bevel_surface()
        verify_invalid_engine_inputs()
        verify_visible_object_target_discovery()
        verify_same_object_unselected_target_discovery()
        verify_operator_workflow_and_undo()
        verify_explicit_face_target_workflow()
        verify_select_target_then_project_workflow()
        verify_separate_face_target_returns_to_source()
        verify_topology_change_is_rejected()
        verify_projection_weld_sequence()
        print("PROJECTION_VERIFICATION_OK")
    finally:
        projection_state.clear()
        for module in (face_weld_op, x_weld_op, heavy_weld_op):
            module.unregister()
        if hasattr(bpy.types.Scene, "super_fuse"):
            del bpy.types.Scene.super_fuse
        bpy.utils.unregister_class(FuseGeometryProps)
        if hasattr(bpy.types.Scene, "rcad_projection_weld"):
            del bpy.types.Scene.rcad_projection_weld
        for cls in reversed(registered):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    main()
