"""Run with Blender --background --factory-startup --python this_file.py."""

import math
from pathlib import Path
import sys

import bmesh
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carve_along_path
from carve_along_path.extrude import cap_buf
from carve_along_path.extrude_along_path import extrude
from carve_along_path.profile_extrusion import extrude_profile_endpoints


def carve_profile(manual, select_mode):
    if bpy.context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    bpy.context.tool_settings.mesh_select_mode = select_mode
    bpy.ops.mesh.primitive_cube_add(size=2)
    obj = bpy.context.object
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    # Other disconnected geometry must not reverse either supporting normal.
    distant = bmesh.ops.create_cube(bm, size=2)['verts']
    bmesh.ops.translate(bm, verts=distant, vec=Vector((-10, 0, 10)))
    path = [bm.verts.new(co) for co in
            [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]]
    for a, b in zip(path, path[1:] + path[:1]):
        bm.edges.new((a, b))
    profile = [bm.verts.new((-1 + .6 * math.sin(i * math.pi / 20), 0,
                            1 - .6 * math.cos(i * math.pi / 20)))
               for i in range(11)]
    for a, b in zip(profile, profile[1:]):
        bm.edges.new((a, b))
    for face in bm.faces:
        face.select_set(False)
    for edge in bm.edges:
        edge.select_set(False)
    for vertex in bm.verts:
        vertex.select_set(False)
    for vertex in profile:
        vertex.select_set(True)
    bm.verts.index_update()
    probe = bm.copy()
    original_count = len(probe.verts)
    extrude_profile_endpoints(probe, [v.index for v in profile], obj.matrix_world)
    assert len(probe.verts) == original_count + 2
    added = list(probe.verts)[original_count:]
    left = min(added, key=lambda v: v.co.x)
    top = max(added, key=lambda v: v.co.z)
    assert left.co.x < -1 and top.co.z > 1
    assert abs(left.co.z - profile[0].co.z) < 1e-6
    assert abs(top.co.x - profile[-1].co.x) < 1e-6
    probe.verts.ensure_lookup_table()
    assert all((probe.verts[v.index].co - v.co).length == 0 for v in profile)
    probe.free()
    cap_buf.list_ek = [[a.index, b.index] for a, b in zip(path, path[1:] + path[:1])]
    cap_buf.list_sp = [path[0].index]
    cap_buf.mesh_name = obj.data.name
    if manual:
        for endpoint, offset in [(profile[0], (-.04, 0, 0)),
                                 (profile[-1], (0, 0, .04))]:
            vertex = bm.verts.new(endpoint.co + Vector(offset))
            vertex.select_set(True)
            bm.edges.new((endpoint, vertex)).select_set(True)
    for edge in bm.edges:
        edge.select_set(all(v.select for v in edge.verts))
    bm.to_mesh(obj.data)
    bm.free()
    bpy.ops.object.mode_set(mode='EDIT')
    assert bpy.ops.mesh.cap_carve(keep_profile=False) == {'FINISHED'}
    bpy.ops.object.mode_set(mode='OBJECT')
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    volume = abs(bm.calc_volume())
    counts = (len(bm.verts), len(bm.edges), len(bm.faces))
    coordinates = tuple(sorted(tuple(round(c, 5) for c in v.co) for v in bm.verts))
    bm.free()
    assert counts[2] > 12 and 0 < volume < 15.99, (volume, counts)
    return volume, (counts, coordinates)


def main():
    carve_along_path.register()
    fill_faces = extrude.fill_faces

    def check_sweep(bm, endpoints, loop, shifted, path, original=None, **kwargs):
        indices = set(original or shifted)
        edges = [e for e in bm.edges if all(v.index in indices for v in e.verts)]
        assert all(e.select for e in edges), 'The sweep lost profile edges'
        before = set(bm.faces)
        result = fill_faces(bm, endpoints, loop, shifted, path, original, **kwargs)
        expected = len(edges) * (len(path) if loop else len(path) - 1)
        assert len(set(bm.faces) - before) == expected
        return result

    extrude.fill_faces = check_sweep
    try:
        for mode in [(True, False, False), (False, True, False), (False, False, True)]:
            automatic, auto_counts = carve_profile(False, mode)
            manual, manual_counts = carve_profile(True, mode)
            assert abs(automatic - manual) < 1e-5
            assert auto_counts == manual_counts
            print('PASS complete profile sweep and manual carve comparison:', mode)
    finally:
        extrude.fill_faces = fill_faces


if __name__ == '__main__':
    main()
