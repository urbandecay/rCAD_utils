# hole_in_mesh.py — Dispatch punched-hole resampling across supported hole types.

from . import hole_punch_face, hole_punch_solid
from .resample_common import execute_aligned_loops_logic


def _cleanup_selected_ring_edges(bm, cleanup_edges, ring_edges):
    if not cleanup_edges:
        return

    ring_vert_set = set()
    for edge in ring_edges:
        if not edge.is_valid:
            continue
        edge.select = True
        ring_vert_set.update(edge.verts)

    touched_non_ring_verts = set()
    for edge in cleanup_edges:
        if not edge.is_valid:
            continue
        edge.select = False
        for vert in edge.verts:
            if vert not in ring_vert_set:
                touched_non_ring_verts.add(vert)

    for vert in ring_vert_set:
        if vert.is_valid:
            vert.select = True

    for vert in touched_non_ring_verts:
        if not vert.is_valid:
            continue
        if any(edge.select for edge in vert.link_edges):
            continue
        vert.select = False

    bm.select_flush_mode()


def detect(bm, report=None):
    data = hole_punch_solid.detect(bm, report=report)
    if data is not None:
        return data

    return hole_punch_face.detect(bm, report=report)


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm, report=report)
    if not data:
        return {'CANCELLED'}
    if not data['groups']:
        if report is not None:
            report({'ERROR'}, "Could not detect a valid punched hole.")
        return {'CANCELLED'}

    if data['invalid_components'] and report is not None:
        report(
            {'WARNING'},
            f"Skipped {data['invalid_components']} invalid hole component(s).",
        )

    for group_data in data['groups']:
        if any(
            not vert.is_valid
            for loop in group_data['rings'][0]
            for vert in loop
        ):
            if report is not None:
                report({'WARNING'}, "Skipped stale hole ring after mesh update.")
            continue
        _cleanup_selected_ring_edges(
            bm,
            group_data.get('cleanup_edges', set()),
            group_data.get('cleanup_ring_edges', set()),
        )
        execute_aligned_loops_logic(
            bm,
            obj,
            group_data['rings'],
            direction,
            report=report,
            use_seams=group_data.get('use_seams', True),
            migrate_seams=group_data.get('migrate_seams'),
            max_seams=group_data.get('max_seams'),
        )
    return {'FINISHED'}
