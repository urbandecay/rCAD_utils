# hole_in_mesh.py — Dispatch punched-hole resampling across supported hole types.

from ..debug import (
    begin_debug_operation,
    debug_log,
    debug_separator,
    end_debug_operation,
    mesh_stats,
)
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


def detect(bm):
    data = hole_punch_solid.detect(bm)
    if data is not None:
        return data

    data = hole_punch_face.detect(bm)
    if data is not None:
        debug_log(
            "hole_detect",
            "Using flat-face hole detection path.",
            groups=len(data['groups']),
        )
        return data

    return None


def execute(bm, obj, direction, report=None, data=None):
    op_id = begin_debug_operation("hole_resample")
    if data is None:
        data = detect(bm)
    if not data:
        end_debug_operation(op_id, "hole_resample")
        debug_separator()
        return {'CANCELLED'}
    if not data['groups']:
        if report is not None:
            report({'ERROR'}, "Could not detect a valid punched hole.")
        end_debug_operation(op_id, "hole_resample")
        debug_separator()
        return {'CANCELLED'}

    if data['invalid_components'] and report is not None:
        report(
            {'WARNING'},
            f"Skipped {data['invalid_components']} invalid hole component(s).",
        )

    debug_log(
        "hole_execute",
        "Executing hole-in-mesh resample groups.",
        direction=direction,
        group_count=len(data['groups']),
        invalid_components=data['invalid_components'],
        mesh=mesh_stats(bm),
    )

    for group_data in data['groups']:
        debug_log(
            "hole_execute",
            "Dispatching aligned loop logic for hole group.",
            ring_count=len(group_data['rings'][0]),
            ring_sizes=[len(loop) for loop in group_data['rings'][0]],
            use_seams=group_data.get('use_seams', True),
            migrate_seams=group_data.get('migrate_seams'),
            max_seams=group_data.get('max_seams'),
        )
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
    end_debug_operation(op_id, "hole_resample")
    debug_separator()
    return {'FINISHED'}
