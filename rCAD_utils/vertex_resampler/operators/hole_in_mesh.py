# hole_in_mesh.py — Dispatch punched-hole resampling across supported hole types.

import bmesh

from . import hole_punch_face, hole_punch_solid
from .resample_common import execute_aligned_loops_logic
from ..debug import debug_log, edge_ref, mesh_stats, vert_ref
from ..seam_manager import realign_seams_to_ring_normals


def _safe_debug_attr(item, name, default=None):
    try:
        return getattr(item, name)
    except Exception:
        return default


def _debug_sort_key(item):
    value = _safe_debug_attr(item, "index", -1)
    return value if isinstance(value, int) else -1


def _debug_is_valid(item):
    return bool(_safe_debug_attr(item, "is_valid", False))


def _debug_is_selected(item):
    return bool(_safe_debug_attr(item, "select", False))


def _debug_vert_refs(verts):
    return [
        vert_ref(vert)
        for vert in sorted(verts, key=_debug_sort_key)
    ]


def _debug_edge_refs(edges):
    return [
        edge_ref(edge)
        for edge in sorted(edges, key=_debug_sort_key)
    ]


def _selected_vert_refs(bm):
    return _debug_vert_refs(vert for vert in bm.verts if _debug_is_selected(vert))


def _debug_vert_ids(verts):
    return [
        _safe_debug_attr(vert, "index", "<removed>")
        for vert in sorted(verts, key=_debug_sort_key)
    ]


def _debug_edge_ids(edges):
    return [
        _safe_debug_attr(edge, "index", "<removed>")
        for edge in sorted(edges, key=_debug_sort_key)
    ]


def _debug_report(report, message):
    if report is not None:
        report({'INFO'}, f"Hole debug: {message}")


def _refresh_bmesh_indices(bm):
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.index_update()
    bm.faces.index_update()


def _select_only_ring_edges(bm, ring_groups):
    ring_verts = set()
    ring_edges = set()

    ring_groups = [ring_group for ring_group in ring_groups if ring_group is not None]
    if not ring_groups:
        return

    for ring_group in ring_groups:
        for ring_info in ring_group.rings:
            loop = [vert for vert in ring_info.verts if _debug_is_valid(vert)]
            if len(loop) < 2:
                continue

            ring_verts.update(loop)
            count = len(loop)
            edge_limit = count if ring_info.is_closed else count - 1
            for index in range(edge_limit):
                edge = bm.edges.get([loop[index], loop[(index + 1) % count]])
                if edge is not None and _debug_is_valid(edge):
                    ring_edges.add(edge)

    for face in bm.faces:
        face.select = False
    for edge in bm.edges:
        edge.select = False
    for vert in bm.verts:
        vert.select = False

    for vert in ring_verts:
        vert.select = True
    for edge in ring_edges:
        edge.select = True
        for vert in edge.verts:
            vert.select = True


def _cleanup_selected_ring_edges(bm, cleanup_edges, ring_edges, report=None, group_index=None):
    group_label = "?" if group_index is None else group_index
    if not cleanup_edges:
        debug_log(
            "hole_cleanup",
            "No cleanup edges for this hole group.",
            mesh=mesh_stats(bm),
            ring_edges=_debug_edge_refs(edge for edge in ring_edges if _debug_is_valid(edge)),
        )
        _debug_report(
            report,
            f"group={group_label} cleanup=none selected={len([v for v in bm.verts if _debug_is_selected(v)])}",
        )
        return set()

    ring_vert_set = set()
    for edge in ring_edges:
        if not _debug_is_valid(edge):
            continue
        edge.select = True
        ring_vert_set.update(edge.verts)

    live_cleanup_edges = {
        edge for edge in cleanup_edges
        if _debug_is_valid(edge)
    }
    selected_before_cleanup = {vert for vert in bm.verts if _debug_is_selected(vert)}
    debug_log(
        "hole_cleanup",
        "Before cleanup edge deselection.",
        mesh=mesh_stats(bm),
        ring_edges=_debug_edge_refs(edge for edge in ring_edges if _debug_is_valid(edge)),
        cleanup_edges=_debug_edge_refs(live_cleanup_edges),
        ring_verts=_debug_vert_refs(ring_vert_set),
        selected_verts=_selected_vert_refs(bm),
    )
    _debug_report(
        report,
        f"group={group_label} before cleanup "
        f"ring_verts={_debug_vert_ids(ring_vert_set)} "
        f"cleanup_edges={_debug_edge_ids(live_cleanup_edges)} "
        f"selected={_debug_vert_ids(selected_before_cleanup)}",
    )

    touched_non_ring_verts = set()
    preserve_non_ring_verts = set()
    for edge in live_cleanup_edges:
        edge.select = False
        for vert in edge.verts:
            if vert not in ring_vert_set:
                touched_non_ring_verts.add(vert)
                if vert in selected_before_cleanup:
                    preserve_non_ring_verts.add(vert)

    debug_log(
        "hole_cleanup",
        "After cleanup edges were deselected, before selection flush.",
        mesh=mesh_stats(bm),
        touched_non_ring_verts=_debug_vert_refs(touched_non_ring_verts),
        preserve_non_ring_verts=_debug_vert_refs(preserve_non_ring_verts),
        selected_verts=_selected_vert_refs(bm),
    )
    _debug_report(
        report,
        f"group={group_label} after cleanup edge off "
        f"touched_non_ring={_debug_vert_ids(touched_non_ring_verts)} "
        f"preserve={_debug_vert_ids(preserve_non_ring_verts)} "
        f"selected={_debug_vert_ids(vert for vert in bm.verts if _debug_is_selected(vert))}",
    )

    for vert in ring_vert_set:
        if _debug_is_valid(vert):
            vert.select = True

    for vert in touched_non_ring_verts:
        if not _debug_is_valid(vert):
            continue
        if vert in preserve_non_ring_verts:
            continue
        if any(_debug_is_selected(edge) for edge in vert.link_edges):
            continue
        vert.select = False

    bm.select_flush_mode()
    dropped_by_flush = {
        vert for vert in preserve_non_ring_verts
        if _debug_is_valid(vert) and not _debug_is_selected(vert)
    }
    debug_log(
        "hole_cleanup",
        "After Blender selection flush.",
        mesh=mesh_stats(bm),
        dropped_preserved_verts=_debug_vert_refs(dropped_by_flush),
        selected_verts=_selected_vert_refs(bm),
    )
    _debug_report(
        report,
        f"group={group_label} after flush "
        f"dropped_preserved={_debug_vert_ids(dropped_by_flush)} "
        f"selected={_debug_vert_ids(vert for vert in bm.verts if _debug_is_selected(vert))}",
    )
    for vert in preserve_non_ring_verts:
        if _debug_is_valid(vert):
            vert.select = True
    selected_after_restore = {vert for vert in bm.verts if _debug_is_selected(vert)}
    missing_after_restore = {
        vert for vert in selected_before_cleanup
        if _debug_is_valid(vert) and not _debug_is_selected(vert)
    }
    debug_log(
        "hole_cleanup",
        "After restoring preserved non-ring verts.",
        mesh=mesh_stats(bm),
        restored_non_ring_verts=_debug_vert_refs(
            vert for vert in preserve_non_ring_verts if _debug_is_valid(vert)
        ),
        still_unselected_preserved_verts=_debug_vert_refs(
            vert for vert in preserve_non_ring_verts
            if _debug_is_valid(vert) and not _debug_is_selected(vert)
        ),
        selected_verts=_selected_vert_refs(bm),
    )
    _debug_report(
        report,
        f"group={group_label} after restore "
        f"restored={_debug_vert_ids(preserve_non_ring_verts)} "
        f"missing_from_original={_debug_vert_ids(missing_after_restore)} "
        f"selected={_debug_vert_ids(selected_after_restore)}",
    )
    return preserve_non_ring_verts


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

    debug_log(
        "hole_execute",
        "Starting hole-in-mesh execute.",
        mode_label=data.get('mode_label'),
        direction=direction,
        group_count=len(data['groups']),
        invalid_components=data['invalid_components'],
        mesh=mesh_stats(bm),
        selected_verts=_selected_vert_refs(bm),
    )

    final_realign_groups = []
    successful_ring_groups = []
    for group_index, group_data in enumerate(data['groups']):
        if any(
            not _debug_is_valid(vert)
            for loop in group_data['rings'][0]
            for vert in loop
        ):
            if report is not None:
                report({'WARNING'}, "Skipped stale hole ring after mesh update.")
            continue
        preserved_non_ring_verts = _cleanup_selected_ring_edges(
            bm,
            group_data.get('cleanup_edges', set()),
            group_data.get('cleanup_ring_edges', set()),
            report=report,
            group_index=group_index,
        )
        debug_log(
            "hole_execute",
            "Before aligned resample for hole group.",
            group_index=group_index,
            preserved_non_ring_verts=_debug_vert_refs(preserved_non_ring_verts),
            rings=[
                _debug_vert_refs(loop)
                for loop in group_data['rings'][0]
            ],
            mesh=mesh_stats(bm),
            selected_verts=_selected_vert_refs(bm),
        )
        result_info = {}
        result = execute_aligned_loops_logic(
            bm,
            obj,
            group_data['rings'],
            direction,
            report=report,
            use_seams=group_data.get('use_seams', True),
            migrate_seams=group_data.get('migrate_seams'),
            max_seams=group_data.get('max_seams'),
            repair_topology=group_data.get('repair_topology', True),
            align_seams_to_ring_normals=group_data.get(
                'align_seams_to_ring_normals',
                False,
            ),
            result_info=result_info,
        )
        _refresh_bmesh_indices(bm)
        if result == {'FINISHED'} and result_info.get('ring_group') is not None:
            successful_ring_groups.append(result_info['ring_group'])
        if (
            result == {'FINISHED'}
            and group_data.get('align_seams_to_ring_normals', False)
            and result_info.get('ring_group') is not None
        ):
            final_realign_groups.append(result_info['ring_group'])
        invalid_preserved = {
            vert for vert in preserved_non_ring_verts
            if not _debug_is_valid(vert)
        }
        unselected_preserved = {
            vert for vert in preserved_non_ring_verts
            if _debug_is_valid(vert) and not _debug_is_selected(vert)
        }
        _debug_report(
            report,
            f"group={group_index} after resample "
            f"result={result} "
            f"invalid_preserved={_debug_vert_ids(invalid_preserved)} "
            f"unselected_preserved={_debug_vert_ids(unselected_preserved)} "
            f"selected={_debug_vert_ids(vert for vert in bm.verts if _debug_is_selected(vert))}",
        )
        debug_log(
            "hole_execute",
            "After aligned resample for hole group.",
            group_index=group_index,
            result=result,
            preserved_non_ring_verts=_debug_vert_refs(preserved_non_ring_verts),
            invalid_preserved_non_ring_verts=_debug_vert_refs(
                invalid_preserved
            ),
            unselected_preserved_non_ring_verts=_debug_vert_refs(
                unselected_preserved
            ),
            mesh=mesh_stats(bm),
            selected_verts=_selected_vert_refs(bm),
        )
    if final_realign_groups:
        for _pass_index in range(3):
            moved = False
            for ring_group in final_realign_groups:
                moved = realign_seams_to_ring_normals(
                    bm,
                    ring_group,
                    search_steps=2,
                    sample_steps=2,
                ) or moved
            if not moved:
                break
        _refresh_bmesh_indices(bm)
    if successful_ring_groups:
        _refresh_bmesh_indices(bm)
        _select_only_ring_edges(bm, successful_ring_groups)
        bmesh.update_edit_mesh(obj.data)
    return {'FINISHED'}
