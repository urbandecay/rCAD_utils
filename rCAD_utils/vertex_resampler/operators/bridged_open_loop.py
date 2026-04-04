# bridged_open_loop.py — Resample bridged open-loop selections.

from ..debug import mesh_stats
from .open_strip_common import detect_open_strip_selection, get_last_detect_reason
from .resample_common import execute_aligned_loops_logic


def _trace_open_bridge(message, **details):
    print(f"[vertex_resampler:open_bridge] {message}")
    for key, value in details.items():
        print(f"  {key}: {value}")


def detect(bm):
    data = detect_open_strip_selection(bm)
    if data is None:
        _trace_open_bridge(
            "Bridged open loop rejected: shared detector returned no data.",
            reason=get_last_detect_reason(),
            mesh=mesh_stats(bm),
        )
        return None

    if data['has_extra_selected_faces'] or data['has_outside_side_faces']:
        _trace_open_bridge(
            "Bridged open loop rejected by strip flags.",
            has_extra_selected_faces=data['has_extra_selected_faces'],
            has_outside_side_faces=data['has_outside_side_faces'],
            reason=get_last_detect_reason(),
        )
        return None

    _trace_open_bridge(
        "Bridged open loop matched.",
        group_count=len(data['groups']),
        reason=get_last_detect_reason(),
    )
    return {
        'groups': data['groups'],
        'mode_label': 'Bridged open loop',
    }


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for rings_data in data['groups']:
        execute_aligned_loops_logic(bm, obj, rings_data, direction, report=report)
    return {'FINISHED'}
