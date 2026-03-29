# bridged_open_loop.py — Resample bridged open-loop selections.

from .open_strip_common import detect_open_strip_selection
from .resample_common import execute_aligned_loops_logic


def detect(bm):
    print("[vertex_resampler:dispatch] checking Bridged open loop")
    data = detect_open_strip_selection(bm)
    if (
        data is None
        or data['has_extra_selected_faces']
        or data['has_outside_side_faces']
    ):
        print("[vertex_resampler:dispatch] Bridged open loop rejected")
        return None

    print("[vertex_resampler:dispatch] Bridged open loop matched")
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
