# bridged_open_loop_with_corners.py — Resample bridged open loops with corners.

from .open_strip_common import detect_open_strip_selection


def _groups_are_open(groups):
    return bool(groups) and all(not rings_data[1] for rings_data in groups)


def detect(bm):
    data = detect_open_strip_selection(bm)
    if data is None:
        return None
    if not _groups_are_open(data['groups']):
        return None
    if not (
        data['has_extra_selected_faces']
        or data['has_outside_side_faces']
    ):
        return None

    return {
        'groups': data['groups'],
        'mode_label': 'Bridged open loop with corners',
    }


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    if report is not None:
        report({'INFO'}, "Open loop bridge corner")

    # Resample execution is intentionally disabled here.
    # return bridged_open_loop.execute(
    #     bm,
    #     obj,
    #     direction,
    #     report=report,
    #     data=data,
    # )
    return {'FINISHED'}
