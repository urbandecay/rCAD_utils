# closed_loop_bridged.py — Resample bridged closed-loop selections.

from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
from . import hole_punch_solid
from .resample_common import execute_aligned_loops_logic


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and bridged_data[1] and len(bridged_data[0]) >= 2:
        return {
            'groups': [bridged_data],
            'mode_label': 'Closed loop bridged',
        }

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        return {
            'groups': [auto_bridged],
            'mode_label': 'Closed loop bridged',
        }

    shaft_face_data = hole_punch_solid.detect_shaft_face_groups(
        bm,
        require_embedded=False,
    )
    if shaft_face_data is not None and (
        shaft_face_data['groups'] or shaft_face_data['invalid_components']
    ):
        shaft_face_data['mode_label'] = 'Closed loop bridged'
        return shaft_face_data

    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for group_data in data['groups']:
        if isinstance(group_data, dict):
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
            continue

        execute_aligned_loops_logic(bm, obj, group_data, direction, report=report)
    return {'FINISHED'}
