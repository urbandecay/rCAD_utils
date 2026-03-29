# closed_loop_bridged.py — Resample bridged closed-loop selections.

from .bridge_utils import get_auto_bridged_chain, get_bridged_chain
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

    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for rings_data in data['groups']:
        execute_aligned_loops_logic(bm, obj, rings_data, direction, report=report)
    return {'FINISHED'}
