# bridged_open_loop.py — Resample bridged open-loop selections.

from .bridge_utils import get_bridged_chain
from . import corner
from .resample_common import execute_aligned_loops_logic


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and not bridged_data[1] and len(bridged_data[0]) >= 2:
        return {
            'groups': [bridged_data],
            'mode_label': 'Bridged open loop',
        }

    strip_data = corner.detect_strip_groups(bm)
    if strip_data and strip_data.get('groups'):
        return {
            'groups': strip_data['groups'],
            'mode_label': 'Bridged open loop',
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
