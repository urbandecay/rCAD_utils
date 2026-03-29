# closed_loop.py — Resample plain closed-loop selections.

from .detection_utils import get_selected_islands
from .resample_common import execute_floating_logic


def detect(bm):
    islands = get_selected_islands(bm)
    closed_islands = [island for island in islands if island['closed']]
    if closed_islands:
        return {
            'islands': closed_islands,
            'mode_label': 'Closed loop',
        }
    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    return execute_floating_logic(bm, obj, direction, islands=data['islands'])
