# open_loop.py — Resample plain open-chain selections.

from .detection_utils import get_selected_islands
from .resample_common import execute_floating_logic


def detect(bm):
    islands = get_selected_islands(bm)
    open_islands = [island for island in islands if not island['closed']]
    if open_islands:
        return {'islands': open_islands}
    return None


def execute(bm, obj, direction, report=None):
    data = detect(bm)
    if not data:
        return {'CANCELLED'}

    return execute_floating_logic(bm, obj, direction, islands=data['islands'])
