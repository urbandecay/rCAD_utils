# open_loop.py — Resample open chains and loose loop selections.

from .detection_utils import get_selected_islands
from .resample_common import execute_floating_logic


def detect(bm):
    islands = get_selected_islands(bm)
    open_islands = [island for island in islands if not island['closed']]
    return open_islands or None


def execute(bm, obj, direction):
    islands = detect(bm)
    if not islands:
        return {'CANCELLED'}
    return execute_floating_logic(bm, obj, direction, islands=islands)
