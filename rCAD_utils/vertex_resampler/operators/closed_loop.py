# closed_loop.py — Umbrella closed-loop operator for simple loops and bridge variants.

from . import closed_loop_bridged
from . import closed_loop_bridged_with_corners
from .detection_utils import get_selected_islands
from .resample_common import execute_floating_logic


def _detect_simple_closed_loops(bm):
    islands = get_selected_islands(bm)
    closed_islands = [island for island in islands if island['closed']]
    if not closed_islands:
        return None

    return {
        'handler': 'simple',
        'islands': closed_islands,
        'mode_label': 'Closed loop',
        'variant_label': 'Closed loop',
    }


def detect(bm):
    corner_data = closed_loop_bridged_with_corners.detect(bm)
    if corner_data:
        return {
            'handler': 'bridge_with_corners',
            'data': corner_data,
            'mode_label': 'Closed loop',
            'variant_label': corner_data.get(
                'mode_label',
                'Closed loop bridged with corners',
            ),
        }

    bridged_data = closed_loop_bridged.detect(bm)
    if bridged_data:
        return {
            'handler': 'bridge',
            'data': bridged_data,
            'mode_label': 'Closed loop',
            'variant_label': bridged_data.get(
                'mode_label',
                'Closed loop bridged',
            ),
        }

    return _detect_simple_closed_loops(bm)


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    variant_label = data.get('variant_label')
    if report is not None and variant_label:
        report({'INFO'}, f"Closed loop detected: {variant_label}")

    handler = data.get('handler')
    if handler == 'bridge_with_corners':
        return closed_loop_bridged_with_corners.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data.get('data'),
        )

    if handler == 'bridge':
        return closed_loop_bridged.execute(
            bm,
            obj,
            direction,
            report=report,
            data=data.get('data'),
        )

    if handler == 'simple':
        return execute_floating_logic(
            bm,
            obj,
            direction,
            islands=data['islands'],
        )

    return {'CANCELLED'}
