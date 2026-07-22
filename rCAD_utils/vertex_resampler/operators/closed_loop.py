# closed_loop.py — Umbrella closed-loop operator for simple loops and bridge variants.

from . import closed_loop_bridged
from . import closed_loop_bridged_with_corners
from .detection_utils import get_selected_islands
from .resample_common import execute_floating_logic


def _detect_simple_closed_loops(bm, islands=None):
    if islands is None:
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


def _detect_specialized(bm):
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

    return None


def _is_loose_island(island):
    return not any(vert.link_faces for vert in island['verts'])


def _capture_island_selection(islands):
    verts = {
        vert
        for island in islands
        for vert in island['verts']
        if vert.is_valid
    }
    edges = {
        edge
        for vert in verts
        for edge in vert.link_edges
        if edge.select and all(edge_vert in verts for edge_vert in edge.verts)
    }
    return verts, edges


def _set_island_selection(selection, selected):
    verts, edges = selection
    if selected:
        for vert in verts:
            if vert.is_valid:
                vert.select = True
        for edge in edges:
            if edge.is_valid:
                edge.select = True
        return

    for edge in edges:
        if edge.is_valid:
            edge.select = False
    for vert in verts:
        if vert.is_valid:
            vert.select = False


def detect(bm):
    islands = get_selected_islands(bm)
    closed_islands = [island for island in islands if island['closed']]
    loose_islands = []
    attached_islands = []
    for island in closed_islands:
        target = loose_islands if _is_loose_island(island) else attached_islands
        target.append(island)

    # A loose wire loop is valid input, but it must not make the topology-aware
    # detectors reject otherwise valid connected loops. Detect those loops with
    # the wire selection temporarily hidden, then execute both parts separately.
    if loose_islands and attached_islands:
        loose_selection = _capture_island_selection(loose_islands)
        _set_island_selection(loose_selection, False)
        try:
            attached_data = _detect_specialized(bm)
        finally:
            _set_island_selection(loose_selection, True)

        if attached_data:
            return {
                'handler': 'mixed',
                'attached_data': attached_data,
                'loose_islands': loose_islands,
                'loose_selection': loose_selection,
                'mode_label': 'Closed loop',
                'variant_label': 'Connected and loose closed loops',
            }

    specialized_data = _detect_specialized(bm)
    if specialized_data:
        return specialized_data

    return _detect_simple_closed_loops(bm, islands=islands)


def _execute_detected(bm, obj, direction, report, data):
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


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    variant_label = data.get('variant_label')
    if report is not None and variant_label:
        report({'INFO'}, f"Closed loop detected: {variant_label}")

    handler = data.get('handler')
    if handler == 'mixed':
        loose_selection = data['loose_selection']
        _set_island_selection(loose_selection, False)
        try:
            attached_result = _execute_detected(
                bm,
                obj,
                direction,
                report,
                data['attached_data'],
            )
        finally:
            _set_island_selection(loose_selection, True)

        if attached_result != {'FINISHED'}:
            return attached_result

        return execute_floating_logic(
            bm,
            obj,
            direction,
            islands=data['loose_islands'],
        )

    return _execute_detected(bm, obj, direction, report, data)
