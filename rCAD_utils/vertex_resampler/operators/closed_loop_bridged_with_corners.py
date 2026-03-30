# closed_loop_bridged_with_corners.py — Resample bridged closed loops with corners.

from . import closed_loop_bridged
from .bridge_utils import get_auto_bridged_chain, get_bridged_chain


def _group_loops(group_data):
    if isinstance(group_data, dict):
        return group_data['rings'][0]
    return group_data[0]


def _group_is_closed(group_data):
    if isinstance(group_data, dict):
        return group_data['rings'][1]
    return group_data[1]


def _count_corner_cross_sections(groups):
    seen_loops = set()
    corner_count = 0

    for group_data in groups:
        for loop in _group_loops(group_data):
            loop_key = tuple(sorted(vert.index for vert in loop))
            if loop_key in seen_loops:
                continue
            seen_loops.add(loop_key)

            if loop and all(len(vert.link_edges) == 4 for vert in loop):
                corner_count += 1

    return corner_count


def _result(groups, invalid_components=0):
    if not groups:
        return None
    if not all(_group_is_closed(group_data) for group_data in groups):
        return None

    corner_count = _count_corner_cross_sections(groups)
    if corner_count <= 0:
        return None

    return {
        'groups': groups,
        'invalid_components': invalid_components,
        'mode_label': 'Closed loop bridged with corners',
        'corner_count': corner_count,
    }


def _detect_face_corner_groups(bm):
    sel_verts = [vert for vert in bm.verts if vert.select]
    if len(sel_verts) < 4:
        return None

    selected_faces = closed_loop_bridged._selected_face_set(bm, set(sel_verts))
    if not selected_faces:
        return None

    shaft_faces = {
        face for face in selected_faces
        if closed_loop_bridged._is_shaft_face(face, selected_faces)
    }
    if not shaft_faces:
        return None

    if not (selected_faces - shaft_faces):
        return None

    groups = []
    invalid_components = 0
    for component in closed_loop_bridged._face_components(shaft_faces):
        rings_data = closed_loop_bridged._rings_from_component(component)
        if rings_data is None:
            invalid_components += 1
            continue
        groups.append({
            'rings': rings_data,
            'use_seams': True,
            'migrate_seams': True,
            'max_seams': 2,
        })

    if not groups and not invalid_components:
        return None

    return _result(groups, invalid_components=invalid_components)


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and bridged_data[1] and len(bridged_data[0]) >= 2:
        result = _result([bridged_data])
        if result is not None:
            return result

    auto_bridged = get_auto_bridged_chain(bm)
    if auto_bridged and len(auto_bridged[0]) >= 2:
        result = _result([auto_bridged])
        if result is not None:
            return result

    return _detect_face_corner_groups(bm)


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    if report is not None:
        report({'INFO'}, "Closed loop bridge corner")

    # Resample execution is intentionally disabled here.
    # return closed_loop_bridged.execute(
    #     bm,
    #     obj,
    #     direction,
    #     report=report,
    #     data=data,
    # )
    return {'FINISHED'}
