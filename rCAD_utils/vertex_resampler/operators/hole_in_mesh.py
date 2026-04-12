# hole_in_mesh.py — Dispatch punched-hole resampling across hole types.

from . import hole_punch_face, hole_punch_solid
from .resample_common import execute_aligned_loops_logic


def detect(bm, report=None):
    data = hole_punch_solid.detect(bm, report=report)
    if data is not None:
        return data

    return hole_punch_face.detect(bm, report=report)


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm, report=report)
    if not data:
        return {'CANCELLED'}
    if not data['groups']:
        if report is not None:
            report({'ERROR'}, "Could not detect a valid punched hole.")
        return {'CANCELLED'}

    if data['invalid_components'] and report is not None:
        report(
            {'WARNING'},
            f"Skipped {data['invalid_components']} invalid hole component(s).",
        )

    for group_data in data['groups']:
        execute_aligned_loops_logic(
            bm,
            obj,
            group_data['rings'],
            direction,
            report=report,
            use_seams=group_data.get('use_seams', True),
            migrate_seams=group_data.get('migrate_seams'),
            max_seams=group_data.get('max_seams'),
            seam_drift_factor=group_data.get('seam_drift_factor', 0.5),
            persist_seam_homes=group_data.get('persist_seam_homes', True),
            anchor_to_seams=group_data.get('anchor_to_seams', False),
            use_endpoint_seam_anchors=group_data.get('use_endpoint_seam_anchors', False),
        )
    return {'FINISHED'}
