"""Debug helpers for vertex resampler tracing."""

import os


def _env_enabled():
    value = os.environ.get("RCAD_VERTEX_RESAMPLER_DEBUG", "0").strip().lower()
    return value not in {"0", "false", "off", "no"}


ENABLED = _env_enabled()
_CURRENT_OPERATION = None
_NEXT_OPERATION_ID = 1


def _safe_attr(item, name, default=None):
    try:
        return getattr(item, name)
    except Exception:
        return default


def _fmt_vec(vec):
    if vec is None:
        return "(?)"
    try:
        return f"({vec.x:.4f}, {vec.y:.4f}, {vec.z:.4f})"
    except Exception:
        return "(?)"


def vert_ref(vert):
    if vert is None:
        return "None"

    valid = bool(_safe_attr(vert, "is_valid", False))
    index = _safe_attr(vert, "index", "?")
    if not valid:
        return f"v{index}<invalid>"

    co = _fmt_vec(_safe_attr(vert, "co"))
    selected = bool(_safe_attr(vert, "select", False))
    degree = len(_safe_attr(vert, "link_edges", ()))
    return f"v{index}{'*' if selected else ''}@{co}/deg={degree}"


def edge_ref(edge):
    if edge is None:
        return "None"

    valid = bool(_safe_attr(edge, "is_valid", False))
    index = _safe_attr(edge, "index", "?")
    if not valid:
        return f"e{index}<invalid>"

    verts = _safe_attr(edge, "verts", ())
    vert_ids = [str(_safe_attr(vert, "index", "?")) for vert in verts]
    face_count = len(_safe_attr(edge, "link_faces", ()))
    selected = bool(_safe_attr(edge, "select", False))
    return f"e{index}{'*' if selected else ''}[{'-'.join(vert_ids)}]/faces={face_count}"


def face_ref(face):
    if face is None:
        return "None"

    valid = bool(_safe_attr(face, "is_valid", False))
    index = _safe_attr(face, "index", "?")
    if not valid:
        return f"f{index}<invalid>"

    verts = _safe_attr(face, "verts", ())
    vert_ids = [str(_safe_attr(vert, "index", "?")) for vert in verts]
    selected = bool(_safe_attr(face, "select", False))
    return f"f{index}{'*' if selected else ''}[{','.join(vert_ids)}]/n={len(vert_ids)}"


def face_refs(faces):
    return [face_ref(face) for face in faces]


def loop_ref(loop):
    return [vert_ref(vert) for vert in loop]


def seam_ref(seam_verts):
    return [vert_ref(vert) for vert in sorted(seam_verts, key=lambda vert: vert.index)]


def pair_ref(a, b):
    return f"{vert_ref(a)} <-> {vert_ref(b)}"


def ring_group_ref(ring_group):
    rings = []
    for idx, ring_info in enumerate(ring_group.rings):
        rings.append({
            "ring_index": idx,
            "is_closed": ring_info.is_closed,
            "verts": loop_ref(ring_info.verts),
            "seams": seam_ref(ring_info.seam_verts),
        })
    return rings


def mesh_stats(bm):
    return {
        "verts": len(bm.verts),
        "edges": len(bm.edges),
        "faces": len(bm.faces),
        "selected_verts": sum(1 for vert in bm.verts if vert.select),
        "selected_edges": sum(1 for edge in bm.edges if edge.select),
        "selected_faces": sum(1 for face in bm.faces if face.select),
    }


def debug_log(stage, message, **details):
    if not ENABLED:
        return

    prefix = f"[vertex_resampler:{stage}]"
    if _CURRENT_OPERATION is not None:
        prefix += f"[op:{_CURRENT_OPERATION}]"
    print(f"{prefix} {message}")
    for key, value in details.items():
        print(f"  {key}: {value}")


def debug_separator(lines=10):
    if not ENABLED:
        return
    print("\n" * max(1, int(lines)), end="")


def begin_debug_operation(label="operation"):
    global _CURRENT_OPERATION, _NEXT_OPERATION_ID
    if not ENABLED:
        return None

    op_id = _NEXT_OPERATION_ID
    _NEXT_OPERATION_ID += 1
    _CURRENT_OPERATION = op_id
    print(f"[vertex_resampler:op][op:{op_id}] BEGIN {label}")
    return op_id


def end_debug_operation(op_id=None, label="operation"):
    global _CURRENT_OPERATION
    if not ENABLED:
        return

    active_id = _CURRENT_OPERATION if op_id is None else op_id
    if active_id is not None:
        print(f"[vertex_resampler:op][op:{active_id}] END {label}")
    _CURRENT_OPERATION = None
