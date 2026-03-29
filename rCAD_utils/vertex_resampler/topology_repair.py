# topology_repair.py — Post-dissolve topology repair.
# Restores proper face structure after vert deletion.
# No knowledge of rings or seams — just takes neighbor pairs.

import bmesh

from .debug import debug_log, edge_ref, face_ref, face_refs, mesh_stats, pair_ref


def _normalize_contexts(repair_items):
    contexts = []
    for item in repair_items:
        if isinstance(item, dict):
            context = dict(item)
            context.setdefault("kind", "delete_pair")
            contexts.append(context)
            continue

        a, b = item
        contexts.append({
            "kind": "generic_pair",
            "a": a,
            "b": b,
            "all_pairs": [item],
            "shell_hint_verts": [],
        })
    return contexts


def _pair_contains(face, pair):
    if pair is None:
        return False
    a, b = pair
    return a.is_valid and b.is_valid and a in face.verts and b in face.verts


def _pair_ref_from_context(context):
    return pair_ref(context["a"], context["b"])


def _face_score(face, context, preferred_pair=None):
    score = 0

    if preferred_pair is not None and _pair_contains(face, preferred_pair):
        score += 1000

    for pair in context.get("all_pairs", []):
        if _pair_contains(face, pair):
            score += 100

    shell_hint_verts = [vert for vert in context.get("shell_hint_verts", []) if vert.is_valid]
    score += sum(1 for vert in shell_hint_verts if vert in face.verts) * 10

    return (score, -len(face.verts), -face.index)


def _pick_candidate_face(candidate_faces, context, preferred_pair=None):
    if not candidate_faces:
        return None

    scored_faces = sorted(
        candidate_faces,
        key=lambda face: _face_score(face, context, preferred_pair=preferred_pair),
        reverse=True,
    )
    return scored_faces[0]


def _sorted_candidate_faces(candidate_faces, context, preferred_pair=None):
    return sorted(
        candidate_faces,
        key=lambda face: _face_score(face, context, preferred_pair=preferred_pair),
        reverse=True,
    )


def _candidate_faces_for_pair(a, b):
    return [face for face in a.link_faces if face.is_valid and b in face.verts]


def _cleanup_loose_edges(bm, edges):
    loose_edges = [
        edge for edge in edges
        if edge is not None and edge.is_valid and not edge.link_faces
    ]
    if loose_edges:
        bmesh.ops.delete(bm, geom=loose_edges, context='EDGES')


def _shared_delete_face(contexts):
    shared_faces = None
    for context in contexts:
        faces = set(_candidate_faces_for_pair(context["a"], context["b"]))
        if shared_faces is None:
            shared_faces = faces
        else:
            shared_faces &= faces
    return list(shared_faces or [])


def _combined_context(contexts):
    shell_hint_verts = []
    all_pairs = []
    for context in contexts:
        shell_hint_verts.extend(context.get("shell_hint_verts", []))
        all_pairs.extend(context.get("all_pairs", []))
    return {
        "kind": "paired_delete",
        "a": contexts[0]["a"],
        "b": contexts[0]["b"],
        "shell_hint_verts": shell_hint_verts,
        "all_pairs": all_pairs,
    }


def _quad_cycle_from_contexts(bm, contexts):
    a0 = contexts[0]["a"]
    b0 = contexts[0]["b"]
    a1 = contexts[1]["a"]
    b1 = contexts[1]["b"]

    cycles = [
        (a0, b0, b1, a1),
        (a0, b0, a1, b1),
    ]

    for cycle in cycles:
        v0, v1, v2, v3 = cycle
        if bm.edges.get([v1, v2]) is None:
            continue
        if bm.edges.get([v3, v0]) is None:
            continue
        return cycle

    return None


def _face_contains_cycle(face, cycle):
    return all(vert.is_valid and vert in face.verts for vert in cycle)


def _quad_face_for_cycle(cycle):
    verts = set(cycle)
    for vert in cycle:
        for face in vert.link_faces:
            if not face.is_valid or len(face.verts) != 4:
                continue
            if set(face.verts) == verts:
                return face
    return None


def _cyclic_face_path(face_verts, start_index, end_index):
    count = len(face_verts)
    path = [face_verts[start_index]]
    index = start_index
    while index != end_index:
        index = (index + 1) % count
        path.append(face_verts[index])
    return path


def _local_quad_cycle_for_pair(bm, face, a, b):
    if face is None or not face.is_valid:
        return None
    if not (a.is_valid and b.is_valid):
        return None

    face_verts = list(face.verts)
    try:
        index_a = face_verts.index(a)
        index_b = face_verts.index(b)
    except ValueError:
        return None

    candidate_paths = [
        _cyclic_face_path(face_verts, index_a, index_b),
        _cyclic_face_path(face_verts, index_b, index_a),
    ]

    for path in candidate_paths:
        if len(path) != 4:
            continue
        v0, v1, v2, v3 = path
        if bm.edges.get([v1, v2]) is None:
            continue
        if v0 == a and v3 == b:
            return (v0, v1, v2, v3)
        if v0 == b and v3 == a:
            return (a, v2, v1, b)

    return None


def _safe_local_pair_split(bm, face, a, b):
    cycle = _local_quad_cycle_for_pair(bm, face, a, b)
    if cycle is None:
        return False

    split_result = bmesh.utils.face_split(face, a, b)
    quad_face = _quad_face_for_cycle(cycle)
    if quad_face is None:
        debug_log(
            "repair_step",
            "Local quad split did not produce the expected quad face.",
            pair=pair_ref(a, b),
            merged_face=face_ref(face),
            cycle=[pair_ref(cycle[0], cycle[1]), pair_ref(cycle[3], cycle[2])],
            split_result=repr(split_result),
        )
        return False

    debug_log(
        "repair_step",
        "Recovered delete pair with safe local quad split.",
        pair=pair_ref(a, b),
        merged_face=face_ref(face),
        cycle=[pair_ref(cycle[0], cycle[1]), pair_ref(cycle[3], cycle[2])],
        split_result=repr(split_result),
        quad_face=face_ref(quad_face),
    )
    return True


def _skip_pair_split_fallback(face, context):
    if face is None:
        return False
    if context.get("kind") != "delete_pair":
        return False
    return len(face.verts) > 12


def _repair_paired_delete_contexts(bm, contexts):
    if len(contexts) != 2:
        return False
    if any(context.get("kind") != "delete_pair" for context in contexts):
        return False

    cycle = _quad_cycle_from_contexts(bm, contexts)
    if cycle is None:
        return False

    candidate_faces = [
        face for face in _shared_delete_face(contexts)
        if _face_contains_cycle(face, cycle)
    ]
    if not candidate_faces:
        return False

    combined_context = _combined_context(contexts)
    v0, v1, v2, v3 = cycle
    candidate_faces = _sorted_candidate_faces(candidate_faces, combined_context)
    errors = []

    for merged_face in candidate_faces:
        if merged_face is None or len(merged_face.verts) <= 4:
            continue

        created_edges = []
        try:
            edge01 = bm.edges.get([v0, v1])
            if edge01 is None:
                edge01 = bm.edges.new([v0, v1])
                created_edges.append(edge01)

            edge23 = bm.edges.get([v2, v3])
            if edge23 is None:
                edge23 = bm.edges.new([v2, v3])
                created_edges.append(edge23)

            split_faces = bmesh.utils.face_split_edgenet(merged_face, [edge01, edge23])
            quad_face = _quad_face_for_cycle(cycle)
            if quad_face is None:
                raise RuntimeError("paired edgenet split did not produce the local quad")

            debug_log(
                "repair_step",
                "Paired delete repair split merged face with edgenet.",
                merged_face=face_ref(merged_face),
                cycle=[pair_ref(v0, v1), pair_ref(v3, v2)],
                split_faces=face_refs(split_faces),
                quad_face=face_ref(quad_face),
            )
            return True
        except Exception as exc:
            _cleanup_loose_edges(bm, created_edges)
            errors.append({
                "merged_face": face_ref(merged_face),
                "error": repr(exc),
            })

    debug_log(
        "repair_step",
        "Paired delete repair fallback triggered.",
        cycle=[pair_ref(v0, v1), pair_ref(v3, v2)],
        candidate_faces=face_refs(candidate_faces),
        errors=errors,
    )
    return False


def repair_after_dissolve(bm, neighbor_pairs):
    """Repair topology after dissolve_verts.

    When dissolve_verts removes a ring vert from a hole-in-mesh cylinder,
    it merges the shaft quad + mesh face into one big n-gon — the ring edge
    A-B ends up buried inside it. Splitting the n-gon at (A, B) restores
    the shaft quad and mesh face as separate faces.

    Works for any deletion position — just needs the two neighbor verts
    that were on either side of the deleted vert.
    """
    contexts = _normalize_contexts(neighbor_pairs)
    if _repair_paired_delete_contexts(bm, contexts):
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.normal_update()
        debug_log(
            "repair_step",
            "Completed repair pass.",
            post_mesh=mesh_stats(bm),
        )
        return

    previous_pair = None

    for context in contexts:
        a = context["a"]
        b = context["b"]
        debug_log(
            "repair_step",
            "Inspecting neighbor pair for repair.",
            pair=_pair_ref_from_context(context),
            kind=context.get("kind"),
            pre_mesh=mesh_stats(bm),
        )
        if not (a.is_valid and b.is_valid):
            debug_log(
                "repair_step",
                "Skipping repair because one of the verts is invalid.",
                pair=_pair_ref_from_context(context),
            )
            continue
        existing_edge = bm.edges.get([a, b])
        if existing_edge is not None:
            debug_log(
                "repair_step",
                "Edge already exists, no repair needed.",
                pair=_pair_ref_from_context(context),
                edge=edge_ref(existing_edge),
                shared_faces=face_refs([face for face in a.link_faces if face.is_valid and b in face.verts]),
            )
            previous_pair = (a, b)
            continue  # edge already exists, no repair needed

        candidate_faces = []
        for f in a.link_faces:
            if b in f.verts:
                candidate_faces.append(f)

        merged_face = _pick_candidate_face(
            candidate_faces,
            context,
            preferred_pair=previous_pair,
        )

        debug_log(
            "repair_step",
            "Collected face candidates for repair pair.",
            pair=_pair_ref_from_context(context),
            candidate_faces=face_refs(candidate_faces),
            chosen_face=face_ref(merged_face),
            chosen_face_score=(
                _face_score(merged_face, context, preferred_pair=previous_pair)
                if merged_face is not None else None
            ),
            preferred_pair=pair_ref(*previous_pair) if previous_pair is not None else None,
        )

        if _skip_pair_split_fallback(merged_face, context):
            if _safe_local_pair_split(bm, merged_face, a, b):
                previous_pair = (a, b)
            else:
                debug_log(
                    "repair_step",
                    "Skipped unsafe giant-face split fallback for delete pair.",
                    pair=_pair_ref_from_context(context),
                    merged_face=face_ref(merged_face),
                )
                previous_pair = (a, b)
        elif merged_face is not None and len(merged_face.verts) > 4:
            split_result = bmesh.utils.face_split(merged_face, a, b)
            edge_after_split = bm.edges.get([a, b])
            shared_faces_after_split = [
                face for face in a.link_faces
                if face.is_valid and b in face.verts
            ]
            debug_log(
                "repair_step",
                "Split merged face.",
                pair=_pair_ref_from_context(context),
                merged_face=face_ref(merged_face),
                split_result=repr(split_result),
                edge_after_split=edge_ref(edge_after_split),
                shared_faces_after_split=face_refs(shared_faces_after_split),
            )
            previous_pair = (a, b)
        elif merged_face is None:
            new_edge = bm.edges.new([a, b])
            debug_log(
                "repair_step",
                "Created missing edge directly because no shared face was found.",
                pair=_pair_ref_from_context(context),
                new_edge=edge_ref(new_edge),
            )
            previous_pair = (a, b)
        else:
            debug_log(
                "repair_step",
                "Found a shared face but it is too small to split.",
                pair=_pair_ref_from_context(context),
                merged_face=face_ref(merged_face),
            )
            previous_pair = (a, b)

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()

    debug_log(
        "repair_step",
        "Completed repair pass.",
        post_mesh=mesh_stats(bm),
    )
