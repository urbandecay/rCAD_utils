# bridged_open_loop_with_corners.py — Resample bridged open loops with corners.

import bmesh

from . import bridged_open_loop
from .. import anchor_overlay
from ..debug import debug_log
from .bridge_utils import _is_ordered_open_chain
from .detection_utils import get_selected_islands
from .open_strip_common import (
    _face_components,
    _faces_to_component_verts,
    _detect_open_strip_component,
    _selected_vert_components,
    _strip_length_metrics,
    _component_span_metrics,
    _strip_candidate_sort_key,
    _selected_face_set,
    _trim_group_to_terminal_subset,
    detect_open_strip_selection,
)
from .resample_common import execute_aligned_loops_logic

_POSITION_TOLERANCE = 1.0e-6
_FACE_POSITION_TOLERANCE = 1.0e-5


def _groups_are_open(groups):
    return bool(groups) and all(not rings_data[1] for rings_data in groups)


def _position_key(vec, tolerance):
    return (
        round(vec.x / tolerance),
        round(vec.y / tolerance),
        round(vec.z / tolerance),
    )


def _debug_step(step, **details):
    debug_log("open_corner_split", step, **details)


def _trace_corner(message, **details):
    return


def _trace_focus(message, **details):
    print(f"[vertex_resampler:open_corner_detect] {message}")
    for key, value in details.items():
        print(f"  {key}: {value}")


def _vert_debug_label(vert):
    if vert is None or not getattr(vert, "is_valid", False):
        return None
    return f"v{vert.index}"


def _edge_debug_label(edge):
    if edge is None or not getattr(edge, "is_valid", False):
        return None
    vert_ids = [_vert_debug_label(vert) for vert in edge.verts]
    return f"e{edge.index}[{','.join(vert_ids)}]"


def _group_debug_key(group):
    loops = group[0] if group else []
    keys = []
    for loop in loops:
        loop_key = _loop_key(loop)
        if loop_key is not None:
            keys.append(loop_key)
    return tuple(keys)


def _safe_normalized(vec):
    if vec.length <= _POSITION_TOLERANCE:
        return None
    return vec.normalized()


def _selected_seed_loop(bm):
    islands = get_selected_islands(bm)
    _trace_corner(
        "Start detection: checking selected strip.",
        island_count=len(islands),
        selected_vert_count=len(_selected_verts(bm)),
    )
    if len(islands) == 1:
        island = islands[0]
        if not island.get('closed'):
            loop = [
                vert for vert in island.get('verts', [])
                if getattr(vert, "is_valid", False)
            ]
            is_chain = len(loop) >= 2 and _is_ordered_open_chain(loop)
            _trace_corner(
                "Start detection: edge-island candidate.",
                loop_size=len(loop),
                is_closed=bool(island.get('closed')),
                is_ordered_chain=is_chain,
            )
            if is_chain:
                return loop

    selected_components = _selected_vert_components(_selected_verts(bm))
    if len(selected_components) != 1:
        _trace_corner(
            "Start detection rejected.",
            reason="selected verts did not form exactly one connected component",
            component_count=len(selected_components),
            component_sizes=[len(component) for component in selected_components],
        )
        return None

    loop = _order_open_chain_component(selected_components[0])
    if loop is None:
        _trace_corner(
            "Start detection rejected.",
            reason="could not order selected component into an open chain",
            component_size=len(selected_components[0]),
        )
        return None

    is_chain = len(loop) >= 2 and _is_ordered_open_chain(loop)
    _trace_corner(
        "Start detection: ordered selected component.",
        component_size=len(selected_components[0]),
        ordered_size=len(loop),
        is_ordered_chain=is_chain,
        ordered_indices=[vert.index for vert in loop],
    )
    if len(loop) < 2 or not is_chain:
        _trace_corner(
            "Start detection rejected.",
            reason="ordered component was not a valid open chain",
            ordered_size=len(loop),
        )
        return None
    return loop


def _selected_seed_loops(bm):
    seed_loop = _selected_seed_loop(bm)
    if seed_loop is not None:
        return [seed_loop]

    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        return []

    seed_loops = []
    for component_index, component in enumerate(selected_components, start=1):
        loop = _order_open_chain_component(component)
        if loop is None:
            _trace_focus(
                "Selected seed loop failed.",
                component_index=component_index,
                reason="could not order component into an open chain",
                component_size=len(component),
            )
            return None

        if len(loop) < 2 or not _is_ordered_open_chain(loop):
            _trace_focus(
                "Selected seed loop failed.",
                component_index=component_index,
                reason="ordered component was not a valid open chain",
                component_size=len(component),
                loop_size=len(loop),
            )
            return None

        seed_loops.append(loop)

    if len(seed_loops) > 1:
        _trace_focus(
            "Selected seed loops built.",
            component_count=len(seed_loops),
            component_sizes=[len(loop) for loop in seed_loops],
        )
    return seed_loops


def _face_indices(faces):
    return {
        face.index for face in faces
        if face is not None and getattr(face, "is_valid", False)
    }


def _face_position(face):
    if face is None or not getattr(face, "is_valid", False):
        return None
    count = len(face.verts)
    if count == 0:
        return None
    center = face.verts[0].co.copy()
    for vert in face.verts[1:]:
        center += vert.co
    center /= count
    return center


def _face_positions(faces):
    return [
        position for position in (_face_position(face) for face in faces)
        if position is not None
    ]


def _live_faces_from_indices(bm, face_indices):
    index_set = set(face_indices)
    return {
        face for face in bm.faces
        if getattr(face, "is_valid", False) and face.index in index_set
    }


def _build_face_position_lookup(bm, tolerance=_FACE_POSITION_TOLERANCE):
    lookup = {}
    for face in bm.faces:
        if not getattr(face, "is_valid", False):
            continue
        face_position = _face_position(face)
        if face_position is None:
            continue
        lookup.setdefault(_position_key(face_position, tolerance), []).append((face_position, face))
    return lookup


def _cached_face_match(position, tolerance, lookup, match_cache=None):
    cache_key = None
    if match_cache is not None:
        cache_key = (position.x, position.y, position.z)
        if cache_key in match_cache:
            return match_cache[cache_key]

    matched_face = None
    for candidate_position, face in lookup.get(_position_key(position, tolerance), []):
        if (candidate_position - position).length <= tolerance:
            matched_face = face
            break

    if match_cache is not None:
        match_cache[cache_key] = matched_face
    return matched_face


def _live_faces_from_positions(bm, positions, tolerance=_FACE_POSITION_TOLERANCE, lookup=None, match_cache=None):
    live_faces = set()
    if not positions:
        return live_faces

    if lookup is None:
        lookup = _build_face_position_lookup(bm, tolerance=tolerance)

    for position in positions:
        matched_face = _cached_face_match(
            position,
            tolerance,
            lookup,
            match_cache=match_cache,
        )
        if matched_face is not None:
            live_faces.add(matched_face)

    return live_faces


def _selected_faces(bm):
    return {
        face for face in bm.faces
        if getattr(face, "is_valid", False) and face.select
    }


def _selected_verts(bm):
    return {
        vert for vert in bm.verts
        if getattr(vert, "is_valid", False) and vert.select
    }


def _open_groups_from_shaft_faces(shaft_faces):
    live_shaft_faces = _live_bmesh_items(shaft_faces)
    if not live_shaft_faces:
        return []

    open_groups = []
    for face_component in _face_components(live_shaft_faces):
        rings_data = _detect_open_strip_component(
            _faces_to_component_verts(face_component)
        )
        if rings_data is None or rings_data[1]:
            return []
        open_groups.append(rings_data)

    return open_groups


def _live_bmesh_items(items):
    return {
        item for item in items
        if item is not None and getattr(item, "is_valid", False)
    }


def _sort_key_desc(sort_key):
    return tuple(
        -value if isinstance(value, (int, float)) else value
        for value in sort_key
    )


def _connected_quad_faces(start_faces):
    live_start_faces = {
        face for face in start_faces
        if face is not None and getattr(face, "is_valid", False) and len(face.verts) == 4
    }
    if not live_start_faces:
        return set()

    visited = set(live_start_faces)
    stack = list(live_start_faces)

    while stack:
        face = stack.pop()
        for edge in face.edges:
            for neighbor in edge.link_faces:
                if (
                    neighbor in visited
                    or not getattr(neighbor, "is_valid", False)
                    or len(neighbor.verts) != 4
                ):
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

    return visited


def _quad_face_from_vert_set(verts):
    ordered_verts = [
        vert for vert in verts
        if vert is not None and getattr(vert, "is_valid", False)
    ]
    vert_set = set(ordered_verts)
    if len(vert_set) != 4:
        return None

    shared_faces = set(ordered_verts[0].link_faces)
    for vert in ordered_verts[1:]:
        shared_faces.intersection_update(vert.link_faces)

    return next(
        (
            face for face in shared_faces
            if getattr(face, "is_valid", False)
            and len(face.verts) == 4
            and set(face.verts) == vert_set
        ),
        None,
    )


def _shaft_faces_from_open_group(group):
    loops, is_closed = group
    if is_closed or len(loops) < 2:
        return set()

    shaft_faces = set()
    for current_loop, next_loop in zip(loops, loops[1:]):
        if len(current_loop) != len(next_loop) or len(current_loop) < 2:
            return set()

        for curr_a, curr_b, next_a, next_b in zip(
            current_loop,
            current_loop[1:],
            next_loop,
            next_loop[1:],
        ):
            face = _quad_face_from_vert_set((curr_a, curr_b, next_b, next_a))
            if face is None:
                return set()
            shaft_faces.add(face)

    return shaft_faces


def _corner_face_component_from_shaft_faces(shaft_seed_faces):
    shaft_faces = _connected_quad_faces(shaft_seed_faces)
    if not shaft_faces:
        return set(), set()

    face_component = set(shaft_faces)
    stack = list(shaft_faces)
    visited = set(shaft_faces)

    while stack:
        face = stack.pop()
        for edge in face.edges:
            for neighbor in edge.link_faces:
                if neighbor in visited or not getattr(neighbor, "is_valid", False):
                    continue
                visited.add(neighbor)
                face_component.add(neighbor)
                if len(neighbor.verts) == 4:
                    stack.append(neighbor)

    return face_component, shaft_faces


def _ordered_face_edges(face):
    loops = list(face.loops)
    if len(loops) != 4:
        return None
    return [loop.edge for loop in loops]


def _quad_face_neighbor(face, edge, selected_faces):
    return next(
        (
            other_face for other_face in edge.link_faces
            if other_face is not face
            and other_face in selected_faces
            and len(other_face.verts) == 4
        ),
        None,
    )


def _edge_orientation_bit(face, edge):
    ordered_edges = _ordered_face_edges(face)
    if ordered_edges is None:
        return None

    try:
        edge_index = ordered_edges.index(edge)
    except ValueError:
        return None

    return edge_index % 2


def _open_face_strip_candidates(face_component):
    quad_faces = {
        face for face in face_component
        if len(face.verts) == 4
    }
    if len(quad_faces) < 2:
        return []

    adjacency = {}
    for face in sorted(quad_faces, key=lambda item: item.index):
        ordered_edges = _ordered_face_edges(face)
        if ordered_edges is None:
            continue
        for bit in (0, 1):
            node = (face, bit)
            adjacency.setdefault(node, set())
            for edge in (ordered_edges[bit], ordered_edges[(bit + 2) % 4]):
                neighbor = _quad_face_neighbor(face, edge, quad_faces)
                if neighbor is None:
                    continue
                neighbor_bit = _edge_orientation_bit(neighbor, edge)
                if neighbor_bit is None:
                    continue
                adjacency[node].add((neighbor, neighbor_bit))

    candidates = []
    visited = set()

    for start_node in sorted(
        adjacency.keys(),
        key=lambda item: (item[0].index, item[1]),
    ):
        if start_node in visited:
            continue

        stack = [start_node]
        component_nodes = set()
        while stack:
            node = stack.pop()
            if node in visited:
                continue
            visited.add(node)
            component_nodes.add(node)
            for neighbor in adjacency.get(node, ()):
                if neighbor not in visited:
                    stack.append(neighbor)

        strip_faces = {face for face, _bit in component_nodes}
        if len(strip_faces) < 2:
            continue
        if len(strip_faces) != len(component_nodes):
            continue

        endpoint_count = 0
        valid = True
        for node in component_nodes:
            distinct_neighbors = len(adjacency.get(node, set()) & component_nodes)
            if distinct_neighbors not in {1, 2}:
                valid = False
                break
            if distinct_neighbors == 1:
                endpoint_count += 1

        if not valid or endpoint_count != 2:
            continue

        rings_data = _detect_open_strip_component(
            _faces_to_component_verts(strip_faces)
        )
        if rings_data is None:
            continue

        extra_faces = set(face_component) - set(strip_faces)
        boundary_edges = _shaft_corner_boundary_edges(strip_faces, extra_faces)
        raw_boundary_components = _boundary_path_components(boundary_edges)
        boundary_components = []
        for boundary_component in raw_boundary_components:
            chain = _ordered_chain_verts(boundary_component)
            key = _canonical_chain_key(chain)
            if key is None or not _is_corner_boundary_chain(chain):
                continue
            boundary_components.append({
                'key': key,
                'edges': set(boundary_component),
            })

        candidate = {
            'strip_faces': set(strip_faces),
            'rings': rings_data,
            'extra_faces': extra_faces,
            'face_count': len(strip_faces),
            'boundary_components': boundary_components,
            **_strip_length_metrics(rings_data),
            **_component_span_metrics(face_component, rings_data),
        }
        candidate['sort_key'] = _strip_candidate_sort_key(candidate)
        candidates.append(candidate)

    return sorted(
        candidates,
        key=lambda item: item['sort_key'],
        reverse=True,
    )


def _boundary_path_components(edges):
    live_edges = {
        edge for edge in edges
        if edge is not None and getattr(edge, "is_valid", False)
    }
    if not live_edges:
        return []

    adjacency = {}
    for edge in live_edges:
        vert_a, vert_b = edge.verts
        adjacency.setdefault(vert_a, []).append(edge)
        adjacency.setdefault(vert_b, []).append(edge)

    branch_verts = {
        vert for vert, linked_edges in adjacency.items()
        if len(linked_edges) != 2
    }

    visited_edges = set()
    path_components = []

    def _walk_path(start_vert, start_edge):
        component = []
        current_vert = start_vert
        current_edge = start_edge

        while current_edge is not None and current_edge not in visited_edges:
            visited_edges.add(current_edge)
            component.append(current_edge)
            next_vert = current_edge.other_vert(current_vert)
            if next_vert in branch_verts:
                break

            next_edge = next(
                (
                    edge for edge in adjacency.get(next_vert, [])
                    if edge is not current_edge and edge not in visited_edges
                ),
                None,
            )
            current_vert = next_vert
            current_edge = next_edge

        if component:
            path_components.append(set(component))

    for vert in sorted(branch_verts, key=lambda item: item.index):
        for edge in sorted(adjacency.get(vert, []), key=lambda item: item.index):
            if edge in visited_edges:
                continue
            _walk_path(vert, edge)

    for edge in sorted(live_edges, key=lambda item: item.index):
        if edge in visited_edges:
            continue
        cycle_component = set()
        stack = [edge]
        visited_edges.add(edge)
        while stack:
            current = stack.pop()
            cycle_component.add(current)
            for vert in current.verts:
                for neighbor in adjacency.get(vert, []):
                    if neighbor in visited_edges:
                        continue
                    visited_edges.add(neighbor)
                    stack.append(neighbor)
        if cycle_component:
            path_components.append(cycle_component)

    return path_components


def _shaft_corner_boundary_edges(shaft_faces, corner_faces):
    split_edges = set()
    for corner_face in corner_faces:
        for edge in corner_face.edges:
            if any(
                other_face is not corner_face and other_face in shaft_faces
                for other_face in edge.link_faces
            ):
                split_edges.add(edge)
    return split_edges


def _boundary_tip_verts(split_edges):
    vert_counts = {}
    for edge in split_edges:
        if not getattr(edge, "is_valid", False):
            continue
        for vert in edge.verts:
            vert_counts[vert] = vert_counts.get(vert, 0) + 1

    return {
        vert for vert, count in vert_counts.items()
        if count == 1 and getattr(vert, "is_valid", False)
    }


def _ordered_chain_verts(edges):
    adjacency = {}
    for edge in edges:
        if not getattr(edge, "is_valid", False):
            continue
        vert_a, vert_b = edge.verts
        adjacency.setdefault(vert_a, []).append(vert_b)
        adjacency.setdefault(vert_b, []).append(vert_a)

    if not adjacency:
        return []

    endpoints = [
        vert for vert, neighbors in adjacency.items()
        if len(neighbors) == 1
    ]
    if endpoints:
        start = min(endpoints, key=lambda vert: vert.index)
    else:
        start = min(adjacency, key=lambda vert: vert.index)

    ordered = [start]
    visited = {start}
    current = start

    while len(ordered) < len(adjacency):
        next_vert = None
        for neighbor in sorted(adjacency[current], key=lambda vert: vert.index):
            if neighbor not in visited:
                next_vert = neighbor
                break
        if next_vert is None:
            break
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered


def _boundary_chain_verts(split_edges):
    verts = set()
    for edge in split_edges:
        if not getattr(edge, "is_valid", False):
            continue
        for vert in edge.verts:
            if getattr(vert, "is_valid", False):
                verts.add(vert)
    return verts


def _is_corner_boundary_chain(chain):
    if len(chain) < 2:
        return False

    last_index = len(chain) - 1
    for index, vert in enumerate(chain):
        if not getattr(vert, "is_valid", False):
            return False

        edge_count = len(vert.link_edges)
        if index == 0 or index == last_index:
            if edge_count not in {2, 3}:
                return False
        elif edge_count != 4:
            return False

    return True


def _split_control_verts(split_edges):
    tip_verts = _boundary_tip_verts(split_edges)
    control_verts = set()
    for vert in _boundary_chain_verts(split_edges):
        if not getattr(vert, "is_valid", False):
            continue

        edge_count = len(vert.link_edges)
        if vert in tip_verts:
            if edge_count in {2, 3}:
                control_verts.add(vert)
        elif edge_count == 4:
            control_verts.add(vert)

    return control_verts


def _tip_shaft_rail_edges(split_edges, shaft_faces, corner_faces):
    extra_edges = set()
    for vert in _boundary_tip_verts(split_edges):
        for edge in vert.link_edges:
            if edge in split_edges or not getattr(edge, "is_valid", False):
                continue

            linked_faces = {
                face for face in edge.link_faces
                if getattr(face, "is_valid", False)
            }
            if not (linked_faces & shaft_faces):
                continue
            if linked_faces & corner_faces:
                continue

            extra_edges.add(edge)

    return extra_edges


def _select_only_faces(bm, selected_faces):
    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    face_set = {
        face for face in selected_faces
        if face is not None and getattr(face, "is_valid", False)
    }
    for face in face_set:
        face.select = True
        for edge in face.edges:
            if edge.is_valid:
                edge.select = True
        for vert in face.verts:
            if vert.is_valid:
                vert.select = True


def _select_only_edges(bm, selected_edges):
    for face in bm.faces:
        if face.is_valid:
            face.select = False
    for edge in bm.edges:
        if edge.is_valid:
            edge.select = False
    for vert in bm.verts:
        if vert.is_valid:
            vert.select = False

    edge_set = {
        edge for edge in selected_edges
        if edge is not None and getattr(edge, "is_valid", False)
    }
    for edge in edge_set:
        edge.select = True
        for vert in edge.verts:
            if vert.is_valid:
                vert.select = True


def _canonical_chain_key(chain):
    ids = tuple(
        vert.index for vert in chain
        if vert is not None and getattr(vert, "is_valid", False)
    )
    if not ids:
        return None
    reversed_ids = tuple(reversed(ids))
    return min(ids, reversed_ids)


def _route_face_key(faces):
    return tuple(
        sorted(
            face.index for face in faces
            if face is not None and getattr(face, "is_valid", False)
        )
    )


def _all_seam_records_from_face_components(face_components):
    seam_records = {}
    all_shaft_faces = set()
    for face_component in face_components:
        candidates = _open_face_strip_candidates(face_component)
        if not candidates:
            _debug_step("candidate filter", component_face_count=len(face_component), kept=0)
            continue

        chosen_candidates = []
        occupied_faces = set()
        for candidate in candidates:
            strip_faces = _live_bmesh_items(candidate.get('strip_faces', set()))
            if not strip_faces or strip_faces & occupied_faces:
                continue
            normalized_candidate = {
                **candidate,
                'strip_faces': strip_faces,
                'extra_faces': _live_bmesh_items(candidate.get('extra_faces', set())),
            }
            normalized_candidate['sort_key'] = candidate.get('sort_key', _strip_candidate_sort_key(normalized_candidate))
            chosen_candidates.append(normalized_candidate)
            occupied_faces.update(strip_faces)

        _debug_step(
            "candidate filter",
            component_face_count=len(face_component),
            kept=len(chosen_candidates),
        )
        for candidate in chosen_candidates:
            all_shaft_faces.update(candidate.get('strip_faces', set()))

        seam_entries = {}
        for candidate_index, candidate in enumerate(chosen_candidates, start=1):
            shaft_faces = candidate.get('strip_faces', set())
            corner_faces = candidate.get('extra_faces', set())
            if not shaft_faces or not corner_faces:
                continue

            boundary_components = candidate.get('boundary_components', [])
            _debug_step(
                "candidate seams",
                component_face_count=len(face_component),
                candidate_index=candidate_index,
                shaft_face_count=len(shaft_faces),
                corner_face_count=len(corner_faces),
                seam_count=len(boundary_components),
            )
            for boundary_component in boundary_components:
                key = boundary_component['key']
                record = seam_entries.setdefault(
                    key,
                    {
                        'boundary_edges': set(),
                        'candidate_ids': set(),
                        'candidate_data': {},
                    },
                )
                record['boundary_edges'].update(boundary_component['edges'])
                record['candidate_ids'].add(candidate_index)
                record['candidate_data'][candidate_index] = {
                    'shaft_faces': set(shaft_faces),
                    'corner_faces': set(corner_faces),
                    'sort_key': candidate.get('sort_key', ()),
                }
        _debug_step(
            "candidate chain",
            component_face_count=len(face_component),
            path=[candidate_index for candidate_index in range(1, len(chosen_candidates) + 1)],
        )

        component_strong_records = {}
        component_fallback_records = {}
        for seam_key, seam_entry in seam_entries.items():
            representative_id = max(
                seam_entry['candidate_ids'],
                key=lambda candidate_id: seam_entry['candidate_data'][candidate_id]['sort_key'],
            )
            representative = seam_entry['candidate_data'][representative_id]
            record = {
                'key': seam_key,
                'boundary_edges': set(seam_entry['boundary_edges']),
                'shaft_faces': set(representative['shaft_faces']),
                'corner_faces': set(representative['corner_faces']),
                'support': len(seam_entry['candidate_ids']),
                'shaft_face_count': len(representative['shaft_faces']),
                'sort_key': representative['sort_key'],
            }
            if record['support'] >= 2:
                component_strong_records[seam_key] = record
            else:
                component_fallback_records[seam_key] = record

        chosen_component_records = (
            component_strong_records
            if component_strong_records else component_fallback_records
        )
        seam_records.update(chosen_component_records)

    filtered_records = sorted(
        seam_records.values(),
        key=lambda item: (
            -item['support'],
            _sort_key_desc(item.get('sort_key', ())),
            item['key'],
        )
    )
    return filtered_records, _face_positions(all_shaft_faces)


def _all_seam_records(bm):
    selected_faces = _selected_face_set(bm, _selected_verts(bm))
    if not selected_faces:
        return [], []
    return _all_seam_records_from_face_components(_face_components(selected_faces))


def _detect_corner_endpoint_seed(bm):
    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        _trace_corner("Endpoint-seed corner detect rejected: no selected components.")
        return None

    route_records = {}
    for component_index, component in enumerate(selected_components, start=1):
        component_set = {
            vert for vert in component
            if getattr(vert, "is_valid", False)
        }
        if len(component_set) < 2:
            continue

        start_faces = {
            face
            for vert in component_set
            for face in vert.link_faces
            if getattr(face, "is_valid", False) and len(face.verts) == 4
        }
        if not start_faces:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="no adjacent quad faces",
            )
            return None

        face_component = _connected_quad_faces(start_faces)
        if len(face_component) < 2:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="connected quad component too small",
                face_count=len(face_component),
            )
            return None

        matched_candidate = None
        for candidate in _open_face_strip_candidates(face_component):
            if not candidate.get('extra_faces'):
                continue
            trimmed_group = _trim_group_to_terminal_subset(candidate['rings'], component_set)
            if trimmed_group is None:
                continue
            matched_candidate = candidate
            break

        if matched_candidate is None:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="no corner strip candidate matched terminal selection",
                component_face_count=len(face_component),
            )
            return None

        route_key = _route_face_key(matched_candidate.get('strip_faces', set()))
        if not route_key:
            _trace_corner(
                "Endpoint-seed corner detect rejected.",
                component_index=component_index,
                reason="matched candidate had no shaft faces",
            )
            return None

        route_record = route_records.setdefault(
            route_key,
            {
                'group': matched_candidate['rings'],
                'component_faces': set(),
                'shaft_faces': set(),
                'extra_faces': set(),
            },
        )
        route_record['component_faces'].update(face_component)
        route_record['shaft_faces'].update(
            _live_bmesh_items(matched_candidate.get('strip_faces', set()))
        )
        route_record['extra_faces'].update(
            _live_bmesh_items(matched_candidate.get('extra_faces', set()))
        )
        _trace_corner(
            "Endpoint-seed corner detect matched component.",
            component_index=component_index,
            component_face_count=len(face_component),
            shaft_face_count=len(route_record['shaft_faces']),
            corner_face_count=len(route_record['extra_faces']),
        )

    seed_components = [
        {
            'face_component': set(record['component_faces']),
            'shaft_faces': set(record['shaft_faces']),
            'extra_faces': set(record['extra_faces']),
        }
        for record in route_records.values()
        if record['extra_faces']
    ]
    if not seed_components:
        _trace_corner("Endpoint-seed corner detect rejected: no corner components found.")
        return None

    _trace_corner(
        "Endpoint-seed corner detect matched.",
        group_count=len(route_records),
        corner_component_count=len(seed_components),
    )
    return {
        'groups': [record['group'] for record in route_records.values()],
        'components': [],
        'seed_components': seed_components,
        'mode_label': 'Bridged open loop with corners',
    }


def _selected_verts_match_any_loop(rings_data, selected_verts):
    loops, is_closed = rings_data
    if is_closed or not loops:
        return None

    selected_set = {
        vert for vert in selected_verts
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_score = None
    for loop_index, loop in enumerate(loops, start=1):
        loop_set = {
            vert for vert in loop
            if getattr(vert, "is_valid", False)
        }
        match_count = len(selected_set & loop_set)
        if match_count == 0:
            continue

        selected_coverage = match_count / max(1, len(selected_set))
        loop_coverage = match_count / max(1, len(loop_set))
        score = (
            match_count,
            round(selected_coverage, 6),
            round(loop_coverage, 6),
            -loop_index,
        )
        if best_score is None or score > best_score:
            best_score = score

    return best_score


def _selected_subset_range_any_loop(rings_data, selected_component):
    loops, is_closed = rings_data
    if is_closed or not loops:
        return None

    selected_chain = _order_open_chain_component(selected_component)
    if selected_chain is None:
        selected_chain = [
            vert for vert in selected_component
            if getattr(vert, "is_valid", False)
        ]

    selected_set = {
        vert for vert in selected_chain
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_range = None
    best_score = None
    for loop_index, loop in enumerate(loops):
        loop_positions = {
            vert: index for index, vert in enumerate(loop)
            if getattr(vert, "is_valid", False)
        }
        matched_indices = [
            loop_positions[vert]
            for vert in selected_chain
            if vert in loop_positions
        ]
        average_distance = 0.0

        if len(matched_indices) < 2:
            matched_indices = []
            total_distance = 0.0
            valid_matches = 0
            loop_verts = [
                vert for vert in loop
                if getattr(vert, "is_valid", False)
            ]
            if len(loop_verts) < 2:
                continue
            for selected_vert in selected_chain:
                if not getattr(selected_vert, "is_valid", False):
                    continue
                nearest_index = min(
                    range(len(loop_verts)),
                    key=lambda index: (loop_verts[index].co - selected_vert.co).length,
                )
                total_distance += (loop_verts[nearest_index].co - selected_vert.co).length
                matched_indices.append(nearest_index)
                valid_matches += 1
            if valid_matches < 2:
                continue
            average_distance = total_distance / valid_matches

        matched_indices = sorted(set(matched_indices))
        if len(matched_indices) < 2:
            continue

        start_index = matched_indices[0]
        end_index = matched_indices[-1]
        span_count = end_index - start_index + 1
        if span_count < 2:
            continue

        match_count = len(matched_indices)
        selected_coverage = match_count / max(1, len(selected_set))
        span_fill = match_count / max(1, span_count)
        score = (
            match_count,
            round(selected_coverage, 6),
            round(span_fill, 6),
            -round(average_distance, 6),
            -loop_index,
        )
        if best_score is None or score > best_score:
            best_score = score
            best_range = {
                'loop_index': loop_index,
                'start_index': start_index,
                'end_index': end_index,
                'score': score,
            }

    return best_range


def _slice_loop_for_partial_range(loop, partial_range):
    if partial_range is None:
        return list(loop)

    start_index = partial_range.get('start_index', 0)
    end_index = partial_range.get('end_index', len(loop) - 1)
    if start_index < 0 or end_index >= len(loop) or start_index >= end_index:
        return list(loop)
    return list(loop[start_index:end_index + 1])


def _detect_corner_any_cross_section_seed(bm):
    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        _trace_focus("Cross-section input missing.")
        return None

    _trace_focus(
        "Cross-section input.",
        component_count=len(selected_components),
        component_sizes=[len(component) for component in selected_components],
    )

    matched_groups = []
    partial_ranges = []
    group_keys = []
    for component_index, component in enumerate(selected_components, start=1):
        component_set = {
            vert for vert in component
            if getattr(vert, "is_valid", False)
        }
        if len(component_set) < 2:
            _trace_focus(
                "Selected cross section skipped.",
                component_index=component_index,
                reason="component too small",
                component_size=len(component_set),
            )
            continue

        start_faces = {
            face
            for vert in component_set
            for face in vert.link_faces
            if getattr(face, "is_valid", False) and len(face.verts) == 4
        }
        if not start_faces:
            _trace_focus(
                "Selected cross section match failed.",
                component_index=component_index,
                reason="no adjacent quad faces",
                component_size=len(component_set),
            )
            return None

        face_component = _connected_quad_faces(start_faces)
        if len(face_component) < 2:
            _trace_focus(
                "Selected cross section match failed.",
                component_index=component_index,
                reason="connected quad component too small",
                component_size=len(component_set),
                start_face_count=len(start_faces),
                face_count=len(face_component),
            )
            return None

        candidates = _open_face_strip_candidates(face_component)
        _trace_focus(
            "Selected cross section candidate scan.",
            component_index=component_index,
            component_size=len(component_set),
            start_face_count=len(start_faces),
            component_face_count=len(face_component),
            candidate_count=len(candidates),
            candidate_loop_counts=[len(candidate['rings'][0]) for candidate in candidates],
            candidate_loop_sizes=[
                len(candidate['rings'][0][0]) if candidate['rings'][0] else 0
                for candidate in candidates
            ],
        )

        matched_candidate = None
        matched_score = None
        fallback_candidate = None
        for candidate_index, candidate in enumerate(candidates, start=1):
            if fallback_candidate is None:
                fallback_candidate = candidate
            candidate_score = _selected_verts_match_any_loop(candidate['rings'], component_set)
            _trace_focus(
                "Selected cross section candidate result.",
                component_index=component_index,
                candidate_index=candidate_index,
                loop_count=len(candidate['rings'][0]),
                loop_size=len(candidate['rings'][0][0]) if candidate['rings'][0] else 0,
                match_score=candidate_score,
            )
            if candidate_score is None:
                continue
            if matched_score is None or candidate_score > matched_score:
                matched_candidate = candidate
                matched_score = candidate_score

        if matched_candidate is None:
            matched_candidate = fallback_candidate
            if matched_candidate is None:
                _trace_focus(
                    "Selected cross section match failed.",
                    component_index=component_index,
                    reason="no bridge candidates in connected quad component",
                    component_size=len(component_set),
                    start_face_count=len(start_faces),
                    component_face_count=len(face_component),
                    candidate_count=0,
                )
                return None
            _trace_focus(
                "Selected cross section route fallback.",
                component_index=component_index,
                reason="no candidate loop overlapped the selected verts",
                component_size=len(component_set),
                component_face_count=len(face_component),
                candidate_count=len(candidates),
            )

        matched_groups.append(matched_candidate['rings'])
        partial_range = _selected_subset_range_any_loop(
            matched_candidate['rings'],
            component,
        )
        partial_ranges.append(partial_range)
        group_key = _group_debug_key(matched_candidate['rings'])
        group_keys.append(group_key)
        _trace_focus(
            "Selected cross section matched route.",
            component_index=component_index,
            component_size=len(component_set),
            component_face_count=len(face_component),
            loop_count=len(matched_candidate['rings'][0]),
            loop_size=len(matched_candidate['rings'][0][0]) if matched_candidate['rings'][0] else 0,
            match_score=matched_score,
            partial_range=partial_range,
            group_key=group_key,
        )

    if not matched_groups:
        _trace_focus("Cross-section input failed to match any route.")
        return None

    duplicate_route_count = len(group_keys) - len(set(group_keys))
    _trace_focus(
        "Cross-section routes matched.",
        group_count=len(matched_groups),
        duplicate_route_count=duplicate_route_count,
        group_keys=group_keys,
    )
    return {
        'groups': matched_groups,
        'components': [],
        'mode_label': 'Bridged open loop with corners',
        'detector_only': True,
        'partial_ranges': partial_ranges,
    }


def _attach_partial_ranges(bm, data):
    if not data or data.get('partial_ranges') is not None:
        return data

    groups = data.get('groups', [])
    if not groups:
        return data

    selected_components = _selected_vert_components(_selected_verts(bm))
    if not selected_components:
        return data

    partial_ranges = []
    matched_any = False
    for group in groups:
        best_range = None
        best_score = None
        for component in selected_components:
            partial_range = _selected_subset_range_any_loop(group, component)
            if partial_range is None:
                continue
            range_score = partial_range.get('score')
            if best_score is None or range_score > best_score:
                best_range = partial_range
                best_score = range_score
        partial_ranges.append(best_range)
        matched_any = matched_any or best_range is not None

    if not matched_any:
        return data

    updated = dict(data)
    updated['partial_ranges'] = partial_ranges
    _trace_corner(
        "Attached partial ranges to detected groups.",
        group_count=len(groups),
        partial_ranges=partial_ranges,
    )
    return updated


def _detect_corner_from_open_groups(groups):
    _trace_corner(
        "Trying plain-route corner detect from open groups.",
        group_count=len(groups or []),
    )
    route_records = {}
    for group_index, group in enumerate(groups or [], start=1):
        shaft_faces = _shaft_faces_from_open_group(group)
        if not shaft_faces:
            _trace_corner(
                "Plain-route corner detect rejected.",
                group_index=group_index,
                reason="could not derive shaft faces from detected open group",
            )
            return None

        face_component, live_shaft_faces = _corner_face_component_from_shaft_faces(shaft_faces)
        extra_faces = {
            face for face in face_component
            if face not in live_shaft_faces and getattr(face, "is_valid", False)
        }
        if not extra_faces:
            _trace_corner(
                "Plain-route corner detect skipped non-corner group.",
                group_index=group_index,
                shaft_face_count=len(live_shaft_faces),
            )
            continue

        route_key = _route_face_key(live_shaft_faces)
        if not route_key:
            continue

        route_record = route_records.setdefault(
            route_key,
            {
                'group': group,
                'component_faces': set(),
                'shaft_faces': set(),
                'extra_faces': set(),
            },
        )
        route_record['component_faces'].update(face_component)
        route_record['shaft_faces'].update(live_shaft_faces)
        route_record['extra_faces'].update(extra_faces)
        _trace_corner(
            "Plain-route corner detect matched group.",
            group_index=group_index,
            component_face_count=len(face_component),
            shaft_face_count=len(live_shaft_faces),
            corner_face_count=len(extra_faces),
        )

    seed_components = [
        {
            'face_component': set(record['component_faces']),
            'shaft_faces': set(record['shaft_faces']),
            'extra_faces': set(record['extra_faces']),
        }
        for record in route_records.values()
        if record['extra_faces']
    ]
    if not seed_components:
        _trace_corner("Plain-route corner detect rejected: no corner components found.")
        return None

    _trace_corner(
        "Plain-route corner detect matched.",
        group_count=len(route_records),
        corner_component_count=len(seed_components),
    )
    return {
        'groups': [record['group'] for record in route_records.values()],
        'components': [],
        'seed_components': seed_components,
        'mode_label': 'Bridged open loop with corners',
    }


def _loop_walk_key(loop):
    return tuple(
        vert.index for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    )


def _order_open_chain_component(component_verts):
    component_set = {
        vert for vert in component_verts
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(component_set) < 2:
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component too small",
            component_size=len(component_set),
        )
        return None

    adjacency = {
        vert: sorted(
            (
                edge.other_vert(vert)
                for edge in vert.link_edges
                if edge.other_vert(vert) in component_set
            ),
            key=lambda item: item.index,
        )
        for vert in component_set
    }
    degree_counts = sorted(len(neighbors) for neighbors in adjacency.values())
    if any(len(neighbors) not in {1, 2} for neighbors in adjacency.values()):
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component degree pattern was not an open strip",
            component_size=len(component_set),
            degree_counts=degree_counts,
            bad_vertices=[
                vert.index for vert, neighbors in adjacency.items()
                if len(neighbors) not in {1, 2}
            ],
        )
        return None

    endpoints = [
        vert for vert, neighbors in adjacency.items()
        if len(neighbors) == 1
    ]
    if len(endpoints) != 2:
        _trace_corner(
            "Start detection ordering rejected.",
            reason="component did not have exactly two endpoints",
            component_size=len(component_set),
            endpoint_count=len(endpoints),
            endpoint_indices=[vert.index for vert in endpoints],
            degree_counts=degree_counts,
        )
        return None

    start = min(endpoints, key=lambda vert: vert.index)
    ordered = [start]
    visited = {start}
    current = start

    while len(ordered) < len(component_set):
        next_vert = next(
            (neighbor for neighbor in adjacency[current] if neighbor not in visited),
            None,
        )
        if next_vert is None:
            _trace_corner(
                "Start detection ordering rejected.",
                reason="ordering walk got stuck before covering the component",
                component_size=len(component_set),
                ordered_size=len(ordered),
                current_vert=current.index,
            )
            return None
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered


def _adjacent_bridge_loops_from_seed(seed_loop):
    seed_set = {
        vert for vert in seed_loop
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(seed_set) != len(seed_loop):
        _trace_corner(
            "Adjacent-loop detect rejected.",
            reason="seed loop contained duplicate or invalid verts",
            seed_size=len(seed_loop),
            unique_size=len(seed_set),
        )
        return None

    partner_map = {}
    all_partners = set()
    for vert in seed_loop:
        partners = []
        for edge in vert.link_edges:
            other = edge.other_vert(vert)
            if other in seed_set or not getattr(other, "is_valid", False):
                continue
            partners.append(other)
        deduped_partners = sorted(set(partners), key=lambda item: item.index)
        if not deduped_partners:
            _trace_corner(
                "Adjacent-loop detect rejected.",
                reason="seed vert had no outward partners",
                seed_vert=vert.index,
            )
            return None
        partner_map[vert] = deduped_partners
        all_partners.update(deduped_partners)

    _trace_corner(
        "Adjacent-loop detect: gathered outward partners.",
        seed_size=len(seed_loop),
        partner_counts={
            vert.index: len(partners)
            for vert, partners in partner_map.items()
        },
        total_partner_count=len(all_partners),
    )

    visited = set()
    components = []
    for vert in sorted(all_partners, key=lambda item: item.index):
        if vert in visited:
            continue
        stack = [vert]
        component = set()
        visited.add(vert)
        while stack:
            current = stack.pop()
            component.add(current)
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other not in all_partners or other in visited:
                    continue
                visited.add(other)
                stack.append(other)
        components.append(component)

    full_size_components = [
        component for component in components
        if len(component) == len(seed_loop)
    ]
    ignored_components = [
        component for component in components
        if len(component) != len(seed_loop)
    ]

    if ignored_components:
        _trace_corner(
            "Adjacent-loop detect: ignoring non-matching side components.",
            seed_size=len(seed_loop),
            ignored_component_sizes=[len(component) for component in ignored_components],
        )

    if len(full_size_components) not in {1, 2}:
        _trace_corner(
            "Adjacent-loop detect rejected.",
            reason="outward partners did not form one or two full-size side components",
            component_count=len(components),
            component_sizes=[len(component) for component in components],
            full_size_component_count=len(full_size_components),
        )
        return None

    aligned_loops = []
    for component in full_size_components:
        aligned = []
        for vert in seed_loop:
            matches = [partner for partner in partner_map[vert] if partner in component]
            if len(matches) != 1:
                _trace_corner(
                    "Adjacent-loop detect rejected.",
                    reason="seed vert did not map cleanly into one side component",
                    seed_vert=vert.index,
                    match_count=len(matches),
                    component_size=len(component),
                )
                return None
            aligned.append(matches[0])

        if not _is_ordered_open_chain(aligned):
            reversed_aligned = list(reversed(aligned))
            if _is_ordered_open_chain(reversed_aligned):
                aligned = reversed_aligned
            else:
                ordered_component = _order_open_chain_component(component)
                if ordered_component is None:
                    return None
                if _is_ordered_open_chain(ordered_component):
                    aligned = ordered_component
                elif _is_ordered_open_chain(list(reversed(ordered_component))):
                    aligned = list(reversed(ordered_component))
                else:
                    _trace_corner(
                        "Adjacent-loop detect rejected.",
                        reason="side component could not be ordered into an open chain",
                        component_size=len(component),
                    )
                    return None

        aligned_loops.append(aligned)

    _trace_corner(
        "Derived adjacent bridge loops from selected cross section.",
        seed_size=len(seed_loop),
        adjacent_loop_count=len(aligned_loops),
    )
    return aligned_loops


def _next_partial_bridge_loop(current_loop, previous_loop=None):
    current_set = {
        vert for vert in current_loop
        if vert is not None and getattr(vert, "is_valid", False)
    }
    previous_set = {
        vert for vert in (previous_loop or ())
        if vert is not None and getattr(vert, "is_valid", False)
    }
    if len(current_set) != len(current_loop):
        _trace_corner(
            "Partial-walk next-loop rejected.",
            reason="current loop contained duplicate or invalid verts",
            current_size=len(current_loop),
            unique_size=len(current_set),
        )
        return None

    partner_map = {}
    all_partners = set()
    for vert in current_loop:
        partners = []
        for edge in vert.link_edges:
            other = edge.other_vert(vert)
            if (
                other in current_set
                or other in previous_set
                or not getattr(other, "is_valid", False)
            ):
                continue
            partners.append(other)
        deduped_partners = sorted(set(partners), key=lambda item: item.index)
        partner_map[vert] = deduped_partners
        if deduped_partners:
            all_partners.update(deduped_partners)

    if not all_partners:
        return []

    visited = set()
    components = []
    for vert in sorted(all_partners, key=lambda item: item.index):
        if vert in visited:
            continue
        stack = [vert]
        component = set()
        visited.add(vert)
        while stack:
            current = stack.pop()
            component.add(current)
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other not in all_partners or other in visited:
                    continue
                visited.add(other)
                stack.append(other)
        components.append(component)

    full_size_components = [
        component for component in components
        if len(component) == len(current_loop)
    ]

    ignored_components = [
        component for component in components
        if len(component) != len(current_loop)
    ]
    if ignored_components:
        _trace_corner(
            "Partial-walk next-loop: ignoring non-matching side components.",
            current_size=len(current_loop),
            ignored_component_sizes=[len(component) for component in ignored_components],
        )

    if not full_size_components:
        _trace_corner(
            "Partial-walk next-loop reached terminal end.",
            current_size=len(current_loop),
            partner_component_sizes=[len(component) for component in components],
        )
        return []

    if len(full_size_components) != 1:
        _trace_corner(
            "Partial-walk next-loop rejected.",
            reason="could not isolate exactly one full-size next component",
            current_size=len(current_loop),
            component_sizes=[len(component) for component in components],
            full_size_component_count=len(full_size_components),
        )
        return None

    component = full_size_components[0]
    aligned = []
    for vert in current_loop:
        matches = [partner for partner in partner_map[vert] if partner in component]
        if len(matches) != 1:
            _trace_corner(
                "Partial-walk next-loop rejected.",
                reason="current vert did not map cleanly into next component",
                current_vert=vert.index,
                match_count=len(matches),
                component_size=len(component),
            )
            return None
        aligned.append(matches[0])

    if not _is_ordered_open_chain(aligned):
        reversed_aligned = list(reversed(aligned))
        if _is_ordered_open_chain(reversed_aligned):
            aligned = reversed_aligned
        else:
            ordered_component = _order_open_chain_component(component)
            if ordered_component is None:
                _trace_corner(
                    "Partial-walk next-loop rejected.",
                    reason="next component could not be ordered",
                    component_size=len(component),
                )
                return None
            if _is_ordered_open_chain(ordered_component):
                aligned = ordered_component
            elif _is_ordered_open_chain(list(reversed(ordered_component))):
                aligned = list(reversed(ordered_component))
            else:
                _trace_corner(
                    "Partial-walk next-loop rejected.",
                    reason="ordered next component was not an open chain",
                    component_size=len(component),
                )
                return None

    return aligned


def _walk_bridge_side(seed_loop, first_loop, visited_keys, seed_key=None, close_on_seed=False):
    if first_loop is None:
        return None
    if not first_loop:
        return {
            'loops': [],
            'closed': False,
        }

    walked = []
    previous_loop = seed_loop
    current_loop = first_loop

    while current_loop:
        current_key = _loop_walk_key(current_loop)
        if not current_key:
            return None
        if current_key in visited_keys:
            if close_on_seed and seed_key is not None and current_key == seed_key:
                _trace_corner(
                    "Partial-walk side closed back on seed.",
                    walked_loop_count=len(walked),
                )
                return {
                    'loops': walked,
                    'closed': True,
                }
            return None
        visited_keys.add(current_key)
        walked.append(current_loop)

        next_loop = _next_partial_bridge_loop(current_loop, previous_loop=previous_loop)
        if next_loop is None:
            return None

        previous_loop = current_loop
        current_loop = next_loop

    return {
        'loops': walked,
        'closed': False,
    }


def _expand_bridge_from_seed_loop(seed_loop, close_on_seed=False):
    if seed_loop is None or len(seed_loop) < 2:
        _trace_corner(
            "Seed expansion rejected.",
            reason="seed loop was missing or too small",
            seed_size=0 if seed_loop is None else len(seed_loop),
        )
        return None

    seed_key = _loop_walk_key(seed_loop)
    if not seed_key:
        _trace_corner(
            "Seed expansion rejected.",
            reason="seed loop key was empty",
            seed_size=len(seed_loop),
        )
        return None

    visited_keys = {seed_key}
    adjacent_loops = _adjacent_bridge_loops_from_seed(seed_loop)
    if adjacent_loops is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="could not derive adjacent loops from selected strip",
            seed_size=len(seed_loop),
        )
        return None

    first_side = adjacent_loops[0] if adjacent_loops else []
    second_side = adjacent_loops[1] if len(adjacent_loops) > 1 else []

    left_result = _walk_bridge_side(
        seed_loop,
        first_side,
        visited_keys,
        seed_key=seed_key,
        close_on_seed=close_on_seed,
    )
    if left_result is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="left-side walk failed",
            first_side_size=len(first_side) if first_side else 0,
        )
        return None
    left_loops = left_result['loops']

    if left_result.get('closed'):
        expanded_loops = [list(seed_loop)] + left_loops
        if len(expanded_loops) < 2:
            return None
        _trace_corner(
            "Expanded closed bridge from selected cross section.",
            seed_size=len(seed_loop),
            expanded_loop_count=len(expanded_loops),
        )
        return (expanded_loops, True)

    right_result = _walk_bridge_side(
        seed_loop,
        second_side,
        visited_keys,
        seed_key=seed_key,
        close_on_seed=close_on_seed,
    )
    if right_result is None:
        _trace_corner(
            "Seed expansion rejected.",
            reason="right-side walk failed",
            second_side_size=len(second_side) if second_side else 0,
        )
        return None
    right_loops = right_result['loops']

    if right_result.get('closed'):
        expanded_loops = [list(seed_loop)] + right_loops
        if len(expanded_loops) < 2:
            return None
        _trace_corner(
            "Expanded closed bridge from selected cross section.",
            seed_size=len(seed_loop),
            expanded_loop_count=len(expanded_loops),
        )
        return (expanded_loops, True)

    expanded_loops = list(reversed(left_loops)) + [list(seed_loop)] + right_loops
    if len(expanded_loops) < 2:
        return None

    _trace_corner(
        "Expanded bridge from selected cross section.",
        seed_size=len(seed_loop),
        expanded_loop_count=len(expanded_loops),
    )
    return (expanded_loops, False)


def _expand_open_bridge_from_seed_loop(seed_loop):
    return _expand_bridge_from_seed_loop(seed_loop, close_on_seed=False)


def _expand_closed_bridge_from_seed_loop(seed_loop):
    return _expand_bridge_from_seed_loop(seed_loop, close_on_seed=True)


def _find_group_from_seed_loop(groups, selected_verts):
    selected_set = {
        vert for vert in selected_verts
        if getattr(vert, "is_valid", False)
    }
    if not selected_set:
        return None

    best_group = None
    best_score = None
    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if is_closed or not loops:
            continue
        for loop_index, loop in enumerate(loops, start=1):
            loop_set = {
                vert for vert in loop
                if getattr(vert, "is_valid", False)
            }
            match_count = len(selected_set & loop_set)
            if match_count == 0:
                continue
            score = (match_count, len(loop_set), -group_index, -loop_index)
            if best_score is None or score > best_score:
                best_score = score
                best_group = group

    if best_group is None:
        return None

    _trace_corner(
        "Selected cross section matched an open group.",
        selected_count=len(selected_set),
        loop_count=len(best_group[0]) if best_group and best_group[0] else 0,
    )
    return best_group


def _build_vert_position_lookup(bm, tolerance=_POSITION_TOLERANCE):
    lookup = {}
    for vert in bm.verts:
        if not getattr(vert, "is_valid", False):
            continue
        lookup.setdefault(_position_key(vert.co, tolerance), []).append(vert)
    return lookup


def _split_infos_from_boundary_components(bm, boundary_components, vert_lookup=None):
    split_infos = []
    for component in boundary_components:
        boundary_edges = component.get('boundary_edges', set())
        chain = _ordered_chain_verts(boundary_edges)
        if not chain:
            continue

        positions = [vert.co.copy() for vert in chain]
        clusters = []
        for position in positions:
            cluster = _verts_at_position(bm, position, lookup=vert_lookup)
            if len(cluster) >= 2:
                clusters.append({
                    'position': position.copy(),
                    'verts': cluster,
                })

        if clusters:
            split_infos.append({
                'anchor_home': positions[0].copy(),
                'clusters': clusters,
            })

    return split_infos


def _same_position(vec_a, vec_b, tolerance=_POSITION_TOLERANCE):
    return (vec_a - vec_b).length <= tolerance


def _group_loops(group_data):
    if isinstance(group_data, dict):
        rings = group_data.get('rings')
        if rings is None:
            return []
        return list(rings[0])
    return list(group_data[0])


def _separate_stuck_boundary_tips(bm, boundary_components, seam_index=None):
    changed = False
    vert_lookup = _build_vert_position_lookup(bm)
    lookup_dirty = False

    def _live_verts(position):
        nonlocal vert_lookup, lookup_dirty
        if lookup_dirty:
            vert_lookup = _build_vert_position_lookup(bm)
            lookup_dirty = False
        return _verts_at_position(bm, position, lookup=vert_lookup)

    for component_index, component in enumerate(boundary_components, start=1):
        boundary_edges = component.get('boundary_edges', set())
        corner_faces = {
            face for face in component.get('corner_faces', set())
            if face is not None and getattr(face, "is_valid", False)
        }
        chain = _ordered_chain_verts(boundary_edges)
        if len(chain) < 2:
            continue

        tip_specs = (
            (chain[0].co.copy(), chain[1].co.copy()),
            (chain[-1].co.copy(), chain[-2].co.copy()),
        )

        for tip_position, neighbor_position in tip_specs:
            live_tip_verts = _live_verts(tip_position)
            _debug_step(
                "tip inspect",
                seam_index=seam_index,
                component_index=component_index,
                live_tip_count=len(live_tip_verts),
                corner_face_count=len(corner_faces),
            )
            if len(live_tip_verts) != 1:
                continue

            tip_vert = live_tip_verts[0]
            seam_edges = [
                edge for edge in tip_vert.link_edges
                if _same_position(edge.other_vert(tip_vert).co, neighbor_position)
            ]
            _debug_step(
                "tip seam edges",
                seam_index=seam_index,
                component_index=component_index,
                seam_edge_count=len(seam_edges),
                edge_count=len(tip_vert.link_edges),
            )
            if not seam_edges:
                continue

            separated = bmesh.utils.vert_separate(tip_vert, seam_edges)
            lookup_dirty = True
            _debug_step(
                "tip vert separate",
                seam_index=seam_index,
                component_index=component_index,
                result_count=len(separated),
            )
            if len(separated) > 1:
                changed = True
                continue

            live_tip_verts = _live_verts(tip_position)
            if len(live_tip_verts) != 1:
                continue

            face_tip_vert = live_tip_verts[0]
            candidate_loops = [
                loop for loop in face_tip_vert.link_loops
                if getattr(loop.face, "is_valid", False) and loop.face in corner_faces
            ]
            if not candidate_loops:
                candidate_loops = list(face_tip_vert.link_loops)

            _debug_step(
                "tip loop fallback",
                seam_index=seam_index,
                component_index=component_index,
                loop_count=len(candidate_loops),
            )
            for loop in candidate_loops:
                live_tip_verts = _live_verts(tip_position)
                if len(live_tip_verts) > 1:
                    break
                try:
                    bmesh.utils.loop_separate(loop)
                    lookup_dirty = True
                    _debug_step(
                        "tip loop separated",
                        seam_index=seam_index,
                        component_index=component_index,
                        face_index=getattr(loop.face, "index", None),
                        live_tip_count=len(_live_verts(tip_position)),
                    )
                    changed = True
                except Exception:
                    _debug_step(
                        "tip loop failed",
                        seam_index=seam_index,
                        component_index=component_index,
                        face_index=getattr(loop.face, "index", None),
                    )
                    continue

    if changed:
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.select_flush_mode()
        bm.normal_update()

    return changed


def _split_seam_records(bm, seam_records):
    if not seam_records:
        return []

    boundary_components = []
    interface_chain_verts = set()
    split_edges = set()
    for record in seam_records:
        boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        shaft_faces = _live_bmesh_items(record.get('shaft_faces', set()))
        corner_faces = _live_bmesh_items(record.get('corner_faces', set()))
        if not boundary_edges or not shaft_faces or not corner_faces:
            continue

        boundary_components.append({
            'boundary_edges': set(boundary_edges),
            'corner_faces': set(corner_faces),
        })
        interface_chain_verts.update(_split_control_verts(boundary_edges))
        split_edges.update(boundary_edges)
        split_edges.update(_tip_shaft_rail_edges(boundary_edges, shaft_faces, corner_faces))

    if not split_edges or not boundary_components:
        return []

    bmesh.ops.split_edges(
        bm,
        edges=list(split_edges),
        verts=list(interface_chain_verts),
        use_verts=bool(interface_chain_verts),
    )
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    _separate_stuck_boundary_tips(bm, boundary_components)
    bm.select_flush_mode()
    bm.normal_update()
    vert_lookup = _build_vert_position_lookup(bm)
    return _split_infos_from_boundary_components(bm, boundary_components, vert_lookup=vert_lookup)


def _verts_at_position(bm, position, tolerance=_POSITION_TOLERANCE, lookup=None):
    if lookup is None:
        lookup = _build_vert_position_lookup(bm, tolerance=tolerance)

    return [
        vert for vert in lookup.get(_position_key(position, tolerance), [])
        if getattr(vert, "is_valid", False)
        and (vert.co - position).length <= tolerance
    ]


def _boundary_positions_from_records(seam_records):
    positions = []
    seen = set()
    for record in seam_records:
        boundary_edges = _live_bmesh_items(record.get('boundary_edges', set()))
        if not boundary_edges:
            continue
        chain = _ordered_chain_verts(boundary_edges)
        for vert in chain:
            if not getattr(vert, "is_valid", False):
                continue
            pos = vert.co.copy()
            key = (round(pos.x, 6), round(pos.y, 6), round(pos.z, 6))
            if key not in seen:
                seen.add(key)
                positions.append(pos)
    return positions


def _weld_boundary_positions(bm, positions, vert_lookup=None):
    if not positions:
        return 0

    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for position in positions:
        live_verts = _verts_at_position(bm, position, lookup=vert_lookup)
        if len(live_verts) < 2:
            continue
        target = live_verts[0]
        for vert in live_verts[1:]:
            if vert is not target:
                targetmap[vert] = target

    if not targetmap:
        _debug_step("weld skip", reason="no duplicate verts at boundary positions")
        return 0

    try:
        bmesh.ops.weld_verts(bm, targetmap=targetmap)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.normal_update()
        _debug_step("weld done", merged=len(targetmap))
        return len(targetmap)
    except Exception:
        _debug_step("weld failed", targetmap_size=len(targetmap))
        return 0


def _cluster_loop_match_count(loop, split_info):
    loop_set = {vert for vert in loop if getattr(vert, "is_valid", False)}
    return sum(
        1
        for cluster in split_info['clusters']
        if any(vert in loop_set for vert in cluster['verts'] if getattr(vert, "is_valid", False))
    )


def _open_group_matches(groups, split_info):
    matches = []
    for group_index, group_data in enumerate(groups or [], start=1):
        loops = _group_loops(group_data)
        best_match_count = 0
        best_loop_index = None
        best_loop_size = 0
        for loop_index, loop in enumerate(loops, start=1):
            match_count = _cluster_loop_match_count(loop, split_info)
            if match_count <= 0:
                continue
            loop_size = sum(
                1 for vert in loop
                if getattr(vert, "is_valid", False)
            )
            candidate = (match_count, loop_size, -loop_index)
            if (
                best_loop_index is None
                or candidate > (best_match_count, best_loop_size, -best_loop_index)
            ):
                best_match_count = match_count
                best_loop_index = loop_index
                best_loop_size = loop_size
        if best_loop_index is None:
            continue
        matches.append({
            'group_index': group_index,
            'loop_index': best_loop_index,
            'loop_size': best_loop_size,
            'matched_positions': best_match_count,
        })
    return matches


def _anchor_index_for_open_loop(loop, anchor_home):
    if not loop:
        return None
    start_dist = (loop[0].co - anchor_home).length if getattr(loop[0], "is_valid", False) else None
    end_dist = (loop[-1].co - anchor_home).length if getattr(loop[-1], "is_valid", False) else None
    if start_dist is None and end_dist is None:
        return None
    if end_dist is not None and (start_dist is None or end_dist < start_dist):
        return -1
    return 0


def _reverse_loop_with_fixed_start(loop):
    if len(loop) <= 1:
        return list(loop)
    return [loop[0]] + list(reversed(loop[1:]))


def _pair_distance_score(loop_a, loop_b):
    if len(loop_a) != len(loop_b):
        return None
    return sum(
        (vert_a.co - vert_b.co).length
        for vert_a, vert_b in zip(loop_a, loop_b)
        if getattr(vert_a, "is_valid", False) and getattr(vert_b, "is_valid", False)
    )


def _live_open_loop_pair(groups, split_info):
    matches = _open_group_matches(groups, split_info)
    if len(matches) < 2:
        return None

    matches = sorted(
        matches,
        key=lambda item: (
            item['matched_positions'],
            item['loop_size'],
            -item['group_index'],
            -item['loop_index'],
        ),
        reverse=True,
    )
    first = matches[0]
    second = next(
        (item for item in matches[1:] if item['group_index'] != first['group_index']),
        None,
    )
    if second is None:
        return None

    pair_loops = []
    for match in (first, second):
        group_data = groups[match['group_index'] - 1]
        loops = _group_loops(group_data)
        loop = list(loops[match['loop_index'] - 1])
        if not loop:
            return None
        anchor_index = _anchor_index_for_open_loop(loop, split_info['anchor_home'])
        if anchor_index == -1:
            loop = list(reversed(loop))
        pair_loops.append(loop)

    loop_a, loop_b = pair_loops
    if len(loop_a) != len(loop_b):
        return None

    reversed_b = _reverse_loop_with_fixed_start(loop_b)
    forward_score = _pair_distance_score(loop_a, loop_b)
    reverse_score = _pair_distance_score(loop_a, reversed_b)
    if reverse_score is not None and (
        forward_score is None or reverse_score < forward_score
    ):
        loop_b = reversed_b

    return (loop_a, loop_b)


def _weld_live_open_loops(bm, split_infos):
    live_data = bridged_open_loop.detect(bm)
    groups = live_data.get('groups') if live_data else []
    if not groups:
        return 0

    targetmap = {}
    for split_info in split_infos:
        pair = _live_open_loop_pair(groups, split_info)
        if pair is None:
            continue
        loop_a, loop_b = pair
        for vert_a, vert_b in zip(loop_a, loop_b):
            if (
                not getattr(vert_a, "is_valid", False)
                or not getattr(vert_b, "is_valid", False)
                or vert_a is vert_b
            ):
                continue
            targetmap[vert_b] = vert_a

    if not targetmap:
        return 0

    try:
        bmesh.ops.weld_verts(bm, targetmap=targetmap)
    except Exception:
        return 0

    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    return len(targetmap)


def _targetmap_from_split_clusters(split_infos):
    targetmap = {}
    for split_info in split_infos:
        for cluster in split_info['clusters']:
            valid_verts = [
                vert for vert in cluster['verts']
                if getattr(vert, "is_valid", False)
            ]
            if len(valid_verts) < 2:
                continue
            target = valid_verts[0]
            for vert in valid_verts[1:]:
                if vert is not target:
                    targetmap[vert] = target
    return targetmap


def _targetmap_from_live_split_positions(bm, split_infos, vert_lookup=None):
    if vert_lookup is None:
        vert_lookup = _build_vert_position_lookup(bm)
    targetmap = {}
    for split_info in split_infos:
        for cluster in split_info['clusters']:
            live_verts = _verts_at_position(bm, cluster['position'], lookup=vert_lookup)
            if len(live_verts) < 2:
                continue
            target = live_verts[0]
            for vert in live_verts[1:]:
                if vert is not target:
                    targetmap[vert] = target
    return targetmap


def _anchor_open_group_endpoints(rings_data):
    loops, _is_closed = rings_data
    forced_seam_verts = []
    for loop in loops:
        seam_verts = set()
        if loop:
            first_vert = loop[0]
            last_vert = loop[-1]
            if getattr(first_vert, "is_valid", False):
                seam_verts.add(first_vert)
            if getattr(last_vert, "is_valid", False):
                seam_verts.add(last_vert)
        forced_seam_verts.append(seam_verts)

    return {
        'rings': rings_data,
        'use_seams': True,
        'migrate_seams': False,
        'max_seams': 2,
        'forced_seam_verts': forced_seam_verts,
    }


def _loop_key(loop):
    ids = tuple(
        vert.index for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    )
    if not ids:
        return None
    reversed_ids = tuple(reversed(ids))
    return min(ids, reversed_ids)


def _loop_center(loop):
    live_verts = [
        vert for vert in loop
        if vert is not None and getattr(vert, "is_valid", False)
    ]
    if not live_verts:
        return None

    center = live_verts[0].co.copy()
    for vert in live_verts[1:]:
        center += vert.co
    center /= len(live_verts)
    return center


def _world_loop_segments(obj, loop, is_closed=False):
    if obj is None:
        return []

    matrix_world = obj.matrix_world
    segments = []
    for vert_a, vert_b in zip(loop, loop[1:]):
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            continue
        segments.append((
            matrix_world @ vert_a.co.copy(),
            matrix_world @ vert_b.co.copy(),
        ))
    if is_closed and len(loop) > 2:
        vert_a = loop[-1]
        vert_b = loop[0]
        if (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            segments.append((
                matrix_world @ vert_a.co.copy(),
                matrix_world @ vert_b.co.copy(),
            ))
    return segments


def _chain_edges(loop, is_closed=False):
    edges = []
    pair_iter = zip(loop, loop[1:])
    for vert_a, vert_b in pair_iter:
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            return None
        edge = next(
            (
                item for item in vert_a.link_edges
                if getattr(item, "is_valid", False) and item.other_vert(vert_a) is vert_b
            ),
            None,
        )
        if edge is None:
            return None
        edges.append(edge)

    if is_closed and len(loop) > 2:
        vert_a = loop[-1]
        vert_b = loop[0]
        if not (
            getattr(vert_a, "is_valid", False)
            and getattr(vert_b, "is_valid", False)
        ):
            return None
        edge = next(
            (
                item for item in vert_a.link_edges
                if getattr(item, "is_valid", False) and item.other_vert(vert_a) is vert_b
            ),
            None,
        )
        if edge is None:
            return None
        edges.append(edge)

    return edges


def _sharp_corner_indices(points, threshold_degrees=45.0, is_closed=False):
    if len(points) < 3:
        return []

    threshold_cos = __import__("math").cos(__import__("math").radians(threshold_degrees))
    indices = []
    point_count = len(points)
    if is_closed:
        index_iter = range(point_count)
    else:
        index_iter = range(1, point_count - 1)

    for index in index_iter:
        prev_point = points[index - 1]
        current_point = points[index]
        next_point = points[(index + 1) % point_count]
        if prev_point is None or current_point is None or next_point is None:
            continue

        incoming = _safe_normalized(current_point - prev_point)
        outgoing = _safe_normalized(next_point - current_point)
        if incoming is None or outgoing is None:
            continue
        if incoming.dot(outgoing) <= threshold_cos:
            indices.append(index)

    return indices


def _cluster_corner_indices(indices, min_gap=2):
    ordered = sorted(set(indices))
    if not ordered:
        return []

    clusters = [[ordered[0]]]
    for index in ordered[1:]:
        if index - clusters[-1][-1] <= min_gap:
            clusters[-1].append(index)
        else:
            clusters.append([index])

    clustered = []
    for cluster in clusters:
        clustered.extend(cluster)
    return clustered


def _detected_cross_sections(groups, partial_ranges=None):
    end_sections = []
    corner_sections = []
    all_sections = []
    collected_section_indices = []

    _trace_focus(
        "Preview build started.",
        group_count=len(groups or []),
        partial_range_count=len(partial_ranges or []),
    )

    for group_index, group in enumerate(groups or [], start=1):
        loops, is_closed = group
        if not loops:
            continue

        if len(loops) < 2:
            continue
        if is_closed and len(loops) < 3:
            continue

        centers = [_loop_center(loop) for loop in loops]
        end_indices = set() if is_closed else {0, len(loops) - 1}
        raw_corner_indices = [
            index for index in _sharp_corner_indices(centers, is_closed=is_closed)
            if is_closed or 0 < index < len(loops) - 1
        ]
        corner_indices = _cluster_corner_indices(raw_corner_indices)
        partial_range = None
        if partial_ranges and len(partial_ranges) >= group_index:
            partial_range = partial_ranges[group_index - 1]

        _trace_focus(
            "Preview group analysis.",
            group_index=group_index,
            loop_count=len(loops),
            loop_size=len(loops[0]) if loops and loops[0] else 0,
            is_closed=is_closed,
            end_indices=sorted(end_indices),
            raw_corner_indices=raw_corner_indices,
            corner_indices=corner_indices,
            partial_range=partial_range,
            group_key=_group_debug_key(group),
        )

        for index in sorted(end_indices):
            end_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        for index in corner_indices:
            corner_sections.append(_slice_loop_for_partial_range(loops[index], partial_range))

        group_section_indices = []
        for loop_index, loop in enumerate(loops):
            all_sections.append(_slice_loop_for_partial_range(loop, partial_range))
            group_section_indices.append(loop_index)

        collected_section_indices.append({
            'group_index': group_index,
            'section_indices': group_section_indices,
        })

        _trace_focus(
            "Preview group output.",
            group_index=group_index,
            collected_section_indices=group_section_indices,
            collected_section_count=len(group_section_indices),
            end_section_count=len(sorted(end_indices)),
            corner_section_count=len(corner_indices),
        )

    return end_sections, corner_sections, all_sections, collected_section_indices


def _capture_detected_cross_sections(obj, end_sections, corner_sections, all_sections=None):
    if obj is None:
        return

    end_points = []
    corner_points = []
    all_points = []
    segments = []
    matrix_world = obj.matrix_world

    for section in end_sections:
        center = _loop_center(section)
        if center is not None:
            end_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    for section in corner_sections:
        center = _loop_center(section)
        if center is not None:
            corner_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    for section in all_sections or []:
        center = _loop_center(section)
        if center is not None:
            all_points.append(matrix_world @ center)
        segments.extend(_world_loop_segments(obj, section, is_closed=False))

    if all_points:
        anchor_overlay.add_points(all_points, label_prefix="S")
    if end_points:
        anchor_overlay.add_points(end_points, label_prefix="E")
    if corner_points:
        anchor_overlay.add_points(corner_points, label_prefix="C")
    if segments:
        anchor_overlay.add_segments(segments)

def detect(bm):
    _trace_focus("Detect started.")
    seed_loops = _selected_seed_loops(bm)
    if seed_loops:
        groups = []
        partial_ranges = []
        for component_index, seed_loop in enumerate(seed_loops, start=1):
            seed_group = _expand_open_bridge_from_seed_loop(seed_loop)
            if seed_group is None:
                seed_group = _expand_closed_bridge_from_seed_loop(seed_loop)
            if seed_group is None:
                _trace_focus(
                    "Selected seed expansion failed.",
                    component_index=component_index,
                    seed_size=len(seed_loop),
                )
                return None
            groups.append(seed_group)
            partial_ranges.append({
                'loop_index': 0,
                'start_index': 0,
                'end_index': len(seed_loop) - 1,
                'score': (len(seed_loop), 1.0, 1.0, 0.0, 0),
            })

        _trace_focus(
            "Detect path chosen: selected seed loops.",
            group_count=len(groups),
            seed_sizes=[len(seed_loop) for seed_loop in seed_loops],
            loop_counts=[len(group[0]) for group in groups],
        )
        return {
            'groups': groups,
            'components': [],
            'mode_label': 'Bridged open loop with corners',
            'detector_only': True,
            'partial_ranges': partial_ranges,
        }

    data = detect_open_strip_selection(bm)
    if data is not None and _groups_are_open(data['groups']):
        selected_verts = _selected_verts(bm)
        seed_group = _find_group_from_seed_loop(data['groups'], selected_verts)
        if seed_group is not None:
            _trace_focus(
                "Detect path chosen: matched selected cross section inside open groups.",
                group_count=len(data.get('groups', [])),
            )
            return _attach_partial_ranges(bm, {
                'groups': [seed_group],
                'components': data.get('components', []),
                'mode_label': 'Bridged open loop with corners',
                'detector_only': True,
            })

        _trace_focus(
            "Detect path chosen: generic open-group scan.",
            group_count=len(data.get('groups', [])),
            loop_counts=[len(group[0]) for group in data.get('groups', [])],
            loop_sizes=[
                len(group[0][0]) if group[0] else 0
                for group in data.get('groups', [])
            ],
        )
        return _attach_partial_ranges(bm, {
            'groups': data['groups'],
            'components': data.get('components', []),
            'mode_label': 'Bridged open loop with corners',
            'detector_only': True,
        })

    _trace_focus("Detect failed.")
    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        _trace_focus("Execute cancelled: detect returned no data.")
        return {'CANCELLED'}

    end_sections, corner_sections, all_sections, collected_section_indices = _detected_cross_sections(
        data.get('groups', []),
        partial_ranges=data.get('partial_ranges'),
    )
    _capture_detected_cross_sections(obj, end_sections, corner_sections, all_sections=all_sections)

    _trace_focus(
        "Preview build finished.",
        end_cross_section_count=len(end_sections),
        corner_cross_section_count=len(corner_sections),
        propagated_cross_section_count=len(all_sections),
        collected_section_indices=collected_section_indices,
    )

    shaft_face_positions = _face_positions(
        {
            face
            for group in data.get('groups', [])
            for face in _shaft_faces_from_open_group(group)
        }
    )
    split_edges = set()
    split_section_logs = []
    for section_index, section in enumerate(corner_sections, start=1):
        chain_edges = _chain_edges(section, is_closed=False)
        if not chain_edges:
            split_section_logs.append({
                'section_index': section_index,
                'vert_indices': [vert.index for vert in section if getattr(vert, "is_valid", False)],
                'edge_indices': None,
            })
            continue
        split_section_logs.append({
            'section_index': section_index,
            'vert_indices': [vert.index for vert in section if getattr(vert, "is_valid", False)],
            'edge_indices': [edge.index for edge in chain_edges if getattr(edge, "is_valid", False)],
            'edge_labels': [_edge_debug_label(edge) for edge in chain_edges if getattr(edge, "is_valid", False)],
        })
        split_edges.update(chain_edges)

    _trace_focus(
        "Split execution plan.",
        corner_section_count=len(corner_sections),
        split_section_logs=split_section_logs,
        split_edge_count=len(split_edges),
        split_edge_indices=sorted(
            edge.index for edge in split_edges
            if getattr(edge, "is_valid", False)
        ),
        split_edge_labels=sorted(
            _edge_debug_label(edge) for edge in split_edges
            if getattr(edge, "is_valid", False)
        ),
    )

    segment_count = 0
    if split_edges:
        bmesh.ops.split_edges(bm, edges=list(split_edges))
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bm.select_flush_mode()
        bm.normal_update()

        live_shaft_faces = _live_faces_from_positions(bm, shaft_face_positions)
        _trace_focus(
            "Post-split shaft face recovery.",
            shaft_face_count=len(live_shaft_faces),
        )
        if live_shaft_faces:
            _select_only_faces(bm, live_shaft_faces)
            bm.select_flush_mode()
            bm.normal_update()

            fresh_data = bridged_open_loop.detect(bm)
            segment_count = len(fresh_data.get('groups', [])) if fresh_data else 0
            _trace_focus(
                "Post-split open bridge detect.",
                group_count=segment_count,
            )

            if fresh_data and fresh_data.get('groups'):
                for group_index, rings_data in enumerate(fresh_data['groups'], start=1):
                    anchored_group = _anchor_open_group_endpoints(rings_data)
                    loops = anchored_group['rings'][0]
                    _trace_focus(
                        "Segment resample execute.",
                        group_index=group_index,
                        loop_count=len(loops),
                        loop_size=len(loops[0]) if loops else 0,
                    )
                    execute_aligned_loops_logic(
                        bm,
                        obj,
                        anchored_group['rings'],
                        direction,
                        report=report,
                        use_seams=anchored_group.get('use_seams', True),
                        migrate_seams=anchored_group.get('migrate_seams'),
                        max_seams=anchored_group.get('max_seams'),
                        forced_seam_verts=anchored_group.get('forced_seam_verts'),
                    )
        bmesh.update_edit_mesh(obj.data)

    if report is not None:
        if split_edges:
            report(
                {'INFO'},
                "Open loop bridge with corners split corner sections and resampled segments: "
                f"corners={len(corner_sections)}, edges={len(split_edges)}, segments={segment_count}.",
            )
        else:
            report(
                {'INFO'},
                "Open loop bridge with corners detector only: "
                f"ends={len(end_sections)}, corners={len(corner_sections)}.",
            )

    return {'FINISHED'}
