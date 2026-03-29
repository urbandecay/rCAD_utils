# pipe.py — Detect and resample closed quad pipe shells.

from mathutils import Vector

from .resample_common import execute_aligned_loops_logic


def _selected_face_set(bm, sel_set):
    return {
        face for face in bm.faces
        if len(face.verts) >= 3 and (
            face.select
            or all(vert in sel_set for vert in face.verts)
            or all(edge.select for edge in face.edges)
        )
    }


def _selected_face_neighbors(face, selected_faces):
    neighbors = set()
    for edge in face.edges:
        for other_face in edge.link_faces:
            if other_face is face or other_face not in selected_faces:
                continue
            neighbors.add(other_face)
    return neighbors


def _face_components(faces):
    visited = set()
    components = []

    for face in sorted(faces, key=lambda item: item.index):
        if face in visited:
            continue

        component = set()
        stack = [face]
        visited.add(face)

        while stack:
            current = stack.pop()
            component.add(current)
            for neighbor in sorted(
                _selected_face_neighbors(current, faces),
                key=lambda item: item.index,
            ):
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

        components.append(component)

    return components


def _component_edge_face_counts(component):
    counts = {}
    for face in component:
        if len(face.verts) != 4:
            return None
        for edge in face.edges:
            counts[edge] = counts.get(edge, 0) + 1
    return counts


def _component_verts(component):
    verts = set()
    for face in component:
        verts.update(face.verts)
    return verts


def _ordered_face_edges(face):
    loops = list(face.loops)
    if len(loops) != 4:
        return None
    return [loop.edge for loop in loops]


def _is_closed_quad_shell(component):
    edge_face_counts = _component_edge_face_counts(component)
    if edge_face_counts is None:
        return None
    if any(count != 2 for count in edge_face_counts.values()):
        return None

    component_edges = set(edge_face_counts)
    for vert in _component_verts(component):
        degree = sum(1 for edge in vert.link_edges if edge in component_edges)
        if degree != 4:
            return None

    return component_edges


def _color_component_edges(component, component_edges):
    component_faces = set(component)
    colors = {}

    for start_edge in sorted(component_edges, key=lambda item: item.index):
        if start_edge in colors:
            continue

        colors[start_edge] = 0
        stack = [start_edge]

        while stack:
            edge = stack.pop()
            bit = colors[edge]

            for face in sorted(edge.link_faces, key=lambda item: item.index):
                if face not in component_faces:
                    continue

                ordered_edges = _ordered_face_edges(face)
                if ordered_edges is None:
                    return None

                try:
                    edge_index = ordered_edges.index(edge)
                except ValueError:
                    continue

                constraints = (
                    (ordered_edges[(edge_index + 2) % 4], bit),
                    (ordered_edges[(edge_index + 1) % 4], 1 - bit),
                    (ordered_edges[(edge_index - 1) % 4], 1 - bit),
                )

                for other_edge, expected_bit in constraints:
                    if other_edge not in component_edges:
                        return None

                    current_bit = colors.get(other_edge)
                    if current_bit is None:
                        colors[other_edge] = expected_bit
                        stack.append(other_edge)
                        continue
                    if current_bit != expected_bit:
                        return None

    return colors


def _order_edge_loop(edges):
    adjacency = {}
    for edge in edges:
        v1, v2 = edge.verts
        adjacency.setdefault(v1, []).append(v2)
        adjacency.setdefault(v2, []).append(v1)

    if not adjacency:
        return None
    if any(len(neighbors) != 2 for neighbors in adjacency.values()):
        return None

    start = min(adjacency, key=lambda vert: vert.index)
    ordered = [start]
    previous = None
    current = start

    while True:
        next_vert = None
        for neighbor in sorted(adjacency[current], key=lambda vert: vert.index):
            if neighbor is previous:
                continue
            next_vert = neighbor
            break

        if next_vert is None:
            return None
        if next_vert is start:
            break
        if next_vert in ordered:
            return None

        ordered.append(next_vert)
        previous = current
        current = next_vert

    return ordered if len(ordered) == len(adjacency) else None


def _edge_components(edges):
    if not edges:
        return []

    vert_edges = {}
    for edge in edges:
        for vert in edge.verts:
            vert_edges.setdefault(vert, set()).add(edge)

    visited = set()
    components = []

    for edge in sorted(edges, key=lambda item: item.index):
        if edge in visited:
            continue

        component = set()
        stack = [edge]
        visited.add(edge)

        while stack:
            current = stack.pop()
            component.add(current)
            for vert in current.verts:
                for other in sorted(vert_edges[vert], key=lambda item: item.index):
                    if other in visited:
                        continue
                    visited.add(other)
                    stack.append(other)

        components.append(component)

    return components


def _orientation_loops(component_edges, edge_colors, bit):
    edges = {
        edge for edge in component_edges
        if edge_colors.get(edge) == bit
    }
    if not edges:
        return None

    loops = []
    for edge_component in _edge_components(edges):
        ordered = _order_edge_loop(edge_component)
        if ordered is None or len(ordered) < 3:
            return None
        loops.append(ordered)

    return sorted(
        loops,
        key=lambda loop: min(vert.index for vert in loop),
    )


def _newell_normal(loop):
    normal = Vector((0.0, 0.0, 0.0))
    count = len(loop)
    for index, vert in enumerate(loop):
        next_vert = loop[(index + 1) % count]
        normal.x += (vert.co.y - next_vert.co.y) * (vert.co.z + next_vert.co.z)
        normal.y += (vert.co.z - next_vert.co.z) * (vert.co.x + next_vert.co.x)
        normal.z += (vert.co.x - next_vert.co.x) * (vert.co.y + next_vert.co.y)
    return normal


def _loop_planarity_score(loop):
    normal = _newell_normal(loop)
    if normal.length <= 1e-8:
        return float('inf')

    centroid = sum((vert.co for vert in loop), Vector((0.0, 0.0, 0.0))) / len(loop)
    plane_normal = normal.normalized()
    return sum(
        abs((vert.co - centroid).dot(plane_normal))
        for vert in loop
    ) / len(loop)


def _loop_normal_alignment_score(loops):
    normals = []
    for loop in loops:
        normal = _newell_normal(loop)
        if normal.length <= 1e-8:
            return float('inf')
        normals.append(normal.normalized())

    reference = normals[0]
    return sum(
        1.0 - abs(reference.dot(normal))
        for normal in normals[1:]
    ) / max(len(normals) - 1, 1)


def _aligned_neighbor_loop(source_loop, target_loop_set, cross_edges):
    aligned = []
    for vert in source_loop:
        matches = [
            edge.other_vert(vert)
            for edge in sorted(vert.link_edges, key=lambda item: item.index)
            if edge in cross_edges and edge.other_vert(vert) in target_loop_set
        ]
        if len(matches) != 1:
            return None
        aligned.append(matches[0])
    return aligned


def _order_loop_stack(loops, component_edges, edge_colors, ring_bit):
    if len(loops) < 4:
        return None

    cross_edges = {
        edge for edge in component_edges
        if edge_colors.get(edge) == 1 - ring_bit
    }
    loop_sets = [set(loop) for loop in loops]
    loop_index_by_vert = {}

    for index, loop in enumerate(loops):
        if len(loop) != len(loops[0]):
            return None
        for vert in loop:
            loop_index_by_vert[vert] = index

    for vert in loop_index_by_vert:
        neighbor_loops = {
            loop_index_by_vert[edge.other_vert(vert)]
            for edge in vert.link_edges
            if edge in cross_edges and edge.other_vert(vert) in loop_index_by_vert
        }
        if len(neighbor_loops) != 2:
            return None

    start_index = min(
        range(len(loops)),
        key=lambda item: min(vert.index for vert in loops[item]),
    )
    start_loop = loops[start_index]
    first_neighbors = sorted({
        loop_index_by_vert[edge.other_vert(start_loop[0])]
        for edge in start_loop[0].link_edges
        if edge in cross_edges and edge.other_vert(start_loop[0]) in loop_index_by_vert
    })
    if len(first_neighbors) != 2:
        return None

    for first_neighbor in first_neighbors:
        aligned_neighbor = _aligned_neighbor_loop(
            start_loop,
            loop_sets[first_neighbor],
            cross_edges,
        )
        if aligned_neighbor is None:
            continue

        ordered = [list(start_loop), aligned_neighbor]
        visited = {start_index, first_neighbor}
        previous_index = start_index
        current_index = first_neighbor
        current_loop = aligned_neighbor
        valid = True

        while len(visited) < len(loops):
            next_candidates = sorted({
                loop_index_by_vert[edge.other_vert(current_loop[0])]
                for edge in current_loop[0].link_edges
                if edge in cross_edges
                and edge.other_vert(current_loop[0]) in loop_index_by_vert
                and loop_index_by_vert[edge.other_vert(current_loop[0])] != previous_index
            })

            next_loop = None
            next_index = None
            for candidate in next_candidates:
                if candidate in visited:
                    continue
                next_loop = _aligned_neighbor_loop(
                    current_loop,
                    loop_sets[candidate],
                    cross_edges,
                )
                if next_loop is not None:
                    next_index = candidate
                    break

            if next_loop is None or next_index is None:
                valid = False
                break

            ordered.append(next_loop)
            visited.add(next_index)
            previous_index = current_index
            current_index = next_index
            current_loop = next_loop

        if valid:
            return ordered

    return None


def _score_loop_family(ordered_loops):
    return (
        _loop_normal_alignment_score(ordered_loops),
        sum(_loop_planarity_score(loop) for loop in ordered_loops) / len(ordered_loops),
        -len(ordered_loops),
    )


def _detect_pipe_component(component):
    component_edges = _is_closed_quad_shell(component)
    if component_edges is None:
        return None

    edge_colors = _color_component_edges(component, component_edges)
    if edge_colors is None:
        return None

    candidates = []
    for ring_bit in (0, 1):
        loops = _orientation_loops(component_edges, edge_colors, ring_bit)
        if loops is None:
            continue

        ordered_loops = _order_loop_stack(loops, component_edges, edge_colors, ring_bit)
        if ordered_loops is None:
            continue

        candidates.append((
            _score_loop_family(ordered_loops),
            ordered_loops,
        ))

    if not candidates:
        return None

    _, ordered_loops = min(candidates, key=lambda item: item[0])
    return {
        'rings': (ordered_loops, True),
        'use_seams': False,
        'migrate_seams': False,
        'stack_is_cyclic': True,
    }


def detect(bm):
    sel_verts = {vert for vert in bm.verts if vert.select}
    selected_faces = _selected_face_set(bm, sel_verts)
    if not selected_faces:
        return None

    groups = []
    for component in _face_components(selected_faces):
        group = _detect_pipe_component(component)
        if group is not None:
            groups.append(group)

    if not groups:
        return None

    return {
        'groups': groups,
        'mode_label': 'Pipe',
    }


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for group_data in data['groups']:
        execute_aligned_loops_logic(
            bm,
            obj,
            group_data['rings'],
            direction,
            report=report,
            use_seams=group_data.get('use_seams', False),
            migrate_seams=group_data.get('migrate_seams', False),
            stack_is_cyclic=group_data.get('stack_is_cyclic', False),
        )

    return {'FINISHED'}
