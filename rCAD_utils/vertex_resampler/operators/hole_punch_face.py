# hole_punch_face.py — Detect punched holes cut into a flat face.

from ..ring_analyzer import analyze_rings


def _order_boundary_loop(edges):
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


def _boundary_loops(edges):
    edge_neighbors = {}
    vert_edges = {}
    for edge in edges:
        edge_neighbors[edge] = set()
        for vert in edge.verts:
            vert_edges.setdefault(vert, set()).add(edge)

    for edge in edges:
        for vert in edge.verts:
            edge_neighbors[edge].update(vert_edges[vert])
        edge_neighbors[edge].discard(edge)

    visited = set()
    loops = []

    for edge in sorted(edges, key=lambda item: item.index):
        if edge in visited:
            continue

        component_edges = set()
        stack = [edge]
        visited.add(edge)

        while stack:
            current = stack.pop()
            component_edges.add(current)
            for other in sorted(edge_neighbors[current], key=lambda item: item.index):
                if other in visited:
                    continue
                visited.add(other)
                stack.append(other)

        ordered = _order_boundary_loop(component_edges)
        if ordered is None:
            return None
        loops.append((ordered, component_edges))

    return loops


def _selected_edge_graph(selected_edges):
    adjacency = {}
    for edge in selected_edges:
        v1, v2 = edge.verts
        adjacency.setdefault(v1, []).append((v2, edge))
        adjacency.setdefault(v2, []).append((v1, edge))
    return adjacency


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


def _is_closed_quad_shell(component):
    edge_face_counts = _component_edge_face_counts(component)
    if edge_face_counts is None:
        return False
    if any(count != 2 for count in edge_face_counts.values()):
        return False

    component_edges = set(edge_face_counts)
    component_verts = set()
    for face in component:
        component_verts.update(face.verts)

    for vert in component_verts:
        degree = sum(1 for edge in vert.link_edges if edge in component_edges)
        if degree != 4:
            return False

    return True


def _has_closed_quad_shell_selection(bm):
    sel_verts = {vert for vert in bm.verts if vert.select}
    selected_faces = _selected_face_set(bm, sel_verts)
    if not selected_faces:
        return False

    return any(
        _is_closed_quad_shell(component)
        for component in _face_components(selected_faces)
    )


def _tree_path_edges(parent_edge, depth, vert_a, vert_b):
    path_edges = []
    current_a = vert_a
    current_b = vert_b

    while depth[current_a] > depth[current_b]:
        edge = parent_edge[current_a]
        path_edges.append(edge)
        current_a = edge.other_vert(current_a)

    tail_edges = []
    while depth[current_b] > depth[current_a]:
        edge = parent_edge[current_b]
        tail_edges.append(edge)
        current_b = edge.other_vert(current_b)

    while current_a is not current_b:
        edge_a = parent_edge[current_a]
        edge_b = parent_edge[current_b]
        path_edges.append(edge_a)
        tail_edges.append(edge_b)
        current_a = edge_a.other_vert(current_a)
        current_b = edge_b.other_vert(current_b)

    path_edges.extend(reversed(tail_edges))
    return path_edges


def _extract_cycle_loops(selected_edges):
    adjacency = _selected_edge_graph(selected_edges)
    visited = set()
    parent_edge = {}
    depth = {}
    tree_edges = set()
    loops = []
    seen_cycles = set()

    def visit(start_vert):
        stack = [(start_vert, None, 0, iter(sorted(adjacency[start_vert], key=lambda item: item[1].index)))]
        visited.add(start_vert)
        depth[start_vert] = 0
        parent_edge[start_vert] = None

        while stack:
            current, parent_vert, current_depth, edge_iter = stack[-1]
            try:
                other, edge = next(edge_iter)
            except StopIteration:
                stack.pop()
                continue

            if other is parent_vert:
                continue

            if other not in visited:
                visited.add(other)
                parent_edge[other] = edge
                depth[other] = current_depth + 1
                tree_edges.add(edge)
                stack.append(
                    (
                        other,
                        current,
                        current_depth + 1,
                        iter(sorted(adjacency[other], key=lambda item: item[1].index)),
                    )
                )
                continue

            if depth.get(other, 0) >= depth[current]:
                continue

            cycle_edges = set(_tree_path_edges(parent_edge, depth, current, other))
            cycle_edges.add(edge)
            cycle_key = tuple(sorted(item.index for item in cycle_edges))
            if cycle_key in seen_cycles:
                continue
            seen_cycles.add(cycle_key)

            ordered = _order_boundary_loop(cycle_edges)
            if ordered is not None:
                loops.append((ordered, cycle_edges))

    for vert in sorted(adjacency, key=lambda item: item.index):
        if vert in visited:
            continue
        visit(vert)

    return loops


def _loop_area(loop):
    if len(loop) < 3:
        return 0.0

    xs = [vert.co.x for vert in loop]
    ys = [vert.co.y for vert in loop]
    zs = [vert.co.z for vert in loop]
    ranges = [
        max(xs) - min(xs),
        max(ys) - min(ys),
        max(zs) - min(zs),
    ]
    drop_axis = min(range(3), key=lambda index: ranges[index])

    coords = []
    for vert in loop:
        if drop_axis == 0:
            coords.append((vert.co.y, vert.co.z))
        elif drop_axis == 1:
            coords.append((vert.co.x, vert.co.z))
        else:
            coords.append((vert.co.x, vert.co.y))

    area = 0.0
    for index in range(len(coords)):
        x1, y1 = coords[index]
        x2, y2 = coords[(index + 1) % len(coords)]
        area += (x1 * y2) - (x2 * y1)
    return abs(area) * 0.5


def _loop_centroid_2d(loop):
    if not loop:
        return (0.0, 0.0)

    xs = [vert.co.x for vert in loop]
    ys = [vert.co.y for vert in loop]
    zs = [vert.co.z for vert in loop]
    ranges = [
        max(xs) - min(xs),
        max(ys) - min(ys),
        max(zs) - min(zs),
    ]
    drop_axis = min(range(3), key=lambda index: ranges[index])

    coords = []
    for vert in loop:
        if drop_axis == 0:
            coords.append((vert.co.y, vert.co.z))
        elif drop_axis == 1:
            coords.append((vert.co.x, vert.co.z))
        else:
            coords.append((vert.co.x, vert.co.y))

    cx = sum(coord[0] for coord in coords) / len(coords)
    cy = sum(coord[1] for coord in coords) / len(coords)
    return (cx, cy)


def _point_in_loop(point, loop):
    xs = [vert.co.x for vert in loop]
    ys = [vert.co.y for vert in loop]
    zs = [vert.co.z for vert in loop]
    ranges = [
        max(xs) - min(xs),
        max(ys) - min(ys),
        max(zs) - min(zs),
    ]
    drop_axis = min(range(3), key=lambda index: ranges[index])

    coords = []
    for vert in loop:
        if drop_axis == 0:
            coords.append((vert.co.y, vert.co.z))
        elif drop_axis == 1:
            coords.append((vert.co.x, vert.co.z))
        else:
            coords.append((vert.co.x, vert.co.y))

    px, py = point
    inside = False
    prev_x, prev_y = coords[-1]
    for curr_x, curr_y in coords:
        intersects = ((curr_y > py) != (prev_y > py))
        if intersects:
            edge_x = prev_x + ((curr_x - prev_x) * (py - prev_y) / (curr_y - prev_y))
            if edge_x > px:
                inside = not inside
        prev_x, prev_y = curr_x, curr_y
    return inside


def _dedupe_loops(loop_data):
    seen = set()
    deduped = []
    for ring, component_edges in sorted(
        loop_data,
        key=lambda item: (_loop_area(item[0]), min(vert.index for vert in item[0])),
    ):
        key = tuple(sorted(vert.index for vert in ring))
        if key in seen:
            continue
        seen.add(key)
        deduped.append((ring, component_edges))
    return deduped


def _disjoint_loops(loop_data):
    accepted = []
    used_verts = set()
    for ring, component_edges in sorted(
        loop_data,
        key=lambda item: (_loop_area(item[0]), min(vert.index for vert in item[0])),
    ):
        ring_set = set(ring)
        if ring_set.intersection(used_verts):
            continue
        accepted.append((ring, component_edges))
        used_verts.update(ring_set)
    return accepted


def detect(bm, report=None):
    selected_edges = {edge for edge in bm.edges if edge.select}
    if not selected_edges:
        return None

    if _has_closed_quad_shell_selection(bm):
        if report is not None:
            report({'INFO'}, "Face hole check: rejected closed quad shell selection")
        return None

    loop_data = _extract_cycle_loops(selected_edges)
    if not loop_data:
        if report is not None:
            report({'INFO'}, "Face hole check: no closed loops found")
        return None

    candidate_loops = []
    for ring, component_edges in loop_data:
        if len(ring) < 4:
            continue
        ring_group = analyze_rings([ring], is_closed=True)
        if not ring_group.rings[0].seam_verts:
            continue
        candidate_loops.append((ring, component_edges))

    if not candidate_loops:
        if report is not None:
            report(
                {'INFO'},
                f"Face hole check: loops={len(loop_data)} hole-rings=0",
            )
        return None

    container_loops = set()
    for index_a, (loop_a, _edges_a) in enumerate(candidate_loops):
        area_a = _loop_area(loop_a)
        for index_b, (loop_b, _edges_b) in enumerate(candidate_loops):
            if index_a == index_b:
                continue
            if area_a <= _loop_area(loop_b):
                continue
            if _point_in_loop(_loop_centroid_2d(loop_b), loop_a):
                container_loops.add(index_a)
                break

    filtered_loops = [
        candidate_loops[index]
        for index in range(len(candidate_loops))
        if index not in container_loops
    ]
    if filtered_loops:
        candidate_loops = filtered_loops

    candidate_loops = _dedupe_loops(candidate_loops)
    candidate_loops = _disjoint_loops(candidate_loops)

    if report is not None:
        report(
            {'INFO'},
            f"Face hole check: loops={len(loop_data)} hole-rings={len(candidate_loops)}",
        )

    all_ring_component_edges = set()
    for _ring, component_edges in candidate_loops:
        all_ring_component_edges.update(component_edges)

    cleanup_edges = {
        edge for edge in selected_edges
        if edge not in all_ring_component_edges and edge.is_valid
    }
    cleanup_ring_edges = {
        edge for edge in all_ring_component_edges
        if edge.is_valid
    }

    groups = []
    for ring, component_edges in candidate_loops:
        ring_group = analyze_rings([ring], is_closed=True)
        seam_verts = ring_group.rings[0].seam_verts

        use_seams = len(seam_verts) < len(ring)

        groups.append({
            'rings': ([ring], True),
            'use_seams': use_seams,
            'migrate_seams': False,
            'max_seams': None,
            'repair_topology': False,
            'align_seams_to_ring_normals': True,
            'cleanup_edges': cleanup_edges,
            'cleanup_ring_edges': cleanup_ring_edges,
        })

    if not groups:
        if report is not None:
            report({'INFO'}, "Face hole check: groups=0")
        return None

    return {
        'groups': groups,
        'invalid_components': 0,
        'mode_label': 'Face hole punch',
    }
