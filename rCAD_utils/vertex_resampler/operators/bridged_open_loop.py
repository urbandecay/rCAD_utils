# bridged_open_loop.py — Resample bridged open-loop selections.

from .bridge_utils import get_bridged_chain
from .resample_common import execute_aligned_loops_logic


def _selected_component_edges(component):
    edges = set()
    component_set = set(component)
    for vert in component:
        for edge in vert.link_edges:
            if edge.select and edge.other_vert(vert) in component_set:
                edges.add(edge)
    return list(edges)


def _selected_components(bm):
    selected_verts = [vert for vert in bm.verts if vert.select]
    visited = set()
    components = []

    for vert in selected_verts:
        if vert in visited:
            continue
        stack = [vert]
        component = []
        visited.add(vert)

        while stack:
            current = stack.pop()
            component.append(current)
            for edge in current.link_edges:
                if not edge.select:
                    continue
                other = edge.other_vert(current)
                if other.select and other not in visited:
                    visited.add(other)
                    stack.append(other)

        if len(component) >= 2:
            components.append(component)

    return components


def _fully_selected_faces(edge, sel_set):
    return sum(
        1 for face in edge.link_faces if all(vert in sel_set for vert in face.verts)
    )


def _order_path_from_edges(edges):
    adjacency = {}
    for edge in edges:
        v1, v2 = edge.verts
        adjacency.setdefault(v1, []).append(v2)
        adjacency.setdefault(v2, []).append(v1)

    endpoints = [vert for vert, neighbors in adjacency.items() if len(neighbors) == 1]
    if len(endpoints) != 2:
        return None

    ordered = [endpoints[0]]
    visited = {endpoints[0]}
    current = endpoints[0]

    while len(ordered) < len(adjacency):
        next_vert = None
        for neighbor in adjacency[current]:
            if neighbor not in visited:
                next_vert = neighbor
                break
        if next_vert is None:
            break
        ordered.append(next_vert)
        visited.add(next_vert)
        current = next_vert

    return ordered if len(ordered) == len(adjacency) else None


def _split_boundary_chains(boundary_edges, cut_edges):
    cut_set = set(cut_edges)
    remaining = [edge for edge in boundary_edges if edge not in cut_set]
    if not remaining:
        return None

    visited = set()
    chains = []
    for edge in remaining:
        if edge in visited:
            continue
        stack = [edge]
        component_edges = []
        visited.add(edge)

        while stack:
            current = stack.pop()
            component_edges.append(current)
            current_verts = set(current.verts)
            for other in remaining:
                if other in visited:
                    continue
                if current_verts.intersection(other.verts):
                    visited.add(other)
                    stack.append(other)

        ordered = _order_path_from_edges(component_edges)
        if ordered is None:
            return None
        chains.append(ordered)

    return chains if len(chains) == 2 else None


def _edge_lookup(edges):
    return {frozenset(edge.verts): edge for edge in edges}


def _align_chain_pair(chain_a, chain_b, edge_map):
    for candidate in (chain_b, list(reversed(chain_b))):
        if len(candidate) != len(chain_a):
            continue
        if all(
            frozenset((va, vb)) in edge_map for va, vb in zip(chain_a, candidate)
        ):
            return candidate
    return None


def _detect_bridged_strip_component(component):
    component_set = set(component)
    component_edges = _selected_component_edges(component)
    if len(component_edges) < 4:
        return None

    selected_degree = {
        vert: sum(
            1
            for edge in vert.link_edges
            if edge.select and edge.other_vert(vert) in component_set
        )
        for vert in component
    }
    if any(degree < 2 or degree > 3 for degree in selected_degree.values()):
        return None

    boundary_edges = [
        edge
        for edge in component_edges
        if _fully_selected_faces(edge, component_set) == 1
    ]
    if len(boundary_edges) < 4:
        return None

    edge_map = _edge_lookup(component_edges)
    candidate_cut_edges = [
        edge
        for edge in boundary_edges
        if all(selected_degree[vert] == 2 for vert in edge.verts)
    ]
    if len(candidate_cut_edges) < 2:
        candidate_cut_edges = boundary_edges

    best_pair = None
    best_length = -1.0

    for index, edge_a in enumerate(candidate_cut_edges):
        verts_a = set(edge_a.verts)
        for edge_b in candidate_cut_edges[index + 1:]:
            if verts_a.intersection(edge_b.verts):
                continue

            chains = _split_boundary_chains(boundary_edges, (edge_a, edge_b))
            if chains is None:
                continue

            chain_a, chain_b = chains
            if len(chain_a) != len(chain_b) or len(chain_a) < 2:
                continue

            aligned_b = _align_chain_pair(chain_a, chain_b, edge_map)
            if aligned_b is None:
                continue

            expected_edge_count = (2 * (len(chain_a) - 1)) + len(chain_a)
            if len(component_edges) != expected_edge_count:
                continue
            if len(boundary_edges) != 2 * len(chain_a):
                continue

            total_length = 0.0
            for idx in range(len(chain_a) - 1):
                total_length += (chain_a[idx].co - chain_a[idx + 1].co).length
                total_length += (aligned_b[idx].co - aligned_b[idx + 1].co).length

            if total_length > best_length:
                best_length = total_length
                best_pair = ([chain_a, aligned_b], False)

    return best_pair


def _detect_bridged_strips(bm):
    components = _selected_components(bm)
    if not components:
        return None

    strip_groups = []
    covered_verts = set()
    for component in components:
        strip_group = _detect_bridged_strip_component(component)
        if strip_group is None:
            continue
        strip_groups.append(strip_group)
        covered_verts.update(component)

    if not strip_groups:
        return None

    selected_verts = {vert for vert in bm.verts if vert.select}
    if covered_verts != selected_verts:
        return None

    return strip_groups


def detect(bm):
    bridged_data = get_bridged_chain(bm)
    if bridged_data and not bridged_data[1] and len(bridged_data[0]) >= 2:
        return {
            'groups': [bridged_data],
            'mode_label': 'Bridged open loop',
        }

    strip_groups = _detect_bridged_strips(bm)
    if strip_groups:
        return {
            'groups': strip_groups,
            'mode_label': 'Bridged open loop',
        }

    return None


def execute(bm, obj, direction, report=None, data=None):
    if data is None:
        data = detect(bm)
    if not data:
        return {'CANCELLED'}

    for rings_data in data['groups']:
        execute_aligned_loops_logic(bm, obj, rings_data, direction, report=report)
    return {'FINISHED'}
