import bpy
import bmesh
import mathutils
from math import degrees
from mathutils import Vector
from mathutils.geometry import intersect_line_plane, intersect_point_line

def edit_mode_out():
    bpy.ops.object.mode_set(mode='OBJECT')

def edit_mode_in():
    bpy.ops.object.mode_set(mode='EDIT')

def check_lukap(bm):
    if hasattr(bm.verts, "ensure_lookup_table"):
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

def get_active_element_and_its_indices(bm_data):
    for elem in reversed(bm_data.select_history):
        if isinstance(elem, (bmesh.types.BMVert, bmesh.types.BMEdge, bmesh.types.BMFace)):
            return elem.index, str(elem)[3:4]
    return None, None

def get_adjacent_vertices(edge_list):
    adj = {}
    for edge in edge_list:
        a, b = edge
        if a in adj:
            adj[a].append(b)
        else:
            adj[a] = [b]
        if b in adj:
            adj[b].append(a)
        else:
            adj[b] = [a]
    return adj

def is_loop_(endpoint_list):
    return len(endpoint_list) == 0

def form_edge_chain(first_vertex, edge_list, last_vertex):
    chain = [first_vertex]
    current = first_vertex
    remaining = edge_list.copy()

    while remaining:
        found_connection = False
        for edge in remaining:
            if edge[0] == current:
                chain.append(edge[1])
                current = edge[1]
                remaining.remove(edge)
                found_connection = True
                break
            elif edge[1] == current:
                chain.append(edge[0])
                current = edge[0]
                remaining.remove(edge)
                found_connection = True
                break
        if not found_connection:
            break
        if current == last_vertex:
            break
    return chain

def form_edge_loop(first_vertex, edge_list):
    loop = [first_vertex]
    current = first_vertex
    remaining = edge_list.copy()

    while remaining:
        found_connection = False
        for edge in remaining:
            if edge[0] == current:
                loop.append(edge[1])
                current = edge[1]
                remaining.remove(edge)
                found_connection = True
                break
            elif edge[1] == current:
                loop.append(edge[0])
                current = edge[0]
                remaining.remove(edge)
                found_connection = True
                break
        if not found_connection:
            break
        if current == first_vertex:
            break
    return loop

def calculate_edge_normal(bm_data, vertex_index, current_point, previous_point):
    if hasattr(bm_data.verts, "ensure_lookup_table"):
        bm_data.verts.ensure_lookup_table()
    vertex_co = bm_data.verts[vertex_index].co.copy()
    ip = intersect_point_line(vertex_co, current_point, previous_point)[0]
    normal = vertex_co - ip
    return normal

def get_original_connectivity(bm_data, profile_indices):
    connectivity = {
        'edges': [],
        'faces': []
    }

    if not profile_indices:
        return connectivity

    for edge in bm_data.edges:
        if edge.is_valid:
            v1_idx = edge.verts[0].index
            v2_idx = edge.verts[1].index
            if v1_idx in profile_indices and v2_idx in profile_indices:
                connectivity['edges'].append([v1_idx, v2_idx])

    for face in bm_data.faces:
        if face.is_valid:
            face_verts = [v.index for v in face.verts]
            if all(v_idx in profile_indices for v_idx in face_verts):
                connectivity['faces'].append(face_verts)

    return connectivity

def diagnose_closest_path_vertex_to_profile(bm_data, path_edge_list, profile_indices):
    if not profile_indices:
        return None

    if not path_edge_list:
        return None

    profile_center = Vector((0, 0, 0))
    valid_profile_count = 0
    for v_idx in profile_indices:
        if v_idx < len(bm_data.verts) and bm_data.verts[v_idx].is_valid:
            profile_center += bm_data.verts[v_idx].co
            valid_profile_count += 1

    if valid_profile_count == 0:
        return None

    profile_center /= valid_profile_count

    path_vertices = set()
    for edge in path_edge_list:
        path_vertices.add(edge[0])
        path_vertices.add(edge[1])

    closest_vertex = None
    min_distance = float('inf')

    for v_idx in path_vertices:
        if v_idx < len(bm_data.verts) and bm_data.verts[v_idx].is_valid:
            vertex_co = bm_data.verts[v_idx].co
            distance = (vertex_co - profile_center).length
            if distance < min_distance:
                min_distance = distance
                closest_vertex = v_idx

    return closest_vertex

def find_path_endpoints(edge_list):
    if not edge_list:
        return []

    vertex_connections = {}
    for edge in edge_list:
        v1, v2 = edge
        vertex_connections[v1] = vertex_connections.get(v1, 0) + 1
        vertex_connections[v2] = vertex_connections.get(v2, 0) + 1

    endpoints = [v for v, count in vertex_connections.items() if count == 1]
    return endpoints

def analyze_profile_anchor_connectivity(bm_data, profile_anchor, path_edges):
    adjacent = get_adjacent_vertices(path_edges)
    anchor_neighbors = adjacent.get(profile_anchor, [])
    
    if len(anchor_neighbors) <= 1:
        return "endpoint"
    else:
        return "corner_or_junction"

def determine_extrusion_start_parameters(bm_data):
    original_manual_start_point = eap_buf.list_sp[0] if eap_buf.list_sp else None
    eap_buf.original_manual_start_point = original_manual_start_point

    if not eap_buf.list_ek or original_manual_start_point is None:
        eap_buf.profile_anchor_connectivity = "unknown"
        eap_buf.effective_endpoints_for_ordering = []
        return

    profile_indices = [v.index for v in bm_data.verts if v.select and v.is_valid]
    if not profile_indices:
        eap_buf.profile_anchor_connectivity = "unknown"
        eap_buf.effective_endpoints_for_ordering = []
        return

    profile_anchor_on_path = diagnose_closest_path_vertex_to_profile(
        bm_data, eap_buf.list_ek, profile_indices
    )

    if profile_anchor_on_path is None:
        eap_buf.profile_anchor_connectivity = "unknown"
        eap_buf.effective_endpoints_for_ordering = []
        return

    path_adjacencies = get_adjacent_vertices(eap_buf.list_ek)
    initial_anchor_neighbors = path_adjacencies.get(profile_anchor_on_path, [])
    
    if len(initial_anchor_neighbors) == 1:
        profile_anchor_on_path = initial_anchor_neighbors[0]

    eap_buf.list_sp[0] = profile_anchor_on_path

    anchor_connectivity = analyze_profile_anchor_connectivity(bm_data, profile_anchor_on_path, eap_buf.list_ek)
    eap_buf.profile_anchor_connectivity = anchor_connectivity

    path_endpoints = find_path_endpoints(eap_buf.list_ek)

    if len(path_endpoints) == 0:
        eap_buf.effective_endpoints_for_ordering = []
    else:
        eap_buf.effective_endpoints_for_ordering = path_endpoints

def calculate_projection_parameters(current_point, previous_point, next_point, use_rake_mode=False):
    previous_edge = current_point - previous_point
    next_edge = current_point - next_point

    if use_rake_mode:
        prev_edge_xy = previous_edge.copy()
        next_edge_xy = next_edge.copy()
        prev_edge_xy.z = 0
        next_edge_xy.z = 0

        xy_angle = prev_edge_xy.angle(next_edge_xy) if prev_edge_xy.length > 0 and next_edge_xy.length > 0 else 0

        if round(degrees(xy_angle)) in (0, 180):
            projection_point = current_point
            projection_normal = prev_edge_xy
        else:
            projection_point = current_point.copy()
            vec_towards_prev = previous_point - current_point 
            vec_towards_next = next_point - current_point
            vec_towards_prev.z = 0
            vec_towards_next.z = 0
            vec_towards_prev_normalized = vec_towards_prev.normalized()
            vec_towards_next_normalized = vec_towards_next.normalized()
            bisector_direction = vec_towards_prev_normalized + vec_towards_next_normalized
            bisected_xy = bisector_direction.normalized()
            z_vec = Vector((0, 0, 1))
            projection_normal = bisected_xy.cross(z_vec)
            projection_normal.normalize()
    else:
        ang = previous_edge.angle(next_edge)
        if round(degrees(ang)) in (0, 180):
            projection_point = current_point
            projection_normal = previous_edge
        else:
            projection_point = current_point.copy()
            vec_towards_prev = previous_point - current_point 
            vec_towards_next = next_point - current_point
            vec_towards_prev_normalized = vec_towards_prev.normalized()
            vec_towards_next_normalized = vec_towards_next.normalized()
            bisector_direction = vec_towards_prev_normalized + vec_towards_next_normalized
            plane_normal_of_edges = vec_towards_prev.cross(vec_towards_next)
            projection_normal = bisector_direction.normalized().cross(plane_normal_of_edges.normalized())
            projection_normal.normalize()

    return projection_point, projection_normal

def _generate_corner_profile_on_L_path_unified(bm_data, list_fl, path_is_loop, use_rake_mode=False):
    list_of_selected_profile_indices = [v.index for v in bm_data.verts if v.select and v.is_valid]
    if not list_of_selected_profile_indices and hasattr(eap_buf, "profile_verts"):
        list_of_selected_profile_indices = [
            idx for idx in eap_buf.profile_verts
            if idx < len(bm_data.verts) and bm_data.verts[idx].is_valid
        ]
    if not list_of_selected_profile_indices:
        return []

    if path_is_loop:
        edge_sequence_list = form_edge_loop(eap_buf.list_sp[0], eap_buf.list_ek.copy())
        if edge_sequence_list:
            del edge_sequence_list[-1]
    else:
        end_index = list_fl[1] if eap_buf.list_sp[0] == list_fl[0] else list_fl[0]
        edge_sequence_list = form_edge_chain(eap_buf.list_sp[0], eap_buf.list_ek.copy(), end_index)

    length_of_edge_sequence = len(edge_sequence_list)
    count_of_profile_vertices = len(list_of_selected_profile_indices)
    copy_of_selected_profile_indices = list(list_of_selected_profile_indices)
    dict_of_new_vertex_position = {k: [k] for k in list_of_selected_profile_indices}

    for current_edge_index in range(length_of_edge_sequence):
        if path_is_loop:
            if current_edge_index == 0:
                continue
        else:
            if current_edge_index == 0 or current_edge_index == (length_of_edge_sequence - 1):
                continue

        current_point = bm_data.verts[edge_sequence_list[current_edge_index]].co.copy()
        previous_point = bm_data.verts[edge_sequence_list[(current_edge_index - 1) % length_of_edge_sequence]].co.copy()
        next_point = bm_data.verts[edge_sequence_list[(current_edge_index + 1) % length_of_edge_sequence]].co.copy()

        projection_point, projection_normal = calculate_projection_parameters(
            current_point, previous_point, next_point, use_rake_mode=use_rake_mode
        )

        previous_edge = current_point - previous_point
        if use_rake_mode:
            prev_edge_xy = previous_edge.copy()
            prev_edge_xy.z = 0
            offset_direction_normalized = prev_edge_xy.normalized()
        else:
            offset_direction_normalized = previous_edge.normalized()

        for j in range(count_of_profile_vertices):
            v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
            new_co = intersect_line_plane(
                v,
                v + (offset_direction_normalized * 0.1),
                projection_point,
                projection_normal
            )
            new_vert = bm_data.verts.new(new_co)
            bm_data.verts.index_update()
            if hasattr(bm_data.verts, "ensure_lookup_table"):
                bm_data.verts.ensure_lookup_table()
            copy_of_selected_profile_indices[j] = new_vert.index
            dict_of_new_vertex_position[list_of_selected_profile_indices[j]].append(new_vert.index)

    new_verts = []
    for original_index, vert_chain in dict_of_new_vertex_position.items():
        if len(vert_chain) > 1:
            new_verts.extend(vert_chain[1:])

    return new_verts

def trace_path_to_end(start_vertex, first_neighbor, edge_list):
    adjacent_for_trace = get_adjacent_vertices(edge_list)
    
    current = first_neighbor
    previous = start_vertex
    actual_path_traced = [start_vertex, current]
    
    max_iterations = len(edge_list) + 5 
    iterations = 0

    while iterations < max_iterations:
        iterations += 1
        neighbors_of_current = adjacent_for_trace.get(current, [])
        next_candidates = [n for n in neighbors_of_current if n != previous]
        
        if not next_candidates:
            return current, actual_path_traced
        elif len(next_candidates) == 1:
            previous = current
            current = next_candidates[0]
            actual_path_traced.append(current) 
        else:
            return current, actual_path_traced
            
    return current, actual_path_traced

def get_ordered_path(bm_data, all_stored_path_edges_ek, profile_anchor_sp, true_path_endpoints_fl, is_loop_path):
    if is_loop_path:
        full_loop = form_edge_loop(profile_anchor_sp, all_stored_path_edges_ek.copy())
        if full_loop and len(full_loop) > 1 and full_loop[0] == full_loop[-1]:
            full_loop.pop()
        return full_loop if (full_loop and len(full_loop) > 1) else [], True, []

    adj = get_adjacent_vertices(all_stored_path_edges_ek)
    anchor_neighbors = adj.get(profile_anchor_sp, [])

    if not anchor_neighbors:
        return [profile_anchor_sp] if profile_anchor_sp is not None else [], False, [] 

    generated_paths = []
    generated_fl_data = []

    for neighbor in anchor_neighbors:
        end_of_this_branch, current_branch_chain = trace_path_to_end(
            profile_anchor_sp, neighbor, all_stored_path_edges_ek
        )

        if current_branch_chain and len(current_branch_chain) > 1: 
            is_duplicate = False
            for existing_path in generated_paths:
                if (current_branch_chain == existing_path or 
                    (len(current_branch_chain) == len(existing_path) and 
                     all(current_branch_chain[i] == existing_path[-(i+1)] for i in range(len(current_branch_chain))))):
                    is_duplicate = True
                    break
            if not is_duplicate:
                generated_paths.append(current_branch_chain)
                generated_fl_data.append([current_branch_chain[0], current_branch_chain[-1]])

    if not generated_paths:
        return [profile_anchor_sp] if profile_anchor_sp is not None else [], False, []
    elif len(generated_paths) == 1:
        return generated_paths[0], False, generated_fl_data[0]
    else: 
        return generated_paths, False, generated_fl_data

def get_ordered_path_legacy(bm_data, all_selected_edges_ek, start_point_sp, first_last_verts_fl, is_loop):
    if is_loop:
        full_loop = form_edge_loop(start_point_sp, all_selected_edges_ek.copy())
        if full_loop and len(full_loop) > 1 and full_loop[0] == full_loop[-1]:
            full_loop.pop()
        return full_loop, True, []
    else:
        if len(first_last_verts_fl) >= 2:
            other_endpoint = first_last_verts_fl[1] if start_point_sp == first_last_verts_fl[0] else first_last_verts_fl[0]
        else:
            other_endpoint = first_last_verts_fl[0] if first_last_verts_fl else start_point_sp
        
        full_chain = form_edge_chain(start_point_sp, all_selected_edges_ek.copy(), other_endpoint)
        
        if full_chain:
            return full_chain, False, [full_chain[0], full_chain[-1]]
        else:
            return [], False, []

class EAPBuffer:
    def __init__(self):
        self.list_ek = []
        self.list_sp = []
        self.profile_verts = []
        self.new_faces = []
        self.profile_anchor_connectivity = "unknown"
        self.effective_endpoints_for_ordering = []
        self.original_manual_start_point = None
        self.force_straight_start_orientation = False

eap_buf = EAPBuffer()