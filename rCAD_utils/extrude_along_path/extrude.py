import bmesh
import math
import mathutils
from math import degrees
from mathutils import Vector
from mathutils.geometry import intersect_line_plane, intersect_point_line
from mathutils import Matrix

from .helper_functions import (
    form_edge_chain,
    form_edge_loop,
    calculate_edge_normal,
    calculate_projection_parameters,
    eap_buf,
    get_adjacent_vertices,
    _generate_corner_profile_on_L_path_unified,
    get_original_connectivity
)

def create_endpoint_projected_profile(bm_data, original_profile_indices, endpoint_vertex_idx, outgoing_direction, use_rake_mode=False):
    if not original_profile_indices:
        return []
    
    if hasattr(bm_data.verts, "ensure_lookup_table"):
        bm_data.verts.ensure_lookup_table()
    
    endpoint_co = bm_data.verts[endpoint_vertex_idx].co.copy()
    
    if use_rake_mode:
        outgoing_xy = outgoing_direction.copy()
        outgoing_xy.z = 0
        if outgoing_xy.length > 0.0001:
            plane_normal = outgoing_xy.normalized()
        else:
            plane_normal = Vector((1, 0, 0))
    else:
        if outgoing_direction.length > 0.0001:
            plane_normal = outgoing_direction.normalized()
        else:
            plane_normal = Vector((1, 0, 0))
    
    projection_point = endpoint_co
    
    original_connectivity = get_original_connectivity(bm_data, original_profile_indices)
    
    new_vertex_indices = []
    original_to_new_mapping = {}
    
    for orig_idx in original_profile_indices:
        orig_vertex = bm_data.verts[orig_idx]
        orig_co = orig_vertex.co.copy()
        
        if use_rake_mode:
            projection_direction = outgoing_direction.copy()
            projection_direction.z = 0
            if projection_direction.length < 0.0001:
                projection_direction = Vector((1, 0, 0))
            projection_direction = projection_direction.normalized()
        else:
            projection_direction = outgoing_direction.normalized()
        
        projected_co = intersect_line_plane(
            orig_co,
            orig_co + projection_direction,
            projection_point,
            plane_normal
        )
        
        if projected_co is None:
            projected_co = endpoint_co.copy()
        
        new_vert = bm_data.verts.new(projected_co)
        bm_data.verts.index_update()
        if hasattr(bm_data.verts, "ensure_lookup_table"):
            bm_data.verts.ensure_lookup_table()
        
        new_vertex_indices.append(new_vert.index)
        original_to_new_mapping[orig_idx] = new_vert.index
    
    for edge_pair in original_connectivity['edges']:
        orig_v1, orig_v2 = edge_pair
        if orig_v1 in original_to_new_mapping and orig_v2 in original_to_new_mapping:
            new_v1_idx = original_to_new_mapping[orig_v1]
            new_v2_idx = original_to_new_mapping[orig_v2]
            try:
                new_v1 = bm_data.verts[new_v1_idx]
                new_v2 = bm_data.verts[new_v2_idx]
                new_edge = bm_data.edges.new([new_v1, new_v2])
            except Exception:
                pass
    
    for face_verts in original_connectivity['faces']:
        try:
            new_face_verts = []
            valid_face = True
            for orig_vert_idx in face_verts:
                if orig_vert_idx in original_to_new_mapping:
                    new_vert_idx = original_to_new_mapping[orig_vert_idx]
                    new_face_verts.append(bm_data.verts[new_vert_idx])
                else:
                    valid_face = False
                    break
            
            if valid_face and len(new_face_verts) >= 3:
                new_face = bm_data.faces.new(new_face_verts)
        except Exception:
            pass
    
    bm_data.edges.index_update()
    bm_data.faces.index_update()
    if hasattr(bm_data.edges, "ensure_lookup_table"):
        bm_data.edges.ensure_lookup_table()
    if hasattr(bm_data.faces, "ensure_lookup_table"):
        bm_data.faces.ensure_lookup_table()
    
    return new_vertex_indices

def create_corner_connectivity_if_needed(bm_data, shifted_profile, original_profile_indices, extrusion_type):
    if extrusion_type not in ["profiles"]:
        return

    if shifted_profile != original_profile_indices:
        original_connectivity = get_original_connectivity(bm_data, original_profile_indices)
        original_to_shifted = {}
        for i, orig_idx in enumerate(original_profile_indices):
            if i < len(shifted_profile):
                original_to_shifted[orig_idx] = shifted_profile[i]
        start_point_vertex_index = eap_buf.list_sp[0] if eap_buf.list_sp else None
        for orig_idx in original_profile_indices:
            if orig_idx < len(bm_data.verts):
                orig_vert = bm_data.verts[orig_idx]
                if start_point_vertex_index is not None and orig_idx == start_point_vertex_index:
                    orig_vert.tag = True
        for edge_pair in original_connectivity['edges']:
            orig_v1, orig_v2 = edge_pair
            if orig_v1 in original_to_shifted and orig_v2 in original_to_shifted:
                new_v1_idx = original_to_shifted[orig_v1]
                new_v2_idx = original_to_shifted[orig_v2]
                try:
                    new_v1 = bm_data.verts[new_v1_idx]
                    new_v2 = bm_data.verts[new_v2_idx]
                    edge_exists = False
                    for existing_edge in bm_data.edges:
                        if ((existing_edge.verts[0] == new_v1 and existing_edge.verts[1] == new_v2) or
                            (existing_edge.verts[0] == new_v2 and existing_edge.verts[1] == new_v1)):
                            edge_exists = True
                            existing_edge.tag = True
                            break
                    if not edge_exists:
                        new_edge = bm_data.edges.new([new_v1, new_v2])
                        new_edge.tag = True
                except Exception:
                    pass
        for edge_pair in original_connectivity['edges']:
            orig_v1, orig_v2 = edge_pair
            for edge in bm_data.edges:
                if edge.is_valid:
                    edge_verts = [v.index for v in edge.verts]
                    if ((edge_verts[0] == orig_v1 and edge_verts[1] == orig_v2) or
                        (edge_verts[0] == orig_v2 and edge_verts[1] == orig_v1)):
                        if (start_point_vertex_index is not None and 
                            (orig_v1 == start_point_vertex_index or orig_v2 == start_point_vertex_index)):
                            edge.tag = True
                        break
        for face_verts in original_connectivity['faces']:
            new_face_verts = []
            valid_face = True
            for orig_vert_idx in face_verts:
                if orig_vert_idx in original_to_shifted:
                    new_vert_idx = original_to_shifted[orig_vert_idx]
                    new_face_verts.append(bm_data.verts[new_vert_idx])
                else:
                    valid_face = False
                    break
            if valid_face and len(new_face_verts) >= 3:
                face_exists = False
                for existing_face in bm_data.faces:
                    if set(existing_face.verts) == set(new_face_verts):
                        face_exists = True
                        existing_face.tag = True
                        break
                if not face_exists:
                    try:
                        new_face = bm_data.faces.new(new_face_verts)
                        new_face.tag = True
                    except Exception:
                        pass
        for face_verts in original_connectivity['faces']:
            for face in bm_data.faces:
                if face.is_valid:
                    face_vert_indices = [v.index for v in face.verts]
                    if set(face_vert_indices) == set(face_verts):
                        if (start_point_vertex_index is not None and 
                            start_point_vertex_index in face_verts):
                            face.tag = True
                        break
        bm_data.edges.index_update()
        bm_data.faces.index_update()
        if hasattr(bm_data.edges, "ensure_lookup_table"):
            bm_data.edges.ensure_lookup_table()
        if hasattr(bm_data.faces, "ensure_lookup_table"):
            bm_data.faces.ensure_lookup_table()

def extrude_vertices_only(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False, use_fan_mode=False):
    length_of_edge_sequence = len(edge_sequence_list)
    list_of_selected_profile_indices = shifted_profile
    count_of_profile_vertices = len(list_of_selected_profile_indices)
    copy_of_selected_profile_indices = list(list_of_selected_profile_indices)
    
    dict_keys = original_profile_indices if original_profile_indices is not None else shifted_profile
    dict_of_new_vertex_position = {dict_keys[i]: [shifted_profile[i]] for i in range(len(shifted_profile))}

    for current_edge_index in range(length_of_edge_sequence):
        current_point = bm_data.verts[edge_sequence_list[current_edge_index]].co.copy()
        previous_point = bm_data.verts[edge_sequence_list[(current_edge_index - 1) % length_of_edge_sequence]].co.copy()
        next_point = bm_data.verts[edge_sequence_list[(current_edge_index + 1) % length_of_edge_sequence]].co.copy()
        
        previous_edge = current_point - previous_point
        
        projection_point, projection_normal = calculate_projection_parameters(
            current_point, previous_point, next_point, use_rake_mode=use_rake_mode
        )
        
        if path_is_loop:
            if current_edge_index != 0:
                for j in range(count_of_profile_vertices):
                    v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                    new_co = intersect_line_plane(
                        v,
                        v + (previous_edge.normalized() * 0.1),
                        projection_point,
                        projection_normal
                    )
                    new_vert = bm_data.verts.new(new_co)
                    new_vert.tag = True
                    bm_data.verts.index_update()
                    if hasattr(bm_data.verts, "ensure_lookup_table"):
                        bm_data.verts.ensure_lookup_table()
                    copy_of_selected_profile_indices[j] = new_vert.index
                    dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
        else:
            if current_edge_index == (length_of_edge_sequence - 1):
                if use_fan_mode:
                    if length_of_edge_sequence >= 3:
                        p_final = current_point
                        p_second_last = previous_point
                        p_third_last = bm_data.verts[edge_sequence_list[current_edge_index - 2]].co.copy()
                        vec_penultimate = p_second_last - p_third_last
                        vec_final = p_final - p_second_last
                        if vec_penultimate.length > 0.0001 and vec_final.length > 0.0001:
                            rotation_diff = vec_penultimate.rotation_difference(vec_final)
                            ghost_vec = rotation_diff @ vec_final
                            ghost_point = p_final + ghost_vec
                            projection_point, projection_normal = calculate_projection_parameters(
                                p_final, p_second_last, ghost_point, use_rake_mode=use_rake_mode
                            )
                        else:
                            projection_normal = previous_edge.normalized()
                            projection_point = current_point - (projection_normal * 0.1)
                    else:
                        junction_idx = edge_sequence_list[0]
                        endpoint_idx = edge_sequence_list[-1]
                        
                        other_neighbor_idx = None
                        for stored_edge in eap_buf.list_ek:
                            if junction_idx in stored_edge:
                                other_idx = stored_edge[0] if stored_edge[1] == junction_idx else stored_edge[1]
                                if other_idx != endpoint_idx:
                                    other_neighbor_idx = other_idx
                                    break
                        
                        if other_neighbor_idx is not None:
                            p_final = bm_data.verts[endpoint_idx].co
                            p_second_last = bm_data.verts[junction_idx].co
                            p_third_last = bm_data.verts[other_neighbor_idx].co
                            
                            vec_penultimate = p_second_last - p_third_last
                            vec_final = p_final - p_second_last
                            
                            if vec_penultimate.length > 0.0001 and vec_final.length > 0.0001:
                                rotation_diff = vec_penultimate.rotation_difference(vec_final)
                                ghost_vec = rotation_diff @ vec_final
                                ghost_point = p_final + ghost_vec
                                
                                _, projection_normal = calculate_projection_parameters(
                                    p_final, p_second_last, ghost_point, use_rake_mode=use_rake_mode
                                )
                                projection_point = p_final
                            else:
                                projection_normal = (p_final - p_second_last).normalized()
                                projection_point = p_final
                        else:
                            norm = calculate_edge_normal(bm_data, list_fl[1], current_point, previous_point)
                            projection_point = current_point - (norm.normalized() * 0.1)
                            projection_normal = previous_edge.normalized()
                else:
                    norm = calculate_edge_normal(bm_data, list_fl[1], current_point, previous_point)
                    projection_point = current_point - (norm.normalized() * 0.1)
                    projection_normal = previous_edge.normalized()
                    
                for j in range(count_of_profile_vertices):
                    v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                    new_co = intersect_line_plane(
                        v,
                        v + (previous_edge.normalized() * 0.1),
                        projection_point,
                        projection_normal
                    )
                    new_vert = bm_data.verts.new(new_co)
                    new_vert.tag = True
                    bm_data.verts.index_update()
                    if hasattr(bm_data.verts, "ensure_lookup_table"):
                        bm_data.verts.ensure_lookup_table()
                    copy_of_selected_profile_indices[j] = new_vert.index
                    dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
            else:
                if current_edge_index != 0:
                    for j in range(count_of_profile_vertices):
                        v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                        new_co = intersect_line_plane(
                            v,
                            v + (previous_edge.normalized() * 0.1),
                            projection_point,
                            projection_normal
                        )
                        new_vert = bm_data.verts.new(new_co)
                        new_vert.tag = True
                        bm_data.verts.index_update()
                        if hasattr(bm_data.verts, "ensure_lookup_table"):
                            bm_data.verts.ensure_lookup_table()
                        copy_of_selected_profile_indices[j] = new_vert.index
                        dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
    
    return dict_of_new_vertex_position, list_of_selected_profile_indices, length_of_edge_sequence, (original_profile_indices or shifted_profile)


    



def extrude_vertices_only_legacy(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False):
    length_of_edge_sequence = len(edge_sequence_list)
    list_of_selected_profile_indices = shifted_profile
    count_of_profile_vertices = len(list_of_selected_profile_indices)
    copy_of_selected_profile_indices = list(list_of_selected_profile_indices)
    
    dict_keys = original_profile_indices if original_profile_indices else shifted_profile
    min_length = min(len(shifted_profile), len(dict_keys))
    dict_of_new_vertex_position = {dict_keys[i]: [shifted_profile[i]] for i in range(min_length)}
    
    for current_edge_index in range(length_of_edge_sequence):
        current_point = bm_data.verts[edge_sequence_list[current_edge_index]].co.copy()
        previous_point = bm_data.verts[edge_sequence_list[(current_edge_index - 1) % length_of_edge_sequence]].co.copy()
        next_point = bm_data.verts[edge_sequence_list[(current_edge_index + 1) % length_of_edge_sequence]].co.copy()
        
        previous_edge = current_point - previous_point
        
        projection_point, projection_normal = calculate_projection_parameters(
            current_point, previous_point, next_point, use_rake_mode=use_rake_mode
        )
        
        if path_is_loop:
            if current_edge_index != 0:
                for j in range(count_of_profile_vertices):
                    v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                    new_co = intersect_line_plane(
                        v,
                        v + (previous_edge.normalized() * 0.1),
                        projection_point,
                        projection_normal
                    )
                    new_vert = bm_data.verts.new(new_co)
                    new_vert.tag = True
                    bm_data.verts.index_update()
                    if hasattr(bm_data.verts, "ensure_lookup_table"):
                        bm_data.verts.ensure_lookup_table()
                    copy_of_selected_profile_indices[j] = new_vert.index
                    if j < min_length:
                        dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
        else:
            if current_edge_index == (length_of_edge_sequence - 1):
                end_index = list_fl[1] if eap_buf.list_sp[0] == list_fl[0] else list_fl[0]
                norm = calculate_edge_normal(bm_data, end_index, current_point, previous_point)
                projection_point_at_path_end = current_point - (norm.normalized() * 0.1)
                projection_normal_at_path_end = previous_edge.normalized()
                for j in range(count_of_profile_vertices):
                    v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                    new_co = intersect_line_plane(
                        v,
                        v + (previous_edge.normalized() * 0.1),
                        projection_point_at_path_end,
                        projection_normal_at_path_end
                    )
                    new_vert = bm_data.verts.new(new_co)
                    new_vert.tag = True
                    bm_data.verts.index_update()
                    if hasattr(bm_data.verts, "ensure_lookup_table"):
                        bm_data.verts.ensure_lookup_table()
                    copy_of_selected_profile_indices[j] = new_vert.index
                    if j < min_length:
                        dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
            elif current_edge_index != 0:
                for j in range(count_of_profile_vertices):
                    v = bm_data.verts[copy_of_selected_profile_indices[j]].co.copy()
                    new_co = intersect_line_plane(
                        v,
                        v + (previous_edge.normalized() * 0.1),
                        projection_point,
                        projection_normal
                    )
                    new_vert = bm_data.verts.new(new_co)
                    new_vert.tag = True
                    bm_data.verts.index_update()
                    if hasattr(bm_data.verts, "ensure_lookup_table"):
                        bm_data.verts.ensure_lookup_table()
                    copy_of_selected_profile_indices[j] = new_vert.index
                    if j < min_length:
                        dict_of_new_vertex_position[dict_keys[j]].append(new_vert.index)
    
    return dict_of_new_vertex_position, list_of_selected_profile_indices, length_of_edge_sequence, (original_profile_indices or shifted_profile)

def fill_faces(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False, use_fan_mode=False):
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode, use_fan_mode=use_fan_mode
    )
    
    list_of_selected_edges = [
        [v.index for v in e.verts] for e in bm_data.edges if e.select and e.is_valid
    ]
    
    if not list_of_selected_edges and original_profile_indices is not None:
        for i in range(len(original_profile_indices)):
            for j in range(i+1, len(original_profile_indices)):
                for e in bm_data.edges:
                    if e.is_valid and ((e.verts[0].index == original_profile_indices[i] and e.verts[1].index == original_profile_indices[j]) or
                        (e.verts[1].index == original_profile_indices[i] and e.verts[0].index == original_profile_indices[j])):
                        list_of_selected_edges.append([original_profile_indices[i], original_profile_indices[j]])
                        break
    
    if not list_of_selected_edges and profile_indices is not None:
        for i in range(len(profile_indices)):
            for j in range(i+1, len(profile_indices)):
                for e in bm_data.edges:
                    if e.is_valid and ((e.verts[0].index == profile_indices[i] and e.verts[1].index == profile_indices[j]) or
                        (e.verts[1].index == profile_indices[i] and e.verts[0].index == profile_indices[j])):
                        list_of_selected_edges.append([profile_indices[i], profile_indices[j]])
                        break
    
    for t in range(len(list_of_selected_edges)):
        v1_original = list_of_selected_edges[t][0]
        v2_original = list_of_selected_edges[t][1]
        
        if v1_original not in dict_of_new_vertex_position:
            continue
        if v2_original not in dict_of_new_vertex_position:
            continue
        
        for o in range(length_seq if path_is_loop else (length_seq - 1)):
            try:
                v1_sequence = dict_of_new_vertex_position[v1_original]
                v2_sequence = dict_of_new_vertex_position[v2_original]
                face_verts = [
                    bm_data.verts[v1_sequence[o]],
                    bm_data.verts[v2_sequence[o]],
                    bm_data.verts[v2_sequence[(o + 1) % len(v2_sequence)]],
                    bm_data.verts[v1_sequence[(o + 1) % len(v1_sequence)]]
                ]
                new_face = bm_data.faces.new(face_verts)
                new_face.tag = True
            except Exception:
                pass
            
            bm_data.faces.index_update()
            if hasattr(bm_data.faces, "ensure_lookup_table"):
                bm_data.faces.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def fill_faces_legacy(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False):
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only_legacy(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode
    )
    
    list_of_selected_edges = [
        [v.index for v in e.verts] for e in bm_data.edges if e.select and e.is_valid
    ]
    
    if not list_of_selected_edges and original_profile_indices is not None:
        for i in range(len(original_profile_indices)):
            for j in range(i+1, len(original_profile_indices)):
                for e in bm_data.edges:
                    if e.is_valid and ((e.verts[0].index == original_profile_indices[i] and e.verts[1].index == original_profile_indices[j]) or
                        (e.verts[1].index == original_profile_indices[i] and e.verts[0].index == original_profile_indices[j])):
                        list_of_selected_edges.append([original_profile_indices[i], original_profile_indices[j]])
                        break
    
    if not list_of_selected_edges and profile_indices is not None:
        for i in range(len(profile_indices)):
            for j in range(i+1, len(profile_indices)):
                for e in bm_data.edges:
                    if e.is_valid and ((e.verts[0].index == profile_indices[i] and e.verts[1].index == profile_indices[j]) or
                        (e.verts[1].index == profile_indices[i] and e.verts[0].index == profile_indices[j])):
                        list_of_selected_edges.append([profile_indices[i], profile_indices[j]])
                        break
    
    for t in range(len(list_of_selected_edges)):
        v1_original = list_of_selected_edges[t][0]
        v2_original = list_of_selected_edges[t][1]
        
        if v1_original not in dict_of_new_vertex_position:
            continue
        if v2_original not in dict_of_new_vertex_position:
            continue
        
        for o in range(length_seq if path_is_loop else (length_seq - 1)):
            try:
                v1_sequence = dict_of_new_vertex_position[v1_original]
                v2_sequence = dict_of_new_vertex_position[v2_original]
                face_verts = [
                    bm_data.verts[v1_sequence[o]],
                    bm_data.verts[v2_sequence[o]],
                    bm_data.verts[v2_sequence[(o + 1) % len(v2_sequence)]],
                    bm_data.verts[v1_sequence[(o + 1) % len(v1_sequence)]]
                ]
                new_face = bm_data.faces.new(face_verts)
                new_face.tag = True
            except Exception:
                pass
            
            bm_data.faces.index_update()
            if hasattr(bm_data.faces, "ensure_lookup_table"):
                bm_data.faces.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def extrude_edges_only(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False, use_fan_mode=False):
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode, use_fan_mode=use_fan_mode
    )
    create_corner_connectivity_if_needed(bm_data, shifted_profile, original_profile_indices, "edges")
    for original_vert_idx in dict_of_new_vertex_position:
        vert_sequence = dict_of_new_vertex_position[original_vert_idx]
        for i in range(len(vert_sequence) - 1):
            try:
                v1 = bm_data.verts[vert_sequence[i]]
                v2 = bm_data.verts[vert_sequence[i + 1]]
                new_edge = bm_data.edges.new([v1, v2])
                new_edge.tag = True
            except Exception:
                pass
        if path_is_loop and len(vert_sequence) > 2:
            try:
                v1 = bm_data.verts[vert_sequence[-1]]
                v2 = bm_data.verts[vert_sequence[0]]
                new_edge = bm_data.edges.new([v1, v2])
                new_edge.tag = True
            except Exception:
                pass
    
    bm_data.edges.index_update()
    if hasattr(bm_data.edges, "ensure_lookup_table"):
        bm_data.edges.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def extrude_edges_only_legacy(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False):
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only_legacy(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode
    )
    
    for original_vert_idx in dict_of_new_vertex_position:
        vert_sequence = dict_of_new_vertex_position[original_vert_idx]
        for i in range(len(vert_sequence) - 1):
            try:
                v1 = bm_data.verts[vert_sequence[i]]
                v2 = bm_data.verts[vert_sequence[i + 1]]
                new_edge = bm_data.edges.new([v1, v2])
                new_edge.tag = True
            except Exception:
                pass
        if path_is_loop and len(vert_sequence) > 2:
            try:
                v1 = bm_data.verts[vert_sequence[-1]]
                v2 = bm_data.verts[vert_sequence[0]]
                new_edge = bm_data.edges.new([v1, v2])
                new_edge.tag = True
            except Exception:
                pass
    
    bm_data.edges.index_update()
    if hasattr(bm_data.edges, "ensure_lookup_table"):
        bm_data.edges.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def extrude_profile_with_connectivity(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False, use_fan_mode=False):
    profile_indices_to_analyze = original_profile_indices if original_profile_indices else shifted_profile
    connectivity = get_original_connectivity(bm_data, profile_indices_to_analyze)
    
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode, use_fan_mode=use_fan_mode
    )
    create_corner_connectivity_if_needed(bm_data, shifted_profile, original_profile_indices, "profiles")
    steps_to_process = length_seq if path_is_loop else (length_seq - 1)
    
    for step in range(steps_to_process):
        for edge_pair in connectivity['edges']:
            orig_v1, orig_v2 = edge_pair
            if orig_v1 in dict_of_new_vertex_position and orig_v2 in dict_of_new_vertex_position:
                try:
                    v1_sequence = dict_of_new_vertex_position[orig_v1]
                    v2_sequence = dict_of_new_vertex_position[orig_v2]
                    if step + 1 < len(v1_sequence) and step + 1 < len(v2_sequence):
                        v1 = bm_data.verts[v1_sequence[step + 1]]
                        v2 = bm_data.verts[v2_sequence[step + 1]]
                        new_edge = bm_data.edges.new([v1, v2])
                        new_edge.tag = True
                except Exception:
                    pass
        for face_verts in connectivity['faces']:
            try:
                new_face_verts = []
                valid_face = True
                for orig_vert_idx in face_verts:
                    if orig_vert_idx in dict_of_new_vertex_position:
                        vert_sequence = dict_of_new_vertex_position[orig_vert_idx]
                        if step + 1 < len(vert_sequence):
                            new_face_verts.append(bm_data.verts[vert_sequence[step + 1]])
                        else:
                            valid_face = False
                            break
                    else:
                        valid_face = False
                        break
                if valid_face and len(new_face_verts) >= 3:
                    new_face = bm_data.faces.new(new_face_verts)
                    new_face.tag = True
            except Exception:
                pass
    
    bm_data.edges.index_update()
    bm_data.faces.index_update()
    if hasattr(bm_data.edges, "ensure_lookup_table"):
        bm_data.edges.ensure_lookup_table()
    if hasattr(bm_data.faces, "ensure_lookup_table"):
        bm_data.faces.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def extrude_profile_with_connectivity_legacy(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, use_rake_mode=False):
    profile_indices_to_analyze = original_profile_indices if original_profile_indices else shifted_profile
    connectivity = get_original_connectivity(bm_data, profile_indices_to_analyze)
    
    dict_of_new_vertex_position, profile_indices, length_seq, orig_profile = extrude_vertices_only_legacy(
        bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices, use_rake_mode=use_rake_mode
    )
    
    steps_to_process = length_seq if path_is_loop else (length_seq - 1)
    
    for step in range(steps_to_process):
        for edge_pair in connectivity['edges']:
            orig_v1, orig_v2 = edge_pair
            if orig_v1 in dict_of_new_vertex_position and orig_v2 in dict_of_new_vertex_position:
                try:
                    v1_sequence = dict_of_new_vertex_position[orig_v1]
                    v2_sequence = dict_of_new_vertex_position[orig_v2]
                    if step + 1 < len(v1_sequence) and step + 1 < len(v2_sequence):
                        v1 = bm_data.verts[v1_sequence[step + 1]]
                        v2 = bm_data.verts[v2_sequence[step + 1]]
                        new_edge = bm_data.edges.new([v1, v2])
                        new_edge.tag = True
                except Exception:
                    pass
        for face_verts in connectivity['faces']:
            try:
                new_face_verts = []
                valid_face = True
                for orig_vert_idx in face_verts:
                    if orig_vert_idx in dict_of_new_vertex_position:
                        vert_sequence = dict_of_new_vertex_position[orig_vert_idx]
                        if step + 1 < len(vert_sequence):
                            new_face_verts.append(bm_data.verts[vert_sequence[step + 1]])
                        else:
                            valid_face = False
                            break
                    else:
                        valid_face = False
                        break
                if valid_face and len(new_face_verts) >= 3:
                    new_face = bm_data.faces.new(new_face_verts)
                    new_face.tag = True
            except Exception:
                pass
    
    bm_data.edges.index_update()
    bm_data.faces.index_update()
    if hasattr(bm_data.edges, "ensure_lookup_table"):
        bm_data.edges.ensure_lookup_table()
    if hasattr(bm_data.faces, "ensure_lookup_table"):
        bm_data.faces.ensure_lookup_table()
    
    return dict_of_new_vertex_position, profile_indices, length_seq, orig_profile

def extrude_data_dispatcher(bm_data, list_fl, path_is_loop, shifted_profile, edge_sequence_list, original_profile_indices=None, extrusion_type="faces", use_rake=False, use_fan=False, is_advanced_mode=True):
    if is_advanced_mode:
        if extrusion_type == "edges":
            return extrude_edges_only(bm_data, list_fl, path_is_loop, shifted_profile,
                                      edge_sequence_list, original_profile_indices, use_rake_mode=use_rake, use_fan_mode=use_fan)
        elif extrusion_type == "profiles":
            return extrude_profile_with_connectivity(bm_data, list_fl, path_is_loop, shifted_profile,
                                                     edge_sequence_list, original_profile_indices, use_rake_mode=use_rake, use_fan_mode=use_fan)
        elif extrusion_type == "faces":
            return fill_faces(bm_data, list_fl, path_is_loop, shifted_profile,
                              edge_sequence_list, original_profile_indices, use_rake_mode=use_rake, use_fan_mode=use_fan)
    else:
        if extrusion_type == "edges":
            return extrude_edges_only_legacy(bm_data, list_fl, path_is_loop, shifted_profile,
                                            edge_sequence_list, original_profile_indices, use_rake_mode=use_rake)
        elif extrusion_type == "profiles":
            return extrude_profile_with_connectivity_legacy(bm_data, list_fl, path_is_loop, shifted_profile,
                                                           edge_sequence_list, original_profile_indices, use_rake_mode=use_rake)
        elif extrusion_type == "faces":
            return fill_faces_legacy(bm_data, list_fl, path_is_loop, shifted_profile,
                                    edge_sequence_list, original_profile_indices, use_rake_mode=use_rake)
    
    return fill_faces(bm_data, list_fl, path_is_loop, shifted_profile,
                      edge_sequence_list, original_profile_indices, use_rake_mode=use_rake, use_fan_mode=use_fan)

def prepare_L_path_for_corner_profile(bm_data, list_fl_of_current_segment, path_is_loop, use_rake, extrusion_type="faces", force_straight_start_orientation=False):
    original_profile_indices = [v.index for v in bm_data.verts if v.select and v.is_valid]
    
    if not eap_buf.list_ek:
        return original_profile_indices, original_profile_indices
    
    profile_anchor = eap_buf.list_sp[0] if eap_buf.list_sp else None
    
    if profile_anchor is None:
        return original_profile_indices, original_profile_indices
    
    if not (0 <= profile_anchor < len(bm_data.verts)):
        return original_profile_indices, original_profile_indices
    
    anchor_connectivity = getattr(eap_buf, 'profile_anchor_connectivity', 'unknown')
    
    if anchor_connectivity == "endpoint":
        adjacent = get_adjacent_vertices(eap_buf.list_ek)
        anchor_neighbors = adjacent.get(profile_anchor, [])
        
        if not anchor_neighbors:
            return original_profile_indices, original_profile_indices
        
        outgoing_neighbor = anchor_neighbors[0]
        anchor_co = bm_data.verts[profile_anchor].co
        neighbor_co = bm_data.verts[outgoing_neighbor].co
        outgoing_direction = neighbor_co - anchor_co
        
        try:
            projected_profile_indices = create_endpoint_projected_profile(
                bm_data, original_profile_indices, profile_anchor, outgoing_direction, use_rake
            )
            
            if projected_profile_indices:
                return projected_profile_indices, original_profile_indices
            else:
                return original_profile_indices, original_profile_indices
                
        except Exception:
            return original_profile_indices, original_profile_indices
    
    elif anchor_connectivity == "corner_or_junction":
        adjacent = get_adjacent_vertices(eap_buf.list_ek)
        anchor_neighbors = adjacent.get(profile_anchor, [])
        
        if len(anchor_neighbors) > 2:
            return [], []
        
        if len(anchor_neighbors) < 2:
            return original_profile_indices, original_profile_indices
        
        endpoint1 = anchor_neighbors[0]
        endpoint2 = anchor_neighbors[1]
        
        selected_profile_verts_geom = [v for v in bm_data.verts if getattr(v, "select", False)]
        if selected_profile_verts_geom:
            profile_center_co = mathutils.Vector((0, 0, 0))
            for v_geom in selected_profile_verts_geom:
                profile_center_co += v_geom.co
            profile_center_co /= len(selected_profile_verts_geom)
        else:
            profile_center_co = bm_data.verts[profile_anchor].co.copy()
        
        vector_corner_to_profile_center = profile_center_co - bm_data.verts[profile_anchor].co
        current_p_co = bm_data.verts[profile_anchor].co
        prev_p_co = bm_data.verts[endpoint1].co
        next_p_co = bm_data.verts[endpoint2].co
        
        vec_towards_prev = prev_p_co - current_p_co
        vec_towards_next = next_p_co - current_p_co
        
        if vec_towards_prev.length > 0.0001 and vec_towards_next.length > 0.0001:
            bisector_direction = vec_towards_prev.normalized() + vec_towards_next.normalized()
            plane_normal_of_edges_test = vec_towards_prev.cross(vec_towards_next)
            
            if plane_normal_of_edges_test.length < 0.0001:
                if bisector_direction.length > 0.0001:
                     test_projection_normal = bisector_direction.normalized()
                else:
                    test_projection_normal = vec_towards_prev.normalized().cross(mathutils.Vector((0,0,1))).normalized()
                    if test_projection_normal.length < 0.0001:
                        test_projection_normal = mathutils.Vector((1,0,0))
            else:
                if bisector_direction.length > 0.0001:
                    test_projection_normal = bisector_direction.normalized().cross(plane_normal_of_edges_test.normalized())
                else:
                    test_projection_normal = vec_towards_prev.normalized().cross(mathutils.Vector((0,0,1))).normalized()
                    if test_projection_normal.length < 0.0001:
                        test_projection_normal = mathutils.Vector((1,0,0))
            
            if test_projection_normal.length > 0.0001 and test_projection_normal.dot(vector_corner_to_profile_center) < 0:
                endpoint1, endpoint2 = endpoint2, endpoint1
        
        new_list_fl_for_corner_func = [endpoint1, endpoint2]
        corner_specific_edge_chain = [[endpoint1, profile_anchor], [profile_anchor, endpoint2]]
        corner_specific_start_point = endpoint1
        
        original_eap_list_ek = eap_buf.list_ek[:]
        original_eap_list_sp_first_element = eap_buf.list_sp[0]
        
        shifted_profile_verts = []
        try:
            eap_buf.list_ek = corner_specific_edge_chain
            eap_buf.list_sp[0] = corner_specific_start_point
            
            shifted_profile_verts = _generate_corner_profile_on_L_path_unified(
                bm_data, new_list_fl_for_corner_func, False, use_rake_mode=use_rake
            )
        finally:
            eap_buf.list_ek = original_eap_list_ek
            eap_buf.list_sp[0] = original_eap_list_sp_first_element
        
        if shifted_profile_verts:
            return shifted_profile_verts, original_profile_indices
        else:
            return original_profile_indices, original_profile_indices
    
    elif anchor_connectivity == "straight":
        return original_profile_indices, original_profile_indices
    
    else:
        return original_profile_indices, original_profile_indices