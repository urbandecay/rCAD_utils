import bpy
import bmesh
from mathutils import Vector
from mathutils.geometry import intersect_line_plane, intersect_point_line

from .extrude import (
    extrude_data_dispatcher,
    prepare_L_path_for_corner_profile
)
from .helper_functions import (
    edit_mode_out,
    edit_mode_in,
    check_lukap,
    get_active_element_and_its_indices,
    get_adjacent_vertices,
    is_loop_,
    form_edge_chain,
    form_edge_loop,
    calculate_edge_normal,
    eap_buf,
    get_ordered_path,
    get_ordered_path_legacy,
    diagnose_closest_path_vertex_to_profile,
    find_path_endpoints,
    determine_extrusion_start_parameters
)

def setup_bmesh_from_context(context):
    edit_mode_out()
    ob_act = context.active_object

    bm_data = bmesh.new()
    bm_data.from_mesh(ob_act.data)
    check_lukap(bm_data)

    return ob_act, bm_data

def validate_extrusion_data(bm_data):
    if not eap_buf.list_ek:
        return False, "No path edges stored. Please store a path first."
    
    if not eap_buf.list_sp:
        return False, "No start point stored. Please store a start point first."

    selected_verts_objects = [v for v in bm_data.verts if v.select and v.is_valid]
    if not selected_verts_objects:
        return False, "No profile vertices selected. Please select profile vertices first."

    return True, None

def analyze_path_structure(bm_data):
    try:
        initial_edge_chain = get_adjacent_vertices(eap_buf.list_ek)
        initial_list_fl = [i for i in initial_edge_chain if (len(initial_edge_chain[i]) == 1)]
        initial_is_loop = is_loop_(initial_list_fl)
        
        return True, (initial_edge_chain, initial_list_fl, initial_is_loop)
    except Exception as e:
        return False, f"Error analyzing path: {e}"

def handle_profile_placement(bm_data, initial_list_fl, initial_is_loop, use_rake, extrusion_type):
    try:
        force_straight = getattr(eap_buf, 'force_straight_start_orientation', False)

        shifted_profile, original_profile_indices = prepare_L_path_for_corner_profile(
            bm_data, initial_list_fl, initial_is_loop, use_rake, extrusion_type, 
            force_straight_start_orientation=force_straight
        )

        if not shifted_profile and not original_profile_indices and [v for v in bm_data.verts if v.select and v.is_valid]:
            profile_anchor = eap_buf.list_sp[0] if eap_buf.list_sp else 'N/A'
            error_msg = f"Path anchor (vert {profile_anchor}) has >2 branches. Invalid path."
            return False, error_msg

        return True, (shifted_profile, original_profile_indices)
    except Exception as e:
        return False, f"Error in profile placement: {e}"

def get_ordered_path_data(bm_data, initial_list_fl, initial_is_loop, is_advanced_mode=True):
    try:
        if is_advanced_mode:
            path_data, loop_data, fl_data = get_ordered_path(
                bm_data, 
                eap_buf.list_ek, 
                eap_buf.list_sp[0],
                initial_list_fl,
                initial_is_loop
            )
        else:
            path_data, loop_data, fl_data = get_ordered_path_legacy(
                bm_data, 
                eap_buf.list_ek, 
                eap_buf.list_sp[0],
                initial_list_fl,
                initial_is_loop
            )
        return True, (path_data, loop_data, fl_data)
    except Exception as e:
        return False, f"Error getting ordered path: {e}. Please check your path and start point."

def execute_extrusion_operation(bm_data, path_data, loop_data, fl_data, shifted_profile, original_profile_indices, extrusion_type, use_rake, use_fan, is_advanced_mode):
    try:
        is_multi_path = False
        if path_data and isinstance(path_data[0], list):
            is_multi_path = True

        if is_multi_path:
            for i in range(len(path_data)):
                current_path_sequence = path_data[i]
                current_loop_status = loop_data 
                current_fl_for_path = fl_data[i] if fl_data and i < len(fl_data) else []

                dict_new, profile_indices, length_seq, orig_profile = extrude_data_dispatcher(
                    bm_data, current_fl_for_path, current_loop_status, shifted_profile,
                    current_path_sequence, original_profile_indices, extrusion_type, use_rake, use_fan, is_advanced_mode
                )

        else:
            current_path_sequence = path_data
            current_loop_status = loop_data
            current_fl_for_path = fl_data

            dict_new, profile_indices, length_seq, orig_profile = extrude_data_dispatcher(
                bm_data, current_fl_for_path, current_loop_status, shifted_profile,
                current_path_sequence, original_profile_indices, extrusion_type, use_rake, use_fan, is_advanced_mode
            )

        return True, None

    except Exception as e:
        return False, f"Error during extrusion: Turn off Rake mode"

      
      
def handle_separation_mode(bm_data, extrusion_type, start_point_vertex_index, original_profile_indices, shifted_profile, ob_act, flip_normals=False):
    try:
        # THIS IS THE LINE I DELETED BY MISTAKE. IT IS NOW RESTORED.
        vertices_to_keep = set()

        if extrusion_type == "edges":
            tagged_edges = [e for e in bm_data.edges if e.is_valid and e.tag]
            tagged_faces = [f for f in bm_data.faces if f.is_valid and f.tag]

            for edge in tagged_edges:
                if edge.is_valid:
                    vertices_to_keep.update([v for v in edge.verts if v.is_valid])
            for face in tagged_faces:
                if face.is_valid:
                    vertices_to_keep.update([v for v in face.verts if v.is_valid])

            if original_profile_indices:
                if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()
                for orig_idx in original_profile_indices:
                    if 0 <= orig_idx < len(bm_data.verts):
                        original_vert = bm_data.verts[orig_idx]
                        if original_vert.is_valid:
                            vertices_to_keep.add(original_vert)

            untagged_faces = [f for f in bm_data.faces if f.is_valid and not f.tag]
            if untagged_faces:
                bmesh.ops.delete(bm_data, geom=untagged_faces, context='FACES')

            bm_data.edges.index_update(); bm_data.verts.index_update()
            if hasattr(bm_data.edges, "ensure_lookup_table"): bm_data.edges.ensure_lookup_table()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            untagged_edges = [e for e in bm_data.edges if e.is_valid and not e.tag]
            if untagged_edges:
                bmesh.ops.delete(bm_data, geom=untagged_edges, context='EDGES')

            bm_data.verts.index_update()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            verts_to_delete = [v for v in bm_data.verts if v.is_valid and v not in vertices_to_keep]
            if verts_to_delete:
                bmesh.ops.delete(bm_data, geom=verts_to_delete, context='VERTS')

        elif extrusion_type == "profiles":
            tagged_edges = [e for e in bm_data.edges if e.is_valid and e.tag]
            tagged_faces = [f for f in bm_data.faces if f.is_valid and f.tag]

            for edge in tagged_edges:
                if edge.is_valid:
                    vertices_to_keep.update([v for v in edge.verts if v.is_valid])
            for face in tagged_faces:
                if face.is_valid:
                    vertices_to_keep.update([v for v in face.verts if v.is_valid])

            if original_profile_indices:
                if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()
                for orig_idx in original_profile_indices:
                    if 0 <= orig_idx < len(bm_data.verts):
                        original_vert = bm_data.verts[orig_idx]
                        if original_vert.is_valid:
                            vertices_to_keep.add(original_vert)

            untagged_faces = [f for f in bm_data.faces if f.is_valid and not f.tag]
            if untagged_faces:
                bmesh.ops.delete(bm_data, geom=untagged_faces, context='FACES')

            bm_data.edges.index_update(); bm_data.verts.index_update()
            if hasattr(bm_data.edges, "ensure_lookup_table"): bm_data.edges.ensure_lookup_table()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            untagged_edges = [e for e in bm_data.edges if e.is_valid and not e.tag]
            if untagged_edges:
                bmesh.ops.delete(bm_data, geom=untagged_edges, context='EDGES')

            bm_data.verts.index_update()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            verts_to_delete = [v for v in bm_data.verts if v.is_valid and v not in vertices_to_keep]
            if verts_to_delete:
                bmesh.ops.delete(bm_data, geom=verts_to_delete, context='VERTS')

        elif extrusion_type == "faces":
            tagged_faces = [f for f in bm_data.faces if f.is_valid and f.tag]

            for face_bm in tagged_faces:
                vertices_to_keep.update(v for v in face_bm.verts if v.is_valid)

            for v_bm in bm_data.verts:
                if v_bm.is_valid and v_bm.tag:
                    vertices_to_keep.add(v_bm)

            if original_profile_indices:
                if hasattr(bm_data.verts, "ensure_lookup_table"):
                    bm_data.verts.ensure_lookup_table()
                for orig_idx in original_profile_indices:
                    if 0 <= orig_idx < len(bm_data.verts):
                        original_vert = bm_data.verts[orig_idx]
                        if original_vert.is_valid:
                            vertices_to_keep.add(original_vert)

            untagged_faces_list = [f for f in bm_data.faces if f.is_valid and not f.tag]
            if untagged_faces_list:
                bmesh.ops.delete(bm_data, geom=untagged_faces_list, context='FACES')

            bm_data.edges.index_update(); bm_data.verts.index_update()
            if hasattr(bm_data.edges, "ensure_lookup_table"): bm_data.edges.ensure_lookup_table()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            edges_to_delete = []
            for e in bm_data.edges:
                if e.is_valid and not e.tag:
                    is_supporting_kept_face = False
                    for f_linked in e.link_faces:
                        if f_linked.is_valid and f_linked.tag:
                            is_supporting_kept_face = True
                            break
                    if not is_supporting_kept_face:
                        edges_to_delete.append(e)

            if edges_to_delete:
                valid_edges_to_delete = [edge_del for edge_del in edges_to_delete if edge_del.is_valid]
                if valid_edges_to_delete:
                    bmesh.ops.delete(bm_data, geom=valid_edges_to_delete, context='EDGES')

            bm_data.verts.index_update()
            if hasattr(bm_data.verts, "ensure_lookup_table"): bm_data.verts.ensure_lookup_table()

            verts_to_delete_final = [v_bm_final for v_bm_final in bm_data.verts if v_bm_final.is_valid and v_bm_final not in vertices_to_keep]
            if verts_to_delete_final:
                valid_verts_to_delete_final = [v_bm_d for v_bm_d in verts_to_delete_final if v_bm_d.is_valid]
                if valid_verts_to_delete_final:
                    bmesh.ops.delete(bm_data, geom=valid_verts_to_delete_final, context='VERTS')
        
        tagged_faces = [f for f in bm_data.faces if f.tag]
        if tagged_faces:
            bmesh.ops.recalc_face_normals(bm_data, faces=tagged_faces)
            if flip_normals:
                bmesh.ops.reverse_faces(bm_data, faces=tagged_faces)

        extrusion_suffix = f"_{extrusion_type}"
        new_mesh_name_base = ob_act.name if "." not in ob_act.name else ob_act.name.split(".")[0]
        new_mesh_name = new_mesh_name_base + extrusion_suffix
        count = 1
        while new_mesh_name in bpy.data.objects:
            new_mesh_name = f"{new_mesh_name_base}{extrusion_suffix}.{str(count).zfill(3)}"
            count += 1
        
        new_mesh_data = bpy.data.meshes.new(name=new_mesh_name + "_mesh")
        bm_data.to_mesh(new_mesh_data)
        bm_data.free()

        new_obj = bpy.data.objects.new(new_mesh_name, new_mesh_data)
        bpy.context.collection.objects.link(new_obj)

        new_obj.matrix_world = ob_act.matrix_world

        if bpy.ops.object.select_all.poll():
            bpy.ops.object.select_all(action='DESELECT')
        new_obj.select_set(False)
        ob_act.select_set(True)
        bpy.context.view_layer.objects.active = ob_act

        edit_mode_in()
        return True, None

    except Exception as e:
        if 'bm_data' in locals() and bm_data is not None and hasattr(bm_data, 'free') and bm_data.is_valid:
            try:
                bm_data.free()
            except Exception:
                pass
        return False, f"Error during separation: {e}"

    






      
def handle_non_separate_mode(bm_data, ob_act, original_profile_indices, shifted_profile, flip_normals=False):
    try:
        for v in bm_data.verts:
            v.select = False

        if original_profile_indices:
            if hasattr(bm_data.verts, "ensure_lookup_table"):
                bm_data.verts.ensure_lookup_table()

            for idx in original_profile_indices:
                if 0 <= idx < len(bm_data.verts):
                    vert_to_select = bm_data.verts[idx]
                    if vert_to_select.is_valid:
                        vert_to_select.select = True

        tagged_faces = [f for f in bm_data.faces if f.tag]
        if tagged_faces:
            bmesh.ops.recalc_face_normals(bm_data, faces=tagged_faces)
            if flip_normals:
                bmesh.ops.reverse_faces(bm_data, faces=tagged_faces)

        bm_data.to_mesh(ob_act.data)
        edit_mode_in()
        bm_data.free()

        return True, None

    except Exception as e:
        if bm_data and bm_data.is_valid:
            try:
                bm_data.free()
            except Exception:
                pass
        return False, f"Error in non-separate mode: {e}"

        

    



def handle_path_deletion(ob_act):
    try:
        edit_mode_out()
        bm_del = bmesh.new()
        bm_del.from_mesh(ob_act.data)
        check_lukap(bm_del)
        path_edges_to_delete = []
        for edge in bm_del.edges:
            if edge.is_valid:
                edge_indices = sorted([v.index for v in edge.verts])
                for stored_edge in eap_buf.list_ek:
                    if sorted(stored_edge) == edge_indices:
                        path_edges_to_delete.append(edge)
                        break
        if path_edges_to_delete:
            bmesh.ops.delete(bm_del, geom=path_edges_to_delete, context='EDGES')
        isolated_verts = [v for v in bm_del.verts if v.is_valid and len(v.link_edges) == 0]
        if isolated_verts:
            bmesh.ops.delete(bm_del, geom=isolated_verts, context='VERTS')
        bm_del.to_mesh(ob_act.data)
        bm_del.free()
        edit_mode_in()
        return True, None
    except Exception as e:
        return False, f"Path deletion failed: {e}"