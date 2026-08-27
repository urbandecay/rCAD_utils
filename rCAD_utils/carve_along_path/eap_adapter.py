"""Run Extrude Along Path's normal face branch for Carve Along Path.

Carve uses the established EAP path ordering, anchor detection, profile
placement, corner handling, and face extrusion.  It runs on a temporary copy
of the active mesh and asks EAP's separation path for the resulting cutter;
the caller can then apply Cool Bool's Difference workflow to the original.
"""

import bpy

from .extrude_along_path import extrude as eap_extrude
from .extrude_along_path import helper_functions as eap_helpers
from .extrude_along_path import operator_functions as eap_operators


def _activate_work_buffer(path_edges, path_start):
    work_buffer = eap_helpers.EAPBuffer()
    work_buffer.list_ek = [list(edge) for edge in path_edges]
    work_buffer.list_sp = [path_start]

    modules = (eap_helpers, eap_extrude, eap_operators)
    old_buffers = tuple(module.eap_buf for module in modules)
    for module in modules:
        module.eap_buf = work_buffer
    return work_buffer, modules, old_buffers


def _restore_work_buffer(modules, old_buffers):
    for module, old_buffer in zip(modules, old_buffers):
        module.eap_buf = old_buffer


def _remove_temporary_object(object_mesh):
    if object_mesh is None:
        return
    mesh_data = object_mesh.data if object_mesh.type == 'MESH' else None
    if object_mesh.name in bpy.data.objects:
        bpy.data.objects.remove(object_mesh, do_unlink=True)
    if mesh_data is not None and mesh_data.users == 0 and mesh_data.name in bpy.data.meshes:
        bpy.data.meshes.remove(mesh_data)


def extrude_faces_to_cutter(context, source_object, path_edges, path_start):
    """Return the separated face extrusion object made by EAP."""
    objects_before = set(bpy.data.objects)
    work_buffer, modules, old_buffers = _activate_work_buffer(path_edges, path_start)
    bm_data = None
    result_cutter = None

    try:
        context.view_layer.objects.active = source_object
        source_object.select_set(True)

        ob_act, bm_data = eap_operators.setup_bmesh_from_context(context)
        valid, error_message = eap_operators.validate_extrusion_data(bm_data)
        if not valid:
            return None, error_message

        # This is the normal advanced EAP path: automatic anchor selection,
        # endpoint/corner profile placement, ordered path data, then Faces.
        eap_helpers.determine_extrusion_start_parameters(bm_data)
        start_point_vertex_index = work_buffer.list_sp[0]

        success, path_analysis = eap_operators.analyze_path_structure(bm_data)
        if not success:
            return None, path_analysis
        _, initial_list_fl, initial_is_loop = path_analysis

        effective_list_fl = (
            work_buffer.effective_endpoints_for_ordering
            if work_buffer.effective_endpoints_for_ordering
            else initial_list_fl
        )
        success, profile_placement = eap_operators.handle_profile_placement(
            bm_data,
            effective_list_fl,
            initial_is_loop,
            False,
            "faces",
        )
        if not success:
            return None, profile_placement
        shifted_profile_indices, base_profile_indices = profile_placement

        success, ordered_path = eap_operators.get_ordered_path_data(
            bm_data,
            effective_list_fl,
            initial_is_loop,
            True,
        )
        if not success:
            return None, ordered_path
        path_data, loop_data, fl_data = ordered_path
        if not path_data:
            return None, "The stored path could not be ordered."

        success, error_message = eap_operators.execute_extrusion_operation(
            bm_data,
            path_data,
            loop_data,
            fl_data,
            shifted_profile_indices,
            base_profile_indices,
            "faces",
            False,
            False,
            True,
        )
        if not success:
            return None, error_message

        # Force EAP's cutter isolation path.  It keeps the generated tagged
        # faces and removes the source mesh geometry from this temporary copy.
        success, error_message = eap_operators.handle_separation_mode(
            bm_data,
            "faces",
            start_point_vertex_index,
            base_profile_indices,
            shifted_profile_indices,
            ob_act,
            False,
        )
        bm_data = None
        if not success:
            return None, error_message

        objects_after = set(bpy.data.objects)
        new_objects = [
            obj for obj in objects_after - objects_before
            if obj.type == 'MESH'
        ]
        if len(new_objects) != 1:
            for obj in new_objects:
                _remove_temporary_object(obj)
            return None, "EAP did not produce one cutter object."
        # Keep EAP's surface open.  Cool Bool uses the cutter face winding to
        # choose the retained side, and capping these boundaries changes that
        # behavior (and makes Carve differ from EAP followed by Cool Bool).
        cutter = new_objects[0]
        result_cutter = cutter
        return result_cutter, None
    except Exception as error:
        return None, f"EAP face extrusion failed: {error}"
    finally:
        if bm_data is not None and bm_data.is_valid:
            bm_data.free()
        if result_cutter is None:
            for obj in set(bpy.data.objects) - objects_before:
                _remove_temporary_object(obj)
        _restore_work_buffer(modules, old_buffers)
