import bpy
import bmesh
from mathutils import Vector
from mathutils.geometry import intersect_line_plane, intersect_point_line

from .operator_functions import (
    setup_bmesh_from_context,
    validate_extrusion_data,
    analyze_path_structure,
    handle_profile_placement,
    get_ordered_path_data,
    execute_extrusion_operation,
    handle_separation_mode,
    handle_non_separate_mode,
    handle_path_deletion
)
from .helper_functions import (
    eap_buf,
    get_active_element_and_its_indices,
    edit_mode_out,
    edit_mode_in,
    check_lukap,
    diagnose_closest_path_vertex_to_profile,
    find_path_endpoints,
    determine_extrusion_start_parameters
)

class OT_ExtrudeAlongPath_Store_Path(bpy.types.Operator):
    bl_label = "Store"
    bl_idname = "mesh.eap_store_path"
    bl_options = {'REGISTER'}

    def execute(self, context):
        wm = context.window_manager
        is_advanced_mode = getattr(wm, "eap_is_advanced_mode", False)
        
        if is_advanced_mode:
            edit_mode_out()
            ob_act = context.active_object
            bm_data = bmesh.new()
            bm_data.from_mesh(ob_act.data)
            check_lukap(bm_data)

            # Snapshot the original selection
            initial_selected_faces = [f for f in bm_data.faces if f.select]
            initial_selected_edges = [e for e in bm_data.edges if e.select]
            
            eap_buf.list_sp[:] = []
            eap_buf.list_ek[:] = []
            start_point_set = False

            # --- COMBINED LOGIC ---
            # 1. Did the user select a face? 
            if initial_selected_faces:
                path_face = initial_selected_faces[0]
                eap_buf.list_sp.append(path_face.verts[0].index)
                start_point_set = True
                
                for edge in path_face.edges:
                    eap_buf.list_ek.append([v.index for v in edge.verts])
                    
            # 2. If NO face, it must be a path-only selection.
            elif initial_selected_edges:
                av = get_active_element_and_its_indices(bm_data)
                
                # Find start point from active edge OR active vert
                if av[1] == 'E':
                    active_edge = bm_data.edges[av[0]]
                    if active_edge.select:
                        eap_buf.list_sp.append(active_edge.verts[0].index)
                        start_point_set = True
                elif av[1] == 'V':
                    active_vert = bm_data.verts[av[0]]
                    if active_vert.select:
                        eap_buf.list_sp.append(active_vert.index)
                        start_point_set = True
                
                # The path is all selected edges
                for edge in initial_selected_edges:
                    eap_buf.list_ek.append([v.index for v in edge.verts])

            if start_point_set:
                # Deselect everything that was originally selected
                for f in initial_selected_faces:
                    f.select_set(False)
                for e in initial_selected_edges:
                    e.select_set(False)
                
                bm_data.to_mesh(ob_act.data)
                bm_data.free()
                edit_mode_in()
                return {'FINISHED'}
            else:
                bm_data.free()
                self.report({'WARNING'}, "Couldn't find a profile or start point. Select a face OR a path.")
                edit_mode_in()
                return {'CANCELLED'}
        else:
            # Legacy mode remains unchanged
            edit_mode_out()
            ob_act = context.active_object
            bm_data = bmesh.new()
            bm_data.from_mesh(ob_act.data)
            check_lukap(bm_data)
            eap_buf.list_ek[:] = []
            for e in bm_data.edges:
                if e.select and e.is_valid:
                    eap_buf.list_ek.append([v.index for v in e.verts])
                    e.select_set(False)
            bm_data.to_mesh(ob_act.data)
            edit_mode_in()
            bm_data.free()
            return {'FINISHED'}

class OT_ExtrudeAlongPath_Store_Start_Point(bpy.types.Operator):
    bl_label = "Store"
    bl_idname = "mesh.eap_store_start_point"
    bl_options = {'REGISTER'}

    def execute(self, context):
        edit_mode_out()
        ob_act = context.active_object
        bm_data = bmesh.new()
        bm_data.from_mesh(ob_act.data)
        check_lukap(bm_data)
        eap_buf.list_sp[:] = []
        for v in bm_data.verts:
            if v.select and v.is_valid:
                eap_buf.list_sp.append(v.index)
                v.select_set(False)
        bm_data.to_mesh(ob_act.data)
        edit_mode_in()
        bm_data.free()
        return {'FINISHED'}

class OT_ExtrudeAlongPath_Store_Both(bpy.types.Operator):
    bl_label = "Store"
    bl_idname = "mesh.eap_store_both"
    bl_options = {'REGISTER'}

    def execute(self, context):
        edit_mode_out()
        ob_act = context.active_object
        bm_data = bmesh.new()
        bm_data.from_mesh(ob_act.data)
        check_lukap(bm_data)

        av = get_active_element_and_its_indices(bm_data)
        start_point_stored_successfully = False
        if av[1] == 'V':
            eap_buf.list_sp[:] = []
            v = bm_data.verts[av[0]]
            if v.select and v.is_valid:
                eap_buf.list_sp.append(v.index)
                v.select_set(False)
                start_point_stored_successfully = True

        if start_point_stored_successfully:
            bm_data.to_mesh(ob_act.data)

        bm_data.free()

        if start_point_stored_successfully:
            bpy.ops.mesh.eap_store_path('INVOKE_DEFAULT')
        else:
            self.report({'WARNING'}, "Active element was not a vertex. Path not stored.")
            if context.mode != 'EDIT_MESH':
                edit_mode_in()
            return {'CANCELLED'}

        return {'FINISHED'}

class OT_ExtrudeAlongPath_Extrude(bpy.types.Operator):
    bl_label = "Extrude"
    bl_idname = "mesh.eap_extrude"
    bl_options = {'REGISTER', 'UNDO'}

    # New operator properties for real-time (lower left) parameter adjustment
    rake: bpy.props.BoolProperty(name="Rake", default=False)
    fan: bpy.props.BoolProperty(name="Fan", default=False)
    flip_normals: bpy.props.BoolProperty(name="Flip", description="Flip the normals of the extruded faces", default=False)
    separate: bpy.props.BoolProperty(name="Separate", default=False)
    delete_path: bpy.props.BoolProperty(name="Delete Path", default=False)

    def execute(self, context):
        wm = context.window_manager
        is_advanced_mode = getattr(wm, "eap_is_advanced_mode", False)
        extrusion_type = getattr(wm, "eap_extrusion_type_poc", "faces")

        # Use these operator properties rather than global WM properties
        use_rake = self.rake
        use_fan = self.fan
        flip_normals = self.flip_normals
        separate = self.separate
        delete_path = self.delete_path

        ob_act, bm_data = setup_bmesh_from_context(context)

        valid, error_msg = validate_extrusion_data(bm_data)
        if not valid:
            self.report({'ERROR'}, error_msg)
            if bm_data and bm_data.is_valid:
                bm_data.free()
            edit_mode_in()
            return {'CANCELLED'}

        if is_advanced_mode:
            determine_extrusion_start_parameters(bm_data)

        start_point_vertex_index = eap_buf.list_sp[0] if eap_buf.list_sp else None

        success, result_path_analysis = analyze_path_structure(bm_data)
        if not success:
            self.report({'ERROR'}, result_path_analysis)
            if bm_data and bm_data.is_valid:
                bm_data.free()
            edit_mode_in()
            return {'CANCELLED'}
        _, initial_list_fl, initial_is_loop = result_path_analysis

        if is_advanced_mode:
            effective_list_fl = eap_buf.effective_endpoints_for_ordering if eap_buf.effective_endpoints_for_ordering else initial_list_fl

            success, result_placement = handle_profile_placement(
                bm_data, effective_list_fl, initial_is_loop, use_rake, extrusion_type
            )
            if not success:
                self.report({'ERROR'}, result_placement)
                if bm_data and bm_data.is_valid:
                    bm_data.free()
                edit_mode_in()
                return {'CANCELLED'}
            shifted_profile_indices, base_profile_indices = result_placement

            success, result_ordered_path = get_ordered_path_data(bm_data, effective_list_fl, initial_is_loop, is_advanced_mode)
        else:
            shifted_profile_indices = [v.index for v in bm_data.verts if v.select and v.is_valid]
            base_profile_indices = shifted_profile_indices[:]

            success, result_ordered_path = get_ordered_path_data(bm_data, initial_list_fl, initial_is_loop, is_advanced_mode)

        if not success:
            self.report({'ERROR'}, result_ordered_path)
            if bm_data and bm_data.is_valid:
                bm_data.free()
            edit_mode_in()
            return {'CANCELLED'}
        path_data, loop_data, fl_data = result_ordered_path

        if is_advanced_mode and not path_data:
            error_message = f"Path anchor at vertex {eap_buf.list_sp[0]} has too many branches (>2). Operation cancelled."
            self.report({'ERROR'}, error_message)
            if bm_data and bm_data.is_valid:
                bm_data.free()
            edit_mode_in()
            return {'CANCELLED'}

        success, error_msg_extrusion = execute_extrusion_operation(
            bm_data, path_data, loop_data, fl_data,
            shifted_profile_indices, base_profile_indices,
            extrusion_type, use_rake, use_fan, is_advanced_mode
        )
        if not success:
            self.report({'ERROR'}, error_msg_extrusion)
            if bm_data and bm_data.is_valid:
                bm_data.free()
            edit_mode_in()
            return {'CANCELLED'}

        active_bm_data_ref = bm_data

        if separate:
            success, error_msg_sep = handle_separation_mode(
                active_bm_data_ref, extrusion_type, start_point_vertex_index,
                base_profile_indices, shifted_profile_indices, ob_act, flip_normals
            )
            active_bm_data_ref = None
            if not success:
                self.report({'ERROR'}, error_msg_sep)
                edit_mode_in()
                return {'CANCELLED'}
        else:
            success, error_msg_nonsep = handle_non_separate_mode(
                active_bm_data_ref, ob_act, base_profile_indices, shifted_profile_indices, flip_normals
            )
            active_bm_data_ref = None
            if not success:
                self.report({'ERROR'}, error_msg_nonsep)
                edit_mode_in()
                return {'CANCELLED'}

        current_active_object_for_deletion = ob_act

        if delete_path:
            if context.mode == 'EDIT_MESH':
                bpy.ops.object.mode_set(mode='OBJECT')
            success, error_msg_path = handle_path_deletion(current_active_object_for_deletion)
            if not success:
                self.report({'WARNING'}, error_msg_path)
            if bpy.context.view_layer.objects.active and bpy.context.view_layer.objects.active.type == 'MESH':
                if bpy.context.view_layer.objects.active.mode != 'EDIT':
                    bpy.ops.object.mode_set(mode='EDIT')

        if active_bm_data_ref is not None and active_bm_data_ref.is_valid:
            active_bm_data_ref.free()

        return {'FINISHED'}

class PT_ExtrudeAlongPath(bpy.types.Panel):
    bl_label = "Extrude Along Path"
    bl_idname = "EXTRUDE_PT_AlongPath"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "OSC"

    def draw(self, context):
        layout = self.layout
        wm = context.window_manager
        
        is_advanced = getattr(wm, "eap_is_advanced_mode", False)
        box = layout.box()

        row_toggle = box.row()
        if not is_advanced:
            row_toggle.alert = True
        row_toggle.prop(wm, "eap_is_advanced_mode", text="Advanced" if is_advanced else "Legacy", toggle=True)

        row_path = box.row()
        row_path.label(text="Path: ")
        row_path.operator("mesh.eap_store_path", text="Store")

        if not is_advanced:
            row_start = box.row()
            row_start.label(text="Start Point: ")
            row_start.operator("mesh.eap_store_start_point", text="Store")

            row_both = box.row()
            row_both.label(text="Both: ")
            row_both.operator("mesh.eap_store_both", text="Store")

        row_type = box.row()
        row_type.prop(wm, "eap_extrusion_type_poc", text="Type")

        row_extrude = box.row()
        row_extrude.operator("mesh.eap_extrude")

_classes = [
    OT_ExtrudeAlongPath_Store_Path,
    OT_ExtrudeAlongPath_Store_Start_Point,
    OT_ExtrudeAlongPath_Store_Both,
    OT_ExtrudeAlongPath_Extrude,
    PT_ExtrudeAlongPath,
]

def register():
    for cls in _classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()