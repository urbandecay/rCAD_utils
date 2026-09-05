import bpy
import bmesh
import mathutils
from collections import deque


# --- CORE HELPERS ---

ISLAND_MARKER = "__cool_bool_island_id__"

def find_selected_islands(bm):
    islands_indices = []
    visited = set()
    selected_verts = [v for v in bm.verts if v.select]
    if not selected_verts: return islands_indices
    selected_verts_dict = {v.index: v for v in selected_verts}
    for start_vert in selected_verts:
        if start_vert.index not in visited:
            current_indices = set()
            q = deque([start_vert])
            visited.add(start_vert.index)
            current_indices.add(start_vert.index)
            while q:
                v = q.popleft()
                for edge in v.link_edges:
                    other = edge.other_vert(v)
                    if other.index in selected_verts_dict and other.index not in visited:
                        visited.add(other.index)
                        current_indices.add(other.index)
                        q.append(other)
            if current_indices: islands_indices.append(current_indices)
    return islands_indices


def mark_selected_islands(bm, islands):
    """Attach a temporary stable ID to every selected island vertex."""
    bm.verts.ensure_lookup_table()
    marker = bm.verts.layers.int.get(ISLAND_MARKER)
    if marker is None:
        marker = bm.verts.layers.int.new(ISLAND_MARKER)

    for vertex in bm.verts:
        vertex[marker] = -1
    for island_index, island in enumerate(islands):
        for vertex_index in island:
            bm.verts[vertex_index][marker] = island_index
    return ISLAND_MARKER


def select_island_geometry_by_marker(bm, marker_name, island_index):
    """Select one original island without matching coincident coordinates."""
    marker = bm.verts.layers.int.get(marker_name)
    if marker is None:
        return False

    for v in bm.verts: v.select_set(False)
    for e in bm.edges: e.select_set(False)
    for f in bm.faces: f.select_set(False)
    selected_verts = {
        vertex for vertex in bm.verts
        if vertex[marker] == island_index
    }
    if not selected_verts:
        return False

    for vertex in selected_verts:
        vertex.select_set(True)
    for e in bm.edges:
        if all(vertex in selected_verts for vertex in e.verts):
            e.select_set(True)
    for f in bm.faces:
        if all(vertex in selected_verts for vertex in f.verts):
            f.select_set(True)
    return True


def remove_island_marker(obj, marker_name):
    """Remove the temporary island ID from an object mesh."""
    if obj is None or obj.type != 'MESH':
        return

    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        marker = bm.verts.layers.int.get(marker_name)
        if marker is None:
            return
        bm.verts.layers.int.remove(marker)
        bm.to_mesh(obj.data)
        obj.data.update()
    finally:
        bm.free()


def separate_and_find_new(context, objects_before):
    bpy.ops.mesh.separate(type='SELECTED')
    objects_after = set(context.scene.objects)
    new_objects = list(objects_after - objects_before)
    if len(new_objects) == 1: return new_objects[0]
    return None


def duplicate_mesh_object(source, name):
    """Create an independent object/data copy for a second Boolean result."""
    duplicate = source.copy()
    duplicate.data = source.data.copy()
    duplicate.name = name
    collections = list(source.users_collection)
    if collections:
        for collection in collections:
            collection.objects.link(duplicate)
    else:
        bpy.context.collection.objects.link(duplicate)
    duplicate.matrix_world = source.matrix_world.copy()
    return duplicate


def reverse_mesh_normals(obj):
    """Reverse all face winding without depending on the active mode.

    For an open Boolean operand, face winding is the only indication of which
    side of the surface is treated as the cutter.  Reversing every face keeps
    the relative winding of a multi-face cutter intact while changing that
    side globally.
    """
    mesh = obj.data
    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
        bmesh.ops.reverse_faces(bm, faces=list(bm.faces))
        bm.to_mesh(mesh)
        mesh.update()
    finally:
        bm.free()


def mesh_is_closed(obj):
    """Return whether *obj* is a closed, two-manifold mesh shell.

    Reversing the winding of an open cutter changes the side of its surface
    that Blender classifies as the cutter.  A closed cutter instead describes
    a volume, so its opposite side is the volume *inside* the cutter and must
    be obtained with an intersection.  Treat non-manifold meshes as open so
    their existing winding-based behavior remains available.
    """
    if obj is None or obj.type != 'MESH':
        return False

    mesh = obj.data
    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
        if not bm.faces or not bm.edges:
            return False
        return all(
            len(edge.link_faces) == 2
            for edge in bm.edges
        ) and all(
            bool(vertex.link_faces)
            for vertex in bm.verts
        )
    finally:
        bm.free()


# --- OPERATOR ---

class MESH_OT_CoolBool(bpy.types.Operator):
    bl_idname = "mesh.cool_bool"
    bl_label = "Cool Bool"
    bl_options = {'REGISTER', 'UNDO'}

    operation_mode: bpy.props.EnumProperty(
        items=[
            ('UNION', "Union", ""),
            ('SUBTRACT', "Subtract", ""),
            ('INTERSECT', "Intersect", ""),
            ('SPLIT', "Split", ""),
        ],
        default='UNION',
    )
    keep_cutter: bpy.props.BoolProperty(name="Keep Cutter", default=False)
    intersect_with_cutter: bpy.props.BoolProperty(name="Intersect with Cutter", default=False)
    merge_intersections: bpy.props.BoolProperty(name="Merge Intersections", default=False)
    swap_subtract: bpy.props.BoolProperty(name="Swap Cutter", default=False, description="Targets cut into the cutter instead of cutter cutting into targets")
    invert_cutter: bpy.props.BoolProperty(name="Invert", default=False, description="Keep the opposite side of the cutter")
    merge_subtract_results: bpy.props.BoolProperty(name="Merge Results", default=False)
    limited_dissolve: bpy.props.BoolProperty(name="Limited Dissolve", default=True)
    dissolve_per_iteration: bpy.props.BoolProperty(name="Per Iteration", default=True)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "keep_cutter")
        if self.operation_mode == 'SUBTRACT':
            layout.prop(self, "swap_subtract")
            layout.prop(self, "invert_cutter")
            layout.prop(self, "merge_subtract_results")
        elif self.operation_mode == 'INTERSECT':
            row = layout.row()
            row.prop(self, "intersect_with_cutter")
            if self.intersect_with_cutter:
                row.prop(self, "merge_intersections")
        if self.operation_mode != 'SPLIT':
            row = layout.row()
            row.prop(self, "limited_dissolve")
            if self.limited_dissolve:
                row.prop(self, "dissolve_per_iteration")

    def execute(self, context):
        wm = context.window_manager
        wm.progress_begin(0, 100)

        try:
            solver_mode = context.scene.cool_bool_solver
            keep_cutter_setting = self.keep_cutter

            original_obj = context.active_object
            original_name = original_obj.name

            bm = bmesh.from_edit_mesh(original_obj.data)
            bm.verts.ensure_lookup_table()
            islands = find_selected_islands(bm)
            if len(islands) < 2:
                self.report({'ERROR'}, "Need 2+ islands.")
                wm.progress_end()
                return {'CANCELLED'}

            cutter_index = 0
            active_index = -1
            if bm.select_history.active:
                elem = bm.select_history.active
                if isinstance(elem, bmesh.types.BMVert): active_index = elem.index
                elif isinstance(elem, bmesh.types.BMEdge): active_index = elem.verts[0].index
                elif isinstance(elem, bmesh.types.BMFace): active_index = elem.verts[0].index
            if active_index != -1:
                for i, island in enumerate(islands):
                    if active_index in island: cutter_index = i; break

            island_marker_name = mark_selected_islands(bm, islands)
            bmesh.update_edit_mesh(original_obj.data)
            target_island_indices = [
                i for i in range(len(islands))
                if i != cutter_index
            ]

            bpy.ops.object.mode_set(mode='EDIT')
            bm = bmesh.from_edit_mesh(original_obj.data)
            select_island_geometry_by_marker(
                bm,
                island_marker_name,
                cutter_index,
            )
            bmesh.update_edit_mesh(original_obj.data)
            objs_before = set(context.scene.objects)
            cutter_obj = separate_and_find_new(context, objs_before)
            cutter_obj.name = "CB_Main_Temp"

            target_objects = []
            for i, island_index in enumerate(target_island_indices):
                context.view_layer.objects.active = original_obj
                bpy.ops.object.mode_set(mode='EDIT')
                bm = bmesh.from_edit_mesh(original_obj.data)
                if select_island_geometry_by_marker(
                    bm,
                    island_marker_name,
                    island_index,
                ):
                    bmesh.update_edit_mesh(original_obj.data)
                    objs_before = set(context.scene.objects)
                    t_obj = separate_and_find_new(context, objs_before)
                    if t_obj: t_obj.name = f"CB_Target_{i}"; target_objects.append(t_obj)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            for marker_obj in [original_obj, cutter_obj] + target_objects:
                remove_island_marker(marker_obj, island_marker_name)
            final_objects = []
            objects_to_delete = []

            def apply_bool_and_clean(main, operand, op, force_invert=False):
                if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
                for recalc_obj in (main, operand):
                    context.view_layer.objects.active = recalc_obj
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.normals_make_consistent(inside=False)
                    bpy.ops.object.mode_set(mode='OBJECT')
                invert_subtract = (
                    op == 'DIFFERENCE'
                    and (
                        force_invert
                        or (
                            self.operation_mode == 'SUBTRACT'
                            and self.invert_cutter
                            and (
                                (not self.swap_subtract and operand is cutter_obj)
                                or (self.swap_subtract and main is cutter_obj)
                            )
                        )
                    )
                )
                boolean_operation = op
                if invert_subtract:
                    if mesh_is_closed(operand):
                        # A closed split operand represents a volume.  Its
                        # opposite side is the part shared by the main object
                        # and operand; face reversal would make the closed
                        # operand invalid for Boolean volume classification.
                        boolean_operation = 'INTERSECT'
                    else:
                        # Recalculate first, then invert.  Recalculating an
                        # open mesh can choose a new global winding and would
                        # otherwise cancel the user's choice before the
                        # Boolean modifier sees it.
                        reverse_mesh_normals(operand)
                mod = main.modifiers.new("CB", 'BOOLEAN')
                mod.operation = boolean_operation
                mod.object = operand
                mod.solver = solver_mode
                bpy.ops.object.select_all(action='DESELECT')
                context.view_layer.objects.active = main
                main.select_set(True)
                bpy.ops.object.modifier_apply(modifier=mod.name)
                if self.limited_dissolve and self.dissolve_per_iteration:
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                    bpy.ops.object.mode_set(mode='OBJECT')

            if self.operation_mode == 'UNION':
                main_obj = cutter_obj
                for t_obj in target_objects:
                    apply_bool_and_clean(main_obj, t_obj, 'UNION')
                    objects_to_delete.append(t_obj)
                final_objects.append(main_obj)

            elif self.operation_mode == 'SUBTRACT':
                if self.swap_subtract:
                    # Swapped: each target cuts into the cutter
                    main_obj = cutter_obj
                    for t_obj in target_objects:
                        apply_bool_and_clean(main_obj, t_obj, 'DIFFERENCE')
                        objects_to_delete.append(t_obj)
                    final_objects.append(main_obj)
                else:
                    # Normal: cutter cuts into each target
                    subtract_results = []
                    for t_obj in target_objects:
                        apply_bool_and_clean(t_obj, cutter_obj, 'DIFFERENCE')
                        subtract_results.append(t_obj)
                    if keep_cutter_setting: final_objects.append(cutter_obj)
                    else: objects_to_delete.append(cutter_obj)

                    if self.merge_subtract_results and len(subtract_results) > 1:
                        merged = subtract_results[0]
                        for r_obj in subtract_results[1:]:
                            apply_bool_and_clean(merged, r_obj, 'UNION')
                            objects_to_delete.append(r_obj)
                        final_objects.append(merged)
                    else:
                        final_objects.extend(subtract_results)

            elif self.operation_mode == 'INTERSECT':
                if self.intersect_with_cutter:
                    intersection_results = []
                    for t_obj in target_objects:
                        apply_bool_and_clean(t_obj, cutter_obj, 'INTERSECT')
                        intersection_results.append(t_obj)
                    if keep_cutter_setting:
                        final_objects.append(cutter_obj)
                    else:
                        objects_to_delete.append(cutter_obj)
                    if self.merge_intersections and len(intersection_results) > 1:
                        merged = intersection_results[0]
                        for r_obj in intersection_results[1:]:
                            apply_bool_and_clean(merged, r_obj, 'UNION')
                            objects_to_delete.append(r_obj)
                        final_objects.append(merged)
                    else:
                        final_objects.extend(intersection_results)
                else:
                    all_objects = [cutter_obj] + target_objects
                    result_obj = all_objects[0]
                    for i in range(1, len(all_objects)):
                        next_obj = all_objects[i]
                        apply_bool_and_clean(result_obj, next_obj, 'INTERSECT')
                        objects_to_delete.append(next_obj)
                    final_objects.append(result_obj)

            elif self.operation_mode == 'SPLIT':
                split_results = []
                closed_cutter = mesh_is_closed(cutter_obj)
                for index, t_obj in enumerate(target_objects):
                    opposite_obj = duplicate_mesh_object(t_obj, f"CB_Split_{index}")

                    # Keep the ordinary outside and inside results as two
                    # independent mesh objects.  A closed cutter's opposite
                    # side is an intersection; an open cutter needs the same
                    # winding reversal used by inverted subtraction.
                    apply_bool_and_clean(t_obj, cutter_obj, 'DIFFERENCE')
                    if closed_cutter:
                        apply_bool_and_clean(opposite_obj, cutter_obj, 'INTERSECT')
                    else:
                        split_cutter = duplicate_mesh_object(
                            cutter_obj,
                            f"CB_Split_Cutter_{index}",
                        )
                        apply_bool_and_clean(
                            opposite_obj,
                            split_cutter,
                            'DIFFERENCE',
                            force_invert=True,
                        )
                        objects_to_delete.append(split_cutter)

                    split_results.extend((t_obj, opposite_obj))

                if keep_cutter_setting:
                    final_objects.append(cutter_obj)
                else:
                    objects_to_delete.append(cutter_obj)
                final_objects.extend(split_results)

            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')

            # End-of-all dissolve (when per_iteration is off)
            if self.limited_dissolve and not self.dissolve_per_iteration:
                valid_finals = [o for o in final_objects if o.name in bpy.data.objects]
                for o in valid_finals:
                    context.view_layer.objects.active = o
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.select_all(action='SELECT')
                    bpy.ops.mesh.dissolve_limited(angle_limit=0.0872665)
                    bpy.ops.object.mode_set(mode='OBJECT')

            bpy.ops.object.select_all(action='DESELECT')
            for obj in objects_to_delete:
                try: bpy.data.objects.remove(obj, do_unlink=True)
                except: pass
            valid_finals = [o for o in final_objects if o.name in bpy.data.objects]
            for o in valid_finals: o.select_set(True)
            original_obj.select_set(True)
            context.view_layer.objects.active = original_obj
            bpy.ops.object.join()
            original_obj.name = original_name
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='DESELECT')
            self.report({'INFO'}, "Cool Bool Finished")

        except Exception as e:
            self.report({'ERROR'}, f"Script Failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            wm.progress_end()
        return {'FINISHED'}
