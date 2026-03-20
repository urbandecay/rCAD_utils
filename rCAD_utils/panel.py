import bpy


class RCAD_PT_Main(bpy.types.Panel):
    bl_label = "rCAD Utils"
    bl_idname = "RCAD_PT_Main"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"

    def draw(self, context):
        pass


class RCAD_PT_ExtrudeAlongPath(bpy.types.Panel):
    bl_label = "Extrude Along Path"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

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


class RCAD_PT_PlaceProfile(bpy.types.Panel):
    bl_label = "Place Profile"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        layout.operator_context = 'EXEC_DEFAULT'
        scene = context.scene
        box = layout.box()

        row = box.row(align=True)
        row.label(text="Profile:")
        row.operator("object.store_profile_info_edit", text="Store")

        box.prop(scene, "profile_path_mode", text="Custom Normal")

        row = box.row(align=True)
        row.operator("object.place_profile_on_edges_edit", text="Place")


class RCAD_PT_CoolBool(bpy.types.Panel):
    bl_label = "Cool Bool"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        box = layout.box()

        box.prop(scene, "cool_bool_solver", text="")

        col = box.column(align=True)
        row = col.row(align=True)
        row.operator("mesh.cool_bool", text="Union").operation_mode = 'UNION'
        row.operator("mesh.cool_bool", text="Subtract").operation_mode = 'SUBTRACT'
        row.operator("mesh.cool_bool", text="Intersect").operation_mode = 'INTERSECT'
        col.separator()


class RCAD_PT_MeshTiler(bpy.types.Panel):
    bl_label = "Mesh Tiler"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.operator("mesh.mesh_tiler", text="Tile")


class RCAD_PT_MirrorAlongPlane(bpy.types.Panel):
    bl_label = "Mirror Across Plane"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()

        row = box.row(align=True)
        row.label(text="Plane:")
        row.operator("mesh.store_plane_vertices")

        row = box.row(align=True)
        row.operator("mesh.reflect_across_plane")
