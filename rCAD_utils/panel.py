import importlib
import sys
import traceback

import bpy


_reload_pending = False


def _reload_rCAD_utils_timer():
    """Reload the complete package after the reload button returns."""
    global _reload_pending
    _reload_pending = False

    package_name = __package__.split('.')[0]
    old_package = sys.modules.get(package_name)
    if old_package is None:
        print(f"rCAD Utils reload failed: {package_name!r} is not loaded")
        return None

    addon_enabled = getattr(old_package, "__addon_enabled__", False)
    addon_persistent = getattr(old_package, "__addon_persistent__", False)

    try:
        old_package.unregister()
        package_modules = [
            name for name in tuple(sys.modules)
            if name == package_name or name.startswith(package_name + ".")
        ]
        for name in sorted(
            package_modules,
            key=lambda item: (item.count('.'), item),
            reverse=True,
        ):
            sys.modules.pop(name, None)

        importlib.invalidate_caches()
        new_package = importlib.import_module(package_name)
        new_package.register()
        new_package.__addon_enabled__ = addon_enabled
        new_package.__addon_persistent__ = addon_persistent
        print("rCAD Utils reloaded")
    except Exception:
        traceback.print_exc()

    return None


class RCAD_OT_ReloadAddon(bpy.types.Operator):
    bl_idname = "wm.rcad_reload_addon"
    bl_label = "Reload rCAD Utils"
    bl_description = "Reload rCAD Utils without restarting Blender or saving the file"
    bl_options = {'REGISTER'}

    def execute(self, context):
        global _reload_pending
        if _reload_pending:
            self.report({'WARNING'}, "rCAD Utils reload is already pending.")
            return {'CANCELLED'}

        _reload_pending = True
        bpy.app.timers.register(_reload_rCAD_utils_timer, first_interval=0.1)
        self.report({'INFO'}, "rCAD Utils will reload after this operation finishes.")
        return {'FINISHED'}


class RCAD_PT_Main(bpy.types.Panel):
    bl_label = "rCAD Utils"
    bl_idname = "RCAD_PT_Main"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"

    def draw(self, context):
        pass


class RCAD_PT_ExtrudeAlongPath(bpy.types.Panel):
    bl_label = "Extrude Along Path"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"
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
    bl_category = "rCAD Utils"
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
    bl_category = "rCAD Utils"
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
    bl_category = "rCAD Utils"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        row = box.row(align=True)
        row.operator("mesh.mesh_tiler_preview", text="Preview")
        row.operator("mesh.mesh_tiler", text="Tile")


class RCAD_PT_MirrorAlongPlane(bpy.types.Panel):
    bl_label = "Mirror Across Plane"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"
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


class RCAD_PT_AxisEdgeHighlighter(bpy.types.Panel):
    bl_label = "Axis Edge Highlighter"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()

        axis_box = box.box()
        header = axis_box.row(align=True)
        header.label(text="Axis Edge Highlighter")
        header.operator_context = 'INVOKE_DEFAULT'
        header.operator("rcad.axis_edge_highlight_preferences", text="", icon='PREFERENCES')
        axis_box.label(text="Select a mesh, then activate the highlight")
        row = axis_box.row(align=True)
        label = "Hide Axis Edges" if context.scene.rcad_axis_highlight_enabled else "Highlight Axis Edges"
        row.operator("rcad.toggle_axis_edge_highlight", text=label, icon='MESH_DATA')
        row.operator("rcad.clear_axis_edge_highlight", text="Clear", icon='X')
        axis_box.prop(context.scene, "rcad_axis_highlight_tolerance", text="Tolerance")
        axis_box.prop(context.scene, "rcad_axis_highlight_show_other", text="Show Other")
        axis_box.label(text="X = red   Y = green   Z = blue")


class RCAD_PT_PartSeparator(bpy.types.Panel):
    bl_label = "Part Separator"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.label(text="Recover welded boards")
        box.operator("mesh.rcad_separate_parts", text="Separate 2x4 Islands")
        box.operator("mesh.rcad_mark_part_ids", text="Mark IDs for Future Edits")
        box.label(text="Keeps all parts inside one object.")


class RCAD_PT_AddonDevelopment(bpy.types.Panel):
    bl_label = "Addon Development"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "rCAD Utils"
    bl_parent_id = "RCAD_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        box.label(text="Reload the complete rCAD Utils addon")
        box.operator("wm.rcad_reload_addon", text="Reload rCAD Utils", icon='FILE_REFRESH')
