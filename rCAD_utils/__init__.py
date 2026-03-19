bl_info = {
    "name": "rCAD Utils",
    "author": "RobbieK",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > rCAD",
    "description": "Container addon housing rCAD mesh tools",
    "category": "Mesh",
}

import bpy
from bpy.props import BoolProperty
from . import panel
from .extrude_along_path import ui as eap_ui
from .extrude_along_path import options_manager as eap_options
from .place_profile import (
    OBJECT_OT_store_profile_info_edit,
    OBJECT_OT_place_profile_on_edges_edit,
)
from .mirror_along_plane import (
    MESH_OT_store_plane_vertices,
    MESH_OT_point_reflection,
    VertexStorage,
)

classes = [
    # Extrude Along Path operators
    eap_ui.OT_ExtrudeAlongPath_Store_Path,
    eap_ui.OT_ExtrudeAlongPath_Store_Start_Point,
    eap_ui.OT_ExtrudeAlongPath_Store_Both,
    eap_ui.OT_ExtrudeAlongPath_Extrude,
    # Place Profile operators
    OBJECT_OT_store_profile_info_edit,
    OBJECT_OT_place_profile_on_edges_edit,
    # Mirror Across Plane operators
    MESH_OT_store_plane_vertices,
    MESH_OT_point_reflection,
    # Container panels
    panel.RCAD_PT_Main,
    panel.RCAD_PT_ExtrudeAlongPath,
    panel.RCAD_PT_PlaceProfile,
    panel.RCAD_PT_MirrorAlongPlane,
]


def register():
    eap_options.register_options()
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.profile_path_mode = BoolProperty(name="Path Mode", default=False)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    eap_options.unregister_options()
    VertexStorage._instance = None
    if hasattr(bpy.types.Scene, "profile_path_mode"):
        del bpy.types.Scene.profile_path_mode


if __name__ == "__main__":
    register()
