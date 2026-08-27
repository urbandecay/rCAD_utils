bl_info = {
    "name": "Carve Along Path",
    "author": "RobbieK",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > rCAD Utils",
    "description": "Carve a selected face profile along a stored path with a Boolean Difference",
    "category": "Mesh",
}

import bpy

from . import options_manager
from . import ui


classes = [
    ui.OT_CarveAlongPath_Store_Path,
    ui.OT_CarveAlongPath_Carve,
]


def register():
    options_manager.register_options()
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    options_manager.unregister_options()

