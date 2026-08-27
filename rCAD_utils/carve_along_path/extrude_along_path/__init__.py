bl_info = {
    "name": "Extrude Along Path",
    "author": "Zmj100, 1D_inc, RobbieK",
    "version": (2, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > OSC",
    "description": "Extrude or Rake profile geometry Along a defined Path",
    "category": "Mesh",
}

import bpy
from . import ui
from . import options_manager

classes = [
    ui.OT_ExtrudeAlongPath_Store_Path,
    ui.OT_ExtrudeAlongPath_Store_Start_Point,
    ui.OT_ExtrudeAlongPath_Store_Both,
    ui.OT_ExtrudeAlongPath_Extrude,
    ui.PT_ExtrudeAlongPath,
]

def register():
    options_manager.register_options()
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except Exception as e:
            print("Error registering class", cls, ":", e)

def unregister():
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception as e:
            print("Error unregistering class", cls, ":", e)
    options_manager.unregister_options()

if __name__ == "__main__":
    register()