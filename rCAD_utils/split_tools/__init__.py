bl_info = {
    "name": "Split Tools",
    "author": "RobbieK",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > rCAD Utils > Split Tools",
    "description": "Split selected edges at edge and vertex intersections.",
    "category": "Mesh",
}

from . import execute_split
from . import ui


_registered_modules = []


def register():
    global _registered_modules
    if _registered_modules:
        return

    for module in (ui, execute_split):
        module.register()
        _registered_modules.append(module)


def unregister():
    global _registered_modules
    for module in reversed(_registered_modules):
        module.unregister()
    _registered_modules = []

