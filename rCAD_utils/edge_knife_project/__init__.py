"""Project selected Edit Mode edges onto the active mesh and knife it."""

import bpy

from . import operators, ui


def register():
    for cls in operators.classes + ui.classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(operators.classes + ui.classes):
        bpy.utils.unregister_class(cls)
