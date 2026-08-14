import bpy

from . import operators, ui
from .storage import projection_source


def register():
    for cls in operators.classes + ui.classes:
        bpy.utils.register_class(cls)


def unregister():
    projection_source.clear()
    for cls in reversed(operators.classes + ui.classes):
        bpy.utils.unregister_class(cls)
