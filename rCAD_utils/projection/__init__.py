"""Explicit, viewport-independent mesh surface projection feature."""

import bpy
from bpy.props import EnumProperty

from . import operators, ui
from .storage import projection_state


def register():
    for cls in operators.classes + ui.classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.rcad_projection_direction = EnumProperty(
        name="Projection Direction",
        description="Direction used to project stored vertices onto the target",
        items=(
            ('NORMAL', "Face Normal", "Project along the target face normal"),
            ('HORIZONTAL', "Horizontal", "Project toward the target in the world horizontal plane"),
            ('VERTICAL', "Vertical", "Project along the global vertical axis"),
        ),
        default='NORMAL',
    )


def unregister():
    if hasattr(bpy.types.Scene, "rcad_projection_direction"):
        del bpy.types.Scene.rcad_projection_direction
    projection_state.clear()
    for cls in reversed(operators.classes + ui.classes):
        bpy.utils.unregister_class(cls)
