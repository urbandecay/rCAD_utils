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
            ('HORIZONTAL', "Horizontal", "Project along a fixed world horizontal axis"),
        ),
        default='NORMAL',
    )
    bpy.types.Scene.rcad_projection_horizontal_axis = EnumProperty(
        name="Horizontal Axis",
        description="World axis used for horizontal projection",
        items=(
            ('X', "World X", "Project along the global X axis"),
            ('Y', "World Y", "Project along the global Y axis"),
        ),
        default='Y',
    )


def unregister():
    if hasattr(bpy.types.Scene, "rcad_projection_horizontal_axis"):
        del bpy.types.Scene.rcad_projection_horizontal_axis
    if hasattr(bpy.types.Scene, "rcad_projection_direction"):
        del bpy.types.Scene.rcad_projection_direction
    projection_state.clear()
    for cls in reversed(operators.classes + ui.classes):
        bpy.utils.unregister_class(cls)
