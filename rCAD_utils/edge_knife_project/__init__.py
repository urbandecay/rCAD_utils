"""Project selected Edit Mode edges onto the active mesh and knife it."""

import bpy

from . import operators, ui, face_cut


def register():
    bpy.types.Scene.rcad_knife_separate_split = bpy.props.BoolProperty(
        name="Separate Split",
        description="Disconnect the cut seam within the mesh; disable to keep cut faces joined",
        default=True,
    )
    for cls in operators.classes + face_cut.classes + ui.classes:
        bpy.utils.register_class(cls)


def unregister():
    operators.stop_preview()
    del bpy.types.Scene.rcad_knife_separate_split
    for cls in reversed(operators.classes + face_cut.classes + ui.classes):
        bpy.utils.unregister_class(cls)
