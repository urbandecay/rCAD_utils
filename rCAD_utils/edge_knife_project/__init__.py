"""Project selected Edit Mode edges onto the active mesh and knife it."""

import bpy

from . import operators, ui, face_cut


def register():
    bpy.types.Scene.rcad_face_cut_fill = bpy.props.BoolProperty(
        name="Fill Faces", default=False,
        description="Fill holes in each target island after separating the split",
    )
    bpy.types.Scene.rcad_face_cut_solver = bpy.props.EnumProperty(
        name="Solver", items=face_cut.SOLVER_ITEMS, default='EXACT',
        description="Solver used by Cut by Face",
    )
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
    del bpy.types.Scene.rcad_face_cut_solver
    del bpy.types.Scene.rcad_face_cut_fill
    for cls in reversed(operators.classes + face_cut.classes + ui.classes):
        bpy.utils.unregister_class(cls)
