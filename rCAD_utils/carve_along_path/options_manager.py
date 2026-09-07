import bpy


SOLVER_ITEMS = [
    ('EXACT', "Exact", "Use Blender's exact Boolean solver"),
]


def register_options():
    scene = bpy.types.Scene
    if hasattr(scene, "carve_along_path_solver"):
        return

    scene.carve_along_path_solver = bpy.props.EnumProperty(
        name="Solver",
        description="Boolean solver used to subtract the swept cutter",
        items=SOLVER_ITEMS,
        default='EXACT',
    )


def unregister_options():
    if hasattr(bpy.types.Scene, "carve_along_path_solver"):
        del bpy.types.Scene.carve_along_path_solver
