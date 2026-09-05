import bpy
from bpy.props import BoolProperty, FloatProperty, PointerProperty
from bpy.types import Panel, PropertyGroup


class SplitGeometryProps(PropertyGroup):
    search_radius: FloatProperty(
        name="Search Radius",
        description="World-space tolerance used when looking for edge and vertex intersections",
        default=1e-4,
        min=0.0,
        soft_max=0.01,
        subtype='DISTANCE',
    )

    x: BoolProperty(
        name="X",
        description="Split selected edges where they intersect other mesh edges, including T and X crossings",
        default=False,
    )

    dot: BoolProperty(
        name=".",
        description="Split selected edges at any mesh vertex lying on them",
        default=False,
    )

    separate: BoolProperty(
        name="Separate",
        description="Disconnect the edge arms at each newly created split point",
        default=False,
    )


class RCAD_PT_SplitTools(Panel):
    bl_label = "Split Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD Utils'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = getattr(context.scene, "split_tools", None)
        if props is None:
            layout.label(text="Split Tools properties not initialized.", icon='ERROR')
            return

        box_params = layout.box()
        col_params = box_params.column(align=True)
        col_params.label(text="Parameters", icon='PREFERENCES')
        col_params.prop(props, "search_radius", text="Search Radius")

        box = layout.box()
        col = box.column(align=True)
        row = col.row(align=True)
        row.prop(props, "x", text="X")
        row.prop(props, "dot", text=".")

        col.prop(props, "separate", text="Separate")

        col.separator()
        col.operator("mesh.split_tools_execute", text="Split", icon='EDGESEL')


classes = (SplitGeometryProps, RCAD_PT_SplitTools)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.split_tools = PointerProperty(type=SplitGeometryProps)


def unregister():
    if hasattr(bpy.types.Scene, "split_tools"):
        del bpy.types.Scene.split_tools
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
