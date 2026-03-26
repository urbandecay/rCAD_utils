# ui.py

import bpy
from bpy.types import Panel, PropertyGroup
from bpy.props import BoolProperty, FloatProperty, PointerProperty


class FuseGeometryProps(PropertyGroup):
    # Global parameters (available in the add-on UI; used by all operators)
    search_radius: FloatProperty(
        name="Search Radius",
        description="Unified world-space search radius used by all Super Fuse operations. "
                    "For X, also acts as merge distance.",
        default=1e-4, min=0.0, soft_max=0.01, subtype='DISTANCE'
    )

    # Top-level toggles
    x: BoolProperty(
        name="X",
        description="Enable X crossings (edge-edge)",
        default=False,
    )
    t: BoolProperty(
        name="T",
        description="Enable T junctions (only weld selected vertices that belong to an edge; ignores loose verts)",
        default=False,
    )
    l: BoolProperty(
        name="L",
        description="Enable L endpoint weld (coincident endpoints of selected edges)",
        default=False,
    )
    dot: BoolProperty(
        name=".",
        description="Enable verts-to-edges for loose verts only (merge selected loose verts to selected edges)",
        default=False,
    )
    dotdot: BoolProperty(
        name="..",
        description="Enable Heavy Weld: for each selected vertex, split ALL nearby selected edges and weld to a single central point",
        default=False,
    )
    square: BoolProperty(
        name="■",
        description="Enable Face-on-Face weld: split nearby edges of selected target face(s) and weld the overlay polygon/face onto them",
        default=False,
    )


class OSC_PT_fuse_geometry(Panel):
    bl_label = "Weld Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = context.scene.super_fuse if hasattr(context.scene, "super_fuse") else None
        if props is None:
            layout.label(text="Super Fuse properties not initialized.", icon='ERROR')
            return

        # Global parameters (set before running; not only in the redo panel)
        box_params = layout.box()
        colp = box_params.column(align=True)
        colp.label(text="Parameters", icon='PREFERENCES')
        colp.prop(props, "search_radius", text="Search Radius")

        # Toggles in execution order: L, T, ., .., X, ■
        # Enable Operations label removed from UI — execution order: L → T → . → .. → X → ■
        box = layout.box()
        col = box.column(align=True)

        row1 = col.row(align=True)
        row1.prop(props, "l", text="L")
        row1.prop(props, "t", text="T")
        row1.prop(props, "dot", text=".")

        row2 = col.row(align=True)
        row2.prop(props, "dotdot", text="..")
        row2.prop(props, "x", text="X")
        row2.prop(props, "square", text="■")

        col.separator()
        col.operator("osc.super_fuse_execute", text="Fuse", icon='AUTOMERGE_ON')


classes = (FuseGeometryProps, OSC_PT_fuse_geometry)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.super_fuse = PointerProperty(type=FuseGeometryProps)


def unregister():
    if hasattr(bpy.types.Scene, "super_fuse"):
        del bpy.types.Scene.super_fuse
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)