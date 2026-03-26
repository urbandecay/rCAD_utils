# ui.py — Panel and property group for 1D Tools (ported from 1D_Lite)

import bpy
from bpy.types import Panel, PropertyGroup
from bpy.props import BoolProperty, FloatProperty, IntProperty, PointerProperty


def _update_arc_size(self, context):
    from .barc_op import update_moved_vertex
    update_moved_vertex(self, context)


class RCAD_1DToolsProps(PropertyGroup):
    # Spread Loop
    spread_x: BoolProperty(name='Spread X', default=False)
    spread_y: BoolProperty(name='Spread Y', default=False)
    spread_z: BoolProperty(name='Spread Z', default=False)
    uniform: BoolProperty(name='Uniform', default=False)
    shape_spline: BoolProperty(name='Shape Spline', default=False)
    smooth_transition: BoolProperty(name='Smooth Transition', default=False)
    shape_inf: IntProperty(name="shape_inf", min=0, max=200, default=0)
    spline_Bspline2: BoolProperty(name="spline_Bspline2", default=True)

    # Create B-Arc
    barc_rad: FloatProperty(name="barc_rad")
    radius: FloatProperty(name='radius', default=0.0)
    arc_size: FloatProperty(
        name='Arc Angle',
        default=0.0,
        min=-180.0,
        max=180.0,
        step=100,
        precision=1,
        update=_update_arc_size,
    )

    # Corner Extend
    only_active_edge: BoolProperty(name="Only Active Edge", default=False)
    to_active_edge: BoolProperty(name="To Active Edge", default=False)

    # Select Loose (Fedge)
    fedge_verts: BoolProperty(name='Verts', default=False)
    fedge_edges: BoolProperty(name='Edges', default=False)
    fedge_ngons: BoolProperty(name='N-gons', default=False)
    fedge_tris: BoolProperty(name='Tris', default=False)
    fedge_area: BoolProperty(name='Area', default=False)
    fedge_wrong_area: FloatProperty(name='', default=0.02, precision=4)
    fedge_three: BoolProperty(name='N-gons', default=True)
    fedge_WRONG_AREA: FloatProperty(name="WRONG_AREA", default=0.02, precision=4)
    fedge_zerop: BoolProperty(name='Zero Area', default=True)


# -- Parent panel for all 1D tools --

class RCAD_PT_1DTools(Panel):
    bl_label = "1D Tools"
    bl_idname = "RCAD_PT_1DTools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_Main'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        pass


# -- Spread Loop sub-panel --

class RCAD_PT_SpreadLoop(Panel):
    bl_label = "Spread Loop"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_1DTools'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = context.window_manager.rcad_1d_props

        box = layout.box()
        col = box.column()
        col.prop(props, "spread_x")
        col.prop(props, "spread_y")
        col.prop(props, "spread_z")
        col.prop(props, "uniform")

        box2 = layout.box()
        col2 = box2.column()
        col2.prop(props, "shape_spline")
        col2.prop(props, "smooth_transition")

        layout.operator("rcad.spread_loop")


# -- Create B-Arc sub-panel --

class RCAD_PT_CreateBArc(Panel):
    bl_label = "Create B-Arc"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_1DTools'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = context.window_manager.rcad_1d_props

        box = layout.box()
        col = box.column(align=True)
        col.operator("rcad.create_b_arc", text="Create B-Arc")
        col.prop(props, "arc_size", slider=True)


# -- Corner Extend sub-panel --

class RCAD_PT_CornerEdges(Panel):
    bl_label = "Corner Edges"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_1DTools'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = context.scene.rcad_1d_props

        box = layout.box()
        col = box.column(align=True)
        col.operator("rcad.corner_extend")
        col.prop(props, "only_active_edge")
        col.prop(props, "to_active_edge")


# -- Select Loose sub-panel --

class RCAD_PT_SelectLoose(Panel):
    bl_label = "Select Loose"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'rCAD'
    bl_parent_id = 'RCAD_PT_1DTools'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        props = context.window_manager.rcad_1d_props

        box = layout.box()
        col = box.column()
        col.prop(props, "fedge_verts")
        col.prop(props, "fedge_edges")
        col.prop(props, "fedge_ngons")
        col.prop(props, "fedge_tris")
        row = box.row()
        row.prop(props, "fedge_area")
        row.prop(props, "fedge_wrong_area")

        layout.operator('rcad.select_loose')


classes = (
    RCAD_1DToolsProps,
    RCAD_PT_1DTools,
    RCAD_PT_SpreadLoop,
    RCAD_PT_CreateBArc,
    RCAD_PT_CornerEdges,
    RCAD_PT_SelectLoose,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.rcad_1d_props = PointerProperty(type=RCAD_1DToolsProps)
    bpy.types.WindowManager.rcad_1d_props = PointerProperty(type=RCAD_1DToolsProps)


def unregister():
    if hasattr(bpy.types.WindowManager, "rcad_1d_props"):
        del bpy.types.WindowManager.rcad_1d_props
    if hasattr(bpy.types.Scene, "rcad_1d_props"):
        del bpy.types.Scene.rcad_1d_props
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
