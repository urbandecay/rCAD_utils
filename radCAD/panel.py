bl_info = {
    "name": "radCAD",
    "blender": (4, 2, 0),
    "category": "3D View",
}

import bpy
import bpy.utils.previews
import os
from .modal_state import state 
from .modal_core import DrawManager

CURRENT_DIR = os.path.dirname(__file__)
POSSIBLE_PATHS = [
    os.path.join(CURRENT_DIR, "icons"),
    os.path.join(CURRENT_DIR, "Toolbar Icons"),
    "/home/molotovgirl/Desktop/ArcTools/Toolbar Icons/"
]

ICON_FOLDER = ""
for p in POSSIBLE_PATHS:
    if os.path.exists(p):
        ICON_FOLDER = p
        break

HEADER_HEIGHT = 1.5 

IMPLEMENTED_TOOLS = {
    "arc_1_point",
    "arc_2_point",
    "arc_3_point",
    "circle_2_points",
    "circle_3_points",    "circle_tangent_to_three_curves", 
    "circle_tangent_to_two_curves", 
    "ellipse_from_radius",  
    "ellipse_foci_point",   
    "ellipse_from_endpoints",
    "ellipse_from_corners",
    "polygon_cen_cor",
    "polygon_cen_tan",
    "polygon_cor_cor",
    "polygon_size_size", 
    "line", 
    "line_perpendicular_from_curve", 
    "line_tangent_to_two_curves", 
    "line_perpendicular_to_two_curves",
    "line_tangent_from_curve", 
    "curve_interpolate_points",
    "curve_freehand",
    "point_by_arcs",
    "point_center",
    "rectangle_from_center",
    "rectangle_from_corners",
    "rectangle_3_points", 
}

SVG_FILES = {
    "arc_1_point": "1_point_arc.svg",
    "arc_2_point": "2_point_arc.svg",
    "arc_3_point": "3_point_arc.svg",
    "arc_from_endpoint": "arc_from_endpoint.svg",
    "line": "line.svg",
    "curve_freehand": "line_freehand.svg",
    "line_tangent_from_curve": "line_tangent_from_curve.svg",
    "line_tangent_to_two_curves": "line_tangent_to_two_curves.svg",
    "line_perpendicular_from_curve": "line_perpendicular_from_curve.svg",
    "line_perpendicular_to_two_curves": "line_perpendicular_to_two_curves.svg",
    "circle": "circle.svg",
    "circle_2_points": "circle_2_points.svg",
    "circle_3_points": "circle_3_points.svg",
    "circle_tangent_to_three_curves": "circle_tangent_to_three_curves.svg",
    "circle_tangent_to_two_curves": "circle_tangent_to_two_curves.svg",
    "curve_interpolate_points": "curve_interpolate_points.svg",
    "ellipse": "ellipse.svg",
    "ellipse_foci_point": "ellipse_foci_point.svg",
    "ellipse_from_corners": "ellipse_from_corners.svg",
    "ellipse_from_endpoints": "ellipse_from_endpoints.svg",
    "ellipse_from_radius": "ellipse_from_radius.svg",
    "polygon_cen_cor": "polygon_cen_cor.svg",
    "polygon_cen_tan": "polygon_cen_tan.svg",
    "polygon_cor_cor": "polygon_cor_cor.svg",
    "polygon_size_size": "polygon_size_size.svg",
    "rectangle_from_center": "rectangle_from_center.svg",
    "rectangle_from_corners": "rectangle_from_corners.svg",
    "rectangle_3_points": "rectangle_3_points.svg",
}

preview_collection = None

def _has_icon(key: str) -> bool:
    return (preview_collection is not None) and (key in preview_collection)

class RADCAD_OT_reset_overlays(bpy.types.Operator):
    bl_idname = "radcad.reset_overlays"
    bl_label = "Clear Stuck Overlays"
    
    def execute(self, context):
        state["active"] = False
        DrawManager.clear_all()
        
        # Clear legacy/driver based handles if any persist
        if "radcad_handles" in bpy.app.driver_namespace:
            for h, region_type in bpy.app.driver_namespace["radcad_handles"]:
                try: bpy.types.SpaceView3D.draw_handler_remove(h, region_type)
                except Exception: pass
            bpy.app.driver_namespace["radcad_handles"] = []
            
        context.area.tag_redraw()
        return {'FINISHED'}

class RADCAD_OT_generic(bpy.types.Operator):
    bl_idname = "radcad.generic"
    bl_label = "CAD Tool"

    name: bpy.props.StringProperty()
    panel: bpy.props.StringProperty()

    def execute(self, context):
        setattr(context.scene, f"radcad_{self.panel}_icon", self.name)
        
        if self.panel == "line":
            if self.name == "line":
                bpy.ops.view3d.line_polyline('INVOKE_DEFAULT')
            elif self.name == "line_perpendicular_from_curve": 
                bpy.ops.view3d.line_perp_from_curve('INVOKE_DEFAULT')
            elif self.name == "line_tangent_to_two_curves": 
                bpy.ops.view3d.line_tan_tan('INVOKE_DEFAULT')
            elif self.name == "line_perpendicular_to_two_curves":
                bpy.ops.view3d.line_perp_to_two_curves('INVOKE_DEFAULT')
            elif self.name == "line_tangent_from_curve":
                bpy.ops.view3d.line_tangent_from_curve('INVOKE_DEFAULT')

        elif self.panel == "curve":
            if self.name == "curve_interpolate_points":
                bpy.ops.view3d.curve_interpolate('INVOKE_DEFAULT')
            elif self.name == "curve_freehand":
                bpy.ops.view3d.curve_freehand('INVOKE_DEFAULT')

        elif self.panel == "arc" and self.name == "arc_1_point":
            bpy.ops.view3d.arc_overlay_preview('INVOKE_DEFAULT')
        elif self.panel == "arc" and self.name == "arc_2_point":
            bpy.ops.view3d.arc_2pt('INVOKE_DEFAULT')
        elif self.panel == "arc" and self.name == "arc_3_point":
            bpy.ops.view3d.arc_3pt('INVOKE_DEFAULT')
            
        elif self.panel == "circle" and self.name == "circle_2_points":
            bpy.ops.view3d.circle_2pt('INVOKE_DEFAULT')
        elif self.panel == "circle" and self.name == "circle_3_points": 
            bpy.ops.view3d.circle_3pt('INVOKE_DEFAULT')
        elif self.panel == "circle" and self.name == "circle_tangent_to_three_curves":
            bpy.ops.view3d.radcad_circle_tan_tan_tan('INVOKE_DEFAULT')
        elif self.panel == "circle" and self.name == "circle_tangent_to_two_curves":
            bpy.ops.view3d.radcad_circle_tan_tan('INVOKE_DEFAULT')

        elif self.panel == "ellipse" and self.name == "ellipse_from_radius":
            bpy.ops.view3d.ellipse_radius('INVOKE_DEFAULT')
        elif self.panel == "ellipse" and self.name == "ellipse_foci_point":
            bpy.ops.view3d.ellipse_foci('INVOKE_DEFAULT')
        elif self.panel == "ellipse" and self.name == "ellipse_from_endpoints":
            bpy.ops.view3d.ellipse_endpoints('INVOKE_DEFAULT')
        elif self.panel == "ellipse" and self.name == "ellipse_from_corners":
            bpy.ops.view3d.ellipse_corners('INVOKE_DEFAULT')

        elif self.panel == "polygon" and self.name == "polygon_cen_cor":
            bpy.ops.view3d.polygon_cen_cor('INVOKE_DEFAULT')
        elif self.panel == "polygon" and self.name == "polygon_cen_tan":
            bpy.ops.view3d.polygon_cen_tan('INVOKE_DEFAULT')
        elif self.panel == "polygon" and self.name == "polygon_cor_cor":
            bpy.ops.view3d.polygon_cor_cor('INVOKE_DEFAULT')
        elif self.panel == "polygon" and self.name == "polygon_size_size": 
            bpy.ops.view3d.polygon_edge('INVOKE_DEFAULT')
            
        elif self.panel == "point":
            if self.name == "point_by_arcs":
                bpy.ops.view3d.point_by_arcs('INVOKE_DEFAULT')
            elif self.name == "point_center":
                bpy.ops.view3d.point_center('INVOKE_DEFAULT')

        elif self.panel == "rectangle":
            if self.name == "rectangle_from_center":
                bpy.ops.view3d.rectangle_cen_cor('INVOKE_DEFAULT')
            elif self.name == "rectangle_from_corners":
                bpy.ops.view3d.rectangle_cor_cor('INVOKE_DEFAULT')
            elif self.name == "rectangle_3_points":
                bpy.ops.view3d.rectangle_3_points('INVOKE_DEFAULT')
            
        return {'FINISHED'}

def draw_header(layout, icon_key):
    col = layout.column(align=True)
    col.alignment = 'CENTER'
    col.ui_units_y = HEADER_HEIGHT
    if _has_icon(icon_key):
        col.template_icon(icon_value=preview_collection[icon_key].icon_id, scale=2)
    else:
        col.label(text=" ")
    layout.separator()

def draw_tool_button(layout, key, panel_name):
    row = layout.row()
    if key not in IMPLEMENTED_TOOLS:
        row.enabled = False 
    
    if _has_icon(key):
        op = row.operator("radcad.generic", text=key, icon_value=preview_collection[key].icon_id)
    else:
        op = row.operator("radcad.generic", text=key)
        
    op.name = key
    op.panel = panel_name

class RADCAD_PT_Main(bpy.types.Panel):
    bl_label = "radCAD"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.scale_y = 1.2
        row.operator("radcad.reset_overlays", icon='TRASH') 
        layout.separator()

class RADCAD_PT_Point(bpy.types.Panel):
    bl_label = "Point"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_point_icon)
        draw_tool_button(self.layout, "point_by_arcs", "point")
        draw_tool_button(self.layout, "point_center", "point")

class RADCAD_PT_Line(bpy.types.Panel):
    bl_label = "Line"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_line_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("line")):
            draw_tool_button(self.layout, key, "line")

class RADCAD_PT_Arc(bpy.types.Panel):
    bl_label = "Arc"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_arc_icon)
        keys = [k for k in SVG_FILES if k.startswith("arc")]
        for key in sorted(keys):
            draw_tool_button(self.layout, key, "arc")

class RADCAD_PT_Circle(bpy.types.Panel):
    bl_label = "Circle"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_circle_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("circle")):
            draw_tool_button(self.layout, key, "circle")

class RADCAD_PT_Ellipse(bpy.types.Panel):
    bl_label = "Ellipse"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_ellipse_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("ellipse")):
            draw_tool_button(self.layout, key, "ellipse")

class RADCAD_PT_Polygon(bpy.types.Panel):
    bl_label = "Polygon"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_polygon_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("polygon")):
            draw_tool_button(self.layout, key, "polygon")

class RADCAD_PT_Curve(bpy.types.Panel):
    bl_label = "Curve"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_curve_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("curve")):
            draw_tool_button(self.layout, key, "curve")

class RADCAD_PT_Rectangle(bpy.types.Panel):
    bl_label = "Rectangle"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "radCAD"
    bl_parent_id = "RADCAD_PT_Main"

    def draw(self, context):
        draw_header(self.layout, context.scene.radcad_rectangle_icon)
        for key in sorted(k for k in SVG_FILES if k.startswith("rectangle")):
            draw_tool_button(self.layout, key, "rectangle")

classes = (
    RADCAD_OT_reset_overlays, 
    RADCAD_OT_generic,
    RADCAD_PT_Main,
    RADCAD_PT_Point,
    RADCAD_PT_Line,
    RADCAD_PT_Arc,
    RADCAD_PT_Circle,
    RADCAD_PT_Ellipse,
    RADCAD_PT_Polygon,
    RADCAD_PT_Curve,
    RADCAD_PT_Rectangle,
)

def register():
    global preview_collection
    preview_collection = bpy.utils.previews.new()

    if ICON_FOLDER and os.path.exists(ICON_FOLDER):
        for key, filename in SVG_FILES.items():
            path = os.path.join(ICON_FOLDER, filename)
            if os.path.exists(path):
                preview_collection.load(key, path, "IMAGE")

    bpy.types.Scene.radcad_line_icon = bpy.props.StringProperty(default="line")
    bpy.types.Scene.radcad_arc_icon = bpy.props.StringProperty(default="arc_1_point")
    bpy.types.Scene.radcad_circle_icon = bpy.props.StringProperty(default="circle")
    bpy.types.Scene.radcad_ellipse_icon = bpy.props.StringProperty(default="ellipse")
    bpy.types.Scene.radcad_polygon_icon = bpy.props.StringProperty(default="polygon_cen_cor")
    bpy.types.Scene.radcad_curve_icon = bpy.props.StringProperty(default="curve_interpolate_points")
    bpy.types.Scene.radcad_rectangle_icon = bpy.props.StringProperty(default="rectangle_from_center")
    bpy.types.Scene.radcad_point_icon = bpy.props.StringProperty(default="point_by_arcs")

    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.radcad_line_icon
    del bpy.types.Scene.radcad_arc_icon
    del bpy.types.Scene.radcad_circle_icon
    del bpy.types.Scene.radcad_ellipse_icon
    del bpy.types.Scene.radcad_polygon_icon
    del bpy.types.Scene.radcad_curve_icon
    del bpy.types.Scene.radcad_rectangle_icon
    del bpy.types.Scene.radcad_point_icon

    if preview_collection:
        bpy.utils.previews.remove(preview_collection)

if __name__ == "__main__":
    register()