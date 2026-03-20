# radCAD/preferences.py

import bpy

def get_prefs():
    """Helper to access the addon preferences from anywhere."""
    try:
        return bpy.context.preferences.addons[__package__].preferences
    except (KeyError, AttributeError):
        return None

class RADCAD_Preferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    # =========================================================================
    # UI STATE PROPERTIES
    # =========================================================================
    show_global_settings: bpy.props.BoolProperty(name="Global Settings", default=True)
    show_arc_settings: bpy.props.BoolProperty(name="1 Point Arc Settings", default=True)
    show_points_by_arc_settings: bpy.props.BoolProperty(name="Points by Arc Settings", default=True)
    show_arc_2pt_settings: bpy.props.BoolProperty(name="2 Point Arc Settings", default=True)
    show_arc_3pt_settings: bpy.props.BoolProperty(name="3 Point Arc Settings", default=True)
    show_circle_2pt_settings: bpy.props.BoolProperty(name="2 Point Circle Settings", default=True)
    show_circle_3pt_settings: bpy.props.BoolProperty(name="3 Point Circle Settings", default=True)
    show_line_settings: bpy.props.BoolProperty(name="Line Settings", default=True)
    show_line_perp_settings: bpy.props.BoolProperty(name="Line Perpendicular from Curve Settings", default=True)
    show_line_perp2_settings: bpy.props.BoolProperty(name="Line Perpendicular to Two Curves Settings", default=True)
    show_line_tangent_settings: bpy.props.BoolProperty(name="Line Tangent from Curve Settings", default=True)
    show_line_tan_tan_settings: bpy.props.BoolProperty(name="Line Tangent to Two Curves Settings", default=True)
    show_circle_tan3_settings: bpy.props.BoolProperty(name="Circle Tangent to Three Curves Settings", default=True)
    show_circle_tan2_settings: bpy.props.BoolProperty(name="Circle Tangent to Two Curves Settings", default=True)
    show_ellipse_foci_settings: bpy.props.BoolProperty(name="Ellipse (Foci Point) Settings", default=True)
    show_ellipse_corners_settings: bpy.props.BoolProperty(name="Ellipse (From Corners) Settings", default=True)
    show_ellipse_endpoints_settings: bpy.props.BoolProperty(name="Ellipse (From Endpoints) Settings", default=True)
    show_ellipse_radius_settings: bpy.props.BoolProperty(name="Ellipse (From Radius) Settings", default=True)
    show_polygon_settings: bpy.props.BoolProperty(name="Polygon Settings", default=True)
    show_rectangle_settings: bpy.props.BoolProperty(name="Rectangle Settings", default=True)
    show_curve_settings: bpy.props.BoolProperty(name="Curve Settings", default=True)

    # =========================================================================
    # SETTINGS PROPERTIES
    # =========================================================================
    
    use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the line will turn the axis color (Red, Green, Blue). If off, it stays the default color",
        default=True
    )

    # --- Line Perpendicular from Curve ---
    line_perp_show_catmull: bpy.props.BoolProperty(
        name="Show Catmull Overlay",
        description="Toggle the visibility of the Catmull-Rom spline overlay for the Perpendicular Line tool",
        default=True
    )

    line_perp_col_catmull: bpy.props.FloatVectorProperty(
        name="Catmull Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5), # Cyan-ish
        description="Color for the Catmull-Rom spline preview in the Perpendicular Line tool"
    )

    line_perp_width_catmull: bpy.props.FloatProperty(
        name="Catmull Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline preview in the Perpendicular Line tool",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Line Perpendicular to Two Curves ---
    line_perp2_show_catmull: bpy.props.BoolProperty(
        name="Show Catmull Overlay",
        description="Toggle the visibility of the Catmull-Rom spline overlay for the Perp to Two Curves tool",
        default=True
    )

    line_perp2_col_catmull: bpy.props.FloatVectorProperty(
        name="Catmull Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the Catmull-Rom spline preview in the Perp to Two Curves tool"
    )

    line_perp2_width_catmull: bpy.props.FloatProperty(
        name="Catmull Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline preview in the Perp to Two Curves tool",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Line Tangent from Curve ---
    line_tangent_show_catmull: bpy.props.BoolProperty(
        name="Show Catmull Overlay",
        description="Toggle the visibility of the Catmull-Rom spline overlay for the Tangent tool",
        default=True
    )

    line_tangent_col_catmull: bpy.props.FloatVectorProperty(
        name="Catmull Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5), # Cyan-ish
        description="Color for the Catmull-Rom spline preview in the Tangent tool"
    )

    line_tangent_width_catmull: bpy.props.FloatProperty(
        name="Catmull Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline preview in the Tangent tool",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Line Tangent to Two Curves ---
    line_tan_tan_show_catmull: bpy.props.BoolProperty(
        name="Show Catmull Overlay",
        description="Toggle the visibility of the Catmull-Rom spline overlay for the Tangent to Two Curves tool",
        default=True
    )

    line_tan_tan_col_catmull: bpy.props.FloatVectorProperty(
        name="Catmull Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the Catmull-Rom spline preview in the Tangent to Two Curves tool"
    )

    line_tan_tan_width_catmull: bpy.props.FloatProperty(
        name="Catmull Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline preview in the Tangent to Two Curves tool",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Circle Tangent to Three Curves ---
    circle_tan3_show_curves: bpy.props.BoolProperty(
        name="Show Curve Overlays",
        description="Toggle the visibility of the Catmull-Rom spline overlays for the 3 source curves",
        default=True
    )

    circle_tan3_col_curves: bpy.props.FloatVectorProperty(
        name="Curve Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the Catmull-Rom spline overlays"
    )

    circle_tan3_width_curves: bpy.props.FloatProperty(
        name="Curve Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline overlays",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    circle_tan3_show_tangent: bpy.props.BoolProperty(
        name="Show Tangent Circle",
        description="Toggle the visibility of the calculated tangent circle",
        default=True
    )

    circle_tan3_col_tangent: bpy.props.FloatVectorProperty(
        name="Tangent Circle Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the tangent circle preview"
    )

    circle_tan3_col_preview: bpy.props.FloatVectorProperty(
        name="Preview Geometry Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.0, 0.0, 1.0),
        description="Color for the mesh geometry preview (Default: 0, 0, 0)"
    )

    circle_tan3_width_tangent: bpy.props.FloatProperty(
        name="Tangent Circle Thickness",
        description="Line thickness for the tangent circle preview",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Circle Tangent to Two Curves ---
    circle_tan2_show_curves: bpy.props.BoolProperty(
        name="Show Curve Overlays",
        description="Toggle the visibility of the Catmull-Rom spline overlays for the 2 source curves",
        default=True
    )

    circle_tan2_col_curves: bpy.props.FloatVectorProperty(
        name="Curve Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the Catmull-Rom spline overlays"
    )

    circle_tan2_width_curves: bpy.props.FloatProperty(
        name="Curve Overlay Thickness",
        description="Line thickness for the Catmull-Rom spline overlays",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    circle_tan2_show_tangent: bpy.props.BoolProperty(
        name="Show Tangent Circle",
        description="Toggle the visibility of the calculated tangent circle",
        default=True
    )

    circle_tan2_col_tangent: bpy.props.FloatVectorProperty(
        name="Tangent Circle Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.8, 1.0, 0.5),
        description="Color for the tangent circle preview"
    )

    circle_tan2_col_preview: bpy.props.FloatVectorProperty(
        name="Preview Geometry Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 0.0, 0.0, 1.0),
        description="Color for the mesh geometry preview (Default: 0, 0, 0)"
    )

    circle_tan2_width_tangent: bpy.props.FloatProperty(
        name="Tangent Circle Thickness",
        description="Line thickness for the tangent circle preview",
        default=2.0,
        min=0.5, max=10.0,
        precision=1,
        step=10
    )

    # --- Ellipse from Foci Points ---
    ellipse_foci_col_foci_lines: bpy.props.FloatVectorProperty(
        name="Foci Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 1.0, 0.0, 1.0),
        description="Color for the lines connecting foci to the mouse cursor"
    )

    ellipse_foci_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_ellipse_foci_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    # --- Ellipse from Radius ---
    ellipse_radius_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_ellipse_radius_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    # --- Ellipse from Endpoints ---
    ellipse_endpoints_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_ellipse_endpoints_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    # --- Ellipse from Corners ---
    ellipse_corners_color: bpy.props.FloatVectorProperty(
        name="Corners Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.0, 1.0, 0.0, 1.0),
        description="Color for the bounding box lines in corners mode"
    )

    # --- Polygon Tools ---
    polygon_center_corner_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_polygon_center_corner_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    polygon_center_tangent_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_polygon_center_tangent_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    polygon_corner_corner_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_polygon_corner_corner_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    polygon_edge_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_polygon_edge_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    # --- Rectangle Tools ---
    rectangle_center_corner_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_rectangle_center_corner_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    rectangle_corner_corner_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_rectangle_corner_corner_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    rectangle_3pt_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_rectangle_3pt_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0),
        description="The color for the guide lines when not using axis colors"
    )

    axis_color_dim: bpy.props.FloatProperty(
        name="Axis Color Dimmer",
        description="Controls how much the axis colors are dimmed when snapped. 1.0 is full brightness, 0.0 is black",
        default=1.0,
        min=0.0,
        max=1.0,
        precision=2
    )

    compass_size: bpy.props.IntProperty(
        name="Compass Size",
        description="Controls the size of the visual compass circle is on your screen",
        default=125,
        min=50,
        max=500
    )

    # --- FONT SETTINGS ---
    font_size_hotkey: bpy.props.IntProperty(
        name="Hotkey Font Size",
        description="Change size of fonts",
        default=12,
        min=6,
        max=64
    )

    font_size_label: bpy.props.IntProperty(
        name="Label Font Size",
        description="Change size of fonts",
        default=10,
        min=6,
        max=64
    )

    preview_vertex_size: bpy.props.IntProperty(
        name="Preview Vertex Size",
        description="Customize the size of the points shown during drawing",
        default=5,
        min=1,
        max=50
    )
    
    # VISUAL COLORS (1-Point)
    color_arc_start: bpy.props.FloatVectorProperty(
        name="Start Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.2, 0.8, 0.2, 1.0), # Green, matches End Color
        description="These are your arc start and end point previews"
    )

    color_arc_end: bpy.props.FloatVectorProperty(
        name="End Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.2, 0.8, 0.2, 1.0), 
        description="These are your arc start and end point previews"
    )

    # VISUAL COLORS (2-Point)
    arc_2pt_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the chord and height lines will turn the axis color. If off, they stay the default color",
        default=True
    )

    color_arc_2pt_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0), # Darker Grey
        description="The color for the chord and height guide lines when not using axis colors"
    )

    # --- 3 Point Arc ---
    snap_marker_size_3pt: bpy.props.IntProperty(name="Marker Size", default=6, min=2, max=20)
    snap_marker_color_3pt: bpy.props.FloatVectorProperty(
        name="Marker Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 1.0)
    )
    snap_line_color_3pt: bpy.props.FloatVectorProperty(
        name="Line Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 0.7)
    )
    arc_3pt_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )
    color_arc_3pt_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0), # Darker Grey
        description="The color for the guide lines when not using axis colors"
    )

    # --- 2 Point Circle ---
    snap_marker_size_c2pt: bpy.props.IntProperty(name="Marker Size", default=6, min=2, max=20)
    snap_marker_color_c2pt: bpy.props.FloatVectorProperty(
        name="Marker Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 1.0)
    )
    snap_line_color_c2pt: bpy.props.FloatVectorProperty(
        name="Line Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 0.7)
    )
    circle_2pt_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )
    color_circle_2pt_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0), # Darker Grey
        description="The color for the guide lines when not using axis colors"
    )

    # --- 3 Point Circle ---
    snap_marker_size_c3pt: bpy.props.IntProperty(name="Marker Size", default=6, min=2, max=20)
    snap_marker_color_c3pt: bpy.props.FloatVectorProperty(
        name="Marker Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 1.0)
    )
    snap_line_color_c3pt: bpy.props.FloatVectorProperty(
        name="Line Color", subtype='COLOR', size=4, min=0.0, max=1.0, default=(1.0, 1.0, 0.0, 0.7)
    )
    circle_3pt_use_axis_colors: bpy.props.BoolProperty(
        name="Use Axis Colors",
        description="When snapped to X, Y, or Z, the guide lines will turn the axis color. If off, they stay the default color",
        default=True
    )
    color_circle_3pt_overlay: bpy.props.FloatVectorProperty(
        name="Overlay Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.1, 0.1, 0.1, 1.0), # Darker Grey
        description="The color for the guide lines when not using axis colors"
    )

    # VISUAL COLORS (Points by Arc)
    color_points_by_arc_1: bpy.props.FloatVectorProperty(
        name="Arc 1 Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.2, 0.8, 0.2, 1.0), # Green
        description="Color for the first reference arc"
    )

    color_points_by_arc_2: bpy.props.FloatVectorProperty(
        name="Arc 2 Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.2, 0.8, 0.2, 1.0), # Green
        description="Color for the second reference arc"
    )

    color_points_by_arc_start: bpy.props.FloatVectorProperty(
        name="Start Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.5, 0.5, 0.5, 1.0), # Grey
        description="Color for the radius guide line"
    )

    color_points_by_arc_end: bpy.props.FloatVectorProperty(
        name="End Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(0.5, 0.5, 0.5, 1.0), # Grey
        description="Color for the angle guide line"
    )

    points_by_arc_crosshair_size: bpy.props.IntProperty(
        name="Crosshair Size",
        description="Size of the crosshair marker during second radius setup",
        default=25,
        min=1,
        max=500
    )

    points_by_arc_square_size: bpy.props.IntProperty(
        name="Intersection Square Size",
        description="Size of the final result intersection square",
        default=3,
        min=1,
        max=100
    )

    # --- SNAP MARKER SETTINGS (1-POINT) ---
    snap_marker_size: bpy.props.IntProperty(
        name="Marker Size",
        description="Customize the little target thingy that shows up where you are snapping",
        default=6,
        min=1,
        max=50
    )
    
    snap_marker_color: bpy.props.FloatVectorProperty(
        name="Marker Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(1.0, 0.8, 0.0, 1.0),
        description="Customize the little target thingy that shows up where you are snapping"
    )

    snap_line_color: bpy.props.FloatVectorProperty(
        name="Snap Pointer Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(1.0, 1.0, 0.0, 0.7),
        description="Colors the little leash connecting your mouse to the pivot."
    )
    
    # --- SNAP MARKER SETTINGS (2-POINT) ---
    snap_marker_size_2pt: bpy.props.IntProperty(
        name="Marker Size",
        description="Customize the little target thingy that shows up where you are snapping",
        default=6,
        min=1,
        max=50
    )
    
    snap_marker_color_2pt: bpy.props.FloatVectorProperty(
        name="Marker Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(1.0, 0.8, 0.0, 1.0),
        description="Customize the little target thingy that shows up where you are snapping"
    )

    snap_line_color_2pt: bpy.props.FloatVectorProperty(
        name="Snap Pointer Line Color",
        subtype='COLOR',
        size=4,
        min=0.0, max=1.0,
        default=(1.0, 1.0, 0.0, 0.7),
        description="Colors the little leash connecting your mouse to the pivot."
    )
    
    # PARAMETER OVERLAY OFFSETS
    overlay_offset_x: bpy.props.IntProperty(
        name="Param X Offset",
        description="This nudges the text labels (like 'R: 5m') away from the center point",
        default=75,
        min=-1000,
        max=1000
    )

    overlay_offset_y: bpy.props.IntProperty(
        name="Param Y Offset",
        description="This nudges the text labels (like 'R: 5m') away from the center point",
        default=0,
        min=-1000,
        max=1000
    )

    # HOTKEYS PANEL OFFSETS
    show_hotkeys: bpy.props.BoolProperty(
        name="Show Hotkeys",
        description="Toggles that little cheat sheet overlay",
        default=True
    )
    
    hotkeys_offset_x: bpy.props.IntProperty(
        name="Hotkeys X",
        description="This just moves the hotkey cheat sheet around",
        default=1000,
        min=0,
        max=2000
    )
    
    hotkeys_offset_y: bpy.props.IntProperty(
        name="Hotkeys Y (From Top)",
        description="This just moves the hotkey cheat sheet around",
        default=20,
        min=0,
        max=2000
    )

    # SNAPPING
    angle_snap_type: bpy.props.EnumProperty(
        name="Angle Snap Increment",
        items=[
            ('1', "1 Degree", ""), ('2', "2 Degrees", ""), ('3', "3 Degrees", ""),
            ('5', "5 Degrees", ""), ('10', "10 Degrees", ""), ('15', "15 Degrees", ""),
            ('22.5', "22.5 Degrees", ""), ('30', "30 Degrees", ""), ('45', "45 Degrees", ""),
            ('90', "90 Degrees", ""),
        ],
        default='15'
    )

    angle_snap_type_rad: bpy.props.EnumProperty(
        name="Radian Snap Increment",
        items=[
            ('1', "Pi/180 (1°)", ""), ('2', "Pi/90 (2°)", ""), ('3', "Pi/60 (3°)", ""),
            ('5', "Pi/36 (5°)", ""), ('10', "Pi/18 (10°)", ""), ('15', "Pi/12 (15°)", ""),
            ('22.5', "Pi/8 (22.5°)", ""), ('30', "Pi/6 (30°)", ""), ('45', "Pi/4 (45°)", ""),
            ('60', "Pi/3 (60°)", ""), ('90', "Pi/2 (90°)", ""),
        ],
        default='15'
    )

    use_angle_snap: bpy.props.BoolProperty(name="Use Angle Snap", default=True)
    use_radians: bpy.props.BoolProperty(name="Use Radians", default=False)
    
    snap_strength: bpy.props.FloatProperty(
        name="Snap Strength", 
        default=6.0, min=0.1, max=45.0,
        description="Think of this as how 'sticky' the angle snapping feels"
    )

    snap_to_verts: bpy.props.BoolProperty(name="Vertices", default=True)
    snap_to_edges: bpy.props.BoolProperty(name="Edges", default=True)
    snap_to_faces: bpy.props.BoolProperty(name="Faces", default=True)

    display_precision: bpy.props.IntProperty(
        name="Display Precision", default=3, min=0, max=9
    )

    weld_radius: bpy.props.FloatProperty(
        name="Weld Radius", default=0.001, precision=5, min=0.00001, max=1.0
    )
    weld_to_faces: bpy.props.BoolProperty(name="Cut Faces (Knife Project)", default=True)

    lift_compass: bpy.props.FloatProperty(name="Compass Lift (Ortho)", default=4.0, min=0.0, max=500.0)
    lift_arc: bpy.props.FloatProperty(name="Arc Line Lift (Ortho)", default=20.0, min=0.0, max=5000.0)
    lift_perspective: bpy.props.FloatProperty(name="Perspective Bias (%)", default=0.2, min=0.0, max=10.0, precision=3)

    # =========================================================================
    # DRAWING HELPERS
    # =========================================================================
    def draw_section_header(self, layout, title, prop_name, icon='NONE', tool_key=None):
        """Draws a collapsible box header with optional SVG icon support."""
        box = layout.box()
        row = box.row(align=True)
        
        is_expanded = getattr(self, prop_name)
        icon_state = "TRIA_DOWN" if is_expanded else "TRIA_RIGHT"
        
        row.prop(self, prop_name, icon=icon_state, text="", icon_only=True, emboss=False)
        
        # Try to get SVG icon from panel
        icon_val = 0
        if tool_key:
            try:
                from . import panel
                pcoll = getattr(panel, "preview_collection", None)
                if pcoll and tool_key in pcoll:
                    icon_val = pcoll[tool_key].icon_id
            except ImportError:
                pass
        
        if icon_val:
            row.label(text=title, icon_value=icon_val)
        else:
            row.label(text=title, icon=icon)
        
        if is_expanded:
            return box.column(align=True)
        return None

    def draw_group_label(self, col, text, icon='NONE'):
        """Indented once group label."""
        row = col.row()
        row.separator(factor=2.0)
        row.label(text=text, icon=icon)

    def draw_property_row(self, col, label, prop_name, icon='BLANK1', enabled=True):
        """Twice indented property row."""
        split = col.split(factor=0.5, align=True)
        row_l = split.row()
        row_l.separator(factor=4.0)
        row_l.enabled = enabled
        row_l.label(text=label, icon=icon)
        row_p = split.row()
        row_p.enabled = enabled
        row_p.prop(self, prop_name, text="")

    def draw(self, context):
        layout = self.layout
        
        # 1. GLOBAL SETTINGS
        col = self.draw_section_header(layout, "Global Settings", "show_global_settings", icon='PREFERENCES')
        if col:
            row_save = col.row()
            row_save.scale_y = 1.5
            row_save.operator("wm.save_userpref", text="Save Preferences", icon='FILE_TICK')
            
            col.separator()
            self.draw_group_label(col, "Weld / Auto-Connect:", icon='AUTOMERGE_ON')
            self.draw_property_row(col, "Search Radius (Magnet):", "weld_radius")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Geometry Snaps:", icon='SNAP_PEEL_OBJECT')
            self.draw_property_row(col, "Snap Strength:", "snap_strength")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Axis Snapping:", icon='COLOR')
            self.draw_property_row(col, "Axis Color Dimmer:", "axis_color_dim")

            col.separator(factor=2.0)
            self.draw_group_label(col, "Z-Fighting Tweaks", icon='OPTIONS')
            # Custom split for multiple props in one row
            split = col.split(factor=0.5, align=True)
            row_l = split.row()
            row_l.separator(factor=4.0)
            row_l.label(text="Ortho Lifts (Compass/Arc):", icon='BLANK1')
            sub = split.row(align=True)
            sub.prop(self, "lift_compass", text="Compass")
            sub.prop(self, "lift_arc", text="Arc")
            self.draw_property_row(col, "Perspective Lift %:", "lift_perspective")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Hotkeys Helper:", icon='HELP')
            self.draw_property_row(col, "Show Panel:", "show_hotkeys")
            if self.show_hotkeys:
                split = col.split(factor=0.5, align=True)
                row_l = split.row()
                row_l.separator(factor=4.0)
                row_l.label(text="Screen Position:", icon='BLANK1')
                sub = split.row(align=True)
                sub.prop(self, "hotkeys_offset_x", text="X")
                sub.prop(self, "hotkeys_offset_y", text="Y")

            col.separator(factor=2.0)
            self.draw_group_label(col, "Metric Display:", icon='DOT')
            self.draw_property_row(col, "Decimal Precision:", "display_precision")

        # 2. POINTS BY ARC
        col = self.draw_section_header(layout, "Points by Arc Settings", "show_points_by_arc_settings", icon='GP_SELECT_POINTS', tool_key='point_by_arcs')
        if col:
            self.draw_group_label(col, "Visual Colors:", icon='COLOR')
            self.draw_property_row(col, "Arc 1 Color:", "color_points_by_arc_1")
            self.draw_property_row(col, "Arc 2 Color:", "color_points_by_arc_2")
            self.draw_property_row(col, "Start Line Color:", "color_points_by_arc_start")
            self.draw_property_row(col, "End Line Color:", "color_points_by_arc_end")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Marker Sizes:", icon='SNAP_ON')
            self.draw_property_row(col, "Crosshair Size (Setup):", "points_by_arc_crosshair_size")
            self.draw_property_row(col, "Intersection Square (Final):", "points_by_arc_square_size")

        # 3. LINE
        col = self.draw_section_header(layout, "Line Settings", "show_line_settings", icon='LINCURVE', tool_key='line')
        if col:
            self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
            self.draw_property_row(col, "Use Axis Colors:", "use_axis_colors")

        # 4-7. Curve based tools (excluding tan3)
        tools = [
            ("perp", "Line Perpendicular from Curve Settings", "show_line_perp_settings", "line_perpendicular_from_curve", "line_perp"),
            ("perp2", "Line Perpendicular to Two Curves Settings", "show_line_perp2_settings", "line_perpendicular_to_two_curves", "line_perp2"),
            ("tangent", "Line Tangent from Curve Settings", "show_line_tangent_settings", "line_tangent_from_curve", "line_tangent"),
            ("tan_tan", "Line Tangent to Two Curves Settings", "show_line_tan_tan_settings", "line_tangent_to_two_curves", "line_tan_tan")
        ]
        for key, title, show_prop, tool_key, prop_prefix in tools:
            col = self.draw_section_header(layout, title, show_prop, icon='CURVE_NCURVE', tool_key=tool_key)
            if col:
                self.draw_group_label(col, "Catmull Overlay Settings:", icon='COLOR')
                self.draw_property_row(col, "Show Overlay:", f"{prop_prefix}_show_catmull")
                self.draw_property_row(col, "Overlay Color:", f"{prop_prefix}_col_catmull")
                self.draw_property_row(col, "Overlay Thickness:", f"{prop_prefix}_width_catmull")

        # 8. 1 POINT ARC
        col = self.draw_section_header(layout, "1 Point Arc Settings", "show_arc_settings", icon='CURVE_DATA', tool_key='arc_1_point')
        if col:
            self.draw_group_label(col, "Display & Fonts:", icon='FONT_DATA')
            self.draw_property_row(col, "Display Compass Size:", "compass_size")
            self.draw_property_row(col, "Hotkey Font Size:", "font_size_hotkey")
            self.draw_property_row(col, "Label/Param Font Size:", "font_size_label")
            self.draw_property_row(col, "Preview Vertex Size:", "preview_vertex_size")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Angle Snap:", icon='DRIVER_ROTATIONAL_DIFFERENCE')
            self.draw_property_row(col, "Increment:", "angle_snap_type_rad" if self.use_radians else "angle_snap_type")
            self.draw_property_row(col, "Angle Units:", "use_radians")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Snap Guides:", icon='SNAP_ON')
            split = col.split(factor=0.5, align=True)
            row_l = split.row(); row_l.separator(factor=4.0)
            row_l.label(text="Marker (Size/Color):", icon='BLANK1')
            sub = split.row(align=True)
            sub.prop(self, "snap_marker_size", text="")
            sub.prop(self, "snap_marker_color", text="")
            self.draw_property_row(col, "Pointer Line Color:", "snap_line_color")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Overlay Position:", icon='OVERLAY')
            split = col.split(factor=0.5, align=True)
            row_l = split.row(); row_l.separator(factor=4.0)
            row_l.label(text="Offset (Compass):", icon='BLANK1')
            sub = split.row(align=True)
            sub.prop(self, "overlay_offset_x", text="X"); sub.prop(self, "overlay_offset_y", text="Y")
            
            col.separator(factor=2.0) 
            self.draw_group_label(col, "Visuals (Preview Lines):", icon='COLOR')
            self.draw_property_row(col, "Start Line Color:", "color_arc_start")
            self.draw_property_row(col, "End Line Color:", "color_arc_end")

        # 9-12. 2/3 Point Arcs and Circles
        arc_circle_tools = [
            ("2pt", "2 Point Arc Settings", "show_arc_2pt_settings", "arc_2_point", "arc_2pt"),
            ("3pt", "3 Point Arc Settings", "show_arc_3pt_settings", "arc_3_point", "arc_3pt"),
            ("c2pt", "2 Point Circle Settings", "show_circle_2pt_settings", "circle_2_points", "circle_2pt"),
            ("c3pt", "3 Point Circle Settings", "show_circle_3pt_settings", "circle_3_points", "circle_3pt")
        ]
        for key, title, show_prop, tool_key, prefix in arc_circle_tools:
            col = self.draw_section_header(layout, title, show_prop, icon='CURVE_DATA' if 'Arc' in title else 'MESH_CIRCLE', tool_key=tool_key)
            if col:
                self.draw_group_label(col, "Snap Guides:", icon='SNAP_ON')
                split = col.split(factor=0.5, align=True)
                row_l = split.row(); row_l.separator(factor=4.0)
                row_l.label(text="Marker (Size/Color):", icon='BLANK1')
                sub = split.row(align=True)
                sub.prop(self, f"snap_marker_size_{key}", text="")
                sub.prop(self, f"snap_marker_color_{key}", text="")
                self.draw_property_row(col, "Pointer Line Color:", f"snap_line_color_{key}")
                
                col.separator(factor=2.0)
                self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
                self.draw_property_row(col, "Use Axis Colors:", f"{prefix}_use_axis_colors")
                self.draw_property_row(col, "Overlay Color:", f"color_{prefix}_overlay", enabled=not getattr(self, f"{prefix}_use_axis_colors"))

        # 13. CIRCLE TANGENT TO THREE CURVES (Moved under 3 Point Circle)
        col = self.draw_section_header(layout, "Circle Tangent to Three Curves Settings", "show_circle_tan3_settings", icon='CURVE_NCURVE', tool_key='circle_tangent_to_three_curves')
        if col:
            self.draw_group_label(col, "Curve Overlays:", icon='COLOR')
            self.draw_property_row(col, "Show Curves:", "circle_tan3_show_curves")
            self.draw_property_row(col, "Curve Overlay Color:", "circle_tan3_col_curves")
            self.draw_property_row(col, "Curve Overlay Thickness:", "circle_tan3_width_curves")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Tangent Circle:", icon='MESH_CIRCLE')
            self.draw_property_row(col, "Show Tangent:", "circle_tan3_show_tangent")
            self.draw_property_row(col, "Tangent Circle Color:", "circle_tan3_col_tangent")
            self.draw_property_row(col, "Preview Geometry Color:", "circle_tan3_col_preview")
            self.draw_property_row(col, "Preview Vertex Size:", "preview_vertex_size")
            self.draw_property_row(col, "Tangent Circle Thickness:", "circle_tan3_width_tangent")

        # 14. CIRCLE TANGENT TO TWO CURVES (Moved under Tan 3)
        col = self.draw_section_header(layout, "Circle Tangent to Two Curves Settings", "show_circle_tan2_settings", icon='CURVE_NCURVE', tool_key='circle_tangent_to_two_curves')
        if col:
            self.draw_group_label(col, "Curve Overlays:", icon='COLOR')
            self.draw_property_row(col, "Show Curves:", "circle_tan2_show_curves")
            self.draw_property_row(col, "Curve Overlay Color:", "circle_tan2_col_curves")
            self.draw_property_row(col, "Curve Overlay Thickness:", "circle_tan2_width_curves")
            
            col.separator(factor=2.0)
            self.draw_group_label(col, "Tangent Circle:", icon='MESH_CIRCLE')
            self.draw_property_row(col, "Show Tangent:", "circle_tan2_show_tangent")
            self.draw_property_row(col, "Tangent Circle Color:", "circle_tan2_col_tangent")
            self.draw_property_row(col, "Preview Geometry Color:", "circle_tan2_col_preview")
            self.draw_property_row(col, "Tangent Circle Thickness:", "circle_tan2_width_tangent")

        # 15. ELLIPSE FROM FOCI POINTS
        col = self.draw_section_header(layout, "Ellipse from Foci Points Settings", "show_ellipse_foci_settings", icon='CURVE_NCURVE', tool_key='ellipse_foci_point')
        if col:
            self.draw_group_label(col, "Visuals:", icon='COLOR')
            self.draw_property_row(col, "Foci Line Color:", "ellipse_foci_col_foci_lines")

            col.separator(factor=2.0)
            self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
            self.draw_property_row(col, "Use Axis Colors:", "ellipse_foci_use_axis_colors")
            self.draw_property_row(col, "Overlay Color:", "color_ellipse_foci_overlay", enabled=not self.ellipse_foci_use_axis_colors)

        # 16. Ellipse Corners
        col = self.draw_section_header(layout, "Ellipse (From Corners) Settings", "show_ellipse_corners_settings", icon='CURVE_NCURVE', tool_key='ellipse_corners')
        if col:
            self.draw_group_label(col, "Visuals:", icon='COLOR')
            self.draw_property_row(col, "Corners Color:", "ellipse_corners_color")

        # 17-18. Ellipse Tools (Endpoints & Radius with Snapping Visuals)
        ellipse_tools = [
            ("endpoints", "Ellipse (From Endpoints) Settings", "show_ellipse_endpoints_settings", "ellipse_from_radius"),
            ("radius", "Ellipse (From Radius) Settings", "show_ellipse_radius_settings", "ellipse_from_radius")
        ]
        for prop_prefix, title, show_prop, tool_key in ellipse_tools:
            col = self.draw_section_header(layout, title, show_prop, icon='CURVE_NCURVE', tool_key=tool_key)
            if col:
                self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
                self.draw_property_row(col, "Use Axis Colors:", f"ellipse_{prop_prefix}_use_axis_colors")
                self.draw_property_row(col, "Overlay Color:", f"color_ellipse_{prop_prefix}_overlay", enabled=not getattr(self, f"ellipse_{prop_prefix}_use_axis_colors"))

        # 20-23. Polygon Tools (with Snapping Visuals)
        polygon_tools = [
            ("center_corner", "Polygon (Center/Corner) Settings", "show_polygon_settings", "polygon_cen_cor"),
            ("center_tangent", "Polygon (Center/Tangent) Settings", "show_polygon_settings", "polygon_cen_tan"),
            ("corner_corner", "Polygon (Corner/Corner) Settings", "show_polygon_settings", "polygon_cor_cor"),
            ("edge", "Polygon (Edge) Settings", "show_polygon_settings", "polygon_edge")
        ]
        for prop_prefix, title, show_prop, tool_key in polygon_tools:
            col = self.draw_section_header(layout, title, show_prop, icon='MESH_CIRCLE', tool_key=tool_key)
            if col:
                self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
                self.draw_property_row(col, "Use Axis Colors:", f"polygon_{prop_prefix}_use_axis_colors")
                self.draw_property_row(col, "Overlay Color:", f"color_polygon_{prop_prefix}_overlay", enabled=not getattr(self, f"polygon_{prop_prefix}_use_axis_colors"))

        # 24-26. Rectangle Tools (with Snapping Visuals)
        rectangle_tools = [
            ("center_corner", "Rectangle (Center/Corner) Settings", "show_rectangle_settings", "rectangle_center_corner"),
            ("corner_corner", "Rectangle (Corner/Corner) Settings", "show_rectangle_settings", "rectangle_corner_corner"),
            ("3pt", "Rectangle (3 Points) Settings", "show_rectangle_settings", "rectangle_3pt")
        ]
        for prop_prefix, title, show_prop, tool_key in rectangle_tools:
            col = self.draw_section_header(layout, title, show_prop, icon='MESH_PLANE', tool_key=tool_key)
            if col:
                self.draw_group_label(col, "Snapping Visuals:", icon='COLOR')
                self.draw_property_row(col, "Use Axis Colors:", f"rectangle_{prop_prefix}_use_axis_colors")
                self.draw_property_row(col, "Overlay Color:", f"color_rectangle_{prop_prefix}_overlay", enabled=not getattr(self, f"rectangle_{prop_prefix}_use_axis_colors"))

        # 27. Curve Settings
        self.draw_section_header(layout, "Curve Settings", "show_curve_settings", icon='CURVE_BEZCURVE', tool_key='curve_interpolate_points')

def register():
    bpy.utils.register_class(RADCAD_Preferences)

def unregister():
    bpy.utils.unregister_class(RADCAD_Preferences)
