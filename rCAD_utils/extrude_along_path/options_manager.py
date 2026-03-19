import bpy

def register_options():
    wm = bpy.types.WindowManager
    
    wm.eap_is_advanced_mode = bpy.props.BoolProperty(
        name="Is Advanced Mode", 
        default=True
    )
    
    wm.eap_extrusion_type_poc = bpy.props.EnumProperty(
        name="Extrusion Type",
        items=[
            ('faces', 'Faces', 'Create faces between profile edges (default behavior)'),
            ('edges', 'Edges', 'Create vertices connected by edges along the path'),
            ('profiles', 'Profiles', 'Maintain exact connectivity of original selection'),
        ],
        default='faces'
    )

def unregister_options():
    wm = bpy.types.WindowManager
    del wm.eap_is_advanced_mode
    del wm.eap_extrusion_type_poc