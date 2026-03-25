import bpy
import bmesh
import mathutils
import gpu
from gpu_extras.batch import batch_for_shader

bl_info = {
    "name": "Intersection Finder",
    "blender": (3, 0, 0),
    "category": "Mesh",
    "version": (1, 0, 0),
    "author": "rCAD",
}

_state = {"plane_verts": None, "handle": None}


def get_world_bounds(obj):
    """Get world-space bounding box of an object."""
    corners = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
    min_x = min(c.x for c in corners)
    min_y = min(c.y for c in corners)
    min_z = min(c.z for c in corners)
    max_x = max(c.x for c in corners)
    max_y = max(c.y for c in corners)
    max_z = max(c.z for c in corners)
    return mathutils.Vector((min_x, min_y, min_z)), mathutils.Vector((max_x, max_y, max_z))


def find_intersection_plane(obj1, obj2):
    """Find the intersection plane between two overlapping meshes."""
    min1, max1 = get_world_bounds(obj1)
    min2, max2 = get_world_bounds(obj2)

    # Calculate intersection bounds
    ix0 = max(min1.x, min2.x)
    ix1 = min(max1.x, max2.x)
    iy0 = max(min1.y, min2.y)
    iy1 = min(max1.y, max2.y)
    iz0 = max(min1.z, min2.z)
    iz1 = min(max1.z, max2.z)

    # Check if they actually overlap
    if ix0 > ix1 + 0.001 or iy0 > iy1 + 0.001 or iz0 > iz1 + 0.001:
        return None, None, None

    # Center of intersection
    center = mathutils.Vector(((ix0 + ix1) / 2, (iy0 + iy1) / 2, (iz0 + iz1) / 2))

    # Determine cut direction based on center offsets
    dx = abs((min1.x + max1.x) / 2 - (min2.x + max2.x) / 2)
    dz = abs((min1.z + max1.z) / 2 - (min2.z + max2.z) / 2)

    x_overlap = max(0, ix1 - ix0)
    z_overlap = max(0, iz1 - iz0)
    min_width = min(max1.x - min1.x, max2.x - min2.x)
    min_height = min(max1.z - min1.z, max2.z - min2.z)

    if dx >= dz:
        # Vertical cut (side by side)
        if z_overlap < min_height * 0.5:
            return None, None, None
        normal = mathutils.Vector((1, 0, 0))
        plane_verts = _quad_verts_vertical(center.x, min1, min2, max1, max2)
    else:
        # Horizontal cut (stacked)
        if x_overlap < min_width * 0.5:
            return None, None, None
        normal = mathutils.Vector((0, 0, 1))
        plane_verts = _quad_verts_horizontal(center.z, min1, min2, max1, max2)

    return center, normal, plane_verts


def _quad_verts_vertical(cx, min1, min2, max1, max2):
    """YZ-plane quad for vertical cut."""
    merged_min = mathutils.Vector((
        min(min1.x, min2.x),
        min(min1.y, min2.y),
        min(min1.z, min2.z)
    ))
    merged_max = mathutils.Vector((
        max(max1.x, max2.x),
        max(max1.y, max2.y),
        max(max1.z, max2.z)
    ))

    y0, y1 = merged_min.y - 0.05, merged_max.y + 0.05
    z0, z1 = merged_min.z - 0.05, merged_max.z + 0.05
    return [
        mathutils.Vector((cx, y0, z0)),
        mathutils.Vector((cx, y1, z0)),
        mathutils.Vector((cx, y1, z1)),
        mathutils.Vector((cx, y0, z1)),
    ]


def _quad_verts_horizontal(cz, min1, min2, max1, max2):
    """XY-plane quad for horizontal cut."""
    merged_min = mathutils.Vector((
        min(min1.x, min2.x),
        min(min1.y, min2.y),
        min(min1.z, min2.z)
    ))
    merged_max = mathutils.Vector((
        max(max1.x, max2.x),
        max(max1.y, max2.y),
        max(max1.z, max2.z)
    ))

    x0, x1 = merged_min.x - 0.05, merged_max.x + 0.05
    y0, y1 = merged_min.y - 0.05, merged_max.y + 0.05
    return [
        mathutils.Vector((x0, y0, cz)),
        mathutils.Vector((x1, y0, cz)),
        mathutils.Vector((x1, y1, cz)),
        mathutils.Vector((x0, y1, cz)),
    ]


def _draw_callback():
    """Draw the intersection plane overlay."""
    if _state["plane_verts"] is None:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('LESS_EQUAL')

    verts = _state["plane_verts"]

    # Draw filled plane
    batch = batch_for_shader(shader, 'TRIS', {"pos": verts}, indices=[(0, 1, 2), (0, 2, 3)])
    shader.bind()
    shader.uniform_float("color", (0.2, 1.0, 0.2, 0.3))  # Green semi-transparent
    batch.draw(shader)

    # Draw outline
    outline = verts + [verts[0]]
    batch_line = batch_for_shader(shader, 'LINE_STRIP', {"pos": outline})
    shader.uniform_float("color", (0.0, 1.0, 0.0, 1.0))  # Bright green
    batch_line.draw(shader)

    gpu.state.blend_set('NONE')
    gpu.state.depth_test_set('NONE')


def remove_overlay():
    """Remove the draw handler and clear state."""
    if _state["handle"] is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_state["handle"], 'WINDOW')
        _state["handle"] = None
    _state["plane_verts"] = None


class MESH_OT_FindIntersection(bpy.types.Operator):
    bl_idname = "mesh.find_intersection"
    bl_label = "Find Intersection Plane"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def execute(self, context):
        remove_overlay()

        # Multi-object Edit Mode: get all objects being edited
        selected = list(context.objects_in_mode)
        if len(selected) < 2:
            self.report({'ERROR'}, f"Select 2 meshes and enter Edit Mode (found {len(selected)})")
            return {'CANCELLED'}

        obj1, obj2 = selected[0], selected[1]
        center, normal, plane_verts = find_intersection_plane(obj1, obj2)

        if plane_verts is None:
            self.report({'ERROR'}, "Meshes don't overlap or are diagonal")
            return {'CANCELLED'}

        _state["plane_verts"] = plane_verts
        _state["handle"] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_callback, (), 'WINDOW', 'POST_VIEW'
        )

        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

        self.report({'INFO'}, f"Intersection plane at ({center.x:.2f}, {center.y:.2f}, {center.z:.2f})")
        return {'FINISHED'}


class VIEW3D_PT_IntersectionPanel(bpy.types.Panel):
    bl_label = "Intersection Finder"
    bl_idname = "VIEW3D_PT_intersection"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Intersect'

    def draw(self, context):
        layout = self.layout
        layout.operator("mesh.find_intersection", text="Find Intersection Plane")


def register():
    bpy.utils.register_class(MESH_OT_FindIntersection)
    bpy.utils.register_class(VIEW3D_PT_IntersectionPanel)


def unregister():
    bpy.utils.unregister_class(MESH_OT_FindIntersection)
    bpy.utils.unregister_class(VIEW3D_PT_IntersectionPanel)


if __name__ == "__main__":
    register()
