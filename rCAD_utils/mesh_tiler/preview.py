import bpy
import bmesh
import mathutils
import gpu
from gpu_extras.batch import batch_for_shader
from . import find_selected_islands

_state = {"planes": [], "handle": None}


def _get_island_bounds(bm, island_indices, mw):
    coords = [mw @ bm.verts[i].co for i in island_indices]
    return (
        mathutils.Vector((min(c.x for c in coords), min(c.y for c in coords), min(c.z for c in coords))),
        mathutils.Vector((max(c.x for c in coords), max(c.y for c in coords), max(c.z for c in coords))),
    )


def _find_preview_planes(all_bounds):
    """Check every pair. Intersection shape decides cut direction."""
    planes = []
    for i in range(len(all_bounds)):
        for j in range(i + 1, len(all_bounds)):
            b1, b2 = all_bounds[i], all_bounds[j]

            ix0 = max(b1[0].x, b2[0].x); ix1 = min(b1[1].x, b2[1].x)
            iy0 = max(b1[0].y, b2[0].y); iy1 = min(b1[1].y, b2[1].y)
            iz0 = max(b1[0].z, b2[0].z); iz1 = min(b1[1].z, b2[1].z)

            if ix0 > ix1 + 0.001 or iy0 > iy1 + 0.001 or iz0 > iz1 + 0.001:
                continue

            dx = abs((b1[0].x + b1[1].x) / 2 - (b2[0].x + b2[1].x) / 2)
            dz = abs((b1[0].z + b1[1].z) / 2 - (b2[0].z + b2[1].z) / 2)

            merged_mn = mathutils.Vector((min(b1[0].x, b2[0].x), min(b1[0].y, b2[0].y), min(b1[0].z, b2[0].z)))
            merged_mx = mathutils.Vector((max(b1[1].x, b2[1].x), max(b1[1].y, b2[1].y), max(b1[1].z, b2[1].z)))

            x_overlap = max(0, ix1 - ix0)
            z_overlap = max(0, iz1 - iz0)
            min_width = min(b1[1].x - b1[0].x, b2[1].x - b2[0].x)
            min_height = min(b1[1].z - b1[0].z, b2[1].z - b2[0].z)

            if dx >= dz:
                # Side by side → vertical plane (skip diagonal)
                if z_overlap < min_height * 0.5:
                    continue
                cx = (ix0 + ix1) / 2
                planes.append(_quad_verts(cx, merged_mn, merged_mx))
            else:
                # Stacked → horizontal plane (skip diagonal)
                if x_overlap < min_width * 0.5:
                    continue
                cz = (iz0 + iz1) / 2
                planes.append(_quad_verts_hz(cz, merged_mn, merged_mx))
    return planes


def _quad_verts(cx, mn, mx):
    """YZ-plane quad at x=cx, spanning the full bounds."""
    y0, y1 = mn.y - 0.05, mx.y + 0.05
    z0, z1 = mn.z - 0.05, mx.z + 0.05
    return [
        mathutils.Vector((cx, y0, z0)),
        mathutils.Vector((cx, y1, z0)),
        mathutils.Vector((cx, y1, z1)),
        mathutils.Vector((cx, y0, z1)),
    ]


def _quad_verts_hz(cz, mn, mx):
    """XY-plane quad at z=cz, spanning the full bounds."""
    x0, x1 = mn.x - 0.05, mx.x + 0.05
    y0, y1 = mn.y - 0.05, mx.y + 0.05
    return [
        mathutils.Vector((x0, y0, cz)),
        mathutils.Vector((x1, y0, cz)),
        mathutils.Vector((x1, y1, cz)),
        mathutils.Vector((x0, y1, cz)),
    ]


def _draw_callback():
    if not _state["planes"]:
        return
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('LESS_EQUAL')
    for verts in _state["planes"]:
        batch = batch_for_shader(shader, 'TRIS', {"pos": verts}, indices=[(0, 1, 2), (0, 2, 3)])
        shader.bind()
        shader.uniform_float("color", (1.0, 0.65, 0.0, 0.25))
        batch.draw(shader)
        outline = verts + [verts[0]]
        batch_line = batch_for_shader(shader, 'LINE_STRIP', {"pos": outline})
        shader.uniform_float("color", (1.0, 0.65, 0.0, 0.9))
        batch_line.draw(shader)
    gpu.state.blend_set('NONE')
    gpu.state.depth_test_set('NONE')


def _remove_handle(context):
    if _state["handle"]:
        bpy.types.SpaceView3D.draw_handler_remove(_state["handle"], 'WINDOW')
        _state["handle"] = None
        _state["planes"] = []
    context.area.tag_redraw()


class MESH_OT_MeshTilerPreview(bpy.types.Operator):
    bl_idname = "mesh.mesh_tiler_preview"
    bl_label = "Preview"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        if _state["handle"]:
            _remove_handle(context)
            return {'FINISHED'}

        obj = context.active_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        islands = find_selected_islands(bm)
        if len(islands) < 2:
            self.report({'ERROR'}, "Need 2+ islands.")
            return {'CANCELLED'}

        mw = obj.matrix_world
        all_bounds = [_get_island_bounds(bm, island, mw) for island in islands]

        planes = _find_preview_planes(all_bounds)
        _state["planes"] = planes
        _state["handle"] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_callback, (), 'WINDOW', 'POST_VIEW'
        )
        context.window_manager.modal_handler_add(self)
        context.area.tag_redraw()

        if planes:
            self.report({'INFO'}, f"{len(planes)} cut plane(s) — Esc to dismiss")
        else:
            self.report({'WARNING'}, "No overlapping islands found")

        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        context.area.tag_redraw()
        if event.type in {'ESC', 'RIGHTMOUSE'} and event.value == 'PRESS':
            _remove_handle(context)
            return {'CANCELLED'}
        return {'PASS_THROUGH'}
