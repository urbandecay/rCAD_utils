"""Python-first rendered previews for shader nodes."""

from collections import deque
import os
import tempfile

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty
from mathutils import Vector


PROPERTY_NAME = "rcad_node_preview_enabled"
PREVIEW_IMAGE_PREFIX = "rCAD Node Preview "
PREVIEW_RESOLUTION = 128

_draw_handle = None
_preview_timer_running = False
_preview_jobs = deque()
_queued_jobs = set()
_preview_cache = {}
_preview_scene = None
_preview_object = None
_preview_shader = None
_image_shader = None
_gpu_texture_cache = {}


def _safe_pointer(datablock):
    try:
        return datablock.as_pointer()
    except (AttributeError, ReferenceError, RuntimeError):
        return 0


def _shader_editor_contexts():
    """Yield open Shader Editor areas with their currently edited tree."""
    window_manager = getattr(bpy.context, "window_manager", None)
    if window_manager is None:
        return

    for window in tuple(window_manager.windows):
        screen = getattr(window, "screen", None)
        if screen is None:
            continue
        for area in tuple(screen.areas):
            if area.type != 'NODE_EDITOR':
                continue
            space = area.spaces.active
            if space.tree_type != 'ShaderNodeTree':
                continue
            tree = getattr(space, "edit_tree", None)
            if tree is None:
                tree = getattr(space, "node_tree", None)
            if tree is None:
                continue
            yield area, space, tree


def _material_for_space(space, tree=None):
    """Return the material represented by a Shader Editor space."""
    node_id = getattr(space, "id", None)
    if isinstance(node_id, bpy.types.Material):
        return node_id

    # Blender versions differ on whether Shader Editor ``id`` is the
    # material, object, or the source ID. Check all of those paths.
    view_layer = getattr(bpy.context, "view_layer", None)
    view_layer_objects = getattr(view_layer, "objects", None)
    active_object = getattr(view_layer_objects, "active", None)
    candidates = (
        node_id,
        getattr(space, "id_from", None),
        active_object,
    )
    for candidate in candidates:
        if isinstance(candidate, bpy.types.Material):
            return candidate
        if isinstance(candidate, bpy.types.Object):
            materials = []
            active_material = candidate.active_material
            if active_material is not None:
                materials.append(active_material)
            materials.extend(
                slot.material
                for slot in candidate.material_slots
                if slot.material is not None and slot.material not in materials
            )
            for material in materials:
                if tree is None or material.node_tree == tree:
                    return material
    return None


def _value_signature(value):
    """Convert common Blender values into a stable, hashable signature."""
    if isinstance(value, (str, int, float, bool, type(None))):
        if isinstance(value, float):
            return round(value, 7)
        return value
    try:
        return tuple(_value_signature(item) for item in value)
    except TypeError:
        return repr(value)


def _tree_signature(tree):
    """Return a lightweight revision signature for a shader node tree."""
    node_data = []
    for node in sorted(tree.nodes, key=lambda item: item.name):
        inputs = []
        for socket in node.inputs:
            links = tuple(
                (
                    link.from_node.name,
                    link.from_socket.name,
                    link.to_socket.name,
                )
                for link in socket.links
            )
            default = None
            if not socket.is_linked and hasattr(socket, "default_value"):
                try:
                    default = _value_signature(socket.default_value)
                except (AttributeError, RuntimeError):
                    default = None
            inputs.append((socket.name, links, default))

        image = getattr(node, "image", None)
        node_data.append(
            (
                node.name,
                node.bl_idname,
                node.label,
                node.mute,
                _safe_pointer(image),
                tuple(inputs),
                _value_signature(getattr(node, "operation", None)),
                _value_signature(getattr(node, "blend_type", None)),
                _value_signature(getattr(node, "noise_dimensions", None)),
            )
        )

    links = tuple(
        (
            link.from_node.name,
            link.from_socket.name,
            link.to_node.name,
            link.to_socket.name,
        )
        for link in tree.links
    )
    return hash(repr((tuple(node_data), links)))


def _is_previewable_node(node):
    if node.type in {'FRAME', 'REROUTE', 'OUTPUT_MATERIAL'}:
        return False
    return bool(node.outputs)


def _cache_key(material, tree, node):
    return (
        _safe_pointer(material),
        _safe_pointer(tree),
        _safe_pointer(node),
    )


def _remove_preview_image(image):
    if image is None:
        return
    _gpu_texture_cache.pop(_safe_pointer(image), None)
    try:
        if image.name in bpy.data.images:
            bpy.data.images.remove(image)
    except (ReferenceError, RuntimeError):
        pass


def _clear_preview_cache():
    _preview_jobs.clear()
    _queued_jobs.clear()
    for entry in tuple(_preview_cache.values()):
        _remove_preview_image(entry.get("image"))
    _preview_cache.clear()
    _gpu_texture_cache.clear()


def _look_at(object_data, target):
    direction = Vector(target) - object_data.location
    object_data.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def _ensure_preview_scene():
    """Create the private scene used for Python-side thumbnail rendering."""
    global _preview_scene, _preview_object
    if _preview_scene is not None and _preview_scene.name in bpy.data.scenes:
        return _preview_scene

    scene = bpy.data.scenes.new("rCAD Node Preview Scene")
    try:
        scene.render.engine = 'BLENDER_EEVEE_NEXT'
    except (TypeError, ValueError):
        try:
            scene.render.engine = 'BLENDER_EEVEE'
        except (TypeError, ValueError):
            pass
    scene.render.resolution_x = PREVIEW_RESOLUTION
    scene.render.resolution_y = PREVIEW_RESOLUTION
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = 'PNG'

    world = bpy.data.worlds.new("rCAD Node Preview World")
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    if background is not None:
        background.inputs["Color"].default_value = (0.035, 0.035, 0.035, 1.0)
        background.inputs["Strength"].default_value = 0.35
    scene.world = world

    # The original Node Preview scene uses a flat, UV-mapped surface. This
    # keeps Image Texture and color outputs readable instead of projecting
    # them onto a generic shaded sphere.
    mesh = bpy.data.meshes.new("rCAD Node Preview Plane Mesh")
    mesh.from_pydata(
        [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0),
         (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)],
        [],
        [(0, 1, 2, 3)],
    )
    mesh.update()
    uv_layer = mesh.uv_layers.new(name="UVMap")
    uv_coordinates = {
        0: (0.0, 0.0),
        1: (1.0, 0.0),
        2: (1.0, 1.0),
        3: (0.0, 1.0),
    }
    for loop in mesh.loops:
        uv_layer.data[loop.index].uv = uv_coordinates[loop.vertex_index]

    preview_object = bpy.data.objects.new("rCAD Node Preview Plane", mesh)
    scene.collection.objects.link(preview_object)

    camera_data = bpy.data.cameras.new("rCAD Node Preview Camera")
    camera = bpy.data.objects.new("rCAD Node Preview Camera", camera_data)
    scene.collection.objects.link(camera)
    camera_data.type = 'ORTHO'
    camera_data.ortho_scale = 2.08
    camera.location = (0.0, 0.0, 3.7)
    _look_at(camera, (0.0, 0.0, 0.0))
    scene.camera = camera

    light_data = bpy.data.lights.new("rCAD Node Preview Key", 'AREA')
    light_data.energy = 450
    light_data.shape = 'DISK'
    light_data.size = 4.0
    key_light = bpy.data.objects.new("rCAD Node Preview Key", light_data)
    key_light.location = (0.0, 0.0, 4.5)
    _look_at(key_light, (0.0, 0.0, 0.0))
    scene.collection.objects.link(key_light)

    fill_data = bpy.data.lights.new("rCAD Node Preview Fill", 'AREA')
    fill_data.energy = 180
    fill_data.size = 5.0
    fill_light = bpy.data.objects.new("rCAD Node Preview Fill", fill_data)
    fill_light.location = (-2.0, 1.0, 2.0)
    _look_at(fill_light, (0.0, 0.0, 0.0))
    scene.collection.objects.link(fill_light)

    _preview_scene = scene
    _preview_object = preview_object
    return scene


def _destroy_preview_scene():
    global _preview_scene, _preview_object
    scene = _preview_scene
    _preview_scene = None
    _preview_object = None
    if scene is None:
        return

    objects = tuple(scene.objects)
    world = scene.world
    for object_data in objects:
        object_type = object_data.type
        data = object_data.data
        bpy.data.objects.remove(object_data, do_unlink=True)
        if data is not None and getattr(data, "users", 0) == 0:
            collection = {
                'MESH': bpy.data.meshes,
                'CAMERA': bpy.data.cameras,
                'LIGHT': bpy.data.lights,
            }.get(object_type)
            if collection is not None:
                try:
                    collection.remove(data)
                except (ReferenceError, RuntimeError):
                    pass
    bpy.data.scenes.remove(scene)
    if world is not None and world.users == 0:
        bpy.data.worlds.remove(world)


def _first_output(node, socket_type=None):
    for socket in node.outputs:
        if socket_type is None or socket.type == socket_type:
            return socket
    return None


def _configure_preview_material(preview_material, node_name):
    """Connect one copied node's output to a new preview material output."""
    node_tree = preview_material.node_tree
    target = node_tree.nodes.get(node_name)
    if target is None or not target.outputs:
        return False

    for node in tuple(node_tree.nodes):
        if node.type == 'OUTPUT_MATERIAL':
            node_tree.nodes.remove(node)

    output_node = node_tree.nodes.new("ShaderNodeOutputMaterial")
    output_node.location = (700.0, 0.0)
    shader_output = _first_output(target, 'SHADER')
    if shader_output is not None:
        node_tree.links.new(shader_output, output_node.inputs["Surface"])
        return True

    value_output = (
        _first_output(target, 'RGBA')
        or _first_output(target, 'VECTOR')
        or _first_output(target, 'VALUE')
    )
    if value_output is None:
        return False

    # Data-producing nodes should show their actual color/value, not a
    # generic lit sphere/plane. Emission keeps image and procedural texture
    # previews faithful to the node output; shader-producing nodes above are
    # still rendered through their own shader output.
    shader_node = node_tree.nodes.new("ShaderNodeEmission")
    shader_node.location = (400.0, 0.0)
    node_tree.links.new(value_output, shader_node.inputs["Color"])
    node_tree.links.new(shader_node.outputs["Emission"], output_node.inputs["Surface"])
    return True


def _render_node_preview(material, node, signature):
    """Render a copied material with one node connected to the output."""
    if material is None or node is None or material.node_tree is None:
        return None
    if material.node_tree.nodes.get(node.name) is None:
        # Nested group previews are deliberately left for the next iteration;
        # the top-level material path is the reliable Python baseline.
        return None

    preview_material = material.copy()
    preview_material.name = f"rCAD Preview Material {abs(hash((material.name, node.name))) % 1000000}"
    if not _configure_preview_material(preview_material, node.name):
        bpy.data.materials.remove(preview_material)
        return None

    scene = _ensure_preview_scene()
    preview_object = _preview_object
    old_materials = tuple(preview_object.data.materials)
    preview_object.data.materials.clear()
    preview_object.data.materials.append(preview_material)

    temporary_path = None
    old_filepath = scene.render.filepath
    try:
        scene.render.resolution_x = PREVIEW_RESOLUTION
        scene.render.resolution_y = PREVIEW_RESOLUTION
        file_handle, temporary_path = tempfile.mkstemp(
            prefix="rcad_node_preview_",
            suffix=".png",
        )
        os.close(file_handle)
        os.unlink(temporary_path)
        scene.render.filepath = temporary_path
        bpy.ops.render.render(scene=scene.name, write_still=True)
        if not os.path.isfile(temporary_path):
            return None
        image_name = f"{PREVIEW_IMAGE_PREFIX}{abs(hash((material.name, node.name, signature))) % 100000000}"
        image = bpy.data.images.get(image_name)
        if image is not None:
            bpy.data.images.remove(image)
        image = bpy.data.images.load(temporary_path, check_existing=False)
        image.name = image_name
        image.pack()
        image["rcad_node_preview"] = True
        return image
    except (AttributeError, RuntimeError, TypeError, ValueError) as error:
        print(f"[rCAD Node Preview] Render failed for {node.name}: {error}")
        return None
    finally:
        scene.render.filepath = old_filepath
        if temporary_path:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass
        preview_object.data.materials.clear()
        for old_material in old_materials:
            preview_object.data.materials.append(old_material)
        bpy.data.materials.remove(preview_material)


def _queue_job(key, material, tree, node, signature):
    if key in _queued_jobs:
        return
    _queued_jobs.add(key)
    _preview_jobs.append((key, material, tree, node.name, signature))


def _collect_preview_jobs():
    for _area, space, tree in _shader_editor_contexts():
        material = _material_for_space(space, tree)
        if material is None or material.node_tree != tree:
            continue

        signature = _tree_signature(tree)
        for node in tree.nodes:
            if not _is_previewable_node(node):
                continue
            key = _cache_key(material, tree, node)
            entry = _preview_cache.get(key)
            if entry is None or entry["signature"] != signature:
                if entry is not None:
                    _remove_preview_image(entry.get("image"))
                entry = {
                    "signature": signature,
                    "image": None,
                    "error": None,
                }
                _preview_cache[key] = entry
                _queue_job(key, material, tree, node, signature)


def _tag_node_editor_redraws():
    for area, _space, _tree in _shader_editor_contexts():
        area.tag_redraw()


def _process_preview_job():
    if not _preview_jobs:
        return
    key, material, tree, node_name, signature = _preview_jobs.popleft()
    _queued_jobs.discard(key)
    entry = _preview_cache.get(key)
    if entry is None or entry["signature"] != signature:
        return

    node = tree.nodes.get(node_name)
    if node is None:
        return
    image = _render_node_preview(material, node, signature)
    if image is None:
        entry["error"] = True
        return
    old_image = entry.get("image")
    if old_image is not image:
        _remove_preview_image(old_image)
    entry["image"] = image
    entry["error"] = None
    _tag_node_editor_redraws()


def _preview_timer():
    global _preview_timer_running
    scene = getattr(bpy.context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        _preview_timer_running = False
        return None

    try:
        _collect_preview_jobs()
        _process_preview_job()
    except (AttributeError, ReferenceError, RuntimeError, TypeError, ValueError) as error:
        print(f"[rCAD Node Preview] Update failed: {error}")
    _tag_node_editor_redraws()
    return 0.2


def _start_preview_timer():
    global _preview_timer_running
    if _preview_timer_running:
        return
    _preview_timer_running = True
    try:
        bpy.app.timers.register(_preview_timer, first_interval=0.05)
    except (RuntimeError, ValueError):
        _preview_timer_running = False


def _draw_image(image, bottom_left, top_right):
    global _image_shader
    if image is None:
        return

    try:
        import gpu
        from gpu_extras.batch import batch_for_shader

        if _image_shader is None:
            for shader_name in ('IMAGE', '2D_IMAGE'):
                try:
                    _image_shader = gpu.shader.from_builtin(shader_name)
                    break
                except (RuntimeError, ValueError):
                    continue
        if _image_shader is None:
            return

        image_pointer = _safe_pointer(image)
        texture = _gpu_texture_cache.get(image_pointer)
        if texture is None:
            texture = gpu.texture.from_image(image)
            _gpu_texture_cache[image_pointer] = texture

        x0, y0 = bottom_left
        x1, y1 = top_right
        batch = batch_for_shader(
            _image_shader,
            'TRI_FAN',
            {
                "pos": ((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
                "texCoord": ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
            },
        )
        gpu.state.blend_set('ALPHA')
        _image_shader.bind()
        _image_shader.uniform_sampler("image", texture)
        batch.draw(_image_shader)
        gpu.state.blend_set('NONE')
    except (AttributeError, RuntimeError, TypeError, ValueError):
        return


def _draw_previews(_context=None):
    # SpaceNodeEditor.draw_handler_add invokes draw callbacks with the
    # arguments supplied at registration time. This handler is registered
    # without arguments, so use Blender's current draw context here.
    context = _context if _context is not None else bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        return

    space = getattr(context, "space_data", None)
    area = getattr(context, "area", None)
    region = getattr(context, "region", None)
    if (
        space is None
        or area is None
        or region is None
        or space.tree_type != 'ShaderNodeTree'
    ):
        return

    tree = getattr(space, "edit_tree", None)
    if tree is None:
        tree = getattr(space, "node_tree", None)
    material = _material_for_space(space, tree)
    if tree is None or material is None or material.node_tree != tree:
        return

    view2d = region.view2d
    ui_scale = context.preferences.system.ui_scale
    for node in tree.nodes:
        if not _is_previewable_node(node) or node.hide:
            continue
        key = _cache_key(material, tree, node)
        entry = _preview_cache.get(key)
        if entry is None or entry.get("image") is None:
            continue

        location = node.location
        node_left, node_bottom = view2d.view_to_region(
            location.x,
            location.y,
            clip=False,
        )
        node_right, node_top = view2d.view_to_region(
            location.x + node.width,
            location.y + node.height,
            clip=False,
        )
        node_width = abs(node_right - node_left)
        node_height = abs(node_top - node_bottom)
        preview_size = min(node_width - 8.0 * ui_scale, 150.0 * ui_scale)
        if preview_size < 24.0:
            continue

        center_x = (node_left + node_right) * 0.5
        bottom_y = max(node_top, node_bottom) + 8.0 * ui_scale
        top_y = bottom_y + preview_size
        bottom_left = (center_x - preview_size * 0.5, bottom_y)
        top_right = (center_x + preview_size * 0.5, top_y)
        if top_right[0] < 0 or bottom_left[0] > region.width:
            continue
        if top_right[1] < 0 or bottom_left[1] > region.height:
            continue
        _draw_image(entry["image"], bottom_left, top_right)


def _draw_node_preview_header(self, context):
    space = getattr(context, "space_data", None)
    if (
        space is None
        or space.type != 'NODE_EDITOR'
        or space.tree_type != 'ShaderNodeTree'
    ):
        return
    self.layout.separator()
    self.layout.prop(context.scene, PROPERTY_NAME, text="Node Preview")


def _enabled_update(scene, _context):
    if getattr(scene, PROPERTY_NAME, False):
        _start_preview_timer()
    else:
        _clear_preview_cache()
        _destroy_preview_scene()
        _tag_node_editor_redraws()


@persistent
def _load_post(_dummy):
    _clear_preview_cache()
    if getattr(bpy.context.scene, PROPERTY_NAME, False):
        _start_preview_timer()


def register():
    global _draw_handle
    bpy.types.Scene.rcad_node_preview_enabled = BoolProperty(
        name="Node Preview",
        description="Render small previews above shader nodes",
        default=False,
        update=_enabled_update,
    )
    bpy.types.NODE_HT_header.append(_draw_node_preview_header)
    if not bpy.app.background:
        _draw_handle = bpy.types.SpaceNodeEditor.draw_handler_add(
            _draw_previews,
            (),
            'WINDOW',
            'POST_PIXEL',
        )
    if _load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_load_post)
    if getattr(bpy.context.scene, PROPERTY_NAME, False):
        _start_preview_timer()


def unregister():
    global _draw_handle, _preview_timer_running
    try:
        bpy.types.NODE_HT_header.remove(_draw_node_preview_header)
    except (AttributeError, ValueError):
        pass
    if _draw_handle is not None:
        try:
            bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle, 'WINDOW')
        except (AttributeError, RuntimeError, ValueError):
            pass
        _draw_handle = None
    try:
        bpy.app.timers.unregister(_preview_timer)
    except (RuntimeError, ValueError):
        pass
    _preview_timer_running = False
    if _load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_load_post)
    _clear_preview_cache()
    _destroy_preview_scene()
    if hasattr(bpy.types.Scene, PROPERTY_NAME):
        delattr(bpy.types.Scene, PROPERTY_NAME)
