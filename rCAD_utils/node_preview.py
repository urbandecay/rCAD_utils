"""Python-first rendered previews for shader nodes."""

from collections import deque
from time import monotonic

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty
from mathutils import Vector
from . import preview_mouse


PROPERTY_NAME = "rcad_node_preview_enabled"
PREVIEW_IMAGE_PREFIX = "rCAD Node Preview "
PREVIEW_RESOLUTION = 128
PREVIEW_SETTLE_SECONDS = 0.35

_draw_handle = None
_preview_timer_running = False
_preview_jobs = deque()
_queued_jobs = set()
_live_preview_keys = set()
_preview_cache = {}
_tree_states = {}
_preview_scene = None
_preview_object = None
_preview_shader_object = None
_preview_shader_camera = None
_preview_shader = None
_image_shader = None
_gpu_texture_cache = {}
_display_shader = None


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


def _node_signature(node):
    """Return a lightweight revision signature for one shader node."""
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
    return hash(repr((
        node.name,
        node.bl_idname,
        node.label,
        node.mute,
        _safe_pointer(image),
        tuple(inputs),
        tuple((socket.identifier, socket.enabled, socket.is_linked,
               _value_signature(getattr(socket, "default_value", None)))
              for socket in node.outputs),
        _value_signature(getattr(node, "operation", None)),
        _value_signature(getattr(node, "blend_type", None)),
        _value_signature(getattr(node, "noise_dimensions", None)),
    )))


def _tree_signature(tree):
    """Return a lightweight revision signature for a shader node tree."""
    node_data = tuple(
        (node.name, _node_signature(node))
        for node in sorted(tree.nodes, key=lambda item: item.name)
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
        node.name,
        node.bl_idname,
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
    _tree_states.clear()
    _preview_jobs.clear()
    _queued_jobs.clear()
    _live_preview_keys.clear()
    for entry in tuple(_preview_cache.values()):
        _remove_preview_image(entry.get("image"))
        _free_offscreen(entry)
    _preview_cache.clear()
    _gpu_texture_cache.clear()


def _free_offscreen(entry):
    entry["gpu_ready"] = False
    offscreen = entry.pop("offscreen", None)
    if offscreen is not None:
        try:
            offscreen.free()
        except (ReferenceError, RuntimeError):
            pass


def _look_at(object_data, target):
    direction = Vector(target) - object_data.location
    object_data.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def _ensure_preview_scene():
    """Create the private scene used for Python-side thumbnail rendering."""
    global _preview_scene, _preview_object, _preview_shader_object
    global _preview_shader_camera
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

    # Shader outputs need a lit three-dimensional surface.  A flat plane can
    # make a Principled result look like its base-color input, especially
    # under a studio environment.  Keep the plane for data nodes and use a
    # compact cube for shader-producing nodes.
    cube_mesh = bpy.data.meshes.new("rCAD Node Preview Shader Mesh")
    cube_mesh.from_pydata(
        [(-1.0, -1.0, -1.0), (1.0, -1.0, -1.0),
         (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
         (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0),
         (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0)],
        [],
        [(0, 3, 2, 1), (4, 5, 6, 7),
         (0, 1, 5, 4), (1, 2, 6, 5),
         (2, 3, 7, 6), (3, 0, 4, 7)],
    )
    cube_mesh.update()
    cube_uv_layer = cube_mesh.uv_layers.new(name="UVMap")
    face_uvs = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    for polygon in cube_mesh.polygons:
        for corner, loop_index in enumerate(polygon.loop_indices):
            cube_uv_layer.data[loop_index].uv = face_uvs[corner]
    shader_object = bpy.data.objects.new("rCAD Node Preview Shader", cube_mesh)
    scene.collection.objects.link(shader_object)

    camera_data = bpy.data.cameras.new("rCAD Node Preview Camera")
    camera = bpy.data.objects.new("rCAD Node Preview Camera", camera_data)
    scene.collection.objects.link(camera)
    camera_data.type = 'ORTHO'
    camera_data.ortho_scale = 2.08
    camera.location = (0.0, 0.0, 3.7)
    _look_at(camera, (0.0, 0.0, 0.0))
    scene.camera = camera

    shader_camera_data = bpy.data.cameras.new("rCAD Node Preview Shader Camera")
    shader_camera = bpy.data.objects.new(
        "rCAD Node Preview Shader Camera", shader_camera_data
    )
    scene.collection.objects.link(shader_camera)
    shader_camera_data.type = 'ORTHO'
    shader_camera_data.ortho_scale = 3.7
    shader_camera.location = (3.8, -5.2, 3.2)
    _look_at(shader_camera, (0.0, 0.0, 0.0))

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
    _preview_shader_object = shader_object
    _preview_shader_camera = shader_camera
    return scene


def _sync_preview_appearance(scene, source_scene):
    """Use the visible material-preview environment instead of a frontal lamp."""
    scene.display_settings.display_device = source_scene.display_settings.display_device
    for name in ("view_transform", "look", "exposure", "gamma"):
        setattr(scene.view_settings, name, getattr(source_scene.view_settings, name))
    shading = None
    screen = getattr(bpy.context, "screen", None)
    if screen is not None:
        for area in screen.areas:
            if area.type == 'VIEW_3D' and area.spaces.active.shading.type == 'MATERIAL':
                shading = area.spaces.active.shading
                break
    # Studio environment lighting is available even when the viewport is closed.
    studio_name = shading.studio_light if shading is not None else "forest.exr"
    studio = next((light for light in bpy.context.preferences.studio_lights
                   if light.type == 'WORLD' and light.name == studio_name), None)
    if studio is None:
        return
    world_tree = scene.world.node_tree
    environment = world_tree.nodes.get("rCAD Preview Environment")
    if environment is None:
        environment = world_tree.nodes.new("ShaderNodeTexEnvironment")
        environment.name = "rCAD Preview Environment"
        coordinates = world_tree.nodes.new("ShaderNodeTexCoord")
        mapping = world_tree.nodes.new("ShaderNodeMapping")
        mapping.name = "rCAD Preview Rotation"
        world_tree.links.new(coordinates.outputs["Generated"], mapping.inputs["Vector"])
        world_tree.links.new(mapping.outputs["Vector"], environment.inputs["Vector"])
        world_tree.links.new(environment.outputs["Color"], world_tree.nodes["Background"].inputs["Color"])
    if environment.image is None or environment.image.filepath != studio.path:
        previous = environment.image
        environment.image = bpy.data.images.load(studio.path, check_existing=False)
        if previous is not None:
            bpy.data.images.remove(previous)
    world_tree.nodes["rCAD Preview Rotation"].inputs["Rotation"].default_value[2] = (
        shading.studiolight_rotate_z if shading is not None else 0.0
    )
    world_tree.nodes["Background"].inputs["Strength"].default_value = (
        shading.studiolight_intensity if shading is not None else 1.0
    )
    for obj in scene.objects:
        if obj.type == 'LIGHT':
            obj.hide_render = True


def _destroy_preview_scene():
    global _preview_scene, _preview_object, _preview_shader_object
    global _preview_shader_camera
    scene = _preview_scene
    _preview_scene = None
    _preview_object = None
    _preview_shader_object = None
    _preview_shader_camera = None
    if scene is None:
        return

    objects = tuple(scene.objects)
    world = scene.world
    if world is not None and world.node_tree is not None:
        environment = world.node_tree.nodes.get("rCAD Preview Environment")
        if environment is not None and environment.image is not None:
            bpy.data.images.remove(environment.image)
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


def _preview_output(node):
    """Prefer an enabled connected output, then the first enabled output."""
    outputs = [socket for socket in node.outputs
               if socket.enabled and not getattr(socket, "is_unavailable", False)]
    return next((socket for socket in outputs if socket.is_linked),
                outputs[0] if outputs else None)


def _uses_shader_preview(node):
    output = _preview_output(node)
    return output is not None and output.type == 'SHADER'


def _configure_preview_material(preview_material, node_name):
    """Connect one copied node's output to a new preview material output."""
    node_tree = preview_material.node_tree
    target = node_tree.nodes.get(node_name)
    if target is None or not target.outputs:
        return False

    # Select before deleting output nodes, which removes their incoming links.
    selected_output = _preview_output(target)
    if selected_output is None:
        return False

    for node in tuple(node_tree.nodes):
        if node.type == 'OUTPUT_MATERIAL':
            node_tree.nodes.remove(node)

    output_node = node_tree.nodes.new("ShaderNodeOutputMaterial")
    output_node.location = (700.0, 0.0)
    if selected_output.type == 'SHADER':
        destination = "Volume" if selected_output.name == "Volume" else "Surface"
        node_tree.links.new(selected_output, output_node.inputs[destination])
        return True

    value_output = selected_output
    if value_output.type not in {'RGBA', 'VECTOR', 'VALUE', 'INT', 'BOOLEAN'}:
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


def _direct_image_preview(node, signature):
    """Create a cached thumbnail without invoking Blender's render engine."""
    source = getattr(node, "image", None)
    if source is None or source.size[0] <= 0 or source.size[1] <= 0:
        return None

    max_size = PREVIEW_RESOLUTION
    scale = min(max_size / source.size[0], max_size / source.size[1], 1.0)
    width = max(1, round(source.size[0] * scale))
    height = max(1, round(source.size[1] * scale))
    image_name = (
        f"{PREVIEW_IMAGE_PREFIX}direct "
        f"{abs(hash((source.name, node.name, signature))) % 100000000}"
    )
    preview = source.copy()
    preview.name = image_name
    if preview.size[0] != width or preview.size[1] != height:
        preview.scale(width, height)
    preview.pack()
    preview["rcad_node_preview"] = True
    return preview


def _can_direct_image_preview(node):
    if node.bl_idname != "ShaderNodeTexImage":
        return False
    output = _preview_output(node)
    if output is None or output.name != "Color":
        return False
    vector_input = node.inputs.get("Vector")
    return vector_input is None or not vector_input.is_linked

def _queue_job(key, material, tree, node, signature):
    if key in _queued_jobs:
        # Replace superseded work instead of leaving stale jobs ahead of it.
        remaining = [job for job in _preview_jobs if job[0] != key]
        _preview_jobs.clear()
        _preview_jobs.extend(remaining)
    _queued_jobs.add(key)
    _preview_jobs.append((key, material, tree, node.name, signature))


def _queue_preview_node(material, tree, node, force=False):
    key = _cache_key(material, tree, node)
    signature = _node_signature(node)
    entry = _preview_cache.get(key)
    if entry is None:
        entry = {
            "signature": signature,
            "image": None,
            "error": None,
        }
        _preview_cache[key] = entry
    elif entry["signature"] != signature:
        _queued_jobs.discard(key)
        entry["signature"] = signature
        entry["error"] = None
    elif not force and entry.get("image") is not None:
        return
    _queue_job(key, material, tree, node, signature)


def _active_node(tree):
    active = getattr(tree.nodes, "active", None)
    if active is not None:
        return active
    selected = [node for node in tree.nodes if getattr(node, "select", False)]
    return selected[-1] if selected else None


def _downstream_node_names(tree, start_names):
    nodes_by_name = {node.name: node for node in tree.nodes}
    pending = list(start_names)
    result = set(start_names)
    while pending:
        node = nodes_by_name.get(pending.pop())
        if node is None:
            continue
        for output in node.outputs:
            for link in output.links:
                name = link.to_node.name
                if name not in result:
                    result.add(name)
                    pending.append(name)
    return result


def _collect_preview_jobs():
    now = monotonic()
    for _area, space, tree in _shader_editor_contexts():
        material = _material_for_space(space, tree)
        if material is None or material.node_tree != tree:
            continue

        valid_keys = {
            _cache_key(material, tree, node) for node in tree.nodes
            if _is_previewable_node(node)
        }
        prefix = (_safe_pointer(material), _safe_pointer(tree))
        for key in tuple(_preview_cache):
            if key[:2] == prefix and key not in valid_keys:
                removed = _preview_cache.pop(key)
                _remove_preview_image(removed.get("image"))
                _free_offscreen(removed)
                _queued_jobs.discard(key)
        remaining = [job for job in _preview_jobs
                     if job[0][:2] != prefix or job[0] in valid_keys]
        _preview_jobs.clear()
        _preview_jobs.extend(remaining)

        tree_key = _safe_pointer(tree)
        state = _tree_states.setdefault(
            tree_key,
            {
                "node_signatures": {},
                "pending_changed": set(),
                "last_change": 0.0,
            },
        )
        current_signatures = {
            node.name: _node_signature(node)
            for node in tree.nodes
        }

        if not state["node_signatures"]:
            state["node_signatures"] = current_signatures
            for node in tree.nodes:
                if _is_previewable_node(node):
                    _queue_preview_node(material, tree, node, force=True)
            continue

        changed_names = {
            name
            for name, signature in current_signatures.items()
            if state["node_signatures"].get(name) != signature
        }
        state["node_signatures"] = current_signatures
        if changed_names:
            state["pending_changed"].update(changed_names)
            state["last_change"] = now
            active = _active_node(tree)
            if active is not None and active.name in changed_names:
                immediate_names = {active.name}
            else:
                immediate_names = changed_names
            for node in tree.nodes:
                if node.name in immediate_names and _is_previewable_node(node):
                    _live_preview_keys.add(_cache_key(material, tree, node))
                    _queue_preview_node(material, tree, node, force=True)

        if (
            state["pending_changed"]
            and not preview_mouse.buttons_held()
            and now - state["last_change"] >= PREVIEW_SETTLE_SECONDS
        ):
            affected_names = _downstream_node_names(
                tree,
                state["pending_changed"],
            )
            for node in tree.nodes:
                if node.name in affected_names and _is_previewable_node(node):
                    _queue_preview_node(material, tree, node, force=True)
            state["pending_changed"].clear()


def _tag_node_editor_redraws():
    for area, _space, _tree in _shader_editor_contexts():
        area.tag_redraw()


def _draw_node_offscreen(material, node, entry):
    """Evaluate Blender's shaders straight into a reusable GPU texture."""
    import gpu

    screen = getattr(bpy.context, "screen", None)
    area = next((area for area in screen.areas if area.type == 'VIEW_3D'), None) if screen else None
    if area is None:
        raise RuntimeError("Keep a 3D Viewport open for live node previews")
    region = next(region for region in area.regions if region.type == 'WINDOW')
    space = area.spaces.active
    scene = _ensure_preview_scene()
    _sync_preview_appearance(scene, bpy.context.scene)
    preview_material = material.copy()
    if not _configure_preview_material(preview_material, node.name):
        bpy.data.materials.remove(preview_material)
        raise RuntimeError("Unsupported node output")
    offscreen = entry.get("offscreen")
    if offscreen is None:
        offscreen = gpu.types.GPUOffScreen(PREVIEW_RESOLUTION, PREVIEW_RESOLUTION)
        entry["offscreen"] = offscreen
    shader_preview = _uses_shader_preview(node)
    preview_object = (
        _preview_shader_object if shader_preview else _preview_object
    )
    preview_camera = (
        _preview_shader_camera if shader_preview else scene.camera
    )
    if preview_object is None or preview_camera is None:
        bpy.data.materials.remove(preview_material)
        raise RuntimeError("Preview geometry is unavailable")
    shading = space.shading
    old_settings = {name: getattr(shading, name) for name in
                    ("type", "use_scene_world", "use_scene_lights")}
    old_overlays = space.overlay.show_overlays
    old_camera = scene.camera
    preview_objects = tuple(
        obj for obj in (_preview_object, _preview_shader_object)
        if obj is not None
    )
    old_visibility = {
        obj: (obj.hide_render, obj.hide_viewport)
        for obj in preview_objects
    }
    try:
        for obj in preview_objects:
            obj.hide_render = obj is not preview_object
            obj.hide_viewport = obj is not preview_object
        preview_object.data.materials.clear()
        preview_object.data.materials.append(preview_material)
        scene.camera = preview_camera
        shading.type = 'MATERIAL'
        shading.use_scene_world = True
        shading.use_scene_lights = True
        space.overlay.show_overlays = False
        with bpy.context.temp_override(scene=scene, view_layer=scene.view_layers[0],
                                       area=area, region=region):
            scene.view_layers[0].update()
            depsgraph = bpy.context.evaluated_depsgraph_get()
            camera = scene.camera
            projection = camera.calc_matrix_camera(depsgraph, x=PREVIEW_RESOLUTION,
                                                   y=PREVIEW_RESOLUTION)
            offscreen.draw_view3d(scene, scene.view_layers[0], space, region,
                                 camera.matrix_world.inverted(), projection,
                                 do_color_management=True, draw_background=True)
        entry["gpu_ready"] = True
    finally:
        for name, value in old_settings.items():
            setattr(shading, name, value)
        space.overlay.show_overlays = old_overlays
        scene.camera = old_camera
        preview_object.data.materials.clear()
        for obj, (hide_render, hide_viewport) in old_visibility.items():
            obj.hide_render = hide_render
            obj.hide_viewport = hide_viewport
        bpy.data.materials.remove(preview_material)


def _process_preview_job():
    # Keep the active node responsive while a drag is held, but leave all
    # other queued previews untouched until the button is released.
    mouse_held = preview_mouse.buttons_held()
    for _ in range(len(_preview_jobs)):
        key, material, tree, node_name, signature = _preview_jobs.popleft()
        if mouse_held and key not in _live_preview_keys:
            _preview_jobs.append((key, material, tree, node_name, signature))
            continue
        _queued_jobs.discard(key)
        _live_preview_keys.discard(key)
        entry = _preview_cache.get(key)
        if entry is None or entry["signature"] != signature:
            continue

        node = tree.nodes.get(node_name)
        if node is None or _cache_key(material, tree, node) != key:
            continue
        if not _can_direct_image_preview(node):
            try:
                _draw_node_offscreen(material, node, entry)
                _remove_preview_image(entry.get("image"))
                entry["image"] = None
                entry["error"] = None
            except (AttributeError, RuntimeError, ReferenceError, ValueError, SystemError) as error:
                entry["error"] = str(error)
                print(f"[rCAD Node Preview] Live preview: {error}")
            _tag_node_editor_redraws()
            break
        image = _direct_image_preview(node, signature)
        if image is None:
            entry["error"] = True
        else:
            _free_offscreen(entry)
            entry["gpu_ready"] = False
            old_image = entry.get("image")
            if old_image is not image:
                _remove_preview_image(old_image)
            entry["image"] = image
            entry["error"] = None
        _tag_node_editor_redraws()

        # All direct image thumbnails can update in the same pass. GPU
        # viewport previews above consume at most one draw per timer tick.

    if not mouse_held:
        _live_preview_keys.clear()


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
    return 0.05


def _start_preview_timer():
    global _preview_timer_running
    if bpy.app.timers.is_registered(_preview_timer):
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
            # GPU image textures contain linear RGB. Convert to display sRGB
            # when drawing into the node editor; IMAGE alone makes them dark.
            _image_shader = gpu.shader.from_builtin(
                'IMAGE_SCENE_LINEAR_TO_REC709_SRGB'
            )

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


def _draw_offscreen(entry, bottom_left, top_right):
    global _display_shader
    import gpu
    from gpu_extras.batch import batch_for_shader
    if _display_shader is None:
        # draw_view3d already applied the scene's display transform.
        _display_shader = gpu.shader.from_builtin('IMAGE')
    x0, y0 = bottom_left
    x1, y1 = top_right
    batch = batch_for_shader(_display_shader, 'TRI_FAN', {
        "pos": ((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
        "texCoord": ((0, 0), (1, 0), (1, 1), (0, 1)),
    })
    _display_shader.bind()
    _display_shader.uniform_sampler("image", entry["offscreen"].texture_color)
    batch.draw(_display_shader)


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
        if entry is None or (entry.get("image") is None and not entry.get("gpu_ready")):
            continue

        location = node.location.copy()
        parent = node.parent
        while parent is not None:
            location += parent.location
            parent = parent.parent
        node_left, node_bottom = view2d.view_to_region(
            location.x * ui_scale,
            location.y * ui_scale,
            clip=False,
        )
        node_right, node_top = view2d.view_to_region(
            (location.x + node.width) * ui_scale,
            location.y * ui_scale,
            clip=False,
        )
        node_width = abs(node_right - node_left)
        pixels_per_unit = node_width / max(node.width, 1.0)
        preview_size = min(max(node.width - 8.0, 1.0), 150.0) * pixels_per_unit
        if preview_size < 1.0:
            continue

        center_x = (node_left + node_right) * 0.5
        bottom_y = max(node_top, node_bottom) + 8.0 * pixels_per_unit
        top_y = bottom_y + preview_size
        bottom_left = (center_x - preview_size * 0.5, bottom_y)
        top_right = (center_x + preview_size * 0.5, top_y)
        if top_right[0] < 0 or bottom_left[0] > region.width:
            continue
        if top_right[1] < 0 or bottom_left[1] > region.height:
            continue
        if entry.get("gpu_ready"):
            _draw_offscreen(entry, bottom_left, top_right)
        else:
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
    global _preview_scene, _preview_object, _preview_shader_object
    global _preview_shader_camera, _image_shader, _display_shader
    # File loading replaces all RNA data and removes nonpersistent timers.
    # Drop old references before touching the newly loaded file.
    _preview_scene = None
    _preview_object = None
    _preview_shader_object = None
    _preview_shader_camera = None
    _image_shader = None
    _display_shader = None
    _clear_preview_cache()
    if getattr(bpy.context.scene, PROPERTY_NAME, False):
        _start_preview_timer()


def register():
    global _draw_handle
    bpy.types.Scene.rcad_node_preview_enabled = BoolProperty(
        name="Node Preview",
        description="Live GPU previews above shader nodes; keep a 3D Viewport open",
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
    scene = getattr(bpy.context, "scene", None)
    if scene is not None and getattr(scene, PROPERTY_NAME, False):
        _start_preview_timer()


def unregister():
    global _draw_handle, _preview_timer_running
    preview_mouse.close()
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
