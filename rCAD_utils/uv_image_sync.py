"""Keep the UV Editor image aligned with the active textured face."""

import bmesh
import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty


PROPERTY_NAME = "rcad_sync_uv_editor_image"

_sync_timer_pending = False
_last_face_state = None
_uv_material_timer_pending = False
_pending_uv_space_pointer = 0
_uv_monitor_running = False
_monitor_face_state = None
_monitor_face_image_pointer = None
_monitor_uv_space_pointer = None
_monitor_uv_image_pointer = None
_MESSAGE_BUS_OWNER = object()


def draw_uv_editor_header(self, context):
    """Draw the sync toggle directly in the UV Editor header."""
    space = getattr(context, "space_data", None)
    if space is None or space.type != 'IMAGE_EDITOR':
        return
    if getattr(space, "mode", None) != 'UV':
        return

    self.layout.separator()
    self.layout.prop(
        context.scene,
        PROPERTY_NAME,
        text="Sync Face Image",
    )


def _is_face(component):
    """Return whether a BMesh selection-history item is a face."""
    return (
        component is not None
        and getattr(component, "is_valid", False)
        and hasattr(component, "material_index")
        and hasattr(component, "loops")
    )


def _material_image(material):
    """Find the most likely image texture used by a material."""
    image_node = _find_image_texture_node(material)
    return image_node.image if image_node is not None else None


def _find_image_texture_node(material):
    """Find the image node that represents a material's visible texture."""
    if material is None or not material.use_nodes or material.node_tree is None:
        return None

    nodes = material.node_tree.nodes

    # Prefer an image texture connected to a Principled Base Color socket.
    for node in nodes:
        if node.type != 'BSDF_PRINCIPLED':
            continue
        base_color = node.inputs.get("Base Color")
        if base_color is None:
            continue
        for link in base_color.links:
            source = link.from_node
            if source.type == 'TEX_IMAGE':
                return source

    # Fall back to any image texture in the material. This also supports
    # materials using an image through a custom node setup.
    for node in nodes:
        if node.type == 'TEX_IMAGE':
            return node

    return None


def _set_material_image(material, image):
    """Set or create the material Image Texture node used by the selected face."""
    if material is None:
        return False

    if not material.use_nodes:
        if image is None:
            return False
        material.use_nodes = True

    node = _find_image_texture_node(material)
    if node is None:
        if image is None or material.node_tree is None:
            return False

        nodes = material.node_tree.nodes
        links = material.node_tree.links
        node = nodes.new("ShaderNodeTexImage")
        node.label = "UV Sync Image"

        principled = next(
            (item for item in nodes if item.type == 'BSDF_PRINCIPLED'),
            None,
        )
        base_color = principled.inputs.get("Base Color") if principled else None
        if base_color is not None and not base_color.links:
            links.new(node.outputs["Color"], base_color)

    if node.image != image:
        node.image = image
        return True
    return False


def _selected_face_material(context):
    """Return the selected face's material and an object/face state key."""
    if context is None or context.mode != 'EDIT_MESH':
        return None, (0, -1)

    mesh_object = context.edit_object
    if mesh_object is None or mesh_object.type != 'MESH':
        return None, (0, -1)

    try:
        bmesh_data = bmesh.from_edit_mesh(mesh_object.data)
        active = bmesh_data.select_history.active
        face = active if _is_face(active) and active.select else None
        if face is None:
            face = next((item for item in bmesh_data.faces if item.select), None)

        object_pointer = mesh_object.as_pointer()
        if face is None:
            return None, (object_pointer, -1)

        material_index = face.material_index
        if material_index < 0 or material_index >= len(mesh_object.data.materials):
            return None, (object_pointer, face.index)
        return mesh_object.data.materials[material_index], (object_pointer, face.index)
    except (AttributeError, ReferenceError, RuntimeError):
        return None, (0, -1)


def _selected_face_image(context):
    """Return the image on the active selected face and a change-detection key."""
    material, face_state = _selected_face_material(context)
    image = _material_image(material)
    image_pointer = image.as_pointer() if image is not None else 0
    return image, (*face_state, image_pointer)


def _uv_editor_space(space_pointer=0):
    """Find the current or requested UV Editor space."""
    current_space = getattr(bpy.context, "space_data", None)
    if (
        current_space is not None
        and getattr(current_space, "type", None) == 'IMAGE_EDITOR'
        and getattr(current_space, "mode", None) == 'UV'
    ):
        if not space_pointer or current_space.as_pointer() == space_pointer:
            return current_space

    window_manager = getattr(bpy.context, "window_manager", None)
    if window_manager is None:
        return None

    first_space = None
    for window in tuple(window_manager.windows):
        screen = getattr(window, "screen", None)
        if screen is None:
            continue
        for area in tuple(screen.areas):
            if area.type != 'IMAGE_EDITOR':
                continue
            space = area.spaces.active
            if getattr(space, "mode", None) != 'UV':
                continue
            if space_pointer and space.as_pointer() == space_pointer:
                return space
            if first_space is None:
                first_space = space
    return first_space


def _apply_uv_image_to_selected_material():
    """Copy the UV Editor's selected image into the active face material."""
    global _uv_material_timer_pending, _pending_uv_space_pointer
    _uv_material_timer_pending = False
    space_pointer = _pending_uv_space_pointer
    _pending_uv_space_pointer = 0

    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        return None
    if context.mode != 'EDIT_MESH':
        return None

    space = _uv_editor_space(space_pointer)
    material, _face_state = _selected_face_material(context)
    if space is None or material is None:
        return None

    _set_material_image(material, space.image)
    return None


def _schedule_uv_material_sync(space_pointer=0):
    """Queue a UV-image-to-material update after the header change completes."""
    global _uv_material_timer_pending, _pending_uv_space_pointer
    if space_pointer:
        _pending_uv_space_pointer = space_pointer
    if _uv_material_timer_pending:
        return

    _uv_material_timer_pending = True
    try:
        bpy.app.timers.register(
            _apply_uv_image_to_selected_material,
            first_interval=0.0,
        )
    except (RuntimeError, ValueError):
        _uv_material_timer_pending = False


def _uv_editor_image_changed(*_args):
    """Receive UV Editor image-selector changes through Blender's message bus."""
    space = getattr(bpy.context, "space_data", None)
    if (
        space is None
        or getattr(space, "type", None) != 'IMAGE_EDITOR'
        or getattr(space, "mode", None) != 'UV'
    ):
        return
    _schedule_uv_material_sync(space.as_pointer())


def _reset_monitor_state():
    """Forget the previous UV/face pair before the next edit session."""
    global _monitor_face_state, _monitor_face_image_pointer
    global _monitor_uv_space_pointer, _monitor_uv_image_pointer
    _monitor_face_state = None
    _monitor_face_image_pointer = None
    _monitor_uv_space_pointer = None
    _monitor_uv_image_pointer = None


def _poll_uv_image_changes():
    """Catch UV image-selector changes even when Blender gives no UI context."""
    global _uv_monitor_running
    global _monitor_face_state, _monitor_face_image_pointer
    global _monitor_uv_space_pointer, _monitor_uv_image_pointer

    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        _uv_monitor_running = False
        _reset_monitor_state()
        return None

    if context.mode != 'EDIT_MESH':
        _reset_monitor_state()
        return 0.1

    try:
        space = _uv_editor_space(_pending_uv_space_pointer)
        material, face_state = _selected_face_material(context)
        face_image = _material_image(material)
        face_image_pointer = face_image.as_pointer() if face_image else 0

        if space is None:
            _monitor_face_state = face_state
            _monitor_face_image_pointer = face_image_pointer
            _monitor_uv_space_pointer = None
            _monitor_uv_image_pointer = None
            return 0.1

        uv_image = space.image
        uv_image_pointer = uv_image.as_pointer() if uv_image else 0
        face_changed = face_state != _monitor_face_state
        face_image_changed = face_image_pointer != _monitor_face_image_pointer
        uv_space_changed = space.as_pointer() != _monitor_uv_space_pointer
        uv_image_changed = uv_image_pointer != _monitor_uv_image_pointer

        # A new face selection or a material change drives the UV Editor.
        # A changed UV image drives the selected material in the other
        # direction. Face selection wins when both happen in the same tick.
        if _monitor_uv_space_pointer is None or face_changed:
            _set_uv_editor_image(face_image)
        elif uv_image_changed:
            _set_material_image(material, uv_image)
        elif face_image_changed:
            _set_uv_editor_image(face_image)

        # Capture the post-update values so our own assignment is not treated
        # as a second user edit on the next timer tick.
        updated_space = _uv_editor_space(space.as_pointer()) or space
        updated_uv_image = updated_space.image
        updated_material, updated_face_state = _selected_face_material(context)
        updated_face_image = _material_image(updated_material)
        _monitor_face_state = updated_face_state
        _monitor_face_image_pointer = (
            updated_face_image.as_pointer() if updated_face_image else 0
        )
        _monitor_uv_space_pointer = updated_space.as_pointer()
        _monitor_uv_image_pointer = (
            updated_uv_image.as_pointer() if updated_uv_image else 0
        )
    except (AttributeError, ReferenceError, RuntimeError):
        pass

    return 0.1


def _start_uv_monitor():
    """Start the small polling loop used for UV Editor header changes."""
    global _uv_monitor_running
    if _uv_monitor_running:
        return

    _uv_monitor_running = True
    try:
        bpy.app.timers.register(_poll_uv_image_changes, first_interval=0.05)
    except (RuntimeError, ValueError):
        _uv_monitor_running = False


def _set_uv_editor_image(image):
    """Set the displayed image in every UV Editor area in open windows."""
    window_manager = getattr(bpy.context, "window_manager", None)
    if window_manager is None:
        return

    for window in tuple(window_manager.windows):
        screen = getattr(window, "screen", None)
        if screen is None:
            continue
        for area in tuple(screen.areas):
            if area.type != 'IMAGE_EDITOR':
                continue

            space = area.spaces.active
            # SpaceImageEditor uses mode='UV' for Blender's UV Editor mode.
            if getattr(space, "mode", None) != 'UV':
                continue

            # A pinned image would otherwise override the selected face.
            if getattr(space, "use_image_pin", False):
                space.use_image_pin = False
            if space.image != image:
                space.image = image
            area.tag_redraw()


def _apply_sync():
    """Apply the latest selected-face image after Blender finishes the click."""
    global _sync_timer_pending, _last_face_state
    _sync_timer_pending = False

    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        return None
    if context.mode != 'EDIT_MESH':
        return None

    image, state = _selected_face_image(context)
    _last_face_state = state
    _set_uv_editor_image(image)
    return None


def _schedule_sync():
    """Queue one update for Blender's next event-loop tick."""
    global _sync_timer_pending
    if _sync_timer_pending:
        return

    _sync_timer_pending = True
    try:
        bpy.app.timers.register(_apply_sync, first_interval=0.0)
    except (RuntimeError, ValueError):
        _sync_timer_pending = False


@persistent
def _face_selection_update_handler(_scene, _depsgraph):
    """Notice active-face changes made in the 3D View or UV Editor."""
    global _last_face_state

    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        _last_face_state = None
        return
    if context.mode != 'EDIT_MESH':
        _last_face_state = None
        return

    _image, state = _selected_face_image(context)
    if state == _last_face_state:
        return

    _last_face_state = state
    _schedule_sync()


def _enabled_update(scene, _context):
    """Refresh the UV Editor as soon as the checkbox is enabled."""
    global _last_face_state
    _last_face_state = None
    if getattr(scene, PROPERTY_NAME, False):
        _start_uv_monitor()
        _schedule_sync()


def register():
    global _last_face_state

    bpy.types.Scene.rcad_sync_uv_editor_image = BoolProperty(
        name="Sync UV Editor Image",
        description="Show the selected face's material image in the UV Editor",
        default=False,
        update=_enabled_update,
    )
    if _face_selection_update_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_face_selection_update_handler)
    image_menus = getattr(bpy.types, "IMAGE_MT_editor_menus", None)
    if image_menus is not None:
        image_menus.prepend(draw_uv_editor_header)
    bpy.msgbus.subscribe_rna(
        key=(bpy.types.SpaceImageEditor, "image"),
        owner=_MESSAGE_BUS_OWNER,
        args=(),
        notify=_uv_editor_image_changed,
    )
    _last_face_state = None
    scene = getattr(bpy.context, "scene", None)
    if scene is not None and getattr(scene, PROPERTY_NAME, False):
        _start_uv_monitor()


def unregister():
    global _sync_timer_pending
    global _uv_material_timer_pending, _pending_uv_space_pointer, _last_face_state
    global _uv_monitor_running

    try:
        bpy.app.timers.unregister(_apply_sync)
    except (RuntimeError, ValueError):
        pass
    _sync_timer_pending = False
    try:
        bpy.app.timers.unregister(_apply_uv_image_to_selected_material)
    except (RuntimeError, ValueError):
        pass
    _uv_material_timer_pending = False
    _pending_uv_space_pointer = 0
    try:
        bpy.app.timers.unregister(_poll_uv_image_changes)
    except (RuntimeError, ValueError):
        pass
    _uv_monitor_running = False
    _reset_monitor_state()

    bpy.msgbus.clear_by_owner(_MESSAGE_BUS_OWNER)

    if _face_selection_update_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_face_selection_update_handler)
    image_menus = getattr(bpy.types, "IMAGE_MT_editor_menus", None)
    if image_menus is not None:
        try:
            image_menus.remove(draw_uv_editor_header)
        except ValueError:
            pass
    _last_face_state = None

    if hasattr(bpy.types.Scene, PROPERTY_NAME):
        delattr(bpy.types.Scene, PROPERTY_NAME)
