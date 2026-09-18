"""Add an image from the operating system clipboard as an Image Empty."""

import os
import shutil
import subprocess
import tempfile

import bmesh
import bpy
from mathutils import Vector


_GTK_CLIPBOARD_IMAGE_SCRIPT = r'''
import sys

try:
    import gi
    gi.require_version("Gtk", "3.0")
    from gi.repository import Gdk, Gtk

    clipboard = Gtk.Clipboard.get(Gdk.SELECTION_CLIPBOARD)
    if not clipboard.wait_is_image_available():
        raise RuntimeError("The clipboard does not contain an image")

    pixbuf = clipboard.wait_for_image()
    if pixbuf is None:
        raise RuntimeError("The clipboard image could not be read")

    success, png_data = pixbuf.save_to_bufferv("png", [], [])
    if not success:
        raise RuntimeError("The clipboard image could not be converted")

    sys.stdout.buffer.write(bytes(png_data))
except Exception:
    sys.exit(1)
'''

_CLIPBOARD_IMAGE_TYPES = (
    ("image/png", ".png"),
    ("image/jpeg", ".jpg"),
    ("image/jpg", ".jpg"),
    ("image/bmp", ".bmp"),
    ("image/tiff", ".tif"),
    ("image/webp", ".webp"),
)


def _read_command_output(command):
    """Return binary command output, or None if the command is unavailable."""
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    if completed.returncode != 0 or not completed.stdout:
        return None
    return completed.stdout


def _read_gtk_clipboard_image():
    """Ask GTK to convert a desktop clipboard image to PNG bytes."""
    python_commands = ["/usr/bin/python3"]
    system_python = shutil.which("python3")
    if system_python and system_python not in python_commands:
        python_commands.append(system_python)

    for python_command in python_commands:
        if not os.path.isfile(python_command):
            continue
        png_data = _read_command_output(
            [python_command, "-c", _GTK_CLIPBOARD_IMAGE_SCRIPT]
        )
        if png_data:
            return png_data, ".png"
    return None


def _read_native_clipboard_image():
    """Read common image clipboard targets exposed by X11 or Wayland tools."""
    wl_paste = shutil.which("wl-paste")
    if wl_paste:
        for mime_type, suffix in _CLIPBOARD_IMAGE_TYPES:
            image_data = _read_command_output(
                [wl_paste, "--no-newline", "--type", mime_type]
            )
            if image_data:
                return image_data, suffix

    xclip = shutil.which("xclip")
    if xclip:
        for mime_type, suffix in _CLIPBOARD_IMAGE_TYPES:
            image_data = _read_command_output(
                [xclip, "-selection", "clipboard", "-t", mime_type, "-o"]
            )
            if image_data:
                return image_data, suffix

    return None


def _read_system_clipboard_image():
    """Return clipboard image bytes and their temporary-file suffix."""
    return _read_gtk_clipboard_image() or _read_native_clipboard_image()


def _load_clipboard_image(image_data, suffix):
    """Load clipboard bytes into a packed Blender Image datablock."""
    temporary_path = None
    image = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as temporary_file:
            temporary_file.write(image_data)
            temporary_path = temporary_file.name

        image = bpy.data.images.load(temporary_path, check_existing=False)
        image.pack()
        image.name = "Clipboard Image"
        return image
    except (OSError, RuntimeError) as exc:
        if image is not None:
            bpy.data.images.remove(image)
        raise RuntimeError("Blender could not load the image from the clipboard") from exc
    finally:
        if temporary_path:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass


def _paste_clipboard_image_native(context):
    """Use Blender's native clipboard support and return the pasted Image datablock."""
    area = context.area
    if area is None or area.type not in {'VIEW_3D', 'IMAGE_EDITOR'}:
        raise RuntimeError("Run Add Clipboard Image from a 3D View")

    original_area_type = area.type
    image_editor_image = None
    image_pointers_before = {
        image.as_pointer()
        for image in bpy.data.images
    }

    try:
        # Blender's clipboard operator is available in the Image Editor. A
        # temporary type switch lets the feature be launched directly from
        # the 3D View's Shift+A menu without opening another editor.
        if original_area_type != 'IMAGE_EDITOR':
            area.type = 'IMAGE_EDITOR'

        image_editor = area.spaces.active
        image_editor_image = image_editor.image
        region = next(
            (item for item in area.regions if item.type == 'WINDOW'),
            None,
        )
        if region is None:
            raise RuntimeError("Could not access the Image Editor")

        clipboard_paste = getattr(bpy.ops.image, "clipboard_paste", None)
        if clipboard_paste is None:
            raise RuntimeError("This Blender version cannot paste images from the clipboard")

        override = {
            "area": area,
            "region": region,
        }
        if context.window is not None:
            override["window"] = context.window

        try:
            if hasattr(context, "temp_override"):
                with context.temp_override(**override):
                    result = clipboard_paste()
            else:
                result = clipboard_paste()
        except (AttributeError, RuntimeError) as exc:
            raise RuntimeError(
                "No compatible image is available in the system clipboard"
            ) from exc

        if 'FINISHED' not in result:
            raise RuntimeError("No compatible image is available in the system clipboard")

        # The native operator normally assigns the new image to the editor.
        # The datablock fallback also handles Blender versions that create it
        # without changing the active editor image.
        new_images = [
            image for image in bpy.data.images
            if image.as_pointer() not in image_pointers_before
        ]
        if new_images:
            return new_images[-1]

        image_editor_image = image_editor.image
        if image_editor_image is None:
            raise RuntimeError("Blender could not create an image from the clipboard")
        return image_editor_image
    finally:
        if original_area_type != 'IMAGE_EDITOR' and area.type == 'IMAGE_EDITOR':
            area.type = original_area_type


def _paste_clipboard_image(context):
    """Paste using Blender first, then fall back to the desktop clipboard."""
    area = context.area
    if area is None or area.type not in {'VIEW_3D', 'IMAGE_EDITOR'}:
        raise RuntimeError("Run Add Clipboard Image from a 3D View")

    try:
        return _paste_clipboard_image_native(context)
    except RuntimeError as native_error:
        clipboard_image = _read_system_clipboard_image()
        if clipboard_image is None:
            raise RuntimeError(
                "No compatible image is available in the system clipboard"
            ) from native_error
        image_data, suffix = clipboard_image
        return _load_clipboard_image(image_data, suffix)


def _move_to_dedicated_collection(scene, empty):
    """Give one clipboard image its own collection for independent selection."""
    for collection in empty.users_collection:
        if collection.get("rcad_clipboard_image_collection", False):
            return collection

    collection = bpy.data.collections.new(f"{empty.name} Collection")
    collection["rcad_clipboard_image_collection"] = True
    scene.collection.children.link(collection)

    for old_collection in tuple(empty.users_collection):
        old_collection.objects.unlink(empty)
    collection.objects.link(empty)
    return collection


def _create_image_empty(context, image):
    """Create a normal selectable Image Empty at the current 3D cursor."""
    empty = bpy.data.objects.new("Clipboard Image", None)
    _move_to_dedicated_collection(context.scene, empty)

    empty.empty_display_type = 'IMAGE'
    empty.data = image
    empty.location = context.scene.cursor.location
    empty.rotation_euler = context.scene.cursor.rotation_euler

    # Make the imported image behave like every other newly added Blender
    # object: it is the only selected object and the active object.
    for selected_object in context.view_layer.objects:
        selected_object.select_set(False)
    empty.hide_select = False
    empty.empty_image_depth = 'DEFAULT'
    empty.select_set(True)
    context.view_layer.objects.active = empty
    empty["rcad_clipboard_image"] = True
    return empty


def _create_textured_face(context, image):
    """Create an aspect-correct UV-mapped quad on the active Edit Mode mesh."""
    mesh_object = context.edit_object
    if mesh_object is None or mesh_object.type != 'MESH':
        raise RuntimeError("Edit Mode clipboard import needs an active mesh object")

    image_width, image_height = image.size
    if image_width <= 0 or image_height <= 0:
        raise RuntimeError("The clipboard image has no usable dimensions")

    material = bpy.data.materials.new(name=f"{image.name} Material")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()

    output_node = nodes.new("ShaderNodeOutputMaterial")
    shader_node = nodes.new("ShaderNodeBsdfPrincipled")
    image_node = nodes.new("ShaderNodeTexImage")
    image_node.image = image
    links.new(image_node.outputs["Color"], shader_node.inputs["Base Color"])
    links.new(shader_node.outputs["BSDF"], output_node.inputs["Surface"])
    if image_node.outputs.get("Alpha") and shader_node.inputs.get("Alpha"):
        links.new(image_node.outputs["Alpha"], shader_node.inputs["Alpha"])

    if hasattr(material, "surface_render_method"):
        material.surface_render_method = 'DITHERED'
    elif hasattr(material, "blend_method"):
        material.blend_method = 'BLEND'

    if material.name not in mesh_object.data.materials:
        mesh_object.data.materials.append(material)
    material_index = mesh_object.data.materials[:].index(material)

    cursor = context.scene.cursor
    world_center = cursor.location.copy()
    cursor_rotation = cursor.rotation_euler.to_matrix()
    world_x = cursor_rotation @ Vector((1.0, 0.0, 0.0))
    world_y = cursor_rotation @ Vector((0.0, 1.0, 0.0))

    face_width = 1.0
    face_height = face_width * image_height / image_width
    half_x = world_x * (face_width * 0.5)
    half_y = world_y * (face_height * 0.5)
    world_points = (
        world_center - half_x - half_y,
        world_center + half_x - half_y,
        world_center + half_x + half_y,
        world_center - half_x + half_y,
    )
    world_to_local = mesh_object.matrix_world.inverted()
    local_points = [world_to_local @ point for point in world_points]

    bmesh_data = bmesh.from_edit_mesh(mesh_object.data)
    vertices = [bmesh_data.verts.new(point) for point in local_points]
    try:
        face = bmesh_data.faces.new(vertices)
    except ValueError as exc:
        for vertex in vertices:
            if vertex.is_valid:
                bmesh_data.verts.remove(vertex)
        raise RuntimeError("Could not create the clipboard image face here") from exc

    face.material_index = material_index
    uv_layer = bmesh_data.loops.layers.uv.verify()
    for loop, uv in zip(
        face.loops,
        ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
    ):
        loop[uv_layer].uv = uv

    for vertex in bmesh_data.verts:
        vertex.select_set(False)
    for edge in bmesh_data.edges:
        edge.select_set(False)
    for existing_face in bmesh_data.faces:
        existing_face.select_set(False)
    for vertex in vertices:
        vertex.select_set(True)
    for edge in face.edges:
        edge.select_set(True)
    face.select_set(True)
    bmesh_data.select_history.clear()
    bmesh_data.select_history.add(face)
    bmesh.update_edit_mesh(mesh_object.data, loop_triangles=True, destructive=True)
    return material


def configure_existing_clipboard_images():
    """Restore normal object behavior to images made by an older version."""
    for empty in bpy.data.objects:
        if empty.type != 'EMPTY':
            continue

        image_name = getattr(getattr(empty, "data", None), "name", "")
        is_clipboard_image = (
            empty.get("rcad_clipboard_image", False)
            or empty.name.startswith("Clipboard Image")
            or image_name.startswith("Clipboard Image")
        )
        if not is_clipboard_image:
            continue

        empty.empty_image_depth = 'DEFAULT'
        empty.hide_select = False
        _move_to_dedicated_collection(bpy.context.scene, empty)


class OBJECT_OT_add_clipboard_image(bpy.types.Operator):
    """Add the bitmap as an Image Empty or a textured Edit Mode face."""

    bl_idname = "object.add_clipboard_image"
    bl_label = "Image from Clipboard"
    bl_description = "Add the image currently stored in the system clipboard"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode in {'OBJECT', 'EDIT_MESH'}

    def execute(self, context):
        try:
            image = _paste_clipboard_image(context)
        except RuntimeError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        # Clipboard images otherwise have no source file to reload from when
        # the blend is opened again. Packing keeps the pasted pixels in the
        # blend file alongside the Image Empty.
        try:
            if image.packed_file is None:
                image.pack()
        except RuntimeError as exc:
            self.report({'WARNING'}, f"Image added but could not be packed: {exc}")

        if context.mode == 'EDIT_MESH':
            try:
                material = _create_textured_face(context, image)
            except RuntimeError as exc:
                self.report({'ERROR'}, str(exc))
                return {'CANCELLED'}
            self.report({'INFO'}, f"Created a textured face using {material.name}")
        else:
            empty = _create_image_empty(context, image)
            self.report({'INFO'}, f"Added {empty.name} from the clipboard")
        return {'FINISHED'}


def draw_clipboard_image_menu(self, context):
    """Add the operator to Blender's Object Mode image submenu."""
    if context.mode != 'OBJECT':
        return

    self.layout.separator()
    self.layout.operator(
        OBJECT_OT_add_clipboard_image.bl_idname,
        text=OBJECT_OT_add_clipboard_image.bl_label,
        icon='IMAGE_DATA',
    )


def draw_edit_clipboard_image_menu(self, context):
    """Add the textured-face operator to the Edit Mode Shift+A menu."""
    if context.mode != 'EDIT_MESH':
        return

    self.layout.separator()
    self.layout.operator(
        OBJECT_OT_add_clipboard_image.bl_idname,
        text="Textured Face from Clipboard",
        icon='IMAGE_DATA',
    )


classes = (OBJECT_OT_add_clipboard_image,)
