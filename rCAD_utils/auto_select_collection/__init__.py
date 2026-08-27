"""Automatically select the active object's collection after a click."""

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty


bl_info = {
    "name": "Auto Select Collection",
    "author": "rCAD Utils",
    "version": (1, 0, 0),
    "blender": (3, 0, 0),
    "location": "3D View > Select",
    "description": "Select all visible objects that share a collection with the active object",
    "category": "Object",
}


PROPERTY_NAME = "rcad_auto_select_collection"
_selection_guard = False
_selection_timer_pending = False
_last_selection_state = None


def _is_hidden(obj, view_layer):
    """Return whether an object is hidden in the current view layer."""
    try:
        return obj.hide_get(view_layer=view_layer)
    except (AttributeError, RuntimeError, TypeError):
        return bool(obj.hide_viewport)


def _shared_collection_objects(active, view_layer):
    """Return visible view-layer objects sharing a collection with *active*."""
    shared_collections = set(active.users_collection)
    if not shared_collections:
        return [active]

    objects = []
    for obj in view_layer.objects:
        if _is_hidden(obj, view_layer):
            continue
        if shared_collections.intersection(obj.users_collection):
            objects.append(obj)
    return objects


def _selection_state(context):
    """Return a cheap state key for the current view-layer selection."""
    view_layer = context.view_layer
    active = view_layer.objects.active
    active_pointer = active.as_pointer() if active is not None else 0
    return (
        view_layer.as_pointer(),
        active_pointer,
        len(view_layer.objects.selected),
    )


def _remember_selection_state(context):
    global _last_selection_state
    try:
        _last_selection_state = _selection_state(context)
    except (AttributeError, ReferenceError, RuntimeError):
        _last_selection_state = None


def select_active_collection(context):
    """Select the active object's shared-collection group.

    This mirrors Object Mode's Select Grouped > Collection behavior: the
    current selection is replaced by visible objects that share at least one
    collection with the active object.
    """
    if context is None or context.mode != 'OBJECT':
        return 0

    view_layer = context.view_layer
    active = view_layer.objects.active
    if active is None or _is_hidden(active, view_layer):
        return 0

    group = _shared_collection_objects(active, view_layer)
    group_set = set(group)

    global _selection_guard
    if _selection_guard:
        return len(group)

    _selection_guard = True
    try:
        for obj in view_layer.objects:
            if not _is_hidden(obj, view_layer):
                obj.select_set(obj in group_set)
        view_layer.objects.active = active
    finally:
        _selection_guard = False

    _remember_selection_state(context)
    return len(group)


@persistent
def _selection_update_handler(_scene, _depsgraph):
    """Notice selection changes, including clicking empty viewport space."""
    global _last_selection_state

    if _selection_guard:
        return

    context = bpy.context
    try:
        state = _selection_state(context)
    except (AttributeError, ReferenceError, RuntimeError):
        return

    if state == _last_selection_state:
        return
    _last_selection_state = state

    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        return
    if context.mode != 'OBJECT':
        return

    view_layer = context.view_layer
    active = view_layer.objects.active
    if active is None or len(view_layer.objects.selected) == 0:
        return
    try:
        if not active.select_get(view_layer=view_layer):
            return
    except (AttributeError, ReferenceError, RuntimeError):
        return

    _schedule_selection()


def _apply_pending_selection():
    """Apply selection after Blender finishes the click that changed it."""
    global _selection_timer_pending
    _selection_timer_pending = False

    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not getattr(scene, PROPERTY_NAME, False):
        return None

    view_layer = context.view_layer
    active = view_layer.objects.active
    if active is None or len(view_layer.objects.selected) == 0:
        return None
    try:
        if not active.select_get(view_layer=view_layer):
            return None
    except (AttributeError, ReferenceError, RuntimeError):
        return None

    select_active_collection(context)
    return None


def _schedule_selection():
    """Queue one selection update for the next Blender event-loop tick."""
    global _selection_timer_pending
    if _selection_timer_pending:
        return

    _selection_timer_pending = True
    try:
        bpy.app.timers.register(_apply_pending_selection, first_interval=0.0)
    except (RuntimeError, ValueError):
        _selection_timer_pending = False


def _enabled_update(scene, context):
    """Queue an update when the feature is enabled in the panel."""
    if getattr(scene, PROPERTY_NAME, False):
        _schedule_selection()


@persistent
def _reset_selection_state_after_file_load(_dummy):
    """Discard pointers from the previous file after a file load."""
    global _last_selection_state
    _last_selection_state = None


def _draw_select_menu(self, context):
    """Add the automatic collection-selection toggle to Blender's Select menu."""
    self.layout.separator()
    self.layout.prop(
        context.scene,
        PROPERTY_NAME,
        text="Select Collection on Click",
    )


def register():
    global _last_selection_state
    bpy.types.Scene.rcad_auto_select_collection = BoolProperty(
        name="Select Collection on Click",
        description="Automatically select visible objects sharing a collection with the active object",
        default=True,
        update=_enabled_update,
    )
    bpy.types.VIEW3D_MT_select_object.append(_draw_select_menu)
    if _selection_update_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_selection_update_handler)
    if _reset_selection_state_after_file_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_reset_selection_state_after_file_load)
    _last_selection_state = None


def unregister():
    global _last_selection_state, _selection_timer_pending
    try:
        bpy.types.VIEW3D_MT_select_object.remove(_draw_select_menu)
    except (AttributeError, ValueError):
        pass
    try:
        bpy.app.timers.unregister(_apply_pending_selection)
    except (RuntimeError, ValueError):
        pass
    _selection_timer_pending = False

    if _selection_update_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_selection_update_handler)
    if _reset_selection_state_after_file_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_reset_selection_state_after_file_load)
    _last_selection_state = None

    if hasattr(bpy.types.Scene, PROPERTY_NAME):
        delattr(bpy.types.Scene, PROPERTY_NAME)


if __name__ == "__main__":
    register()
