# __init__.py

bl_info = {
    "name": "Super Fuse",
    "author": "GPT-5 (refactor)",
    "version": (1, 0, 1),
    "blender": (4, 0, 0),
    "location": "3D Viewport > Sidebar (N) > OSC > Fuse Geometry",
    "description": "Unified geometry fusion tools: X crossings, T junctions, L endpoints, and verts-to-edges from one panel.",
    "category": "Mesh",
}

from importlib import import_module

_registered_modules = []  # registration order (UI first)


def _import_required(module_name: str):
    try:
        return import_module(f".{module_name}", package=__name__)
    except Exception as ex:
        raise ImportError(f"[Super Fuse] Required module missing: {module_name} ({ex})")


def _import_optional(module_name: str):
    try:
        return import_module(f".{module_name}", package=__name__)
    except Exception as ex:
        print(f"[Super Fuse] Optional module not found: {module_name} ({ex})")
        return None


def register():
    global _registered_modules
    if _registered_modules:
        return

    # Required modules
    utils = _import_required("utils")
    ui = _import_required("ui")
    x_weld_op = _import_required("x_weld_op")
    t_weld_op = _import_required("t_weld_op")
    l_weld_op = _import_required("l_weld_op")
    vert_weld_op = _import_required("vert_weld_op")
    heavy_weld_op = _import_required("heavy_weld_op")
    face_weld_op = _import_required("face_weld_op")
    execute_weld = _import_required("execute_weld")  # simple executor only

    # Optional helpers used directly by ops
    _import_optional("x_weld_brute_force")
    _import_optional("deselect_manager")  # new module (no register/unregister needed)

    # Defensive: if the old complex executor is present, try to unregister it
    try:
        old_exec = _import_optional("execute_weld_op")
        if old_exec and hasattr(old_exec, "unregister"):
            old_exec.unregister()
    except Exception:
        pass

    to_register = [ui, x_weld_op, t_weld_op, l_weld_op, vert_weld_op, heavy_weld_op, face_weld_op, execute_weld]
    for mod in to_register:
        if hasattr(mod, "register"):
            mod.register()
            _registered_modules.append(mod)


def unregister():
    global _registered_modules
    if not _registered_modules:
        return
    for mod in reversed(_registered_modules):
        try:
            if hasattr(mod, "unregister"):
                mod.unregister()
        except Exception as ex:
            print(f"[Super Fuse] Unregister warning for {mod.__name__}: {ex}")
    _registered_modules = []


if __name__ == "__main__":
    register()