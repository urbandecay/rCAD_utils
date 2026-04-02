# vertex_resampler — Ported from PolyTangents addon
# Features: Smart vertex resampling for curves, loops, and stacked geometry

import importlib

_modules = []


def _import_required(name):
    mod = importlib.import_module(f".{name}", package=__name__)
    _modules.append(mod)
    return mod


def register():
    _modules.clear()
    anchor_overlay = _import_required("anchor_overlay")
    ui = _import_required("ui")
    operators = _import_required("operators")

    for mod in [anchor_overlay, ui, operators]:
        if hasattr(mod, "register"):
            mod.register()


def unregister():
    for mod in reversed(_modules):
        if hasattr(mod, "unregister"):
            try:
                mod.unregister()
            except Exception:
                pass
    _modules.clear()
