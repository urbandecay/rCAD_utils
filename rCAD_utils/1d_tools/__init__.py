# 1d_tools — Ported from 1D_Lite addon
# Features: Spread Loop, Create B-Arc, Corner Extend, Select Loose Parts

import importlib

_modules = []


def _import_required(name):
    mod = importlib.import_module(f".{name}", package=__name__)
    _modules.append(mod)
    return mod


def register():
    _modules.clear()
    ui = _import_required("ui")
    spread_op = _import_required("spread_op")
    barc_op = _import_required("barc_op")
    corner_extend_op = _import_required("corner_extend_op")
    select_loose_op = _import_required("select_loose_op")

    # Register UI first (property group + panels), then operators
    for mod in [ui, spread_op, barc_op, corner_extend_op, select_loose_op]:
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
