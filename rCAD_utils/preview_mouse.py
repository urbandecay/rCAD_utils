"""Read held pointer buttons on the X11 desktop, including stationary drags."""

import ctypes as ct
import os


_xlib = None
_display = None
_attempted = False


def buttons_held():
    """Return True while any primary pointer button is physically held."""
    global _xlib, _display, _attempted

    if not _attempted:
        _attempted = True
        if not os.environ.get("DISPLAY"):
            return False
        try:
            _xlib = ct.CDLL("libX11.so.6")
            _xlib.XOpenDisplay.argtypes = [ct.c_char_p]
            _xlib.XOpenDisplay.restype = ct.c_void_p
            _xlib.XDefaultRootWindow.argtypes = [ct.c_void_p]
            _xlib.XDefaultRootWindow.restype = ct.c_ulong
            _xlib.XQueryPointer.argtypes = [
                ct.c_void_p,
                ct.c_ulong,
                ct.POINTER(ct.c_ulong),
                ct.POINTER(ct.c_ulong),
                ct.POINTER(ct.c_int),
                ct.POINTER(ct.c_int),
                ct.POINTER(ct.c_int),
                ct.POINTER(ct.c_int),
                ct.POINTER(ct.c_uint),
            ]
            _xlib.XQueryPointer.restype = ct.c_int
            _xlib.XCloseDisplay.argtypes = [ct.c_void_p]
            _display = _xlib.XOpenDisplay(None)
        except OSError:
            return False

    if not _display:
        return False

    root = ct.c_ulong()
    child = ct.c_ulong()
    root_x = ct.c_int()
    root_y = ct.c_int()
    win_x = ct.c_int()
    win_y = ct.c_int()
    mask = ct.c_uint()
    _xlib.XQueryPointer(
        _display,
        _xlib.XDefaultRootWindow(_display),
        ct.byref(root),
        ct.byref(child),
        ct.byref(root_x),
        ct.byref(root_y),
        ct.byref(win_x),
        ct.byref(win_y),
        ct.byref(mask),
    )
    return bool(mask.value & ((1 << 8) | (1 << 9) | (1 << 10)))


def close():
    """Release the X11 connection when the addon is disabled."""
    global _display, _attempted

    if _display:
        _xlib.XCloseDisplay(_display)
    _display = None
    _attempted = False
