# -*- coding:utf-8 -*-

# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110- 1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>

# ----------------------------------------------------------
# Author: Stephen Leger (s-leger)
#
# ----------------------------------------------------------


bl_info = {
    'name': 'CAD Transform',
    'description': 'Cad like transform',
    'author': '<s-leger> support@blender-archipack.org',
    'license': 'GPL',
    'deps': '',
    'blender': (3, 0, 0),
    'version': (0, 93, 2),
    'location': 'View3D > Tools > Cad',
    'warning': '',
    'wiki_url': 'https://github.com/s-leger/blender_cad_transforms/wiki',
    'tracker_url': 'https://github.com/s-leger/blender_cad_transforms/issues',
    'link': 'https://github.com/s-leger/blender_cad_transforms',
    'support': 'COMMUNITY',
    'category': '3D View'
    }

__author__ = bl_info['author']
__version__ = ".".join(map(str, bl_info['version']))


import bpy

_register_impl = None
_unregister_impl = None
_load_error = None


def _load():
    global _register_impl, _unregister_impl, _load_error

    if _register_impl is not None or _load_error is not None:
        return

    # The viewport GPU context is unavailable in background Blender runs.
    # Keep rCAD Utils importable for automated checks and headless usage.
    if bpy.app.background:
        return

    try:
        from .slcad_transform import register as register_impl
        from .slcad_transform import unregister as unregister_impl
    except Exception as exc:
        _load_error = exc
        print("{} {}: disabled ({})".format(bl_info['name'], __version__, exc))
        return

    _register_impl = register_impl
    _unregister_impl = unregister_impl


def register():
    _load()
    if _register_impl is not None:
        try:
            _register_impl()
        except Exception as exc:
            print("{} {}: registration failed ({})".format(bl_info['name'], __version__, exc))


def unregister():
    if _unregister_impl is not None:
        try:
            _unregister_impl()
        except Exception as exc:
            print("{} {}: unregistration failed ({})".format(bl_info['name'], __version__, exc))


if __name__ == "__main__":
    register()
