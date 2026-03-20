# operator_arc_1pt.py

import bpy
from .modal_core import begin_modal, modal_arc_common, finish_modal

class VIEW3D_OT_arc_overlay_preview(bpy.types.Operator):
    bl_idname = "view3d.arc_overlay_preview"
    bl_label = "Arc Tool (Rotating Compass + Element Snap)"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, ctx, ev):
        return begin_modal(self, ctx, ev)

    def modal(self, ctx, ev):
        return modal_arc_common(self, ctx, ev)

    def finish(self, ctx):
        finish_modal(self, ctx)