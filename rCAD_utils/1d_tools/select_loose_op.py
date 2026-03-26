# select_loose_op.py — Select Loose Parts operator (ported from 1D_Lite)

import bpy


class RCAD_OT_SelectLoose(bpy.types.Operator):
    """Select loose parts: edges, vertices, n-gons, triangles, zero-area faces"""
    bl_idname = "rcad.select_loose"
    bl_label = "Find Loose Parts"

    selected_show = False
    selected_hide = False

    def make_edges(self, edges):
        for e in edges:
            if e.is_loose:
                return True
        return False

    def make_indeces(self, list_data, vertices):
        for e in list_data:
            for i in e.vertices:
                vertices.add(i)

    def make_areas(self, pols):
        config = bpy.context.window_manager.rcad_1d_props
        zerop = config.fedge_zerop
        three = config.fedge_three
        WRONG_AREA = config.fedge_WRONG_AREA
        for p in pols:
            if p.area <= WRONG_AREA and zerop:
                return True
            if len(p.vertices) == 3 and three:
                return True
        return False

    def verts(self, obj, selected_hide, selected_show):
        config = bpy.context.window_manager.rcad_1d_props
        if not config.fedge_verts:
            return selected_show, selected_hide
        bpy.ops.mesh.select_mode(type='VERT')
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.object.editmode_toggle()
        vertices = set()
        self.make_indeces(obj.data.edges, vertices)
        self.make_indeces(obj.data.polygons, vertices)
        for i, ver in enumerate(obj.data.vertices):
            if i not in vertices and not ver.hide:
                ver.select = True
                selected_show = True
            elif i not in vertices and ver.hide:
                selected_hide = True
        bpy.ops.object.editmode_toggle()
        return selected_show, selected_hide

    def edges(self, obj, selected_hide, selected_show):
        config = bpy.context.window_manager.rcad_1d_props
        if not config.fedge_edges:
            return selected_show, selected_hide
        if not selected_show:
            bpy.ops.mesh.select_mode(type='EDGE')
            bpy.ops.mesh.select_all(action='DESELECT')
            bpy.ops.object.editmode_toggle()
            for edg in obj.data.edges:
                if edg.is_loose and not edg.hide:
                    edg.select = True
                    selected_show = True
                elif edg.is_loose and edg.hide:
                    selected_hide = True
            bpy.ops.object.editmode_toggle()
        return selected_show, selected_hide

    def zero(self, obj, selected_hide, selected_show):
        config = bpy.context.window_manager.rcad_1d_props
        WRONG_AREA = config.fedge_WRONG_AREA
        if not config.fedge_zerop:
            return selected_show, selected_hide
        if not selected_show:
            bpy.ops.mesh.select_mode(type='FACE')
            bpy.ops.mesh.select_all(action='DESELECT')
            bpy.ops.object.editmode_toggle()
            for pol in obj.data.polygons:
                if pol.area <= WRONG_AREA and not pol.hide:
                    pol.select = True
                    selected_show = True
                elif pol.area <= WRONG_AREA and pol.hide:
                    selected_hide = True
            bpy.ops.object.editmode_toggle()
        return selected_show, selected_hide

    def three(self, obj, selected_hide, selected_show):
        config = bpy.context.window_manager.rcad_1d_props
        if not config.fedge_three:
            return selected_show, selected_hide
        if not selected_show:
            bpy.ops.mesh.select_mode(type='FACE')
            bpy.ops.mesh.select_all(action='DESELECT')
            bpy.ops.object.editmode_toggle()
            for pol in obj.data.polygons:
                if len(pol.vertices) > 4 and not pol.hide:
                    pol.select = True
                    selected_show = True
                elif len(pol.vertices) > 4 and pol.hide:
                    selected_hide = True
            bpy.ops.object.editmode_toggle()
        return selected_show, selected_hide

    def tris(self, obj, selected_hide, selected_show):
        config = bpy.context.window_manager.rcad_1d_props
        if not config.fedge_tris:
            return selected_show, selected_hide
        if not selected_show:
            bpy.ops.mesh.select_mode(type='FACE')
            bpy.ops.mesh.select_all(action='DESELECT')
            bpy.ops.object.editmode_toggle()
            for pol in obj.data.polygons:
                if len(pol.vertices) == 3 and not pol.hide:
                    pol.select = True
                    selected_show = True
                elif len(pol.vertices) == 3 and pol.hide:
                    selected_hide = True
            bpy.ops.object.editmode_toggle()
        return selected_show, selected_hide

    def select_loose_objt(self):
        config = bpy.context.window_manager.rcad_1d_props
        objects = bpy.context.selected_objects

        if not objects:
            self.report({'ERROR'}, 'No objects selected. Select objects or enter edit mode.')
            return

        bpy.ops.object.select_all(action='DESELECT')

        for obj in objects:
            if obj.type != 'MESH':
                continue
            data = obj.data
            should_select = False
            # loose verts
            if config.fedge_verts:
                vertices = set()
                self.make_indeces(data.edges, vertices)
                self.make_indeces(data.polygons, vertices)
                v = set(range(len(data.vertices)))
                if v.difference(vertices):
                    should_select = True
            # zero area
            if config.fedge_zerop:
                if self.make_areas(data.polygons):
                    should_select = True
            # loose edges
            if config.fedge_edges:
                if self.make_edges(data.edges):
                    should_select = True
            # triangles
            if config.fedge_three:
                for p in data.polygons:
                    if len(p.vertices) == 3:
                        should_select = True
                        break
            if should_select:
                obj.select_set(True)

    def select_loose_edit(self):
        obj = bpy.context.active_object
        selected_show = False
        selected_hide = False

        mess_info = ''
        selected_show, selected_hide = self.verts(obj, selected_hide, selected_show)
        if selected_show:
            mess_info = 'verts'

        selected_show, selected_hide = self.edges(obj, selected_hide, selected_show)
        if selected_show and not mess_info:
            mess_info = 'edges'

        selected_show, selected_hide = self.three(obj, selected_hide, selected_show)
        if selected_show and not mess_info:
            mess_info = 'ngons'

        selected_show, selected_hide = self.tris(obj, selected_hide, selected_show)
        if selected_show and not mess_info:
            mess_info = 'tris'

        selected_show, selected_hide = self.zero(obj, selected_hide, selected_show)
        if selected_show and not mess_info:
            mess_info = 'zero area'

        if mess_info:
            self.report({'INFO'}, 'Select Loose: ' + mess_info)
        elif selected_hide:
            self.report({'INFO'}, 'Select Loose: Nothing found (but hidden)')
        else:
            bpy.ops.object.editmode_toggle()
            self.report({'INFO'}, 'Select Loose: Your object is clean')

    def execute(self, context):
        if bpy.context.mode == 'OBJECT':
            self.select_loose_objt()
        elif bpy.context.mode == 'EDIT_MESH':
            self.select_loose_edit()
        return {'FINISHED'}


classes = (RCAD_OT_SelectLoose,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
