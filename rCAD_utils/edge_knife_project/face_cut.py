"""Cut the active connected mesh island using selected intersecting faces."""

import bmesh
import bpy

from .operators import _active_edit_mesh, _clear_selection


def _target_island(bm):
    active = bm.select_history.active
    if 'FACE' in bm.select_mode and bm.faces.active is not None and bm.faces.active.select:
        active = bm.faces.active
    if active is None or not active.select:
        raise ValueError("Select a target vertex, edge, or face last to make it active.")
    seeds = [active] if isinstance(active, bmesh.types.BMVert) else active.verts
    verts = set(seeds)
    pending = list(verts)
    while pending:
        vert = pending.pop()
        for edge in vert.link_edges:
            other = edge.other_vert(vert)
            if other not in verts:
                verts.add(other)
                pending.append(other)
    return {face for vert in verts for face in vert.link_faces}


class MESH_OT_RCAD_CutByFace(bpy.types.Operator):
    bl_idname = "mesh.rcad_cut_by_face"
    bl_label = "Cut by Face"
    bl_description = (
        "Cut the connected mesh containing the active vertex, edge, or face "
        "with the other selected faces; keep the cutter faces intact"
    )
    bl_options = {'REGISTER', 'UNDO'}

    separate_split: bpy.props.BoolProperty(
        name="Separate Split", default=True,
        description="Disconnect intersection seams within the target mesh",
    )

    @classmethod
    def poll(cls, context):
        return _active_edit_mesh(context) is not None

    def execute(self, context):
        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        try:
            target = _target_island(bm)
        except ValueError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        # Multi-object Edit Mode may put each selected plane in a different
        # mesh. Snapshot every cutter in target-local space before editing.
        cutters = []
        inverse = obj.matrix_world.inverted_safe()
        for source in context.objects_in_mode_unique_data:
            if source.type != 'MESH':
                continue
            source_bm = bmesh.from_edit_mesh(source.data)
            transform = inverse @ source.matrix_world
            cutters.extend(
                [transform @ vert.co for vert in face.verts]
                for face in source_bm.faces
                if face.select and not face.hide
                and (source.data != obj.data or face not in target)
            )
        if not target or not cutters:
            self.report({'ERROR'}, "Select cutter faces, then select a target mesh element last. Cutters must be disconnected from the target.")
            return {'CANCELLED'}

        # Intersect copies so the selected cutter faces are never modified.
        # Custom data follows subdivided faces and identifies the copies for cleanup.
        bm.faces.index_update()
        target_indices = {f.index for f in target}
        cutter_count = len(cutters)
        tag = bm.faces.layers.int.new("__rcad_face_cut_role")
        target = {f for f in bm.faces if f.index in target_indices}
        tag_name = tag.name
        hidden = [(element, element.hide) for elements in (bm.verts, bm.edges, bm.faces)
                  for element in elements]
        for face in bm.faces:
            face[tag] = 1 if face in target and not face.hide else 0
        for points in cutters:
            face = bm.faces.new([bm.verts.new(point) for point in points])
            face[tag] = 2
        _clear_selection(bm)
        for face in bm.faces:
            face.hide_set(face[tag] == 0)
        for face in bm.faces:
            if face[tag] in {1, 2}:
                face.hide_set(False)
                face.select_set(face[tag] == 2)
        bmesh.update_edit_mesh(obj.data)
        # Intersect operates on every object in Edit Mode. Hide the external
        # source geometry temporarily so it cannot cut or change itself.
        other_states = []
        for source in context.objects_in_mode_unique_data:
            if source.type != 'MESH' or source.data == obj.data:
                continue
            other_bm = bmesh.from_edit_mesh(source.data)
            state = [[(element.hide, element.select) for element in elements]
                     for elements in (other_bm.verts, other_bm.edges, other_bm.faces)]
            other_states.append((source, state))
            for face in other_bm.faces:
                face.hide_set(True)
            bmesh.update_edit_mesh(source.data)
        try:
            bpy.ops.mesh.intersect(
                mode='SELECT_UNSELECT',
                separate_mode='ALL' if self.separate_split else 'CUT',
                solver='EXACT',
            )
        finally:
            for source, state in other_states:
                other_bm = bmesh.from_edit_mesh(source.data)
                for elements, values in zip((other_bm.verts, other_bm.edges, other_bm.faces), state):
                    for element, (was_hidden, was_selected) in zip(elements, values):
                        element.hide = was_hidden
                        element.select = was_selected
                bmesh.update_edit_mesh(source.data)
            bm = bmesh.from_edit_mesh(obj.data)
            tag = bm.faces.layers.int.get(tag_name)
            bmesh.ops.delete(bm, geom=[f for f in bm.faces if f[tag] == 2], context='FACES')
            for element, was_hidden in hidden:
                if element.is_valid:
                    element.hide = was_hidden
            bm.faces.layers.int.remove(tag)
            _clear_selection(bm)
            bmesh.update_edit_mesh(obj.data, destructive=True)
        self.report({'INFO'}, f"Cut target with {cutter_count} face(s).")
        return {'FINISHED'}


classes = (MESH_OT_RCAD_CutByFace,)
