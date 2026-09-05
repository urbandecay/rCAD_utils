from collections import defaultdict

import bmesh
import bpy
from mathutils import Vector

from .utils import (
    EPS,
    PARAM_EPS,
    aabb_for_segment,
    aabb_overlaps,
    closest_point_on_segment,
    edge_intersection,
    split_edge_at_cuts,
)


def _segment_data(edge, matrix_world):
    first = matrix_world @ edge.verts[0].co
    second = matrix_world @ edge.verts[1].co
    return {
        "edge": edge,
        "p0": first,
        "p1": second,
    }


def _is_interior_factor(factor):
    return PARAM_EPS < float(factor) < 1.0 - PARAM_EPS


def _add_cut(cuts_by_edge, edge, factor, local_point, edge_length):
    factor = float(factor)
    if not _is_interior_factor(factor):
        return

    # Several edges can meet the target at the same T/X point. Store one cut.
    factor_tolerance = max(PARAM_EPS, EPS / max(edge_length, EPS))
    for old_factor, old_point in cuts_by_edge[edge]:
        if abs(old_factor - factor) <= factor_tolerance or (old_point - local_point).length <= EPS:
            return
    cuts_by_edge[edge].append((factor, Vector(local_point)))


def _collect_edge_cuts(obj, selected_edges, all_edges, tolerance):
    matrix_world = obj.matrix_world.copy()
    inverse_world = matrix_world.inverted_safe()
    segments = [
        _segment_data(edge, matrix_world)
        for edge in all_edges
        if edge.is_valid and not edge.hide
    ]
    target_segments = {
        item["edge"]: item
        for item in segments
        if item["edge"] in selected_edges
    }
    cuts_by_edge = defaultdict(list)

    for target_edge in selected_edges:
        target = target_segments.get(target_edge)
        if target is None:
            continue

        target_length = (target["p1"] - target["p0"]).length
        padded_bounds = aabb_for_segment(target["p0"], target["p1"], tolerance)
        for cutter in segments:
            if cutter["edge"] is target_edge:
                continue
            if not aabb_overlaps(padded_bounds, aabb_for_segment(cutter["p0"], cutter["p1"], tolerance)):
                continue

            hit = edge_intersection(
                target["p0"], target["p1"],
                cutter["p0"], cutter["p1"],
                tolerance,
            )
            if hit is None:
                continue

            target_factor, hit_world = hit
            if not _is_interior_factor(target_factor):
                continue
            hit_local = inverse_world @ hit_world
            _add_cut(cuts_by_edge, target_edge, target_factor, hit_local, target_length)

    return cuts_by_edge


def _collect_vertex_cuts(obj, selected_edges, all_verts, tolerance, cuts_by_edge):
    matrix_world = obj.matrix_world.copy()
    inverse_world = matrix_world.inverted_safe()

    for edge in selected_edges:
        if not edge.is_valid or edge.hide:
            continue
        p0 = matrix_world @ edge.verts[0].co
        p1 = matrix_world @ edge.verts[1].co
        edge_length = (p1 - p0).length

        for vert in all_verts:
            if not vert.is_valid or vert.hide or vert in edge.verts:
                continue
            vert_world = matrix_world @ vert.co
            closest, factor = closest_point_on_segment(p0, p1, vert_world)
            if not _is_interior_factor(factor):
                continue
            if (closest - vert_world).length > max(float(tolerance), EPS):
                continue
            _add_cut(
                cuts_by_edge,
                edge,
                factor,
                inverse_world @ vert_world,
                edge_length,
            )


class MESH_OT_SplitToolsExecute(bpy.types.Operator):
    bl_idname = "mesh.split_tools_execute"
    bl_label = "Split"
    bl_description = "Split selected edges at edge intersections and selected vertices"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.edit_object is not None

    def execute(self, context):
        obj = context.edit_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Active object must be a mesh.")
            return {'CANCELLED'}

        props = getattr(context.scene, "split_tools", None)
        if props is None:
            self.report({'ERROR'}, "Split Tools properties not found.")
            return {'CANCELLED'}
        if not props.x and not props.dot:
            self.report({'INFO'}, "Enable X or . before running Split Tools.")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        selected_edges = [edge for edge in bm.edges if edge.select and not edge.hide]
        if not selected_edges:
            self.report({'INFO'}, "Select at least one edge.")
            return {'CANCELLED'}

        all_edges = [edge for edge in bm.edges if not edge.hide]
        tolerance = max(0.0, float(props.search_radius))
        cuts_by_edge = defaultdict(list)

        if props.x:
            for edge, cuts in _collect_edge_cuts(
                obj, selected_edges, all_edges, tolerance
            ).items():
                cuts_by_edge[edge].extend(cuts)

        if props.dot:
            all_verts = [vert for vert in bm.verts if not vert.hide]
            _collect_vertex_cuts(
                obj, selected_edges, all_verts, tolerance, cuts_by_edge
            )

        if not cuts_by_edge:
            self.report({'INFO'}, "No selected edges intersected another edge or selected vertex.")
            return {'CANCELLED'}

        total_cuts = 0
        affected_edges = 0
        split_verts = []
        for edge, cuts in list(cuts_by_edge.items()):
            if not edge or not edge.is_valid:
                continue
            split_count, new_verts = split_edge_at_cuts(bm, edge, cuts)
            if split_count:
                affected_edges += 1
                total_cuts += split_count
                split_verts.extend(new_verts)

        if not total_cuts:
            self.report({'INFO'}, "No valid interior intersections found.")
            return {'CANCELLED'}

        separated_points = 0
        if props.separate:
            separate_edges = set()
            for vert in split_verts:
                if not vert or not vert.is_valid:
                    continue
                linked_edges = [edge for edge in vert.link_edges if edge.is_valid]
                if linked_edges:
                    separate_edges.update(linked_edges)
                    separated_points += 1
            if separate_edges:
                bmesh.ops.split_edges(bm, edges=list(separate_edges))

        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=True)
        separate_suffix = f"; separated at {separated_points} point(s)" if props.separate else ""
        self.report(
            {'INFO'},
            f"Split Tools: {total_cuts} cut(s) across {affected_edges} selected edge(s)"
            f"{separate_suffix}.",
        )
        return {'FINISHED'}


classes = (MESH_OT_SplitToolsExecute,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
