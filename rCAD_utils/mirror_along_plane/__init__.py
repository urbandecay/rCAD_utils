# bl_info needs to be at the top
bl_info = {
    "name": "Mirror Across Plane",
    "author": "RobbieK",
    "version": (1, 4, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > OSC",
    "description": ("Stores a planar selection, mirrors geometry across it, and provides all options in the Redo panel."),
    "warning": "",
    "doc_url": "",
    "category": "Mesh",
}

import bpy
import bmesh
from mathutils import Vector, Matrix
import numpy as np
from bpy.props import BoolProperty


class VertexStorage:
    _instance = None

    def __init__(self):
        self._stored_verts_local = None
        self._stored_matrix_world = None
        self.plane_status = "None"

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = VertexStorage()
        return cls._instance

    def store_geometry(self, vertices_local, matrix_world):
        if vertices_local and matrix_world:
            self._stored_verts_local = [v.copy() for v in vertices_local]
            self._stored_matrix_world = matrix_world.copy()
            self.plane_status = "Stored"
        else:
            self.fail_plane_storage()

    def fail_plane_storage(self):
        self._stored_verts_local = None
        self._stored_matrix_world = None
        self.plane_status = "Error"

    def has_geometry(self):
        return self._stored_verts_local is not None

    def get_stored_verts(self):
        return [v.copy() for v in self._stored_verts_local] if self._stored_verts_local else None

    def get_stored_matrix(self):
        return self._stored_matrix_world.copy() if self._stored_matrix_world else None

    def clear(self):
        self._stored_verts_local = None
        self._stored_matrix_world = None
        self.plane_status = "None"


class MESH_OT_store_plane_vertices(bpy.types.Operator):
    """Store the current planar selection as geometry for dynamic mirroring"""
    bl_idname = "mesh.store_plane_vertices"
    bl_label = "Store"
    bl_options = {'REGISTER', 'UNDO'}

    EPSILON = 1e-5

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode == 'EDIT'

    def execute(self, context):
        obj = context.edit_object
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        bm.verts.ensure_lookup_table()
        storage = VertexStorage.get_instance()

        sel_verts_local = [v.co.copy() for v in bm.verts if v.select]
        num_verts = len(sel_verts_local)
        
        if num_verts < 2:
            self.report({'WARNING'}, "Select at least 2 vertices to define a plane/edge.")
            storage.fail_plane_storage()
            return {'CANCELLED'}

        world_mat = obj.matrix_world.copy()

        # Planarity check is only needed for 3 or more vertices
        if num_verts >= 3:
            world_coords = [world_mat @ co for co in sel_verts_local]
            
            # Use PCA to find the best-fit plane normal
            center = sum(world_coords, Vector()) / num_verts
            points = np.array([(v - center) for v in world_coords])
            try:
                covariance_matrix = np.cov(points.T)
                eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
                normal_world = Vector(eigenvectors[:, np.argmin(eigenvalues)]).normalized()
            except Exception as e:
                self.report({'WARNING'}, f"Could not analyze selection: {e}")
                storage.fail_plane_storage()
                return {'CANCELLED'}

            # Check if all points are on the calculated plane within the tolerance
            is_planar = all(abs((v - center).dot(normal_world)) < self.EPSILON for v in world_coords)
            
            if not is_planar:
                self.report({'WARNING'}, "Selection is not planar. Geometry not stored.")
                storage.fail_plane_storage()
                return {'CANCELLED'}

        storage.store_geometry(sel_verts_local, world_mat)
        self.report({'INFO'}, "Planar geometry stored successfully!")
        return {'FINISHED'}


class MESH_OT_point_reflection(bpy.types.Operator):
    """Duplicate and mirror the selected geometry dynamically across the stored plane"""
    bl_idname = "mesh.reflect_across_plane"
    bl_label = "Mirror"
    bl_options = {'REGISTER', 'UNDO'}

    mirror_along_edge: BoolProperty(
        name="Mirror Along Edge",
        description="If enabled, use the stored edge's direction as the plane normal",
        default=False,
    )
    erase_original: BoolProperty(
        name="Erase Original",
        description="Delete the original geometry after mirroring",
        default=False,
    )
    separate_object: BoolProperty(
        name="Separate New Object",
        description="Place the mirrored geometry into a new object",
        default=False,
    )

    EPSILON = 1e-5

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        storage = VertexStorage.get_instance()
        return (obj is not None and obj.type == 'MESH' and obj.mode == 'EDIT'
                and storage.has_geometry())

    def calculate_normal_pca(self, vertices_world):
        if len(vertices_world) < 3:
            raise ValueError("Need at least 3 vertices for PCA")
        center = sum(vertices_world, Vector()) / len(vertices_world)
        points = np.array([ (v - center) for v in vertices_world ])
        covariance_matrix = np.cov(points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        normal_vec = eigenvectors[:, np.argmin(eigenvalues)]
        return Vector(normal_vec).normalized()

    def execute(self, context):
        obj = context.edit_object
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        
        storage = VertexStorage.get_instance()
        stored_verts_local = storage.get_stored_verts()
        stored_matrix = storage.get_stored_matrix()
        
        world_coords = [stored_matrix @ co for co in stored_verts_local]
        num_verts = len(world_coords)
        
        origin_world = None
        normal_world = None
        plane_defined = False
        error_message = ""
        
        if num_verts == 2:
            v1_world, v2_world = world_coords
            edge_vec_world = v2_world - v1_world
            if edge_vec_world.length < self.EPSILON:
                error_message = "Stored edge has zero length."
            else:
                if self.mirror_along_edge:
                    normal_world = edge_vec_world.normalized()
                    origin_world = (v1_world + v2_world) / 2.0
                    plane_defined = True
                else:
                    edge_vec_world.normalize()
                    view_mode = context.space_data.region_3d.view_perspective if context.space_data else None
                    if view_mode == 'ORTHO':
                        region3d = context.space_data.region_3d
                        try:
                            view_matrix_inv = region3d.view_matrix.inverted()
                            view_vec = view_matrix_inv.col[2].to_3d().normalized()
                        except Exception:
                            view_vec = None
                            error_message = "Could not determine view vector"
                        if view_vec and (edge_vec_world.cross(view_vec).length > self.EPSILON):
                            normal_world = edge_vec_world.cross(view_vec).normalized()
                            origin_world = (v1_world + v2_world) / 2.0
                            plane_defined = True
                        else:
                            error_message = "Stored edge parallel to view direction"
                    elif view_mode in ('PERSP', 'CAMERA'):
                        world_z = Vector((0.0, 0.0, 1.0))
                        normal_world = edge_vec_world.cross(world_z)
                        if normal_world.length > self.EPSILON:
                            normal_world.normalize()
                            origin_world = (v1_world + v2_world) / 2.0
                            plane_defined = True
                        else:
                            error_message = "Cannot make a plane from stored vertical edge"
                    else:
                        error_message = f"Unsupported view mode: {view_mode}"
        elif num_verts >= 3:
            # Planarity was already verified by the store operator.
            try:
                normal_world = self.calculate_normal_pca(world_coords)
                if normal_world.length > self.EPSILON:
                    origin_world = sum(world_coords, Vector()) / len(world_coords)
                    plane_defined = True
            except Exception as e:
                error_message = f"PCA error on stored geometry: {e}"
        
        if not plane_defined:
             self.report({'WARNING'}, error_message or "Could not define a valid mirror plane.")
             return {'CANCELLED'}

        verts_in_selection = {v for v in bm.verts if v.select}
        if not verts_in_selection:
            self.report({'WARNING'}, "No geometry selected for mirroring.")
            return {'CANCELLED'}
        
        faces_to_mirror = [f for f in bm.faces if f.is_valid and all(v in verts_in_selection for v in f.verts)]
        edges_to_mirror = [e for e in bm.edges if e.is_valid and all(v in verts_in_selection for v in e.verts)]
        if faces_to_mirror:
            geom_to_duplicate = faces_to_mirror
        elif edges_to_mirror:
            geom_to_duplicate = edges_to_mirror
        else:
            geom_to_duplicate = list(verts_in_selection)
        
        ret = bmesh.ops.duplicate(bm, geom=geom_to_duplicate)
        dup_geom = ret['geom']
        dup_verts = [elem for elem in dup_geom if isinstance(elem, bmesh.types.BMVert)]
        
        world_mat = obj.matrix_world
        inv_world_mat = world_mat.inverted_safe()
        origin_local = inv_world_mat @ origin_world
        normal_local = (inv_world_mat.transposed().to_3x3() @ normal_world).normalized()
        
        for v in dup_verts:
            if v.is_valid:
                vec_to_origin = v.co - origin_local
                distance = vec_to_origin.dot(normal_local)
                v.co -= 2 * distance * normal_local
        
        if [f for f in dup_geom if isinstance(f, bmesh.types.BMFace)]:
            bmesh.ops.reverse_faces(bm, faces=[f for f in dup_geom if isinstance(f, bmesh.types.BMFace)])
        
        if self.erase_original:
            try:
                if faces_to_mirror:
                    bmesh.ops.delete(bm, geom=faces_to_mirror, context='FACES')
                elif edges_to_mirror:
                    bmesh.ops.delete(bm, geom=edges_to_mirror, context='EDGES')
                elif verts_in_selection:
                    bmesh.ops.delete(bm, geom=list(verts_in_selection), context='VERTS')
            except Exception as e:
                self.report({'WARNING'}, f"Error deleting original geometry: {e}")
        
        if self.separate_object:
            new_mesh = bpy.data.meshes.new(name=obj.data.name + "_mirror")
            new_obj = bpy.data.objects.new(name=obj.name + "_mirror", object_data=new_mesh)
            context.collection.objects.link(new_obj)
            
            bm_new = bmesh.new()
            vert_map = {v: bm_new.verts.new(v.co) for v in dup_verts}
            bm_new.verts.ensure_lookup_table()
            
            for edge in [e for e in dup_geom if isinstance(e, bmesh.types.BMEdge)]:
                if all(v in vert_map for v in edge.verts):
                    bm_new.edges.new((vert_map[edge.verts[0]], vert_map[edge.verts[1]]))
            
            for face in [f for f in dup_geom if isinstance(f, bmesh.types.BMFace)]:
                 if all(v in vert_map for v in face.verts):
                    bm_new.faces.new(tuple(vert_map[v] for v in face.verts))
            
            bm_new.to_mesh(new_mesh)
            bm_new.free()
            
            bmesh.ops.delete(bm, geom=dup_geom, context='VERTS')
        
        bmesh.update_edit_mesh(mesh)
        self.report({'INFO'}, "Mirror operation complete.")
        return {'FINISHED'}


class MESH_PT_reflect_across_plane_panel(bpy.types.Panel):
    bl_label = "Mirror Across Plane"
    bl_idname = "MESH_PT_reflect_across_plane"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'OSC'

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode == 'EDIT'

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        row = box.row(align=True)
        row.label(text="Plane:")
        row.operator(MESH_OT_store_plane_vertices.bl_idname)
        
        row = box.row(align=True)
        row.operator(MESH_OT_point_reflection.bl_idname)


classes = (
    MESH_OT_store_plane_vertices,
    MESH_OT_point_reflection,
    MESH_PT_reflect_across_plane_panel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    VertexStorage._instance = None

if __name__ == "__main__":
    register()