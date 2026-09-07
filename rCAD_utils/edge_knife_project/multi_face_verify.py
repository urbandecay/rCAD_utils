"""Background Blender regression for five cutters in multi-object Edit Mode."""
import sys
from itertools import product
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import bpy,bmesh
from rCAD_utils.edge_knife_project import face_cut
bpy.utils.register_class(face_cut.MESH_OT_RCAD_CutByFace)
for solver, split, fill, open_end in product(('EXACT', 'FLOAT'), (False, True), (False, True), (False, True)):
 bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
 bpy.ops.mesh.primitive_cube_add();cube=bpy.context.object
 if open_end:
  bm=bmesh.new();bm.from_mesh(cube.data)
  end=next(face for face in bm.faces if face.normal.x > 0.9)
  bmesh.ops.delete(bm, geom=[end], context='FACES_ONLY')
  bm.to_mesh(cube.data);bm.free()
 planes=[]
 for x in (-.8,-.4,0,.4,.8):
  mesh=bpy.data.meshes.new('cutter');mesh.from_pydata([(0,-2,-2),(0,2,-2),(0,2,2),(0,-2,2)],[],[(0,1,2,3)])
  ob=bpy.data.objects.new('cutter',mesh);bpy.context.collection.objects.link(ob);ob.location.x=x;ob.select_set(True);planes.append(ob)
 cube.select_set(True);bpy.context.view_layer.objects.active=cube
 bpy.ops.object.mode_set(mode='EDIT');bpy.context.tool_settings.mesh_select_mode=(False,False,True)
 for ob in [cube]+planes:
  bm=bmesh.from_edit_mesh(ob.data)
  for f in bm.faces:f.select_set(True)
 bm=bmesh.from_edit_mesh(cube.data);bm.faces.ensure_lookup_table();bm.faces.active=bm.faces[0]
 assert bpy.ops.mesh.rcad_cut_by_face(separate_split=split, solver=solver, fill_faces=fill)=={'FINISHED'}
 bm=bmesh.from_edit_mesh(cube.data)
 assert len(bm.faces)==(36 if split and fill else 26-int(open_end)),len(bm.faces)
 if (fill and split) or (not split and not open_end):
  assert all(edge.is_manifold and edge.is_contiguous for edge in bm.edges), 'open or reversed cap'
 unseen=set(bm.verts);count=0
 while unseen:
  todo=[unseen.pop()];count+=1
  while todo:
   for e in todo.pop().link_edges:
    for v in e.verts:
     if v in unseen:unseen.remove(v);todo.append(v)
 assert count==(6 if split else 1),count
 for ob in planes:
  bm=bmesh.from_edit_mesh(ob.data)
  assert len(bm.faces)==1 and len(bm.verts)==4
  assert all(f.select and not f.hide for f in bm.faces)
 bpy.ops.object.mode_set(mode='OBJECT')
print('MULTI_OBJECT_FIVE_CUTTERS_OK')
