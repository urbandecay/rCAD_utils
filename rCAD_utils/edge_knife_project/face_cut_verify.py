"""Run with Blender --background --factory-startup --python this_file.py."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import bpy,bmesh
from rCAD_utils.edge_knife_project import face_cut
bpy.utils.register_class(face_cut.MESH_OT_RCAD_CutByFace)
for split in (False,True):
 for kind in ('VERT','EDGE','FACE'):
  bpy.ops.object.select_all(action='SELECT')
  bpy.ops.object.delete(use_global=False)
  mesh=bpy.data.meshes.new('test'); obj=bpy.data.objects.new('test',mesh)
  bpy.context.collection.objects.link(obj); obj.select_set(True); bpy.context.view_layer.objects.active=obj
  bm=bmesh.new(); bmesh.ops.create_cube(bm,size=2)
  for x in (-.4,.4):
   vs=[bm.verts.new((x,y,z)) for y,z in ((-2,-2),(2,-2),(2,2),(-2,2))]
   bm.faces.new(vs)
  bm.to_mesh(mesh);bm.free();bpy.ops.object.mode_set(mode='EDIT')
  bm=bmesh.from_edit_mesh(mesh)
  for f in bm.faces: f.select_set(True)
  bm.verts.ensure_lookup_table(); bm.edges.ensure_lookup_table(); bm.faces.ensure_lookup_table()
  active={'VERT':bm.verts[0],'EDGE':bm.edges[0],'FACE':bm.faces[0]}[kind]
  bm.select_history.add(active)
  result=bpy.ops.mesh.rcad_cut_by_face(separate_split=split)
  bm=bmesh.from_edit_mesh(mesh)
  print(split,kind,result,len(bm.faces),len(bm.verts))
  assert len(bm.faces)==16, len(bm.faces)
  assert sum(1 for f in bm.faces if len(f.verts)==4 and abs(f.calc_area()-16)<1e-5)==2
  unseen=set(bm.verts); components=0
  while unseen:
   stack=[unseen.pop()];components+=1
   while stack:
    for e in stack.pop().link_edges:
     for v in e.verts:
      if v in unseen: unseen.remove(v); stack.append(v)
  assert components==(5 if split else 3),components
  bpy.ops.object.mode_set(mode='OBJECT')
print('FACE_CUT_OK')
