# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 3
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# ##### END GPL LICENSE BLOCK #####

# Contact for more information about the Addon:
# Email:    germano.costa@ig.com.br
# Twitter:  wii_mano @mano_wii

try:
    import bgl
except ImportError:
    # Blender 4.x+ removed the legacy bgl module.  The snap renderer does
    # not otherwise depend on it; error checks simply become unavailable.
    bgl = None
import gpu
import numpy as np
from mathutils.geometry import interpolate_bezier
from mathutils import Matrix
import bmesh
import time

TYP_OBJ = 1
TYP_ORIGIN = 2
TYP_BOUNDS = 4

USE_PRIMITIVE_BUFFER = True


vert_3d = '''
uniform mat4 MVP;

in vec3 pos;
in float primitive_id;
out float primitive_id_var;

void main()
{
    
    primitive_id_var = primitive_id;
    gl_Position = MVP * vec4(pos, 1.0);
    
}
'''


primitive_id_frag = '''
uniform float offset;

in float primitive_id_var;
out vec4 FragColor;

vec4 cast_to_4_bytes(float f){
    vec4 color;
    color.a = float(int(f)%256);
    color.b = float(int(f/256)%256); 
    color.g = float(int(f/65536)%256);
    color.r = float(int(f/16581376)%256);
    return color / 255.0;
}

void main()
{
    FragColor = cast_to_4_bytes(offset + primitive_id_var);
}
'''

gl_version = '''
#version 330
'''

def interp_bezier(p0, p1, segs, resolution):
    """
     Bezier curve approximation
    """
    if (resolution == 0 or
            (p0.handle_right_type == 'VECTOR' and
             p1.handle_left_type == 'VECTOR')):
        segs.append(p0.co[0:3])

    else:
        seg = interpolate_bezier(p0.co,
                                 p0.handle_right,
                                 p1.handle_left,
                                 p1.co,
                                 resolution + 1)
        segs.extend([p[0:3] for p in seg[:-2]])


def get_bmesh_loose_edges(bm):
    # TODO: handle "visible" state
    looseedges = [i.verts for i in bm.edges if i.is_wire]
    edges_co = None
    if len(looseedges) > 0:
        edges_co = []
        for i, (v0, v1) in enumerate(looseedges):
            edges_co.extend([v0.co, v1.co])
    return edges_co


def get_bmesh_loose_verts(bm):
    # TODO: handle "visible" state
    pos = [v.co for v in bm.verts if not v.link_edges]
    if len(pos) < 1:
        return None
    return pos


def get_curve_arrays(curve):
    # ~0.1s / 100k pts on POLY
    t = time.time()
    segs = []
    pos = []
    k0 = 0
    for i, spl in enumerate(curve.splines):
        # limited support for nurbs
        if spl.type in {'POLY', 'NURBS'}:
            if len(spl.points) < 2:
                continue
            pos.extend([p.co[0:3] for p in spl.points])
        elif spl.type == 'BEZIER':
            pts = spl.bezier_points
            # limit resolution on huge curves
            if len(pts) < 2:
                continue
            elif len(pts) > 500:
                resolution = 0
            else:
                resolution = curve.resolution_u
            for j, p1 in enumerate(pts[1:]):
                interp_bezier(pts[j], p1, pos, resolution)
            if spl.use_cyclic_u:
                interp_bezier(pts[-1], pts[0], pos, resolution)
            else:
                pos.append(pts[-1].co[0:3])
        else:
            # fix issue #9 Nurbs curve crash blender
            continue
        k1 = len(pos)
        segs.extend([(j, j + 1) for j in range(k0, k1 - 1)])
        if spl.use_cyclic_u:
            segs.append((k1 - 1, k0))
        k0 = k1
    if len(segs) < 1:
        return None

    edges_co = []
    for i, (v0, v1) in enumerate(segs):
        edges_co.extend([pos[v0], pos[v1]])
    return edges_co


class _Object_Arrays:

    def __init__(self, obj, typ):

        self.segs_co = None
        self.origin_co = None
        self.bounds_co = None
        self.is_mesh = False

        if typ == TYP_ORIGIN:

            self.origin_co = obj

        elif typ == TYP_BOUNDS:

            self.bounds_co = obj

        elif obj.type == 'CURVE':
            self.segs_co = get_curve_arrays(obj.data)
        
        elif obj.type == 'MESH':

            # @TODO: properly store isolated verts
            if obj.mode == "EDIT":
                bm = bmesh.from_edit_mesh(obj.data)
            else:
                bm = bmesh.new(use_operators=True)
                bm.from_mesh(obj.data)

            # apply both rotation and scale
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            self.origin_co = get_bmesh_loose_verts(bm)
            self.segs_co = get_bmesh_loose_edges(bm)

            self.is_mesh = True

            if obj.mode != "EDIT":
                bm.free()

    def __del__(self):
        del self.segs_co
        del self.origin_co
        del self.bounds_co


def check_error(msg):
    if bgl is None:
        return
    err = bgl.glGetError()
    if err != bgl.GL_NO_ERROR:
        print ("slcad_snap GL Error: %s %s" % (msg, err))


class GPU_Indices:

    shader = gpu.types.GPUShader(vert_3d, primitive_id_frag)
    fmt = gpu.types.GPUVertFormat()
    fmt.attr_add(id="pos", comp_type='F32', len=3, fetch_mode='FLOAT')
    if USE_PRIMITIVE_BUFFER:
        fmt.attr_add(id="primitive_id", comp_type='F32', len=1, fetch_mode='FLOAT')

    P = Matrix()

    def create_buffers(self, co, batch_type):
        _len = len(co)
        if batch_type == 'POINTS':
            indices = np.arange(_len, dtype='i')
        else:
            indices = [(i, i + 1) for i in range(0, _len, 2)]

        vbo = gpu.types.GPUVertBuf(len=_len, format=self.fmt)
        vbo.attr_fill(id="pos", data=co)

        if USE_PRIMITIVE_BUFFER:
            if batch_type == 'POINTS':
                primitive_id = np.arange(_len, dtype='f4')
            else:
                primitive_id = np.repeat(np.arange(_len / 2, dtype='f4'), 2)
            vbo.attr_fill(id="primitive_id", data=primitive_id)
            del primitive_id

        ebo = gpu.types.GPUIndexBuf(type=batch_type, seq=indices)
        batch = gpu.types.GPUBatch(type=batch_type, buf=vbo, elem=ebo)
        del indices

        return _len, vbo, ebo, batch

    def __init__(self, obj, typ):

        self.MVP = Matrix()

        self.first_index = 0

        self.ebo_segs = None
        self.vbo_segs  = None

        self.ebo_origin= None
        self.vbo_origin = None

        self.vbo_bounds = None
        self.ebo_bounds = None

        self.batch_bounds = None
        self.batch_origin = None
        self.batch_segs = None

        self.num_segs = 0
        self.num_origins = 0
        self.num_bounds = 0

        self._draw_segs = False
        self._draw_origins = False
        self._draw_bounds = False
        self.is_mesh = False

        self.snap_mode = 0

        _arrays = _Object_Arrays(obj, typ)

        if _arrays.segs_co:

            self.num_segs, \
            self.vbo_segs, \
            self.ebo_segs, \
            self.batch_segs = self.create_buffers(_arrays.segs_co, "LINES")

            self._draw_segs = True

        # objects origin
        if _arrays.origin_co:

            self.num_origins, \
            self.vbo_origin, \
            self.ebo_origin, \
            self.batch_origin = self.create_buffers(_arrays.origin_co, "POINTS")

            self.is_mesh = _arrays.is_mesh
            self._draw_origins = True

        # objects bound box
        if _arrays.bounds_co:

            self.num_bounds, \
            self.vbo_bounds, \
            self.ebo_bounds, \
            self.batch_bounds = self.create_buffers(_arrays.bounds_co, "POINTS")
            self._draw_bounds = True

        self._arrays = _arrays


    def get_tot_elems(self):
        tot = 0
        if self.draw_segs:
            tot += self.num_segs
        if self.draw_origin:
            tot += self.num_origins
        if self.draw_bounds:
            tot += self.num_bounds
        return tot
    
    @property
    def draw_segs(self):
        return self._draw_segs and self.batch_segs is not None

    @property
    def draw_bounds(self):
        return self._draw_bounds and self.batch_bounds is not None

    @property
    def draw_origin(self):
        # origins and isolated vertices
        return (self._draw_origins or (self._draw_segs and self.is_mesh)) and self.batch_origin is not None

    def set_snap_mode(self, snap_mode):
        self.snap_mode = snap_mode

    def set_draw_mode(self, draw_segs, draw_origin, draw_bounds):
        self._draw_segs = draw_segs
        self._draw_origins = draw_origin
        self._draw_bounds = draw_bounds
        print("draw_segs state", self.draw_segs)


    @classmethod
    def set_ProjectionMatrix(cls, P):
        cls.P = P

    def set_ModelViewMatrix(self, MV):
        self.MVP = self.P @ MV

    def draw(self, index_offset):

        self.first_index = index_offset

        self.shader.bind()
        self.shader.uniform_float("MVP", self.MVP)

        gpu.state.depth_mask_set(False)
        gpu.state.blend_set('NONE')
        gpu.state.line_width_set(1.0)

        if self.draw_segs:
            self.shader.uniform_float("offset", float(index_offset))
            self.batch_segs.draw(self.shader)
            if USE_PRIMITIVE_BUFFER:
                index_offset += int(self.num_segs / 2)
            else:
                index_offset += self.num_segs

        if self.draw_origin:
            self.shader.uniform_float("offset", float(index_offset))
            self.batch_origin.draw(self.shader)
            index_offset += self.num_origins

        if self.draw_bounds:
            self.shader.uniform_float("offset", float(index_offset))
            self.batch_bounds.draw(self.shader)
            index_offset += self.num_bounds

    def get_seg_co(self, index):

        if USE_PRIMITIVE_BUFFER:
            i = index * 2
        else:
            i = 2 * int(index / 2)

        if i + 2 > self.num_segs:
            print("index", i, ">", self.num_segs)
            i = self.num_segs - 2

        return self._arrays.segs_co[i:i + 2]

    def get_origin_co(self, index):
        return self._arrays.origin_co[index]

    def get_bounds_co(self, index):
        return self._arrays.bounds_co[index]

    def __del__(self):
        pass
