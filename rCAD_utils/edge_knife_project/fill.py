"""Fill holes one connected target island at a time after splitting."""

import bmesh


def fill_split_islands(bm, target_faces):
    remaining = {face for face in target_faces if face.is_valid}
    added = []
    while remaining:
        island = {remaining.pop()}
        pending = list(island)
        while pending:
            for vert in pending.pop().verts:
                for face in vert.link_faces:
                    if face in remaining:
                        remaining.remove(face)
                        island.add(face)
                        pending.append(face)
        boundary = {edge for face in island for edge in face.edges if edge.is_boundary}
        if not boundary:
            continue
        caps = bmesh.ops.holes_fill(bm, edges=list(boundary), sides=0)['faces']
        if caps:
            bmesh.ops.recalc_face_normals(bm, faces=list(island) + caps)
            added.extend(caps)
    return added
