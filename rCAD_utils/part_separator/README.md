# Part Separator

The operator is intended for a joined collection of closed board meshes.

Detection is attempted in this order:

1. `rcad_part_id`, a face attribute written by **Mark IDs for Future Edits**.
2. Manifold face shells, which ignore vertex-only welds and non-manifold weld
   seams.
3. Welded-box recovery, which recreates a coincident cap face when
   `bmesh.weld_verts` removed one of two overlapping cap faces.
4. Fused-prism recognition, which reads rectangular cells from an end
   cross-section and rebuilds each cell as an independent closed solid. This
   handles a completely manifold four-board stack with no internal faces.

The result is one Blender object containing disconnected mesh islands. The
operator does not leave one Blender object per board.

In Edit Mode, selecting any vertex, edge, or face chooses its complete
edge-connected mesh region. Only those selected regions are rebuilt; all
disconnected unselected geometry in the object is preserved unchanged. Object
Mode continues to process every selected mesh object in full.

This covers endpoint, corner, T, L, stacked, face-to-face, and multiple-board
welds when either the original board faces or the cross-section seam loops
still exist. If a cleanup operation also dissolves every seam loop, the final
mesh may be indistinguishable from a single larger beam. In that case the
separator reports one ambiguous part; use stored face IDs or keep the boards
as separate objects before editing.
