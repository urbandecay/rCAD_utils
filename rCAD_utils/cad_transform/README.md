# CAD Transform

Vendored from `slcad_transform_0.93.2.beta.3.zip` and registered as part of
the `rCAD Utils` container addon.

## Current behavior

- Translate, rotate, and scale are modal CAD-style transforms.
- `X`, `Y`, and `Z` constrain the active transform to the global axes.
- `E` toggles edge snapping.
- `P` toggles perpendicular-edge snapping.
- `Alt+P` toggles parallel-edge snapping.
- `V`, `F`, `G`, `N`, `O`, and `B` toggle vertex, face, grid, normal, origin,
  and bounding-box snapping respectively.
- `C` starts a user-defined constraint during movement; `Shift+C` switches to
  a world constraint.
- `Ctrl` temporarily disables snapping, `Alt` rounds values, and `D` toggles
  duplication.

The CAD Transform toolbar tools are also registered for object, mesh-edit,
and curve-edit modes.  The rCAD Utils sidebar exposes direct Translate,
Rotate, and Scale buttons under the CAD Transform section.

The addon previews snapped geometry, edge direction, axis constraints, and
transform widgets while a modal operation is active.  It does not permanently
recolor every existing edge that happens to be axis-aligned.
