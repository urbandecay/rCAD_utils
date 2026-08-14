# Future projection-tool rebuild prompt

Rebuild the projection tool from zero. Do not reuse the previous projection implementation or its assumptions.

The finished tool must let the user store selected source vertices, select the target object or target geometry, and project the stored vertices onto that target.

It must work consistently when the target is a single face, an angled face, or a connected surface made from many faceted bevel faces. The result must remain coherent across the whole source selection and must not depend on viewport orientation, screen overlap, perspective, or accidental visibility.

The workflow must make source and target selection unambiguous, report invalid or incomplete selections clearly, preserve undo behavior, and restore the intended source selection after completion.

The rebuild must include verification for flat, angled, separated, and multifaced-bevel examples, plus cases where projection cannot produce a valid result.
