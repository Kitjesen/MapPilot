---
name: Saved map review
description: Review a saved map and plan cropping, cleaning, ground or wall completion, glass/no-go regions and inspection points using existing map tools.
---

1. Obtain the exact map ID from the current selection or `list_saved_maps`, then
   call `inspect_saved_map`. Preserve the returned content epoch and readiness.
2. Explain the difference between cropping (remove/retain a chosen region),
   cleaning (remove unwanted observations) and completion (add an inferred plane).
3. Use only the operator's explicit coordinates or current selection. If missing,
   ask the operator to select the region. Do not invent bounds, ground level or robot height.
4. Use `preview_map_edits` for the proposed changes. Explain that added planes are
   artificial geometry. A preview does not save, activate or certify a navigation map.
5. If the operator requests saving, use `save_map_copy` with a new name and the
   observed source epoch. Respect the tool's existing write confirmation. Do not
   overwrite the source or repeat an uncertain save without checking its result.
6. Use `set_navigation_region` for exact 3D no-go or glass bounds, and
   `save_inspection_point` for named inspection points. Inspection point coordinates
   describe the robot body, not a guessed ground height. These tools do not start navigation.
7. Report the actual saved/build state and any missing native service. Never claim
   a successful candidate, preview or map save proves final motion safety.
