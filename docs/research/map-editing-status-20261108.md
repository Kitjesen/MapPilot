# Map cleaning, completion and editing: verified status

2026-11-08. Current source and local Windows/Web artifacts; no NX deployment.

## Cleaning and alignment

LTLIO selects valid LiDAR returns, applies configured range limits and sampling,
and estimates motion. Pose graph optimization changes scan poses. Reconstructing
the saved map still uses those scans; excluding a point from pose estimation is
not the same as removing it from the exported point cloud.

The test6 candidate used existing DUFOMap cleanup, followed by the native radius
edit operation (0.2 m, minimum 12 points including the query point). DUFOMap only
removed points classified as seen-free by accumulated ray evidence. It removed
250,674 records but left all 50 points below -1 m. Radius filtering removed those
50, plus other sparse points: 40,636 additional removals in total. The -1 m value
was an evaluation threshold, not a deletion rule.

The 3-5 m band lost 16.3% of its points. Accordingly, this radius setting is a
reviewed map-copy operation, not a universal default or a claim that every
removed point was noise. Original captures and the unfiltered map remain.

Existing native edit operations in `src/maps/cpp/edit.cpp` are voxel sampling,
statistical outlier filtering, radius filtering, small-cluster filtering and
RANSAC plane-range cropping, as well as selection delete/keep. Plane cropping
retains existing points inside expanded fitted-plane bounds; it adds no ground.

## Integration fixed in this change

- Saved-map `MapView` now exposes 点云整理 and mounts the existing editor through
  `SavedMapEditor`. It accepts an inactive map with its real content epoch.
- The saved editor reads the identity-checked PCD artifact directly. The existing
  active-map `/points` endpoint is intentionally unchanged; using it caused the
  first browser test to return HTTP 409 for an inactive map.
- Selection, filters, native full-source preview, draft/undo, save-as and OctoMap
  rebuild reuse `MapEditor` and the existing native endpoints. No second filter
  or editor implementation was added.
- The local 4326 workspace permits saved-map preview/save-as/OctoMap build and
  independent no-go region writes. It declares this map-edit capability, while reporting
  the actual idle/no-Product/no-active-map state. Robot/control writes remain
  unavailable. Its Windows file adapter remains a local development boundary.
- Rebuilding an OctoMap now replays its existing `voxel_edits.jsonl` in order,
  including after resolution changes. Both single-artifact and navigation-package
  builders use the same native tree edit function. Failed rebuilds preserve the
  old artifacts and journal. This is not automatic transfer of edits from a
  previous processed map into an unrelated original-capture reoptimization.
- The legacy SceneView controls now say 添加占据 / 设为空闲. Existing API aliases
  remain compatible; the UI no longer calls free voxels 可通行 or implies that
  clear restores the original sensor map.

## Completion and forbidden-region capabilities

| Capability | Actual implementation |
| --- | --- |
| Global inferred support | Shared Maps `SurfaceIndex` identifies bounded internal holes by expected height, fits observed boundary planes and supplies the global planner. Defaults: 0.20 m radius, 0.16 m² area, fit slope 0.7. |
| Surface fitting | Local linear plane fitting reuses existing Maps math; it is not an imported full polynomial MLS pipeline. |
| Known-free veto | Candidate ground cells query the saved OcTree; known-free cells are excluded while free space above a ground cell does not invalidate it. |
| Inferred-support Web view | Navigation voxels → 补全支撑 reads `/maps/{name}/surface`. Cyan observed/orange inferred layers share the camera and use actual native XYZ. Counts are complete; displayed observed points may be sampled. |
| Map changes | Loading a new map or changing surface options rebuilds the index and connections. Per-frame regional live-map updates remain unimplemented. |
| Manual point/ground addition | Missing from both edit operation types and their native implementation. `plane_crop` is not completion. |
| Independent no-go layer | `no_go.json` stores finite XYZ boxes independently of PCD/OctoMap. Settings → Map supports add/update/delete. Changes increment the map content epoch; existing active-map edit restrictions apply. |
| Unified no-go execution | The active map artifact carries one shared policy into global OctoPlanner, local SCAN and final braking. Local coordinate conversion preserves map-frame regions and distinct Z layers. Explicit manual takeover retains its existing bypass. |

Current inferred support is not written into original PCD/OctoMap and is not a
continuous mesh. A future manual support patch still needs explicit inferred
geometry and its 3D height. No-go policy is now independent: deleting a region
does not erase a real obstacle. Saved-map voxel views and the point-cloud editor
render policy boxes separately; the standalone point-cloud overview does not yet
render this overlay.

Cleaning controls moved from the floating canvas card into Settings → Map.
Reading, previewing and saving display their real stage and elapsed time. Native
filter requests do not report a per-point completion percentage, so their progress
bar is indeterminate. Save-as and OctoMap rebuild are shown as steps 1/2 and 2/2.

The real test6 browser preview exposed a configuration mismatch: the filter ran
for 61.77 seconds, then mapd rejected its JSON with `response_too_large`. The
protocol/client allowed 4 MiB, but the product YAML still limited mapd to 1 MiB.
The YAML now uses 4 MiB and the C++ protocol default references the generated
setting. A real UDS test returns both 30,000 kept and 30,000 removed sample points;
an explicit 1 MiB override still reports the size failure. Preview errors no
longer say that a map save failed. This fix does not speed up filtering.

The before/after cleaning report is `artifacts/maps/cleaning-report-20261108/index.html`.
The method and pinned-source comparison is
[`terrain-completion-options-20261108.md`](terrain-completion-options-20261108.md).
The implemented surface method, full test6 comparison and evidence are in
[`surface-completion-20261108.md`](surface-completion-20261108.md).

## Algorithm references

- [DUFOMap](https://kth-rpl.github.io/dufomap/): ray-based dynamic-map cleanup.
- [PCL radius outliers](https://pointclouds.org/documentation/tutorials/remove_outliers.html).
- [PCL statistical outliers](https://pointclouds.org/documentation/tutorials/statistical_outlier.html).
- [PCL voxel sampling](https://pointclouds.org/documentation/tutorials/voxel_grid.html).
- [PCL RANSAC plane segmentation](https://pointclouds.org/documentation/tutorials/planar_segmentation.html).
- [PCL MLS](https://pointclouds.org/documentation/tutorials/resampling.html): local surface smoothing/resampling.
- [Open3D Poisson reconstruction](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html#poisson-surface-reconstruction): may extrapolate into low-density regions; a closed mesh is not evidence of navigable ground.

## Verification

Native real-OctoMap store tests passed, including edit replay, resolution change,
ordered free overrides and failed rebuild preservation. The formal Windows mapd
was rebuilt and synchronized into the local workspace. Editing/draft/preview
tests, the inactive-map artifact-loader regression, touched lint, TypeScript and
Vite build passed. Local HTTP capability and full-source no-op preview passed for
11,417,909 points without an active map.

Detailed native evidence: `artifacts/maps/voxel-edit-rebuild-20261108/README.md`.
Workspace HTTP and browser evidence: `artifacts/maps/edit-workspace-20261108/`.
No-go verification is recorded in `artifacts/maps/no-go-native-20261108/`,
`artifacts/maps/no-go-local-20261108.md` and `artifacts/maps/no-go-20261108/`.
Native surface behavior, full-map exports and the Web surface query are covered
in `artifacts/maps/surface-completion-20261108/`. No real robot motion or NX
performance is claimed.
