# OctoPlanner3D Backend

This directory is the complete product-owned integration boundary for the
default 3D global planner.

```text
navd
  -> GlobalPlanRequest / GlobalPlanResult
  -> octoplanner3d_runtime
  -> vendor/planner
  -> system liboctomap
```

`vendor/` contains only the four OctoPlanner3D files used by the product:
the constrained planner header/source and the optional offline PCD converter
header/source. The ROS2/RViz wrapper, duplicate runtime, sample assets, build
directories, and precompiled libraries are not product inputs.

The online `navd` path reads an immutable, Maps-validated `octomap.ot` snapshot
and does not link PCL. PCL is limited to the standalone offline
`octoplanner3d_pcd_to_octomap` tool. OctoMap must come from the target system
(`liboctomap-dev`); checked-in `.so` files are intentionally rejected so an
x86_64 binary cannot leak into an aarch64 release.

The vendored code derives from
[JackJu-HIT/OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D), commit
`9a9cc431ea905a5878975cc6fbbce6c9618b31a4`, under the MIT license. LingTu's
copy includes robot-envelope, ground-support, floor-continuity, step/slope,
terminal-tolerance, and cancellation changes. See `LICENSE`.

FAR is a separate optional 2D global planner under `../far`; it is not part of
this vendor tree and does not replace OctoPlanner3D as the product default.

## Saved point-cloud conversion

`octoplanner3d_pcd_to_octomap` consumes saved, voxel-sampled point clouds.
Save-time pruning owns persistent-map cleanup. The converter preserves each
finite, in-bounds occupied voxel; point multiplicity after voxel sampling is
not an observation count. Requiring several points per voxel or applying a
second small-cluster filter can erase both ground and thin obstacles.

PCL supplies the ASCII/binary reader when available; the no-PCL ASCII reader
feeds the same occupancy, support dilation, and free-envelope builder. Free
envelopes are generated only above detected support surfaces. A cloud with no
usable occupied voxels fails without writing an empty OctoMap. The optional
direct-PCD planner backend still uses its separate vendored converter; normal
Product navigation consumes the saved OctoMap instead.

Support seeds must be exposed (no occupied voxel immediately above) and have
exposed neighbors in at least three of the four directly adjacent XY cells,
allowing one voxel of height difference for treads. Do not probe across several
empty cells: nearby parallel walls can otherwise satisfy the horizontal-support
test together and acquire incorrect padding and free envelopes. Sparse points
that fail this support test remain occupied; they are not deleted. The generated
free envelope is a configured surface-based inference, not measured ray clearing.
Support padding also checks the original target column: if an occupied surface
already exists more than one voxel below, it does not invent an upper surface
there. This prevents raised object tops from growing into aisles above observed
floor. Existing upper-floor and obstacle points remain occupied.

`octoplanner3d_sampled_pcd_smoke` checks one-point-per-voxel ground, an isolated
thin obstacle, support-only free space, and empty-result rejection. It exercises
support/free dilation of both zero and one cell, preserves obstacles inside the
free envelope, rejects isolated/parallel vertical walls as support, and retains
one-voxel stair transitions. When PCL
is available it checks both ASCII and binary PCD, while
`octoplanner3d_sampled_ascii_pcd_smoke` checks the no-PCL reader as well.

Footprint support samples use the cells containing the physical radius edges.
For a 0.43 m radius at 0.2 m resolution, the offset is two cells (0.4 m);
rounding up to three queried a cell entirely outside the footprint. The grid
query regression checks both a supported narrow floor and a genuinely missing
edge support. Collision-envelope sampling is unchanged.

## Search neighborhoods

Each request first searches with the basic 26 adjacent voxel offsets. If that
search exhausts its reachable graph or its first-stage budget without a path,
it restarts from the same resolved endpoints with additional stair connections. With 0.1 m
voxels, a 0.45 m step limit, and no slope bound, fallback uses 26 + 320 offsets.
The extra offsets are candidate connections and still require the existing
support, body collision, intermediate collision, step, and slope checks.

The two searches have separate frontiers and path costs, share immutable cell
query caches, and consume one `max_iterations` budget. The basic stage receives
one quarter of that budget, capped at 25000 iterations for goals outside the
same-floor Z tolerance. This reserves work for stair candidates without forcing
long same-floor detours into the expanded graph prematurely. A stage budget is
not evidence that its graph has no path. Cancellation and invalid endpoints
never trigger fallback; total budget exhaustion returns `search_iteration_limit`.
A successful basic route is accepted without searching for a shorter
route through stair connections. This can avoid step crossings but does not
promise the shortest route in the expanded graph.

`OctoPlanner3D::searchInfo()` and the native stage log distinguish basic and
fallback searches. These describe grid planning, not verified robot stair
climbing capability.

## Support surfaces and collision envelopes

The default surface route uses direct support below the centre and four
footprint samples. Side samples may lie on different tread heights within the
step limit (rounded outward for voxelized support heights); isolated narrow
ridges cannot substitute for a supported stance. Actual search edges retain
the metric step and slope checks.

For an uneven supported stance, the surface-rooted envelope distinguishes a
nearby tread from a taller obstacle. It never ignores occupancy in the route
node's own XY column. Flat-ground clearance and explicit body-centre cylinder
clearance retain their occupied-volume checks. These tests establish geometric
route feasibility, not a foothold sequence or the locomotion policy's ability
to climb the resulting staircase.

## Files

| File | Responsibility |
| --- | --- |
| `octoplanner3d_core.hpp/.cpp` | Adapts `GlobalPlanRequest/Result` to OctoPlanner3D, validates temporary overlays, loads immutable OctoMaps, caches a map-bound `PlannerSession`, and applies terminal constraints. |
| `octoplanner3d_headless.cpp` | Standalone JSON stdin/stdout frontend used by diagnostics and compatibility tooling. |
| `pcd_to_octomap.cpp` | Offline PCD-to-`.ot`/`.bt` conversion CLI with support/free-envelope options; not part of online `navd`. |
| `edit_octomap.cpp` | Offline editor for occupied, free, preblocked, traversable, and cleared regions. |
| `dump_octomap.cpp` | Exports bounded occupied voxel centers for inspection and debugging. |
| `make_test_octomap.cpp` | Generates deterministic two-floor and spiral-stair OctoMaps for tests. |
| `no_air_climb_smoke.cpp` | Checks ground support, body clearance, floor continuity, overlays, and no-air-climb behavior. |
| `neighborhood_fallback_smoke.cpp` | Checks basic-first routing, stair fallback, blocked routes, shared budgets, cancellation, and runtime failure reasons. |
| `grid_query_smoke.cpp` | Compares cached grid occupancy and leaf-center queries with OctoMap, including map and overlay invalidation. |
| `queue_node_compare_smoke.cpp` | Checks the vendored planner priority-queue ordering. |
| `edit_octomap_smoke.cmake` | Drives the binary OctoMap edit/read/write smoke test. |
| `vendor/planner/include/global_planner.h` | Vendored constrained search types and `OctoPlanner3D` interface. |
| `vendor/planner/src/global_planner.cpp` | Vendored 3D grid/A* search implementation with LingTu constraints. |
| `vendor/octomap/include/pcd2octomap_converter.h` | Optional vendored PCD converter interface. |
| `vendor/octomap/src/pcd2octomap_converter.cpp` | Optional vendored PCD converter implementation. |
| `CMakeLists.txt` | Wires system OctoMap, optional PCL, runtime library, CLIs, and smoke tests. |
| `LICENSE` | MIT license and provenance for the vendored subset. |
