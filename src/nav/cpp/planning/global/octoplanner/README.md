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
| `queue_node_compare_smoke.cpp` | Checks the vendored planner priority-queue ordering. |
| `edit_octomap_smoke.cmake` | Drives the binary OctoMap edit/read/write smoke test. |
| `vendor/planner/include/global_planner.h` | Vendored constrained search types and `OctoPlanner3D` interface. |
| `vendor/planner/src/global_planner.cpp` | Vendored 3D grid/A* search implementation with LingTu constraints. |
| `vendor/octomap/include/pcd2octomap_converter.h` | Optional vendored PCD converter interface. |
| `vendor/octomap/src/pcd2octomap_converter.cpp` | Optional vendored PCD converter implementation. |
| `CMakeLists.txt` | Wires system OctoMap, optional PCL, runtime library, CLIs, and smoke tests. |
| `LICENSE` | MIT license and provenance for the vendored subset. |
