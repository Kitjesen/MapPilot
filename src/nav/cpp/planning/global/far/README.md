# FAR Global Planner

FAR is an optional ROS-free 2D global-planning backend for the native
navigation endpoint. OctoPlanner3D remains the product default.

## Ownership

This is an independent LingTu implementation. The design was informed by the
visibility-graph architecture in the CMU autonomy stack, but no upstream FAR
source was copied because the inspected upstream source headers and package
metadata do not provide one unambiguous reusable license grant.

The product boundary is:

```text
validated active occupancy.npz
  -> ActiveOccupancyGate
  -> FarGridMap (UNKNOWN/FREE/OCCUPIED + identity + generation)
  -> FarPlanner
  -> GlobalPlanResult
```

## Safety Contract

- The map must be the active Maps artifact and pass record, hash, source,
  frame, and version validation.
- Planning uses a private immutable artifact snapshot; a map switch or hash
  change invalidates the cached graph.
- Known-free planning runs first. Unknown-space traversal is disabled by
  default and can only be enabled explicitly.
- Goal epoch, frame epoch, map identity, and map generation are checked before
  an asynchronous result can enter the local planner.
- Graph nodes, visibility pairs, and search expansions are bounded.
- Invalid configuration, malformed trinary grids, stale results, and cancelled
  searches fail closed.

## Algorithm

The planner inflates occupied cells by the robot envelope, extracts obstacle
contour corners, builds bounded supercover line-of-sight edges, and runs A* on
the resulting visibility graph. Unchanged edges are reused across compatible
map generations. Endpoint snapping and optional path simplification preserve
the same occupancy policy used by the search.

## Interfaces

| File | Role |
| --- | --- |
| `planner.hpp/.cpp` | C++ planner and diagnostics |
| `api.h/.cpp` | Stable versioned C ABI for non-C++ consumers |
| `../../../endpoint/nav/input/active/occupancy.*` | Maps artifact and TOCTOU gate |

Select it explicitly with `NAV_GLOBAL_PLANNER=far`. ProductControl writes
the selected occupancy artifact to `FAR_OCCUPANCY_PATH` in the session
environment. `/etc/lingtu/nav.env` may tune FAR, but it must not select a map.

## Verification

`scripts/build/build_nav_endpoint.sh` requires `test_far_planner`,
`test_far_c_api`, and `test_active_occupancy_gate` in the endpoint CTest
catalog. Missing tests fail the build rather than silently reducing coverage.
