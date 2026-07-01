# OctoPlanner3D algorithm

This package wires OctoPlanner3D into LingTu as the default ROS-free 3D global
planner implementation named `octoplanner3d`. PCT remains available only as an
explicit legacy/reference implementation.

It intentionally does **not** import ROS2 or route Navigation through ROS topics.
The full OctoPlanner3D source used by LingTu lives under
`global_planner/algorithm/OctoPlanner3D`; LingTu's production path calls it through
`nav.GlobalPlanner`.

## Runtime path

```text
Navigation
  -> GlobalPlanner(planner_name="octoplanner3d")
  -> OctoPlanner3D implementation
  -> octoplanner3d_headless
  -> OctoPlanner3D C++ GlobalPlanner
```

The ROS2 RViz node is not on this path.

## Service contract

Python callers use `nav.services.plan.contracts.GlobalPlanRequest` and
`GlobalPlanResult`:

```text
request:  start, goal, safe_goal_tolerance
result:   path, plan_ms, reached_goal, error, diagnostics, report
```

`Navigation` and plan preview call `PlannerService.plan_request(request)`.
`plan(start, goal, ...)` remains only as a compatibility wrapper. Native
OctoPlanner3D still receives the JSON executable protocol described below.

The runtime path does not use the generic registry to select algorithms anymore.
`GlobalPlanner` directly maps `octoplanner3d`, `pct`, and `direct` to their
planner classes. The old `planner_backend` registry entry remains only for
compatibility and metadata tests.

## Python module layout

- `octoplanner3d.py`: registry compatibility entry only. `OctoPlanner3DBackend`
  exists because the current planner factory contract is still named
  `planner_backend`.
- `octoplanner3d_planner.py`: GlobalPlanner-compatible planner logic.
- `octoplanner3d_protocol.py`: payload schema, constraint normalization, and path
  parsing.
- `octoplanner3d_runtime.py`: native module discovery, executable discovery, WSL
  path conversion, and subprocess execution.
- `direct_path.py`: mapless direct planner used by Thunder Lite.
- `OctoPlanner3D/`: complete upstream OctoPlanner3D source plus the ROS-free
  runtime wrapper in `OctoPlanner3D/runtime`.

Algorithmically, the upstream OctoPlanner3D core runs a constrained 3D A*
search over OctoMap cells. In LingTu, `octoplanner3d` is still the planner
implementation name; `octomap_3d_astar` is exposed only as the internal search-kernel
diagnostic. Product profiles pass quadruped body-envelope and terrain-support
constraints into the native core instead of relying on the upstream hard-coded
defaults.

## Build the headless executable

The upstream OctoPlanner3D implementation is copied into
`global_planner/algorithm/OctoPlanner3D`. Build the LingTu wrapper directly:

```bash
scripts/build/build_octoplanner3d.sh
```

The build requires the local OctoMap/OctoMath libraries shipped by the
OctoPlanner3D source. PCL is **not** a planner dependency; it is only one possible
point-cloud-to-OctoMap converter dependency. Without PCL, the headless executable
still builds and accepts OctoMap file inputs such as `.bt` / `.ot`. With PCL, the
same executable can additionally convert `.pcd` point clouds before planning.
Override the source path only when testing a different OctoPlanner3D checkout:

```bash
export LINGTU_OCTOPLANNER3D_SOURCE_DIR=/path/to/OctoPlanner3D
export LINGTU_OCTOPLANNER3D_BUILD_DIR=/tmp/lingtu-octoplanner3d-build
scripts/build/build_octoplanner3d.sh
```

## Runtime configuration

OctoPlanner3D uses the executable configured by:

```bash
export LINGTU_OCTOPLANNER3D_EXECUTABLE=/path/to/octoplanner3d_headless
```

The implementation also searches the default build output:

```text
build/octoplanner3d_headless/octoplanner3d_headless
```

`tomogram_path` is the legacy constructor argument name used by
`GlobalPlanner`. Treat it as the OctoPlanner3D map source path; the
runtime sends both the legacy `map_path` field and the newer generic
`map_source` descriptor to the headless executable without requiring ROS2.

## Executable protocol

The executable protocol is JSON over stdin/stdout. Inputs:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "map_path": "/path/to/map.bt",
  "map_source": {
    "kind": "octomap_file",
    "path": "/path/to/map.bt",
    "format": "bt",
    "frame": "map"
  },
  "map_format": "bt",
  "start": [0.0, 0.0, 0.0],
  "goal": [1.0, 2.0, 0.5],
  "obstacle_thr": 49.9,
  "options": {
    "planner_family": "octoplanner3d_constrained_global_planner",
    "search_algorithm": "octomap_3d_astar",
    "constraint_model": "quadruped_bounding_cylinder_ground_support",
    "robot_radius": 0.6,
    "max_iterations": 800000,
    "snap_search_radius_cells": 12,
    "require_ground_support": true,
    "strict_direct_ground_support": false,
    "ground_support_xy_radius_cells": 1,
    "ground_support_depth_cells": 1,
    "enable_preblocked_costmap": true,
    "preblocked_costmap_radius_cells": 3,
    "preblocked_costmap_weight": 2.5,
    "lowest_traversable_only": false
  }
}
```

Input fields:

- `map_source`: required generic map descriptor. Current native runtime consumes
  file sources: `octomap_file` (`.bt`, `.ot`, `.octomap`) or `point_cloud_file`
  (`.pcd` when the binary was built with PCL conversion support).
- `map_path`: legacy alias for `map_source.path`, kept for compatibility with
  existing LingTu planner-service calls and the current C++ wrapper.
- `map_format`: `bt`, `ot`, `octomap`, `pcd`, or `auto`.
- `start`: required `[x, y, z]` in LingTu planning/map frame, meters.
- `goal`: required `[x, y, z]` in LingTu planning/map frame, meters.
- `obstacle_thr`: compatibility field from LingTu's planner service. Native
  OctoPlanner3D uses OctoMap occupancy, robot radius, ground support, and
  traversability checks for collision/traversal decisions.
- `options`: native OctoPlanner3D options. LingTu product profiles set the
  quadruped bounding radius and terrain-support constraints here; the native
  wrapper applies them before calling `GlobalPlanner::setOctomap()`.

Output:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "ok": true,
  "path": [[0.0, 0.0, 0.0], [1.0, 2.0, 0.5]],
  "reached_goal": true,
  "diagnostics": {
    "source": "octoplanner3d_headless",
    "goal_error_m": 0.0,
    "octomap_file": true,
    "pcd_conversion": false,
    "ros2_required": false,
    "constraints": {
      "robot_radius": 0.6,
      "require_ground_support": true
    }
  }
}
```

Output fields:

- `ok`: true when the native planner returned a non-empty global path.
- `path`: global waypoints as `[[x, y, z], ...]` in LingTu planning/map frame.
- `reached_goal`: true when the final waypoint is inside native goal tolerance.
- `diagnostics`: native run details such as source, path point count,
  `goal_error_m`, elapsed time, and failure reason.

Supported map inputs:

- `.bt` OctoMap binary files; no PCL required.
- `.ot` / `.octomap` generic OctoMap files through `octomap::AbstractOcTree`.
- `.pcd` point clouds when the headless executable is built with PCL conversion
  support. Without PCL, convert point clouds upstream to OctoMap or install PCL
  for this converter plugin; the planner itself remains ROS2/PCL-independent.

The supported map formats are also exposed in planner metadata and planner
diagnostics, so callers can discover the accepted map inputs and converter
requirements without constructing ROS2 nodes or importing the upstream wrapper.

When the executable is not configured or cannot be built on the current host, the
implementation imports successfully and fails soft so existing PCT/A*/direct planner
paths are unaffected.
