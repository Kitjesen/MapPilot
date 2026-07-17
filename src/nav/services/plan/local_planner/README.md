# LocalPlanner I/O Contract

`LocalPlanner` is the local path planning Module.

The scoring math, fixed 343/7/36 path-library cardinalities, output-path
semantics, PathFollower boundary, and native-endpoint/Python parameter
differences are defined once in
[Local Planning and Tracking Contract](../../../../../docs/architecture/LOCAL_PLANNING_AND_TRACKING_CONTRACT.md).
This file documents Module ports and wiring only.

- Runtime name: `nav.local_planner`
- Code: `src/nav/services/plan/local_planner/service.py`
- Port source of truth: `nav.services.plan.contracts.LOCAL_PLAN_PORT_CONTRACT`
- Backends: `nanobind` preferred, `cmu_py` debug fallback, `simple` tests only
- Algorithm: CMU-style precomputed local path library scoring over obstacle points
  plus traversability risk-grid scoring
- Frame contract: all planning inputs must already be in `planning_frame_id`
  (`map` by default; `odom` only when the profile opts in). `LocalPlanner`
  does not perform TF transforms.

It does not output velocity. Velocity is produced later by:

```text
nav.local_planner.local_path
  -> nav.path_follower
  -> nav.velocity_mux
  -> Driver
```

## Input Groups

### 1. Localization State

| Port | Type | Source | Wired today | Used today |
| --- | --- | --- | --- | --- |
| `odometry` | `Odometry` | `ctx.nav_odom_src`, usually `SlamModule` or driver | Yes, through `odometry_fanout_specs()` | Yes. Updates robot pose/yaw and triggers planning when a waypoint exists. |
| `map_frame_jump_event` | `dict` | `SlamModule.map_frame_jump_event` or compatible SLAM adapter | Yes when SLAM/localization is active | Yes. Clears stale local path after relocalization or map frame jumps. |

Notes:

- Profiles with `slam_profile=none` can still provide odometry from the driver.
- If neither SLAM nor driver odometry exists, local planning cannot run correctly.
- If odometry arrives in a different frame from `planning_frame_id`, the module
  publishes an empty `local_path` and a `control_hint.reason=frame_mismatch`.

### 2. Mission / Global Planning Dispatch

| Port | Type | Source | Wired today | Used today |
| --- | --- | --- | --- | --- |
| `waypoint` | `PoseStamped` | `nav.mission.waypoint` | Yes | Yes. Current local target. |
| `global_path` | `Path` | `nav.mission.global_path` | Yes | Yes. Used to select a corridor goal along the global path. |
| `clear_path` | `bool` | `nav.mission.clear_path` | Yes | Yes. Clears published local path and control hint. |

Notes:

- `waypoint` and `global_path` are produced after the mission layer accepts a goal and global planning succeeds.
- A frontend click should become a mission goal first; it must not go directly to `LocalPlanner`.
- `waypoint` and `global_path` frames must match `planning_frame_id`.

### 3. Terrain / Map Evidence

| Port | Type | Source | Wired today | Used today |
| --- | --- | --- | --- | --- |
| `terrain_map` | `PointCloud2` | `nav.terrain.terrain_map` | Yes | Yes. Main obstacle/terrain point cloud. |
| `terrain_map_ext` | `PointCloud2` | `nav.terrain.terrain_map_ext` | Yes | Yes. Extended local terrain/obstacle cloud. |
| `traversability` | `dict` | `nav.terrain.traversability` | Yes | Yes. High-risk cells are hard-blocked, soft-risk cells reduce candidate path scores, and summary is published in diagnostics/control hint. |
| `esdf` | `dict` | `TraversabilityCostModule.esdf_field` | Yes when map layers are active | Reserved. Stored but not consumed by the C++ scorer yet. |

Upstream map flow:

```text
SlamModule.map_cloud or driver.map_cloud
  -> nav.terrain
  -> terrain_map / terrain_map_ext / traversability
  -> nav.local_planner

OccupancyGridModule + ESDFModule + ElevationMapModule + nav.terrain.traversability
  -> TraversabilityCostModule
  -> esdf_field
  -> nav.local_planner.esdf
```

Current behavior:

- `traversability` and `esdf` are available at the port level.
- If a traversability payload includes `frame_id`, it must match
  `planning_frame_id`; otherwise planning is rejected for that tick.
- `traversability` changes local path selection in both supported planning backends:
  - `nanobind`: the C++ core consumes the risk grid directly for near-field stop,
    hard path rejection, and soft path score penalties.
  - `cmu_py`: the Python fallback samples the same risk grid and also converts
    hard-risk cells into virtual obstacles for legacy obstacle-count logic.
  - hard risk: cells `>= traversability_hard_cost` are rejected by path scoring.
  - soft risk: cells above `traversability_soft_cost` lower candidate path scores by `traversability_weight`.
- `esdf` is stored but does not yet change the selected local path.

### 4. Safety / Operator Overlays

| Port | Type | Source | Wired today | Used today |
| --- | --- | --- | --- | --- |
| `boundary` | `PointCloud2` | Intended: boundary/geofence cloud publisher | No default producer found | Yes if provided. Treated as hard obstacle points. |
| `added_obstacles` | `PointCloud2` | Intended: operator/debug/perception obstacle injection | No default producer found | Yes if provided. Treated as hard obstacle points. |
| `check_obstacle` | `bool` | Intended: safety/debug policy switch | No default producer found | Yes if provided. Disables all obstacle cloud use when false. |

Current gap:

- `GeofenceManagerModule` publishes alerts and stop commands, but it does not publish a `boundary` point cloud.
- `added_obstacles` and `check_obstacle` are test/debug-ready ports without a production source.

## Outputs

| Port | Type | Consumer | Wired today | Meaning |
| --- | --- | --- | --- | --- |
| `local_path` | `Path` | `nav.path_follower.local_path`, `nav.safety.path`; native endpoint publishes `/nav/local_path` | Yes | Trackable local path in the planning frame. Empty path means stop/failed local plan. |
| `control_hint` | `dict` | `nav.path_follower.control_hint` | Yes | `slow_down`, `near_field_stop`, `safety_stop`, `path_found`, `recovery_state`, `reason`, traversability summary. |
| `alive` | `bool` | runtime health | Module-local | Module liveness signal. |

## What Is Actually Available Today

Available and wired in normal navigation stack:

- `odometry`
- `waypoint`
- `global_path`
- `clear_path`
- `map_frame_jump_event` when SLAM/localization is active
- `terrain_map`
- `terrain_map_ext`
- `traversability`
- `esdf` when map layers are active
- `local_path`
- `control_hint`

Port exists but no default upstream producer is wired:

- `boundary`
- `added_obstacles`
- `check_obstacle`

Available but not fully used by local path scoring:

- `esdf`

## Planner Core Flow

```text
odometry + waypoint/global_path
  -> select effective local goal
  -> read traversability risk grid
  -> sync traversability grid into C++ core or Python scorer
  -> merge terrain_map + terrain_map_ext + boundary + added_obstacles
     (cmu_py fallback also adds virtual traversability obstacles)
  -> project obstacle points into CMU path-library voxels
  -> score candidate path groups with obstacle + terrain + traversability risk
  -> publish local_path + control_hint
```

The 343 primitives are scoring/voting templates. The emitted `local_path` is
the rotated/scaled canonical `startPath` of the selected rotation/group bin;
it is not one of the 343 primitive trajectories copied verbatim.

## Required Fixes

1. Add a real producer for `boundary` if geofence boundaries should shape local path planning.
2. Add a real producer or remove the runtime port for `added_obstacles`.
3. Add a real policy source or remove the runtime port for `check_obstacle`.
4. Wire `esdf` into native scoring if clearance-aware local path smoothing becomes necessary.
