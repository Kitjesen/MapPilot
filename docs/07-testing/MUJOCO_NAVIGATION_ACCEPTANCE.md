# MuJoCo Navigation Acceptance

Status: 2026-07-16

LiDAR optical and scan-time fidelity is a separate gate. See
[`MUJOCO_MID360_FIDELITY.md`](MUJOCO_MID360_FIDELITY.md) before interpreting a
navigation result as sensor evidence.

This document defines the required assets and evidence for a MuJoCo navigation
run that moves the ThunderV4 robot with the same product logic used by the real
robot: map, plan, local follow, and locomotion.

## Current 60 m Function Gate

The canonical long-range function gate proves this bounded claim:

> An existing map can drive the native global planner, native local planner,
> embedded PathFollower, final safety gate, typed-DDS command path, and
> ThunderV4 policy to one 50-70 m goal in MuJoCo.

It deliberately uses MuJoCo truth odometry/TF/localization health so Fast-LIO2
scale and rolling-scan fidelity cannot hide a navigation-control defect:

The chain uses `Port -> Wire -> Transport` roles explicitly:

| Producer port / role | Wire / payload | Transport / consumer |
| --- | --- | --- |
| existing `map.pcd` / `octomap.ot` and goal input | global-plan request / global path | direct C++ OctoPlanner3D call inside the native endpoint |
| MuJoCo truth odometry/TF and 10 Hz instantaneous LiDAR adapter | typed odometry, TF, registered-cloud, and traversability payloads | native DDS to the C++ endpoint |
| endpoint local-planner output | canonical `local_path_body` | direct C++ PathFollower call |
| PathFollower pre-safety velocity | final command after independent safety | native DDS wire `rt/nav/cmd_vel` to ThunderV4 |
| ThunderV4 policy motion | terminal navigation state | native endpoint `goal_reached` evidence |

Python owns MuJoCo physics, sensor adaptation, process lifecycle, video
rendering, and the acceptance report only. It does not run a global planner,
local planner, PathFollower, or command mux in this gate. `autonomy`, `teleop`,
and `teleop_avoid` remain mutually exclusive native endpoint modes; Python
`CmdVelMux` must not run beside them.

The sensor/SLAM gate remains separate:

```text
physical_rolling MID-360 + 200 Hz IMU
-> native DDS -> Fast-LIO2
-> odometry / registered cloud / map accuracy
```

A passed function gate is not evidence for real MID-360 optics, Fast-LIO2
scale stability, or long-term mapping quality.

### Accepted 2026-07-16 run

```powershell
$env:PYTHONPATH='src;.'
python sim/scripts/mujoco/native_navigation_acceptance.py `
  --manifest config/runtime_graph/endpoints/mujoco_industrial_park_60m_navigation_acceptance.json `
  --mode motion `
  --no-prepare-assets `
  --record-video `
  --out-dir artifacts/codex_mid360_native_nav_60m_20260716_retry8
```

| Check | Result |
| --- | ---: |
| Overall | passed; `blockers=[]` |
| Requested goal distance | `59.942 m` |
| Final XY error | `0.075 m` |
| Executed XY path | `67.827 m` |
| Global/local path points | `311 / 101` |
| Native goal state | `goal_reached=true` |
| Signed linear commands | `3,239` forward, `0` reverse |
| Input-gate ready fraction | `0.9894` |
| Longest observed non-ready interval | `1.0 s` |
| Odom-TF / cloud-pose rejections | `0 / 1` |
| Navigation loop overrun P99 / peak | `62.62 / 94.49 ms` |
| Motion/debug log writer drops | `0 / 0` |
| Video | H.264, 1920x1080, 5,843 frames, 243.46 s |

Evidence files:

```text
artifacts/codex_mid360_native_nav_60m_20260716_retry8/report.json
artifacts/codex_mid360_native_nav_60m_20260716_retry8/motion/report.json
artifacts/codex_mid360_native_nav_60m_20260716_retry8/motion/native_navigation.mp4
```

This manifest sets `require_video_artifact=true`; omitting `--record-video` or
failing to render a decodable video blocks the 60 m acceptance result.

This is one accepted run, not the requested `10/10` repeatability result. The
ten-run campaign starts from zero after the single-run chain is stable.

This gate consumes a prebuilt deterministic MuJoCo scene map whose metadata
truthfully records `mujoco_synthetic_map`. That is permitted because the gate
starts at **existing map** and isolates navigation/control. It is not LiDAR
mapping or product map-quality evidence. Live local avoidance still consumes
the 10 Hz MuJoCo LiDAR frames described above.

The run contained no injected moving obstacle (`dynamic_object_frames=0`), so
it does not accept dynamic-object de-ghosting. Candidate diagnostics recorded
hard terrain/collision rejection, but no selected soft terrain penalty; soft
terrain-cost influence also remains a separate scenario gate.

## Native SLAM Variant Boundary

The native-SLAM variant proves saved-map navigation with live native-DDS
localization and local avoidance in MuJoCo. It runs the same C++ SLAM,
traversability, navigation endpoint, OctoPlanner3D, LocalPlanner, PathFollower,
and final command safety used by the field runtime. It is retained for
localization and integrated-stack testing, but it does not block the isolated
60 m navigation function gate.

The native-SLAM variant does this:

```text
MuJoCo MID-360 + IMU
-> C++ livox_sdk2_stream typed DDS
-> C++ Fast-LIO2 localization
-> odometry + map->odom + registered_cloud
-> C++ traversability DDS
-> C++ native nav endpoint
     -> OctoPlanner3D(saved octomap.ot)
     -> LocalPlanner(live registered cloud + traversability)
     -> embedded PathFollower(local_path_body)
     -> independent final command safety
-> typed DDS rt/nav/cmd_vel
-> ThunderV4 RL policy motion
```

It does **not** yet merge live obstacles into the persistent global search map:

```text
live registered_cloud
-> temporary global OctoMap overlay
-> online OctoPlanner3D global replanning while the robot moves
```

Live registered clouds already drive LocalPlanner and command safety. The
remaining limitation is specifically global-map overlay/replanning, not the
local obstacle chain.

## Native SLAM Variant Required Chain

```text
MuJoCo ThunderV4 + MID-360/IMU
-> typed DDS Fast-LIO2 localization
-> saved map.pcd + octomap.ot
-> OctoPlanner3D global_path
-> live traversability + C++ LocalPlanner local_path
-> embedded PathFollower pre-safety command
-> final command safety -> typed DDS cmd_vel
-> ThunderV4 RL policy -> MuJoCo robot motion
```

The goal is not a no-motion preview. The robot must move in MuJoCo and reduce
distance to the goal.

## Native SLAM Variant Required Inputs

| Item | Required | Current Source |
| --- | --- | --- |
| Robot XML | yes | `sim/robots/thunderv4/mjcf/thunderv4.xml` |
| Stairs / harder world XML | yes for advanced gate | `sim/robots/thunderv4/mjcf/thunderv4_stairs.xml` |
| RL policy | yes | `sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.onnx` |
| MID360 pattern | yes | `sim/assets/livox/mid360.npy` |
| MuJoCo LiDAR point cloud | yes | `mujoco_lidar` backend |
| Map package | yes | auto-generated under the run `prepared_assets/same_source_map/` directory from MuJoCo MID-360 scans |
| Global planner | yes | OctoPlanner3D only |
| Local planner | yes | C++ `LocalPlannerCore` inside the native nav endpoint |
| Path follower | yes | C++ `NavLoop` embedded follower |

## Native SLAM Product-Gate Required Outputs

| Output | Purpose |
| --- | --- |
| `map.pcd` | saved MuJoCo point cloud map |
| `octomap.ot` | OctoPlanner3D planning map |
| `metadata.json` | map artifact provenance and hashes |
| `report.json` | machine-readable acceptance result |
| `trajectory.csv` | robot trajectory and command samples |
| `preview.svg` | top-down map/path/trajectory preview |
| MP4 video | required clean visual demo, separate from diagnostic artifacts |

## Acceptance Criteria

Minimum gate:

| Metric | Threshold |
| --- | --- |
| `ok` | `true` |
| `uses_ros` | `false` |
| `policy_loaded` | `true` |
| `plan.path_count` | `> 1` |
| `tracking.nonzero_cmd_count` | `> 0` |
| `tracking.arrived` or `tracking.final_error_m` | `true` or `<= acceptance_radius_m` |

Product-level gate:

| Metric | Threshold |
| --- | --- |
| `plan.diagnostics.constraints.require_ground_support` | `true` |
| `map-source` | `mujoco_lidar` |
| `drive-mode` | `policy` |
| local obstacle checking | enabled |
| `video` | decodable, non-empty, clean product view |
| scene | includes stairs or multi-level terrain |

## Native DDS Closure Command

```powershell
$env:PYTHONPATH='src;.'
python sim/scripts/mujoco/native_navigation_acceptance.py `
  --mode both `
  --domain-id 223 `
  --out-dir artifacts/mujoco_native_navigation_acceptance_final_v7
```

The gate uses two consecutive domains, runs `no_motion` before `motion`, and
requires all owned processes to be gone at cleanup. When no map is supplied it
first scans the generated scene with the MuJoCo MID-360 model and builds a
same-source `map.pcd`, `octomap.ot`, and `metadata.json`. The accepted
2026-07-11 run is recorded in
`artifacts/mujoco_native_navigation_acceptance_final_v7/report.json`.

Accepted evidence:

| Check | Result |
| --- | ---: |
| Overall | passed, no blockers |
| LiDAR map points | 536,904 |
| No-motion global/local path | 5 / 101 points |
| No-motion command publication | 0 final commands |
| Motion global/local path | 4 / 101 points |
| Motion nonzero final commands | 121 |
| MuJoCo executed path | 1.121 m |
| Goal-distance reduction | 0.996 m |
| Map-frame localization XY error | 0.035 m (`<= 0.35 m`) |
| Continuous map tracking | 38 successes, 0 rejections |
| Fast-LIO2 processed scan rate | about 9.85 Hz in motion |
| IMU input rate | about 198.19 Hz in motion |
| ROS used | no |

The report keeps raw `odom->body` drift visible, but localization correctness
is evaluated in the frame actually consumed by navigation:
`map->body = map->odom * odom->body`. A run cannot pass if map tracking is
disabled, has consecutive failures, or exceeds the map-frame error budget.

The saved-map source metadata deliberately says
`mujoco_lidar_ground_truth_registered_map`: map points come from the simulated
MID-360 and are accumulated with MuJoCo truth poses. This gate validates native
saved-map localization and navigation; it does not claim that Fast-LIO2 created
the saved map. Fast-LIO2 owns the live localization outputs used during both
navigation phases.

## Recording Requirements

Product video must prioritize visibility of the robot and environment:

- no top-left telemetry text
- no bottom-right diagnostic route inset
- no large overlay that blocks robot, stairs, doorways, or obstacles
- 1920x1080 or higher when possible
- H.264 transcode for Windows media playback

Diagnostic evidence belongs in separate files:

- `report.json` for planner/tracking metrics
- `trajectory.csv` for executed motion samples
- `local_path_timeseries.jsonl` for per-tick local path curves in map frame
- `local_path_debug.mp4` for top-down OctoMap occupied voxels, global path,
  local path, and executed trail playback
- `octomap_occupied.xyz` for the occupied leaf centers dumped from the exact
  `octomap.ot` consumed by OctoPlanner3D
- `preview.svg` for top-down map, global path, and executed trail
- optional diagnostic video may enable telemetry/inset when debugging

Use `--video-clean` for product recording. Use the older overlay style only for
diagnostic recordings.

The product gate is not complete if local obstacle checking is disabled. A run
without `--check-obstacle` may demonstrate path following, but it must not be
reported as obstacle-aware navigation.

## OctoPlanner3D Definition

The MuJoCo gate calls the same OctoPlanner3D adapter used by the planning
service:

```text
src/nav/services/plan/global_planner/algorithm/octoplanner3d_planner.py
src/nav/services/plan/global_planner/algorithm/octoplanner3d_runtime.py
src/nav/services/plan/global_planner/service.py
```

For this gate, OctoPlanner3D receives the saved map file:

```text
<run>/plan/same_source_map/octomap.ot
```

The map comes from `mujoco_lidar` scan collection in
`sim/scripts/mujoco/saved_map_plan_gate.py`. `NativeMapsService` submits the
saved PCD to `MapPipelineCore`, which builds and transactionally publishes the
OctoMap artifact. The planner is not reading live point clouds during this
gate. The local planner obstacle input is separate: product validation uses
live MuJoCo LiDAR scans from `engine.get_lidar_points()`, not the saved-map PCD.

## Latest Product Gate

### Building Room-To-Room Product Gate

This gate places the goal in the other room of the building, not one meter
ahead of the robot. It uses MuJoCo MID360 point-cloud mapping, builds
`map.pcd -> octomap.ot`, plans with OctoPlanner3D ground support enabled,
follows with the C++ local planner/path follower, and moves the ThunderV4 robot
with the RL policy.

Current status: passed with obstacle checking enabled. The earlier
obstacle-aware failure was caused by feeding the local planner static saved-map
PCD points as near-field obstacles. The current product gate uses live MuJoCo
LiDAR for local obstacle checking and uses the dumped OctoMap occupied voxels
for top-down diagnostics.

```powershell
python sim\scripts\mujoco_navigation_gate.py `
  --out-dir artifacts\mujoco_nav_building_room_to_room_live_obstacle_v3 `
  --map-source mujoco_lidar `
  --scene-preset building `
  --lidar-duration 25.0 `
  --lidar-scans 80 `
  --lidar-vx 0.5 `
  --lidar-wz 0.10 `
  --duration 140 `
  --drive-mode policy `
  --start 2.0 3.0 0.5 `
  --goal 18.0 11.0 0.5 `
  --acceptance-radius-m 1.0 `
  --max-speed 0.45 `
  --check-obstacle `
  --local-obstacle-source live_lidar `
  --obstacle-radius-m 3.0 `
  --max-obstacle-points 1500 `
  --product-acceptance `
  --video-clean `
  --video-out artifacts\mujoco_nav_building_room_to_room_live_obstacle_v3\nav_room_to_room_clean.mp4 `
  --path-video-out artifacts\mujoco_nav_building_room_to_room_live_obstacle_v3\local_path_debug.mp4 `
  --video-width 1920 `
  --video-height 1080 `
  --video-fps 20 `
  --strict `
  --json-out artifacts\mujoco_nav_building_room_to_room_live_obstacle_v3\report.json
```

Accepted obstacle-aware result:

```text
ok=true
acceptance.product_ready=true
acceptance.product_blockers=[]
scene_preset=building
map_source=mujoco_lidar
plan.diagnostics.constraints.require_ground_support=true
plan.diagnostics.constraints.ground_support_xy_radius_cells=2
plan.path_count=85
plan.path_length_m=20.138
plan.diagnostics.path_points=84
plan.diagnostics.goal_reached=true
plan.diagnostics.goal_xy_error_m=0.141422
tracking.arrived=true
tracking.check_obstacle=true
tracking.local_obstacle_source=live_lidar
tracking.final_error_m=0.2497
tracking.nonzero_cmd_count=1068
tracking.near_field_stop_count=0
tracking.local_path_points_avg=97.78
tracking.obstacle_points_avg=1248.93
tracking.obstacle_points_max=1500
tracking.trajectory_length_m=18.6535
tracking.trajectory_points=5338
policy_loaded=true
uses_ros=false
video.exists=true
video.frames=1068
path_debug_video.frames=1068
path_debug_map.source=octomap_occupied
path_debug_map.used_for_path_debug=true
path_debug_map.points=22657
scan.lidar_point_count=3348928
scan.point_count=3349814
```

Artifacts:

```text
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/report.json
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/trajectory.csv
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/preview.svg
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/local_path_timeseries.jsonl
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/local_path_debug.mp4
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/local_path_debug_h264.mp4
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/local_path_debug_mid.png
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/nav_room_to_room_clean.mp4
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/nav_room_to_room_clean_h264.mp4
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/nav_room_to_room_clean_mid.png
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/octomap_occupied.xyz
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/plan/same_source_map/map.pcd
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/plan/same_source_map/octomap.ot
artifacts/mujoco_nav_building_room_to_room_live_obstacle_v3/plan/same_source_map/metadata.json
```

Important finding: the earlier 8-second building map was not sufficient for
room-to-room navigation because the internal wall/door area was under-observed.
It produced a path that a kinematic body could replay but the RL robot collided
near the wall. The accepted product run uses a longer mapping pass before
planning, so OctoPlanner3D routes through the observed door opening.

### Previous Building Short-Range Probe

This earlier result is kept only as a regression smoke test. It is not a
full navigation proof because the goal is still in the same room.

```powershell
python sim\scripts\mujoco_navigation_gate.py `
  --out-dir artifacts\mujoco_nav_building_lidar_policy_long_closeup `
  --map-source mujoco_lidar `
  --scene-preset building `
  --lidar-duration 6.0 `
  --lidar-scans 24 `
  --lidar-vx 0.7 `
  --duration 18 `
  --drive-mode policy `
  --start 2.0 3.0 0.5 `
  --goal 6.0 3.0 0.5 `
  --acceptance-radius-m 0.65 `
  --product-acceptance `
  --video-out artifacts\mujoco_nav_building_lidar_policy_long_closeup\nav_long_closeup.mp4 `
  --video-width 1920 `
  --video-height 1080 `
  --video-fps 20 `
  --strict `
  --json-out artifacts\mujoco_nav_building_lidar_policy_long_closeup\report.json
```

Result:

```text
ok=true
acceptance.product_ready=true
acceptance.product_blockers=[]
scene_preset=building
map_source=mujoco_lidar
plan.diagnostics.constraints.require_ground_support=true
plan.path_count=22
plan.diagnostics.path_points=21
plan.diagnostics.goal_reached=true
plan.diagnostics.goal_xy_error_m=0.141421
tracking.arrived=true
tracking.final_error_m=0.2495
tracking.nonzero_cmd_count=318
tracking.near_field_stop_count=0
tracking.trajectory_length_m=3.7708
tracking.trajectory_points=1596
policy_loaded=true
uses_ros=false
video.exists=true
video.frames=319
scan.lidar_point_count=900937
scan.trajectory_support_point_count=331
```

Artifacts:

```text
artifacts/mujoco_nav_building_lidar_policy_long_closeup/report.json
artifacts/mujoco_nav_building_lidar_policy_long_closeup/trajectory.csv
artifacts/mujoco_nav_building_lidar_policy_long_closeup/preview.svg
artifacts/mujoco_nav_building_lidar_policy_long_closeup/nav_long_closeup.mp4
artifacts/mujoco_nav_building_lidar_policy_long_closeup/nav_long_closeup_frame_mid.png
artifacts/mujoco_nav_building_lidar_policy_long_closeup/plan/same_source_map/map.pcd
artifacts/mujoco_nav_building_lidar_policy_long_closeup/plan/same_source_map/octomap.ot
artifacts/mujoco_nav_building_lidar_policy_long_closeup/plan/same_source_map/metadata.json
```

The building preset needs a short mapping motion before planning. The MuJoCo
MID360 scan can miss immediate support under the start/goal because the sensor
is above the body and has self-occlusion. The gate therefore records a small
trajectory support footprint from MuJoCo odometry while collecting the scan.
This is simulation-only map-production support, not a real-robot SLAM shortcut.

### Corridor Gate

```powershell
python sim\scripts\mujoco_navigation_gate.py `
  --out-dir artifacts\mujoco_nav_product_gate_video `
  --map-source mujoco_lidar `
  --scene-preset corridor `
  --lidar-duration 1.5 `
  --lidar-scans 6 `
  --duration 8 `
  --drive-mode policy `
  --goal 1.0 0.0 0.3 `
  --acceptance-radius-m 0.35 `
  --product-acceptance `
  --video-out artifacts\mujoco_nav_product_gate_video\nav.mp4 `
  --video-width 960 `
  --video-height 540 `
  --video-fps 20 `
  --strict `
  --json-out artifacts\mujoco_nav_product_gate_video\report.json
```

Latest result:

```text
ok=true
acceptance.product_ready=true
acceptance.product_blockers=[]
acceptance.requires_video=true
plan.diagnostics.constraints.require_ground_support=true
plan.path_count=7
tracking.arrived=true
tracking.final_error_m=0.25
tracking.trajectory_length_m=0.7632
tracking.nonzero_cmd_count=112
policy_loaded=true
uses_ros=false
video.exists=true
video.frames=112
```

Artifacts:

```text
artifacts/mujoco_nav_product_gate_video/report.json
artifacts/mujoco_nav_product_gate_video/trajectory.csv
artifacts/mujoco_nav_product_gate_video/preview.svg
artifacts/mujoco_nav_product_gate_video/nav.mp4
artifacts/mujoco_nav_product_gate_video/plan/same_source_map/map.pcd
artifacts/mujoco_nav_product_gate_video/plan/same_source_map/octomap.ot
artifacts/mujoco_nav_product_gate_video/plan/same_source_map/metadata.json
```

## Fixed Gap

The previous MuJoCo gate only passed with `--planner-no-ground-support`. That is
no longer true. The product gate now keeps OctoPlanner3D ground support enabled
and still reaches the goal with the ThunderV4 RL policy.

Two planner fixes made this pass:

- `findNearestFreeCell()` now scans the snap radius and chooses the nearest
  traversable cell instead of returning the first traversable cell in loop order.
- OctoPlanner3D diagnostics now split terminal error into `goal_error_m`,
  `goal_xy_error_m`, and `goal_z_error_m`. `reached_goal` accepts either strict
  3D distance or XY/Z tolerance for supported-voxel snapping.
- OctoPlanner3D ground support now defaults to a 2-cell XY support radius. At
  0.2 m voxel resolution this models a quadruped support polygon instead of
  requiring support directly under the body center.

## Next Required Work

1. Promote the building gate into CI once the C++ OctoMap runtime is available
   in the CI image.
2. Add a harder stairs-crossing scenario where the path must intentionally
   change floor height, not only navigate on one floor inside the building map.
3. Keep the strict product report checks that reject `planner-no-ground-support`
   for product acceptance.
4. Keep `synthetic_hits` only as a planner smoke test; it is not a product
   evidence source.
