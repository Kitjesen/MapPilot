# Navigation Runtime Dataflow

This document records the shipped navigation-base dataflow. It is scoped to
mapping, saved-map planning, local planning, path following, safety, and command
output. Inspection, semantic memory, and UI-only telemetry are intentionally out
of scope.

## Transport Model

LingTu topics are runtime stream names, not ROS 2 topics.

## SLAM Runtime Rule

Do not write or ship Python SLAM. MuJoCo may simulate the robot, MID-360 raw
frames, and IMU, but pose estimation, map building, and saved-map
relocalization must come from the same native C++ SLAM/localization runtime used
on the robot (`lingtu_slam_cyclone_runtime` / `lingtu-slam-dds.service`) or an
explicit external native SLAM service. Python code may adapt streams, status,
reports, and test evidence only; it must not become a SLAM backend.

| Boundary | Transport | Payload shape | ROS 2 required |
| --- | --- | --- | --- |
| Hardware / external runtime to LingTu | typed DDS | IDL structs under `message.dds_types` | no |
| LingTu module to module, same process | `Out.publish()` to wired `In._deliver()` callback | Python objects | no |
| LingTu worker subprocess boundary | SHM transport when enabled | serialized runtime messages | no |
| OctoPlanner3D global planner | subprocess stdin/stdout | JSON request/result | no |
| Map artifact conversion | subprocess + files | `map.pcd` to `octomap.ot` | no |
| Command output to Thunder brainstem | endpoint then gRPC | `Twist` to brainstem `Walk(Vector3)` | no |

Default internal wiring is callback delivery:

```text
ModuleA.out.publish(msg)
  -> Blueprint wire callback
  -> ModuleB.in._deliver(msg)
  -> ModuleB's subscribed handler
```

Only explicit cross-boundary wires use `dds`, `shm`, or `lcm`.

## External DDS Contract

Registered typed DDS topics are published as `rt/...` DDS topic names. Example:
LingTu `/nav/cmd_vel` becomes DDS `rt/nav/cmd_vel`.

| Direction | LingTu stream | DDS type | Runtime type |
| --- | --- | --- | --- |
| endpoint -> LingTu | `/tf` | `lingtu.dds.TFMessage` | frame transforms |
| endpoint -> LingTu | `/tf_static` | `lingtu.dds.TFMessage` | static frame transforms |
| endpoint -> LingTu | `/lidar/raw_frame` | `lingtu.dds.LivoxFrame` | `PointCloud2` |
| endpoint -> LingTu | `/imu/raw` | `lingtu.dds.Imu` | `Imu` |
| endpoint -> LingTu | `/slam/odometry` | `lingtu.dds.Odometry` | `Odometry` |
| endpoint -> LingTu | `/slam/registered_cloud` | `lingtu.dds.PointCloud2` | `PointCloud2` |
| endpoint -> LingTu | `/slam/map_cloud` | `lingtu.dds.PointCloud2` | `PointCloud2` |
| endpoint -> LingTu | `/slam/saved_map_cloud` | `lingtu.dds.PointCloud2` | `PointCloud2` |
| endpoint -> LingTu | `/slam/localization_health` | `lingtu.dds.Text` | `dict` |
| endpoint -> LingTu | `/slam/localization_quality` | `lingtu.dds.Float32` | `float` |
| endpoint -> LingTu | `/nav/goal_pose` | `lingtu.dds.PoseStamped` | `PoseStamped` |
| endpoint -> LingTu | `/nav/cancel` | `lingtu.dds.Text` | `str` |
| endpoint -> LingTu | `/nav/semantic/instruction` | `lingtu.dds.Text` | `str` |
| LingTu -> endpoint | `/nav/global_path` | `lingtu.dds.Path` | `Path` |
| LingTu -> endpoint | `/nav/local_path` | `lingtu.dds.Path` | `Path` |
| LingTu -> endpoint | `/nav/way_point` | `lingtu.dds.PoseStamped` | `PoseStamped` |
| LingTu -> endpoint | `/nav/cmd_vel` | `lingtu.dds.TwistStamped` | `Twist` |

## Main Runtime Dataflow

| Step | Producer | Consumer | Data | Type | Transport |
| ---: | --- | --- | --- | --- | --- |
| 1 | Livox runtime | DDS endpoint / localization adapter | raw LiDAR frame | `LivoxFrame` -> `PointCloud2` | DDS |
| 2 | IMU runtime | DDS endpoint / localization adapter | IMU | `Imu` | DDS |
| 3 | SLAM/localizer | localization adapter | odometry | `Odometry` | DDS |
| 4 | SLAM/localizer | localization adapter | registered cloud | `PointCloud2` | DDS |
| 5 | SLAM/localizer | localization adapter | live map cloud | `PointCloud2` | DDS |
| 6 | SLAM/localizer | localization adapter | localization health/quality | `dict`, `float` | DDS |
| 7 | localization adapter | Navigation, maps, local planner, path follower, safety, Gateway | odometry fan-out | `Odometry` | callback |
| 8 | localization adapter | MapService, occupancy, elevation, voxel, terrain, Gateway | map cloud fan-out | `PointCloud2` / `MapCloudFrame` | callback |
| 9 | MapService | map directory | saved source map | `map.pcd` | file |
| 10 | MapArtifactBuilder | map directory | OctoMap artifact | `octomap.ot` or `octomap.bt` | subprocess + file |
| 11 | MapArtifactBuilder | map directory | artifact proof | `metadata.json` | file |
| 12 | MapService | map directory | static occupancy | `occupancy.npz` | file |
| 13 | Gateway/MCP/DDS | GoalService or Navigation | goal | `PoseStamped` | HTTP/MCP/DDS then callback |
| 14 | Navigation | OctoPlanner3D runtime | global plan request | JSON | subprocess |
| 15 | OctoPlanner3D runtime | Navigation | global path result | JSON -> `Path` | subprocess |
| 16 | Navigation | LocalPlanner | global path | `Path` | callback |
| 17 | Navigation | LocalPlanner | current waypoint | `PoseStamped` | callback |
| 18 | Terrain | LocalPlanner | near-field terrain cloud | `PointCloud2` | callback |
| 19 | Terrain | LocalPlanner | traversability | `dict` | callback |
| 20 | Map layers | TraversabilityCost | occupancy/elevation/ESDF | `OccupancyGrid`, `dict` | callback |
| 21 | TraversabilityCost | Navigation | global risk gate | `dict` | callback |
| 22 | TraversabilityCost | LocalPlanner | ESDF relay | `dict` | callback |
| 23 | LocalPlanner | PathFollower, SafetyRing, Gateway | local path | `Path` | callback |
| 24 | LocalPlanner | PathFollower | control hint | `dict` | callback |
| 25 | PathFollower | CmdVelMux | autonomous velocity | `Twist` | callback |
| 26 | Teleop / VisualServo / Navigation recovery | CmdVelMux | override or recovery velocity | `Twist` | callback |
| 27 | CmdVelMux | driver / nav out / endpoint | muxed velocity | `Twist` / DDS `TwistStamped` | callback or DDS |
| 28 | endpoint | brainstem | walk command | gRPC `Walk(Vector3)` | gRPC |
| 29 | SafetyRing / Geofence | Navigation and driver | stop command | `int` (`0`, `1`, `2`) | callback |

For the `thunder_field` product endpoint, the DDS navigation boundary is owned
by the C++ `lingtu-nav-dds` service, not Python `nav.in` / `nav.out` adapters.
It subscribes to `rt/nav/goal_pose`, `rt/slam/odometry`,
`rt/slam/registered_cloud`, `rt/nav/traversability`, and publishes
`rt/nav/global_path`, `rt/nav/local_path`, and `rt/nav/cmd_vel`.

## Core Runtime Types

| Type | Required fields |
| --- | --- |
| `PointCloud2` | `points: float32 Nx3/Nx4`, `ts`, `frame_id`, `height`, `width`, `fields`, `is_dense` |
| `MapCloudFrame` | `points`, `mode`, `ts`, `frame_id`, `map_id`, `source`, `sequence`, `metadata` |
| `Imu` | `orientation`, `angular_velocity`, `linear_acceleration`, covariances, `ts`, `frame_id` |
| `Odometry` | `pose`, `twist`, `ts`, `frame_id`, `child_frame_id` |
| `PoseStamped` | `pose`, `ts`, `frame_id` |
| `Path` | `poses: list[PoseStamped]`, `ts`, `frame_id` |
| `Twist` | `linear: Vector3`, `angular: Vector3` |
| `OccupancyGrid` | `grid: int8 HxW`, `resolution`, `origin`, `ts`, `frame_id` |

`dict` payloads are used for diagnostics, risk grids, and metadata-heavy state:
`localization_status`, `mission_status`, `traversability`, `fused_cost`,
`esdf_field`, `slope_grid`, and `control_hint`.

## OctoPlanner3D Map Inputs

OctoPlanner3D is the product global planner. It does not consume the live local
planner terrain stream. It consumes a saved map artifact bundle.

Required for a valid OctoPlanner3D plan:

| Input | Required | Source | Purpose |
| --- | --- | --- | --- |
| `octomap.ot`, `octomap.bt`, or `.octomap` | yes | built from saved `map.pcd` | 3D occupancy tree used by OctoPlanner3D |
| `metadata.json` | yes | MapArtifactBuilder | proves source, frame, and same-source artifact validity |
| `map.pcd` | yes for artifact gate | SLAM map save | source map and same-source hash anchor |
| metadata `frame_id` | yes | metadata | must match expected saved-map/planning frame, normally `map` |
| metadata `artifacts.map_pcd.sha256` | yes | metadata | source hash for same-source validation |
| metadata `artifacts.map_pcd.point_count` | yes | metadata | must be positive |
| metadata `artifacts.octomap.uri` | yes | metadata | points to the OctoMap file |
| metadata `artifacts.octomap.sha256` | yes | metadata | verifies the OctoMap file |
| metadata `artifacts.octomap.source_map_sha256` | yes | metadata | must match `map_pcd.sha256` |
| `occupancy.npz` | optional | MapService | loaded when present as static 2D occupancy/preblocked context |

The runtime request sent to `octoplanner3d_headless` is JSON:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "map_path": "/path/to/octomap.ot",
  "map_source": {
    "kind": "octomap_file",
    "path": "/path/to/octomap.ot",
    "format": "ot",
    "frame": "map"
  },
  "start": [0.0, 0.0, 0.0],
  "goal": [2.0, 1.0, 0.0],
  "options": {
    "planner_family": "octoplanner3d_constrained_global_planner",
    "search_algorithm": "octomap_3d_astar",
    "constraint_model": "quadruped_bounding_cylinder_ground_support",
    "robot_radius": 0.25,
    "max_iterations": 800000,
    "snap_search_radius_cells": 12,
    "require_ground_support": true,
    "floor_change_penalty": 4.0,
    "max_step_height": 0.45,
    "same_floor_preference": true,
    "obstacle_clearance_radius_cells": 4
  }
}
```

The result is JSON:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "ok": true,
  "path": [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0], [2.0, 1.0, 0.0]],
  "reached_goal": true,
  "diagnostics": {}
}
```

`Navigation` converts the returned `path` into runtime `Path` and publishes it
as `global_path`.

## Local Planner Inputs

Local planning uses live runtime data. It does not require the saved map package
to run, although it follows the `global_path` created from the saved map.

| LocalPlanner port | Type | Required for normal navigation | Producer | Notes |
| --- | --- | --- | --- | --- |
| `odometry` | `Odometry` | yes | SLAM/localizer or driver fallback | robot pose and yaw |
| `waypoint` | `PoseStamped` | yes | Navigation | current target selected from global path |
| `global_path` | `Path` | yes | Navigation | corridor reference |
| `terrain_map` | `PointCloud2` | yes for obstacle-aware planning | Terrain | near-field terrain cloud |
| `terrain_map_ext` | `PointCloud2` | optional | Terrain | wider context cloud |
| `traversability` | `dict` | yes for risk scoring | Terrain | native backend uses traversability grid |
| `clear_path` | `bool` | optional | Navigation | reset/clear local path |
| `map_odom_tf` | `dict` | required when map/odom differ | SLAM/localizer | map-to-odom transform |
| `map_frame_jump_event` | `dict` | optional | SLAM/localizer | clears stale path state after relocalization jumps |
| `boundary` | `PointCloud2` | optional | no default producer | reserved overlay |
| `added_obstacles` | `PointCloud2` | optional | no default producer | manual/dynamic obstacle overlay |
| `check_obstacle` | `bool` | optional | no default producer | toggles overlay checking |
| `esdf` | `dict` | wired but reserved | TraversabilityCost | stored, not yet primary native scoring input |

LocalPlanner outputs:

| Output | Type | Consumer |
| --- | --- | --- |
| `local_path` | `Path` | PathFollower, SafetyRing, Gateway/nav out |
| `control_hint` | `dict` | PathFollower |
| `alive` | `bool` | health/diagnostics |

## Path Follower and Command Outputs

| Module | Input | Type | Output | Type |
| --- | --- | --- | --- | --- |
| PathFollower | `odometry`, `local_path`, `control_hint`, `map_odom_tf`, `map_frame_jump_event` | `Odometry`, `Path`, `dict` | `cmd_vel` | `Twist` |
| CmdVelMux | `teleop_cmd_vel`, `visual_servo_cmd_vel`, `recovery_cmd_vel`, `path_follower_cmd_vel` | `Twist` | `driver_cmd_vel` | `Twist` |
| SafetyRing | `odometry`, `path`, `cmd_vel`, `mission_status`, `localization_status` | mixed | `stop_cmd`, `safety_state`, `execution_eval` | `int`, structured status |

CmdVelMux priority:

| Source | Priority |
| --- | ---: |
| teleop | 100 |
| visual servo | 80 |
| recovery | 60 |
| path follower | 40 |

Final command path:

```text
PathFollower.cmd_vel
  -> CmdVelMux.path_follower_cmd_vel
  -> CmdVelMux.driver_cmd_vel
  -> nav.out.cmd_vel or driver.cmd_vel
  -> DDS /nav/cmd_vel when endpoint output is selected
  -> endpoint
  -> brainstem Walk(Vector3)
```

## What Must Be Validated On The Robot

Minimum field proof for the navigation base:

1. DDS receives `/lidar/raw_frame`, `/imu/raw`, `/slam/odometry`, and `/slam/map_cloud`.
2. `map save` creates `map.pcd`, `octomap.ot`, `occupancy.npz`, and `metadata.json`.
3. OctoPlanner3D artifact gate reports `ok: true`.
4. `Navigation` publishes a non-empty `global_path`.
5. `LocalPlanner` publishes a non-empty `local_path`.
6. `PathFollower` publishes non-zero `cmd_vel`.
7. `CmdVelMux` forwards `driver_cmd_vel` from the expected active source.
8. `SafetyRing` is not holding a hard stop.
9. endpoint receives `/nav/cmd_vel` and brainstem accepts `Walk(Vector3)`.
