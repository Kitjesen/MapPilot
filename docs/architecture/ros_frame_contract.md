# LingTu ROS Frame Contract

This document records the frame and topic contract used by the navigation
stack. It describes the boundary that modules, ROS 2 launch files, simulators,
and App/Web telemetry must preserve.

The canonical Python source is `src/core/runtime_interface.py`. The
human-readable mirror is `config/topic_contract.yaml`. Real hardware, Gazebo,
MuJoCo, and CMU Unity must differ only at the source-adapter layer; every
adapter normalizes into the same runtime frames, topics, and algorithm inputs.

## Canonical Frames

| Frame | Meaning | Owner |
| --- | --- | --- |
| `map` | Saved-map and global planning frame. Goals and global paths are expressed here. | localizer / map manager |
| `odom` | Continuous local odometry frame. LIO publishes odometry here before global relocalization correction. | Fast-LIO2 / Point-LIO / simulation bridge |
| `body` | LingTu control body frame. Local planner paths and velocity commands are body-relative. | runtime contract |
| `base_link` | Robot model root link used by URDF/MJCF/Gazebo assets. | robot model |
| `lidar_link` | LingTu canonical LiDAR frame after source-adapter normalization. | runtime contract |
| `livox_frame` | Physical MID-360/raw driver frame alias accepted only at adapter/calibration boundaries. | hardware adapter |
| `camera_link` | LingTu canonical camera frame. | runtime contract |
| `world` | Simulator world frame only. It must be aliased into `map` or `odom` before entering LingTu runtime topics. | simulator |

`body` and `base_link` are not interchangeable inside the codebase. The bridge
boundary must make the alias explicit:

```text
robot model:      base_link
LingTu runtime:   body
bridge contract:  base_link == body, published as odom -> body
```

## Required TF Shape

```text
map -> odom -> body
                 -> lidar_link
                 -> camera_link
```

Body axes are fixed as `x` forward, `y` left, `z` up. LiDAR source adapters
must prove this with known front/left/up point checks before their clouds are
accepted as LingTu runtime data.

`lidar_link` is the LingTu canonical LiDAR frame. `livox_frame` is a physical
driver alias; runtime gates may accept it only when an adapter explicitly
declares and normalizes the alias. Gate reports must show whether an alias was
accepted or whether canonical `lidar_link` was observed directly.

For simulation, `world` may be identical to `map` at startup, but it should not
appear as the frame on `/nav/*` topics. Publish `map -> odom` as identity when
there is no relocalization correction.

## Navigation Topic Frames

Topic frame validation is topic-specific and runtime-specific. The table below
is a human-readable mirror of `runtime_topic_allowed_frame_ids()` in
`src/core/runtime_interface.py`; update that source first, then update this
table and the mirror tests.

| Topic | Thunder field default frame | General allowed frames | Thunder field evidence required | Thunder field allowed frames |
| --- | --- | --- | --- | --- |
| `/nav/lidar_scan` | `lidar_link` | `lidar_link` | yes | `lidar_link` |
| `/nav/imu` | `lidar_link` | `lidar_link` | yes | `lidar_link` |
| `/nav/odometry` | `odom` | `odom`, `map` | yes | `odom`, `map` |
| `/nav/state_estimation_at_scan` | `odom` | `odom` | no | `odom` |
| `/nav/registered_cloud` | `body` | `body` | yes | `body` |
| `/nav/map_cloud` | `map` | `map`, `odom` | yes | `map` |
| `/nav/cumulative_map_cloud` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/saved_map_cloud` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/exploration_grid` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/traversable_frontiers` | `map` | `map`, `odom` | no | `map` |
| `/nav/frontier_candidate` | `map` | `map`, `odom` | no | `map` |
| `/nav/terrain_map` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/terrain_map_ext` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/global_path` | `map` | `map`, `odom` | yes | `map` |
| `/nav/local_path` | `map` | `map`, `odom`, `body` | yes | `map`, `odom`, `body` |
| `/nav/goal_pose` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/way_point` | `map` | `map`, `odom` | no | `map`, `odom` |
| `/nav/cmd_vel` | `body` | `body` | yes | `body` |

`/nav/map_cloud` and `/nav/global_path` are deliberately stricter on real
S100P than in replay or simulation: real runtime evidence must prove both are
in `map`. `/nav/map_cloud` is a map/world cloud, never body-relative points.
`/nav/local_path` may be fixed-frame or body-frame because the local planner
and path follower boundary is allowed to expose either representation, but the
report must declare which frame was observed. `/nav/registered_cloud` is the
current body-frame scan for local planning; it must not be treated as a map
product.

## Raw Sensor And Algorithm Boundary

| Surface | Inputs | Outputs | Owner |
| --- | --- | --- | --- |
| Fast-LIO mapping/localization | `/nav/lidar_scan`, `/nav/imu` | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud` | SLAM |
| Fast-LIO raw validation | `/points_raw`, `/imu_raw` | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud` | MuJoCo/raw-sensor gate |
| Exploration strategy | `/nav/odometry`, `/nav/map_cloud`, `/nav/exploration_grid` | `/exploration/way_point` | TARE or frontier |
| Global planning | `/nav/odometry`, `/nav/map_cloud`, `/exploration/way_point` or `/nav/goal_pose` | `/nav/global_path`, `/nav/way_point` | LingTu navigation |
| Local planning/following | `/nav/odometry`, `/nav/registered_cloud`, `/nav/global_path` | `/nav/local_path`, `/nav/cmd_vel` | LingTu autonomy |

The Python `NavigationModule` deliberately rejects goals, costmaps, and
odometry that do not match its configured `planning_frame_id`. Product saved-map
navigation fixes that frame to `map`: 3D goals, PCT inputs, global paths, and
Navigation odometry are all consumed in `map`.

SLAM, native local planners, and lower-level ROS components may still operate in
`odom`, but the `map -> odom` correction must be applied by the SLAM
bridge/localizer/adapter before data enters Python Navigation. Navigation must
not silently mix `odom` odometry with `map` goals or PCT paths.

## Gazebo/GZ Integration Boundary

Gazebo should be treated as another simulator backend, not as a special
navigation stack. The first supported Gazebo shape is:

```text
Gazebo world/model sensors
  -> ros_gz_bridge or a small Python bridge
  -> /nav/odometry, /nav/registered_cloud, /camera/*
  -> ROS2SimDriverModule / native ROS planners
  -> /nav/cmd_vel
  -> Gazebo velocity controller
```

The bridge must not publish `base_link` or `world` directly into LingTu module
ports. It should translate model frames into the canonical contract above.

## Current Gaps

1. Gazebo/GZ now has a first-class `sim_gazebo` profile and launch entry, but
   the default robot is a navigation proxy model for ROS-native validation, not
   a full Thunder/S100P dynamics model.
2. `body`/`base_link` aliasing is still bridge-level convention, not enforced by
   a real TF validation gate.
3. `/nav/map_cloud` semantics are mixed across legacy simulation code. New code
   must keep map/world clouds separate from body-frame registered clouds, and
   Thunder field evidence must reject `/nav/map_cloud` outside `map`.
4. The ROS local planner may consume `odom` and `body` frames, while the product
   Python navigation path consumes `map`. Tests must state whether they validate
   the lower-level ROS path or the product saved-map Navigation path.
5. Gazebo sensor/controller plugins still need runtime verification on a ROS 2
   host. Static tests verify the model/topic contract; they do not prove Gazebo
   is publishing every bridged topic at rate.
