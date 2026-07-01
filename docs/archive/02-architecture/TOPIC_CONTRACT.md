# Topic Interface Contract

This contract is consumed by the **C++ ROS2 layer** (SLAM, terrain
analysis, local planner, path follower) and by the algorithm-bridge
launch files in `launch/profiles/`. The Module-First Python stack uses
in-process Out/In ports and the transports defined in `runtime.transport.*`,
so most of these names appear only on the C++ side or when running
SLAM as a systemd service.

The single source of truth for the topic names is
[`config/topic_contract.yaml`](../../config/topic_contract.yaml). What
follows is a human-readable summary.

## Why this exists

A given algorithm (Fast-LIO2, Point-LIO, PCT, etc.) ships with its own
topic names. We remap them onto a stable `/nav/*` namespace at launch
time so:

- Switching SLAM or planner = switching launch profile, no code or YAML
  change in the consumers.
- Upper layers (gRPC gateway, status aggregator, health monitor) only
  know the `/nav/*` names.

The launch profiles that perform the remap live in `launch/profiles/`:

```
launch/profiles/
  slam_fastlio2.launch.py
  slam_pointlio.launch.py
  slam_stub.launch.py
  localizer_icp.launch.py
  planner_pct.launch.py
  planner_pct_py.launch.py
  planner_far.launch.py
  planner_stub.launch.py
```

These are loaded by `localization.SLAMModule` / `localization.SlamBridgeModule` and the
PCT adapter via `NativeModule`. They are not the system entry point â€?
`lingtu.py` is.

## Standard topics

### Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/lidar_scan` | `livox_ros_driver2/CustomMsg` | LiDAR points |
| `/nav/imu` | `sensor_msgs/Imu` | IMU |

### SLAM

| Topic / service | Type | Description |
|-----------------|------|-------------|
| `/nav/odometry` | `nav_msgs/Odometry` | Odometry |
| `/nav/registered_cloud` | `sensor_msgs/PointCloud2` | Body-frame cloud |
| `/nav/map_cloud` | `sensor_msgs/PointCloud2` | Map-frame cloud |
| `/nav/save_map` | `interface/srv/SaveMaps` | Save map service |

### Localization

| Topic / service | Type | Description |
|-----------------|------|-------------|
| `/nav/localization_quality` | `std_msgs/Float32` | ICP score |
| `/nav/relocalize` | `interface/srv/Relocalize` | Relocalize service |
| `/nav/relocalize_check` | `interface/srv/IsValid` | Relocalize completion check |

### Global planning

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/global_path` | `nav_msgs/Path` | Global path |
| `/nav/planner_status` | `std_msgs/String` | One of `IDLE`, `PLANNING`, `SUCCESS`, `FAILED`, `WARN_STUCK`, `STUCK` |
| `/nav/adapter_status` | `std_msgs/String` | `pct_path_adapter` JSON: `{"event": "...", "index": N, "total": N}` |

### Local autonomy

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/terrain_map` | `sensor_msgs/PointCloud2` | Base terrain |
| `/nav/terrain_map_ext` | `sensor_msgs/PointCloud2` | Extended terrain |
| `/nav/local_path` | `nav_msgs/Path` | Local path |
| `/nav/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity command |
| `/nav/stop` | `std_msgs/Int8` | E-stop request |
| `/nav/slow_down` | `std_msgs/Int8` | Slow-down request |
| `/nav/way_point` | `geometry_msgs/PointStamped` | Local target |
| `/nav/speed` | `std_msgs/Float32` | Speed scale |
| `/nav/map_clearing` | `std_msgs/Float32` | Optional terrain clearing radius |
| `/nav/cloud_clearing` | `std_msgs/Float32` | Optional cloud clearing radius |
| `/nav/navigation_boundary` | `geometry_msgs/PolygonStamped` | Optional boundary |
| `/nav/added_obstacles` | `sensor_msgs/PointCloud2` | Optional injected obstacles |
| `/nav/check_obstacle` | `std_msgs/Bool` | Optional obstacle-check toggle |

### Top-level

| Topic | Type | Description |
|-------|------|-------------|
| `/nav/goal_pose` | `geometry_msgs/PoseStamped` | High-level navigation goal (used by external bridges; the Module-First Python stack uses in-process ports instead) |

### TF frames

| Frame | Description |
|-------|-------------|
| `map` | Global map frame |
| `odom` | Odometry frame |
| `body` | Robot body frame |

TF chain: `map` â†?`odom` (published by Localizer / PGO) â†?`body`
(published by Fast-LIO2 / Point-LIO).

## Algorithm profiles

A profile is a small launch file that starts an algorithm node and
remaps its native topics onto the standard contract.

### Selecting a profile

The algorithm-bridge launches are not used directly by the operator.
`SLAMModule` and `SlamBridgeModule` decide which one to load based on
`slam_profile` in the active CLI profile (`fastlio2`, `pointlio`,
`localizer`, `bridge`, `none`). Likewise the PCT planner adapter loads
its own profile.

If you do need to invoke them on their own â€?e.g. for a SLAM-only debug
session â€?they accept the same launch arguments as before:

```bash
ros2 launch launch/profiles/slam_fastlio2.launch.py
ros2 launch launch/profiles/planner_pct.launch.py map_path:=/path/to/tomogram
```

### Adding a new algorithm

To add a SLAM backend, e.g. LIO-SAM:

1. Create `launch/profiles/slam_liosam.launch.py`:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package="lio_sam",
               executable="lio_sam_node",
               name="lio_sam_node",
               output="screen",
               remappings=[
                   ("/lio_sam/mapping/odometry", "/nav/odometry"),
                   ("/lio_sam/mapping/cloud_registered", "/nav/registered_cloud"),
                   ("/lio_sam/mapping/map_global", "/nav/map_cloud"),
                   ("/imu_raw", "/nav/imu"),
                   ("/points_raw", "/nav/lidar_scan"),
               ],
           ),
       ])
   ```

2. Register a backend factory under
   `runtime.registry` so `slam("liosam")` resolves to a Module that
   launches this profile via `NativeModule`.

3. No other consumer file needs to change â€?they all read `/nav/*`.

The same recipe applies to a new global planner: emit
`/nav/global_path`, `/nav/planner_status`, and `/nav/way_point`, then
register a backend.

## Backwards compatibility

Every C++ node that reads these topics accepts a parameter override so
old topic names can still be remapped (`grpc_gateway.yaml` is a typical
override file). The defaults baked into the binaries match the contract
above.
