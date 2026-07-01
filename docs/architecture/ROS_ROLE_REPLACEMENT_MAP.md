# ROS Role Replacement Map

LingTu should run without ROS by replacing ROS' current responsibilities with
LingTu-owned contracts and lightweight adapters.  ROS2 remains useful as an
optional compatibility adapter, not as the default runtime substrate.

## Replacement summary

| Current ROS role | Heavy dependency examples | Lightweight replacement | Where it lives |
| --- | --- | --- | --- |
| Message types | `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs` | LingTu-owned `runtime.msgs` and `runtime.portable` frames | `src/runtime/msgs/`, `src/runtime/portable/contracts.py` |
| Communication | ROS topics, services, pub/sub | ModulePort + Blueprint direct wiring for in-process; endpoint adapters for process/network boundaries | `src/runtime/blueprints/`, `src/runtime/runtime_interface.py`, `src/*/adapters/*` |
| Process lifecycle | `rclcpp`/`rclpy` nodes, `launch`, `colcon` | LingTu CLI/runtime/profile resolver + normal Python/CMake/Cargo processes | `lingtu.py`, `cli/`, `src/runtime/profiles/` |
| Global planning | ROS launch wrappers, PCT compatibility packages | Headless `GlobalPlanner` with `octoplanner3d` primary; A*/PCT stay explicit compatibility backends | `src/nav/services/plan/global_planner/service.py`, `src/nav/services/plan/global_planner/backends/octoplanner3d/` |
| SLAM/localization ingress | Implicit `SlamBridgeModule` fallback to ROS2/DDS topics | Product endpoints pass explicit typed DDS adapters (`dds_endpoint` for field); empty adapter selection fails closed and ROS2 requires explicit `ros2_slam_bridge` | `src/runtime/blueprints/adapters/mapping_slam.py`, `src/runtime/blueprints/stacks/slam.py`, `src/runtime/adapters/dds/` |
| Local autonomy | ROS2 `terrainAnalysis`, `localPlanner`, `pathFollower` NativeModule subprocesses | Product profiles resolve to `nanobind` terrain/local planner plus `nav_kernel` path follower; `lite` uses `simple/pid` | `src/nav/local/`, `src/runtime/profiles/binding_policy.py` |
| Exploration | Local TARE NativeModule subprocess | Product `tare_explore` resolves to LingTu-native wavefront/traversable frontier modules; external TARE is endpoint compatibility only | `src/runtime/profiles/catalog/product_intents.py`, `src/runtime/blueprints/stacks/exploration_goal_sources.py` |
| Local device registry | Product startup implicitly opening every enabled device in `config/devices.yaml` | Field and lite endpoints set `enable_device_manager=false`; device registry remains an explicit local hardware-management mode | `src/runtime/profiles/catalog/endpoints.py`, `src/runtime/blueprints/stacks/system.py` |
| LiDAR acquisition | Local `livox_ros_driver2_node` NativeModule process | Field product endpoint sets `enable_lidar=false` and consumes raw LiDAR/IMU through typed DDS endpoint topics; local Livox driver is only reachable through the explicit `lidar_start_driver` compatibility key and is flagged by runtime audit | `src/runtime/profiles/catalog/endpoints.py`, `src/runtime/blueprints/stacks/composition.py`, `src/drivers/real/lidar/` |
| Camera acquisition | ROS2 camera bridge fallback | Perception uses a registered camera bridge or driver-native camera source by default; ROS2 camera bridge requires the explicit `enable_ros2_camera_bridge` compatibility key and is flagged by runtime audit | `src/runtime/blueprints/adapters/perception_gateway.py`, `src/runtime/blueprints/stacks/perception.py`, `src/runtime/profiles/binding_policy.py` |
| Rerun visualization | ROS2 visualization bridge fallback | `--rerun` resolves to `runtime.rerun_module.RerunModule` by default; ROS2 visualization overlays require the explicit `enable_ros2_rerun_bridge` compatibility key and are flagged by runtime audit | `src/runtime/rerun_module.py`, `src/runtime/blueprints/adapters/perception_gateway.py`, `src/runtime/blueprints/stacks/gateway.py` |
| GNSS/RTK acquisition | `ironoa/um982_ros2_driver` publishing ROS2/DDS `/gps/fix` or duplicate device-manager reads | Product GNSS stack passes `/dev/wtrtk980` into `GnssModule`, which reads UART through `GnssSerialDriver`; DDS `/gps/fix` and `GnssBridgeModule` remain explicit compatibility fallbacks | `src/runtime/blueprints/stacks/system.py`, `src/localization/gnss_module.py`, `src/localization/gnss_serial_driver.py` |
| Ecosystem bridges | `pcl_ros`, `pcl_conversions`, `tf2_ros`, `rosbag`, `livox_ros_driver` | Optional adapters/sidecars that translate into LingTu contracts | `src/*/adapters/ros2/`, future `src/nav/pcl_ops/`, replay/endpoint sources |

## Current verified runtime state

- Product `nav`, `explore`, `tare_explore`, `super_lio`, and
  `super_lio_relocation` resolve to `octoplanner3d` for global planning.
- Product `map` resolves to `astar`; `thunder-lite` resolves to `direct`.
- Product terrain/local/path following resolves to `nanobind`, `nanobind`, and
  `nav_kernel`; `thunder-lite` uses `simple`, `simple`, and `pid`.
- The old `portable-lio` / `windows-fastlio2` FastLIO2-like endpoint was removed
  because it was a lightweight estimator, not a hardware-validated Fast-LIO2
  replacement.
- LiDAR input is endpoint-first by default. Enabling `LidarModule` subscribes to
  an existing Livox DDS/endpoint stream; starting the legacy local Livox ROS2
  driver requires `lidar_start_driver=true` and is a runtime-audit violation for
  product/portable profiles.
- CLI shutdown does not import ROS2 compatibility code on ordinary no-ROS
  runs; it only shuts down the compat executor if `runtime.adapters.ros2.context` was
  already loaded by an explicit compatibility adapter.
- Camera input no longer falls back to the removed driver ROS2 camera bridge.
  MuJoCo and other driver-native camera sources stay in-graph.
- Rerun visualization no longer falls back to `gateway.visualization.rerun_bridge` by
  default. The default `--rerun` stack uses `runtime.rerun_module`; ROS2 overlay
  subscriptions require `enable_ros2_rerun_bridge=true`.
- `python lingtu.py rerun` starts the Gateway-backed viewer
  (`scripts/visualization/rerun_gateway_live.py`) by default. The old ROS2 topic
  viewer remains available as `python lingtu.py rerun --ros2`.
- `python lingtu.py doctor` is Gateway/ModulePort-first by default. Legacy
  ROS2 topic and `rclpy` subscription checks require `--ros2` or
  `LINGTU_DOCTOR_ROS2=1`.
- Robot-side `scripts/lingtu doctor` is Gateway/dataflow-first by default.
  ROS2 graph, topic, and camera-topic checks require `--ros2`.
- Manager camera snapshots, navigation commands, and Rerun launch now use
  Gateway endpoints by default instead of direct ROS2 topic readers.
- Feishu and Telegram monitor bots collect status through Gateway endpoints and
  no longer import `rclpy`/`std_msgs` or embed default credentials.
- `scripts/deploy/cut_release.sh` is native-first: it requires `_nav_kernel` and
  OctoPlanner3D native/headless artifacts by default, while ROS2 compatibility
  install package checks require `LINGTU_RELEASE_REQUIRE_ROS2_COMPAT=1`.
- Build, deployment, and script index docs describe native planner kernels,
  Gateway/dataflow diagnostics, and legacy OTA/perception scripts as explicit
  compatibility paths rather than product defaults.
- Field profiles now use the `dds_endpoint` localization adapter with typed DDS
  message bindings. Portable no-ROS LIO extraction and hardware validation are
  still separate SLAM algorithm work.

Next cuts:

1. Replace field SLAM/LIO service ownership with a hardware-validated portable
   LIO adapter.
2. Retire or quarantine legacy ROS2 NativeModule autonomy packages once their
   nanobind/Python backends cover all required field behavior.
3. Keep Gazebo, CMU Unity/TARE, rosbag replay, legacy OTA, live perception demos,
   and explicit `ros2_*` bridges as compatibility endpoints only.
4. Run closed-loop Thunder field validation for OctoPlanner3D global planning,
   nanobind local planning, endpoint-only command egress, and Gateway-first
   operations before claiming full ROS2 removal.

## Dependency direction

Correct direction:

```text
runtime.msgs / runtime.portable contracts
  <- MuJoCo adapter
  <- JSONL/replay adapter
  <- endpoint adapter
  <- hardware adapter
  <- ROS2 adapter only under ros-compat
  <- optional PCL/native adapter
```

Forbidden direction:

```text
core -> rclpy / rclcpp / sensor_msgs / pcl_ros / mujoco / brainstem_api / lcm
```

## PCL policy

PCL itself can be useful, but it is not a default runtime dependency.

- Default local planning must work without PCL.
- Optional native package name: `lingtu_pcl_ops`.
- Python boundary module: `src/nav/local/pcl_ops_backend.py`.
- Public API uses `Nx3`/`Nx4` arrays or LingTu messages, never PCL types.
- Native plugin may internally use PCL and expose small functions such as:
  - `voxel_downsample_xyzi(points, voxel_size)`
  - future: radius filter, k-d tree nearest/radius, terrain preprocessing
- `pcl_ros` and `pcl_conversions` remain `ros-compat` only.

Install target once native wheels exist:

```bash
pip install lingtu-pcl-ops
```

For developers building the plugin, use Conan/vcpkg/CI internally; normal users
should not need to install PCL manually.

## First implemented slice

The first code slice is intentionally small:

- `src/runtime/portable/contracts.py` defines ROS-free bottom-layer frames.
- `src/runtime/portable/topics.py` defines canonical `PortableTopic` specs. It is
  topic metadata, not a transport implementation.
- `src/runtime/portable/topic_transport.py` binds `PortableTopic` to the existing
  mature transport backends: Local/SHM/LCM/DDS. Use LCM/DDS/SHM for real
  cross-process adapter communication; `LocalTopicHub` is only a no-dependency
  unit-test/local probe helper.
- `src/nav/local/pcl_ops_backend.py` detects optional `lingtu_pcl_ops`
  and falls back to NumPy voxel downsampling.
- `src/drivers/sim/mujoco/adapter.py` converts MuJoCo engine state and
  commands to/from `PortableSensorFrame` / `PortableCommandFrame` and can publish
  those frames through `PortableTopicTransport`.
- `tools/validate/validate_portable_lean_package.py` scans bottom-layer imports and
  reports whether the optional PCL plugin is installed.

This proves the replacement direction without requiring a full rewrite.
