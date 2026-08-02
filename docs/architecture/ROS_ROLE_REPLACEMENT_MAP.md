# ROS Role Replacement Map

Status: current migration map; ROS remains compatibility only
Audience: runtime, adapter, deployment, and release maintainers
Replaced by: not replaced

LingTu should run without ROS by replacing ROS' current responsibilities with
LingTu-owned contracts and lightweight adapters.  ROS2 remains useful as an
optional compatibility adapter, not as the default runtime substrate.

## Replacement summary

| Current ROS role | Heavy dependency examples | Lightweight replacement | Where it lives |
| --- | --- | --- | --- |
| Message types | `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs` | LingTu-owned `runtime.msgs` and `runtime.portable` frames | `src/runtime/msgs/`, `src/runtime/portable/contracts.py` |
| Communication | ROS topics, services, pub/sub | ModulePort + Blueprint direct wiring for in-process; endpoint adapters for process/network boundaries | `src/lingtu/assembly/`, `src/runtime/runtime_interface.py`, `src/*/adapters/*` |
| Process lifecycle | `rclcpp`/`rclpy` nodes, `launch`, `colcon` | ProductControl with its internal SystemdRunner for field Products; the local Profile CLI only manages development Hosts | `src/lingtu/control.py`, `src/lingtu/systemd.py`, `src/lingtu/run_plan.py`, `cli/` |
| Global planning | ROS launch wrappers, PCT compatibility packages | Headless `GlobalPlanner` with `octoplanner3d` primary; A*/PCT stay explicit compatibility backends | `src/nav/services/plan/global_planner/service.py`, `src/nav/services/plan/global_planner/algorithm/octoplanner3d_planner.py` |
| SLAM/localization ingress | Legacy ROS bridge or implicit topic fallback | Product endpoints pass explicit typed DDS/native adapters; field runtime uses `CppSlamStatusAdapterModule` plus the C++ `lingtu-slam-dds` service, and empty adapter selection fails closed | `src/lingtu/assembly/stacks/slam.py`, `src/localization/adapters/status.py`, `src/runtime/adapters/dds/` |
| Local autonomy | ROS2 `terrainAnalysis`, `localPlanner`, `pathFollower` NativeModule subprocesses | Resolved RunPlans select `nanobind` terrain/local planner plus `nav_kernel` path follower; `lite` uses `simple/pid` | `src/nav/local/`, `src/runtime/profiles/binding_policy.py` |
| Exploration | Local TARE NativeModule subprocess | `explore` is the single Field Product; ProductControl selects its live-mapping or saved-map-localization variant while native navigation owns motion | `config/runtime_graph/products/explore.yaml`, `src/explore/`, `src/nav/cpp/endpoint/` |
| Local hardware registry | Product startup implicitly opening every enabled device in `config/devices.yaml` | Env-resolved native roles own field devices; Host adapters and local Profiles opt into hardware explicitly | `config/runtime_graph/envs/`, `src/lingtu/assembly/products/host_defaults.py`, `src/lingtu/assembly/stacks/system.py` |
| LiDAR acquisition | Local `livox_ros_driver2_node` NativeModule process | Field Products consume raw LiDAR/IMU from the native Livox SDK2 DDS service; the Host graph has no process-start or ROS message-mirror fallback | `src/lingtu/assembly/stacks/lidar.py`, `src/drivers/real/lidar/` |
| Camera acquisition | ROS2 camera bridge fallback | The canonical `camera` registry role selects only native Orbbec, native SHM/DDS, or MuJoCo simulation backends | `src/runtime/contracts/camera.py`, `src/runtime/adapters/perception_gateway.py`, `src/drivers/real/camera/`, `src/drivers/sim/camera/` |
| Rerun visualization | ROS2 visualization bridge fallback | `--rerun` resolves to `runtime.rerun_module.RerunModule` by default; ROS2 visualization overlays require the explicit `enable_ros2_rerun_bridge` compatibility key and are flagged by runtime audit | `src/runtime/rerun_module.py`, `src/runtime/adapters/perception_gateway.py`, `src/lingtu/assembly/stacks/gateway.py` |
| GNSS/RTK acquisition | `ironoa/um982_ros2_driver` publishing ROS2/DDS `/gps/fix` or duplicate device-manager reads | Product GNSS stack runs C++ `lingtu_gnss_dds`, reads `/dev/wtrtk980`, and publishes `rt/gnss/fix`, `rt/gnss/status`, and optional `rt/gnss/odom`; Python and ROS2 readers are compatibility fallbacks only | `src/drivers/real/gnss/`, `scripts/deploy/thunder/lingtu-gnss-dds.service` |
| Ecosystem bridges | `pcl_ros`, `pcl_conversions`, `tf2_ros`, `rosbag`, `livox_ros_driver` | Optional adapters/sidecars that translate into LingTu contracts | `src/*/adapters/ros2/`, `src/nav/local/pcl_ops.py`, replay/endpoint sources |

## Current verified runtime state

- Product `nav` and the saved-map variant of `explore` resolve to
  `octoplanner3d` for global planning. The live `explore` route uses bounded
  rolling segments rather than sending a saved-map goal to GlobalPlanner.
- Product `map` does not run a field global-navigation planner. Product
  navigation/tracking/inspection/exploration use OctoPlanner3D for saved-map
  global planning. A*/direct/PCT/ROS planner adapters are compatibility or
  diagnostics only.
- Product terrain/local/path following resolves to `nanobind`, `nanobind`, and
  `nav_kernel`; Profile `lite` uses `simple`, `simple`, and `pid`.
- The old `portable-lio` / `windows-fastlio2` FastLIO2-like endpoint was removed
  because it was a lightweight estimator, not a hardware-validated Fast-LIO2
  replacement.
- The canonical `lidar` Host role has one source contract and no product-process
  start switch. Field Livox acquisition is owned by `lingtu-livox-dds.service`.
- CLI shutdown does not import ROS2 compatibility code on ordinary no-ROS
  runs; explicit compatibility adapters own their own executor lifecycle.
- Camera acquisition resolves only the canonical `camera` role. Native Orbbec,
  native SHM/DDS, and MuJoCo simulation are explicit backends of that role.
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
- `scripts/deploy/cut_release.sh` is native-first: the tested `navd` endpoint
  package is mandatory, while the Python nanobind kernel is optional. The
  release records one selected global planner; OctoPlanner3D is the default and
  FAR is an explicit occupancy-backed option. Product identity, native control
  mode, and selected process targets are read from the committed exact RunPlan.
  ROS2 compatibility builds and services are excluded from this release path.
- Build, deployment, and script index docs describe native planner kernels,
  Gateway/dataflow diagnostics, and ROS2 perception scripts as explicit
  compatibility paths rather than product defaults. The obsolete ROS OTA
  script tree has been physically removed.
- Real-env Product Hosts now use the `cpp_slam_status` localization adapter over the
  native C++ SLAM/status endpoint. Older `dds_endpoint` adapter wording refers
  to compatibility diagnostics, not the default `env=real` Product runtime.

Next cuts:

1. Replace field SLAM/LIO service ownership with a hardware-validated portable
   LIO adapter.
2. Retire or quarantine legacy ROS2 NativeModule autonomy packages once their
   nanobind/Python backends cover all required field behavior.
3. Keep Gazebo, CMU Unity/TARE, rosbag replay, live perception demos, and
   explicit `ros2_*` bridges as compatibility endpoints only.
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
- Python boundary module: `src/nav/local/pcl_ops.py`.
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
- `src/nav/local/pcl_ops.py` detects optional `lingtu_pcl_ops`
  and falls back to NumPy voxel downsampling.
- `src/drivers/sim/mujoco/adapter.py` converts MuJoCo engine state and
  commands to/from `PortableSensorFrame` / `PortableCommandFrame` and can publish
  those frames through `PortableTopicTransport`.
- `tools/validate/validate_portable_lean_package.py` scans bottom-layer imports and
  reports whether the optional PCL plugin is installed.

This proves the replacement direction without requiring a full rewrite.
