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
| Message types | `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs` | LingTu-owned Host messages and native IDL | `src/runtime/msgs/`, `src/message/idl/` |
| Communication | ROS topics, services, pub/sub | ModulePort + Blueprint direct wiring for in-process; endpoint adapters for process/network boundaries | `src/lingtu/assembly/`, `src/runtime/runtime_interface.py`, `src/*/adapters/*` |
| Process lifecycle | `rclcpp`/`rclpy` nodes, `launch`, `colcon` | ProductControl resolves one Product in `real` or `sim`; SystemdRunner owns field processes and the sim switch owns direct children | `src/lingtu/control.py`, `src/lingtu/real/systemd.py`, `src/lingtu/sim/`, `src/lingtu/run_plan.py` |
| Global planning | ROS planner launch wrappers | Native `navd` with OctoPlanner3D by default and FAR as an explicit option | `src/nav/cpp/planning/global/`, `src/nav/cpp/endpoint/nav/runtime/goal/` |
| SLAM/localization ingress | Legacy ROS bridge or implicit topic fallback | Product endpoints pass explicit typed DDS/native adapters; field runtime uses `CppSlamStatusAdapterModule` plus the C++ `lt-slam` service, and empty adapter selection fails closed | `src/lingtu/assembly/stacks/slam.py`, `src/localization/adapters/status.py`, `src/runtime/adapters/dds/` |
| Local autonomy | ROS2 `terrainAnalysis`, `localPlanner`, `pathFollower` subprocesses | ProductControl starts the native DDS navigation endpoint selected by the Product RunPlan | `config/runtime_graph/`, `src/nav/cpp/endpoint/` |
| Exploration | Local TARE NativeModule subprocess | `explore` is the single Field Product; ProductControl selects its live-mapping or saved-map-localization variant while native navigation owns motion | `config/runtime_graph/products/explore.yaml`, `src/explore/`, `src/nav/cpp/endpoint/` |
| Local hardware registry | Product startup implicitly opening every enabled device in `config/devices.yaml` | Env-resolved native services own field devices; `devices.yaml` is camera configuration and read-only inventory only | `config/runtime_graph/envs/`, `config/devices.yaml` |
| LiDAR acquisition | Local `livox_ros_driver2_node` NativeModule process | Field Products consume raw LiDAR/IMU from the native Livox SDK2 DDS service; the Host graph has no process-start or ROS message-mirror fallback | `src/lingtu/assembly/stacks/lidar.py`, `src/drivers/real/lidar/` |
| Camera acquisition | ROS2 camera bridge fallback | The canonical `camera` registry role selects only native Orbbec, native SHM/DDS, or MuJoCo simulation backends | `src/runtime/contracts/camera.py`, `src/lingtu/assembly/stacks/perception.py`, `src/drivers/real/camera/`, `src/drivers/sim/camera/` |
| GNSS/RTK acquisition | `ironoa/um982_ros2_driver` publishing ROS2/DDS `/gps/fix` or duplicate Python readers | C++ `lingtu_gnss_dds` reads `/dev/wtrtk980` and publishes `rt/gnss/fix`, `rt/gnss/status`, and optional `rt/gnss/odom`; there is no Python device reader | `src/drivers/real/gnss/`, `scripts/deploy/thunder/lt-gnss.service` |
| Ecosystem bridges | `pcl_ros`, `pcl_conversions`, `tf2_ros`, `rosbag`, `livox_ros_driver` | Optional adapters/sidecars that translate into LingTu contracts | explicit ROS adapters and replay tools |

## Current verified runtime state

- Product `nav` and the saved-map variant of `explore` resolve to
  `octoplanner3d` for global planning. The live `explore` route uses bounded
  rolling segments rather than sending a saved-map goal to GlobalPlanner.
- Product `map` does not run a field global-navigation planner. Product
  navigation/tracking/inspection/exploration use native OctoPlanner3D for
  saved-map global planning; FAR is the only explicit alternative.
- Product global/local planning and path following run inside native `navd` in
  both `real` and `sim`.
- The old `portable-lio` / `windows-fastlio2` FastLIO2-like endpoint was removed
  because it was a lightweight estimator, not a hardware-validated Fast-LIO2
  replacement.
- The canonical `lidar` Host role has one source contract and no product-process
  start switch. Field Livox acquisition is owned by `lt-lidar.service`.
- CLI shutdown does not import ROS2 compatibility code on ordinary no-ROS
  runs; explicit compatibility adapters own their own executor lifecycle.
- Camera acquisition resolves only the canonical `camera` role. Native Orbbec,
  native SHM/DDS, and MuJoCo simulation are explicit backends of that role.
- `python tools/visualization/rerun_gateway_live.py` starts the Gateway-backed
  viewer. ROS2 viewers remain explicit compatibility tools.
- Robot-side `PYTHONPATH=src python -m diagnostics.field.doctor` uses only
  native services and Gateway/dataflow evidence.
- Manager camera snapshots, navigation commands, and Rerun launch now use
  Gateway endpoints by default instead of direct ROS2 topic readers.
- Release operators run the complete native build and then
  `package_native_release.sh`. `deploy_robot.sh` remains a separate,
  Product-scoped checkout build and activation path.
- Build, deployment, and script index docs describe native planner kernels,
  Gateway/dataflow diagnostics, and native Product operations. The obsolete ROS
  compatibility and OTA script trees have been physically removed.
- Real-env Product Hosts now use the `cpp_slam_status` localization adapter over the
  native C++ SLAM/status endpoint. Older `dds_endpoint` adapter wording refers
  to compatibility diagnostics, not the default `env=real` Product runtime.

Next cuts:

1. Replace field SLAM/LIO service ownership with a hardware-validated portable
   LIO adapter.
2. Keep removed ROS2 autonomy packages absent; native `navd` owns the required
   planning and motion behavior.
3. Keep Gazebo, CMU Unity/TARE, rosbag replay, live perception demos, and
   explicit `ros2_*` bridges as compatibility endpoints only.
4. Run closed-loop Thunder field validation for OctoPlanner3D global planning,
   native local planning, endpoint-only command egress, and Gateway-first
   operations before claiming full ROS2 removal.

## Dependency direction

Correct direction:

```text
runtime.msgs
  <- Host Modules

native IDL
  <- native DDS endpoints
```

Forbidden direction:

```text
core -> rclpy / rclcpp / sensor_msgs / pcl_ros / mujoco / brainstem_api / lcm
```
