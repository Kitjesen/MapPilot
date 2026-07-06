# Module And Service Boundary

Status: current cleanup guide

This document fixes the naming confusion between LingTu Modules, OS services,
internal service classes, and compatibility adapters.

## Runtime Words

| Word | Means | Lives in | Boundary | Example |
| --- | --- | --- | --- | --- |
| Module | In-process LingTu runtime unit with typed `In` / `Out` ports. | `src/**` | Blueprint wires and callbacks. | `NavigationModule`, `MapService`, `GatewayModule` |
| System service | OS process supervised by systemd or an equivalent launcher. | `scripts/deploy/**`, robot host | DDS, HTTP, files, status JSON, hardware protocol. | `lingtu-slam-dds.service`, `lingtu-nav-dds.service` |
| Internal service class | Plain helper object used by a Module or route layer. | `src/**/services/**` | Python or C++ function calls only. | `ControlCommandService`, `MapAPIService` |
| Adapter / bridge | Explicit protocol boundary to DDS, LCM, ROS 2, simulator, or hardware. | `src/**/adapters/**`, `sim/engine/bridge/**` | External protocol or compatibility API. | `ROS2NavInModule`, `DDSLocalizationAdapterModule` |
| Native endpoint | C++ process that owns a native runtime boundary. | `src/**/endpoint/**`, `scripts/deploy/**` | Typed DDS plus files/status. | `lingtu_nav_native_endpoint` |

Rules:

- A Module is not automatically a systemd service.
- A class named `SomethingService` is not automatically an OS service.
- ROS-facing code must stay in explicit adapters, bridges, vendor packages, or
  legacy tools. It must not be required by product Modules.
- Product service names should describe process ownership, not algorithms hidden
  inside the process.

## Current Product Navigation Shape

Field chain to the current base target, `/nav/cmd_vel`:

```text
nav-lidar-network.service
  -> lingtu-livox-dds.service
  -> lingtu-slam-dds.service
  -> lingtu-traversability-dds.service
  -> lingtu-nav-dds.service
  -> /nav/cmd_vel
```

What each service owns:

| System service | Owns | Main input | Main output |
| --- | --- | --- | --- |
| `nav-lidar-network.service` | LiDAR NIC setup. | none | reachable Livox network |
| `lingtu-livox-dds.service` | Livox/IMU native DDS source. | hardware | `/lidar/raw_frame`, `/imu/raw` |
| `lingtu-slam-dds.service` | Native SLAM/localization. | `/lidar/raw_frame`, `/imu/raw` | `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/tf` |
| `lingtu-traversability-dds.service` | Near-field traversability. | odom, registered cloud | `/nav/traversability` |
| `lingtu-nav-dds.service` | Current native nav endpoint. | odom, TF, goal, cloud, traversability, map artifact | `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, optional `/nav/cmd_vel` |
| `lingtu.service` | Gateway/API/MCP/status/task entry. | user/task requests, status files | HTTP `5050`, MCP, goal/status commands |

Below `/nav/cmd_vel`:

| System service | Owns | Needed for current base? |
| --- | --- | --- |
| `lingtu-thunder-dds-endpoint.service` | Real robot command sink. | no |
| `robot-brainstem.service` | Robot low-level control bridge. | no |
| `can-setup.service` | CAN setup. | no |

Current `lingtu-nav-dds.service` still contains multiple internal parts:

```text
goal receiver
  -> OctoPlanner3D global planner
  -> global_path publisher
  -> target selector
  -> LocalPlannerCore
  -> PathFollowerCore
  -> cmd_vel publisher gate
```

That is acceptable for the first native endpoint. The split route is recorded in
`docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md` and the active priorities in
`docs/plans/current-roadmap.md`.

## Module Groups

| Module group | Job | Should own process lifecycle? |
| --- | --- | --- |
| `runtime/blueprints` | Compose Modules and wires. | no |
| `gateway` | API, MCP, status, teleop entry. | one process through `lingtu.service` |
| `localization` Modules/adapters | Normalize SLAM/localization state into LingTu. | no, unless wrapping a native endpoint |
| `nav/services/maps.py` | Saved-map lifecycle facade. | no |
| `nav/services/plan/**` | Planner contracts and algorithm backends. | no by default; OctoPlanner3D can become a native service |
| `nav/local/**` | terrain, local planner, path follower Modules/kernels. | no by default |
| `nav/safety/**` | SafetyRing and CmdVelMux Modules. | should become the final command gate service later |
| `drivers/**` | Real and simulated robot adapters. | only hardware endpoints own OS process lifecycle |

## ROS State

Short answer: product navigation is ROS-free on the native DDS path; the
repository is not ROS-free.

ROS-free on the current product path:

- `lingtu-livox-dds.service`;
- `lingtu-slam-dds.service`;
- `lingtu-traversability-dds.service`;
- `lingtu-nav-dds.service`;
- map artifact generation for OctoPlanner3D;
- `/nav/global_path`, `/nav/local_path`, `/nav/cmd_vel` typed DDS flow.

ROS still present, but should be treated as compatibility or legacy:

| Area | Examples | Keep? |
| --- | --- | --- |
| Explicit adapters | `src/runtime/adapters/ros2/**`, `src/nav/adapters/ros2/**`, `src/localization/adapters/ros2/**` | keep only as opt-in compatibility |
| Vendor packages | `src/drivers/real/lidar/livox_ros_driver2`, `src/drivers/real/camera/OrbbecSDK_ROS2` | quarantine or make optional vendor bundles |
| Legacy systemd | `scripts/deploy/s100p/*.service` using `ros2 run` | not product default |
| ROS simulation gates | Gazebo/ROS bridge scripts under `sim/**` | keep as compatibility tests, not product proof |
| Legacy demos/tools | `scripts/perception/live_*.py`, old OTA colcon scripts | archive or mark legacy |
| Docs | historical plans and archived papers | archive stale claims when touched |

## Next Cleanup Order

1. Keep this document and `NAVIGATION_RUNTIME_DATAFLOW.md` as the visible map.
2. Add one product-runtime audit test: product profiles must not import or
   launch ROS unless the endpoint explicitly says compatibility.
3. Move or mark legacy ROS launch/systemd/OTA scripts as compatibility-only.
4. Split `lingtu-nav-dds` internally before splitting processes.
5. Extract global planner service, then command safety gate.

Do not delete vendor ROS packages until the native camera/Livox replacements are
confirmed for every profile that still references them.
