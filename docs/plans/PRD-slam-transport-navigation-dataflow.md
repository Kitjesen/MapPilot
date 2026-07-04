# PRD: Native SLAM Transport and Navigation Data Flow

## Decision

| Topic | Decision |
| --- | --- |
| Message contract | `src/runtime/msgs` remains the Python module model package. |
| `src/message` | Wire-contract registry for typed DDS topic-to-type bindings, with native IDL in `src/message/idl/lingtu_slam.idl` and C++ metadata in `src/message/cpp/dds_topics.hpp`. |
| First native Fast-LIO2 bring-up | Co-located process, local Module callbacks, enqueue-only sensor ingress. |
| Endpoint autonomy if SLAM is unstable | Split SLAM into a supervised process; LiDAR/IMU enter through typed CycloneDDS. |
| Generic pickle/JSON DDS | Not product-grade. Testing/compat only for unregistered topics. Product topics bind to IDL and generated native DDS message types. |
| Protobuf | Not on the point-cloud hot path. Add only for a real generated cross-language boundary. |
| LCM | Legacy endpoint compatibility and JSONL feed validation. Not the default field bus. |
| SHM | Only for large point clouds/images after a process split is proven necessary. |

## 现场修正版：C++ DDS 主链路

结论：机器人主链路不再依赖 `cyclonedds-python`。传感器、SLAM、导航命令这种运行时数据边界统一走 C++ CycloneDDS C API + `idlc` 生成类型；Python 只保留 Gateway/API、状态展示、任务编排和离线测试适配。

板子上 apt 当前提供的是 `cyclonedds-dev 0.8.2` 和 `cyclonedds-tools 0.8.2`。这套包已经包含 C API、头文件、`idlc` 和 CMake 配置，足够支撑 C++ runtime。`cyclonedds==0.10.5` 是 Python 绑定包版本，不是现场 C++ 主链路要求；它会在板子上重新编译 Python 扩展，并要求构建脚本找到匹配的 CycloneDDS 开发包。这个依赖不稳定，不能作为机器人启动门槛。

版本策略：

| 层级 | 运行时要求 | 说明 |
| --- | --- | --- |
| 机器人主链路 | apt `cyclonedds-dev` + `cyclonedds-tools`，当前接受 0.8.2 | C++ 使用 `dds/dds.h` 和 `idlc -l c` 生成类型，避免 Python ABI/包构建问题。 |
| Python Gateway | 不要求 `cyclonedds-python` | Gateway 读取 C++ runtime 的低频状态快照，或消费本地 Module 端口。 |
| Python DDS adapter | 仅测试/兼容 | 可用于开发机调试 typed DDS，不允许成为现场必需依赖。 |
| 未来升级 | 先升级 C++ CycloneDDS 包，再做全链路验证 | 不用为 Python 包版本倒逼系统库升级。 |

当前落地边界：

```text
Livox-SDK2 C++ callback
  -> C++ DDS publisher: rt/lidar/raw_frame, rt/imu/raw
  -> C++ SLAM runtime: feedLidar/feedImu/tick
  -> C++ DDS publisher: rt/slam/odometry, rt/slam/map_cloud, rt/slam/localization_health
  -> C++ status snapshot: /tmp/lingtu_slam_status.json
  -> Python Gateway reads snapshot for UI/API health only
```

这里的 JSON 快照不是实时控制通道，只用于 Gateway 健康页、状态审计和现场调试。导航闭环如果跨进程，必须继续走 C++ DDS/native endpoint；不能让 Python DDS 订阅成为路径规划或安全控制的硬依赖。

## Current Implementation Status

Status: partially implemented, with the C++ DDS runtime path now present.

Completed:

- `LidarModule` owns the MID-360 source and publishes `raw_scan`, `scan`,
  `imu`, and `alive`.
- Blueprint wiring connects `LidarModule.raw_scan -> SlamModule.lidar_raw_scan`
  and `LidarModule.imu -> SlamModule.lidar_imu`.
- `SlamModule` uses enqueue-only callbacks for raw Livox frames, `PointCloud2`,
  and IMU, then drains bounded queues into its runner boundary.
- Native and endpoint localization outputs share the same downstream
  odometry, map-cloud, localization-status, and map-artifact contract.
- Server-side communication/runtime evidence has been exercised through the
  Gateway + ModulePorts + server artifact validation flow. The canonical
  evidence entry is `docs/07-testing/ALGORITHM_VALIDATION_FLOW.md`; historical
  server runs are summarized in `docs/plans/simulation-closure-plan.md`.
- `src/localization/slam/cpp/cyclone_runtime.cpp` provides the process-split
  C++ CycloneDDS runtime. It subscribes to `rt/lidar/raw_frame` and
  `rt/imu/raw` using `src/message/idl/lingtu_slam.idl`, calls
  `ISlamBackend::feedLidar()` / `feedImu()`, ticks the backend, and publishes
  `rt/slam/odometry`, `rt/slam/state_at_scan`, `rt/slam/registered_cloud`,
  `rt/slam/map_cloud`, `rt/slam/localization_quality`, and
  `rt/slam/localization_health`.
- `scripts/deploy/thunder/lingtu-slam-dds.service` is the explicit field
  service for that C++ runtime. `scripts/deploy/thunder/install_services.sh`
  exposes `slam-dds` for the SLAM sidecar and `field-cpp` for DDS endpoint plus
  C++ SLAM runtime installation.
- `src/nav/services/endpoint/cpp/nav_native_endpoint.cpp` is the product
  C++ CycloneDDS navigation endpoint. It subscribes to SLAM odometry,
  SLAM registered cloud, `rt/nav/goal_pose`, `rt/nav/global_path`, and
  `rt/nav/cancel`. When `LINGTU_ACTIVE_OCTOMAP` is set, goals call
  OctoPlanner3D in-process, then the C++ `NavLoop`
  (`LocalPlannerCore -> PathFollowerCore`) publishes typed
  `rt/nav/global_path`, `rt/nav/local_path`, and `rt/nav/cmd_vel` without
  Gateway HTTP polling.
- The old Gateway-polling `nav_cyclone_endpoint.cpp` bridge was removed.
  Command/API testing should use `lingtu_nav_native_endpoint` with command
  publishing disabled.
- `scripts/deploy/thunder/lingtu-nav-dds.service` is the explicit field service
  for the C++ navigation endpoint. `install_slam_dds_service.sh` installs it
  together with the Livox DDS and SLAM DDS services.
- The native DDS path now targets the CycloneDDS C API plus `idlc` C-generated
  types, so the robot only needs apt `cyclonedds-dev`/`cyclonedds-tools`; it
  must not require `cyclonedds-python` or CycloneDDS-CXX.
- `lingtu_slam_cyclone_runtime` writes a low-rate status snapshot when started
  with `--status-json`. This gives Gateway status visibility without importing
  Python DDS bindings.

Not complete:

- The in-process Python `SlamModule` can still load the contract
  `_PythonSlamRunner`; the product field path should instead use the C++ DDS
  SLAM runtime plus the C++ status snapshot adapter.
- The endpoint `thunder_field` path uses `localization_adapter=cpp_slam_status`,
  `native_navigation_endpoint=lingtu-nav-dds`, `enable_nav_in=false`,
  `enable_nav_out=false`, and `enable_lidar=false`. The old Python DDS
  nav.in/nav.out adapters have been removed.
- `cyclonedds-python` is not a robot main-path dependency. The remaining Python
  DDS endpoint service is only the temporary Thunder Brainstem command sink; if
  Python DDS is unavailable, it must fail closed instead of crash-looping. A
  robot that needs real motor commands without `cyclonedds-python` must run a
  dedicated C++ Thunder control sink that subscribes `rt/nav/cmd_vel` and calls
  the hardware control service; that sink is separate from `lingtu-nav-dds`.
- The C++ nav endpoint now consumes `rt/nav/traversability` as a typed
  `lingtu.dds.OccupancyGrid` risk grid and passes it into LocalPlannerCore.
- The older `dds_runtime.cpp` path has been compile- and startup-smoke-tested
  with the contract backend on a ROS2/Livox-equipped Ubuntu host. The native
  CycloneDDS runtime has been compile-, startup-, and native pub/sub-tested
  with the contract backend. It still needs target-machine validation with
  `LINGTU_SLAM_FASTLIO2=ON`, live MID-360 input, and map-save/relocalization
  gates.

## Enqueue-Only Sensor Ingress

`enqueue-only` means the callback path must return quickly:

```text
LiDAR callback -> validate cheap fields -> append frame to bounded SLAM queue -> return
IMU callback   -> validate cheap fields -> append sample to bounded SLAM queue -> return
```

Forbidden in the callback path:

```text
sync LiDAR/IMU
run Fast-LIO2 builder.process()
save maps
publish large derived clouds
wait for another sensor stream
hold a Python lock while native processing runs
```

The Fast-LIO2 backend owns scan/IMU synchronization:

```text
bounded lidar_queue + imu_ring
  -> wait until IMU covers LiDAR scan end
  -> syncPackage()
  -> builder.process()
  -> latest output snapshot
```

## Runtime Data Flow

| Step | Owner | Input | Output | Rule |
| --- | --- | --- | --- | --- |
| Sensor source | `drivers.real.lidar` | Livox SDK2 or typed DDS | raw Livox frame, IMU | Preserve timestamp, sequence, line, tag, per-point time. |
| Module ingress | `LidarModule` | source callbacks | `raw_scan`, `scan`, `imu`, `alive` | MID-360 LiDAR and its embedded IMU stay under one device owner. |
| SLAM interface | `SlamModule` | `lidar_raw_scan`, `lidar_imu`, optional GNSS/visual odom | SLAM outputs | No path or velocity outputs. |
| SLAM runner boundary | Python contract runner now; C++ `FastLioBackend` binding pending | queued LiDAR/IMU | odom, registered cloud, map cloud, status | Own synchronization, buffers, drop counters, processing. Current Python runtime proves the contract path, not real Fast-LIO execution. |
| Map lifecycle | `nav.services.maps` | map cloud and saved artifacts | active map artifacts | Planner consumes artifacts, not SLAM internals. |
| Global planning | `nav.services.plan` | goal, odom, active map | `global_path`, plan status | Reject stale odom, missing map, lost localization. |
| Local planning | `nav.local` / path follower | global path, odom, local maps | `local_path`, `cmd_vel` candidate | Hold on lost localization or unhandled map jump. |
| Safety | `nav.safety` | velocity candidates, SLAM status | final allowed command | Final motion authority. |
| Operator | `gateway`, `runtime.adapters.lcm` | REST/WS/MCP/LCM | goals, status, endpoint messages | No safety bypass. |

## Producer Choice

| Producer | Exists now | Output | Decision |
| --- | --- | --- | --- |
| `LidarModule` | yes | `scan`, `raw_scan`, `imu`, `alive` | Keep as MID-360 owner. |
| standalone `ImuModule` for Livox IMU | no | none | Do not add. It splits one physical device for no gain. |
| future body/external IMU | no | `Imu`, `alive`, `sync_status` | Add only when a second physical IMU exists. |

## Transport Choice

| Data | Bring-up | Product process split | QoS / policy |
| --- | --- | --- | --- |
| raw Livox frame | local callback | typed CycloneDDS | best effort, keep-last small depth, deadline/liveliness. |
| IMU | local callback | typed CycloneDDS | bounded depth, timestamp-order checks, deadline/liveliness. |
| odometry | local callback | typed DDS adapter if external | reliable/latest with freshness. |
| localization status | local callback | typed DDS endpoint | latest plus counters. |
| registered/map cloud | local callback | SHM only if needed | latest/drop old; throttle debug clouds. |
| saved map | filesystem artifact | filesystem artifact | durable metadata gate. |
| commands/status endpoint | typed DDS | typed DDS | seq, timestamp, TTL, zero-on-stale. |

## Fast-LIO2 Inputs

| Input | Interface | Required fields |
| --- | --- | --- |
| raw LiDAR | `SlamModule.lidar_raw_scan` | `schema_version`, `stamp_ns`, `sequence`, `frame_id`, `point_count`, points. |
| point layout | `LivoxPointFrame.points` | `x`, `y`, `z`, `intensity`, `offset_time_ns`, `tag`, `line`, `flags`. |
| IMU | `SlamModule.lidar_imu` | `stamp_ns/ts`, `frame_id`, angular velocity, linear acceleration, optional orientation/covariance. |
| calibration | config | `r_il`, `t_il`, `time_diff_lidar_to_imu`, `acc_scale`, frames, range/filter parameters. |

## Fast-LIO2 Outputs

| Output | Consumer | Meaning |
| --- | --- | --- |
| `odometry` | navigation, maps, gateway, safety | Current pose. |
| `state_estimation_at_scan` | diagnostics | Scan-time pose. |
| `registered_cloud` | gateway/debug | Current registered scan. |
| `map_cloud` | map layers, gateway | Live SLAM map observation. |
| `saved_map` | gateway/map service | Saved-map preview. |
| `localization_status` | navigation, safety, gateway | State, reason, freshness, buffers, drops. |
| `localization_quality` | gateway, safety | Numeric confidence. |
| `alive` | runtime health | Backend liveness. |
| `map_odom_tf` | map consumers | map-to-odom relation. |
| `map_frame_jump_event` | navigation/local/path follower | Hold/replan trigger. |
| `gnss_fusion_health` | safety/gateway | GNSS health. |
| `scene_mode` | diagnostics | Informational only. |

SLAM must not output:

```text
global_path
local_path
cmd_vel
```

## Co-Located Vs Process Split

| Gate | Co-located allowed | Split required |
| --- | --- | --- |
| Callback latency | LiDAR/IMU feed returns under budget | Callback blocks behind processing. |
| Fault isolation | Bench mapping/replay | Endpoint autonomy needs SLAM crash isolation. |
| CPU scheduling | One process meets latency after GIL release | Needs CPU affinity, RT priority, or independent restart. |
| Observability | queue/drop/stale counters visible | Need transport liveliness/deadline too. |
| Motion | No autonomous motion target yet | Endpoint autonomy with unstable SLAM. |

## Readiness Gates Before Robot Motion

| Gate | Requirement |
| --- | --- |
| Ingest starvation | Slow SLAM processing cannot block LiDAR/IMU ingress. |
| Queue bounds | LiDAR/IMU queues have explicit bounds, drop policy, and counters. |
| GIL | native feed/process calls release GIL where C++ can block. |
| Freshness | status includes `odom_age_ms`, `pose_fresh`, `map_cloud_fresh`, `motion_hold_required`. |
| Safety | `LOST`, `FAILED`, `DIVERGED`, or `pose_fresh=false` holds motion. |
| DDS QoS | LiDAR/IMU typed DDS QoS is pinned before process split. |
| Timebase | raw frames include schema version, frame id, source clock, monotonic sequence. |
| Map jump | jump event includes `seq`, `dt_m`, `dyaw_deg`, previous/next metadata. |
| Endpoint TTL | DDS command endpoint requires seq, timestamp, TTL, zero-on-stale. |

## Implementation Order

1. Completed contract layer:
   - `LidarModule` source callbacks and ports;
   - blueprint raw-scan and IMU wires into `SlamModule`;
   - enqueue-only `SlamModule` callbacks and bounded queues;
   - map-cloud/status fan-out into maps, navigation, gateway, and safety.
2. Native Fast-LIO runtime binding:
   - replace the `_PythonSlamRunner` contract fallback with the C++
     `FastLioBackend` bridge;
   - keep callback ingress enqueue-only;
   - let native processing own synchronization and map outputs;
   - release the GIL around blocking native feed/process calls.
3. Native SLAM status:
   - add freshness, hold flags, queue high-water, drop counters.
4. Sensor schema:
   - add version, frame id, source clock, monotonic sequence to raw frames.
5. Safety/navigation gates:
   - stop/hold on failed or stale localization;
   - hold after map-frame jump until fresh post-jump state.
6. Typed DDS process split:
   - C++ runtime exists in `src/localization/slam/cpp/cyclone_runtime.cpp`;
   - use `src/message/idl/lingtu_slam.idl` as the native wire contract;
   - ROS2 message headers/rclcpp remain only in the compatibility executable
     `lingtu_slam_dds_runtime`;
   - define QoS per product topic before promoting the split runtime to motion
     authority.

## Current Checkpoint

- Product entrypoints are locked to six modes: `teleop`, `teleop_avoid`, `map`,
  `tracking`, `nav`, and `inspection`.
- `sim_nav` is the first no-ROS navigation gate. It must use
  `octoplanner3d` for global planning and `nanobind` / `nav_kernel` for the
  local planner and path follower.
- Startup must fail closed when `lingtu_nav_kernel` or OctoPlanner3D native /
  headless runtime is missing. It must not fall back to `simple`, `pid`, A*, or
  direct path and then claim the production chain works.

## Deletion Gates

Delete old ROS bridges, old local planner shims, and migration directories only
after these checks pass:

- `sim_nav` runs goal -> OctoPlanner3D global path -> C++ local planner ->
  path follower -> `cmd_vel` without ROS.
- Native Livox/SLAM mock runs `feedImu` + `feedLidar` -> odometry/map outputs
  without ROS topics.
- Downstream consumers use only SLAM localization/map outputs; no code connects
  a SLAM trajectory path to `global_path`, `local_path`, waypoint, or `cmd_vel`.
