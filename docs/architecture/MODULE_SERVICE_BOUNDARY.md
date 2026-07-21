# Module And Service Boundary

Status: current cleanup guide
Audience: runtime/domain maintainers and reviewers
Replaced by: not replaced

This document fixes the naming confusion between LingTu Modules, OS services,
internal service classes, and compatibility adapters.

## Runtime Words

| Word | Means | Lives in | Boundary | Example |
| --- | --- | --- | --- | --- |
| Module | In-process LingTu runtime unit with typed `In` / `Out` ports. | `src/**` | Blueprint wires and callbacks. | `NavigationModule`, `MapService`, `GatewayModule` |
| System service | OS process supervised by systemd or an equivalent launcher. | `scripts/deploy/**`, robot host | DDS, HTTP, files, status JSON, hardware protocol. | `lingtu-slam-dds.service`, `lingtu-nav-dds.service` |
| Internal service class | Plain helper object used by a Module or route layer. | `src/**/services/**` | Python or C++ function calls only. | `ControlCommandService`, `MapAPIService` |
| Adapter / bridge | Explicit protocol boundary to DDS, simulator, or hardware. | `src/**/adapters/**`, `sim/engine/bridge/**` | External protocol or compatibility API. | `CppSlamStatusAdapterModule`, camera DDS adapter |
| Native endpoint | C++ process that owns a native runtime boundary. | `src/**/endpoint/**`, `scripts/deploy/**` | Typed DDS plus files/status. | `navd` |
| Field diagnostics | Offline or Gateway-facing product evidence/audit helpers. | `src/diagnostics/field/**` | Read-only files, reports, HTTP diagnostics, and existing topics. | `evidence`, `gateway_acceptance` |
| Simulation diagnostics | Sim-only closure and dataflow reports. | `sim/diagnostics/**` | Existing reports and simulation artifacts. | `gap_report`, `dataflow_report` |

Rules:

- A Module is not automatically a systemd service.
- A class named `SomethingService` is not automatically an OS service.
- ROS-facing code must stay in explicit adapters, bridges, vendor packages, or
  legacy tools. It must not be required by product Modules.
- Product service names should describe process ownership, not algorithms hidden
  inside the process.

## Internal vs External Boundary

Use this rule before adding a file, class, or dependency:

```text
Module internal = runtime behavior that can run by function calls over
runtime.msgs, In/Out ports, and pure helpers.

External boundary = anything that talks to another process, OS service,
hardware, DDS/LCM/ROS, HTTP/WebSocket, simulator protocol, filesystem-backed
state, or a long-running native endpoint.
```

| Code kind | Counts as | Put it in | May depend on DDS/ROS/LCM? |
| --- | --- | --- | --- |
| Module lifecycle, ports, tick loop, state machine | internal runtime | domain module package, for example `nav/mission` | no |
| Plain validation, policy, planner request shaping | internal helper | domain `services` or `policy` package | no |
| Pure algorithm / hot path calculation | internal compute | `nav/kernel`, `kernels`, or domain algorithm package | no transport imports |
| Route contract, topic binding, schema catalog | boundary contract | `runtime/route_contract`, `config/runtime_graph` | names/schema only, no live client |
| DDS/LCM/ROS reader/writer, endpoint launcher | external adapter | `runtime/endpoints`, `runtime/adapters`, domain `adapters` | yes, isolated here |
| Hardware SDK ownership, reconnect loop, packet capture | external endpoint | `drivers/**/endpoint`, native process package | yes |
| File/database-backed durable map/session state | external storage boundary | service storage subpackage | no transport imports, explicit URI/path APIs |
| Gateway REST/SSE/WS/MCP | external interface | `gateway` | HTTP/WebSocket only here |

Practical test:

- If it can be unit-tested with only `runtime.msgs` objects and no socket,
  file watcher, systemd process, hardware SDK, or protocol client, it is Module
  internal.
- If it needs a topic name, QoS, channel name, URL, device handle, launcher,
  or path to another process' artifact, it is external boundary code.
- If it only translates between the two, it is an adapter. Do not hide adapters
  inside mission/planner/business classes.

## Blueprint, Route Contract, And DDS

There are three separate runtime contracts:

| Contract | Answers | Example |
| --- | --- | --- |
| `runtime_contract` | What semantic data source does this profile use? | `thunder_field`, `mujoco_fastlio2_live` |
| `endpoint_contract` | What endpoint protocol/schema is used at the process boundary? | `thunder_field_dds_v1` |
| `route_contract` | Which canonical topics are external bus topics, and which Module/endpoint ports bind to them? | `robot`, `replay`, `sim` |

Default product rule:

```text
Blueprint wires = internal Module graph
route_contract = external boundary graph
DDS = endpoint/boundary transport unless Blueprint.routed_delivery(...) is explicitly used
```

So a field run can have:

```text
module_transport=local
endpoint_transport=dds
endpoint_contract=thunder_field_dds_v1
route_contract=robot
```

That means Module callbacks still use local ports, while DDS is the typed
boundary for native Livox/SLAM/navigation endpoints. Only call
`Blueprint.routed_delivery(robot())` when the explicit goal is to move selected
Module wires themselves onto routed delivery. Older `Blueprint.route(robot())`
calls are legacy aliases for that behavior and should not be used in new code.

## Current Product Navigation Shape

Field chain to the current base target, `/nav/cmd_vel`:

```text
nav-lidar-network.service
  -> lingtu-livox-dds.service
  -> lingtu-slam-dds.service
  -> lingtu-traversability-dds.service
  -> lingtu-nav-dds.service
  -> /nav/cmd_vel
  -> lingtu-driver.service
  -> remote Brainstem gRPC
```

What each service owns:

| System service | Owns | Main input | Main output |
| --- | --- | --- | --- |
| `nav-lidar-network.service` | LiDAR NIC setup. | none | reachable Livox network |
| `lingtu-livox-dds.service` | Livox/IMU native DDS source. | hardware | `/lidar/raw_frame`, `/imu/raw` |
| `lingtu-slam-dds.service` | Native SLAM/localization. | `/lidar/raw_frame`, `/imu/raw` | `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/tf` |
| `lingtu-traversability-dds.service` | Near-field traversability. | odom, registered cloud | `/nav/traversability` |
| `lingtu-nav-dds.service` | Current native nav endpoint. | odom, TF, goal, cloud, traversability, map artifact, driver control status | `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, `/nav/cmd_vel` when enabled |
| `lingtu.service` | Gateway/API/MCP/status/task entry. | user/task requests, status files | HTTP `5050`, MCP, goal/status commands |

Below `/nav/cmd_vel`:

| System service | Owns | Needed for current base? |
| --- | --- | --- |
| `lingtu-driver.service` | Native real robot command sink, remote Brainstem gRPC client, lease owner, `WalkChecked` ACK path. | yes for real motion |
| `lingtu-thunder-dds-endpoint.service` | Compatibility Python command sink; conflicts with `lingtu-driver.service`. | no |
| `robot-brainstem.service` | Legacy/local low-level control bridge if used by a non-field setup. | no for current `thunder_field` deployment |
| `can-setup.service` | CAN setup for legacy/local bridge deployments. | no for current remote Brainstem gRPC deployment |

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
| `lingtu/assembly` | Declare LingTu products, Module groups, and wires. | no |
| `runtime/blueprint.py` | Materialize one application Module graph. | no |
| `gateway` | API, MCP, status, teleop entry. | one process through `lingtu.service` |
| `localization` Modules/adapters | Normalize SLAM/localization state into LingTu. | no, unless wrapping a native endpoint |
| `maps.service` / `src/maps/services` | Saved-map lifecycle, artifact build, validation, and MapBundle queries. | no; native worker may own async builds later |
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
- `lingtu-driver.service`;
- map artifact generation for OctoPlanner3D;
- `/nav/global_path`, `/nav/local_path`, `/nav/cmd_vel` typed DDS flow.

ROS still present, but should be treated as compatibility or legacy:

| Area | Examples | Keep? |
| --- | --- | --- |
| Explicit adapters | `src/runtime/adapters/ros2/**`, `src/nav/adapters/ros2/**`, `src/localization/adapters/ros2/**` | keep only as opt-in compatibility |
| Python DDS reader | `src/runtime/adapters/dds/reader.py` | keep as DDS compatibility/diagnostics utility, not product control loop |
| Vendor packages | `src/drivers/adapters/ros2/lidar/livox_driver2`, `src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2` | quarantine or make optional vendor bundles |
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
