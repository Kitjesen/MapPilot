# Module And Service Boundary

Status: current cleanup guide
Audience: runtime/domain maintainers and reviewers
Replaced by: not replaced

This document fixes the naming confusion between LingTu Modules, OS services,
internal service classes, and compatibility adapters.

## Runtime Words

| Word | Means | Lives in | Boundary | Example |
| --- | --- | --- | --- | --- |
| Module | In-process LingTu runtime unit with typed `In` / `Out` ports. | `src/**` | Blueprint wires and callbacks. | `NavCommandModule`, `GatewayModule` |
| System service | OS process supervised by systemd or an equivalent launcher. | `scripts/deploy/**`, robot host | DDS, HTTP, files, status JSON, hardware protocol. | `lt-slam.service`, `lt-maps.service`, `lt-nav.service` |
| Internal service class | Plain helper object used by a Module or route layer. | `src/**/services/**` | Python or C++ function calls only. | `ControlCommandService` |
| Adapter / bridge | Explicit protocol boundary to DDS, simulator, or hardware. | `src/**/adapters/**`, `sim/adapters/gazebo/**` | External protocol or compatibility API. | `CppSlamStatusAdapterModule`, camera DDS adapter |
| Native endpoint | C++ process that owns a native runtime boundary. | domain C++ tree, `scripts/deploy/**` | Typed DDS plus files/status. | `mapd`, `navd` |
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
| Module lifecycle, ports, tick loop, state machine | internal runtime | owning domain package | no |
| Plain validation, policy, planner request shaping | internal helper | domain `services` or `policy` package | no |
| Pure algorithm / hot path calculation | internal compute | owning native algorithm package, for example `nav/cpp/planning` | no transport imports |
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
| `runtime_contract` | What evidence/data-source label does this resolved runtime report? | `real`, `mujoco_fastlio2_live` |
| `endpoint_contract` | What endpoint protocol/schema is used at the process boundary? | `field_dds_v1` |
| `route_contract` | Which canonical topics are external bus topics, and which Module/endpoint ports bind to them? | `robot`, `replay`, `sim` |

Default product rule:

```text
Blueprint wires = internal Module graph
route_contract = external boundary graph
DDS = native endpoint/boundary transport
```

So a field run can have:

```text
endpoint_transport=dds
endpoint_contract=field_dds_v1
route_contract=robot
```

Module callbacks use local ports, while DDS is the typed
boundary for native Livox/SLAM/navigation endpoints.

## Current Product Navigation Shape

Field chain to the current base target, `/nav/cmd_vel`:

```text
nav-lidar-network.service
  -> lt-lidar.service
  -> lt-slam.service
  -> lt-maps.service + lt-terrain.service
  -> lt-nav.service
  -> /nav/cmd_vel
  -> lt-driver.service
  -> remote Brainstem gRPC
```

What each service owns:

| System service | Owns | Main input | Main output |
| --- | --- | --- | --- |
| `nav-lidar-network.service` | LiDAR NIC setup. | none | reachable Livox network |
| `lt-lidar.service` | Livox/IMU native DDS source. | hardware | `/lidar/raw_frame`, `/imu/raw` |
| `lt-slam.service` | Native SLAM/localization. | `/lidar/raw_frame`, `/imu/raw` | `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/tf` |
| `lt-maps.service` | Native `mapd`: realtime layers, saved-map lifecycle, SaveMap coordination, artifacts, and private UDS queries. | `/slam/map_observation`, `SlamMapSnapshotAck`, ProductControl activation request | `/maps/*`, `SlamMapSnapshotRequest`, saved-map products |
| `lt-terrain.service` | Near-field traversability. | odom, registered cloud | `/nav/traversability` |
| `lt-nav.service` | Current native nav endpoint. | odom, TF, goal, cloud, traversability, map artifact, driver control status | `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, `/nav/cmd_vel` when enabled |
| `lt-host.service` | Gateway/API/MCP/status/task entry. | user/task requests, status files | HTTP `5050`, MCP, goal/status commands |

Below `/nav/cmd_vel`:

| System service | Owns | Needed for current base? |
| --- | --- | --- |
| `lt-driver.service` | Native real robot command sink, remote Brainstem gRPC client, lease owner, `WalkChecked` ACK path. | yes for real motion |
| `robot-brainstem.service` | Legacy/local low-level control bridge if used by a non-field setup. | no for current `env=real` deployment |
| `can-setup.service` | CAN setup for legacy/local bridge deployments. | no for current remote Brainstem gRPC deployment |

The former Python DDS field unit, endpoint runner, installer, and deployment
wrapper are removed. Native DDS diagnostics use
`python -m diagnostics.field.dds_readiness`.

Current `lt-nav.service` still contains multiple internal parts:

```text
goal receiver
  -> OctoPlanner3D global planner
  -> global_path publisher
  -> target selector
  -> local::Planner
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
| `gateway` | API, MCP, status, teleop entry. | one process through `lt-host.service` |
| `localization` Modules/adapters | Normalize SLAM/localization state into LingTu. | no, unless wrapping a native endpoint |
| `src/maps/cpp/mapd` | Sole map-management owner: realtime layers, saved-map lifecycle, SaveMap, validation, artifacts, and bundle queries. Gateway uses only a stateless same-host UDS transport. | yes: `lt-maps.service`; ProductControl alone owns public map activation |
| `nav/commands`, `nav/services`, `nav/skills` | Host command admission and capability surface for native navigation. | no |
| `nav/cpp/endpoint` | Global/local planning, tracking, safety, authority, and typed DDS navigation output. | yes: `lt-nav.service` |
| `drivers/**` | Real and simulated robot adapters. | only hardware endpoints own OS process lifecycle |

## ROS State

Short answer: product navigation is ROS-free on the native DDS path; the
repository is not ROS-free.

ROS-free on the current product path:

- `lt-lidar.service`;
- `lt-slam.service`;
- `lt-maps.service`;
- `lt-terrain.service`;
- `lt-nav.service`;
- `lt-driver.service`;
- map artifact generation for OctoPlanner3D;
- `/nav/global_path`, `/nav/local_path`, `/nav/cmd_vel` typed DDS flow.

Non-Product sources remain outside the runtime boundary:

| Area | Examples | Keep? |
| --- | --- | --- |
| Removed legacy systemd seam | Former S100P ROS2 installer and unit templates | keep absent; legacy unit names remain detection tombstones only |
| External simulation gates | Non-Product simulator bridges under `sim/**` | keep as simulator tests, not product proof |
| Docs | historical plans and archived papers | archive stale claims when touched |

## Navigation rule

Keep `lt-nav` as one process unless a measured deployment or scheduling problem
requires another process boundary. Its global planner, local planner, follower,
and final safety components communicate through direct C++ calls. The Host
submits commands and reads status; it does not duplicate those components.
