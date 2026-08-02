# LingTu Product Runtime Architecture

Status: current system design
Audience: all architecture, runtime, product, and field-readiness contributors
Replaced by: not replaced

## Abstract

LingTu is an autonomous navigation runtime for outdoor quadruped robots. A
`Product` declares an env-independent operating mode: logical native process
roles and the scoped Host graph. `ProductControl` is fixed to `env=real` or
`env=sim`, resolves that Product once into one immutable `RunPlan`, stages one
transient session, and applies the plan through its internal systemd runner.
The Host reads that exact RunPlan and uses Blueprint to materialize typed
Modules, explicit wires, and optional Python workers. Blueprint never owns
systemd or native-process lifecycle. The product data boundary is native
typed CycloneDDS. ROS 2, LCM, shared memory, and simulator bridges are
compatibility or optimization adapters rather than the application contract.

This document records the current architecture, the reason for the main
boundaries, and the validation surface that prevents navigation internals from
leaking into drivers, gateway, or UI code.

## 1. Problem Statement

The system must keep the same navigation product contract across three
environments:

1. endpoint or hardware operation when explicitly scheduled;
2. desktop or server simulation;
3. developer and UI integration workflows.

The hard part is not only path planning. The hard part is keeping hardware
drivers, SLAM, maps, global planning, local planning, safety, and UI telemetry
replaceable without turning every module into a ROS/DDS/topic-specific client.

## 2. Design Principles

| Principle | Meaning |
| --- | --- |
| One resolved decision | ProductControl resolves Product + env once into a RunPlan; its internal systemd runner and Host use that exact artifact instead of resolving configuration again. |
| Host graph | Blueprint constructs and wires typed Modules inside the Host process. It does not own Product switching, systemd, native endpoints, or DDS topology. |
| Product assembly | `lingtu.assembly` resolves env-independent Product data against one env without side effects. |
| RunPlan scope | The env maps logical Product roles to concrete targets; RunPlan is the fingerprinted Host/process/startup/check input shared by one run. |
| systemd scope | `lingtu.systemd.SystemdRunner` is an internal ProductControl implementation detail. It applies already-resolved processes and does not resolve Products or expose a second control plane. |
| Product control | `lingtu.control.ProductControl` owns switch, quiesce, restart, stop, rollback, session staging, and current-run commit inside its fixed env. |
| Contract before backend | Navigation depends on `GlobalPlanRequest` and `GlobalPlanResult`, not OctoPlanner3D internals. |
| Adapter isolation | Native DDS, ROS 2, LCM, simulators, and hardware SDKs stay at explicit adapter boundaries. Product native DDS is typed and schema-owned; ROS/LCM are opt-in compatibility only. |
| Safety as a first-class layer | Stop signals, geofence checks, local planner stops, and velocity mux arbitration remain outside planner internals. |
| Evidence over claims | Simulation, endpoint communication, and hardware readiness are separate validation claims. |

## 3. Runtime Model

```text
env + Product definition
  -> ProductControl.switch(Product)
       -> RunPlan
       -> internal SystemdRunner -> native/systemd processes
       -> Host -> Blueprint -> Gateway / Agent / MCP / adapters
```

`Profile` is a configuration-resolver input, not a runtime ownership class.
Local/development Profiles and operator-managed field Products must not be
treated as one architecture category merely because both pass through the same
resolver. `compile_run_plan()` produces a RunPlan without starting it. The
plan decides which managed processes run in that env and which Module graph
the Host materializes. ProductControl is the only Product operation API. Its
internal systemd runner and Host consume the same resolved decision and never
resolve a second one. `Wire` decides which output feeds
which input. `Transport`
decides how bytes or objects move between endpoints. Acceptance endpoints may
delegate lifecycle ownership to one bounded acceptance runner; they do not
publish ProductControl-managed process specs.

`RunPlan` is the immutable JSON execution input written by ProductControl. It
is not a service, scheduler, data bus, runtime state, or user-authored format.
It has only four top-level sections: `identity`, `launch`, `host`, and
`checks`. It exists because systemd and Host must execute the same resolved
Product. No point cloud, map, planner state, or algorithm runs inside it.

The mutable run files are deliberately separate and ephemeral:

```text
/run/lingtu/plan-<sha256>.json  immutable RunPlan
/run/lingtu/session.env        current process parameters
/run/lingtu/current.json       committed current-run pointer
```

systemd-tmpfiles recreates `/run/lingtu` at boot. A reboot therefore cannot
silently restore an old Product mode. Every mode-owned unit requires a valid
`session.env` and fails before `ExecStart` when the Product, env, session id,
RunPlan path, or fingerprint is absent or malformed.

The map Host camera path is explicit: `lingtu-camera-dds.service` owns the
device and publishes RGB-D frames through POSIX SHM, the private `camera`
adapter exposes typed Host ports, and `camera.color_image` feeds
`CameraJpegRelayModule.color_image`. The camera source is critical; the JPEG
relay remains a degradable fallback because WHEP/go2rtc is the primary video
path.

```text
Port:      Module declares In[T] / Out[T].
Wire:      Blueprint declares A.out -> B.in.
Transport: Local, native typed DDS, shared memory, replay, or explicit ROS/LCM compatibility adapter.
```

For Module-to-Module traffic this is not automatically DDS: local callbacks are
the default. For product cross-process field traffic, DDS is the typed endpoint
transport and must use LingTu IDL/message contracts rather than generic Python
payloads.

### 3.1 Actual Field Motion Path

ProductControl starts the Product; it does not route commands or
participate in the control loop. Blueprint only wires the Host-side Gateway,
Agent, MCP, and command/status adapters.

Names in architecture diagrams follow one rule: Product process roles use
lowercase identifiers (`slam`, `maps`, `traversability`, `nav`, `driver`,
`host`); C++ components inside one process use PascalCase. systemd unit and
binary names belong only in deployment tables.

The cross-process field path is:

```text
host           -- DDS command ----------> nav
slam           -- DDS odometry/cloud ---> nav
traversability -- DDS risk grid --------> nav
nav            -- DDS final velocity ---> driver -> Brainstem -> robot
```

`nav` is the only field motion role and the only `/nav/cmd_vel` writer.
`driver` is the only actuator role. `maps` owns live/persistent map data and
scene output; it does not replace the standalone `traversability` control-risk
producer.

The internal `nav` chain depends on the selected control path:

```text
autonomy goal: CommandIngress -> GlobalPlanner -> LocalPlanner -> PathFollower -> CommandSafety
external path: PathIngress ---------------------> LocalPlanner -> PathFollower -> CommandSafety
teleop_avoid:  OperatorIntent ------------------> LocalPlanner -> PathFollower -> CommandSafety
teleop:        OperatorCommand -----------------------------------------------> CommandSafety
```

These C++ components share one process. Only the process boundary uses DDS;
the arrows inside each control path are direct calls and in-memory values.

`/api/v1/runtime/dataflow` is a read-only Gateway observability endpoint kept
for the dashboard. It does not describe startup ownership and does not
orchestrate this motion path.

### 3.2 Supported Entry Points

```text
operator: scripts/lingtu -> ProductControl -> RunPlan -> systemd processes
service:  lingtu.py      -> verify exact RunPlan -> Blueprint.build() -> Host Modules
```

`scripts/lingtu` is the formal field-operation entry. `lingtu.py` remains the
Host-process entry used by the managed runtime service and by local development;
it is not a second field process orchestrator. In managed field operation it
builds the already selected Product instead of compiling another one.

The same entrypoint exposes read-only provenance without adding another
control plane:

```text
scripts/lingtu --env real inspect nav --explain
  -> resolved parameter values and Product/env/default source

scripts/lingtu status --explain
  -> committed RunPlan, transient session, actual process modes, and drop-ins
```

Both commands delegate to `lingtu.explain`; the shell does not parse or
reconstruct Product data.

### 3.3 Product Switch Transaction

There is one field mode-switch implementation:

```text
operator CLI or Gateway
  -> python -m lingtu.control switch
  -> ProductControl
       1. compile and validate the target Product once
       2. atomically publish the immutable RunPlan under /run/lingtu
       3. stop motion and the old session
       4. ask MapService to stage a typed map activation token when required
       5. atomically publish one /run/lingtu/session.env
       6. clear stale navigation and SLAM evidence
       7. apply exactly that RunPlan through the internal systemd runner
       8. require fresh native navigation, SLAM, map, and Gateway readiness
       9. commit the map token and /run/lingtu/current.json
```

Any failure after mutation quiesces conflicting Product processes, rolls back
the staged map token and session file, and removes the uncommitted RunPlan. It
does not enable or disable Product units. An incomplete switch is never
committed as the current run. Gateway starts the same `lingtu.control` module in an
independent systemd scope because replacing the Host would otherwise kill the
caller. `scripts/lingtu mode switch` is only a command-line adapter.

The base `lingtu.service` unit depends only on network readiness. Product
process ordering belongs to ProductControl and must not be duplicated with systemd
`Wants=` or `After=` relationships on the Host unit.

Legacy ROS/systemd unit names remain detection and maintenance inputs behind
`runtime.service_manager`, but the repository ships no S100P installer or boot
units. They are not Product operations and must not add `legacy_*` methods back
to ProductControl.

## 4. Layered System

```text
L0 Safety        SafetyRing, Geofence, CmdVelMux
L1 Hardware      real/sim drivers, camera, LiDAR, GNSS, SLAM/localization
L2 Maps          occupancy, voxel, ESDF, elevation, traversability, map manager
L3 Perception    detection, tracking, reconstruction, semantic mapping
L4 Decision      semantic planner, LLM, visual servo, goal resolution
L5 Planning      Navigation, global planner service, local planner, path follower
L6 Interface     Gateway, MCP, REST/SSE/WS, teleop, optional WebRTC/Rerun
```

Dependencies point downward or toward shared runtime contracts. `gateway/` must
not import `nav/`; `nav/` must not import drivers or gateway. Cross-layer data
flows through ports, runtime messages, and adapter contracts.

## 5. Navigation Method

Navigation is split into five responsibilities:

1. mission state and recovery;
2. global planning;
3. map and traversability safety checks;
4. local trajectory generation;
5. velocity arbitration.

The current global planner contract is defined in
`GLOBAL_PLANNING_CONTRACT.md`. Its public wire result is:

```text
GlobalPlanRequest:
  start: [x, y, z]
  goal: [x, y, z]
  safe_goal_tolerance
  frame_id
  request_id
  map_version

GlobalPlanResult:
  path: [[x, y, z], ...]
  plan_ms
  reached_goal
  adjusted_goal
  diagnostics
  report
```

OctoPlanner3D is the default map-backed algorithm backend. PCT/tomogram,
A*/direct, and ROS planner adapters are compatibility or diagnostic backends
only; they must not become the default field product path.

## 6. Mapping and Planning Artifacts

Saved maps are treated as product artifacts, not planner scratch files. The map
lifecycle is:

```text
sensor streams -> SLAM/map manager -> saved map artifacts
                -> planner map input -> global path
                -> local planner/path follower
```

Planning code consumes occupancy, voxel, or OctoPlanner3D-native artifacts
through the planner service boundary. Conversion and validation belong to map
lifecycle or planner setup, not every planning tick.

## 7. Transport Strategy

Default module traffic is in-process. Cross-process and cross-language paths
must declare transport explicitly.

| Transport | Current role | Constraint |
| --- | --- | --- |
| local port | default module graph | Python object boundary only. |
| DDS | product cross-process field boundary | Must use LingTu IDL/schema/version/frame/time; generic Python DDS is compatibility/diagnostics only. |
| LCM | replay/debug compatibility | Not a product data plane and not selected by field Products. |
| shared memory | high-volume same-host payloads, especially camera frames | Needs DDS/status metadata and explicit readiness gates. |
| ROS 2 | compatibility adapter | Not allowed as normal module business logic. |

## 8. UI and SDK Surface

The UI should consume stable JSON contracts from Gateway and runtime status, not
planner internals. For global planning, the UI-facing payload is
`lingtu.global_plan.v1`. It is produced by Navigation and preview flows and can
be mapped into Dart, Rust, or TypeScript without depending on Python classes.

## 9. Validation

The minimum validation surface is:

```bash
python -m pytest src/runtime/tests/ -q
python -m pytest src/nav/tests/ -q
python -m pytest src/gateway/tests/ -q
```

Focused checks for this architecture:

```bash
python -m pytest src/runtime/tests/test_stack_registry_resolution.py -q
python -m pytest src/runtime/tests/test_runtime_binding_policy.py -q
python -m pytest src/runtime/tests/test_nav_chain_efficiency.py -q
python -m pytest src/nav/tests/test_global_planner_contracts.py -q
```

Current validation is server/simulation/endpoint evidence unless a hardware
campaign is explicitly named. Simulation or endpoint communication evidence
does not equal real-hardware readiness. Hardware readiness requires
calibration, localization health, map artifact checks, safety ring, and command
mux validation on the selected target.

## 10. Current Limitations

- Some historical docs still describe ROS 2 as the primary runtime model.
- Some simulation evidence documents are valid as simulation evidence only.
- DDS product topics now use typed LingTu IDL; remaining generic DDS/LCM
  adapters are compatibility or diagnostics surfaces.
- UI contracts are being normalized; planner internals should not be exposed
  directly to frontend code.

## 11. Roadmap

1. Complete typed planner/map/status contracts for UI and SDK consumption.
2. Make transport selection visible in profile resolution.
3. Keep ROS 2 and LCM compatibility only at explicit adapters.
4. Keep camera/image bulk payloads on the SHM-backed native path with typed DDS
   metadata/status where practical.
5. Move stale design notes into `archive/` once a current replacement exists.
