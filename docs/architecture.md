# Architecture

**Status:** Current system boundary contract
**Audience:** Contributors, integrators, and architecture reviewers
**Runs on:** Repository-wide; applies to `env=real` and `env=sim`

This page defines system ownership, dependency direction, and vocabulary.
[`config/system/`](../config/system/README.md) is authoritative
for Products and environments; typed schemas are authoritative for wire data;
[`config/architecture_layers.yaml`](../config/architecture_layers.yaml)
enforces package direction.

The [Spot architecture comparison](research/spot.md) records public upstream
contracts and a staged LingTu task-interface proposal. It does not replace the
current ownership rules on this page or claim new deployed capabilities.
The [task interface proposal](research/tasks.md) defines the proposed shared
semantics above existing domain executors. The first Follow lifecycle is now
implemented and locally tested; native simulation and field qualification remain
open. It does not change this page's dependency rules.
The [camera-to-motion design](design/autonomy.md) traces observation ownership,
semantic reasoning, domain tasks, and native execution through current code. It
records the implemented shared observation input to Follow and the remaining
Agent/memory migration plan; local contract tests are not evidence of field readiness.
For the current instruction/Agent split, skill ownership, model call paths and
code-reading order, use the [Decision code guide](../src/decision/README.md).
The task proposal's final section records the verified gaps and incremental
cleanup order; it does not describe an already completed runtime refactor.

## Fixed vocabulary

| Term | Exact meaning |
| --- | --- |
| `env` | Outer runtime environment; public values are exactly `real` and `sim` |
| Product | Immutable, env-independent operating-mode declaration |
| RunPlan | Internal immutable execution snapshot produced for ProductControl; not a public lifecycle object |
| ProductControl | Only public owner of a complete Product lifecycle transaction in one fixed env |
| Host | Managed Python process containing Gateway, Agent, adapters, and selected Modules |
| Module | Typed in-process Host runtime unit |
| DDS | Native typed cross-process data plane |
| RobotConfig | Static physical robot, device, and calibration data used by `real` |
| Endpoint | Concrete HTTP, DDS, or native-service access point |

Host module assembly is internal. DDS does not own Product policy.
RobotConfig does not select a runtime mode. An Endpoint is not an env.

## System model

```text
ProductControl request (RobotConfig + Product + env(real | sim))
                 -> internal execution snapshot (RunPlan)
                 -> real: SystemdRunner
                 -> sim: direct-child runner

Host
  -> typed Modules, Ports, and Wires

native processes
  <-> typed DDS
```

ProductControl resolves a Product once. CLI, Gateway, runners, and Host consume
the resulting private execution snapshot. `product_session_id` identifies the run; snapshot paths,
native launch IDs, PIDs, and boot IDs stay private to their runtime boundary.

## Control plane

The public control path is request -> ProductControl -> execution:

1. Product YAML declares capabilities, logical roles, topics, and Host modules.
2. Environment YAML maps roles to implementations, process control, and transport.
3. ProductControl resolves the selected declarations into one private immutable
   execution snapshot without starting processes.
4. `lingtu.control` exposes `switch`, `status`, and `stop`; the selected runner
   executes that snapshot.

ProductControl owns the mutation lock, staged state, readiness, rollback, and
current-run record. `scripts/lingtu` is a thin adapter to
`python -m lingtu.control`.

`lingtu/assembly/graph/` owns declaration loading, variant resolution, and
process contracts. Generic `runtime/` does not import Product assembly.
Real and simulation share request/report semantics, while each owns its process
operations, identity evidence, and rollback.

## Host and Module model

[`src/runtime/host.py`](../src/runtime/host.py) contains Host's internal module
construction and lifecycle implementation. `HostBuilder` is an implementation
helper, not another operating mode or scheduler. Product callers use
ProductControl; Host startup uses the resolved configuration.
[`src/runtime/registry.py`](../src/runtime/registry.py) is the normal
backend and capability registration boundary.

The Module lifecycle is `setup -> start -> stop`. Typed Ports and
Wires describe in-process relationships; transports begin only at an explicit
process, hardware, simulator, or external API seam.

Gateway, Agent, and MCP submit intent and project facts. They do not own Product
lifecycle, map state, planning, or hardware. `gateway/navigation/` owns HTTP
navigation commands and state projection; `gateway/maps/` owns saved-map HTTP
operations and mapd transport. Native owners still execute those operations,
and ProductControl alone activates a map. See the
[Gateway owner map](../src/gateway/README.md) and
[Host lifecycle](../src/runtime/README.md).

## Process and data plane

Use direct calls inside one C++ service. Use typed CycloneDDS between native
Product processes. Use shared memory for declared high-volume same-host
payloads such as camera frames.

ROS 2, LCM, and Gazebo belong only to explicit compatibility, replay, or
diagnostic adapters. Product runtime source is ROS-free. A compatibility
surface cannot silently become the Product data plane.

Scene and Console prefer the configured WHEP endpoint and retain Gateway
JPEG-over-WebSocket as the fallback. Gateway proxies SDP negotiation and session
deletion, never the video payload. Raw color SHM has one native camera owner;
`lingtu_camera_webrtc` consumes it without opening another camera device.
Native `whepserversink` uses GStreamer 1.28 and gst-plugin-webrtc 0.15; the
GStreamer 1.24 reference publishes fixed-rate H.264 through go2rtc WHEP.
go2rtc is an optional machine-level external media sidecar outside ProductControl;
it is not part of any Product, RunPlan, or Product readiness gate. If WHEP is
unavailable, each view falls directly back to JPEG. A connected peer is not
treated as a live camera until the browser decodes a frame. Source changes and
view closure release the old peer; missing initial frames or frozen video trigger
reconnection. Display rotation comes from RobotConfig.
The native media executable is a foreground development path until its actual
board encoder and readiness are qualified for ProductControl ownership. The
[remote-video research](research/video.md) compares shared-source direct WebRTC,
TURN fallback and optional SFU distribution. Public-network traversal, SFU
distribution and board performance remain separate qualification work.

## Dependency direction

```text
All Modules -> runtime foundations, contracts, messages, registry, utilities

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, decision/, drivers/, gateway/
decision/   consumes perception through runtime messages
drivers/    must not import nav/ or semantic code except lazy registration
gateway/    must not import nav/, semantic/, or drivers/
```

Waypoint and path dispatch are message flow, not package dependencies. Reuse
existing factories, registries, and runtime messages before importing a
concrete backend across an owner boundary.

## Functional ownership

| Source owner | Responsibility |
| --- | --- |
| `src/runtime/` | Host Module framework, internal assembly, shared contracts, messages, and registry |
| `src/lingtu/` | ProductControl lifecycle, internal resolution, and real/sim process control |
| `src/nav/` | Navigation command surface and native planning, tracking, safety, and control |
| `src/maps/` | Live map services, map packages, artifacts, and saved-map lifecycle |
| `src/localization/` | SLAM, localization, relocalization, and normalized status |
| `src/drivers/` | Physical and simulation device I/O |
| `src/perception/` | Detection, tracking, reconstruction, and scene products |
| `src/decision/` | Goal resolution, semantic planning, and visual-servo intent |
| `src/memory/` | Semantic, episodic, spatial, and temporal memory |
| `src/gateway/` | REST, SSE, WebSocket, MCP, validation, and status projection |

Semantic memory is a read/query layer around native map identity. It may keep
objects, places, relations, and observation history, but it does not duplicate
occupancy, collision geometry, elevation, traversability, or saved-map
lifecycle. The design and current implementation audit are in
[Persistent semantic mapping](research/semantic-map.md).

Hot-path native code stays with the functional owner under its `cpp/` tree.
The package structure is described in [`src/README.md`](../src/README.md).

### Localization layout

Localization has one production numerical core and several deliberately
bounded collaborators. The directory names describe ownership and lifetime;
they are not competing estimators:

| Path | Boundary | Lifetime | Responsibility |
| --- | --- | --- | --- |
| `src/localization/ltlio/` | library | per `slamd` process | LTLIO scan/IMU estimation and transport-mode state |
| `src/localization/slam/cpp/` | native service | resident field process | DDS, lifecycle, status, map/odom, save/load and relocalization wiring |
| `src/localization/online_mapping/` | library/worker | bounded online mapping worker | keyframe graph, loop factors and corrected map snapshots |
| `src/localization/map_optimization/` | executable/library | save-time job | PGO, loop verification, map-bundle optimization and shared point-cloud/pose file I/O |
| `src/localization/relocalization/` | library/CLI | request-scoped | global/seeded registration and ICP refinement |

The `online_mapping` directory owns both the online scheduler and its backend;
the file reader used by replay and offline optimization is built separately.
`relocalization` is the **relocalization library**.
The upstream `sam` spelling remains only where it identifies the pinned
LIO-SAM-derived implementation or its evidence format.

The production call path is `drivers -> slam/cpp -> ltlio`; `online_mapping` may consume
LTLIO snapshots for online map correction, while `relocalization` and `map_optimization` are
explicit map workflows. Neither `online_mapping` nor `map_optimization` replaces the LTLIO frontend,
and neither owns DDS or Product lifecycle. `ContractBackend` in
`slam/cpp/slam.cpp` is a test seam; it is not a field estimator.

Use [`src/localization/README.md`](../src/localization/README.md) for the
directory-level contract and reading order. Do not add a second `slam2`,
`elevator_lio`, or generic `localization_core` tree to make a new feature fit;
extend the owning boundary and its existing interface.

## Navigation, map, and command ownership

High-level intent moves through typed navigation commands. A goal is not a
motor command.

```text
intent -> goal lifecycle -> global plan -> local plan -> tracking
       -> native final safety -> rt/nav/cmd_vel -> lingtu-driver
```

`navd` owns final navigation arbitration. Standalone traversability owns the
control-risk grid. `mapd` owns live map state and persistent map operations.
Only `lingtu-driver` may forward a checked command to the RobotConfig-selected
hardware adapter.

Python map or planning code may support development and simulation; it cannot
compete with field owners.

### Global and local planning

Product selects planners and declares their configuration. ProductControl
resolves those inputs once and starts the resulting runtime; its internal
execution snapshot preserves the selected values. Neither layer
implements search, selects a local target, or edits a generated trajectory.

| Owner | Inputs and responsibility | Output |
| --- | --- | --- |
| Global planner | Saved 3D map, current map pose, final goal, global search settings | Ordered map-frame XYZ route points and route-search result |
| Goal lifecycle | Request identity, current map identity, global-search completion and persistent execution blockage | Activate/replace/cancel a Route or request another global search |
| Executor | Activated Route, measured pose/velocity, execution state | Local reference/objective and execution feedback |
| Local planner | Reference, measured state, live collision/prediction data and its own settings | Executable local path or B-spline, or typed pending/blocked result |
| Follower and final control | Executable local result, measured state, speed limits and motion authority | Checked body velocity for the driver |

The global-to-local contract is one identified map-frame reference polyline,
not a sequence of goals. LocalPlanner selects its local target, trajectory, and detours;
pose updates do not create routes. Persistent blockage can request one bounded
replacement through the goal lifecycle. Global search does not inspect LocalPlanner's
private failures, and local planning does not rewrite the saved map.

`LocalPlanner` names the module. SCAN is its default algorithm; CMU remains an
explicit alternative. The execution layer depends on the module interface.

See [runtime conditions and parameters](runtime.md#route-trajectory-and-motion-conditions)
and the [native navigation interfaces](../src/nav/cpp/README.md).

## Motion authority

Native `navd` owns motion authority. Web, SDK, and CLI submit typed requests;
they do not publish final robot velocity. Its operator source lease admits one
external source, while its control state selects autonomous or operator motion.
FinalControl applies shaping and safety to that source's candidate command.
Only `lingtu-driver` forwards the resulting command through the selected robot
adapter.

Claim reserves an operator source; it does not interrupt a task by itself.
Deliberate operator motion pauses the ordinary single-goal task and retains its
destination and task identity. After operator input is active, holding sends
zero but retains the source lease. Releasing or expiring that lease leaves
motion held. Closing an unused Claim by Hold, Release, or expiry releases only
source permission; it does not publish velocity zero or interrupt autonomy.
TaskResume
is the explicit request to release that hold and continue the same task; the
generic control-resume request only releases the hold and does not continue a
paused task. Estop is latched and clearing it does not replay old commands.

The default operator sample age limit is 0.35 s; the maximum source lease is
2 s. Valid incoming samples renew the source lease. The web session requests
1 s and sends held input every 20 ms, so these limits do not cap the length of
a teleoperation session. Lease expiry and command freshness are different facts.

Once a task is paused, its coordinator does not publish steady velocity zeros.
The endpoint owns the current operator output and its no-output zero fallback;
stop confirmation still owns zero output while the handoff is pending. Explicit
task continuation resets local tracking and reprojects the retained 3D route
from the measured pose, preserving its goal and execution settings.

Control source, task state, and stop evidence are separate facts. A safety veto
can send zero while the source still owns control. Operator zero publication
does not prove physical rest. The autonomous stop transaction also requires
driver acknowledgement and measured stationary odometry; teleop uses its
declared driver-acknowledgement policy. Stop confirmation advances on ordinary
endpoint ticks: pending evidence keeps output zero while commands and sensors
continue to be processed. A pause request acknowledgement is not a confirmed
PAUSED event.

## Directory names

| Path | Responsibility |
| --- | --- |
| `config/system/` | Product and real/sim declarations used by ProductControl |
| `src/runtime/` | Shared Python Host Module framework and internal assembly |
| `src/nav/cpp/endpoint/nav/app/` | navd composition, control loop, and goal/task coordination |

RunPlan is the private resolved configuration; HostBuilder is Host's internal
construction helper. Neither is an additional task scheduler or user-facing
operating mode. DDS/platform runtime files keep their transport-specific meaning.

## Topics, frames, schemas, and IDs

Topic declarations live in [`src/message/topics/`](../src/message/topics/);
Python uses the generated `message.topics.TOPICS` view. Frame names and transform
rules live in [`src/runtime/tf/frames.py`](../src/runtime/tf/frames.py).
Native cross-process truth comes from the owning IDL/schema and runtime graph.
Wire integer values belong to `src/message/idl/constants.idl`; Python and C++
enums are generated. Robot mounting values belong only to RobotConfig.
`diagnostics/runtime_contract.py` projects diagnostics; it does not redefine
these sources. Compatibility topic aliases live in `runtime/adapters/topics.py`.
`message/catalog.py` is the shared catalogue reader for generation and wiring;
reading a communication contract does not load Product/Env configuration.

Topic literals belong only in the canonical catalog, generated binding code,
or explicit compatibility mappings. Do not duplicate them in business logic.

Saved maps and global paths use the `map` frame. Local sensing and control use
the declared `body` or `odom` contract. `base_link == body` is a compatibility
alias; ROS TF is not the Product data plane.

Native SLAM resolves map/body/scan poses in `src/localization/slam/cpp/frames.hpp`.
Its status export supplies Host with `map_odometry` (map pose, body twist) and
`map_scan_pose` (scan pose, physical ray origin). Host adapters and the browser
consume these values without composing independently received TF and odometry.
Raw odometry and TF remain native algorithm inputs and diagnostic data. The
Gateway frame tree is a diagnostic mirror, not another localization authority.

Use identifiers by meaning, not by string shape. `product_session_id`,
`request_id`, `task_id`, `operation_id`, map identity, and simulation
`session_id` have different owners and lifetimes.

## Simulation boundary

Root `sim/` is a simulation workspace and Python namespace. `env=sim` is an
outer Product runtime value. The workspace can contain offline tools or
component tests that do not constitute a Product run.

MuJoCo owns simulation clock and physical truth. RobotSimUE owns presentation
and render sensors. Adapters translate contracts and cannot become a second
clock, pose, or configuration authority. See [Simulation](./simulation.md).

## Repository and install layout

`src/` owns source, `sim/` owns simulation composition, and
`config/system/envs/{real,sim}.yaml` owns environment mapping.

| Path | Meaning |
| --- | --- |
| `src/` | Functional-owner source tree |
| `sim/` | Simulation workspace; not a third public env |
| `config/system/envs/` | `real` and `sim` implementation mappings |
| `build/`, `install/`, `dist/` | Compile, install-stage, and release outputs |

Generated artifacts move through:

```text
build/ -> install/<platform>-<arch>/<config>/ -> dist/
```

The install prefix contains `bin/`, `lib/`, `etc/lingtu/`, and
`share/lingtu/`. The production root is
`/opt/lingtu/current/{bin,lib,etc/lingtu,share/lingtu}`.

Do not create a tracked root `bin/` or root `real/` tree. Repository placement
details live in [`src/README.md`](../src/README.md),
[`sim/README.md`](../sim/README.md), and
[`scripts/README.md`](../scripts/README.md).

## Sources of truth

| Subject | Authority |
| --- | --- |
| Product and env | `config/system/` |
| Product control | `src/lingtu/` |
| Host graph | `src/runtime/` |
| Robot, device, calibration | `config/robots/` and `config/devices.yaml` |
| Native algorithms and services | Their owning `src/<owner>/cpp/` tree |
| Simulation | `sim/` |
| REST and MCP | `src/gateway/{navigation,maps,routes}/` and Module `@skill` methods |
| Topics, payloads, wire enums | `src/message/topics/`, `src/message/idl/` |
| Frames and physical calibration | `src/runtime/tf/frames.py`, RobotConfig |

Current operational behavior is detailed in [Runtime](./runtime.md). Evidence
and open work belong in [Testing](./testing.md) and [Roadmap](./roadmap.md),
not in this contract.
