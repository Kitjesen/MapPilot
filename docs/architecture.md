# Architecture

**Status:** Current system boundary contract
**Audience:** Contributors, integrators, and architecture reviewers
**Runs on:** Repository-wide; applies to `env=real` and `env=sim`

This page defines ownership, dependency direction, and the system vocabulary.
Running configuration and typed schemas outrank prose when they conflict.

Product and environment truth lives in
[`config/runtime_graph/`](../config/runtime_graph/README.md). Machine-enforced
layer rules live in
[`config/architecture_layers.yaml`](../config/architecture_layers.yaml).

## Fixed vocabulary

| Term | Exact meaning |
| --- | --- |
| `env` | Outer runtime environment; public values are exactly `real` and `sim` |
| Product | Immutable, env-independent operating-mode declaration |
| RunPlan | Immutable execution artifact produced by resolving one Product in one env |
| ProductControl | Only public owner of a complete Product lifecycle transaction in one fixed env |
| Host | Managed Python process containing Gateway, Agent, adapters, and selected Modules |
| Blueprint | Construction and wiring of Modules inside one Host |
| Module | Typed in-process Host runtime unit |
| DDS | Native typed cross-process data plane |
| RobotConfig | Static physical robot, device, and calibration data used by `real` |
| Endpoint | Concrete HTTP, DDS, or native-service access point |

Blueprint is not a process orchestrator. DDS does not own Product policy.
RobotConfig does not select a runtime mode. An Endpoint is not an env.

## System model

```text
RobotConfig + Product + env(real | sim)
                 -> RunPlan
                 -> ProductControl
                 -> real: SystemdRunner
                 -> sim: direct-child runner

Host
  -> Blueprint
  -> typed Modules, Ports, and Wires

native processes
  <-> typed DDS
```

Assembly resolves a Product once. CLI, Gateway, systemd, runners, and Host
consume the published RunPlan; they do not independently resolve it again.

A Product run is identified by `product_session_id`. RunPlan paths, native
launch IDs, PIDs, and protocol boot IDs stay private to their runtime boundary.

## Control plane

The control path has one direction:

1. `config/runtime_graph/products/*.yaml` declares capabilities, logical roles,
   topics, and the Host Blueprint.
2. `config/runtime_graph/envs/{real,sim}.yaml` maps roles to implementations,
   process control, and transports.
3. `src/lingtu/assembly/` resolves without side effects.
4. `src/lingtu/run_plan.py` owns the immutable RunPlan type.
5. `src/lingtu/control.py` exposes `switch`, `status`, and `stop`.
6. The selected runner executes only the resolved plan.

ProductControl owns the mutation lock, staged state, readiness, rollback, and
current-run record. `scripts/lingtu` is a thin adapter to
`python -m lingtu.control`.

`lingtu/assembly/graph/` owns declaration loading, variant resolution, and
process contracts. The compiler passes its selected Product onward; downstream
code validates that value instead of selecting a variant again. The Host's
generic `runtime/` does not import Product assembly or diagnostic projections.

Real and simulation switches share request/report semantics. They do not share
rollback implementation, process operations, runtime identity, or a generic
phase engine.

## Host and Module model

[`src/runtime/blueprint.py`](../src/runtime/blueprint.py) constructs one Host
graph. [`src/runtime/registry.py`](../src/runtime/registry.py) is the normal
backend and capability registration boundary.

The Module lifecycle is `preflight -> setup -> start -> stop`. Typed Ports and
Wires describe in-process data relationships. A transport is selected only at
an explicit process, hardware, simulator, or external API seam.

Gateway, Agent, and MCP are Host surfaces. They submit intent and project
facts; they do not become a native planner, map state machine, driver, or
Product lifecycle owner. Invocation is not a continuous data wire.

`gateway/navigation/` owns the navigation-facing HTTP commands, exact task
queries, admission evaluation, and state projection. HTTP, SSE, readiness, and
the full state snapshot share its evaluated navigation facts. Shared request
deduplication and control transport remain under `gateway/services/`; neither
is another navigation task owner.

`gateway/maps/` groups saved-map HTTP operations, mapd transport, environment
layer projection, tagged locations, and semantic places. The last two retain
their distinct existing data models. Native mapd owns map state and save jobs;
ProductControl alone switches the active map. Gateway adds no map manager.

`gateway/routes/status.py` exposes only aggregate state, scene, and localization.
Health/readiness/metrics live in `routes/health.py`, runtime dataflow inspection
in `routes/diagnostics.py`, and SSE/WebSocket delivery in `routes/realtime.py`.
The public URLs and response contracts do not depend on this code layout.
[`src/gateway/README.md`](../src/gateway/README.md) maps user-facing questions to
their API and source owner.

Detailed lifecycle and delivery rules live in
[`src/runtime/README.md`](../src/runtime/README.md).

## Process and data plane

Use direct calls inside one C++ service. Use typed CycloneDDS between native
Product processes. Use shared memory for declared high-volume same-host
payloads such as camera frames.

ROS 2, LCM, and Gazebo belong only to explicit compatibility, replay, or
diagnostic adapters. Product runtime source is ROS-free. A compatibility
surface cannot silently become the Product data plane.

The canonical camera Web path is go2rtc WHEP, with Gateway
JPEG-over-WebSocket as the fallback. go2rtc is an optional machine-level
external media sidecar outside ProductControl. It is not part of any Product,
RunPlan, or Product readiness gate; if unavailable, the client falls directly
back to JPEG.

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
| `src/runtime/` | Module framework, Blueprint, shared contracts, messages, registry, and graph utilities |
| `src/lingtu/` | Product resolution, RunPlan, lifecycle routing, and real/sim process control |
| `src/nav/` | Navigation command surface and native planning, tracking, safety, and control |
| `src/maps/` | Live map services, map packages, artifacts, and saved-map lifecycle |
| `src/localization/` | SLAM, localization, relocalization, and normalized status |
| `src/drivers/` | Physical and simulation device I/O |
| `src/perception/` | Detection, tracking, reconstruction, and scene products |
| `src/decision/` | Goal resolution, semantic planning, and visual-servo intent |
| `src/memory/` | Semantic, episodic, spatial, and temporal memory |
| `src/gateway/` | REST, SSE, WebSocket, MCP, validation, and status projection |

Hot-path native code stays with the functional owner under its `cpp/` tree.
The package structure is described in [`src/README.md`](../src/README.md).

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

Python map or planning code may exist for development and simulation fallback
work, but it cannot compete with field owners.

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
`config/runtime_graph/envs/{real,sim}.yaml` owns environment mapping.

| Path | Meaning |
| --- | --- |
| `src/` | Functional-owner source tree |
| `sim/` | Simulation workspace; not a third public env |
| `config/runtime_graph/envs/` | `real` and `sim` implementation mappings |
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
| Product and env | `config/runtime_graph/` |
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
