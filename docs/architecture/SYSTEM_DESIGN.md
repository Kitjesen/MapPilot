# LingTu Product Runtime Architecture

Status: current system design
Audience: all architecture, runtime, product, and field-readiness contributors
Replaced by: not replaced

## Abstract

LingTu is an autonomous navigation runtime for outdoor quadruped robots. It has
two deliberately separate orchestration scopes. Inside one Python application
runtime, `Module` is the runtime unit and `Blueprint` materializes typed ports
and explicit wires. The default graph is in-process; optional Python workers
remain owned by the same Blueprint and are not product services. At the host
process boundary, Runtime Graph Product and Endpoint
contracts compile one `Product` containing its Blueprint and optional
`RuntimePlan`; the external `Launcher` applies that process plan. The product
data boundary is native typed CycloneDDS. ROS 2, LCM, shared
memory, and simulator bridges are compatibility or optimization adapters rather
than the application contract.

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
| Module-First, in process | Python runtime behavior lives in `Module` classes with typed `In[T]` and `Out[T]` ports. |
| Blueprint mechanism | `runtime.blueprint` materializes one application-owned Module graph. It may use internal Python workers, but it does not own native services. |
| Product assembly | `lingtu.assembly` chooses LingTu products, stacks, adapters, and wires, then produces a Blueprint. |
| RuntimePlan scope | Product declares logical process roles; Endpoint maps them to deployment targets; RuntimePlan orders startup, shutdown, timeout, and lifecycle ownership. |
| Launcher scope | `lingtu.launcher` is the only process-plan executor. It applies systemd order, mandatory readiness, fail-closed rollback, and guarded single-process restart. Full-plan apply runs outside the managed application service. |
| Product control | `lingtu.control.ProductControl` recompiles the active Profile/Endpoint and delegates process restart to Launcher; it never invents a unit name or readiness rule. |
| Contract before backend | Navigation depends on `GlobalPlanRequest` and `GlobalPlanResult`, not OctoPlanner3D internals. |
| Adapter isolation | Native DDS, ROS 2, LCM, simulators, and hardware SDKs stay at explicit adapter boundaries. Product native DDS is typed and schema-owned; ROS/LCM are opt-in compatibility only. |
| Safety as a first-class layer | Stop signals, geofence checks, local planner stops, and velocity mux arbitration remain outside planner internals. |
| Evidence over claims | Simulation, endpoint communication, and hardware readiness are separate validation claims. |

## 3. Runtime Model

```text
Profile
  -> resolved config + Endpoint
  -> compile_product()
       -> Product
            -> Blueprint -> Module graph -> ports and wires
            -> RuntimePlan -> Launcher -> native processes -> typed DDS
                              ^
                              ProductControl (status / guarded restart)
```

`Profile` selects product intent and endpoint. `compile_product()` is the only
compiler and produces both orchestration scopes without starting either one.
`RuntimePlan` decides which managed native processes run on that endpoint;
`Launcher` executes it. `ProductControl` is the operational API for a running
product and may restart only a process present in that compiled plan.
`lingtu.assembly` decides product composition.
`Blueprint` materializes that application Module graph.
`Wire` decides which output feeds which input. `Transport`
decides how bytes or objects move between endpoints. Acceptance endpoints may
delegate lifecycle ownership to one bounded acceptance runner; they must not
publish a partial RuntimePlan.

```text
Port:      Module declares In[T] / Out[T].
Wire:      Blueprint declares A.out -> B.in.
Transport: Local, native typed DDS, shared memory, replay, or explicit ROS/LCM compatibility adapter.
```

For Module-to-Module traffic this is not automatically DDS: local callbacks are
the default. For product cross-process field traffic, DDS is the typed endpoint
transport and must use LingTu IDL/message contracts rather than generic Python
payloads.

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
| LCM | replay/debug compatibility | Not a product data plane and not selected by field product profiles. |
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
