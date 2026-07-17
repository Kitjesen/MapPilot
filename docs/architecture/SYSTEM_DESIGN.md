# LingTu: Module-First Autonomous Navigation Runtime

## Abstract

LingTu is an autonomous navigation runtime for outdoor quadruped robots. Its
central design decision is to make `Module` the only runtime unit and
`Blueprint` the only orchestration unit. Perception, mapping, planning, safety,
and gateway functions communicate through typed ports and explicit wires. ROS 2,
DDS, LCM, shared memory, and simulator bridges are treated as transports or
compatibility adapters rather than the application contract.

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
| Module-First | Runtime behavior lives in `Module` classes with typed `In[T]` and `Out[T]` ports. |
| Blueprint-only orchestration | Profiles assemble modules and explicit wires; modules do not discover the system graph. |
| Contract before backend | Navigation depends on `GlobalPlanRequest` and `GlobalPlanResult`, not OctoPlanner3D internals. |
| Adapter isolation | ROS 2, DDS, LCM, simulators, and hardware SDKs stay at explicit adapter boundaries. |
| Safety as a first-class layer | Stop signals, geofence checks, local planner stops, and velocity mux arbitration remain outside planner internals. |
| Evidence over claims | Simulation, endpoint communication, and hardware readiness are separate validation claims. |

## 3. Runtime Model

```text
Profile
  -> resolved runtime spec
  -> Blueprint
  -> Module graph
  -> ports and wires
  -> selected transports
```

`Profile` decides product intent. `Blueprint` decides module composition.
`Wire` decides which output feeds which input. `Transport` decides how bytes or
objects move between endpoints.

```text
Port:      Module declares In[T] / Out[T].
Wire:      Blueprint declares A.out -> B.in.
Transport: Local, DDS, LCM, shared memory, ROS adapter, replay, or future binary schema.
```

This is not DDS. DDS can be one transport behind a wire. The product contract is
the module message and channel contract.

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

OctoPlanner3D is the default map-backed algorithm backend. PCT/tomogram is
retired from the product runtime. Direct path planning is used only for
explicit mapless modes.

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
| DDS | selected typed boundaries | Must use schema/version/frame/time, not long-term pickle payloads. |
| LCM | endpoint/field bridge | Useful at robot boundary; not the universal internal bus. |
| shared memory | high-volume candidate | Needs stable point-cloud/image schemas and performance gates. |
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
- DDS and shared-memory transports need stable cross-language schemas before
  they become long-term product interfaces.
- UI contracts are being normalized; planner internals should not be exposed
  directly to frontend code.

## 11. Roadmap

1. Complete typed planner/map/status contracts for UI and SDK consumption.
2. Make transport selection visible in profile resolution.
3. Replace pickle-like DDS payloads with schema-versioned typed messages.
4. Keep ROS 2 compatibility only at explicit adapters.
5. Move stale design notes into `archive/` once a current replacement exists.
