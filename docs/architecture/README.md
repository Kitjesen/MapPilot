# LingTu Architecture Index

Status: current index
Audience: all contributors
Replaced by: not replaced

This directory contains current architecture contracts. Keep speculative plans
in `../plans/`, research in `../research/`, and retired decisions in git
history.

## Canonical Documents

| Document | Status | Scope |
| --- | --- | --- |
| [`SYSTEM_DESIGN.md`](./SYSTEM_DESIGN.md) | current | End-to-end system design, written as a paper-style overview. |
| [`FIELD_PRODUCTS.md`](./FIELD_PRODUCTS.md) | current | Field Product ownership, operator sessions, native processes, and readiness boundaries. |
| [`NAVIGATION_COMPUTE_CONTRACT.md`](./NAVIGATION_COMPUTE_CONTRACT.md) | current | Planning/local-planning/safety/control boundary. |
| [`NAVIGATION_CAPABILITY_MATRIX.md`](./NAVIGATION_CAPABILITY_MATRIX.md) | current evidence ledger | Map, route, waypoint, lifecycle, collision, smoothing, docking, following, and acceptance gaps. |
| [`NAVIGATION_RUNTIME_DATAFLOW.md`](./NAVIGATION_RUNTIME_DATAFLOW.md) | current | ROS-free/native-DDS navigation dataflow, `lingtu-nav-dds -> rt/nav/cmd_vel -> lingtu-driver -> Brainstem WalkChecked`, transports, payloads, OctoPlanner3D map inputs, and local-planner inputs. |
| [`MODULE_SERVICE_BOUNDARY.md`](./MODULE_SERVICE_BOUNDARY.md) | current cleanup guide | Names and boundaries for Modules, system services, internal service helpers, adapters, and remaining ROS surfaces. |
| [`ID_REGISTRY.md`](./ID_REGISTRY.md) | current guardrail | Cross-boundary ownership, generation, scope, lifecycle, retry, parent, exposure, and migration rules for canonical IDs. |
| [`GLOBAL_PLANNING_CONTRACT.md`](./GLOBAL_PLANNING_CONTRACT.md) | current | Global planner request/result, backend, preview, and transport boundary. |
| [`MAP_SERVICE_CONTRACT.md`](./MAP_SERVICE_CONTRACT.md) | current | Map-domain ownership, saved-map lifecycle, artifact capabilities, bundle lookup, and realtime-layer boundary. |
| [`local_planner_io_contract.md`](./local_planner_io_contract.md) | current | Local planner inputs, outputs, and remaining traversability gap. |
| [`LOCAL_PLANNING_AND_TRACKING_CONTRACT.md`](./LOCAL_PLANNING_AND_TRACKING_CONTRACT.md) | current | Local-planning algorithm, scoring, PathFollower boundary, and native-endpoint/Python-Module parameter surfaces; complements the I/O contract above. |
| [`LINGTU_RUNTIME_BUS_DECISION.md`](./LINGTU_RUNTIME_BUS_DECISION.md) | current | Port, channel, and transport policy. |
| [`blueprint_dds_integration.md`](./blueprint_dds_integration.md) | current | Blueprint–DDS integration: transport abstraction, QoS, worker deployment, observability, route contracts. |
| [`NATIVE_RUNTIME.md`](./NATIVE_RUNTIME.md) | current | Product-native C++ services, typed DDS endpoints, the `env=real` native deployment, and driver boundary. |
| [`NATIVE_DATA_PLANE_MIGRATION.md`](./NATIVE_DATA_PLANE_MIGRATION.md) | first batch complete | `NavigationState`, exact `MapObservation`, HostBus, field Product contracts, transitional limits, and the next `mapd` gates. |
| [`NATIVE_CONTROL_MODE_FUNCTIONS.md`](./NATIVE_CONTROL_MODE_FUNCTIONS.md) | current | Native endpoint control-mode ownership, teleop/avoid/autonomy gates, zero behavior, and acceptance responsibilities. |
| [`CAMERA_TRANSPORT_DECISION.md`](./CAMERA_TRANSPORT_DECISION.md) | accepted | Browser camera transport decision plus SHM/DDS robot-side camera data-plane note. |
| [`TOPIC_CONTRACT_POLICY.md`](./TOPIC_CONTRACT_POLICY.md) | current guardrail | Canonical runtime topic ownership, allowed literal layers, static guard, and LiDAR topic status. |
| [`ros_frame_contract.md`](./ros_frame_contract.md) | current | Frame naming and ROS compatibility constraints. |
| [`ROS_ROLE_REPLACEMENT_MAP.md`](./ROS_ROLE_REPLACEMENT_MAP.md) | current migration map | Which ROS roles remain adapters and which are native LingTu. |
| [`BUILDING_MISSION_RUNTIME.md`](./BUILDING_MISSION_RUNTIME.md) | current | Building/floor mission ownership and execution boundary. |
| [`semantic_layer_contract.md`](./semantic_layer_contract.md) | current | Semantic layer data ownership and map binding. |
| [`SEMANTIC_NAVIGATION_CONTRACT.md`](./SEMANTIC_NAVIGATION_CONTRACT.md) | current | Symbolic intent to map-bound navigation goal boundary. |
| [`SIMULATION_INTEGRATION_CONTRACT.md`](./SIMULATION_INTEGRATION_CONTRACT.md) | current | Simulator integration ownership, equivalence limits, and claim boundary. |

## Research And Plans

Architecture contains only current contracts and accepted decisions. Upstream
evaluations, algorithm comparisons, and migration coverage live in the
[research index](../research/README.md). The only active cross-domain work board
is [the current roadmap](../plans/current-roadmap.md). Retired plans are deleted
after their valid decisions enter a contract; use Git history for old context.

## Writing Rules

- Start with the contract, then list implementation paths.
- Name the module boundary before naming a backend.
- Use `Port -> Wire -> Transport` language for data flow.
- Do not present ROS 2 topics as the product API; put them in adapter tables.
- Mark plans as plans. Do not mix desired architecture with shipped behavior.
- Delete superseded plans once their replacement contracts contain the still
  valid decisions; use git history for retired detail instead of keeping a
  second, contradictory architecture tree.
