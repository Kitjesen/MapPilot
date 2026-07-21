# LingTu Architecture Index

Status: current index
Audience: all contributors
Replaced by: not replaced

This directory contains current architecture contracts. Keep speculative plans
in `../plans/` and old decisions in `../archive/`.

## Canonical Documents

| Document | Status | Scope |
| --- | --- | --- |
| [`SYSTEM_DESIGN.md`](./SYSTEM_DESIGN.md) | current | End-to-end system design, written as a paper-style overview. |
| [`PRODUCT_MODE_RUNTIME_CONTRACT.md`](./PRODUCT_MODE_RUNTIME_CONTRACT.md) | current | Product profiles, operator-facing product sessions, endpoint/session/SLAM mode naming. |
| [`NAVIGATION_COMPUTE_CONTRACT.md`](./NAVIGATION_COMPUTE_CONTRACT.md) | current | Planning/local-planning/safety/control boundary. |
| [`NAVIGATION_RUNTIME_DATAFLOW.md`](./NAVIGATION_RUNTIME_DATAFLOW.md) | current | ROS-free/native-DDS navigation dataflow, `lingtu-nav-dds -> rt/nav/cmd_vel -> lingtu-driver -> Brainstem WalkChecked`, transports, payloads, OctoPlanner3D map inputs, and local-planner inputs. |
| [`MODULE_SERVICE_BOUNDARY.md`](./MODULE_SERVICE_BOUNDARY.md) | current cleanup guide | Names and boundaries for Modules, system services, internal service helpers, adapters, and remaining ROS surfaces. |
| [`NAMING_AND_SERVICE_BOUNDARY_PLAN.md`](./NAMING_AND_SERVICE_BOUNDARY_PLAN.md) | cleanup plan | Keeps Blueprint/Module/Service/Agent/Kernel names, locks their responsibilities, and plans service-domain cleanup. |
| [`GLOBAL_PLANNING_CONTRACT.md`](./GLOBAL_PLANNING_CONTRACT.md) | current | Global planner request/result, backend, preview, and transport boundary. |
| [`MAP_SERVICE_CONTRACT.md`](./MAP_SERVICE_CONTRACT.md) | current | Map-domain ownership, saved-map lifecycle, artifact capabilities, bundle lookup, and realtime-layer boundary. |
| [`SOCC_ICP_ADOPTION.md`](./SOCC_ICP_ADOPTION.md) | native baseline implemented; field validation pending | Semantic occupancy, stable ABI/artifact, MapObservation flow, MapIcp integration, and product gates. |
| [`local_planner_io_contract.md`](./local_planner_io_contract.md) | current | Local planner inputs, outputs, and remaining traversability gap. |
| [`LOCAL_PLANNING_AND_TRACKING_CONTRACT.md`](./LOCAL_PLANNING_AND_TRACKING_CONTRACT.md) | current | Local-planning algorithm, scoring, PathFollower boundary, and native-endpoint/Python-Module parameter surfaces; complements the I/O contract above. |
| [`LINGTU_RUNTIME_BUS_DECISION.md`](./LINGTU_RUNTIME_BUS_DECISION.md) | current | Port, channel, and transport policy. |
| [`blueprint_dds_integration.md`](./blueprint_dds_integration.md) | current | Blueprint–DDS integration: transport abstraction, QoS, worker deployment, observability, route contracts. |
| [`NATIVE_RUNTIME.md`](./NATIVE_RUNTIME.md) | current | Product-native C++ services, typed DDS endpoints, `thunder_field` endpoint-only deployment, and driver boundary. |
| [`NATIVE_CONTROL_MODE_FUNCTIONS.md`](./NATIVE_CONTROL_MODE_FUNCTIONS.md) | current | Native endpoint control-mode ownership, teleop/avoid/autonomy gates, zero behavior, and acceptance responsibilities. |
| [`CAMERA_TRANSPORT_DECISION.md`](./CAMERA_TRANSPORT_DECISION.md) | accepted | Browser camera transport decision plus SHM/DDS robot-side camera data-plane note. |
| [`TOPIC_CONTRACT_POLICY.md`](./TOPIC_CONTRACT_POLICY.md) | current guardrail | Canonical runtime topic ownership, allowed literal layers, static guard, and LiDAR topic status. |
| [`ros_frame_contract.md`](./ros_frame_contract.md) | current | Frame naming and ROS compatibility constraints. |
| [`ROS_ROLE_REPLACEMENT_MAP.md`](./ROS_ROLE_REPLACEMENT_MAP.md) | current migration map | Which ROS roles remain adapters and which are native LingTu. |

## Historical / Proposal / Research Notes

These files are useful context, but are not the current product contract unless
their own status header says otherwise:

| Document | Status | Current replacement / guardrail |
| --- | --- | --- |
| [`DART_RUST_PACKAGE_MIGRATION.md`](./DART_RUST_PACKAGE_MIGRATION.md) | proposal | `SYSTEM_DESIGN.md`, `NATIVE_RUNTIME.md` |
| [`PORTABLE_LEAN_PACKAGE_MATRIX.md`](./PORTABLE_LEAN_PACKAGE_MATRIX.md) | packaging proposal | `PRODUCT_MODE_RUNTIME_CONTRACT.md`, profile binding-policy tests |
| [`THUNDER_RUNTIME_REFACTOR_PLAN.md`](./THUNDER_RUNTIME_REFACTOR_PLAN.md) | superseded plan | `NATIVE_RUNTIME.md`, `NAVIGATION_RUNTIME_DATAFLOW.md` |
| [`nav_plan_transport_boundary.md`](./nav_plan_transport_boundary.md) | historical/dev transport note | `NAVIGATION_RUNTIME_DATAFLOW.md`, `LOCAL_PLANNING_AND_TRACKING_CONTRACT.md` |
| [`perception_sota_evaluation.md`](./perception_sota_evaluation.md) | research snapshot | `semantic_layer_contract.md` for runtime contract |
| [`TRAVEXPLORER_LINGTU_ADOPTION.md`](./TRAVEXPLORER_LINGTU_ADOPTION.md) | research/adoption note | `PRODUCT_MODE_RUNTIME_CONTRACT.md` for field exploration chain |
| [`SOCC_ICP_ADOPTION.md`](./SOCC_ICP_ADOPTION.md) | adoption decision with gated field validation | `MAP_SERVICE_CONTRACT.md`, `NATIVE_RUNTIME.md` |
| [`POSE_GRAPH_OPT_GTSAM_COVERAGE.md`](./POSE_GRAPH_OPT_GTSAM_COVERAGE.md) | migration coverage note | `NATIVE_RUNTIME.md` product optimization policy |
| [`BUILDING_MISSION_RUNTIME.md`](./BUILDING_MISSION_RUNTIME.md) | building/facility extension contract | Native navigation endpoint rules still apply |

## Writing Rules

- Start with the contract, then list implementation paths.
- Name the module boundary before naming a backend.
- Use `Port -> Wire -> Transport` language for data flow.
- Do not present ROS 2 topics as the product API; put them in adapter tables.
- Mark plans as plans. Do not mix desired architecture with shipped behavior.
