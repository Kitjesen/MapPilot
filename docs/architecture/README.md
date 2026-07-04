# LingTu Architecture Index

This directory contains current architecture contracts. Keep speculative plans
in `../plans/` and old decisions in `../archive/`.

## Canonical Documents

| Document | Status | Scope |
| --- | --- | --- |
| [`SYSTEM_DESIGN.md`](./SYSTEM_DESIGN.md) | current | End-to-end system design, written as a paper-style overview. |
| [`NAVIGATION_COMPUTE_CONTRACT.md`](./NAVIGATION_COMPUTE_CONTRACT.md) | current | Planning/local-planning/safety/control boundary. |
| [`NAVIGATION_RUNTIME_DATAFLOW.md`](./NAVIGATION_RUNTIME_DATAFLOW.md) | current | ROS-free navigation-base dataflow, transports, payloads, OctoPlanner3D map inputs, and local-planner inputs. |
| [`GLOBAL_PLANNING_CONTRACT.md`](./GLOBAL_PLANNING_CONTRACT.md) | current | Global planner request/result, backend, preview, and transport boundary. |
| [`MAP_SERVICE_CONTRACT.md`](./MAP_SERVICE_CONTRACT.md) | current | Saved-map classes, artifact capabilities, bundle lookup, and builder gap. |
| [`local_planner_io_contract.md`](./local_planner_io_contract.md) | current | Local planner inputs, outputs, and remaining traversability gap. |
| [`LINGTU_RUNTIME_BUS_DECISION.md`](./LINGTU_RUNTIME_BUS_DECISION.md) | current | Port, channel, and transport policy. |
| [`ros_frame_contract.md`](./ros_frame_contract.md) | current | Frame naming and ROS compatibility constraints. |
| [`ROS_ROLE_REPLACEMENT_MAP.md`](./ROS_ROLE_REPLACEMENT_MAP.md) | current migration map | Which ROS roles remain adapters and which are native LingTu. |
| [`ROS2_DECOUPLING_MIGRATION_PLAN.md`](./ROS2_DECOUPLING_MIGRATION_PLAN.md) | proposal | Phased plan to remove the hard ROS 2/Ubuntu 22.04 runtime dependency from the C++ SLAM, base-autonomy, and sensor-ingestion layers. |
| [`DART_RUST_PACKAGE_MIGRATION.md`](./DART_RUST_PACKAGE_MIGRATION.md) | proposal | Future portable UI/package direction. |

## Writing Rules

- Start with the contract, then list implementation paths.
- Name the module boundary before naming a backend.
- Use `Port -> Wire -> Transport` language for data flow.
- Do not present ROS 2 topics as the product API; put them in adapter tables.
- Mark plans as plans. Do not mix desired architecture with shipped behavior.
