# LingTu Runtime Bus Decision

Status: accepted architecture memory
Date: 2026-06-18

## Decision

LingTu should not replace ROS 2 by copying the ROS topic model into business
code. The project will own a small Runtime Bus contract:

```text
Capability Module
  -> typed Module ports
  -> LingTu channel contract
  -> local / lcm / shm / ros2 / replay transport adapters
```

Normal modules speak LingTu channels and `runtime.msgs` types. ROS 2 topics, LCM
channels, simulator feeds, replay files, and future shared-memory paths are
adapter aliases for those channels.

## Communication Plan And Status

Current target: server/simulation/endpoint communication. Do not read this as
real-hardware validation.

| Boundary | Planned transport | Done now | Not done |
| --- | --- | --- | --- |
| Module to Module | Port/Wire with local transport | Yes: Blueprint wires connect typed ports in-process. | Nothing else needed by default. |
| UI / SDK / operator | Gateway REST/SSE/WS/MCP JSON | Partly: status, preview, map, and command surfaces exist. | Generated Dart/Rust/TS SDK contracts are not complete. |
| Map to planner | MapService bundle + artifact path | Yes: `map.bundle`, `map.record`, `octomap.bt`, `occupancy.npz`, `map.pcd`. | Native direct OctoMap/ESDF builders are pending. |
| Global plan output | `GlobalPlanResult.to_wire()` JSON | Yes for Gateway/UI/replay payloads. | No binary planner protocol needed now. |
| Endpoint/replay bridge | LCM endpoint contract or local smoke transport | Partly: LCM adapters, JSON envelope, local smoke source, JSONL validator, and deployment validator exist. | This is not current hardware motion proof. |
| DDS | Typed DDS only for selected boundaries | Partly: typed registry and IDL metadata exist. | Generic pickle DDS is not product-grade; LiDAR/IMU DDS is not the default product bus. |
| SHM | High-volume same-host IPC | Backend exists. | No stable point-cloud/image schema or performance gate yet. |
| Native SLAM hot path | Local callbacks first; typed DDS only after split | Ingress contract exists. | Python runtime still uses the contract runner, not the C++ Fast-LIO backend. |

## Core Rules

- Product and capability code must not use ROS topic names as its primary data
  contract.
- Module-to-module traffic defaults to `ModulePort + LocalTransport`.
- Cross-process or cross-language traffic should use an explicit endpoint
  transport. Thunder endpoint and replay bridges prefer LCM first.
- ROS 2 remains a compatibility adapter only. It belongs under `runtime.adapters.ros2`,
  legacy native launch surfaces, simulator bridges, or field adapter code.
- `runtime.msgs` is the canonical in-process message model.
- Codec layers convert between `runtime.msgs` and ROS messages, LCM payloads, JSON
  envelopes, or future binary/shared-memory schemas.
- Runtime resolver/profile builder chooses transports and endpoint adapters.
  Normal modules should not branch on ROS/LCM/simulator details.

## Channel Names

Use product-neutral LingTu channel names in runtime contracts and new module
code:

| Channel | Meaning |
| --- | --- |
| `localization.odometry` | Canonical robot pose and twist estimate. |
| `localization.scan_synced_odometry` | Odometry sampled at the scan timestamp. |
| `localization.health` | Localization quality and degraded-state signal. |
| `perception.registered_cloud` | Current registered cloud or body-aligned cloud, depending on adapter contract. |
| `mapping.map_cloud` | World/map cloud used for map and terrain consumers. |
| `mapping.terrain_map_ext` | Extended terrain cloud or terrain analysis result. |
| `planning.goal_pose` | Navigation goal input. |
| `planning.global_path` | Global planner output path. |
| `planning.local_path` | Local planner output path. |
| `control.cmd_vel` | Muxed velocity command. |
| `system.health` | Runtime health summary. |

The exact list can grow, but names must stay capability-oriented rather than
transport-oriented.

## Legacy Topic Mapping

Legacy ROS names remain adapter aliases only:

| Legacy topic | LingTu channel | Policy |
| --- | --- | --- |
| `/Odometry` | `localization.odometry` | Adapter alias only. |
| `/nav/odometry` | `localization.odometry` | Canonical ROS adapter alias. |
| `/state_estimation_at_scan` | `localization.scan_synced_odometry` | Keep for TARE/CMU compatibility, not business code. |
| `/nav/state_estimation_at_scan` | `localization.scan_synced_odometry` | Canonical ROS adapter alias. |
| `/cloud_registered` | `perception.registered_cloud` or `mapping.map_cloud` | Adapter must declare frame semantics. |
| `/registered_scan` | `mapping.map_cloud` | TARE/CMU adapter alias. |
| `/sensor_scan` | `perception.sensor_frame_scan` | Legacy optional output; not part of Thunder Lite. |
| `/terrain_map_ext` | `mapping.terrain_map_ext` | Adapter alias only. |
| `/cmd_vel` | `control.cmd_vel` | Adapter alias; hardware safety must stay explicit. |

`/sensor_scan` is not a core Thunder Lite dependency. If the transform logic is
still needed, extract it as a pure kernel and keep ROS publication in an
adapter.

## Transport Direction

Thunder Lite:

- In-process only by default.
- Uses `ModulePort + LocalTransport`.
- Does not package ROS 2, TARE, `sensor_scan_generation`, or ROS SLAM bridges.

Thunder Endpoint/Nav:

- Keeps the module graph local where possible.
- Uses LCM endpoint adapters for cross-process endpoint boundaries.
- Uses ROS 2 only when integrating legacy SLAM, simulator, TARE, or existing
  external services.

Future hot-path streams:

- Use shared memory or binary schemas only for large point cloud/image paths.
- Do not make shared memory the default control-plane transport.

## Implementation Order

1. Add a channel contract module such as `runtime.contracts.channels`.
2. Move topic names in product/runtime code behind channel constants.
3. Keep ROS/LCM aliases in adapter contracts, not in capability modules.
4. Add static tests that ordinary modules do not import ROS packages or hardcode
   adapter topic names.
5. Mark legacy native ROS packages such as `sensor_scan_generation` as optional
   adapter surfaces.
6. Only after the contract is stable, consider extracting pure algorithm kernels
   from legacy ROS nodes.

## Non-Goals

- Do not build a full DDS replacement.
- Do not introduce a large custom discovery system.
- Do not rewrite all legacy ROS packages before the channel contract is stable.
- Do not rename legacy topics in place until every adapter and consumer is
  accounted for.
