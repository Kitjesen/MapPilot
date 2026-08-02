# LingTu Runtime Bus Decision

Status: accepted architecture memory
Date: 2026-06-18
Audience: runtime, endpoint, adapter, and deployment maintainers
Replaced by: not replaced

## Decision

LingTu should not replace ROS 2 by copying the ROS topic model into business
code. Product real-time boundaries use one product bus: typed CycloneDDS.
Algorithms inside one process call each other directly.

```text
Cross-process product data plane
  -> typed DDS topics

Same-process algorithm chain
  -> direct C++/Python function calls and in-memory data structures

High-volume same-host snapshots
  -> DDS metadata + SHM / mmap / binary snapshot when needed

Replay/debug/legacy compatibility
  -> explicit adapters only; LCM is not a product data plane
```

Normal modules speak LingTu channels and `runtime.msgs` types. ROS 2 topics, LCM
channels, simulator feeds, replay files, and shared-memory paths are adapter
aliases only when the resolved Product/env contract explicitly enables them.

## Communication Plan And Status

Current target: product field runtime and real-equivalent simulation. Real-time
product services communicate through typed DDS at service boundaries; internal
algorithm chains stay in-process.

| Boundary | Planned transport | Done now | Not done |
| --- | --- | --- | --- |
| Module to Module | Port/Wire with local transport | Yes: Blueprint wires connect typed ports in-process. | Nothing else needed by default. |
| UI / SDK / operator | Gateway REST/SSE/WS/MCP JSON | Partly: status, preview, map, and command surfaces exist. | Generated Dart/Rust/TS SDK contracts are not complete. |
| Gateway to native navigation | Process-wide C++ CycloneDDS command client loaded through a small C ABI | Goal, cancel, teleop, safety, and inspection commands use typed request/ACK envelopes through one reused `liblingtu_nav_client.so` session. Gateway does not fork a command subprocess and Python owns no field DDS writer. | Validate the ABI and endpoint binary on S100P as part of every deployment gate. |
| Map to planner | MapService capability bundle | Native occupancy, ESDF, traversability and optional embedded OctoMap builders publish versioned bundles; external OctoMap conversion is an explicit build mode. | Remove the remaining planner-side legacy filesystem reader and validate on S100P. |
| Global plan output | `GlobalPlanResult.to_wire()` JSON | Yes for Gateway/UI/replay payloads. | No binary planner protocol needed now. |
| Endpoint/replay bridge | typed DDS for product, local/LCM only for replay or smoke | Partly: LCM adapters and JSONL validators still exist for replay/debug. | LCM must not be selected by field Products. |
| DDS | Typed DDS for all field service boundaries | Livox, Fast-LIO2, traversability, native nav, exploration, camera metadata/status, and teleop request/final-command ownership are C++ CycloneDDS paths. | GNSS and some compatibility readers still need retirement evidence. |
| SHM | High-volume same-host IPC | Camera color/depth payloads use the native POSIX SHM data plane with typed DDS/status metadata; point-cloud/status snapshots also exist. | Continue field readiness checks for SHM sequence freshness and consumer coverage. |
| Native SLAM hot path | C++ Fast-LIO2 with typed DDS ingress/egress | Live field runtime and scan/odometry/cloud contracts exist. | Continue hardware regression and map/localization acceptance; do not reintroduce Python DDS into SLAM. |

## Core Rules

- Product and capability code must not use ROS topic names as its primary data
  contract.
- Product real-time main paths must use typed DDS at service boundaries.
- Module-to-module traffic defaults to `ModulePort + LocalTransport`.
- Same-process C++ endpoint internals must not introduce LCM. OctoPlanner3D,
  LocalPlanner, PathFollower, and command arbitration call each other directly
  when they live inside one endpoint process.
- Cross-process or cross-language Product traffic must use typed DDS. The
  Thunder `env=real` Product runtime uses the `thunder_dds_v1` typed
  CycloneDDS contract.
- LCM is limited to replay, debug, legacy adapters, or external benchmark
  shims. It must not be required by `nav`, `teleop_avoid`, `map`, `tracking`,
  `inspection`, or `explore` Products.
- ROS 2 remains a compatibility adapter only. It belongs outside the product
  source tree or in explicitly quarantined legacy launch/simulator bridges.
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
| `/sensor_scan` | `perception.sensor_frame_scan` | Legacy optional output; not part of the `lite` Profile. |
| `/terrain_map_ext` | `mapping.terrain_map_ext` | Adapter alias only. |
| `/cmd_vel` | `control.cmd_vel` | Adapter alias; hardware safety must stay explicit. |

`/sensor_scan` is not a core `lite` Profile dependency. If the transform logic is
still needed, extract it as a pure kernel and keep ROS publication in an
adapter.

## Transport Direction

`lite` Profile:

- In-process only by default.
- Uses `ModulePort + LocalTransport`.
- Does not package ROS 2, TARE, `sensor_scan_generation`, or ROS SLAM bridges.

Thunder Endpoint/Nav:

- Keeps the module graph local where possible.
- Uses the native typed DDS endpoint contract (`thunder_dds_v1`) and
  `cpp_slam_status` localization adapter for the production field boundary
  (native Livox SDK2 ingest + C++ CycloneDDS SLAM/status).
- Uses LCM endpoint adapters for smoke/replay bridges and optional
  smoke/replay checks that do not require the native DDS sensor stack.
- Uses ROS 2 only when integrating legacy SLAM, simulator, TARE, or existing
  external services.
- Resolves a `route_contract` in addition to `module_transport` and
  `endpoint_transport`. For the Thunder `env=real` RunPlan, the
  expected shape is
  `module_transport=local`, `endpoint_transport=dds`,
  `endpoint_contract=thunder_dds_v1`, and `route_contract=robot`.
  The route contract validates canonical topic ownership and DDS schema
  bindings; it does not by itself make ordinary Modules import or speak DDS.
  Use `Blueprint.route_contract(...)` for metadata-only contracts and reserve
  `Blueprint.routed_delivery(...)` for deliberate internal routed transport.

Future hot-path streams:

- Use shared memory or binary schemas only for large point cloud/image paths.
- Do not make shared memory the default control-plane transport.

## Remaining cyclonedds-python Surface

Python navigation DDS input/output adapters have been deleted. A regression
test now keeps the remaining `runtime.adapters.dds.reader` imports bounded to:

- camera DDS ingestion;
- GNSS DDS ingestion;
- optional IMU and LiDAR compatibility adapters;
- the legacy Python TARE bridge.

Navigation goal/cancel/teleop writers are no longer part of this list. Their
field owner is `liblingtu_nav_client.so`, reused by Gateway and `GoalService`.

The remaining migration order is GNSS, TARE, then compatibility adapters.
Camera high-volume color/depth frames are now a native SHM data plane with
low-rate metadata and health on typed DDS/status. GNSS and TARE should move
their consumers into C++ endpoints. Diagnostic scripts may keep
cyclonedds-python as an optional tool; it must never become a robot startup
dependency.

## Implementation Order

1. **Done:** make C++ navigation the single field command boundary. Goal,
   cancel, teleop, global/local path, waypoint, and final command ownership are
   explicit.
2. **Done:** replace Gateway subprocess and Python DDS command writers with the
   process-wide C++ `NavigationCommandClient`.
3. **Done:** route natural-language requests through `SemanticPlanner`; only a
   resolved map-frame goal may enter DDS.
4. **Done locally:** lock endpoint control modes to `autonomy`, `teleop`, or
   `teleop_avoid`; pure teleop has no SLAM dependency, while teleop avoidance
   fails closed on missing motion context.
5. **Done locally:** native endpoint motion passes input gates, local-path
   safety, final command safety, driver-control readiness, and zero-on-loss
   behavior before `/nav/cmd_vel` can remain non-zero.
6. **Done locally:** typed navigation and inspection requests carry request IDs;
   the client completes submission only after a matching admission ACK and
   propagates the endpoint rejection reason. Goal admission means asynchronous
   planning started, not that planning or navigation completed. The C ABI
   exposes version and capability checks.
7. **Done locally:** move camera color/depth ingestion to C++ DDS metadata plus
   a versioned SHM ring; keep low-rate camera info/health on typed DDS.
8. **Next:** make MuJoCo publish sensor DDS and consume final `/nav/cmd_vel`
   through the same C++ services and learned locomotion sink used by field
   acceptance, with no Python planner substitute.
9. **Then:** migrate GNSS and TARE readers, and delete remaining field-ineligible
   compatibility adapters after equivalent acceptance evidence exists.

## Non-Goals

- Do not build a full DDS replacement.
- Do not introduce a large custom discovery system.
- Do not rewrite all legacy ROS packages before the channel contract is stable.
- Do not rename legacy topics in place until every adapter and consumer is
  accounted for.
