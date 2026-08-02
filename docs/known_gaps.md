# Known LingTu Gaps

Status: current as of 2026-07-28

This file tracks gaps that matter to architecture, productization, or field
readiness. It should not repeat old closed incidents.

## P0/P1 Gaps

| Gap | Severity | Impact | Current direction |
| --- | --- | --- | --- |
| Native release provenance | P0 | MuJoCo Product acceptance refuses to run when selected sensor, SLAM, mapd, navigation, control/client, or tap binaries are older than their source/IDL closure. | Rebuild every selected native artifact, require zero strict-preflight blockers, then archive the report. |
| Field motion closure | P0 | The native DDS/driver code path is locally locked, but full real-motion closure still needs target-side fault injection and supervised motion evidence. | Keep no-motion route preview, driver watchdog checks, and real motion smoke as separate gates. |
| Dynamic-obstacle residual evidence | P1 | Column carving, rolling occupancy, and decay are implemented, but a current moving-person Product run and long-duration MID-360 evidence are still missing. | Pass `moving_person_clear`, dataset replay, and field residual/resource gates before making a no-ghosting claim. |
| Native map control/query convergence | P1 | Realtime `mapd` is native, while some persistent map control/query paths still cross a thin Python Host facade. | Make typed map control/query the only field path and retain Gateway as translation-only. |
| Velocity and path smoothing | P1 | The ROS-free C++ velocity smoother is tested but not wired into the final Product command path; a replaceable collision-aware path smoother is absent. | Integrate smoothing before final safety with hard-stop bypass, diagnostics, MuJoCo tuning, and rollback evidence. |
| Route/follow/dock products | P2 | Inspection waypoints are mature, but generic route operations, target following, and docking/charging do not have complete Product contracts and evidence. | Follow the capability matrix; do not present foundations as delivered products. |
| Cross-language transport schema | P2 | DDS/shared-memory payloads are not yet uniformly versioned for long-term Dart/Rust/C++ interop. | Promote typed schemas with explicit version, frame, timestamp, bounds, and compatibility tests. |

## Current Non-Goals

- Do not make ROS 2 the product API.
- Do not expose planner backend internals directly to Gateway or UI.
- Do not add a custom transport framework beyond the existing Port/Wire/Transport
  model until typed schemas are stable.

## Recently Closed

- Windows is no longer documented as categorically unusable. Current focused
  tests run under the Windows developer environment used by this repository.
- Global planning now has a stable wire payload for preview and mission status:
  `lingtu.global_plan.v1`.
- The physical Thunder command boundary is now documented as `endpoint_only`
  through logical `/nav/cmd_vel` (DDS wire `rt/nav/cmd_vel`), unique
  `lingtu-driver`, and remote Brainstem gRPC. Remaining work is field evidence,
  not another default Python DDS writer.
- Current first-party documentation entrypoints, architecture contracts,
  deployment runbooks, package READMEs, and generated API inventories were
  reorganized on 2026-07-28. Superseded plans and duplicate binary reports were
  deleted; research and dated evidence remain explicitly non-authoritative.
