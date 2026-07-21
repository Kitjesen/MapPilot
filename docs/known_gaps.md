# Known LingTu Gaps

Status: current as of 2026-07-18

This file tracks gaps that matter to architecture, productization, or field
readiness. It should not repeat old closed incidents.

## P0/P1 Gaps

| Gap | Severity | Impact | Current direction |
| --- | --- | --- | --- |
| Cross-language transport schema | P1 | DDS/shared-memory/LCM payloads are not yet uniformly versioned for long-term Dart/Rust/C++ interop. | Promote typed schemas with `schema_version`, `frame_id`, timestamp, and payload version. |
| UI-facing runtime contracts | P1 | UI can call Gateway, but not every runtime object has a stable generated contract. | Start with global planning `lingtu.global_plan.v1`, then map/status/control payloads. |
| Field motion closure | P1 | The native DDS/driver code path is locally locked, but full real-motion closure still needs target-side fault injection and supervised motion evidence. | Keep no-motion route preview, driver watchdog checks, and real motion smoke as separate gates. |
| Historical docs drift | P1 | Some archived, package-local, or older plan docs still describe ROS 2/NOVA/PCT as current product defaults. | Use `docs/README.md`, `docs/CURRENT.md`, and architecture contracts as current entrypoints; update high-traffic docs first and demote stale evidence. |

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
  refreshed on 2026-07-18; dated evidence and hidden tool memory remain
  explicitly non-authoritative.
