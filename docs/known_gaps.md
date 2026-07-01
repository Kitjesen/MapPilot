# Known LingTu Gaps

Status: current as of 2026-06-30

This file tracks gaps that matter to architecture, productization, or field
readiness. It should not repeat old closed incidents.

## P0/P1 Gaps

| Gap | Severity | Impact | Current direction |
| --- | --- | --- | --- |
| Cross-language transport schema | P1 | DDS/shared-memory/LCM payloads are not yet uniformly versioned for long-term Dart/Rust/C++ interop. | Promote typed schemas with `schema_version`, `frame_id`, timestamp, and payload version. |
| UI-facing runtime contracts | P1 | UI can call Gateway, but not every runtime object has a stable generated contract. | Start with global planning `lingtu.global_plan.v1`, then map/status/control payloads. |
| Hardware readiness evidence | P1 | Simulation, server, and endpoint communication evidence must not be presented as real-hardware readiness. | Keep current server/sim claims separate from any future hardware campaign. |
| Historical docs drift | P1 | Some archived or older docs still describe ROS 2/NOVA/PCT as current product defaults. | Use `docs/README.md` and `architecture/SYSTEM_DESIGN.md` as current entrypoints; archive stale docs. |

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
