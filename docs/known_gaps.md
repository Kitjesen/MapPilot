# Known LingTu Gaps

Status: current gap summary
Updated: 2026-09-03

This page lists open productization and field-readiness gaps. It does not keep
closed incidents, exact build output, or dated test results. Those belong in
Git history and [`07-testing/field-runs/`](./07-testing/field-runs/README.md).

## Open Gaps

| Gap | Priority | Why it remains open | Completion boundary |
| --- | --- | --- | --- |
| Native MuJoCo Product parity | P0 | Component and compile evidence does not establish every Product on both required workstation platforms. | Archive exact ProductControl lifecycle, scenario, zero, cleanup, rollback, and repeatability reports independently for Windows and Linux/WSL. |
| Reproducible release | P0 | Development builds and component provenance are not a deployable, reviewed release. | Assemble the canonical install tree, pass package/installer/rollback checks, and clear the complete dependency and license boundary. |
| S100P field motion | P0 | Local and simulation evidence cannot prove physical command safety. | Pass fresh deployment provenance, no-motion readiness, fault injection, bounded supervised motion, terminal zero, and driver acknowledgement on the target. |
| `teleop_avoid` stability | P0 | Mapping/no-map SLAM and assisted avoidance still need representative Product-level closure. | Keep native sensor-to-command flow tracking through free-space, obstacle, stop, cleanup, and repeatability scenarios without production-only threshold weakening. |
| Dynamic-obstacle residuals | P1 | Clearing logic exists, but current labelled replay and long-duration MID-360 evidence are incomplete. | Pass moving-person, replay, residual, thin-obstacle, reset, CPU, memory, and DDS-volume gates. |
| Map control/query convergence | P1 | Native `mapd` owns the hot path, while selected persistent control/query surfaces still cross a thin Host facade. | Make typed native control/query the sole field path and keep Gateway translation-only. |
| Motion and path smoothing | P1 | Native velocity smoothing is wired, but Product and field tuning evidence is incomplete; collision-aware path smoothing remains absent. | Validate ramp, reversal, rotation, emergency zero, and safety ordering, then define a replaceable collision-aware path-smoother contract. |
| Route, following, and docking Products | P2 | Foundations exist without complete lifecycle, safety, and acceptance contracts. | Deliver each as an explicit Product with typed state, failure behavior, simulation evidence, and physical evidence where applicable. |
| Cross-language transport schema | P2 | DDS and shared-memory payloads are not yet uniformly versioned for long-term C++/Rust/Dart interop. | Define explicit versions, frames, timestamps, bounds, and compatibility tests at the owning schema boundary. |

Detailed capability maturity lives in
[`architecture/NAVIGATION_CAPABILITY_MATRIX.md`](./architecture/NAVIGATION_CAPABILITY_MATRIX.md).
Execution order lives in [`plans/current-roadmap.md`](./plans/current-roadmap.md).

## Current Non-Goals

- Do not make ROS 2 the Product API.
- Do not expose planner backend internals directly to Gateway or UI.
- Do not add a second Product lifecycle, field planner, map hot path, or command
  writer.
- Do not add a custom transport framework before the typed schema boundary is
  stable.

When a gap closes, update the owning contract and add the applicable dated
evidence. Remove the gap instead of keeping a permanent “recently closed” log.
