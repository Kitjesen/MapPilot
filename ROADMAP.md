# LingTu Roadmap

Status: current summary as of 2026-07-28

The detailed active work board is
[`docs/plans/current-roadmap.md`](docs/plans/current-roadmap.md). Capability
maturity is tracked separately in
[`NAVIGATION_CAPABILITY_MATRIX.md`](docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md).

## Current Target

LingTu is converging on a ROS-free, typed-DDS field runtime:

```text
native sensor -> native SLAM -> mapd / traversability -> navd
              -> /nav/cmd_vel -> driver -> Brainstem

HostBus <-> Gateway / Agent / MCP
```

`navd` owns navigation state and the final logical command. Standalone native
traversability owns `/nav/traversability`; `mapd` owns map state and scene.
The Python Host is an API/business integration boundary, not a duplicate
navigation or high-rate map runtime.

## Delivery Priorities

| Priority | Outcome | Status |
| --- | --- | --- |
| P0 | Self-contained native release and zero-blocker strict Product preflight. | Active |
| P0 | Stable MuJoCo mapping/no-map SLAM and `teleop_avoid` closed-loop acceptance. | Active |
| P0 | S100P no-motion readiness, fault injection, then supervised motion. | Blocked on preceding gates |
| P1 | Native typed map control/query as the only field path. | Active |
| P1 | Dynamic-obstacle residual and long-duration resource evidence. | Gate implemented; accepted run pending |
| P1 | Product integration of native velocity smoothing and a replaceable path smoother. | Partial |
| P2 | Generic route operations, target following, and docking/charging Products. | Foundations or missing |

## Evidence Rules

- Source and unit tests prove implementation, not Product completion.
- MuJoCo proves only the named simulation scenario.
- No-motion readiness does not prove safe locomotion.
- Physical claims require date-prefixed evidence under
  `docs/07-testing/field-runs/`.
- Research and plans are never runtime truth; use `docs/CURRENT.md` to find the
  authoritative contract.
