# LingTu Roadmap

Status: current summary as of 2026-07-18

This root roadmap is a short product-facing summary. The detailed active work
board lives in [`docs/plans/current-roadmap.md`](./docs/plans/current-roadmap.md).

## Current Target

LingTu's near-term target is a ROS-free, typed-DDS Thunder field runtime with:

- one native navigation command writer;
- one hardware command consumer, `lingtu-driver`;
- remote Brainstem gRPC control through `/opt/lingtu/config/brainstem.env`;
- OctoPlanner3D as the product saved-map global planner;
- Gateway on port `5050` and MCP JSON-RPC on port `8090`;
- ROS 2 retained only as explicit compatibility, replay, or evaluation surface.

Current field command chain:

```text
Gateway / CLI / MCP / semantic goal
  -> lingtu-nav-dds
  -> /nav/cmd_vel
  -> lingtu-driver
  -> remote Brainstem gRPC WalkChecked
```

## Delivery Priorities

| Priority | Outcome | Status |
| --- | --- | --- |
| P0 | Prove saved-map localization alignment on the current field target. | Active |
| P0 | Prove no-motion route preview through OctoPlanner3D and the native endpoint. | Active |
| P0 | Run field fault injection for DDS/nav/driver loss, stale localization, stale traversability, Brainstem disconnect, and lease preemption. | Active |
| P0 | Run supervised motion smoke only after no-motion and safety gates pass. | Blocked until gates pass |
| P1 | Finish typed command acceptance/rejection feedback. | Active |
| P1 | Validate map save -> `octomap.ot` -> OctoPlanner3D preview as one package flow. | Active |
| P1 | Validate `tare_explore` as exploration target generation feeding normal navigation. | Next |
| P1 | Move high-rate camera image payloads to C++ SHM while retaining DDS metadata/health. | Next |
| P1 | Make MuJoCo consume the same final native `/nav/cmd_vel` command sink used by field services. | Next |

## Current Maturity View

| Area | Current state | Remaining gate |
| --- | --- | --- |
| Runtime framework | Mature Module/Blueprint graph and tests. | Continue boundary regression tests. |
| Native DDS field path | Locally locked for endpoint-only command ownership and typed DDS contracts. | Target-side sampling and fault injection. |
| Driver/Brainstem boundary | `lingtu-driver` remote Brainstem path, checked Walk RPC, lease/readiness checks, watchdog, and stale command handling are implemented locally. | Field fault injection and supervised motion smoke. |
| Global planning | OctoPlanner3D is the product default. | Saved-map artifact and live route preview evidence. |
| PCT/A* | Legacy/manual comparison surfaces. | Do not use as product readiness evidence unless explicitly selected. |
| Gateway/MCP | Current operator/integration surfaces. | Keep contracts generated/current and avoid exposing unsafe internals. |
| Web dashboard | Active product UI surface. | Continue UI-state contracts and evidence-backed actions. |
| Semantic/inspection/exploration | Implemented feature paths. | Field validation after navigation safety gates. |

## Evidence Rules

- Local tests prove local contracts only.
- Simulation proves the named simulation gate only.
- No-motion DDS or route preview proves readiness inputs, not real motion.
- Real field readiness needs dated target evidence under
  `docs/07-testing/field-runs/`.
- Plans are not shipped behavior; use `docs/CURRENT.md` to find the
  authoritative current document for a topic.
