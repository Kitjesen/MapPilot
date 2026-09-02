# LingTu Current Plans

Status: current plan index

This directory contains only active forward-looking work. The roadmap owns
cross-domain order; the other files are explicitly bounded implementation plans.
Shipped behavior lives in `docs/architecture/`, and Git history keeps old plans.

| Document | Scope |
| --- | --- |
| [`current-roadmap.md`](./current-roadmap.md) | Current product/runtime roadmap and remaining gates. |
| [`ue5-playable-vertical-slice.md`](./ue5-playable-vertical-slice.md) | Execution plan for the first UE5 human-input-to-MuJoCo playable qualification. |
| [`sensor-noise-injection-tdd.md`](./sensor-noise-injection-tdd.md) | Proposed deterministic per-stream sensor-noise contract and implementation gates. |
| [`robot-mounted-weapon-gameplay-tdd.md`](./robot-mounted-weapon-gameplay-tdd.md) | Proposed sim-only payload, Tactical fire input, deterministic ballistics, recoil, and UE presentation slice. |

## Rules

- Do not keep stale PRDs here after a contract lands.
- Do not copy historical architecture into `docs/archive/`; use git history.
- Plans must name the product mode, runtime boundary, data contract, and the
  check that proves completion.
