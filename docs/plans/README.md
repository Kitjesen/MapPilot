# LingTu Current Plans

Status: current plan index

This directory contains only active forward-looking plans. Shipped behavior
lives in `docs/architecture/`, operator procedures live in `docs/04-deployment/`,
and old snapshots live in git history.

| Document | Scope |
| --- | --- |
| [`current-roadmap.md`](./current-roadmap.md) | Current product/runtime roadmap and remaining gates. |
| [`message-dds-cleanup.md`](./message-dds-cleanup.md) | Active cleanup of duplicate DDS implementations and message ownership. |
| [`ue5-playable-vertical-slice.md`](./ue5-playable-vertical-slice.md) | Execution plan for the first UE5 human-input-to-MuJoCo playable qualification. |
| [`sensor-noise-injection-tdd.md`](./sensor-noise-injection-tdd.md) | Proposed deterministic per-stream sensor-noise contract and implementation gates. |
| [`robot-mounted-weapon-gameplay-tdd.md`](./robot-mounted-weapon-gameplay-tdd.md) | Proposed sim-only payload, Tactical fire input, deterministic ballistics, recoil, and UE presentation slice. |

Current roadmap headline: native Thunder DDS/driver code paths are locally
locked; do not mark field motion complete until target-side no-motion preview,
fault injection, and supervised motion evidence exist.

## Rules

- Do not keep stale PRDs here after a contract lands.
- Do not copy historical architecture into `docs/archive/`; use git history.
- Plans must name the product mode, runtime boundary, data contract, and the
  check that proves completion.
