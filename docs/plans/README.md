# LingTu Current Plans

This directory contains only active forward-looking plans. Shipped behavior
lives in `docs/architecture/`, operator procedures live in `docs/04-deployment/`,
and old snapshots live in git history.

| Document | Scope |
| --- | --- |
| [`current-roadmap.md`](./current-roadmap.md) | Current product/runtime roadmap and remaining gates. |

Current roadmap headline: native Thunder DDS/driver code paths are locally
locked; do not mark field motion complete until target-side no-motion preview,
fault injection, and supervised motion evidence exist.

## Rules

- Do not keep stale PRDs here after a contract lands.
- Do not copy historical architecture into `docs/archive/`; use git history.
- Plans must name the product mode, runtime boundary, data contract, and the
  check that proves completion.
