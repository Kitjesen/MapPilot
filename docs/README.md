# LingTu Documentation

This directory is the written source of truth for LingTu's current runtime,
architecture, deployment, and validation work. Historical notes remain available
under `archive/`, but they are not the authority for new development.

For a one-page authority map, start with [`CURRENT.md`](./CURRENT.md). For
cleanup decisions and archive candidates, use [`DOCS_TRIAGE.md`](./DOCS_TRIAGE.md).

## Current Reading Path

| Order | Document | Purpose |
| ---: | --- | --- |
| 1 | [`architecture/SYSTEM_DESIGN.md`](./architecture/SYSTEM_DESIGN.md) | Paper-style system overview and current technical argument. |
| 2 | [`architecture/NAVIGATION_COMPUTE_CONTRACT.md`](./architecture/NAVIGATION_COMPUTE_CONTRACT.md) | Planning, local planning, safety, and control boundaries. |
| 3 | [`architecture/GLOBAL_PLANNING_CONTRACT.md`](./architecture/GLOBAL_PLANNING_CONTRACT.md) | Global planner input/output and backend boundary. |
| 4 | [`architecture/LINGTU_RUNTIME_BUS_DECISION.md`](./architecture/LINGTU_RUNTIME_BUS_DECISION.md) | Module ports, channels, and transport policy. |
| 5 | [`QUICKSTART.md`](./QUICKSTART.md) | How to run the current profiles. |
| 6 | [`REPO_LAYOUT.md`](./REPO_LAYOUT.md) | Where code and docs belong. |
| 7 | [`07-testing/README.md`](./07-testing/README.md) | Validation and acceptance entrypoint. |

## Document Classes

| Class | Directory | Rule |
| --- | --- | --- |
| Architecture contracts | `architecture/` | Current design only. Keep these concise and testable. |
| Product plans and PRDs | `plans/` | Forward-looking work. Must name owner, scope, and acceptance. |
| Operator docs | `01-getting-started/`, `04-deployment/`, `api/` | Commands, services, and API behavior. |
| Validation docs | `07-testing/` | Evidence, gates, and field/simulation acceptance. |
| Historical material | `archive/`, `superpowers/` | Reference only. Do not cite as current behavior without rechecking code. |
| Paper assets | `09-paper/`, `media/`, `assets/` | Publication drafts, figures, and supporting media. |

## Current Architecture Position

LingTu is Module-First: `Module` is the runtime unit, `Blueprint` is the
orchestration unit, and ports are the module data boundary. ROS 2, DDS, LCM,
shared memory, simulators, and replay files are transports or adapters, not the
business API.

The current product path is:

```text
drivers/localization -> maps -> planning -> local planning -> path following
                     -> safety/cmd mux -> robot driver
```

Global planning is owned by `src/nav/services/plan/`. OctoPlanner3D is an
algorithm backend behind the planning contract, not the public Navigation API.

## Staleness Policy

- If a document describes ROS topics as the primary module API, treat it as
  legacy unless it explicitly says adapter/compatibility.
- If a document mentions NOVA as the current robot name, treat it as historical.
- If a document claims PCT is the default product planner, treat it as historical;
  OctoPlanner3D is the default map-backed planner.
- If a document disagrees with `AGENTS.md`, `src/runtime/blueprint.py`,
  `src/runtime/stream.py`, or `src/nav/services/plan/contracts.py`, update the
  document or move it to `archive/`.
