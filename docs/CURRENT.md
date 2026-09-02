# LingTu Current Status

Status: current authority and claim map
Updated: 2026-09-03

This page answers two questions: where the current source of truth lives, and
what the repository may claim today. It is not a build log, implementation
diary, or substitute for dated validation evidence.

## Runtime Model

LingTu has exactly two public runtime environments: `real` and `sim`. Local
development is a verification context, not a third `env` value.

```text
RobotConfig + Product + env(real | sim)
                 -> RunPlan
                 -> ProductControl
                 -> real SystemdRunner | sim direct-child runner
```

`Product` is env-independent. `RunPlan` is the resolved artifact for one env,
and `ProductControl` is the only public lifecycle entry for `switch`, `status`,
and `stop`.

## Current Support Boundary

| Context | Current use | Claim boundary |
| --- | --- | --- |
| Local checkout | Unit, contract, architecture, and offline integration checks | Does not prove simulator or robot behavior |
| `env=sim` | Windows x64 and Linux/WSL x86_64 native MuJoCo integration and Product qualification | Only an explicit Product acceptance report proves that Product on that platform |
| `env=real` | S100P/RDK X5 native field runtime | No autonomous-motion readiness claim without fresh target, no-motion, fault-injection, and supervised-motion evidence |

The repository is suitable for local development and contract validation.
Open Product, release, and field gates are summarized in
[`known_gaps.md`](./known_gaps.md) and planned in
[`plans/current-roadmap.md`](./plans/current-roadmap.md).

## Authority Map

| Topic | Source of truth |
| --- | --- |
| Product capabilities, process roles, and logical topics | [`config/runtime_graph/products/`](../config/runtime_graph/products/) |
| `real` and `sim` implementation mapping | [`config/runtime_graph/envs/`](../config/runtime_graph/envs/) |
| Product resolution and immutable RunPlan | [`src/lingtu/run_plan.py`](../src/lingtu/run_plan.py) and [`src/lingtu/assembly/`](../src/lingtu/assembly/) |
| Product lifecycle and env routing | [`src/lingtu/control.py`](../src/lingtu/control.py) |
| System ownership and dependency direction | [`architecture/SYSTEM_DESIGN.md`](./architecture/SYSTEM_DESIGN.md) |
| Host Module, Blueprint, Port, Wire, and transport model | [`architecture/LINGTU_RUNTIME_BUS_DECISION.md`](./architecture/LINGTU_RUNTIME_BUS_DECISION.md) |
| Field Product and native process boundary | [`architecture/FIELD_PRODUCTS.md`](./architecture/FIELD_PRODUCTS.md) and [`architecture/NATIVE_RUNTIME.md`](./architecture/NATIVE_RUNTIME.md) |
| Navigation compute and dataflow | [`architecture/NAVIGATION_COMPUTE_CONTRACT.md`](./architecture/NAVIGATION_COMPUTE_CONTRACT.md) and [`architecture/NAVIGATION_RUNTIME_DATAFLOW.md`](./architecture/NAVIGATION_RUNTIME_DATAFLOW.md) |
| Map and environment-map ownership | [`architecture/MAP_SERVICE_CONTRACT.md`](./architecture/MAP_SERVICE_CONTRACT.md) and [`architecture/ENVIRONMENT_MAP.md`](./architecture/ENVIRONMENT_MAP.md) |
| Localization runtime | [`architecture/LOCALIZATION_RUNTIME.md`](./architecture/LOCALIZATION_RUNTIME.md) |
| Simulation workspace | [`sim/ARCHITECTURE.md`](../sim/ARCHITECTURE.md) |
| Repository placement and generated outputs | [`REPO_LAYOUT.md`](./REPO_LAYOUT.md) |
| Commands and task entry points | [`QUICKSTART.md`](./QUICKSTART.md) and [`08-reference/README.md`](./08-reference/README.md) |
| Current capability evidence ledger | [`architecture/NAVIGATION_CAPABILITY_MATRIX.md`](./architecture/NAVIGATION_CAPABILITY_MATRIX.md) |
| Reusable validation gates | [`07-testing/README.md`](./07-testing/README.md) |
| Dated results | [`07-testing/field-runs/README.md`](./07-testing/field-runs/README.md) |

Running configuration and typed schemas outrank prose when they conflict.
Update the stale page rather than creating another source of truth.

## Documentation Status Terms

| Label | Meaning |
| --- | --- |
| Current | Supported behavior or an active contract |
| Reference | Stable interface, command, schema, configuration, or generated inventory |
| Evidence | A dated result limited to the named target, environment, and gate |
| Plan | Intended work that is not shipped behavior |
| Research | Non-authoritative upstream or algorithm investigation |
| Worklog | Session-continuity material, not implementation authority |

## Evidence Boundaries

- A dry run proves Product resolution only.
- A passing local test does not prove `env=sim` or `env=real` behavior.
- Component evidence does not become Product evidence by aggregation.
- Windows evidence does not prove Linux behavior, and Linux/WSL evidence does
  not prove Windows-native behavior.
- Simulation, target-compute, field no-motion, and supervised field motion are
  separate evidence levels.
- A running process is not motion readiness. Map identity, localization,
  freshness, safety, command ownership, and driver acknowledgement still apply.

## Non-Authoritative Locations

| Location | Use |
| --- | --- |
| [`plans/`](./plans/README.md) | Active forward-looking work only |
| [`research/`](./research/README.md) | Upstream reviews and algorithm investigations |
| [`07-testing/field-runs/`](./07-testing/field-runs/README.md) | Immutable dated evidence; never current behavior by itself |
| [`worklogs/`](./worklogs/README.md) | Task recovery and continuity |

Git history is the archive. Do not create a second `docs/archive/` tree or keep
completed implementation diaries in current contracts.
