# LingTu Documentation

**Status:** Current documentation index
**Audience:** Developers, integrators, operators, and reviewers
**Runs on:** Local development, `env=sim`, and `env=real`

LingTu is an autonomous-navigation system for quadruped robots.

The documentation is intentionally flat: ten maintained pages describe the
current system, while code, configuration, schemas, and package READMEs remain
the detailed sources of truth.

## Choose one page

| Need | Page |
| --- | --- |
| Install dependencies, choose a Product, or make a first run | [Getting started](./getting-started.md) |
| Understand ownership, layers, messages, and repository shape | [Architecture](./architecture.md) |
| Understand Products, RunPlans, processes, maps, and motion flow | [Runtime](./runtime.md) |
| Work with packages, sessions, MuJoCo, RobotSimUE, or simulation gates | [Simulation](./simulation.md) |
| Change code, configuration, schemas, or generated outputs | [Development](./development.md) |
| Deploy, update, diagnose, record, or operate a robot | [Operations](./operations.md) |
| Integrate through REST, MCP, SDK, SSE, or camera transport | [API](./api.md) |
| Select the right local, simulation, or field validation level | [Testing](./testing.md) |
| See open work, non-goals, and release gates | [Roadmap](./roadmap.md) |

## The four axes

Do not infer runtime meaning from directory names alone.

| Axis | Meaning |
| --- | --- |
| `src/` | Product and domain source, organized by functional owner |
| `sim/` | Simulation workspace and the `sim.*` Python namespace |
| `env=real|sim` | The outer runtime environment selected when resolving a Product |
| `build/`, `install/`, `dist/` | Development builds, standard install trees, and release packages |

The repository does not duplicate algorithms under root-level `real/` and
`sim/` trees. It also does not commit a root `bin/` directory. Native
executables enter `bin/` only inside a CMake install prefix or installed
release.

## Runtime in one line

```text
RobotConfig + Product + env(real | sim)
                 -> RunPlan
                 -> ProductControl
                 -> real/systemd | sim/direct children
```

A Product is immutable and env-independent. ProductControl is the only public
Product lifecycle entry for `switch`, `status`, and `stop`. Blueprint owns the
Module graph inside one Python Host; it does not orchestrate native processes.

## Authority order

When prose and implementation disagree, fix the stale prose. Use these sources
in this order:

1. Typed schemas and machine-validated configuration.
2. `config/runtime_graph/`, RobotConfig, and the Product compiler.
3. Runtime and domain code under `src/` and `sim/`.
4. Package-local READMEs and these ten documents.
5. Plans, issue discussions, historical commits, and old evidence.

Current Products, roles, topics, and environment mappings are defined in
[`config/runtime_graph/`](../config/runtime_graph/README.md). Import boundaries
are enforced by
[`config/architecture_layers.yaml`](../config/architecture_layers.yaml).

## Evidence boundaries

- A dry run proves Product resolution only.
- A local test does not prove simulation or field behavior.
- A component pass does not prove a complete Product.
- Windows and Linux/WSL evidence are independent.
- Simulation never proves S100P motion safety.
- A running process is not navigation readiness.

Use [Testing](./testing.md) before making a capability claim.

## Documentation policy

The `docs/` tree contains exactly these ten Markdown pages plus referenced
assets. It has no archive, worklog, research, plan, or dated-run subtree.

Current conclusions from useful older pages are consolidated here. Superseded
designs, dated evidence, and implementation diaries remain available through
Git history rather than competing with current guidance.

Detailed instructions stay beside their owner when that reduces duplication,
for example [`src/README.md`](../src/README.md),
[`sim/README.md`](../sim/README.md), and
[`scripts/README.md`](../scripts/README.md).

The Web guide renders this maintained set at `/guide/`. FastAPI keeps `/docs`
for its live OpenAPI interface.
