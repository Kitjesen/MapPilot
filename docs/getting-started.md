# Getting Started

**Status:** Current setup and first-run guide
**Audience:** New contributors, simulation users, and field operators
**Runs on:** Local development, `env=sim`, and `env=real`

This page takes a clean checkout to a resolved Product. It separates local
setup, simulation, and robot-side work so that one result is not mistaken for
another.

## Prerequisites

- Git and Python 3.10 or newer.
- [uv](https://docs.astral.sh/uv/) for the locked Python environment.
- CMake and a supported compiler for native components.
- MuJoCo for native simulation work.
- The target-specific SDK and CycloneDDS toolchain only when that target needs
  them.

Install the portable development environment:

```bash
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
```

No hardware connection or motion occurs during these commands.

## Choose Product and environment

The current Product declarations are:

| Product | Intent | Saved map |
| --- | --- | --- |
| `teleop` | Operator motion through native limits and final gates | No |
| `teleop_avoid` | Operator intent with live local avoidance | No |
| `map` | Build and save a map | No |
| `explore` | Explore while mapping, or cover a selected saved map | Optional |
| `nav` | Navigate on a validated saved map | Yes |
| `tracking` | Follow a selected person through navigation goals | Yes |
| `inspection` | Execute a persisted inspection route | Yes |

Public environment values are exactly `real` and `sim`. Local development is a
verification context, not a third env.

## Preview a Product

Start with a side-effect-free resolution:

```bash
uv run --locked python -m lingtu.control switch teleop \
  --robot doso/thunder_v4 --env sim --dry-run --json
```

The output is a resolved RunPlan. A successful dry run proves that Product,
RobotConfig, and environment resolution agree; it does not prove that any
process can start.

## Run in simulation

Use ProductControl for the full lifecycle:

```bash
uv run --locked python -m lingtu.control switch teleop \
  --robot doso/thunder_v4 --env sim
uv run --locked python -m lingtu.control status \
  --robot doso/thunder_v4 --env sim --json
uv run --locked python -m lingtu.control stop \
  --robot doso/thunder_v4 --env sim
```

The `sim` runner owns direct child processes. Follow
[Simulation](./simulation.md) for package, session, MuJoCo, and RobotSimUE
workflows.

## Inspect a field target without motion

On the robot or target compute:

```bash
python -m lingtu.control status --robot unitree/go2 --env real --json
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
```

Only proceed to a switch after the correct release, RobotConfig, network,
devices, map, and operator authority are known. A typical saved-map start is:

```bash
python -m lingtu.control switch nav \
  --robot unitree/go2 --env real --map MAP_NAME
python -m lingtu.control stop --robot unitree/go2 --env real
```

These commands can affect a physical system. Follow
[Operations](./operations.md) and [Testing](./testing.md); do not treat them as
a generic copy-and-paste local example.

## Build and output layout

The source tree and generated artifacts have different jobs:

```text
source/CMake -> build/<component>/...
cmake --install -> install/<platform>-<arch>/<config>/
release assembly -> dist/
```

A standard install prefix contains:

```text
bin/                 executables
lib/                 runtime libraries
etc/lingtu/          immutable defaults
share/lingtu/        schemas and runtime assets
```

The repository does not commit root `bin/` or `lib/` directories. Production
services must not run from a developer `build/` tree.

Build the complete Linux native release input set and package it with:

```bash
LINGTU_DRIVER_BACKEND=go2 make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh vX.Y.Z dist
```

Use `LINGTU_DRIVER_BACKEND=doso` for Thunder. This does not turn the build into
a cross-compile; the selected build host still determines the architecture.

Windows uses its CMake install prefix, such as
`install/windows-x64/Release`. It is not passed through the Linux OTA script.

Component commands and platform prerequisites live in
[`scripts/build/README.md`](../scripts/build/README.md). Native planner kernels
are the Product default; ROS 2 Humble Desktop is optional for explicit
compatibility work.

## Repository orientation

| Path | What belongs there |
| --- | --- |
| `src/` | Runtime framework and functional owners |
| `sim/` | Simulation packages, sessions, runtime, adapters, and evaluation |
| `config/` | Products, env mappings, robots, devices, and architecture rules |
| `scripts/` | Stable build, deploy, and field entrypoints |
| `tools/` | Offline authoring, calibration, visualization, and validation |
| `tests/` | Cross-module and repository contracts |
| `web/` | Operator and documentation Web application |

Continue with [Architecture](./architecture.md) before changing ownership, or
[Development](./development.md) before editing code.
