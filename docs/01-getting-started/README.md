# Get Started

**Status:** Current
**Audience:** Developers and operators
**Runs on:** Development host, S100P, and simulation host

LingTu exposes Products in exactly two environments: `real` and `sim`.

```text
Robot + env + Product -> ProductControl -> switch / status / stop
```

## Prepare the checkout

```bash
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
uv run --locked python -m pytest src/runtime/tests/test_runtime_graph_contract.py -q
```

## Resolve a simulation Product

Dry-run first; this does not start processes:

```bash
uv run --locked python -m lingtu.control switch teleop \
  --robot doso/thunder_v4 --env sim --dry-run --json
```

For component-level MuJoCo work, use the current native acceptance scripts:

```bash
uv run --locked python sim/scripts/mujoco/native_navigation_acceptance.py --help
uv run --locked python sim/scripts/mujoco/native_control_mode_acceptance.py --help
```

Component scripts are not a second lifecycle and their results are not Product
or field evidence.

## Inspect a robot before motion

On the target controller:

```bash
python -m lingtu.control status --robot unitree/go2 --env real --json
PYTHONPATH=src python -m diagnostics.field.doctor --json --strict
```

Use `switch` only for the supervised task being performed, then call `stop`:

```bash
python -m lingtu.control switch nav \
  --robot unitree/go2 --env real --map building_a
python -m lingtu.control stop --robot unitree/go2 --env real
```

Continue with the [Quick Start](../QUICKSTART.md), [Build Guide](./BUILD_GUIDE.md),
and [Robot-side CLI](../04-deployment/lingtu_cli.md).
