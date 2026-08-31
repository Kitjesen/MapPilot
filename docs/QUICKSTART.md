# LingTu Quick Start

LingTu has one user-facing runtime model:

```text
Robot + env(real | sim) + Product
                 ↓
          switch / status / stop
```

Products are `teleop`, `teleop_avoid`, `map`, `explore`, `nav`, `tracking`, and
`inspection`. The resolved process plan is internal; users do not write it.
Robot and Env are selected independently. A combination starts only when its
concrete physical configuration or simulation session is present.

## Setup

From the repository root:

```bash
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
```

The installed `lingtu` command and `python -m lingtu.control` use the same
implementation. `scripts/lingtu` is the thin robot-side adapter.

## Simulation

Inspect a Product without starting it:

```bash
uv run --locked python -m lingtu.control switch teleop \
  --robot doso/thunder_v4 --env sim --dry-run --json
```

Start, inspect, and stop the Product:

```bash
uv run --locked python -m lingtu.control switch teleop \
  --robot doso/thunder_v4 --env sim
uv run --locked python -m lingtu.control status \
  --robot doso/thunder_v4 --env sim --json
uv run --locked python -m lingtu.control stop \
  --robot doso/thunder_v4 --env sim
```

MuJoCo component experiments use their explicit scripts; they are not a second
Product lifecycle:

```bash
uv run --locked python sim/scripts/mujoco/native_navigation_acceptance.py --help
uv run --locked python sim/scripts/mujoco/native_control_mode_acceptance.py --help
```

## Field Robot

Run read-only status first on the robot:

```bash
python -m lingtu.control status --robot unitree/go2 --env real --json
# or
bash scripts/lingtu --robot unitree/go2 --env real status --json
```

A field switch may start native services and permit motion. Use the exact
Product needed for the supervised task:

```bash
python -m lingtu.control switch map \
  --robot unitree/go2 --env real

python -m lingtu.control switch nav \
  --robot unitree/go2 --env real --map building_a

python -m lingtu.control stop \
  --robot unitree/go2 --env real
```

The examples above use the combinations with current runnable evidence:
Thunder V4 with `sim`, and Go2 with `real`. Go2 still needs a Go2 simulation
session and robot assets; Thunder V4 still needs an adjacent `robot.yaml` for
field use. These are missing inputs, not model-level Env restrictions.

## Keyboard Teleop

Start `teleop` for direct operator control or `teleop_avoid` for live local
avoidance, then open the Web scene's **遥控** panel and click **连接**:

```bash
python -m lingtu.control switch teleop \
  --robot unitree/go2 --env real
```

Hold `W/S` for forward/backward, `A/D` for lateral motion, and `Q/E` to turn.
Hold `Shift` for 40% precision. Release the last movement key or press `Space`
or **保持** to publish hold; **停车** requests the native stop path. Active Web
samples publish at 50 Hz and the native navigation/safety loop runs at 20 Hz.

In `teleop_avoid` only, hold `M` or **按住脱困** together with a movement key
to issue a momentary manual escape command when localization is unavailable or
CMU has no path. Local planning and obstacle/traversability checks are disabled
only while held. Release it to resume CMU on the next sample. E-stop, deadman,
command limits, control-loop health, operator lease, and driver readiness are
never bypassed.

Use `lingtu-drive` only for a supervised single-direction check of at most five
seconds; it is not the continuous keyboard controller. See
[Operator Teleop](./04-deployment/operator_drive.md).

## What Each Result Proves

- `--dry-run` proves Product resolution only.
- `status` reports the current lifecycle state; it does not authorize motion.
- A running process is not the same as DDS/readiness evidence.
- Simulation evidence does not prove field behavior.
- Stopping a motion Product still requires the runtime's real stop/zero
  confirmation.

For deployment details see [Robot-side CLI](./04-deployment/lingtu_cli.md). For
validation levels see [Testing](./07-testing/README.md).
