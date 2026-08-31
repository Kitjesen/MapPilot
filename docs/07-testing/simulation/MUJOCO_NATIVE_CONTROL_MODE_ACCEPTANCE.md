# MuJoCo Native Control-Mode Acceptance

Status: current control-mode promotion contract as of 2026-08-11.

This entry has two explicit actions:

- `run` executes the harness configured for exactly one native endpoint mode,
  captures its raw report, and writes a runner artifact;
- `evaluate` validates that runner artifact and recomputes the gate result from
  the referenced raw report.

It does not accept an arbitrary JSON evidence summary. In particular,
hand-written fields such as `stop_proven: true` or
`forbidden_mode_rejection_proven: true` are not promotion evidence.

## Execute one mode

```powershell
$env:PYTHONPATH='src;.'
python sim/scripts/mujoco/native_control_mode_acceptance.py `
  --control-mode autonomy `
  --action run `
  --artifact-dir artifacts/mujoco_native_control_modes/autonomy `
  --json-out artifacts/mujoco_native_control_modes/autonomy/acceptance.json
```

`--control-mode` is required and accepts one value only:

```text
autonomy | teleop | teleop_avoid
```

To re-evaluate an artifact without re-running the harness:

```powershell
python sim/scripts/mujoco/native_control_mode_acceptance.py `
  --control-mode autonomy `
  --action evaluate `
  --runner-artifact artifacts/mujoco_native_control_modes/autonomy/runner_artifact.json `
  --json-out artifacts/mujoco_native_control_modes/autonomy/recheck.json
```

The mode contract is
`config/runtime_graph/acceptance/mujoco_native_control_mode_acceptance.json`.
It declares the exact endpoint mode, command kind, required and forbidden
processes, native inputs, quality thresholds, and executable harness adapter.

## What is executable today

| Mode | Configured executable | Real coverage today | Promotion result today |
| --- | --- | --- | --- |
| `autonomy` | `native_navigation_acceptance.py --mode motion` | Real MuJoCo sensor/policy process plus native SLAM, traversability, navigation, typed Goal/ACK, final DDS command tap, motion, and cleanup | Fails closed: the reused harness does not yet run the forbidden-Teleop probe, native Stop/no-resume probe, or provide long-run ATE/scale/map-match observations |
| `teleop` | `teleop_native_acceptance.py` | Native typed operator claim/sample/hold/release, simulated physical driver bridge, Driver ACK, forbidden Goal probe, stop, cleanup, and zero barrier in MuJoCo | A dated diagnostic-component PASS exists for 2026-08-10; still not a Brainstem or Sunrise field pass |
| `teleop_avoid` | `teleop_avoid_native_acceptance.py` | Product RunPlan resolves the assisted native chain with mapping SLAM, mapd, standalone traversability, navd, host, and simulated driver bridge after the local native endpoint artifacts are built | Runner exists, but no current accepted Product report is recorded; the latest local attempts remain evidence of wiring/fail-closed behavior, not promotion |

This matrix is intentional. A missing product harness is reported as a blocker;
the evaluator never converts a mirror test or a hand-authored summary into a
passing product gate.

## Runner artifact

`run` writes `runner_artifact.json` containing:

- producer schema, control mode, run ID, and start/finish timestamps;
- exact harness command and exit code;
- a run-scoped harness directory;
- raw harness report path and schema;
- normalized observations and runner blockers.

`evaluate` reopens the raw report, checks its schema and run-scoped paths, and
recomputes normalized observations. It does not trust a hand-written second
summary.
Any nonzero harness exit code also invalidates promotion, even if a report file
was produced.

## Three independent gate layers

| Gate | Question | Required observations |
| --- | --- | --- |
| `control_chain` | Did the selected native mode receive and ACK its typed command, reject the mutually exclusive command kind, execute native Stop, prove no old motion resumed, and alone publish final `rt/nav/cmd_vel`? | request/ACK event records, forbidden-kind rejection event, Stop event plus post-Stop zero window, C++ DDS tap |
| `product_integration` | Did the exact process set use native `navd` as the only planner/final command owner, load the ThunderV4 policy, move MuJoCo, and clean up? | launched process inventory, command source, policy report, motion observations, cleanup report |
| `slam_map_quality` | Is localization/map evidence good enough for the selected function? | health/cloud/traversability freshness plus the mode-specific localization metrics |

All applicable layers and provenance must pass for `promotion_eligible=true`.
Passing the command chain never implies SLAM/map quality passed.

Mode applicability:

| Mode | Control chain | Product integration | SLAM/map quality |
| --- | ---: | ---: | --- |
| `teleop` | required | required; SLAM/traversability forbidden | `not_applicable` |
| `teleop_avoid` | required | required | tracking + fresh health/cloud/traversability + map pose error |
| `autonomy` | required | required | tracking + ATE + trajectory scale + near-field map match |

## Quality thresholds

| Metric | Gate |
| --- | ---: |
| Autonomy ATE RMSE | `<= 0.30 m` |
| Autonomy trajectory scale ratio | `0.80 .. 1.20` |
| Autonomy near-field map match | `>= 0.80` |
| Teleop-avoid map XY error | `<= 0.75 m` |

The known long autonomy run has ATE RMSE `0.268 m`, but still fails product
quality because its trajectory scale ratio is `6.22` and near-field match is
`0.708`.

## Remaining live scenarios required for promotion

### `teleop`

- valid typed Teleop/ACK and velocity clamp;
- source-stamp stale/future rejection and command timeout to zero;
- native Stop and Cancel with a measured post-command zero/no-resume window;
- Goal rejection;
- MuJoCo policy motion with no SLAM or traversability process and with `navd`
  as the only final command owner;
- owned-process cleanup.

### `teleop_avoid`

- the same native DDS command/Stop/motion/cleanup observations as teleop;
- free, slow-obstacle, stop-obstacle, side/height, and terrain-cost cases;
- localization health, odometry/TF, registered-cloud, and traversability
  dropout/recovery sequences measured through the native endpoint;
- Goal rejection and map-pose error.

The executable path is `python -m sim.scripts.mujoco.product_acceptance` with
an exact `--run-plan`, the teleop-avoid `--runner` and `--manifest`, and an
isolated `--state-root`. The dispatcher verifies the RunPlan, owns the complete
ProductControl switch/stop transaction, and calls
`teleop_avoid_native_acceptance.py` as an attached scenario adapter. That
adapter is not a standalone runner. The former Python geometry/config mirror
was removed because it duplicated native safety without exercising the Product
chain.

### `autonomy`

- rejected Teleop and rejected legacy global path observations;
- native Cancel and Stop/no-resume observations;
- localization-dropout fail-closed and recovery observations;
- XY arrival plus final-yaw alignment;
- a runner-produced long-run quality report containing ATE, trajectory scale,
  and near-field map-match metrics.
