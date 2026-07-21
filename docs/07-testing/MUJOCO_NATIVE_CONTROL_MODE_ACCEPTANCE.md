# MuJoCo Native Control-Mode Acceptance

Status: current control-mode promotion contract as of 2026-07-18.

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
`config/runtime_graph/endpoints/mujoco_native_control_mode_acceptance.json`.
It declares the exact endpoint mode, command kind, required and forbidden
processes, native inputs, quality thresholds, and executable harness adapter.

## What is executable today

| Mode | Configured executable | Real coverage today | Promotion result today |
| --- | --- | --- | --- |
| `autonomy` | `native_navigation_acceptance.py --mode motion` | Real MuJoCo sensor/policy process plus native SLAM, traversability, navigation, typed Goal/ACK, final DDS command tap, motion, and cleanup | Fails closed: the reused harness does not yet run the forbidden-Teleop probe, native Stop/no-resume probe, or provide long-run ATE/scale/map-match observations |
| `teleop` | none | No reusable full MuJoCo + native DDS teleop harness exists in the repository | `runner_unavailable:full_mujoco_native_teleop_harness_not_implemented` |
| `teleop_avoid` | `teleop_avoid_gate.py --strict` | Executable MuJoCo geometry/config mirror only | Supplemental evidence only; fails the native DDS control-chain, product-integration, and SLAM/map gates |

This matrix is intentional. A missing product harness is reported as a blocker;
the evaluator never converts a mirror test or a hand-authored summary into a
passing product gate.

## Runner artifact and provenance

`run` writes `runner_artifact.json` containing:

- producer schema, control mode, UUID, and start/finish timestamps;
- SHA-256 of the exact mode manifest;
- exact harness command and exit code;
- a UUID-scoped `runs/<run-id>/harness` directory so a timeout or crash cannot
  reuse a previous run's `report.json`;
- expected harness script path and SHA-256;
- raw harness report path, schema, and SHA-256;
- normalized observations and their SHA-256.

`evaluate` reopens the raw report, checks these paths, schemas, and digests, and
recomputes normalized observations. The observations stored in the runner
artifact are compared with the recomputed observations, not trusted as a
second summary. Any edit to the source report, script, manifest, or normalized
observations invalidates the artifact.
Any nonzero harness exit code also invalidates promotion, even if a report file
was produced.

This is integrity/provenance validation for the local acceptance workflow, not
a cryptographic signature against an attacker who already has arbitrary write
access to the workspace.

## Three independent gate layers

| Gate | Question | Required observations |
| --- | --- | --- |
| `control_chain` | Did the selected native mode receive and ACK its typed command, reject the mutually exclusive command kind, execute native Stop, prove no old motion resumed, and alone publish final `rt/nav/cmd_vel`? | request/ACK event records, forbidden-kind rejection event, Stop event plus post-Stop zero window, C++ DDS tap |
| `product_integration` | Did the exact process set run without Python CmdVelMux/planners, load the ThunderV4 policy, move MuJoCo, and clean up? | launched process inventory, command source, policy report, motion observations, cleanup report |
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
- MuJoCo policy motion with no SLAM, traversability, Python CmdVelMux, or
  Python planner process;
- owned-process cleanup.

### `teleop_avoid`

- the same native DDS command/Stop/motion/cleanup observations as teleop;
- free, slow-obstacle, stop-obstacle, side/height, and terrain-cost cases;
- localization health, odometry/TF, registered-cloud, and traversability
  dropout/recovery sequences measured through the native endpoint;
- Goal rejection and map-pose error.

The current `teleop_avoid_gate.py` covers only the geometry/config mirror.

### `autonomy`

- rejected Teleop and rejected legacy global path observations;
- native Cancel and Stop/no-resume observations;
- localization-dropout fail-closed and recovery observations;
- XY arrival plus final-yaw alignment;
- a runner-produced long-run quality report containing ATE, trajectory scale,
  and near-field map-match metrics.
