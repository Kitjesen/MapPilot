# Thunder V4 CTS MoE v39

`thunderv4_cts_moe@1.0.0` is the only current Thunder controller package. The
robot package's `defaults.controller` selects it; ordinary and stair sessions
inherit that default. All use `policy/policy_cts_moe_v39_61128.onnx` with
`policy/policy_v39_manifest.json` and the `thunderv4_cts_moe` adapter plugin.
The `stairs_up` and `stairs_down` profile names are aliases of that same model
for navigation requests. A YAML anchor declares the artifact and manifest once;
these names do not select a different locomotion policy.

The student accepts `history: N x 530` and `single_obs: N x 53`, and returns
`actions: N x 16`. Its ten-frame history is grouped by observation term, oldest
frame first. The MuJoCo engine selects the runner using the ONNX metadata tag
`policy_contract=thunder_cts_moe_v1`, then reads the embedded runtime and actuator
contracts. Do not substitute the older 53-value or 285-value policy runner.

The compatibility MuJoCo feeder and plan-driven controller provider both use
the CTS observation/runtime contract. Input compatibility alone does not
qualify complete native navigation or physical robot behavior.

The model comes from the supplied `thunder_cts_moe_v39_model61128` bundle:
`model_61128.pt` / `student_61129.pt`, exported as the static 1 x 530 / 1 x 53
ONNX in `policy/policy_cts_moe_v39_61128.onnx`. The file is a copy of the
supplied ONNX with its deployment contract added as metadata; its graph and
weights are unchanged. `policy/v39_61128_deployment_contract.json` preserves
the supplied contract. The adapter comparison against the bundle runtime covered
40 successive randomized observations; see
`build/cts-v39-model61128-20261027/parity.json`. This is not a physical or
hardware qualification. The training actuator's random 0–4-step delay is not
simulated.

For the twelve-step showcase with its original ground friction,
the 0.35 m/s direct locomotion probe did not finish either direction: v39
advanced about 0.53 m up and 0.53 m down in 120 s, with truth heading feedback.
The supplied bundle's own reference program moves on its different friction
and stair model. In an 8 s drive on the *same* LingTu scene, changing only the
ground friction from `1.0 0.5 0.5` to the bundle's training values
`1.8 0.03 0.0002` changed forward displacement from 0.18 m to 3.54 m. Keeping
the original sliding coefficient while using the bundle's torsional and rolling
coefficients (`1.0 0.03 0.0002`) produced 3.34 m. The showcase now uses the
latter values; they are simulation assumptions, not measured field friction.

Historical evidence: with the corrected showcase, handoff probes reached both exits and changed
4998 → v39 at entry and v39 → 4998 at exit without editing the physical pose.
Those runs used MuJoCo truth heading feedback rather than navigation or SLAM;
see `build/cts-v39-model61128-20261027/stair-corrected-handoff/`. They do not
validate the current single-model runtime or model switching on a physical robot.

The earlier `student_80708.pt` candidate and its TorchScript/ONNX conversion
evidence remain in `policy/`, but it is no longer selected by this package.
Its 2026-10-12 fixed twelve-step physical probes did **not** pass complete
traversal:

| Command | Direction | Drive duration | X progress | Result |
| --- | --- | --- | --- | --- |
| 0.15 m/s | Up | 120 s | 0.121 m | Remained on the lower landing |
| 0.15 m/s | Down | 120 s | 0.121 m | Remained on the upper landing |
| 0.35 m/s | Up | 120 s | 2.703 m | Reached the first stair flight, not the exit |
| 0.35 m/s | Down | 120 s | 5.674 m | Traversed stair surfaces, not the exit |
| 0.35 m/s | Down | 240 s | 6.591 m | Ended at X=-0.691 m; exit requires X<=-0.700 m |

No non-foot world contact was observed in these probes. They used explicit
truth heading feedback, not navd or SLAM. Exit restoration was requested but
never reached, so it is not verified by these locomotion runs. The command
response diagnostic confirmed that SI commands reach the model unchanged and
its normalized wheel actions become the actual wheel velocity targets; low
speed response remains unresolved. Evidence is retained under
`build/cts-moe-20261012/`.

Run an isolated physical up/down comparison, keeping the current stair geometry:

```powershell
.venv/Scripts/python.exe -m sim.scripts.mujoco.stair_policy_probe `
  --policies cts_moe --directions up down --heading-hold `
  --speed 0.15 --duration 120 --output build/cts-moe-stairs
```

The probe's optional `--handoff` and `--exit-restore` modes are explicit legacy
comparisons against `4998`; they do not describe current session selection.
`--policies` also accepts an explicit ONNX path. Results include the loaded
runner, actual policy-switch events, contacts, attitude, and sampled trajectory.
The probe uses simulation truth for heading feedback; it is not a native
navigation or SLAM test.
