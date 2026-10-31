# Thunder V4 controller selection

To locate the actual model import, use
[`sim.runtime.control.policy_loader.load_policy()`](../../../../runtime/control/policy_loader.py).
Both current CTS execution paths call it. The
[policy-loading guide](../../../../runtime/control/README.md) separates this
loading entry from the configuration selection below.

The only current selectable package is `thunderv4_cts_moe@1.0.0`, declared in
[`cts_moe/controller.package.yaml`](cts_moe/controller.package.yaml). The Thunder
robot package selects it through `defaults.controller`; current ordinary and
stair sessions inherit that default. Sessions that explicitly set
`controller: null` still disable control.

The selected model is `cts_moe/policy/policy_cts_moe_v39_61128.onnx`, described by
`cts_moe/policy/policy_v39_manifest.json`, with adapter plugin
`thunderv4_cts_moe`. It accepts `history[1,530]` and `single_obs[1,53]`, produces
16 actions, and runs inference at 50 Hz with a 200 Hz low-level loop. History
contains ten frames grouped by observation term, oldest first. The ordinary
policy and navigation's `stairs_up` / `stairs_down` aliases all resolve to this
same artifact; they do not switch between flat and stair models.

The selection chain is RobotPackage `defaults.controller` -> this controller's
`policy` and `adapter` -> Catalog `control_plan` -> the published Product RunPlan.
The compatibility MuJoCo feeder and plan-driven `thunderv4_cts_moe` controller
both reuse `CtsPolicy`, including its observation history and
embedded runtime/actuator contracts. Formal runtimes consume the RunPlan's
resolved values and do not independently resolve the robot default.

Development tools that need the current default outside a running Product can
call `sim.catalog.resolve_robot_default_policy(repo_root, robot_ref)`. It returns
resolved `artifact` and `manifest` paths from the robot and controller packages;
use it instead of copying a default ONNX path into another tool. This helper is
for development defaults, not a second runtime selection path.

The source chain is the user-supplied `thunder_cts_moe_v39_model61128` bundle,
`model_61128.pt` / `student_61129.pt`, its static-batch exported ONNX, then the
packaged ONNX with deployment metadata added without changing graph or weights.
The original deployment contract is retained in
`cts_moe/policy/v39_61128_deployment_contract.json`. See the
[CTS package notes](cts_moe/README.md) for parity and MuJoCo probe evidence.

`locomotion/policy/` retains the older 4998 and 1119 models and manifests.
`locomotion/keyboard.py` requires an explicit `--policy-path` for a legacy model;
it has no current default policy. That tool, `recovery_play.py`, the earlier CTS
candidate, and their recorded evidence remain for baseline comparisons and audit.
They are not alternative current package declarations. Historical model handoff
probes do not qualify the current single-model path. Complete native navigation
and real robot motion with this selection have not been qualified.
