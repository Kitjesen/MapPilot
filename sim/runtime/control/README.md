# Simulation control

Shared scheduling and session code lives in this directory. Thunder-specific
controllers and policies live together in [`thunder/`](thunder/). Other robot
adapters remain selected by the compiled control plan and the existing factory.

## File guide

| File | Responsibility |
| --- | --- |
| [contracts.py](contracts.py) | State, command, result and adapter interfaces. |
| [plan.py](plan.py) | Load the compiled control plan and retain controller settings and actuator order. |
| [runtime.py](runtime.py) | Schedule one controller's inference and actuator output using simulation time. |
| [session.py](session.py) | Connect all controllers in a session and propagate commands, holds and resets. |
| [physics_bridge.py](physics_bridge.py) | Bind actuator names, read physics snapshots and apply controller outputs. |
| [factory.py](factory.py) | Construct the implementation named by the compiled controller declaration. |
| [policy_loader.py](policy_loader.py) | Open a model once and return its policy implementation. |
| [fake.py](fake.py) | Deterministic stand-ins for tests and zero-output bring-up. |
| [__init__.py](__init__.py) | Existing package-level imports. |

| Thunder file | Responsibility |
| --- | --- |
| [cts_controller.py](thunder/cts_controller.py) | Current CTS adapter: runtime state and commands to observations, actions to named torques. |
| [cts_policy.py](thunder/cts_policy.py) | CTS observation history, model inference, action bounds and actuator limits. |
| [joints.py](thunder/joints.py) | MuJoCo and Thunder policy joint-order mappings. |
| [legacy_controller.py](thunder/legacy_controller.py) | Earlier plan-driven adapters and policy backends; also holds Thunder conversions reused by CTS. |
| [legacy_policy.py](thunder/legacy_policy.py) | Historical history/frame ONNX and TorchScript runners for explicit comparisons. |
| [flat.py](thunder/flat.py) | The older flat model's observation/action rules and velocity feedback. |

`legacy_controller.py` is still imported by the current CTS adapter for shared
Thunder conversions. Its location does not make it removable. Historical model
implementations and model assets remain available for their existing probes.

## Where to start

1. Read `contracts.py` and `plan.py` for inputs and outputs.
2. Read `runtime.py` for one control step, then `session.py` and
   `physics_bridge.py` for session and physics wiring.
3. Follow `factory.py` to `thunder/cts_controller.py` and `thunder/cts_policy.py`
   for the current Thunder implementation.

The normal execution flow is:

```text
ControlPlan -> SessionControlRuntime -> factory -> controller and policy
Physics snapshot -> bridge -> ControllerRuntime.step()
                            -> observe -> infer -> actuate
                            -> ActuatorCommand -> bridge -> Physics
```

## Model-loading entry

Both the CTS adapter and the direct MuJoCo engine use the same entry:

```python
from sim.runtime.control.policy_loader import load_policy

policy = load_policy(str(policy_path), cpu_threads=1)
```

`policy_path` comes from the resolved control plan. ONNX loading creates one
session; tests can supply `session=` to reuse one. The current Thunder model is
declared in the
[controller package](../../packages/controllers/doso/thunder_v4/cts_moe/controller.package.yaml).
See the [selection guide](../../packages/controllers/doso/thunder_v4/README.md).

This package serves simulation. Real robot motion uses the selected native
driver and platform SDK; see the
[native field driver](../../../src/drivers/real/motion/README.md).

## Command refresh and recovery

The [bounded diagnostic runner](../coordinator/controlled_run.py) and
[motion recording](../coordinator/motion_recording.py) share `limit_refresh_steps()`
and treat `--refresh-steps` as an upper bound. After preparation they cap each batch using
the existing PhysicsPlan timestep, a 20 ms command interval (50 Hz), and the
ControlPlan command timeout if shorter. For a 5 ms step and 100 ms timeout, a
requested 25-step batch becomes six 4-step batches plus one step. Smaller requested
batches stay smaller. The compiled timeout is unchanged; input loss still expires
the command. If a physics step itself exceeds 20 ms, refresh occurs every step.

The current CTS model declares 50 Hz inference and 200 Hz low-level control.
Command refresh does not change either schedule. This choice matches the model
contract and the common 5 ms physics / 4-step policy cadence in
[Legged Gym](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py),
not a universal robot requirement. UE operator input also targets 50 Hz; key changes
and release publish immediately. Actual input publication is limited by Slate ticks.

An explicit `hold()` immediately resets component memory, including when Physics
is paused before another control step can run. CTS also clears policy history and
its previous target on a safe stop. The next observation initializes the target
from measured joints. A fresh zero-velocity command remains normal inference and
retains history. These operations do not reload the model or reset schedule clocks.

## Live display snapshots

[`live_snapshot.py`](../coordinator/live_snapshot.py) encodes physics snapshots and
sends them to UE over nonblocking localhost UDP. Live visual sessions, recording
and replay reuse its publisher. This display path does not submit velocity commands.

Its standalone CLI uses zero-output controllers for visual bring-up. The standalone
loop waits until each frame's deadline before publishing, with the first frame
immediate; slow physics does not incur an extra wait. Its frame interval is
`steps_per_frame * timestep_s`. Interactive sessions instead use SessionHost's
20 ms display interval (50 Hz); changing the standalone loop does not change that.
