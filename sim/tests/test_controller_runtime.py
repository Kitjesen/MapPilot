# ruff: noqa: S101

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.control import (
    CommandSubmitResult,
    ControllerCommand,
    ControllerPlanError,
    ControllerRuntime,
    ControllerRuntimeError,
    ControllerState,
    DeterministicFakeAdapter,
    DeterministicFakePolicy,
    GenerationStamp,
    SafeStopReason,
    load_control_plan,
)

_ACTUATOR_CHANNELS = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
)
_CONTROL_PLAN: dict[str, Any] = {
    "schema": "lingtu.sim.control-plan.v1",
    "session_id": "controller-test",
    "env": "sim",
    "backends": {"physics": "mujoco", "visual": "unreal"},
    "controllers": [
        {
            "instance_id": "thunder_01",
            "controller_id": "thunder_01.thunderv4_locomotion",
            "package": {
                "id": "thunderv4_locomotion",
                "version": "1.0.0",
                "kind": "controller",
                "manifest": "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml",
            },
            "adapter": {
                "plugin": "quadruped_him",
                "abi": "lingtu.sim.controller-adapter.v1",
            },
            "policy": {
                "runtime": "torchscript",
                "artifact": "sim/packages/controllers/doso/thunder_v4/locomotion/policy/controller.pt",
                "manifest": "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_manifest.json",
            },
            "timing": {"inference_hz": 50, "low_level_hz": 500},
            "state_channels": [
                "joint_position",
                "joint_velocity",
                "base_angular_velocity",
                "projected_gravity",
            ],
            "command_channels": [
                "thunder_01.control.base_twist",
                "thunder_01.control.joint_torque",
            ],
            "actuator_channels": list(_ACTUATOR_CHANNELS),
        }
    ],
    "command_channels": [
        {
            "channel_id": "thunder_01.control.base_twist",
            "direction": "subscribe",
            "owner": "simulation",
            "source": "dds_base_twist/thunder_01",
            "transport": "typed_dds",
            "message_type": "lingtu.dds.FinalVelocityCommand",
            "command_type": "base_twist",
            "target": "base",
        },
        {
            "channel_id": "thunder_01.control.joint_torque",
            "direction": "publish",
            "owner": "physics",
            "source": "thunder_01.thunderv4_locomotion",
            "transport": "in_process",
            "message_type": "lingtu.sim.joint-torque.v1",
            "command_type": "joint_torque",
            "target": "actuators",
        },
    ],
    "stale_stop_authority": {
        "owner": "simulation",
        "hardware_forwarding": False,
        "safe_stop_on_stale": True,
        "stale_timeout_ms": 100,
    },
}


@pytest.fixture
def control_plan_path(tmp_path: Path) -> Path:
    path = tmp_path / "control.plan.json"
    path.write_text(json.dumps(_CONTROL_PLAN), encoding="utf-8")
    return path


def test_control_plan_preserves_stable_actuator_channel_mapping(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)

    controller = plan.controller("thunder_01.thunderv4_locomotion")

    assert controller.actuators.channels == _ACTUATOR_CHANNELS
    assert controller.actuators.index_of("FR_hip_joint") == 0
    assert controller.actuators.index_of("RL_foot_joint") == 15


def test_control_plan_retains_adapter_and_unloaded_policy_selection(
    control_plan_path: Path,
) -> None:
    controller = load_control_plan(control_plan_path).controller("thunder_01.thunderv4_locomotion")

    assert controller.adapter.plugin == "quadruped_him"
    assert controller.adapter.abi == "lingtu.sim.controller-adapter.v1"
    assert controller.policy.runtime == "torchscript"
    assert controller.policy.artifact == "sim/packages/controllers/doso/thunder_v4/locomotion/policy/controller.pt"


def test_control_plan_rejects_malformed_session_identity(
    tmp_path: Path,
    control_plan_path: Path,
) -> None:
    raw = json.loads(control_plan_path.read_text(encoding="utf-8"))
    raw["session_id"] = ""
    invalid_plan = tmp_path / "control.plan.json"
    invalid_plan.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ControllerPlanError, match="session_id"):
        load_control_plan(invalid_plan)


def test_control_plan_rejects_duplicate_actuator_channels(tmp_path: Path) -> None:
    raw = deepcopy(_CONTROL_PLAN)
    channels = raw["controllers"][0]["actuator_channels"]
    channels[-1] = channels[0]
    invalid_plan = tmp_path / "duplicate-actuator.plan.json"
    invalid_plan.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ControllerPlanError, match=r"actuator_channels.*unique"):
        load_control_plan(invalid_plan)


def test_control_plan_rejects_unknown_fields(tmp_path: Path) -> None:
    raw = deepcopy(_CONTROL_PLAN)
    raw["robot_model"] = "thunderv4"
    invalid_plan = tmp_path / "unknown-field.plan.json"
    invalid_plan.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ControllerPlanError, match=r"extra=.*robot_model"):
        load_control_plan(invalid_plan)


def test_runtime_uses_sim_time_for_fixed_rate_inference_and_actuation(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    controller = plan.controller("thunder_01.thunderv4_locomotion")
    generation = GenerationStamp(model_generation=4, reset_generation=2)
    action = {channel: float(index + 1) for index, channel in enumerate(reversed(controller.actuators.channels))}
    runtime = ControllerRuntime(
        plan=plan,
        controller_id=controller.controller_id,
        generation=generation,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy(action),
    )
    accepted = runtime.submit_command(
        ControllerCommand(
            channel_id="thunder_01.control.base_twist",
            instance_id="thunder_01",
            generation=generation,
            sequence=1,
            apply_time_ns=0,
            payload={"linear_x": 0.5, "angular_z": -0.25},
        )
    )
    assert accepted is CommandSubmitResult.ACCEPTED

    steps = [
        runtime.step(
            ControllerState(
                session_id=plan.session_id,
                instance_id="thunder_01",
                generation=generation,
                sequence=millisecond,
                sim_time_ns=millisecond * 1_000_000,
                channels={
                    "joint_position": (),
                    "joint_velocity": (),
                    "base_angular_velocity": (0.0, 0.0, 0.0),
                    "projected_gravity": (0.0, 0.0, -1.0),
                },
            )
        )
        for millisecond in range(23)
    ]

    assert [step.sim_time_ns for step in steps if step.inference_ran] == [
        0,
        20_000_000,
    ]
    actuator_commands = [step.actuator_command for step in steps if step.actuator_command is not None]
    assert [command.apply_time_ns for command in actuator_commands] == [
        millisecond * 1_000_000 for millisecond in range(0, 23, 2)
    ]
    assert all(command.channels == controller.actuators.channels for command in actuator_commands)
    assert actuator_commands[0].values == tuple(action[channel] for channel in controller.actuators.channels)


@pytest.mark.parametrize(
    ("generation", "expected"),
    [
        (
            GenerationStamp(3, 2),
            CommandSubmitResult.REJECTED_STALE_MODEL_GENERATION,
        ),
        (
            GenerationStamp(5, 2),
            CommandSubmitResult.REJECTED_FUTURE_MODEL_GENERATION,
        ),
        (
            GenerationStamp(4, 1),
            CommandSubmitResult.REJECTED_STALE_RESET_GENERATION,
        ),
        (
            GenerationStamp(4, 3),
            CommandSubmitResult.REJECTED_FUTURE_RESET_GENERATION,
        ),
    ],
)
def test_runtime_rejects_commands_outside_the_active_generation(
    generation: GenerationStamp,
    expected: CommandSubmitResult,
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    active = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=active,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((0.0,) * 16),
    )

    result = runtime.submit_command(
        ControllerCommand(
            channel_id="thunder_01.control.base_twist",
            instance_id="thunder_01",
            generation=generation,
            sequence=1,
            apply_time_ns=0,
            payload={},
        )
    )

    assert result is expected


def test_runtime_rejects_nonmonotonic_command_sequence_or_apply_time(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    generation = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=generation,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((0.0,) * 16),
    )

    def command(sequence: int, apply_time_ns: int) -> ControllerCommand:
        return ControllerCommand(
            channel_id="thunder_01.control.base_twist",
            instance_id="thunder_01",
            generation=generation,
            sequence=sequence,
            apply_time_ns=apply_time_ns,
            payload={},
        )

    assert runtime.submit_command(command(1, 20)) is CommandSubmitResult.ACCEPTED
    assert runtime.submit_command(command(1, 21)) is CommandSubmitResult.REJECTED_OUT_OF_ORDER
    assert runtime.submit_command(command(2, 19)) is CommandSubmitResult.REJECTED_OUT_OF_ORDER


@pytest.mark.parametrize(
    "generation",
    [
        GenerationStamp(3, 2),
        GenerationStamp(5, 2),
        GenerationStamp(4, 1),
        GenerationStamp(4, 3),
    ],
)
def test_runtime_rejects_state_outside_the_active_generation(
    generation: GenerationStamp,
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=GenerationStamp(4, 2),
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((0.0,) * 16),
    )

    with pytest.raises(ControllerRuntimeError, match="model/reset generation"):
        runtime.step(
            ControllerState(
                session_id=plan.session_id,
                instance_id="thunder_01",
                generation=generation,
                sequence=0,
                sim_time_ns=0,
                channels={
                    "joint_position": (),
                    "joint_velocity": (),
                    "base_angular_velocity": (0.0, 0.0, 0.0),
                    "projected_gravity": (0.0, 0.0, -1.0),
                },
            )
        )


def test_generation_switch_invalidates_prior_commands_and_policy_action(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    initial = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=initial,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((1.0,) * 16),
    )
    assert (
        runtime.submit_command(
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=initial,
                sequence=1,
                apply_time_ns=0,
                payload={},
            )
        )
        is CommandSubmitResult.ACCEPTED
    )
    initial_output = runtime.step(
        ControllerState(
            session_id=plan.session_id,
            instance_id="thunder_01",
            generation=initial,
            sequence=0,
            sim_time_ns=0,
            channels={
                "joint_position": (),
                "joint_velocity": (),
                "base_angular_velocity": (0.0, 0.0, 0.0),
                "projected_gravity": (0.0, 0.0, -1.0),
            },
        )
    ).actuator_command
    assert initial_output is not None and not initial_output.safe_stop

    reset = GenerationStamp(4, 3)
    runtime.set_generation(reset, start_time_ns=0)
    assert (
        runtime.submit_command(
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=initial,
                sequence=2,
                apply_time_ns=0,
                payload={},
            )
        )
        is CommandSubmitResult.REJECTED_STALE_RESET_GENERATION
    )
    reset_output = runtime.step(
        ControllerState(
            session_id=plan.session_id,
            instance_id="thunder_01",
            generation=reset,
            sequence=0,
            sim_time_ns=0,
            channels={
                "joint_position": (),
                "joint_velocity": (),
                "base_angular_velocity": (0.0, 0.0, 0.0),
                "projected_gravity": (0.0, 0.0, -1.0),
            },
        )
    ).actuator_command

    assert reset_output is not None
    assert reset_output.generation == reset
    assert reset_output.safe_stop_reason is SafeStopReason.NO_COMMAND
    assert reset_output.values == (0.0,) * 16
    assert (
        runtime.submit_command(
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=reset,
                sequence=1,
                apply_time_ns=0,
                payload={},
            )
        )
        is CommandSubmitResult.ACCEPTED
    )


def test_runtime_hold_clears_the_active_command_and_policy_action(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    generation = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=generation,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((1.0,) * 16),
    )
    command = ControllerCommand(
        channel_id="thunder_01.control.base_twist",
        instance_id="thunder_01",
        generation=generation,
        sequence=1,
        apply_time_ns=0,
        payload={},
    )
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED

    active = runtime.step(
        ControllerState(
            session_id=plan.session_id,
            instance_id="thunder_01",
            generation=generation,
            sequence=0,
            sim_time_ns=0,
            channels={
                "joint_position": (),
                "joint_velocity": (),
                "base_angular_velocity": (0.0, 0.0, 0.0),
                "projected_gravity": (0.0, 0.0, -1.0),
            },
        )
    ).actuator_command
    assert active is not None and not active.safe_stop

    runtime.hold()

    held = runtime.step(
        ControllerState(
            session_id=plan.session_id,
            instance_id="thunder_01",
            generation=generation,
            sequence=1,
            sim_time_ns=2_000_000,
            channels={
                "joint_position": (),
                "joint_velocity": (),
                "base_angular_velocity": (0.0, 0.0, 0.0),
                "projected_gravity": (0.0, 0.0, -1.0),
            },
        )
    ).actuator_command

    assert held is not None
    assert held.safe_stop_reason is SafeStopReason.NO_COMMAND
    assert held.values == (0.0,) * 16


def test_runtime_hold_requires_a_new_monotonic_command_to_resume(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    generation = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=generation,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((1.0,) * 16),
    )

    def command(sequence: int, apply_time_ns: int) -> ControllerCommand:
        return ControllerCommand(
            channel_id="thunder_01.control.base_twist",
            instance_id="thunder_01",
            generation=generation,
            sequence=sequence,
            apply_time_ns=apply_time_ns,
            payload={},
        )

    accepted = command(7, 20_000_000)
    assert runtime.submit_command(accepted) is CommandSubmitResult.ACCEPTED
    runtime.hold()

    assert runtime.submit_command(accepted) is CommandSubmitResult.REJECTED_OUT_OF_ORDER
    assert runtime.submit_command(command(8, 20_000_000)) is CommandSubmitResult.ACCEPTED


def test_invalid_generation_schedule_does_not_partially_switch_runtime(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    initial = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=initial,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((1.0,) * 16),
    )

    with pytest.raises(ValueError, match="start_time_ns"):
        runtime.set_generation(GenerationStamp(4, 3), start_time_ns=-1)

    assert runtime.generation == initial


def test_stale_command_safe_stops_until_fresh_inference_runs(
    control_plan_path: Path,
) -> None:
    plan = load_control_plan(control_plan_path)
    generation = GenerationStamp(4, 2)
    runtime = ControllerRuntime(
        plan=plan,
        controller_id="thunder_01.thunderv4_locomotion",
        generation=generation,
        adapter=DeterministicFakeAdapter(),
        policy=DeterministicFakePolicy((2.0,) * 16),
    )

    def state(sequence: int, sim_time_ns: int) -> ControllerState:
        return ControllerState(
            session_id=plan.session_id,
            instance_id="thunder_01",
            generation=generation,
            sequence=sequence,
            sim_time_ns=sim_time_ns,
            channels={
                "joint_position": (),
                "joint_velocity": (),
                "base_angular_velocity": (0.0, 0.0, 0.0),
                "projected_gravity": (0.0, 0.0, -1.0),
            },
        )

    assert (
        runtime.submit_command(
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=generation,
                sequence=1,
                apply_time_ns=0,
                payload={},
            )
        )
        is CommandSubmitResult.ACCEPTED
    )

    before_deadline = runtime.step(state(0, 98_000_000)).actuator_command
    assert before_deadline is not None and not before_deadline.safe_stop

    stale = runtime.step(state(1, 100_000_000)).actuator_command
    assert stale is not None
    assert stale.safe_stop_reason is SafeStopReason.STALE_COMMAND
    assert stale.values == (0.0,) * 16

    assert (
        runtime.submit_command(
            ControllerCommand(
                channel_id="thunder_01.control.base_twist",
                instance_id="thunder_01",
                generation=generation,
                sequence=2,
                apply_time_ns=100_000_000,
                payload={},
            )
        )
        is CommandSubmitResult.ACCEPTED
    )
    waiting = runtime.step(state(2, 102_000_000)).actuator_command
    assert waiting is not None
    assert waiting.safe_stop_reason is SafeStopReason.WAITING_FOR_INFERENCE

    recovered = runtime.step(state(3, 120_000_000)).actuator_command
    assert recovered is not None
    assert not recovered.safe_stop
    assert recovered.values == (2.0,) * 16
