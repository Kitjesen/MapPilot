# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path
from typing import Any

from sim.runtime.control import (
    ActuatorCommand,
    ActuatorLayout,
    AdapterSpec,
    CommandChannelSpec,
    CommandSubmitResult,
    ControllerCommand,
    ControllerSpec,
    ControlPlan,
    DeterministicFakeAdapter,
    DeterministicFakePolicy,
    GenerationStamp,
    PolicySpec,
)
from sim.runtime.control.session import SessionControlRuntime


class _ActuatorHost:
    def bind_actuators(self, **binding: Any) -> dict[str, Any]:
        return {
            "event": "actuator_bound",
            **binding,
            "channel_count": len(binding["channels"]),
        }

    def apply_actuator_command(self, command: ActuatorCommand) -> dict[str, Any]:
        return {
            "event": "actuator_command",
            "source_id": command.controller_id,
            "sequence": command.sequence,
            "result": "applied",
            "safe_stop": command.safe_stop,
            "safe_stop_reason": (
                command.safe_stop_reason.value
                if command.safe_stop_reason is not None
                else None
            ),
        }


def _plan() -> ControlPlan:
    return ControlPlan(
        session_id="a" * 64,
        controllers=(
            ControllerSpec(
                controller_id="robot.controller",
                instance_id="robot",
                adapter=AdapterSpec(
                    plugin="deterministic_fake",
                    abi="lingtu.sim.controller-adapter.v1",
                ),
                policy=PolicySpec(
                    runtime="fake",
                    artifact="unused",
                    manifest="unused",
                ),
                inference_hz=50.0,
                low_level_hz=500.0,
                state_channels=(
                    "joint_position",
                    "joint_velocity",
                    "base_angular_velocity",
                    "projected_gravity",
                ),
                command_channels=("robot.cmd_vel", "robot.joint_torque"),
                actuators=ActuatorLayout(("joint_a", "joint_b")),
            ),
        ),
        command_channels=(
            CommandChannelSpec(
                channel_id="robot.cmd_vel",
                direction="subscribe",
                owner="simulation",
                source="dds/robot",
                transport="typed_dds",
                message_type="lingtu.dds.FinalVelocityCommand",
                command_type="base_twist",
                target="base",
            ),
            CommandChannelSpec(
                channel_id="robot.joint_torque",
                direction="publish",
                owner="physics",
                source="robot.controller",
                transport="in_process",
                message_type="lingtu.sim.joint-torque.v1",
                command_type="joint_torque",
                target="actuators",
            ),
        ),
        stale_timeout_ns=100_000_000,
    )


def _snapshot(*, sequence: int, sim_time_ns: int) -> dict[str, Any]:
    return {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 5,
        "reset_generation": 2,
        "sequence": sequence,
        "physics_step": sequence,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "robot/base_link",
                "instance_id": "robot",
                "frame_id": "base_link",
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.0],
            }
        ],
        "joints": [
            {
                "stable_id": f"robot/{joint}",
                "instance_id": "robot",
                "position_rad": [0.0],
                "velocity_rps": [0.0],
            }
            for joint in ("joint_a", "joint_b")
        ],
        "actuators": [],
    }


def test_session_hold_clears_active_commands_for_prepared_controllers(
    tmp_path: Path,
) -> None:
    plan = _plan()
    session = SessionControlRuntime(
        plan=plan,
        attach_roots={"robot": "base_link"},
        physics_host=_ActuatorHost(),
        component_factory=lambda _controller, _repo_root: (
            DeterministicFakeAdapter(),
            DeterministicFakePolicy((1.0, -1.0)),
        ),
        repo_root=tmp_path,
    )
    session.prepare(
        {
            "event": "ready",
            "model_generation": 5,
            "reset_generation": 2,
            "sim_time_ns": 0,
        }
    )
    generation = GenerationStamp(5, 2)
    assert (
        session.submit_command(
            "robot.controller",
            ControllerCommand(
                channel_id="robot.cmd_vel",
                instance_id="robot",
                generation=generation,
                sequence=1,
                apply_time_ns=0,
                payload={},
            ),
        )
        is CommandSubmitResult.ACCEPTED
    )
    assert session.step(_snapshot(sequence=0, sim_time_ns=0))[0]["safe_stop"] is False

    session.hold()

    held = session.step(_snapshot(sequence=1, sim_time_ns=2_000_000))[0]
    assert held["safe_stop"] is True
    assert held["safe_stop_reason"] == "no_command"
