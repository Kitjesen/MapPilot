from __future__ import annotations

import math
from typing import Any

import pytest

from sim.runtime.control import (
    ActuatorCommand,
    ActuatorLayout,
    AdapterSpec,
    CommandChannelSpec,
    ControlPlan,
    ControllerPhysicsBridge,
    ControllerRuntimeError,
    ControllerSpec,
    GenerationStamp,
    PolicySpec,
)


class FakeActuatorHost:
    def __init__(self) -> None:
        self.bindings: list[dict[str, Any]] = []
        self.commands: list[ActuatorCommand] = []
        self.result = "applied"

    def bind_actuators(self, **binding: Any) -> dict[str, Any]:
        self.bindings.append(binding)
        return {"event": "actuator_bound", **binding, "channel_count": len(binding["channels"])}

    def apply_actuator_command(self, command: ActuatorCommand) -> dict[str, Any]:
        self.commands.append(command)
        return {
            "event": "actuator_command",
            "source_id": command.controller_id,
            "sequence": command.sequence,
            "result": self.result,
        }


def _plan() -> ControlPlan:
    controller_id = "robot.controller"
    return ControlPlan(
        session_id="a" * 64,
        controllers=(
            ControllerSpec(
                controller_id=controller_id,
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
                source=controller_id,
                transport="in_process",
                message_type="lingtu.sim.joint-torque.v1",
                command_type="joint_torque",
                target="actuators",
            ),
        ),
        stale_timeout_ns=100_000_000,
    )


def _snapshot() -> dict[str, Any]:
    half = math.sqrt(0.5)
    return {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 5,
        "reset_generation": 2,
        "sequence": 11,
        "physics_step": 10,
        "sim_time_ns": 20_000_000,
        "bodies": [
            {
                "stable_id": "robot/base_link",
                "instance_id": "robot",
                "frame_id": "base_link",
                "quaternion_wxyz": [half, 0.0, half, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 1.0],
            }
        ],
        "joints": [
            {
                "stable_id": "robot/joint_b",
                "instance_id": "robot",
                "position_rad": [2.0],
                "velocity_rps": [20.0],
            },
            {
                "stable_id": "robot/joint_a",
                "instance_id": "robot",
                "position_rad": [1.0],
                "velocity_rps": [10.0],
            },
        ],
        "actuators": [],
    }


def test_bridge_binds_compiled_actuator_layout_and_projects_physics_state() -> None:
    host = FakeActuatorHost()
    bridge = ControllerPhysicsBridge(
        plan=_plan(),
        controller_id="robot.controller",
        attach_root="base_link",
        host=host,
    )

    bridge.bind()
    state = bridge.project_state(_snapshot())

    assert host.bindings == [
        {
            "source_id": "robot.controller",
            "instance_id": "robot",
            "command_type": "joint_torque",
            "stale_timeout_ns": 100_000_000,
            "channels": ("joint_a", "joint_b"),
        }
    ]
    assert state.generation == GenerationStamp(5, 2)
    assert state.channels["joint_position"] == (1.0, 2.0)
    assert state.channels["joint_velocity"] == (10.0, 20.0)
    assert state.channels["base_angular_velocity"] == pytest.approx((-1.0, 0.0, 0.0))
    assert state.channels["projected_gravity"] == pytest.approx((1.0, 0.0, 0.0))


def test_bridge_forwards_only_current_compiled_actuator_commands() -> None:
    host = FakeActuatorHost()
    bridge = ControllerPhysicsBridge(
        plan=_plan(),
        controller_id="robot.controller",
        attach_root="base_link",
        host=host,
    )
    bridge.bind()
    command = ActuatorCommand(
        session_id="a" * 64,
        channel_id="robot.joint_torque",
        controller_id="robot.controller",
        instance_id="robot",
        generation=GenerationStamp(5, 2),
        sequence=1,
        apply_time_ns=20_000_000,
        command_type="joint_torque",
        channels=("joint_a", "joint_b"),
        values=(0.1, -0.2),
        safe_stop=False,
    )

    assert bridge.apply(command)["result"] == "applied"
    assert host.commands == [command]

    host.result = "rejected_stale_reset_generation"
    with pytest.raises(ControllerRuntimeError, match="rejected_stale_reset_generation"):
        bridge.apply(command)
