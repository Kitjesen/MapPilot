# ruff: noqa: S101

from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.control import CommandSubmitResult
from sim.runtime.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.controlled_run import (
    BaseTwist,
    BaseTwistTarget,
    _optional_truth_odometry_factory,
    _truth_odometry_states,
    run_base_twist_session,
)
from sim.runtime.sensors import (
    SensorReadiness,
    SensorRoute,
    SensorRuntime,
    SensorStreamPlan,
    TruthOdometryEndpointFactory,
)


class _FakeCoordinator:
    def __init__(self, *, ready: bool = True) -> None:
        self.state = RuntimeState.NEW
        self._ready = ready
        self.commands: list[Any] = []
        self.advances: list[int] = []
        self.calls: list[str] = []
        self.sim_time_ns = 0
        self.physics_step = 0

    def prepare(self) -> dict[str, Any]:
        self.calls.append("prepare")
        self.state = RuntimeState.READY if self._ready else RuntimeState.PREPARING
        return self._event("ready")

    def start(self) -> dict[str, Any]:
        self.calls.append("start")
        self.state = RuntimeState.RUNNING
        return self._event("running")

    def submit_controller_command(self, controller_id: str, command: Any) -> Any:
        self.commands.append((controller_id, command))
        return CommandSubmitResult.ACCEPTED

    def advance(self, steps: int) -> dict[str, Any]:
        self.advances.append(steps)
        self.physics_step += steps
        self.sim_time_ns += steps * 2_000_000
        return self._event("snapshot")

    def pause(self) -> dict[str, Any]:
        self.calls.append("pause")
        self.state = RuntimeState.PAUSED
        return self._event("paused")

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        self.state = RuntimeState.STOPPED
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": "a" * 64,
            "model_generation": 3,
            "reset_generation": 4,
            "sequence": self.physics_step,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
            "bodies": [],
            "joints": [],
            "actuators": [],
        }


def test_controlled_runner_refreshes_typed_command_before_stale_timeout() -> None:
    coordinator = _FakeCoordinator()
    target = BaseTwistTarget(
        controller_id="thunder_01.controller",
        instance_id="thunder_01",
        channel_id="thunder_01.control.base_twist",
    )

    snapshot = run_base_twist_session(
        coordinator,
        target,
        BaseTwist(linear_x=0.2, linear_y=0.0, angular_z=-0.1),
        steps=5,
        refresh_steps=2,
    )

    assert coordinator.advances == [2, 2, 1]
    assert [entry[1].sequence for entry in coordinator.commands] == [1, 2, 3]
    assert [entry[1].apply_time_ns for entry in coordinator.commands] == [0, 4_000_000, 8_000_000]
    assert all(entry[0] == target.controller_id for entry in coordinator.commands)
    assert coordinator.commands[0][1].payload == {
        "linear_x": 0.2,
        "linear_y": 0.0,
        "angular_z": -0.1,
    }
    assert snapshot["physics_step"] == 5
    assert coordinator.calls == ["prepare", "start", "pause", "stop"]


def test_controlled_runner_refuses_partially_qualified_session_and_stops() -> None:
    coordinator = _FakeCoordinator(ready=False)

    with pytest.raises(CoordinatorError, match="not READY"):
        run_base_twist_session(
            coordinator,
            BaseTwistTarget("controller", "robot", "command"),
            BaseTwist(),
            steps=1,
            refresh_steps=1,
        )

    assert coordinator.calls == ["prepare", "stop"]
    assert coordinator.state is RuntimeState.STOPPED


def test_controlled_runner_enables_truth_odometry_only_when_binary_is_supplied(
    tmp_path: Path,
) -> None:
    assert _optional_truth_odometry_factory(None, parent_frame="world") is None

    factory = _optional_truth_odometry_factory(
        tmp_path / "lingtu_truth_odom_publisher.exe",
        parent_frame="world",
    )

    assert isinstance(factory, TruthOdometryEndpointFactory)
    assert factory.executable == (
        tmp_path / "lingtu_truth_odom_publisher.exe"
    ).resolve()
    assert factory.parent_frame == "world"


def test_controlled_runner_reports_observed_truth_stream_state() -> None:
    plan = SensorRuntime(
        "a" * 64,
        (
            SensorStreamPlan(
                stream_kind="truth_odom",
                instance_id="thunder_01",
                sensor_id="thunder_01.truth_odom",
                frame_id="thunder_01/base_link",
                message_type="lingtu.dds.Odometry",
                rate_hz=Fraction(100, 1),
                route=SensorRoute(
                    owner="physics",
                    source="mujoco_truth",
                    transport="typed_dds",
                ),
            ),
        ),
    )
    readiness = SensorReadiness.from_runtime(plan).mark_prepared(
        "thunder_01.truth_odom",
        source="mujoco_truth",
    ).mark_active(
        "thunder_01.truth_odom",
        source="mujoco_truth",
    )

    assert _truth_odometry_states(readiness) == {
        "thunder_01.truth_odom": "ACTIVE"
    }


@pytest.mark.parametrize(
    "kwargs",
    [
        {"linear_x": float("nan")},
        {"linear_y": float("inf")},
        {"angular_z": "fast"},
    ],
)
def test_base_twist_rejects_non_finite_or_non_numeric_values(kwargs: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        BaseTwist(**kwargs)
