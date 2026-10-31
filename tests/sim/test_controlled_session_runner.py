
from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from sim.runtime.control import (
    ActuatorLayout,
    AdapterSpec,
    CommandChannelSpec,
    CommandSubmitResult,
    ControllerCommand,
    ControllerRuntime,
    ControllerSpec,
    ControllerState,
    ControlPlan,
    DeterministicFakeAdapter,
    DeterministicFakePolicy,
    GenerationStamp,
    PolicySpec,
    SafeStopReason,
)
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
    def __init__(self, *, ready: bool = True, step_ms: int = 2, timeout_ms: int = 100) -> None:
        self.state = RuntimeState.NEW
        self._ready = ready
        self.commands: list[Any] = []
        self.advances: list[int] = []
        self.calls: list[str] = []
        self.sim_time_ns = 0
        self.physics_step = 0
        self.plan = SimpleNamespace(global_policy=SimpleNamespace(timestep_s=step_ms / 1_000))
        self.command_timeout_ns = timeout_ms * 1_000_000

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
        self.sim_time_ns += steps * round(self.plan.global_policy.timestep_s * 1_000_000_000)
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


class _RuntimeCoordinator(_FakeCoordinator):
    """Use the actual controller freshness checks at each simulated physics step."""

    def __init__(self, *, step_ms: int = 5, timeout_ms: int = 100):
        super().__init__(step_ms=step_ms, timeout_ms=timeout_ms)
        spec = ControllerSpec(
            controller_id="controller", instance_id="robot",
            adapter=AdapterSpec("test", "test"), policy=PolicySpec("analytic", "", ""),
            inference_hz=50, low_level_hz=200, state_channels=(),
            command_channels=("command", "output"), actuators=ActuatorLayout(("joint",)),
        )
        plan = ControlPlan(
            session_id="a" * 64, controllers=(spec,), stale_timeout_ns=self.command_timeout_ns,
            command_channels=(
                CommandChannelSpec("command", "subscribe", "simulation", "input", "typed_dds",
                                   "base_twist", "base_twist", "base"),
                CommandChannelSpec("output", "publish", "physics", "controller", "in_process",
                                   "joint_torque", "joint_torque", "actuators"),
            ),
        )
        self.runtime = ControllerRuntime(
            plan=plan, controller_id=spec.controller_id, generation=GenerationStamp(3, 4),
            adapter=DeterministicFakeAdapter(), policy=DeterministicFakePolicy((0.5,)),
        )
        self.results = []

    def submit_controller_command(self, controller_id, command):
        self.commands.append((controller_id, command))
        return self.runtime.submit_command(command)

    def advance(self, steps):
        step_ns = round(self.plan.global_policy.timestep_s * 1_000_000_000)
        for index in range(steps):
            self.results.append(self.runtime.step(ControllerState(
                session_id="a" * 64, instance_id="robot", generation=GenerationStamp(3, 4),
                sequence=self.physics_step + index, sim_time_ns=self.sim_time_ns + index * step_ns,
                channels={},
            )))
        return super().advance(steps)


@pytest.mark.parametrize("step_ms,timeout_ms", [(1, 100), (2, 100), (5, 100), (5, 40), (5, 10)])
def test_refresh_respects_compiled_timing(step_ms, timeout_ms):
    coordinator = _RuntimeCoordinator(step_ms=step_ms, timeout_ms=timeout_ms)
    snapshot = run_base_twist_session(
        coordinator, BaseTwistTarget("controller", "robot", "command"), BaseTwist(linear_x=0.2),
        steps=100, refresh_steps=25,
    )
    outputs = [result.actuator_command for result in coordinator.results if result.actuator_command is not None]
    assert outputs and all(not output.safe_stop for output in outputs)
    assert snapshot["physics_step"] == 100
    assert all(chunk * step_ms <= min(20, timeout_ms) for chunk in coordinator.advances)
    if (step_ms, timeout_ms) == (5, 100):
        assert coordinator.advances == [4] * 25


def test_missing_refresh_still_times_out():
    coordinator = _RuntimeCoordinator()
    coordinator.prepare()
    coordinator.start()
    command = ControllerCommand("command", "robot", GenerationStamp(3, 4), 1, 0, {})
    assert coordinator.submit_controller_command("controller", command) is CommandSubmitResult.ACCEPTED
    coordinator.advance(25)
    outputs = [result.actuator_command for result in coordinator.results if result.actuator_command is not None]
    assert any(output.safe_stop_reason is SafeStopReason.STALE_COMMAND for output in outputs)
    coordinator.stop()


def test_pause_failure_still_stops_session(monkeypatch):
    coordinator = _FakeCoordinator()

    def fail_pause():
        coordinator.calls.append("pause")
        raise CoordinatorError("pause failed")

    monkeypatch.setattr(coordinator, "pause", fail_pause)
    with pytest.raises(CoordinatorError, match="pause failed"):
        run_base_twist_session(
            coordinator, BaseTwistTarget("controller", "robot", "command"), BaseTwist(),
            steps=1, refresh_steps=1,
        )
    assert coordinator.calls[-2:] == ["pause", "stop"]
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
