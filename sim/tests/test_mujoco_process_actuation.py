from __future__ import annotations

from pathlib import Path

import pytest

from sim.runtime.control import ActuatorCommand, GenerationStamp
from sim.runtime.coordinator import CoordinatorError
from sim.runtime.coordinator.mujoco_process import MujocoProcess


def _command(*, channels: tuple[str, ...] = ("hinge_joint",)) -> ActuatorCommand:
    return ActuatorCommand(
        session_id="a" * 64,
        channel_id="robot.control.joint_torque",
        controller_id="test_controller",
        instance_id="robot",
        generation=GenerationStamp(model_generation=5, reset_generation=2),
        sequence=9,
        apply_time_ns=2_000_000,
        command_type="joint_torque",
        channels=channels,
        values=(0.75,) * len(channels),
        safe_stop=False,
    )


def test_mujoco_process_serializes_one_time_binding_and_dense_commands(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        if line.startswith("bind-actuators "):
            return {
                "event": "actuator_bound",
                "source_id": "test_controller",
                "instance_id": "robot",
                "command_type": "joint_torque",
                "channel_count": 1,
            }
        return {
            "event": "actuator_command",
            "source_id": "test_controller",
            "sequence": 9,
            "result": "applied",
        }

    monkeypatch.setattr(process, "_request", request)

    bound = process.bind_actuators(
        source_id="test_controller",
        instance_id="robot",
        command_type="joint_torque",
        stale_timeout_ns=10_000_000,
        channels=("hinge_joint",),
    )
    applied = process.apply_actuator_command(_command())

    assert bound["event"] == "actuator_bound"
    assert applied["result"] == "applied"
    assert requests == [
        "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        f"actuate test_controller robot joint_torque {'a' * 64} 5 2 9 2000000 0 1 0.75",
    ]


def test_mujoco_process_decodes_one_round_trip_sampled_advance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        return {
            "event": "snapshot_batch",
            "snapshots": [
                {"event": "snapshot", "physics_step": step}
                for step in range(1, 6)
            ],
        }

    monkeypatch.setattr(process, "_request", request)

    snapshots = process.advance_sampled(5)

    assert requests == ["advance-sampled 5"]
    assert [snapshot["physics_step"] for snapshot in snapshots] == [1, 2, 3, 4, 5]


def test_mujoco_process_requests_only_exact_sensor_stride_snapshots(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"), sample_stride_steps=5)
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        return {
            "event": "snapshot_batch",
            "snapshots": [
                {"event": "snapshot", "physics_step": 5},
            ],
        }

    monkeypatch.setattr(process, "_request", request)

    snapshots = process.advance_sampled(5)

    assert requests == ["advance-sampled-realtime 5 5"]
    assert [snapshot["physics_step"] for snapshot in snapshots] == [5]


def test_mujoco_process_applies_and_advances_in_one_realtime_request(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"), sample_stride_steps=5)
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        if line.startswith("bind-actuators "):
            return {
                "event": "actuator_bound",
                "source_id": "test_controller",
                "instance_id": "robot",
                "command_type": "joint_torque",
                "channel_count": 1,
            }
        return {
            "event": "actuator_snapshot_batch",
            "source_id": "test_controller",
            "sequence": 9,
            "result": "applied",
            "snapshots": [
                {"event": "snapshot", "physics_step": 5},
            ],
        }

    monkeypatch.setattr(process, "_request", request)
    process.bind_actuators(
        source_id="test_controller",
        instance_id="robot",
        command_type="joint_torque",
        stale_timeout_ns=10_000_000,
        channels=("hinge_joint",),
    )

    snapshots = process.advance_sampled_with_actuator(_command(), 5)

    assert requests == [
        "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        (
            "actuate-advance-sampled-realtime 5 5 test_controller robot "
            f"joint_torque {'a' * 64} 5 2 9 2000000 0 1 0.75"
        ),
    ]
    assert [snapshot["physics_step"] for snapshot in snapshots] == [5]


def test_mujoco_process_fused_advance_fails_closed_on_actuator_rejection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"), sample_stride_steps=5)
    responses = iter(
        (
            {
                "event": "actuator_bound",
                "source_id": "test_controller",
                "instance_id": "robot",
                "command_type": "joint_torque",
                "channel_count": 1,
            },
            {
                "event": "actuator_snapshot_batch",
                "source_id": "test_controller",
                "sequence": 9,
                "result": "rejected_stale",
                "snapshots": [],
            },
        )
    )
    monkeypatch.setattr(process, "_request", lambda _line: next(responses))
    process.bind_actuators(
        source_id="test_controller",
        instance_id="robot",
        command_type="joint_torque",
        stale_timeout_ns=10_000_000,
        channels=("hinge_joint",),
    )

    with pytest.raises(CoordinatorError, match="rejected_stale"):
        process.advance_sampled_with_actuator(_command(), 5)


@pytest.mark.parametrize("sample_stride_steps", [True, 0, 4097])
def test_mujoco_process_rejects_invalid_sample_stride(
    sample_stride_steps: object,
) -> None:
    with pytest.raises(ValueError, match="sample_stride_steps"):
        MujocoProcess(
            Path("unused.exe"),
            sample_stride_steps=sample_stride_steps,  # type: ignore[arg-type]
        )


def test_mujoco_process_rejects_empty_or_oversized_sparse_batches(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"), sample_stride_steps=5)
    responses = iter(
        (
            {"event": "snapshot_batch", "snapshots": []},
            {
                "event": "snapshot_batch",
                "snapshots": [
                    {"event": "snapshot", "physics_step": step}
                    for step in range(1, 7)
                ],
            },
        )
    )
    monkeypatch.setattr(process, "_request", lambda _line: next(responses))

    with pytest.raises(CoordinatorError, match="snapshot count"):
        process.advance_sampled(5)
    with pytest.raises(CoordinatorError, match="snapshot count"):
        process.advance_sampled(5)


def test_mujoco_process_rejects_layout_drift_before_crossing_process_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    requests: list[str] = []

    def request(line: str) -> dict[str, object]:
        requests.append(line)
        return {
            "event": "actuator_bound",
            "source_id": "test_controller",
            "instance_id": "robot",
            "command_type": "joint_torque",
            "channel_count": 1,
        }

    monkeypatch.setattr(process, "_request", request)
    process.bind_actuators(
        source_id="test_controller",
        instance_id="robot",
        command_type="joint_torque",
        stale_timeout_ns=10_000_000,
        channels=("hinge_joint",),
    )

    with pytest.raises(CoordinatorError, match="actuator channel layout"):
        process.apply_actuator_command(_command(channels=("different_joint",)))

    assert len(requests) == 1


@pytest.mark.parametrize(
    "field,value",
    [
        ("source_id", "bad controller"),
        ("instance_id", "robot\nnext"),
        ("command_type", ""),
    ],
)
def test_mujoco_process_rejects_unsafe_protocol_tokens(
    monkeypatch: pytest.MonkeyPatch,
    field: str,
    value: str,
) -> None:
    process = MujocoProcess(Path("unused.exe"))
    monkeypatch.setattr(
        process,
        "_request",
        lambda line: pytest.fail(f"unexpected request: {line}"),
    )
    arguments = {
        "source_id": "test_controller",
        "instance_id": "robot",
        "command_type": "joint_torque",
    }
    arguments[field] = value

    with pytest.raises(CoordinatorError, match=field):
        process.bind_actuators(
            **arguments,
            stale_timeout_ns=10_000_000,
            channels=("hinge_joint",),
        )
