from __future__ import annotations

# ruff: noqa: S101
import json
import queue
import subprocess
import threading
from fractions import Fraction
from pathlib import Path
from typing import Any, TextIO

import pytest

from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.dds_adapter import encode_imu_sample
from sim.runtime.sensors.extractors import imu_from_snapshot
from sim.runtime.sensors.runtime import SensorRuntime

REPO_ROOT = Path(__file__).resolve().parents[2]
MUJOCO_HEADLESS = (
    REPO_ROOT
    / "build"
    / "mujoco-runtime-physics-win"
    / "Release"
    / "lingtu_mujoco_headless.exe"
)
SESSION_ID = "d" * 64
THUNDER_SESSION_ID = "e" * 64


def _require_headless() -> Path:
    if not MUJOCO_HEADLESS.is_file():
        pytest.skip(f"MuJoCo headless executable is missing: {MUJOCO_HEADLESS}")
    return MUJOCO_HEADLESS


def _launch_headless() -> tuple[subprocess.Popen[str], queue.Queue[str | None]]:
    executable = _require_headless()
    process = subprocess.Popen(  # noqa: S603 - test uses a fixed repo-local binary path.
        [
            str(executable),
            "--session",
            SESSION_ID,
            "5",
            str(REPO_ROOT / "tests/sim/physics/fixtures/scene_world.xml"),
            "--global-policy",
            "0.002",
            "rk4",
            "newton",
            "100",
            "0",
            "0",
            "-9.81",
            "--robot",
            "robot",
            str(REPO_ROOT / "tests/sim/physics/fixtures/actuated_hinge.xml"),
            "test_body",
            "0",
            "0",
            "0",
            "1",
            "0",
            "0",
            "0",
        ],
        cwd=REPO_ROOT,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="strict",
        bufsize=1,
    )
    assert process.stdout is not None
    events: queue.Queue[str | None] = queue.Queue()
    threading.Thread(
        target=_read_stdout,
        args=(process.stdout, events),
        daemon=True,
    ).start()
    return process, events


def _launch_thunder_headless() -> tuple[
    subprocess.Popen[str], queue.Queue[str | None]
]:
    executable = _require_headless()
    process = subprocess.Popen(  # noqa: S603 - fixed repo-local runtime and assets.
        [
            str(executable),
            "--session",
            THUNDER_SESSION_ID,
            "4",
            str(REPO_ROOT / "sim/packages/worlds/open_field/physics/open_field.xml"),
            "--global-policy",
            "0.001",
            "rk4",
            "newton",
            "100",
            "0",
            "0",
            "-9.81",
            "--robot",
            "thunder_01",
            str(REPO_ROOT / "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml"),
            "base_link",
            "0",
            "0",
            "0",
            "1",
            "0",
            "0",
            "0",
            "--initial-keyframe",
            "v4_nominal_stand",
        ],
        cwd=REPO_ROOT,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="strict",
        bufsize=1,
    )
    assert process.stdout is not None
    events: queue.Queue[str | None] = queue.Queue()
    threading.Thread(
        target=_read_stdout,
        args=(process.stdout, events),
        daemon=True,
    ).start()
    return process, events


def _launch_kinematic_headless() -> tuple[
    subprocess.Popen[str], queue.Queue[str | None]
]:
    executable = _require_headless()
    process = subprocess.Popen(  # noqa: S603 - fixed repo-local runtime and assets.
        [
            str(executable),
            "--session",
            SESSION_ID,
            "5",
            str(REPO_ROOT / "tests/sim/physics/fixtures/scene_world.xml"),
            "--global-policy",
            "0.002",
            "rk4",
            "newton",
            "100",
            "0",
            "0",
            "-9.81",
            "--robot",
            "robot",
            str(REPO_ROOT / "tests/sim/physics/fixtures/actuated_hinge.xml"),
            "test_body",
            "0",
            "0",
            "0",
            "1",
            "0",
            "0",
            "0",
            "--kinematic-entity",
            "pedestrian_01",
            str(
                REPO_ROOT
                / "sim/packages/scenarios/open_field_pedestrian_crossing/physics/pedestrian_capsule.xml"
            ),
            "proxy_root",
            "2",
            "0",
            "0",
            "1",
            "0",
            "0",
            "0",
        ],
        cwd=REPO_ROOT,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="strict",
        bufsize=1,
    )
    assert process.stdout is not None
    events: queue.Queue[str | None] = queue.Queue()
    threading.Thread(
        target=_read_stdout,
        args=(process.stdout, events),
        daemon=True,
    ).start()
    return process, events


def _read_stdout(stream: TextIO, events: queue.Queue[str | None]) -> None:
    try:
        for line in stream:
            events.put(line)
    finally:
        events.put(None)


def _read_event(
    process: subprocess.Popen[str], events: queue.Queue[str | None]
) -> dict[str, Any]:
    try:
        line = events.get(timeout=10)
    except queue.Empty as exc:
        process.kill()
        stderr = process.stderr.read() if process.stderr is not None else ""
        raise AssertionError(f"MuJoCo process did not emit an event: {stderr}") from exc
    if not line:
        stderr = process.stderr.read() if process.stderr is not None else ""
        raise AssertionError(f"MuJoCo process closed stdout early: {stderr}")
    event = json.loads(line)
    assert isinstance(event, dict)
    return event


def _request(
    process: subprocess.Popen[str], events: queue.Queue[str | None], command: str
) -> dict[str, Any]:
    assert process.stdin is not None
    process.stdin.write(command + "\n")
    process.stdin.flush()
    return _read_event(process, events)


def _stop(process: subprocess.Popen[str], events: queue.Queue[str | None]) -> None:
    if process.poll() is not None:
        return
    try:
        _request(process, events, "stop")
        process.wait(timeout=5)
    except Exception:
        process.kill()
        process.wait(timeout=5)


def test_mujoco_headless_process_runs_actuation_lifecycle() -> None:
    process, events = _launch_headless()
    try:
        ready = _read_event(process, events)
        assert ready["event"] == "ready"
        assert ready["session_id"] == SESSION_ID
        assert ready["model_generation"] == 5
        assert ready["reset_generation"] == 0

        bound = _request(
            process,
            events,
            "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        )
        assert bound == {
            "event": "actuator_bound",
            "source_id": "test_controller",
            "instance_id": "robot",
            "command_type": "joint_torque",
            "channel_count": 1,
        }

        running = _request(process, events, "start")
        assert running["event"] == "running"

        actuated = _request(
            process,
            events,
            f"actuate test_controller robot joint_torque {SESSION_ID} 5 0 1 0 0 1 0.75",
        )
        assert actuated == {
            "event": "actuator_command",
            "source_id": "test_controller",
            "sequence": 1,
            "result": "applied",
        }

        advanced = _request(process, events, "advance 1")
        assert advanced["event"] == "snapshot"
        assert advanced["physics_step"] == 1
        assert advanced["actuators"][0]["stable_id"] == "robot/hinge_joint"
        assert advanced["actuators"][0]["control"] == pytest.approx(0.75)

        sampled = _request(process, events, "advance-sampled 3")
        assert sampled["event"] == "snapshot_batch"
        snapshots = sampled["snapshots"]
        assert [item["event"] for item in snapshots] == ["snapshot"] * 3
        assert [item["physics_step"] for item in snapshots] == [2, 3, 4]
        assert [item["sim_time_ns"] for item in snapshots] == [
            4_000_000,
            6_000_000,
            8_000_000,
        ]
        assert all(
            item["actuators"][0]["control"] == pytest.approx(0.75)
            for item in snapshots
        )

        strided = _request(process, events, "advance-sampled 5 2")
        assert strided["event"] == "snapshot_batch"
        strided_snapshots = strided["snapshots"]
        assert [item["physics_step"] for item in strided_snapshots] == [6, 8, 9]
        assert [item["sim_time_ns"] for item in strided_snapshots] == [
            12_000_000,
            16_000_000,
            18_000_000,
        ]

        projected = _request(process, events, "advance-sampled-realtime 2 2")
        projected_snapshot = projected["snapshots"][-1]
        assert set(projected_snapshot["bodies"][0]) == {
            "stable_id",
            "instance_id",
            "frame_id",
            "position_m",
            "quaternion_wxyz",
            "linear_velocity_mps",
            "angular_velocity_rps",
        }
        assert set(projected_snapshot["joints"][0]) == {
            "stable_id",
            "position_rad",
            "velocity_rps",
        }
        assert projected_snapshot["actuators"] == []

        snapshot = _request(process, events, "snapshot")
        assert snapshot["event"] == "snapshot"
        assert snapshot["sequence"] == projected_snapshot["sequence"]
        assert snapshot["actuators"][0]["control"] == pytest.approx(0.0)

        paused = _request(process, events, "pause")
        assert paused["event"] == "paused"

        reset = _request(process, events, "reset")
        assert reset["event"] == "snapshot"
        assert reset["reset_generation"] == 1
        assert reset["actuators"][0]["control"] == pytest.approx(0.0)

        stopped = _request(process, events, "stop")
        assert stopped["event"] == "stopped"
        process.wait(timeout=5)
        assert process.returncode == 0
    finally:
        _stop(process, events)


def test_mujoco_headless_fuses_actuation_with_projected_sampled_advance() -> None:
    process, events = _launch_headless()
    try:
        assert _read_event(process, events)["event"] == "ready"
        assert _request(
            process,
            events,
            "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        )["event"] == "actuator_bound"
        assert _request(process, events, "start")["event"] == "running"

        fused = _request(
            process,
            events,
            (
                "actuate-advance-sampled-realtime 5 5 test_controller robot "
                f"joint_torque {SESSION_ID} 5 0 1 0 0 1 0.75"
            ),
        )

        assert fused["event"] == "actuator_snapshot_batch"
        assert fused["source_id"] == "test_controller"
        assert fused["sequence"] == 1
        assert fused["result"] == "applied"
        assert [item["physics_step"] for item in fused["snapshots"]] == [5]
        assert fused["snapshots"][0]["actuators"] == []
        snapshot = _request(process, events, "snapshot")
        assert snapshot["physics_step"] == 5
        assert snapshot["actuators"][0]["control"] == pytest.approx(0.75)
    finally:
        _stop(process, events)


def test_mujoco_headless_rejected_fused_actuation_does_not_advance() -> None:
    process, events = _launch_headless()
    try:
        assert _read_event(process, events)["event"] == "ready"
        assert _request(
            process,
            events,
            "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        )["event"] == "actuator_bound"
        assert _request(process, events, "start")["event"] == "running"

        rejected = _request(
            process,
            events,
            (
                "actuate-advance-sampled-realtime 5 5 test_controller robot "
                f"joint_torque {'f' * 64} 5 0 1 0 0 1 0.75"
            ),
        )

        assert rejected == {
            "event": "actuator_snapshot_batch",
            "source_id": "test_controller",
            "sequence": 1,
            "result": "rejected_session",
            "snapshots": [],
        }
        snapshot = _request(process, events, "snapshot")
        assert snapshot["physics_step"] == 0
        assert snapshot["actuators"][0]["control"] == pytest.approx(0.0)
    finally:
        _stop(process, events)


def test_mujoco_headless_rejects_negative_actuator_timeout_without_binding() -> None:
    process, events = _launch_headless()
    try:
        assert _read_event(process, events)["event"] == "ready"

        rejected = _request(
            process,
            events,
            "bind-actuators test_controller robot joint_torque -1 1 hinge_joint",
        )
        assert rejected["event"] == "error"
        assert "stale timeout" in rejected["message"]

        rebound = _request(
            process,
            events,
            "bind-actuators test_controller robot joint_torque 10000000 1 hinge_joint",
        )
        assert rebound["event"] == "actuator_bound"
        assert rebound["source_id"] == "test_controller"
    finally:
        _stop(process, events)


def test_kinematic_scenario_proxy_changes_mujoco_truth_and_raycast_geometry() -> None:
    process, events = _launch_kinematic_headless()
    try:
        ready = _read_event(process, events)
        assert ready["event"] == "ready"
        initial = _request(process, events, "snapshot")
        proxy = next(
            body
            for body in initial["bodies"]
            if body["stable_id"] == "pedestrian_01/proxy_root"
        )
        assert proxy["position_m"] == pytest.approx([2.0, 0.0, 0.0])

        initial_pose = _request(
            process,
            events,
            (
                f"kinematic-poses {SESSION_ID} 5 0 "
                f"{initial['sequence']} {initial['sim_time_ns']} 1 "
                "pedestrian_01 pedestrian_01/proxy_root 2 0 0 1 0 0 0"
            ),
        )
        assert initial_pose["result"] == "applied"

        before = _request(
            process,
            events,
            (
                f"raycast robot/test_body {SESSION_ID} 5 0 "
                f"{initial['sequence']} {initial['sim_time_ns']} "
                "0.1 10 15 0 1 1 0 0 0"
            ),
        )
        assert before["hit_count"] == 1

        assert _request(process, events, "start")["event"] == "running"
        advanced = _request(process, events, "advance 1")
        assert advanced["event"] == "snapshot"

        bad_binding = _request(
            process,
            events,
            (
                f"kinematic-poses {SESSION_ID} 5 0 "
                f"{advanced['sequence']} {advanced['sim_time_ns']} 1 "
                "pedestrian_01 pedestrian_01/unknown_body 4 0 0 1 0 0 0"
            ),
        )
        assert bad_binding["result"] == "rejected_entity_set"

        applied = _request(
            process,
            events,
            (
                f"kinematic-poses {SESSION_ID} 5 0 "
                f"{advanced['sequence']} {advanced['sim_time_ns']} 1 "
                "pedestrian_01 pedestrian_01/proxy_root 4 0 0 1 0 0 0"
            ),
        )
        assert applied["result"] == "applied"

        duplicate = _request(
            process,
            events,
            (
                f"kinematic-poses {SESSION_ID} 5 0 "
                f"{advanced['sequence']} {advanced['sim_time_ns']} 1 "
                "pedestrian_01 pedestrian_01/proxy_root 6 0 0 1 0 0 0"
            ),
        )
        assert duplicate["result"] == "rejected_sequence"

        moved = _request(process, events, "snapshot")
        proxy = next(
            body
            for body in moved["bodies"]
            if body["stable_id"] == "pedestrian_01/proxy_root"
        )
        assert proxy["position_m"] == pytest.approx([4.0, 0.0, 0.0])
        after = _request(
            process,
            events,
            (
                f"raycast robot/test_body {SESSION_ID} 5 0 "
                f"{moved['sequence']} {moved['sim_time_ns']} "
                "0.1 10 15 0 1 1 0 0 0"
            ),
        )
        assert after["hit_count"] == 1
        assert after["hits"][0]["xyz_sensor"][0] > before["hits"][0]["xyz_sensor"][0] + 1.5

        reset = _request(process, events, "reset")
        proxy = next(
            body
            for body in reset["bodies"]
            if body["stable_id"] == "pedestrian_01/proxy_root"
        )
        assert proxy["position_m"] == pytest.approx([2.0, 0.0, 0.0])
    finally:
        _stop(process, events)


def test_real_thunder_snapshot_drives_the_200hz_imu_contract() -> None:
    process, events = _launch_thunder_headless()
    stream = SensorStreamPlan(
        stream_kind="imu",
        instance_id="thunder_01",
        sensor_id="thunder_01.imu",
        frame_id="thunder_01/imu",
        message_type="lingtu.dds.Imu",
        rate_hz=Fraction(200, 1),
        route=SensorRoute("physics", "mujoco_sensor", "typed_dds"),
    )
    scheduler = SensorRuntime(THUNDER_SESSION_ID, (stream,))
    try:
        ready = _read_event(process, events)
        assert ready["event"] == "ready"
        assert ready["sensor_count"] >= 3

        initial = _request(process, events, "snapshot")
        initial_due = scheduler.advance(
            sim_time_ns=0,
            model_generation=4,
            reset_generation=0,
        ).samples
        assert len(initial_due) == 1
        initial_sample = imu_from_snapshot(initial_due[0], initial)
        assert initial_sample.stamp.sim_time_ns == 0

        assert _request(process, events, "start")["event"] == "running"
        advanced = _request(process, events, "advance 5")
        assert advanced["sim_time_ns"] == 5_000_000
        due = scheduler.advance(
            sim_time_ns=advanced["sim_time_ns"],
            model_generation=advanced["model_generation"],
            reset_generation=advanced["reset_generation"],
        ).samples
        assert len(due) == 1 and due[0].sequence == 1
        sample = imu_from_snapshot(due[0], advanced)
        assert sample.stamp.frame_id == "thunder_01/imu"
        assert sample.orientation_wxyz is not None
        assert sample.angular_velocity_rps is not None
        assert sample.linear_acceleration_mps2 is not None
        encoded = encode_imu_sample(sample)
        assert encoded[:4] == b"LTIM"
        assert len(encoded) == 256
    finally:
        _stop(process, events)
