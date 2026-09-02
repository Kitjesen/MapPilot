# ruff: noqa: S101

from __future__ import annotations

import json
import math
from collections.abc import Callable
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.control import CommandSubmitResult, create_production_components
from sim.runtime.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.controlled_run import BaseTwist, BaseTwistTarget
from sim.runtime.coordinator.motion_recording import (
    FrameCaptureOptions,
    ManeuverSegment,
    MotionRecordingConfig,
    MotionRecordingLaunch,
    MotionRecordingRuntimeConfig,
    _parser,
    _require_recording_bindings,
    create_motion_recording_launch,
    evaluate_maneuver_evidence,
    evaluate_motion_evidence,
    run_motion_recording,
)
from sim.runtime.coordinator.run_allocation import load_resolved_session_bundle
from sim.runtime.replay import SimulationReplay


def _snapshot(
    x: float,
    y: float,
    z: float = 0.5,
    *,
    yaw_rad: float = 0.0,
    sim_time_ns: int = 0,
) -> dict[str, object]:
    return {
        "event": "snapshot",
        "sequence": sim_time_ns // 2_000_000,
        "physics_step": sim_time_ns // 2_000_000,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "instance_id": "thunder_01",
                "frame_id": "base_link",
                "position_m": [x, y, z],
                "quaternion_wxyz": [
                    math.cos(yaw_rad / 2.0),
                    0.0,
                    0.0,
                    math.sin(yaw_rad / 2.0),
                ],
            }
        ],
    }


def test_nonzero_command_requires_finite_truth_displacement() -> None:
    evidence = evaluate_motion_evidence(
        _snapshot(0.0, -76.0),
        _snapshot(0.38, -76.002),
        base_body_id="thunder_01/base_link",
        command=BaseTwist(linear_x=0.25),
        minimum_displacement_m=0.05,
    )

    assert evidence.command_nonzero is True
    assert evidence.horizontal_displacement_m == pytest.approx(0.3800052631)
    assert evidence.moved is True
    assert evidence.expectation_met is True


def test_zero_command_is_never_reported_as_verified_motion() -> None:
    evidence = evaluate_motion_evidence(
        _snapshot(0.0, -76.0),
        _snapshot(0.002, -76.001),
        base_body_id="thunder_01/base_link",
        command=BaseTwist(),
        minimum_displacement_m=0.01,
    )

    assert evidence.command_nonzero is False
    assert evidence.moved is False
    assert evidence.expectation_met is True
    assert evidence.motion_verified is False


def test_recording_config_exposes_an_ordered_maneuver_sequence() -> None:
    maneuvers = (
        ManeuverSegment("forward", BaseTwist(linear_x=0.10), frames=12),
        ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=9),
    )

    config = MotionRecordingConfig(
        maneuvers=maneuvers,
        steps_per_frame=8,
        refresh_steps=4,
        transition_frames=3,
    )

    assert config.sequence == maneuvers
    assert config.total_frames == 24


def test_maneuver_evidence_projects_translation_in_the_start_body_frame() -> None:
    segment = ManeuverSegment("forward", BaseTwist(linear_x=0.10), frames=2)

    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(1.0, 2.0, yaw_rad=math.pi / 2.0, sim_time_ns=10),
            _snapshot(1.0, 2.06, yaw_rad=math.pi / 2.0, sim_time_ns=15),
            _snapshot(1.0, 2.12, yaw_rad=math.pi / 2.0, sim_time_ns=20),
        ),
        maneuver_index=0,
        maneuver=segment,
        base_body_id="thunder_01/base_link",
        start_frame_index=4,
        end_frame_index=6,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.1,
    )

    assert evidence.body_frame_forward_m == pytest.approx(0.12)
    assert evidence.body_frame_left_m == pytest.approx(0.0, abs=1e-9)
    assert evidence.signed_translation_m == pytest.approx(0.12)
    assert evidence.translation_met is True
    assert evidence.expectation_met is True


def test_maneuver_evidence_unwraps_signed_yaw_across_pi() -> None:
    segment = ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=2)

    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(1.0, 2.0, yaw_rad=3.0, sim_time_ns=10),
            _snapshot(1.0, 2.0, yaw_rad=-3.1, sim_time_ns=20),
            _snapshot(1.0, 2.0, yaw_rad=-2.8, sim_time_ns=30),
        ),
        maneuver_index=4,
        maneuver=segment,
        base_body_id="thunder_01/base_link",
        start_frame_index=20,
        end_frame_index=22,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.4,
    )

    assert evidence.signed_yaw_rad == pytest.approx(0.4831853072)
    assert evidence.rotation_met is True
    assert evidence.expectation_met is True


def test_maneuver_evidence_treats_clockwise_yaw_as_progress_for_right_turn() -> None:
    segment = ManeuverSegment("turn_right", BaseTwist(angular_z=-0.35), frames=2)

    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(1.0, 2.0, yaw_rad=0.0, sim_time_ns=10),
            _snapshot(1.0, 2.0, yaw_rad=-0.2, sim_time_ns=20),
            _snapshot(1.0, 2.0, yaw_rad=-0.4, sim_time_ns=30),
        ),
        maneuver_index=5,
        maneuver=segment,
        base_body_id="thunder_01/base_link",
        start_frame_index=30,
        end_frame_index=32,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.3,
    )

    assert evidence.signed_yaw_rad == pytest.approx(0.4)
    assert evidence.rotation_met is True
    assert evidence.expectation_met is True


def test_maneuver_evidence_rejects_a_turn_that_drifts_beyond_its_limit() -> None:
    segment = ManeuverSegment("turn_right", BaseTwist(angular_z=-0.35), frames=2)

    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(0.0, 0.0, yaw_rad=0.0, sim_time_ns=0),
            _snapshot(0.09, 0.01, yaw_rad=-0.2, sim_time_ns=500_000_000),
            _snapshot(0.18, 0.02, yaw_rad=-0.4, sim_time_ns=1_000_000_000),
        ),
        maneuver_index=5,
        maneuver=segment,
        base_body_id="thunder_01/base_link",
        start_frame_index=30,
        end_frame_index=32,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.3,
        maximum_turn_drift_m=0.08,
    )

    assert evidence.signed_yaw_rad == pytest.approx(0.4)
    assert evidence.horizontal_drift_m == pytest.approx(math.hypot(0.18, 0.02))
    assert evidence.rotation_met is True
    assert evidence.turn_drift_met is False
    assert evidence.expectation_met is False


@pytest.mark.parametrize(
    ("name", "angular_z", "measured_yaw_rad", "measured_drift_m", "drift_met"),
    [
        ("turn_left", 0.35, 0.2686, 0.069, True),
        ("turn_right", -0.35, -0.2328, 0.145, False),
    ],
)
def test_run_e_turns_fail_factory_park_measured_response_gate(
    name: str,
    angular_z: float,
    measured_yaw_rad: float,
    measured_drift_m: float,
    drift_met: bool,
) -> None:
    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(0.0, 0.0, yaw_rad=0.0, sim_time_ns=0),
            _snapshot(
                measured_drift_m,
                0.0,
                yaw_rad=measured_yaw_rad,
                sim_time_ns=7_008_000_000,
            ),
        ),
        maneuver_index=4 if angular_z > 0.0 else 5,
        maneuver=ManeuverSegment(name, BaseTwist(angular_z=angular_z), frames=1),
        base_body_id="thunder_01/base_link",
        start_frame_index=0,
        end_frame_index=1,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.35,
        maximum_turn_drift_m=0.10,
    )

    assert evidence.command.angular_z == pytest.approx(angular_z)
    assert evidence.signed_yaw_rad == pytest.approx(abs(measured_yaw_rad))
    assert evidence.horizontal_drift_m == pytest.approx(measured_drift_m)
    assert evidence.rotation_met is False
    assert evidence.turn_drift_met is drift_met
    assert evidence.expectation_met is False
    assert evidence.motion_verified is False


@pytest.mark.parametrize(
    ("name", "command", "end_x", "end_y"),
    [
        ("forward", BaseTwist(linear_x=0.3), 0.12, 0.0),
        ("backward", BaseTwist(linear_x=-0.3), -0.12, 0.0),
        ("left", BaseTwist(linear_y=0.3), 0.0, 0.12),
        ("right", BaseTwist(linear_y=-0.3), 0.0, -0.12),
    ],
)
def test_factory_park_turn_gate_does_not_regress_four_translation_directions(
    name: str,
    command: BaseTwist,
    end_x: float,
    end_y: float,
) -> None:
    evidence = evaluate_maneuver_evidence(
        (
            _snapshot(0.0, 0.0, sim_time_ns=0),
            _snapshot(end_x, end_y, sim_time_ns=1_000_000_000),
        ),
        maneuver_index=0,
        maneuver=ManeuverSegment(name, command, frames=1),
        base_body_id="thunder_01/base_link",
        start_frame_index=0,
        end_frame_index=1,
        minimum_displacement_m=0.05,
        minimum_rotation_rad=0.35,
        maximum_turn_drift_m=0.10,
    )

    assert evidence.signed_translation_m == pytest.approx(0.12)
    assert evidence.translation_met is True
    assert evidence.turn_drift_met is True
    assert evidence.expectation_met is True
    assert evidence.motion_verified is True


def test_maneuver_evidence_rejects_a_truncated_truth_sample_sequence() -> None:
    segment = ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=2)

    with pytest.raises(ValueError, match="complete truth snapshot sequence"):
        evaluate_maneuver_evidence(
            (
                _snapshot(0.0, 0.0, yaw_rad=0.0, sim_time_ns=0),
                _snapshot(0.0, 0.0, yaw_rad=0.4, sim_time_ns=1_000_000_000),
            ),
            maneuver_index=4,
            maneuver=segment,
            base_body_id="thunder_01/base_link",
            start_frame_index=20,
            end_frame_index=22,
            minimum_displacement_m=0.05,
            minimum_rotation_rad=0.35,
            maximum_turn_drift_m=0.10,
        )


def test_maneuver_evidence_rejects_non_monotonic_truth_sample_times() -> None:
    segment = ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=2)

    with pytest.raises(ValueError, match="strictly increasing sim_time_ns"):
        evaluate_maneuver_evidence(
            (
                _snapshot(0.0, 0.0, yaw_rad=0.0, sim_time_ns=10),
                _snapshot(0.0, 0.0, yaw_rad=0.2, sim_time_ns=20),
                _snapshot(0.0, 0.0, yaw_rad=0.4, sim_time_ns=20),
            ),
            maneuver_index=4,
            maneuver=segment,
            base_body_id="thunder_01/base_link",
            start_frame_index=20,
            end_frame_index=22,
            minimum_displacement_m=0.05,
            minimum_rotation_rad=0.35,
            maximum_turn_drift_m=0.10,
        )


def test_maneuver_evidence_fails_closed_when_measured_yaw_is_missing() -> None:
    end = _snapshot(0.0, 0.0, sim_time_ns=1_000_000_000)
    bodies = end["bodies"]
    assert isinstance(bodies, list)
    body = bodies[0]
    assert isinstance(body, dict)
    del body["quaternion_wxyz"]

    with pytest.raises(ValueError, match="quaternion_wxyz must have length 4"):
        evaluate_maneuver_evidence(
            (_snapshot(0.0, 0.0, sim_time_ns=0), end),
            maneuver_index=4,
            maneuver=ManeuverSegment(
                "turn_left",
                BaseTwist(angular_z=0.35),
                frames=1,
            ),
            base_body_id="thunder_01/base_link",
            start_frame_index=20,
            end_frame_index=21,
            minimum_displacement_m=0.05,
            minimum_rotation_rad=0.35,
            maximum_turn_drift_m=0.10,
        )


class _RecordingCoordinator:
    def __init__(self) -> None:
        self.state = RuntimeState.NEW
        self.physics_step = 0
        self.sim_time_ns = 0
        self.x = 0.0
        self.commands: list[Any] = []
        self.episode_artifacts: dict[str, str] = {}
        self.last_failure_reason: str | None = None
        self.manifest_path = Path("session.runtime.json")

    def prepare(self) -> dict[str, Any]:
        self.state = RuntimeState.READY
        return self._event("ready")

    def snapshot(self) -> dict[str, Any]:
        return self._event("snapshot")

    def start(self) -> dict[str, Any]:
        self.state = RuntimeState.RUNNING
        return self._event("running")

    def submit_controller_command(self, controller_id: str, command: Any) -> Any:
        self.commands.append((controller_id, command))
        return CommandSubmitResult.ACCEPTED

    def advance(self, steps: int) -> dict[str, Any]:
        self.physics_step += steps
        self.sim_time_ns += steps * 2_000_000
        self.x += steps * 0.01
        return self._event("snapshot")

    def pause(self) -> dict[str, Any]:
        self.state = RuntimeState.PAUSED
        return self._event("paused")

    def stop(self, *, failure_reason: str | None = None) -> dict[str, Any]:
        self.last_failure_reason = failure_reason
        self.state = RuntimeState.STOPPED
        return self._event("stopped")

    def register_episode_artifact(self, name: str, path: str) -> None:
        self.episode_artifacts[name] = path

    def _event(self, event: str) -> dict[str, Any]:
        return {
            **_snapshot(self.x, -76.0),
            "event": event,
            "session_id": "motion-session",
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": self.physics_step,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
            "joints": [],
            "actuators": [],
        }


class _RecordingHost:
    def __init__(
        self,
        coordinator: _RecordingCoordinator,
        *,
        on_prepare: Callable[[], None] | None = None,
    ) -> None:
        self.coordinator = coordinator
        self.closed = False
        self.on_prepare = on_prepare

    def prepare(self) -> dict[str, Any]:
        event = self.coordinator.prepare()
        if self.on_prepare is not None:
            self.on_prepare()
        return event

    def close(self) -> None:
        self.closed = True
        if self.coordinator.state is RuntimeState.RUNNING:
            self.coordinator.pause()
        self.coordinator.stop()

    def stop(self, *, failure_reason: str | None = None) -> dict[str, Any]:
        self.closed = True
        if self.coordinator.state is RuntimeState.RUNNING:
            self.coordinator.pause()
        return self.coordinator.stop(failure_reason=failure_reason)


class _LateFrameOnCloseHost(_RecordingHost):
    def __init__(self, coordinator: _RecordingCoordinator, frame_dir: Path) -> None:
        super().__init__(coordinator)
        self.frame_dir = frame_dir

    def close(self) -> None:
        super().close()
        (self.frame_dir / "frame_000001.png").write_bytes(b"\x89PNG\r\n\x1a\nlate-frame")


class _RecordingPublisher:
    def __init__(self) -> None:
        self.snapshots: list[dict[str, Any]] = []

    def publish(self, event: dict[str, Any]) -> int:
        self.snapshots.append(event)
        return 1


class _FailingRecordingCoordinator(_RecordingCoordinator):
    def advance(self, steps: int) -> dict[str, Any]:
        del steps
        raise RuntimeError("fixture physics failure")


class _StationaryRecordingCoordinator(_RecordingCoordinator):
    def advance(self, steps: int) -> dict[str, Any]:
        self.physics_step += steps
        self.sim_time_ns += steps * 2_000_000
        return self._event("snapshot")


class _ManeuverRecordingCoordinator(_RecordingCoordinator):
    def __init__(self) -> None:
        super().__init__()
        self.y = -76.0
        self.yaw = 0.0
        self.current_command = BaseTwist()

    def submit_controller_command(self, controller_id: str, command: Any) -> Any:
        result = super().submit_controller_command(controller_id, command)
        self.current_command = BaseTwist(**command.payload)
        return result

    def advance(self, steps: int) -> dict[str, Any]:
        dt = 0.5 * steps
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        self.x += (cosine * self.current_command.linear_x - sine * self.current_command.linear_y) * dt
        self.y += (sine * self.current_command.linear_x + cosine * self.current_command.linear_y) * dt
        self.yaw += self.current_command.angular_z * dt
        self.physics_step += steps
        self.sim_time_ns += steps * 500_000_000
        return self._event("snapshot")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            **_snapshot(
                self.x,
                self.y,
                yaw_rad=self.yaw,
                sim_time_ns=self.sim_time_ns,
            ),
            "event": event,
            "session_id": "motion-session",
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": self.physics_step,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
            "joints": [],
            "actuators": [],
        }


class _AllocationSensitiveCoordinator(_RecordingCoordinator):
    def __init__(self, run_dir: Path) -> None:
        super().__init__()
        self.run_dir = run_dir

    def prepare(self) -> dict[str, Any]:
        if self.run_dir.exists():
            raise RuntimeError("run allocation was pre-created")
        (self.run_dir / "logs").mkdir(parents=True)
        return super().prepare()


def test_recording_run_writes_truth_trajectory_and_motion_evidence(tmp_path: Path) -> None:
    coordinator = _RecordingCoordinator()
    publisher = _RecordingPublisher()
    frame_dir = tmp_path / "frames"

    def capture_fresh_frame() -> None:
        frame_dir.mkdir(parents=True, exist_ok=True)
        (frame_dir / "frame_000000.png").write_bytes(b"\x89PNG\r\n\x1a\nframe")

    host = _RecordingHost(coordinator, on_prepare=capture_fresh_frame)
    capture = FrameCaptureOptions(
        directory=frame_dir,
        capture_every=1,
        maximum_frames=8,
        minimum_frames=1,
        motion_camera_stable_id="thunder_01/base_link",
    )
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=publisher,
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget(
            "thunder_01.thunderv4_locomotion",
            "thunder_01",
            "thunder_01.control.base_twist",
        ),
        base_body_id="thunder_01/base_link",
        capture=capture,
        run_id="fixture-production-motion",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=tmp_path / "motion-evidence.json",
        manifest_path=tmp_path / "session.runtime.json",
    )

    result = run_motion_recording(
        launch,
        MotionRecordingConfig(
            command=BaseTwist(linear_x=0.25),
            frames=4,
            steps_per_frame=3,
            refresh_steps=2,
            minimum_displacement_m=0.05,
            frame_flush_timeout_s=0.01,
            sleep_s=0.0,
        ),
    )

    assert result.motion is not None
    assert result.motion.motion_verified is True
    assert result.motion.horizontal_displacement_m == pytest.approx(0.12)
    assert result.frames.captured_count == 1
    assert result.frames.motion_camera_stable_id == "thunder_01/base_link"
    assert result.frames_published == 4
    assert len(publisher.snapshots) == 4
    assert len(coordinator.commands) == 8
    assert result.trajectory_path.read_text(encoding="utf-8").count("\n") == 5
    assert result.evidence_path.is_file()
    replay = SimulationReplay.open(tmp_path)
    assert result.recording_manifest_path == tmp_path / "simulation-recording.json"
    assert result.recording_timeline_path == tmp_path / "simulation-timeline.jsonl"
    assert replay.frame_count == 5
    assert replay.frames[-1].snapshot["bodies"][0]["position_m"][0] == pytest.approx(0.12)
    assert replay.frames[-1].command["payload"]["linear_x"] == pytest.approx(0.25)
    assert coordinator.episode_artifacts == {
        "simulation_recording": "simulation-recording.json",
        "simulation_timeline": "simulation-timeline.jsonl",
    }
    assert host.closed is True
    assert coordinator.state is RuntimeState.STOPPED


def test_recording_finalizes_frame_count_after_runtime_close(tmp_path: Path) -> None:
    coordinator = _RecordingCoordinator()
    frame_dir = tmp_path / "frames"

    def capture_first_frame() -> None:
        frame_dir.mkdir(parents=True, exist_ok=True)
        (frame_dir / "frame_000000.png").write_bytes(b"\x89PNG\r\n\x1a\nfirst-frame")

    host = _LateFrameOnCloseHost(coordinator, frame_dir)
    host.on_prepare = capture_first_frame
    evidence_path = tmp_path / "motion-evidence.json"
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=_RecordingPublisher(),
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=frame_dir, minimum_frames=1),
        run_id="fixture-final-frame-close",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=evidence_path,
        manifest_path=tmp_path / "session.runtime.json",
    )

    result = run_motion_recording(
        launch,
        MotionRecordingConfig(
            command=BaseTwist(linear_x=0.25),
            frames=1,
            steps_per_frame=1,
            refresh_steps=1,
            minimum_displacement_m=0.005,
            frame_flush_timeout_s=0.01,
            sleep_s=0,
        ),
    )

    assert result.frames.captured_count == 2
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    assert evidence["frames"]["captured_count"] == 2
    assert evidence["frames"]["last_frame"].endswith("frame_000001.png")


def test_recording_run_executes_named_maneuvers_with_zero_command_transitions(
    tmp_path: Path,
) -> None:
    coordinator = _ManeuverRecordingCoordinator()
    publisher = _RecordingPublisher()
    frame_dir = tmp_path / "frames"

    def capture_fresh_frame() -> None:
        frame_dir.mkdir(parents=True, exist_ok=True)
        (frame_dir / "frame_000000.png").write_bytes(b"\x89PNG\r\n\x1a\nframe")

    host = _RecordingHost(coordinator, on_prepare=capture_fresh_frame)
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=publisher,
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=frame_dir, minimum_frames=1),
        run_id="fixture-maneuver-sequence",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=tmp_path / "motion-evidence.json",
        manifest_path=tmp_path / "session.runtime.json",
    )

    result = run_motion_recording(
        launch,
        MotionRecordingConfig(
            maneuvers=(
                ManeuverSegment("forward", BaseTwist(linear_x=0.10), frames=2),
                ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=2),
            ),
            transition_frames=1,
            steps_per_frame=1,
            refresh_steps=1,
            minimum_displacement_m=0.05,
            minimum_rotation_rad=0.1,
            maximum_turn_drift_m=0.10,
            acceptance_profile="factory_park_motion",
            acceptance_profile_version=1,
            frame_flush_timeout_s=0.01,
            sleep_s=0,
        ),
    )

    assert [item.name for item in result.maneuvers] == ["forward", "turn_left"]
    assert all(item.motion_verified for item in result.maneuvers)
    assert [item[1].payload for item in coordinator.commands] == [
        {"linear_x": 0.10, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.10, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.35},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.35},
    ]
    points = [
        json.loads(line)
        for line in result.trajectory_path.read_text(encoding="utf-8").splitlines()
    ]
    assert [point["phase"] for point in points] == [
        "initial",
        "maneuver",
        "maneuver",
        "transition",
        "maneuver",
        "maneuver",
    ]
    assert points[-1]["maneuver_name"] == "turn_left"
    assert points[-1]["quaternion_wxyz"] is not None
    evidence_document = json.loads(result.evidence_path.read_text(encoding="utf-8"))
    assert evidence_document["acceptance_profile"] == {
        "name": "factory_park_motion",
        "version": 1,
    }
    assert result.to_dict()["acceptance_profile"] == {
        "name": "factory_park_motion",
        "version": 1,
    }
    assert evidence_document["acceptance_thresholds"] == {
        "minimum_displacement_m": 0.05,
        "minimum_rotation_rad": 0.1,
        "maximum_turn_drift_m": 0.10,
    }
    turn = evidence_document["maneuvers"][1]
    assert turn["command"] == {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.35,
    }
    assert turn["measured_response"] == {
        "signed_translation_m": 0.0,
        "signed_yaw_rad": pytest.approx(0.35),
        "horizontal_drift_m": pytest.approx(0.0),
    }


def test_recording_does_not_precreate_the_coordinator_run_allocation(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "runs" / "fresh-allocation"
    frame_dir = run_dir / "logs" / "video-frames"
    coordinator = _AllocationSensitiveCoordinator(run_dir)

    def capture_fresh_frame() -> None:
        frame_dir.mkdir()
        (frame_dir / "frame_000000.png").write_bytes(b"\x89PNG\r\n\x1a\nframe")

    host = _RecordingHost(coordinator, on_prepare=capture_fresh_frame)
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=_RecordingPublisher(),
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=frame_dir, minimum_frames=1),
        run_id="fresh-allocation",
        session_id="motion-session",
        trajectory_path=run_dir / "motion-trajectory.jsonl",
        evidence_path=run_dir / "motion-evidence.json",
        manifest_path=run_dir / "session.runtime.json",
    )

    result = run_motion_recording(
        launch,
        MotionRecordingConfig(
            command=BaseTwist(linear_x=0.25),
            frames=1,
            steps_per_frame=10,
            refresh_steps=5,
            minimum_displacement_m=0.01,
            sleep_s=0,
        ),
    )

    assert result.frames.captured_count == 1
    assert host.closed is True


def test_recording_run_rejects_stale_frame_directory_without_deleting_it(
    tmp_path: Path,
) -> None:
    coordinator = _RecordingCoordinator()
    host = _RecordingHost(coordinator)
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    stale = frame_dir / "frame_000000.png"
    stale.write_bytes(b"\x89PNG\r\n\x1a\nstale")
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=_RecordingPublisher(),
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=frame_dir, minimum_frames=1),
        run_id="fixture-stale-capture",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=tmp_path / "motion-evidence.json",
        manifest_path=tmp_path / "session.runtime.json",
    )

    with pytest.raises(CoordinatorError, match="must be empty"):
        run_motion_recording(
            launch,
            MotionRecordingConfig(
                command=BaseTwist(linear_x=0.25),
                frames=1,
                steps_per_frame=10,
                refresh_steps=5,
                sleep_s=0,
            ),
        )

    assert stale.is_file()
    assert host.closed is True


def test_nonzero_recording_fails_closed_when_truth_did_not_move(tmp_path: Path) -> None:
    coordinator = _StationaryRecordingCoordinator()
    frame_dir = tmp_path / "frames"

    def capture_fresh_frame() -> None:
        frame_dir.mkdir(parents=True, exist_ok=True)
        (frame_dir / "frame_000000.png").write_bytes(b"\x89PNG\r\n\x1a\nframe")

    host = _RecordingHost(coordinator, on_prepare=capture_fresh_frame)
    evidence_path = tmp_path / "motion-evidence.json"
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=_RecordingPublisher(),
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=frame_dir, minimum_frames=1),
        run_id="fixture-stationary-failure",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=evidence_path,
        manifest_path=tmp_path / "session.runtime.json",
    )

    with pytest.raises(CoordinatorError, match="did not satisfy motion"):
        run_motion_recording(
            launch,
            MotionRecordingConfig(
                command=BaseTwist(linear_x=0.25),
                frames=2,
                steps_per_frame=2,
                refresh_steps=1,
                minimum_displacement_m=0.01,
                sleep_s=0,
            ),
        )

    assert evidence_path.is_file()
    assert '"motion_verified": false' in evidence_path.read_text(encoding="utf-8")
    assert host.closed is True
    assert coordinator.state is RuntimeState.STOPPED
    assert coordinator.last_failure_reason is not None
    assert "did not satisfy motion" in coordinator.last_failure_reason


def test_recording_run_closes_owned_runtime_after_mid_loop_failure(tmp_path: Path) -> None:
    coordinator = _FailingRecordingCoordinator()
    host = _RecordingHost(coordinator)
    launch = MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=_RecordingPublisher(),
        evidence_watchers=(),
        unreal_process=SimpleNamespace(poll=lambda: None),
        target=BaseTwistTarget("controller", "thunder_01", "base_twist"),
        base_body_id="thunder_01/base_link",
        capture=FrameCaptureOptions(directory=tmp_path / "frames"),
        run_id="fixture-failure",
        session_id="motion-session",
        trajectory_path=tmp_path / "motion-trajectory.jsonl",
        evidence_path=tmp_path / "motion-evidence.json",
        manifest_path=tmp_path / "session.runtime.json",
    )

    with pytest.raises(RuntimeError, match="fixture physics failure"):
        run_motion_recording(
            launch,
            MotionRecordingConfig(
                command=BaseTwist(linear_x=0.25),
                frames=1,
                steps_per_frame=1,
                refresh_steps=1,
                sleep_s=0,
            ),
        )

    assert host.closed is True
    assert coordinator.state is RuntimeState.STOPPED
    assert coordinator.last_failure_reason is not None
    assert "fixture physics failure" in coordinator.last_failure_reason


def test_create_recording_launch_assembles_production_runtime_and_ue_capture(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    session = tmp_path / "factory-park-motion-recording.session.yaml"
    session.write_text(
        """schema: lingtu.sim.session.v1
session_id: thunderv4_factory_park_hf_motion_recording_test
mujoco_version: 3.10.0
seed: 20260808
world: factory_park_hf@1.0.0
robots:
  - instance_id: thunder_01
    package: thunderv4@1.0.3
    sensor_rig: thunderv4_navigation@1.0.0
    controller: thunderv4_locomotion@1.0.0
    spawn:
      position_m: [0.0, -76.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
runtime:
  backend: mujoco
  mode: unreal
  required_bindings: [physics, visual, control]
""",
        encoding="utf-8",
    )
    bundle_dir = CatalogResolver.from_repository(repo_root).resolve(session).write_bundle(tmp_path / "bundle")
    bundle = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)
    run_root = tmp_path / "runs"
    capture = FrameCaptureOptions(
        directory=run_root / "production-recording" / "logs" / "frames",
        capture_every=3,
        maximum_frames=12,
        minimum_frames=2,
        motion_camera_stable_id="thunder_01/base_link",
    )

    launch = create_motion_recording_launch(
        bundle,
        runtime=MotionRecordingRuntimeConfig(
            repo_root=repo_root,
            run_root=run_root,
            mujoco_host=repo_root / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
            unreal_editor=Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
            uproject=repo_root / "sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
            run_id="production-recording",
            snapshot_port=25341,
            ready_timeout_s=120,
            sleep_s=0,
        ),
        capture=capture,
    )
    command = launch.unreal_process.command(
        bundle_dir=bundle.bundle_dir,
        allocation=SimpleNamespace(
            run_id="production-recording",
            path=run_root / "production-recording/run-allocation.json",
            artifact_root=repo_root,
            log_dir=run_root / "production-recording/logs",
        ),
        snapshot_port=25341,
        model_generation=0,
        reset_generation=0,
    )

    assert launch.coordinator._controller_factory is create_production_components
    assert launch.target.controller_id == "thunder_01.thunderv4_locomotion"
    assert launch.base_body_id == "thunder_01/base_link"
    assert f"-LingTuFrameCaptureDir={capture.directory}" in command
    assert "-LingTuRunId=production-recording" in command
    assert "-LingTuFrameCaptureEvery=3" in command
    assert "-LingTuFrameCaptureMax=12" in command
    assert "-LingTuMotionCameraStableId=thunder_01/base_link" in command


def test_create_recording_launch_accepts_one_required_truth_odometry_stream(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    session = tmp_path / "factory-park-truth-recording.session.yaml"
    session.write_text(
        """schema: lingtu.sim.session.v1
session_id: thunderv4_factory_park_truth_recording_test
mujoco_version: 3.10.0
seed: 20260809
world: factory_park_hf@1.0.0
robots:
  - instance_id: thunder_01
    package: thunderv4@1.0.3
    sensor_rig: thunderv4_truth_telemetry@1.0.0
    controller: thunderv4_locomotion@1.0.0
    spawn:
      position_m: [0.0, -76.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
runtime:
  backend: mujoco
  mode: unreal
  required_bindings: [physics, visual, sensors, control]
""",
        encoding="utf-8",
    )
    bundle_dir = CatalogResolver.from_repository(repo_root).resolve(session).write_bundle(tmp_path / "bundle")
    bundle = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)

    launch = create_motion_recording_launch(
        bundle,
        runtime=MotionRecordingRuntimeConfig(
            repo_root=repo_root,
            run_root=tmp_path / "runs",
            mujoco_host=repo_root / "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe",
            unreal_editor=Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
            uproject=repo_root / "sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
            run_id="truth-recording",
            snapshot_port=25342,
            truth_odom_publisher=repo_root
            / "build/mujoco_native_dds/Release/lingtu_truth_odom_publisher.exe",
        ),
        capture=FrameCaptureOptions(
            directory=tmp_path / "runs/truth-recording/logs/frames",
            capture_every=1,
            maximum_frames=2,
            minimum_frames=1,
            motion_camera_stable_id="thunder_01/base_link",
        ),
    )

    assert launch.session_id == bundle.session_id
    assert [stream["sensor_id"] for stream in bundle.plans["sensor.plan.json"]["streams"]["truth_odom"]] == [
        "thunder_01.truth_odom"
    ]


def test_canonical_factory_park_motion_session_is_recording_compatible(
    tmp_path: Path,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    session = (
        repo_root
        / "sim" / "sessions" / "examples"
        / "thunderv4_factory_park_motion"
        / "session.yaml"
    )

    resolved = CatalogResolver.from_repository(repo_root).resolve(session)
    bundle_dir = resolved.write_bundle(tmp_path / "bundle")
    (bundle_dir / "session.yaml").write_text(json.dumps(resolved.session), encoding="utf-8")
    bundle = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)

    _require_recording_bindings(bundle)
    assert bundle.session_spec["runtime"]["required_bindings"] == [
        "physics",
        "visual",
        "sensors",
        "control",
    ]
    assert [
        stream["sensor_id"]
        for stream in bundle.plans["sensor.plan.json"]["streams"]["truth_odom"]
    ] == ["thunder_01.truth_odom"]
    assert bundle.plans["visual.plan.json"]["world"]["package"] == {
        "kind": "world",
        "id": "factory_park_hf",
        "version": "1.0.0",
        "manifest": "sim/packages/worlds/factory_park_hf/world.package.yaml",
    }


def test_motion_recording_cli_exposes_production_capture_and_motion_camera() -> None:
    args = _parser().parse_args(
        [
            "bundle",
            "--mujoco-host",
            "mujoco.exe",
            "--unreal-editor",
            "UnrealEditor.exe",
            "--frame-capture-dir",
            "frames",
            "--linear-x",
            "0.25",
            "--motion-camera-stable-id",
            "thunder_01/base_link",
            "--truth-odom-publisher",
            "truth-odom.exe",
        ]
    )

    assert args.components == "production"
    assert args.linear_x == pytest.approx(0.25)
    assert args.motion_camera_stable_id == "thunder_01/base_link"
    assert args.frame_capture_dir == Path("frames")
    assert args.truth_odom_publisher == Path("truth-odom.exe")


def test_motion_recording_cli_accepts_a_repeatable_named_maneuver_sequence() -> None:
    args = _parser().parse_args(
        [
            "bundle",
            "--mujoco-host",
            "mujoco.exe",
            "--unreal-editor",
            "UnrealEditor.exe",
            "--maneuver",
            "forward:188:0.10:0:0",
            "--maneuver",
            "turn_left:188:0:0:0.35",
            "--transition-frames",
            "32",
            "--minimum-rotation-rad",
            "0.20",
            "--maximum-turn-drift-m",
            "0.08",
        ]
    )

    assert args.maneuvers == [
        ManeuverSegment("forward", BaseTwist(linear_x=0.10), frames=188),
        ManeuverSegment("turn_left", BaseTwist(angular_z=0.35), frames=188),
    ]
    assert args.transition_frames == 32
    assert args.minimum_rotation_rad == pytest.approx(0.20)
    assert args.maximum_turn_drift_m == pytest.approx(0.08)


def test_factory_park_cli_defaults_to_strict_turn_qualification() -> None:
    args = _parser().parse_args(
        [
            "bundle",
            "--mujoco-host",
            "mujoco.exe",
            "--unreal-editor",
            "UnrealEditor.exe",
        ]
    )

    assert args.acceptance_profile == "factory_park_motion"
    assert args.acceptance_profile_version == 1
    assert args.minimum_rotation_rad == pytest.approx(0.35)
    assert args.maximum_turn_drift_m == pytest.approx(0.10)


def test_python_config_keeps_explicit_legacy_compatibility_defaults() -> None:
    config = MotionRecordingConfig(command=BaseTwist(linear_x=0.25), frames=10)

    assert config.acceptance_profile == "legacy_motion"
    assert config.acceptance_profile_version == 1
    assert config.minimum_rotation_rad == pytest.approx(0.1)
    assert config.maximum_turn_drift_m is None
