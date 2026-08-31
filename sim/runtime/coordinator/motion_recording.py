"""Production-controlled MuJoCo-to-Unreal motion recording evidence."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

from sim.runtime.control import (
    CommandSubmitResult,
    ControllerCommand,
    GenerationStamp,
    create_production_components,
)
from sim.runtime.recording import (
    RECORDING_FILENAME,
    TIMELINE_FILENAME,
    SimulationRecordingWriter,
)
from sim.runtime.sensors import TruthOdometryEndpointFactory

from .controlled_run import BaseTwist, BaseTwistTarget, resolve_base_twist_target
from .coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from .external_evidence import ExternalEvidenceWatcher
from .live_snapshot import UdpLoopbackSnapshotPublisher
from .mujoco_process import MujocoProcess
from .run_allocation import (
    ResolvedSessionBundle,
    RunAllocationError,
    load_resolved_session_bundle,
)
from .session_host import SessionHost
from .unreal_process import UnrealProcess

LEGACY_MOTION_ACCEPTANCE_PROFILE = "legacy_motion"
FACTORY_PARK_MOTION_ACCEPTANCE_PROFILE = "factory_park_motion"
MOTION_ACCEPTANCE_PROFILE_VERSION = 1
FACTORY_PARK_MINIMUM_ROTATION_RAD = 0.35
FACTORY_PARK_MAXIMUM_TURN_DRIFT_M = 0.10


@dataclass(frozen=True)
class MotionEvidence:
    """Truth-derived proof for one commanded recording interval."""

    base_body_id: str
    start_position_m: tuple[float, float, float]
    end_position_m: tuple[float, float, float]
    horizontal_displacement_m: float
    minimum_displacement_m: float
    command_nonzero: bool
    moved: bool
    expectation_met: bool
    motion_verified: bool

    def to_dict(self) -> dict[str, Any]:
        """Return strict JSON-ready evidence."""

        return {
            "base_body_id": self.base_body_id,
            "start_position_m": list(self.start_position_m),
            "end_position_m": list(self.end_position_m),
            "horizontal_displacement_m": self.horizontal_displacement_m,
            "minimum_displacement_m": self.minimum_displacement_m,
            "command_nonzero": self.command_nonzero,
            "moved": self.moved,
            "expectation_met": self.expectation_met,
            "motion_verified": self.motion_verified,
        }


@dataclass(frozen=True)
class ManeuverEvidence:
    """Direction-aware Physics-truth proof for one named maneuver."""

    maneuver_index: int
    name: str
    command: BaseTwist
    start_frame_index: int
    end_frame_index: int
    start_sim_time_ns: int
    end_sim_time_ns: int
    base_body_id: str
    start_position_m: tuple[float, float, float]
    end_position_m: tuple[float, float, float]
    start_yaw_rad: float
    end_yaw_rad: float
    body_frame_forward_m: float
    body_frame_left_m: float
    horizontal_drift_m: float
    signed_translation_m: float
    signed_yaw_rad: float
    minimum_displacement_m: float
    minimum_rotation_rad: float
    maximum_turn_drift_m: float | None
    translation_commanded: bool
    rotation_commanded: bool
    translation_met: bool
    rotation_met: bool
    turn_drift_met: bool
    expectation_met: bool
    motion_verified: bool

    def to_dict(self) -> dict[str, Any]:
        """Return strict JSON-ready maneuver evidence."""

        return {
            "maneuver_index": self.maneuver_index,
            "name": self.name,
            "command": self.command.payload(),
            "start_frame_index": self.start_frame_index,
            "end_frame_index": self.end_frame_index,
            "start_sim_time_ns": self.start_sim_time_ns,
            "end_sim_time_ns": self.end_sim_time_ns,
            "base_body_id": self.base_body_id,
            "start_position_m": list(self.start_position_m),
            "end_position_m": list(self.end_position_m),
            "start_yaw_rad": self.start_yaw_rad,
            "end_yaw_rad": self.end_yaw_rad,
            "body_frame_forward_m": self.body_frame_forward_m,
            "body_frame_left_m": self.body_frame_left_m,
            "horizontal_drift_m": self.horizontal_drift_m,
            "signed_translation_m": self.signed_translation_m,
            "signed_yaw_rad": self.signed_yaw_rad,
            "minimum_displacement_m": self.minimum_displacement_m,
            "minimum_rotation_rad": self.minimum_rotation_rad,
            "maximum_turn_drift_m": self.maximum_turn_drift_m,
            "translation_commanded": self.translation_commanded,
            "rotation_commanded": self.rotation_commanded,
            "translation_met": self.translation_met,
            "rotation_met": self.rotation_met,
            "turn_drift_met": self.turn_drift_met,
            "expectation_met": self.expectation_met,
            "motion_verified": self.motion_verified,
            "measured_response": {
                "signed_translation_m": self.signed_translation_m,
                "signed_yaw_rad": self.signed_yaw_rad,
                "horizontal_drift_m": self.horizontal_drift_m,
            },
        }


@dataclass(frozen=True)
class FrameCaptureOptions:
    """Bounded RobotSimUE frame-sequence request and proof threshold."""

    directory: Path
    capture_every: int = 1
    maximum_frames: int = 120
    minimum_frames: int = 2
    session_camera_tag: str | None = None
    motion_camera_stable_id: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "directory", Path(self.directory).resolve())
        for field in ("capture_every", "maximum_frames", "minimum_frames"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{field} must be a positive integer")
        if self.minimum_frames > self.maximum_frames:
            raise ValueError("minimum_frames must not exceed maximum_frames")
        for field in ("session_camera_tag", "motion_camera_stable_id"):
            value = getattr(self, field)
            if value is not None and (not isinstance(value, str) or not value or value != value.strip()):
                raise ValueError(f"{field} must be a non-empty trimmed string when set")


@dataclass(frozen=True)
class ManeuverSegment:
    """One named, bounded base-twist interval in a recording sequence."""

    name: str
    command: BaseTwist
    frames: int

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name or self.name != self.name.strip():
            raise ValueError("maneuver name must be a non-empty trimmed string")
        if not isinstance(self.command, BaseTwist):
            raise ValueError("maneuver command must be a BaseTwist")
        if isinstance(self.frames, bool) or not isinstance(self.frames, int) or self.frames <= 0:
            raise ValueError("maneuver frames must be a positive integer")


@dataclass(frozen=True)
class MotionRecordingConfig:
    """One bounded production-controller recording request."""

    command: BaseTwist | None = None
    frames: int | None = None
    steps_per_frame: int = 8
    refresh_steps: int = 25
    minimum_displacement_m: float = 0.05
    minimum_rotation_rad: float = 0.1
    maximum_turn_drift_m: float | None = None
    acceptance_profile: str = LEGACY_MOTION_ACCEPTANCE_PROFILE
    acceptance_profile_version: int = MOTION_ACCEPTANCE_PROFILE_VERSION
    frame_flush_timeout_s: float = 10.0
    sleep_s: float = 0.01
    maneuvers: tuple[ManeuverSegment, ...] = ()
    transition_frames: int = 0

    def __post_init__(self) -> None:
        for field in ("steps_per_frame", "refresh_steps"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{field} must be a positive integer")
        maneuvers = tuple(self.maneuvers)
        object.__setattr__(self, "maneuvers", maneuvers)
        if maneuvers:
            if self.command is not None or self.frames is not None:
                raise ValueError("maneuvers cannot be combined with legacy command or frames")
            if any(not isinstance(segment, ManeuverSegment) for segment in maneuvers):
                raise ValueError("maneuvers must contain only ManeuverSegment values")
            names = [segment.name for segment in maneuvers]
            if len(names) != len(set(names)):
                raise ValueError("maneuver names must be unique")
        else:
            if not isinstance(self.command, BaseTwist):
                raise ValueError("command must be a BaseTwist when maneuvers are not provided")
            if isinstance(self.frames, bool) or not isinstance(self.frames, int) or self.frames <= 0:
                raise ValueError("frames must be a positive integer when maneuvers are not provided")
        if (
            isinstance(self.transition_frames, bool)
            or not isinstance(self.transition_frames, int)
            or self.transition_frames < 0
        ):
            raise ValueError("transition_frames must be a non-negative integer")
        minimum = _finite_number(self.minimum_displacement_m, "minimum_displacement_m")
        if minimum <= 0.0:
            raise ValueError("minimum_displacement_m must be positive")
        object.__setattr__(self, "minimum_displacement_m", minimum)
        minimum_rotation = _finite_number(self.minimum_rotation_rad, "minimum_rotation_rad")
        if minimum_rotation <= 0.0:
            raise ValueError("minimum_rotation_rad must be positive")
        object.__setattr__(self, "minimum_rotation_rad", minimum_rotation)
        if self.maximum_turn_drift_m is not None:
            maximum_turn_drift = _finite_number(
                self.maximum_turn_drift_m,
                "maximum_turn_drift_m",
            )
            if maximum_turn_drift <= 0.0:
                raise ValueError("maximum_turn_drift_m must be positive when set")
            object.__setattr__(self, "maximum_turn_drift_m", maximum_turn_drift)
        if (
            not isinstance(self.acceptance_profile, str)
            or not self.acceptance_profile
            or self.acceptance_profile != self.acceptance_profile.strip()
        ):
            raise ValueError("acceptance_profile must be a non-empty trimmed string")
        if (
            isinstance(self.acceptance_profile_version, bool)
            or not isinstance(self.acceptance_profile_version, int)
            or self.acceptance_profile_version <= 0
        ):
            raise ValueError("acceptance_profile_version must be a positive integer")
        timeout = _finite_number(self.frame_flush_timeout_s, "frame_flush_timeout_s")
        if timeout < 0.0:
            raise ValueError("frame_flush_timeout_s must be non-negative")
        object.__setattr__(self, "frame_flush_timeout_s", timeout)
        sleep_s = _finite_number(self.sleep_s, "sleep_s")
        if sleep_s < 0.0:
            raise ValueError("sleep_s must be non-negative")
        object.__setattr__(self, "sleep_s", sleep_s)

    @property
    def sequence(self) -> tuple[ManeuverSegment, ...]:
        """Return the effective ordered sequence, including legacy single motion."""

        if self.maneuvers:
            return self.maneuvers
        command = self.command
        frames = self.frames
        if command is None or frames is None:
            raise ValueError("legacy motion recording requires command and frames")
        return (ManeuverSegment("motion", command, frames),)

    @property
    def total_frames(self) -> int:
        """Return commanded plus between-segment transition frame count."""

        sequence = self.sequence
        return sum(segment.frames for segment in sequence) + self.transition_frames * max(0, len(sequence) - 1)


@dataclass(frozen=True)
class MotionRecordingRuntimeConfig:
    """Run-local executable paths and bounded readiness settings."""

    repo_root: Path
    run_root: Path
    mujoco_host: Path
    unreal_editor: Path
    uproject: Path
    run_id: str | None = None
    snapshot_port: int = 25123
    dds_domain: int = 0
    ready_timeout_s: float = 180.0
    warmup_steps: int = 8
    sleep_s: float = 0.01
    truth_odom_publisher: Path | None = None
    truth_odom_parent_frame: str = "map"

    def __post_init__(self) -> None:
        for field in (
            "repo_root",
            "run_root",
            "mujoco_host",
            "unreal_editor",
            "uproject",
        ):
            object.__setattr__(self, field, Path(getattr(self, field)).resolve())
        if self.truth_odom_publisher is not None:
            object.__setattr__(
                self,
                "truth_odom_publisher",
                Path(self.truth_odom_publisher).resolve(),
            )
        if (
            not isinstance(self.truth_odom_parent_frame, str)
            or not self.truth_odom_parent_frame
            or self.truth_odom_parent_frame != self.truth_odom_parent_frame.strip()
            or any(character.isspace() for character in self.truth_odom_parent_frame)
        ):
            raise ValueError("truth_odom_parent_frame must be one non-empty frame token")
        if self.run_id is not None and (
            not isinstance(self.run_id, str) or not self.run_id or self.run_id != self.run_id.strip()
        ):
            raise ValueError("run_id must be a non-empty trimmed string when set")
        if (
            isinstance(self.snapshot_port, bool)
            or not isinstance(self.snapshot_port, int)
            or not 1 <= self.snapshot_port <= 65535
        ):
            raise ValueError("snapshot_port must be an integer in [1, 65535]")
        if isinstance(self.dds_domain, bool) or not isinstance(self.dds_domain, int) or self.dds_domain < 0:
            raise ValueError("dds_domain must be a non-negative integer")
        if isinstance(self.warmup_steps, bool) or not isinstance(self.warmup_steps, int) or self.warmup_steps <= 0:
            raise ValueError("warmup_steps must be a positive integer")
        ready_timeout = _finite_number(self.ready_timeout_s, "ready_timeout_s")
        if ready_timeout <= 0.0:
            raise ValueError("ready_timeout_s must be positive")
        object.__setattr__(self, "ready_timeout_s", ready_timeout)
        sleep_s = _finite_number(self.sleep_s, "sleep_s")
        if sleep_s < 0.0:
            raise ValueError("sleep_s must be non-negative")
        object.__setattr__(self, "sleep_s", sleep_s)


@dataclass(frozen=True)
class CapturedFrameMetadata:
    """Observed, non-empty PNG sequence produced by RobotSimUE."""

    directory: Path
    captured_count: int
    total_bytes: int
    first_frame: Path | None
    last_frame: Path | None
    capture_every: int
    maximum_frames: int
    minimum_frames: int
    session_camera_tag: str | None
    motion_camera_stable_id: str | None
    capture_started_at_ns: int

    def to_dict(self) -> dict[str, Any]:
        """Return strict JSON-ready frame metadata."""

        return {
            "directory": str(self.directory),
            "captured_count": self.captured_count,
            "total_bytes": self.total_bytes,
            "first_frame": str(self.first_frame) if self.first_frame else None,
            "last_frame": str(self.last_frame) if self.last_frame else None,
            "capture_every": self.capture_every,
            "maximum_frames": self.maximum_frames,
            "minimum_frames": self.minimum_frames,
            "session_camera_tag": self.session_camera_tag,
            "motion_camera_stable_id": self.motion_camera_stable_id,
            "capture_started_at_ns": self.capture_started_at_ns,
        }


@dataclass(frozen=True)
class MotionRecordingLaunch:
    """Owned runtime objects and artifact paths for one recording run."""

    host: Any
    coordinator: Any
    publisher: Any
    evidence_watchers: tuple[Any, ...]
    unreal_process: Any
    target: BaseTwistTarget
    base_body_id: str
    capture: FrameCaptureOptions
    run_id: str
    session_id: str
    trajectory_path: Path
    evidence_path: Path
    manifest_path: Path


@dataclass(frozen=True)
class MotionRecordingResult:
    """Machine-readable artifacts from one closed recording session."""

    run_id: str
    session_id: str
    controller: BaseTwistTarget
    acceptance_profile: str
    acceptance_profile_version: int
    motion: MotionEvidence | None
    maneuvers: tuple[ManeuverEvidence, ...]
    frames: CapturedFrameMetadata
    frames_published: int
    trajectory_path: Path
    evidence_path: Path
    manifest_path: Path
    recording_manifest_path: Path
    recording_timeline_path: Path

    def to_dict(self) -> dict[str, Any]:
        """Return strict JSON-ready run evidence."""

        return {
            "run_id": self.run_id,
            "session_id": self.session_id,
            "components": "production",
            "acceptance_profile": {
                "name": self.acceptance_profile,
                "version": self.acceptance_profile_version,
            },
            "controller": {
                "controller_id": self.controller.controller_id,
                "instance_id": self.controller.instance_id,
                "channel_id": self.controller.channel_id,
            },
            "motion": self.motion.to_dict() if self.motion is not None else None,
            "maneuvers": [maneuver.to_dict() for maneuver in self.maneuvers],
            "frames": self.frames.to_dict(),
            "frames_published": self.frames_published,
            "trajectory": str(self.trajectory_path),
            "evidence": str(self.evidence_path),
            "runtime_manifest": str(self.manifest_path),
            "recording_manifest": str(self.recording_manifest_path),
            "recording_timeline": str(self.recording_timeline_path),
        }


def create_motion_recording_launch(
    bundle: ResolvedSessionBundle,
    *,
    runtime: MotionRecordingRuntimeConfig,
    capture: FrameCaptureOptions,
    controller_id: str | None = None,
) -> MotionRecordingLaunch:
    """Assemble one production-controller MuJoCo-to-Unreal recording run."""

    required_bindings = _require_recording_bindings(bundle)
    run_id = runtime.run_id or f"motion-recording-{uuid.uuid4().hex[:12]}"
    run_dir = runtime.run_root / run_id
    level = _unreal_level(bundle)
    target = resolve_base_twist_target(bundle.bundle_dir, controller_id)
    base_body_id = _base_body_id(bundle, target.instance_id)
    physics_host = MujocoProcess(runtime.mujoco_host)
    sensor_endpoint_factory = None
    if "sensors" in required_bindings:
        _require_truth_odometry_only_sensor_plan(bundle)
        if runtime.truth_odom_publisher is None:
            raise CoordinatorError(
                "sensor-bound motion recording requires truth_odom_publisher"
            )
        sensor_endpoint_factory = TruthOdometryEndpointFactory(
            runtime.truth_odom_publisher,
            parent_frame=runtime.truth_odom_parent_frame,
        )
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle.bundle_dir,
        repo_root=runtime.repo_root,
        run_root=runtime.run_root,
        physics_host=physics_host,
        controller_factory=create_production_components,
        sensor_endpoint_factory=sensor_endpoint_factory,
        run_id=run_id,
        dds_domain=runtime.dds_domain,
        ports={"visual_snapshot_udp": runtime.snapshot_port},
    )
    visual_evidence_path = run_dir / "logs" / "visual-readiness.json"
    sensor_evidence_path = run_dir / "logs" / "sensor-readiness.json"
    watchers = (
        ExternalEvidenceWatcher(
            visual_evidence_path,
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-visual",
        ),
        ExternalEvidenceWatcher(
            sensor_evidence_path,
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-camera",
        ),
    )
    unreal = UnrealProcess(
        runtime.unreal_editor,
        runtime.uproject,
        level,
        frame_capture_dir=capture.directory,
        frame_capture_every=capture.capture_every,
        frame_capture_max=capture.maximum_frames,
        session_camera_tag=capture.session_camera_tag,
        motion_camera_stable_id=capture.motion_camera_stable_id,
    )
    publisher = UdpLoopbackSnapshotPublisher(runtime.snapshot_port)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=unreal,
        publisher=publisher,
        evidence_watchers=watchers,
        snapshot_port=runtime.snapshot_port,
        warmup_steps=runtime.warmup_steps,
        ready_timeout_s=runtime.ready_timeout_s,
        sleep_s=runtime.sleep_s,
    )
    return MotionRecordingLaunch(
        host=host,
        coordinator=coordinator,
        publisher=publisher,
        evidence_watchers=watchers,
        unreal_process=unreal,
        target=target,
        base_body_id=base_body_id,
        capture=capture,
        run_id=run_id,
        session_id=bundle.session_id,
        trajectory_path=run_dir / "motion-trajectory.jsonl",
        evidence_path=run_dir / "motion-evidence.json",
        manifest_path=coordinator.manifest_path,
    )


def run_motion_recording(
    launch: MotionRecordingLaunch,
    config: MotionRecordingConfig,
) -> MotionRecordingResult:
    """Run production control, stream visual truth, and close every runtime."""

    trajectory_path = Path(launch.trajectory_path).resolve()
    evidence_path = Path(launch.evidence_path).resolve()
    recording_root = trajectory_path.parent
    recording_manifest_path = recording_root / RECORDING_FILENAME
    recording_timeline_path = recording_root / TIMELINE_FILENAME
    sequence_mode = bool(config.maneuvers)
    current: dict[str, Any]
    start_snapshot: dict[str, Any]
    motion: MotionEvidence | None = None
    maneuver_evidence: list[ManeuverEvidence] = []
    frames_published = 0
    frame_index = 0
    command_sequence = 0
    original_error: BaseException | None = None
    acceptance_error: CoordinatorError | None = None
    try:
        capture_started_at_ns = _prepare_capture_directory(launch.capture.directory)
        launch.host.prepare()
        if launch.coordinator.state is not RuntimeState.READY:
            raise CoordinatorError("motion recording requires a READY session after prepare")
        trajectory_path.parent.mkdir(parents=True, exist_ok=True)
        evidence_path.parent.mkdir(parents=True, exist_ok=True)
        current = dict(launch.coordinator.snapshot())
        start_snapshot = current
        with (
            SimulationRecordingWriter(
                recording_root,
                run_id=launch.run_id,
                session_id=launch.session_id,
            ) as recording,
            trajectory_path.open("w", encoding="utf-8", newline="\n") as trajectory,
        ):
            _write_trajectory_point(
                trajectory,
                current,
                launch.base_body_id,
                frame_index=0,
                phase="initial",
                maneuver_index=None,
                maneuver_name=None,
                phase_frame_index=0,
                command=BaseTwist(),
                schema_version=2 if sequence_mode else 1,
            )
            recording.append(
                current,
                command=_recording_command(launch, BaseTwist(), sequence=0),
                metadata={"phase": "initial", "frame_index": 0},
            )
            launch.coordinator.start()
            sequence = config.sequence
            for segment_index, segment in enumerate(sequence):
                segment_start_frame = frame_index
                segment_snapshots: list[Mapping[str, Any]] = [current]
                for segment_frame_index in range(1, segment.frames + 1):
                    current, command_sequence = _advance_recording_command(
                        launch,
                        current,
                        segment.command,
                        command_sequence=command_sequence,
                        steps=config.steps_per_frame,
                        refresh_steps=config.refresh_steps,
                    )
                    frame_index += 1
                    _publish_recording_snapshot(launch, current)
                    frames_published += 1
                    segment_snapshots.append(current)
                    _write_trajectory_point(
                        trajectory,
                        current,
                        launch.base_body_id,
                        frame_index=frame_index,
                        phase="maneuver",
                        maneuver_index=segment_index,
                        maneuver_name=segment.name,
                        phase_frame_index=segment_frame_index,
                        command=segment.command,
                        schema_version=2 if sequence_mode else 1,
                    )
                    recording.append(
                        current,
                        command=_recording_command(
                            launch,
                            segment.command,
                            sequence=command_sequence,
                        ),
                        metadata={
                            "phase": "maneuver",
                            "frame_index": frame_index,
                            "maneuver_index": segment_index,
                            "maneuver_name": segment.name,
                        },
                    )
                    if config.sleep_s:
                        time.sleep(config.sleep_s)

                maneuver_evidence.append(
                    evaluate_maneuver_evidence(
                        segment_snapshots,
                        maneuver_index=segment_index,
                        maneuver=segment,
                        base_body_id=launch.base_body_id,
                        start_frame_index=segment_start_frame,
                        end_frame_index=frame_index,
                        minimum_displacement_m=config.minimum_displacement_m,
                        minimum_rotation_rad=config.minimum_rotation_rad,
                        maximum_turn_drift_m=config.maximum_turn_drift_m,
                    )
                )

                if segment_index == len(sequence) - 1:
                    continue
                for transition_frame_index in range(1, config.transition_frames + 1):
                    current, command_sequence = _advance_recording_command(
                        launch,
                        current,
                        BaseTwist(),
                        command_sequence=command_sequence,
                        steps=config.steps_per_frame,
                        refresh_steps=config.refresh_steps,
                    )
                    frame_index += 1
                    _publish_recording_snapshot(launch, current)
                    frames_published += 1
                    _write_trajectory_point(
                        trajectory,
                        current,
                        launch.base_body_id,
                        frame_index=frame_index,
                        phase="transition",
                        maneuver_index=None,
                        maneuver_name=None,
                        phase_frame_index=transition_frame_index,
                        command=BaseTwist(),
                        schema_version=2,
                        transition_from=segment.name,
                        transition_to=sequence[segment_index + 1].name,
                    )
                    recording.append(
                        current,
                        command=_recording_command(
                            launch,
                            BaseTwist(),
                            sequence=command_sequence,
                        ),
                        metadata={
                            "phase": "transition",
                            "frame_index": frame_index,
                            "transition_from": segment.name,
                            "transition_to": sequence[segment_index + 1].name,
                        },
                    )
                    if config.sleep_s:
                        time.sleep(config.sleep_s)

        launch.coordinator.register_episode_artifact(
            "simulation_recording",
            recording_manifest_path.name,
        )
        launch.coordinator.register_episode_artifact(
            "simulation_timeline",
            recording_timeline_path.name,
        )
        if not sequence_mode:
            if config.command is None:
                raise CoordinatorError("legacy motion recording has no command")
            motion = evaluate_motion_evidence(
                start_snapshot,
                current,
                base_body_id=launch.base_body_id,
                command=config.command,
                minimum_displacement_m=config.minimum_displacement_m,
            )
        frames = _wait_for_captured_frames(
            launch.capture,
            timeout_s=config.frame_flush_timeout_s,
            capture_started_at_ns=capture_started_at_ns,
        )
        failed_maneuvers = [
            item.name for item in maneuver_evidence if not item.expectation_met
        ]
        if sequence_mode and failed_maneuvers:
            acceptance_error = CoordinatorError(
                "truth evidence did not satisfy maneuvers: "
                + ", ".join(failed_maneuvers)
            )
        elif motion is not None and not motion.expectation_met:
            expectation = "motion" if motion.command_nonzero else "stationary control"
            acceptance_error = CoordinatorError(
                f"truth evidence did not satisfy {expectation}: "
                f"horizontal displacement={motion.horizontal_displacement_m:.6f} m"
            )
    except BaseException as exc:
        original_error = exc
        raise
    finally:
        terminal_error = original_error or acceptance_error
        try:
            if terminal_error is None:
                launch.host.close()
            else:
                message = str(terminal_error).strip()
                reason = (
                    f"{type(terminal_error).__name__}: {message}"
                    if message
                    else type(terminal_error).__name__
                )
                launch.host.stop(failure_reason=reason)
        except Exception as close_error:
            if terminal_error is None:
                raise
            add_note = getattr(terminal_error, "add_note", None)
            if callable(add_note):
                add_note(f"motion recording cleanup failed: {close_error}")

    frames = _captured_frame_metadata(
        launch.capture,
        capture_started_at_ns=capture_started_at_ns,
    )
    document: dict[str, Any] = {
        "schema": (
            "lingtu.sim.motion-recording-evidence.v2"
            if sequence_mode
            else "lingtu.sim.motion-recording-evidence.v1"
        ),
        "run_id": launch.run_id,
        "session_id": launch.session_id,
        "controller": {
            "controller_id": launch.target.controller_id,
            "instance_id": launch.target.instance_id,
            "channel_id": launch.target.channel_id,
        },
        "command": config.command.payload() if config.command is not None else None,
        "maneuver_sequence": (
            {
                "steps_per_frame": config.steps_per_frame,
                "transition_frames": config.transition_frames,
                "total_frames": config.total_frames,
            }
            if sequence_mode
            else None
        ),
        "acceptance_profile": {
            "name": config.acceptance_profile,
            "version": config.acceptance_profile_version,
        },
        "acceptance_thresholds": {
            "minimum_displacement_m": config.minimum_displacement_m,
            "minimum_rotation_rad": config.minimum_rotation_rad,
            "maximum_turn_drift_m": config.maximum_turn_drift_m,
        },
        "frames_published": frames_published,
        "motion": motion.to_dict() if motion is not None else None,
        "maneuvers": [item.to_dict() for item in maneuver_evidence],
        "frames": frames.to_dict(),
        "trajectory": str(trajectory_path),
        "runtime_manifest": str(Path(launch.manifest_path).resolve()),
        "recording_manifest": str(recording_manifest_path),
        "recording_timeline": str(recording_timeline_path),
    }
    _write_json(evidence_path, document)
    if acceptance_error is not None:
        raise acceptance_error
    return MotionRecordingResult(
        run_id=launch.run_id,
        session_id=launch.session_id,
        controller=launch.target,
        acceptance_profile=config.acceptance_profile,
        acceptance_profile_version=config.acceptance_profile_version,
        motion=motion,
        maneuvers=tuple(maneuver_evidence),
        frames=frames,
        frames_published=frames_published,
        trajectory_path=trajectory_path,
        evidence_path=evidence_path,
        manifest_path=Path(launch.manifest_path).resolve(),
        recording_manifest_path=recording_manifest_path,
        recording_timeline_path=recording_timeline_path,
    )


def _recording_command(
    launch: MotionRecordingLaunch,
    command: BaseTwist,
    *,
    sequence: int,
) -> dict[str, Any]:
    return {
        "controller_id": launch.target.controller_id,
        "instance_id": launch.target.instance_id,
        "channel_id": launch.target.channel_id,
        "sequence": sequence,
        "payload": command.payload(),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record production-controller MuJoCo truth rendered by RobotSimUE "
            "and emit finite motion plus frame evidence."
        )
    )
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation"))
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument("--unreal-editor", type=Path, required=True)
    parser.add_argument("--uproject", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--snapshot-port", type=int, default=25123)
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument("--truth-odom-publisher", type=Path)
    parser.add_argument("--controller-id")
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--steps-per-frame", type=int, default=8)
    parser.add_argument("--refresh-steps", type=int, default=25)
    parser.add_argument("--linear-x", type=float, default=0.25)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument(
        "--maneuver",
        dest="maneuvers",
        action="append",
        type=_parse_maneuver,
        help=(
            "repeatable NAME:FRAMES:LINEAR_X:LINEAR_Y:ANGULAR_Z segment; "
            "when present, replaces the legacy single command"
        ),
    )
    parser.add_argument("--transition-frames", type=int, default=0)
    parser.add_argument("--minimum-displacement-m", type=float, default=0.1)
    parser.add_argument(
        "--minimum-rotation-rad",
        type=float,
        default=FACTORY_PARK_MINIMUM_ROTATION_RAD,
    )
    parser.add_argument(
        "--maximum-turn-drift-m",
        type=float,
        default=FACTORY_PARK_MAXIMUM_TURN_DRIFT_M,
    )
    parser.set_defaults(
        acceptance_profile=FACTORY_PARK_MOTION_ACCEPTANCE_PROFILE,
        acceptance_profile_version=MOTION_ACCEPTANCE_PROFILE_VERSION,
    )
    parser.add_argument("--ready-timeout-s", type=float, default=180.0)
    parser.add_argument("--warmup-steps", type=int, default=8)
    parser.add_argument("--sleep-s", type=float, default=0.01)
    parser.add_argument("--frame-capture-dir", type=Path)
    parser.add_argument("--frame-capture-every", type=int, default=3)
    parser.add_argument("--frame-capture-max", type=int, default=120)
    parser.add_argument("--minimum-frames", type=int, default=10)
    parser.add_argument("--frame-flush-timeout-s", type=float, default=20.0)
    parser.add_argument("--session-camera-tag")
    parser.add_argument("--motion-camera-stable-id")
    parser.add_argument(
        "--components",
        choices=("production",),
        default="production",
        help="motion recording is intentionally production-controller only",
    )
    return parser


def _parse_maneuver(value: str) -> ManeuverSegment:
    parts = value.split(":")
    if len(parts) != 5:
        raise argparse.ArgumentTypeError(
            "maneuver must be NAME:FRAMES:LINEAR_X:LINEAR_Y:ANGULAR_Z"
        )
    name, frames_text, linear_x_text, linear_y_text, angular_z_text = parts
    try:
        return ManeuverSegment(
            name,
            BaseTwist(
                linear_x=float(linear_x_text),
                linear_y=float(linear_y_text),
                angular_z=float(angular_z_text),
            ),
            frames=int(frames_text),
        )
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def main(argv: list[str] | None = None) -> int:
    """Run one bounded production motion recording from the command line."""

    args = _parser().parse_args(argv)
    repo_root = args.repo_root.resolve()
    run_root = args.run_root.resolve()
    run_id = args.run_id or f"motion-recording-{uuid.uuid4().hex[:12]}"
    frame_dir = (
        args.frame_capture_dir.resolve()
        if args.frame_capture_dir is not None
        else run_root / run_id / "logs" / "video-frames"
    )
    uproject = (
        args.uproject.resolve()
        if args.uproject is not None
        else repo_root / "sim" / "runtime" / "visual" / "RobotSimUE" / "RobotSimUE.uproject"
    )
    try:
        bundle = load_resolved_session_bundle(args.bundle, repo_root=repo_root)
        capture = FrameCaptureOptions(
            directory=frame_dir,
            capture_every=args.frame_capture_every,
            maximum_frames=args.frame_capture_max,
            minimum_frames=args.minimum_frames,
            session_camera_tag=args.session_camera_tag,
            motion_camera_stable_id=args.motion_camera_stable_id,
        )
        launch = create_motion_recording_launch(
            bundle,
            runtime=MotionRecordingRuntimeConfig(
                repo_root=repo_root,
                run_root=run_root,
                mujoco_host=args.mujoco_host,
                unreal_editor=args.unreal_editor,
                uproject=uproject,
                run_id=run_id,
                snapshot_port=args.snapshot_port,
                dds_domain=args.dds_domain,
                truth_odom_publisher=args.truth_odom_publisher,
                ready_timeout_s=args.ready_timeout_s,
                warmup_steps=args.warmup_steps,
                sleep_s=args.sleep_s,
            ),
            capture=capture,
            controller_id=args.controller_id,
        )
        result = run_motion_recording(
            launch,
            MotionRecordingConfig(
                command=(
                    None
                    if args.maneuvers
                    else BaseTwist(
                        linear_x=args.linear_x,
                        linear_y=args.linear_y,
                        angular_z=args.angular_z,
                    )
                ),
                frames=None if args.maneuvers else args.frames,
                steps_per_frame=args.steps_per_frame,
                refresh_steps=args.refresh_steps,
                minimum_displacement_m=args.minimum_displacement_m,
                minimum_rotation_rad=args.minimum_rotation_rad,
                maximum_turn_drift_m=args.maximum_turn_drift_m,
                acceptance_profile=args.acceptance_profile,
                acceptance_profile_version=args.acceptance_profile_version,
                frame_flush_timeout_s=args.frame_flush_timeout_s,
                sleep_s=args.sleep_s,
                maneuvers=tuple(args.maneuvers or ()),
                transition_frames=args.transition_frames,
            ),
        )
    except (CoordinatorError, RunAllocationError, OSError, RuntimeError, ValueError) as exc:
        print(
            json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False),
            file=sys.stderr,
        )
        return 1

    print(json.dumps({"ok": True, **result.to_dict()}, ensure_ascii=False, indent=2))
    return 0


def evaluate_motion_evidence(
    start_snapshot: Mapping[str, Any],
    end_snapshot: Mapping[str, Any],
    *,
    base_body_id: str,
    command: BaseTwist,
    minimum_displacement_m: float,
) -> MotionEvidence:
    """Compare immutable Physics truth snapshots against the requested motion."""

    minimum = _finite_number(minimum_displacement_m, "minimum_displacement_m")
    if minimum <= 0.0:
        raise ValueError("minimum_displacement_m must be positive")
    start = _body_position(start_snapshot, base_body_id)
    end = _body_position(end_snapshot, base_body_id)
    displacement = math.hypot(end[0] - start[0], end[1] - start[1])
    command_nonzero = any(value != 0.0 for value in (command.linear_x, command.linear_y, command.angular_z))
    moved = displacement >= minimum
    return MotionEvidence(
        base_body_id=base_body_id,
        start_position_m=start,
        end_position_m=end,
        horizontal_displacement_m=displacement,
        minimum_displacement_m=minimum,
        command_nonzero=command_nonzero,
        moved=moved,
        expectation_met=moved if command_nonzero else not moved,
        motion_verified=command_nonzero and moved,
    )


def evaluate_maneuver_evidence(
    snapshots: Sequence[Mapping[str, Any]],
    *,
    maneuver_index: int,
    maneuver: ManeuverSegment,
    base_body_id: str,
    start_frame_index: int,
    end_frame_index: int,
    minimum_displacement_m: float,
    minimum_rotation_rad: float,
    maximum_turn_drift_m: float | None = None,
) -> ManeuverEvidence:
    """Evaluate one maneuver in its starting body frame with unwrapped yaw."""

    if isinstance(maneuver_index, bool) or not isinstance(maneuver_index, int) or maneuver_index < 0:
        raise ValueError("maneuver_index must be a non-negative integer")
    for field, value in (("start_frame_index", start_frame_index), ("end_frame_index", end_frame_index)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{field} must be a non-negative integer")
    if end_frame_index <= start_frame_index:
        raise ValueError("end_frame_index must be greater than start_frame_index")
    samples = tuple(snapshots)
    expected_frame_span = maneuver.frames
    if end_frame_index - start_frame_index != expected_frame_span:
        raise ValueError("maneuver evidence frame span must match the commanded frames")
    expected_sample_count = expected_frame_span + 1
    if len(samples) != expected_sample_count:
        raise ValueError(
            "maneuver evidence requires a complete truth snapshot sequence: "
            f"expected {expected_sample_count}, got {len(samples)}"
        )
    sim_times_ns = tuple(_event_integer(sample, "sim_time_ns") for sample in samples)
    if any(current <= previous for previous, current in pairwise(sim_times_ns)):
        raise ValueError("maneuver truth snapshots require strictly increasing sim_time_ns")
    minimum_translation = _finite_number(minimum_displacement_m, "minimum_displacement_m")
    if minimum_translation <= 0.0:
        raise ValueError("minimum_displacement_m must be positive")
    minimum_rotation = _finite_number(minimum_rotation_rad, "minimum_rotation_rad")
    if minimum_rotation <= 0.0:
        raise ValueError("minimum_rotation_rad must be positive")
    maximum_turn_drift = (
        _finite_number(maximum_turn_drift_m, "maximum_turn_drift_m")
        if maximum_turn_drift_m is not None
        else None
    )
    if maximum_turn_drift is not None and maximum_turn_drift <= 0.0:
        raise ValueError("maximum_turn_drift_m must be positive when set")

    start = _body_position(samples[0], base_body_id)
    end = _body_position(samples[-1], base_body_id)
    yaws = tuple(_body_yaw(snapshot, base_body_id) for snapshot in samples)
    start_yaw = yaws[0]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    forward = cosine * dx + sine * dy
    left = -sine * dx + cosine * dy
    linear_x = maneuver.command.linear_x
    linear_y = maneuver.command.linear_y
    linear_norm = math.hypot(linear_x, linear_y)
    translation_commanded = linear_norm > 0.0
    signed_translation = (
        (forward * linear_x + left * linear_y) / linear_norm
        if translation_commanded
        else 0.0
    )
    yaw_delta = sum(_wrapped_angle(current - previous) for previous, current in pairwise(yaws))
    rotation_commanded = maneuver.command.angular_z != 0.0
    signed_yaw = (
        yaw_delta * (1.0 if maneuver.command.angular_z > 0.0 else -1.0)
        if rotation_commanded
        else yaw_delta
    )
    translation_met = not translation_commanded or signed_translation >= minimum_translation
    rotation_met = not rotation_commanded or signed_yaw >= minimum_rotation
    horizontal_drift = math.hypot(dx, dy)
    pure_turn_commanded = rotation_commanded and not translation_commanded
    turn_drift_met = (
        not pure_turn_commanded
        or maximum_turn_drift is None
        or horizontal_drift <= maximum_turn_drift
    )
    command_nonzero = translation_commanded or rotation_commanded
    expectation_met = command_nonzero and translation_met and rotation_met and turn_drift_met

    return ManeuverEvidence(
        maneuver_index=maneuver_index,
        name=maneuver.name,
        command=maneuver.command,
        start_frame_index=start_frame_index,
        end_frame_index=end_frame_index,
        start_sim_time_ns=sim_times_ns[0],
        end_sim_time_ns=sim_times_ns[-1],
        base_body_id=base_body_id,
        start_position_m=start,
        end_position_m=end,
        start_yaw_rad=start_yaw,
        end_yaw_rad=yaws[-1],
        body_frame_forward_m=forward,
        body_frame_left_m=left,
        horizontal_drift_m=horizontal_drift,
        signed_translation_m=signed_translation,
        signed_yaw_rad=signed_yaw,
        minimum_displacement_m=minimum_translation,
        minimum_rotation_rad=minimum_rotation,
        maximum_turn_drift_m=maximum_turn_drift,
        translation_commanded=translation_commanded,
        rotation_commanded=rotation_commanded,
        translation_met=translation_met,
        rotation_met=rotation_met,
        turn_drift_met=turn_drift_met,
        expectation_met=expectation_met,
        motion_verified=expectation_met,
    )


def _advance_recording_command(
    launch: MotionRecordingLaunch,
    current: Mapping[str, Any],
    command: BaseTwist,
    *,
    command_sequence: int,
    steps: int,
    refresh_steps: int,
) -> tuple[dict[str, Any], int]:
    remaining = steps
    snapshot = dict(current)
    while remaining:
        command_sequence += 1
        generation = GenerationStamp(
            _event_integer(snapshot, "model_generation"),
            _event_integer(snapshot, "reset_generation"),
        )
        result = launch.coordinator.submit_controller_command(
            launch.target.controller_id,
            ControllerCommand(
                channel_id=launch.target.channel_id,
                instance_id=launch.target.instance_id,
                generation=generation,
                sequence=command_sequence,
                apply_time_ns=_event_integer(snapshot, "sim_time_ns"),
                payload=command.payload(),
            ),
        )
        if result is not CommandSubmitResult.ACCEPTED:
            raise CoordinatorError(
                "controller rejected recording command: " + getattr(result, "value", str(result))
            )
        chunk = min(remaining, refresh_steps)
        snapshot = dict(launch.coordinator.advance(chunk))
        remaining -= chunk
    return snapshot, command_sequence


def _publish_recording_snapshot(
    launch: MotionRecordingLaunch,
    snapshot: Mapping[str, Any],
) -> None:
    exit_code = launch.unreal_process.poll()
    if exit_code is not None:
        raise CoordinatorError(f"Unreal process exited during motion recording (exit={exit_code})")
    launch.publisher.publish(snapshot)
    for watcher in launch.evidence_watchers:
        watcher.apply(launch.coordinator)


def _wrapped_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _body_yaw(snapshot: Mapping[str, Any], base_body_id: str) -> float:
    quaternion = _body_quaternion(snapshot, base_body_id)
    w, x, y, z = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _body_quaternion(
    snapshot: Mapping[str, Any],
    base_body_id: str,
) -> tuple[float, float, float, float]:
    bodies = snapshot.get("bodies")
    if not isinstance(bodies, Sequence) or isinstance(bodies, (str, bytes)):
        raise ValueError("truth snapshot bodies must be a sequence")
    matches = [body for body in bodies if isinstance(body, Mapping) and body.get("stable_id") == base_body_id]
    if len(matches) != 1:
        raise ValueError(f"truth snapshot must contain exactly one base body {base_body_id!r}")
    quaternion = matches[0].get("quaternion_wxyz")
    if not isinstance(quaternion, Sequence) or isinstance(quaternion, (str, bytes)) or len(quaternion) != 4:
        raise ValueError(f"truth body {base_body_id!r} quaternion_wxyz must have length 4")
    values = tuple(
        _finite_number(value, f"{base_body_id}.quaternion_wxyz[{index}]")
        for index, value in enumerate(quaternion)
    )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError(f"truth body {base_body_id!r} quaternion_wxyz must be non-zero")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _body_position(
    snapshot: Mapping[str, Any],
    base_body_id: str,
) -> tuple[float, float, float]:
    if not isinstance(base_body_id, str) or not base_body_id or base_body_id != base_body_id.strip():
        raise ValueError("base_body_id must be a non-empty trimmed string")
    bodies = snapshot.get("bodies")
    if not isinstance(bodies, Sequence) or isinstance(bodies, (str, bytes)):
        raise ValueError("truth snapshot bodies must be a sequence")
    matches = [body for body in bodies if isinstance(body, Mapping) and body.get("stable_id") == base_body_id]
    if len(matches) != 1:
        raise ValueError(f"truth snapshot must contain exactly one base body {base_body_id!r}")
    position = matches[0].get("position_m")
    if not isinstance(position, Sequence) or isinstance(position, (str, bytes)) or len(position) != 3:
        raise ValueError(f"truth body {base_body_id!r} position_m must have length 3")
    return tuple(_finite_number(value, f"{base_body_id}.position_m[{index}]") for index, value in enumerate(position))  # type: ignore[return-value]


def _require_recording_bindings(bundle: ResolvedSessionBundle) -> frozenset[str]:
    session = bundle.session_spec
    runtime = session.get("runtime") if isinstance(session, Mapping) else None
    bindings = runtime.get("required_bindings") if isinstance(runtime, Mapping) else None
    if not isinstance(bindings, Sequence) or isinstance(bindings, (str, bytes)):
        raise CoordinatorError("recording bundle session runtime.required_bindings is invalid")
    actual = set(bindings)
    core = {"physics", "visual", "control"}
    allowed = (core, core | {"sensors"})
    if actual not in allowed or len(bindings) != len(actual):
        raise CoordinatorError(
            "production motion recording requires a derived QA session with physics, "
            "visual, control, and optionally sensors bindings"
        )
    return frozenset(actual)


def _require_truth_odometry_only_sensor_plan(bundle: ResolvedSessionBundle) -> None:
    plan = bundle.plans.get("sensor.plan.json")
    streams = plan.get("streams") if isinstance(plan, Mapping) else None
    if not isinstance(streams, Mapping):
        raise CoordinatorError("recording bundle sensor.plan streams are invalid")
    declared = {
        stream_kind: tuple(items)
        for stream_kind, items in streams.items()
        if isinstance(items, Sequence) and not isinstance(items, (str, bytes)) and items
    }
    if set(declared) != {"truth_odom"} or len(declared["truth_odom"]) != 1:
        raise CoordinatorError(
            "sensor-bound motion recording currently requires exactly one truth_odom stream"
        )


def _unreal_level(bundle: ResolvedSessionBundle) -> str:
    visual = bundle.plans.get("visual.plan.json")
    world = visual.get("world") if isinstance(visual, Mapping) else None
    level = world.get("level") if isinstance(world, Mapping) else None
    if not isinstance(level, str) or not level.startswith("/Game/") or level != level.strip():
        raise CoordinatorError("recording bundle visual world level must be a /Game/... path")
    return level


def _base_body_id(bundle: ResolvedSessionBundle, instance_id: str) -> str:
    physics = bundle.plans.get("physics.plan.json")
    robots = physics.get("robots") if isinstance(physics, Mapping) else None
    if not isinstance(robots, Sequence) or isinstance(robots, (str, bytes)):
        raise CoordinatorError("recording bundle physics robots must be a sequence")
    matches = [robot for robot in robots if isinstance(robot, Mapping) and robot.get("instance_id") == instance_id]
    if len(matches) != 1:
        raise CoordinatorError(f"recording bundle must contain exactly one robot instance {instance_id!r}")
    model = matches[0].get("model")
    attach_root = model.get("attach_root") if isinstance(model, Mapping) else None
    if not isinstance(attach_root, str) or not attach_root or attach_root != attach_root.strip():
        raise CoordinatorError(f"recording robot {instance_id!r} has no valid model.attach_root")
    return f"{instance_id}/{attach_root}"


def _trajectory_point(
    snapshot: Mapping[str, Any],
    base_body_id: str,
    *,
    frame_index: int,
    phase: str = "maneuver",
    maneuver_index: int | None = None,
    maneuver_name: str | None = None,
    phase_frame_index: int = 0,
    command: BaseTwist | None = None,
    schema_version: int = 1,
    transition_from: str | None = None,
    transition_to: str | None = None,
) -> dict[str, Any]:
    position = _body_position(snapshot, base_body_id)
    quaternion = _body_quaternion(snapshot, base_body_id)
    return {
        "schema": f"lingtu.sim.motion-trajectory-point.v{schema_version}",
        "frame_index": frame_index,
        "phase": phase,
        "phase_frame_index": phase_frame_index,
        "maneuver_index": maneuver_index,
        "maneuver_name": maneuver_name,
        "transition_from": transition_from,
        "transition_to": transition_to,
        "command": (command or BaseTwist()).payload(),
        "sequence": _event_integer(snapshot, "sequence"),
        "physics_step": _event_integer(snapshot, "physics_step"),
        "sim_time_ns": _event_integer(snapshot, "sim_time_ns"),
        "base_body_id": base_body_id,
        "position_m": list(position),
        "quaternion_wxyz": list(quaternion),
    }


def _write_trajectory_point(
    handle: Any,
    snapshot: Mapping[str, Any],
    base_body_id: str,
    *,
    frame_index: int,
    phase: str = "maneuver",
    maneuver_index: int | None = None,
    maneuver_name: str | None = None,
    phase_frame_index: int = 0,
    command: BaseTwist | None = None,
    schema_version: int = 1,
    transition_from: str | None = None,
    transition_to: str | None = None,
) -> None:
    handle.write(
        json.dumps(
            _trajectory_point(
                snapshot,
                base_body_id,
                frame_index=frame_index,
                phase=phase,
                maneuver_index=maneuver_index,
                maneuver_name=maneuver_name,
                phase_frame_index=phase_frame_index,
                command=command,
                schema_version=schema_version,
                transition_from=transition_from,
                transition_to=transition_to,
            ),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    )
    handle.flush()


def _event_integer(event: Mapping[str, Any], field: str) -> int:
    value = event.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"runtime snapshot {field} must be a non-negative integer")
    return value


def _prepare_capture_directory(directory: Path) -> int:
    path = Path(directory)
    if path.exists():
        if not path.is_dir():
            raise CoordinatorError(f"frame capture path is not a directory: {path}")
        try:
            first_entry = next(path.iterdir(), None)
        except OSError as exc:
            raise CoordinatorError(f"cannot inspect frame capture directory: {path}") from exc
        if first_entry is not None:
            raise CoordinatorError(f"frame capture directory must be empty for a fresh run: {path}")
    else:
        parent = path.parent
        while not parent.exists() and parent != parent.parent:
            parent = parent.parent
        if not parent.is_dir():
            raise CoordinatorError(f"frame capture directory has no usable parent directory: {path}")
    return time.time_ns()


def _valid_pngs(directory: Path, *, capture_started_at_ns: int) -> tuple[Path, ...]:
    if not directory.is_dir():
        return ()
    frames: list[Path] = []
    for path in sorted(directory.glob("frame_*.png")):
        try:
            stat = path.stat()
            if stat.st_mtime_ns < capture_started_at_ns or stat.st_size <= 8:
                continue
            with path.open("rb") as handle:
                if handle.read(8) != b"\x89PNG\r\n\x1a\n":
                    continue
        except OSError:
            continue
        frames.append(path.resolve())
    return tuple(frames)


def _wait_for_captured_frames(
    options: FrameCaptureOptions,
    *,
    timeout_s: float,
    capture_started_at_ns: int,
) -> CapturedFrameMetadata:
    deadline = time.monotonic() + timeout_s
    while True:
        frames = _valid_pngs(
            options.directory,
            capture_started_at_ns=capture_started_at_ns,
        )
        if len(frames) >= options.minimum_frames:
            break
        if time.monotonic() >= deadline:
            raise CoordinatorError(
                "RobotSimUE frame capture did not reach its minimum: "
                f"{len(frames)} < {options.minimum_frames} in {options.directory}"
            )
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    return _captured_frame_metadata(
        options,
        capture_started_at_ns=capture_started_at_ns,
    )


def _captured_frame_metadata(
    options: FrameCaptureOptions,
    *,
    capture_started_at_ns: int,
) -> CapturedFrameMetadata:
    frames = _valid_pngs(
        options.directory,
        capture_started_at_ns=capture_started_at_ns,
    )
    return CapturedFrameMetadata(
        directory=options.directory,
        captured_count=len(frames),
        total_bytes=sum(path.stat().st_size for path in frames),
        first_frame=frames[0] if frames else None,
        last_frame=frames[-1] if frames else None,
        capture_every=options.capture_every,
        maximum_frames=options.maximum_frames,
        minimum_frames=options.minimum_frames,
        session_camera_tag=options.session_camera_tag,
        motion_camera_stable_id=options.motion_camera_stable_id,
        capture_started_at_ns=capture_started_at_ns,
    )


def _write_json(path: Path, document: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


__all__ = [
    "FACTORY_PARK_MAXIMUM_TURN_DRIFT_M",
    "FACTORY_PARK_MINIMUM_ROTATION_RAD",
    "FACTORY_PARK_MOTION_ACCEPTANCE_PROFILE",
    "LEGACY_MOTION_ACCEPTANCE_PROFILE",
    "MOTION_ACCEPTANCE_PROFILE_VERSION",
    "CapturedFrameMetadata",
    "FrameCaptureOptions",
    "ManeuverEvidence",
    "ManeuverSegment",
    "MotionEvidence",
    "MotionRecordingConfig",
    "MotionRecordingLaunch",
    "MotionRecordingResult",
    "MotionRecordingRuntimeConfig",
    "create_motion_recording_launch",
    "evaluate_maneuver_evidence",
    "evaluate_motion_evidence",
    "main",
    "run_motion_recording",
]


if __name__ == "__main__":
    raise SystemExit(main())
