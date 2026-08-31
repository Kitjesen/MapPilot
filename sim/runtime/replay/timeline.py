"""Validated presentation replay of complete simulation truth timelines."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import stat
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from fnmatch import fnmatchcase
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Any

from sim.runtime.recording.timeline import (
    MAX_SENSOR_PAYLOAD_BYTES,
    RECORDING_FILENAME,
    RECORDING_FRAME_SCHEMA,
    RECORDING_SCHEMA,
    SENSOR_PAYLOAD_REFERENCE_SCHEMA,
    SENSOR_PAYLOAD_ROOT,
    SENSOR_PAYLOAD_STORE_SCHEMA,
    TIMELINE_FILENAME,
    TYPED_DDS_PAYLOAD_REFERENCE_SCHEMA,
    TYPED_DDS_PAYLOAD_ROOT,
    TYPED_DDS_PAYLOAD_STORE_SCHEMA,
)

_DIGEST_RE = re.compile(r"[0-9a-f]{64}\Z")
_REPARSE_POINT = 0x400
_LEGACY_CONTENT_STREAMS = (
    "command",
    "truth_snapshot",
    "scenario_event",
    "sensor_metadata",
    "lifecycle_evidence",
)
_CONTENT_STREAMS = (*_LEGACY_CONTENT_STREAMS, "sensor_payload", "typed_dds_payload")
_TERMINAL_STATES = frozenset({"STOPPED", "FAILED"})
TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION = (
    "lingtu.sim.replay-tolerance-policy.v1"
)
TRUSTED_REPLAY_TOLERANCE_MAXIMA: Mapping[str, float] = MappingProxyType(
    {
        "/actuators/*/control": 1e-4,
        "/bodies/*/angular_velocity_rps/*": 1e-2,
        "/bodies/*/linear_velocity_mps/*": 1e-2,
        "/bodies/*/position_m/*": 2e-2,
        "/bodies/*/quaternion_wxyz/*": 1e-4,
        "/joints/*/qpos/*": 1e-4,
        "/joints/*/qvel/*": 1e-3,
    }
)


class SimulationReplayError(RuntimeError):
    """Fail-closed recording validation or replay error."""


@dataclass(frozen=True)
class ReplayFrame:
    """One validated, immutable presentation frame."""

    frame_index: int
    relative_time_ns: int
    snapshot: Mapping[str, Any]
    command: Mapping[str, Any] | None
    metadata: Mapping[str, Any]
    scenario_events: tuple[Mapping[str, Any], ...] = ()
    sensor_metadata: tuple[Mapping[str, Any], ...] = ()
    sensor_payloads: tuple[Mapping[str, Any], ...] = ()
    typed_dds_payloads: tuple[Mapping[str, Any], ...] = ()
    lifecycle_evidence: tuple[Mapping[str, Any], ...] = ()


@dataclass(frozen=True)
class ReplayReport:
    """Observable result of one bounded replay pass."""

    session_id: str
    frames_presented: int
    frames_dropped: int
    duration_ns: int
    event_order: tuple[str, ...] = ()
    generations: tuple[tuple[int, int], ...] = ()
    terminal_state: str | None = None
    clock_authority: str = "recorded_mujoco"
    tolerance_policy_version: str = TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION


@dataclass(frozen=True)
class SensorPayloadReplayReport:
    """Observable result of one bounded sensor-payload replay pass."""

    session_id: str
    samples_presented: int
    samples_dropped: int
    bytes_presented: int
    duration_ns: int
    clock_authority: str = "recorded_mujoco"


@dataclass(frozen=True)
class ReplayComparisonReport:
    """Determinism verdict for two independently validated replay inputs."""

    equivalent: bool
    discrete_match: bool
    continuous_match: bool
    continuous_fields_compared: int
    mismatches: tuple[str, ...]
    clock_authority: str = "recorded_mujoco"
    tolerance_policy_version: str = TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION


@dataclass(frozen=True)
class SimulationReplay:
    """Validated recording with no authority over simulation time."""

    root: Path
    run_id: str
    session_id: str
    model_generation: int
    start_reset_generation: int
    end_reset_generation: int
    duration_ns: int
    frames: tuple[ReplayFrame, ...]
    event_order: tuple[str, ...] = ()
    terminal_state: str | None = None
    continuous_tolerances: Mapping[str, float] = field(
        default_factory=lambda: MappingProxyType({})
    )
    run_allocation: Mapping[str, Any] | None = None
    descriptors: Mapping[str, Mapping[str, Any]] | None = None
    tolerance_policy_version: str = TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION

    @property
    def frame_count(self) -> int:
        """Return the number of committed replay frames."""

        return len(self.frames)

    def read_sensor_payload(self, reference: Mapping[str, Any]) -> bytes:
        """Read one validated payload reference without accepting an external path."""

        if reference.get("session_id") != self.session_id:
            raise SimulationReplayError(
                "sensor payload reference does not belong to this replay session"
            )
        if reference.get("model_generation") != self.model_generation:
            raise SimulationReplayError(
                "sensor payload reference model_generation does not match this replay"
            )
        reset_generation = reference.get("reset_generation")
        if (
            isinstance(reset_generation, bool)
            or not isinstance(reset_generation, int)
            or not self.start_reset_generation
            <= reset_generation
            <= self.end_reset_generation
        ):
            raise SimulationReplayError(
                "sensor payload reference reset_generation does not match this replay"
            )
        return _read_sensor_payload_reference(self.root, reference)

    def read_typed_dds_payload(self, reference: Mapping[str, Any]) -> bytes:
        """Read one validated opaque typed-DDS payload from its content address."""

        if reference.get("session_id") != self.session_id:
            raise SimulationReplayError(
                "typed DDS payload reference does not belong to this replay session"
            )
        if reference.get("model_generation") != self.model_generation:
            raise SimulationReplayError(
                "typed DDS payload reference model_generation does not match this replay"
            )
        reset_generation = reference.get("reset_generation")
        if (
            isinstance(reset_generation, bool)
            or not isinstance(reset_generation, int)
            or not self.start_reset_generation <= reset_generation <= self.end_reset_generation
        ):
            raise SimulationReplayError(
                "typed DDS payload reference reset_generation does not match this replay"
            )
        return _read_typed_dds_payload_reference(self.root, reference)

    @classmethod
    def open(cls, root: Path) -> SimulationReplay:
        """Open and fully validate a committed recording directory."""

        recording_root = Path(root).absolute()
        manifest_path = recording_root / RECORDING_FILENAME
        timeline_path = recording_root / TIMELINE_FILENAME
        for path in (recording_root, manifest_path, timeline_path):
            if _is_reparse(path):
                raise SimulationReplayError("recording paths must not contain reparse points")
        manifest = _read_object(manifest_path)
        if manifest.get("schema") != RECORDING_SCHEMA:
            raise SimulationReplayError("recording manifest schema is invalid")
        _keys_with_optional(
            manifest,
            {
                "schema",
                "run_id",
                "session_id",
                "model_generation",
                "reset_generation",
                "clock",
                "timeline",
            },
            {
                "content",
                "continuous_tolerances",
                "run_allocation",
                "descriptors",
                "sensor_payloads",
                "typed_dds_payloads",
            },
            "recording manifest",
        )
        session_id = manifest.get("session_id")
        if not isinstance(session_id, str) or not session_id.strip():
            raise SimulationReplayError("recording session_id is invalid")
        timeline = manifest.get("timeline")
        if not isinstance(timeline, Mapping) or timeline.get("path") != TIMELINE_FILENAME:
            raise SimulationReplayError("recording timeline path is invalid")
        _keys(timeline, {"path", "sha256", "bytes", "frame_count"}, "recording timeline")
        payload = _read_bytes(timeline_path)
        if timeline.get("bytes") != len(payload):
            raise SimulationReplayError("recording timeline byte count does not match")
        if timeline.get("sha256") != hashlib.sha256(payload).hexdigest():
            raise SimulationReplayError("recording timeline SHA-256 does not match")
        frames = _read_frames(
            payload,
            session_id=session_id,
            recording_root=recording_root,
        )
        if timeline.get("frame_count") != len(frames):
            raise SimulationReplayError("recording frame count does not match")
        if not frames:
            raise SimulationReplayError("recording timeline is empty")
        generation = manifest.get("reset_generation")
        clock = manifest.get("clock")
        if not isinstance(generation, Mapping) or not isinstance(clock, Mapping):
            raise SimulationReplayError("recording generation or clock metadata is invalid")
        _keys(generation, {"start", "end"}, "recording reset_generation")
        _keys(
            clock,
            {"authority", "start_sim_time_ns", "end_sim_time_ns", "duration_ns"},
            "recording clock",
        )
        model_generation = _integer(manifest.get("model_generation"), "model_generation")
        start_reset = _integer(generation.get("start"), "reset_generation.start")
        end_reset = _integer(generation.get("end"), "reset_generation.end")
        duration_ns = _integer(clock.get("duration_ns"), "clock.duration_ns")
        if clock.get("authority") != "mujoco":
            raise SimulationReplayError("recording clock authority must be mujoco")
        first_snapshot = frames[0].snapshot
        last_snapshot = frames[-1].snapshot
        expected = {
            "model_generation": (model_generation, first_snapshot.get("model_generation")),
            "reset_generation.start": (start_reset, first_snapshot.get("reset_generation")),
            "reset_generation.end": (end_reset, last_snapshot.get("reset_generation")),
            "clock.start_sim_time_ns": (clock.get("start_sim_time_ns"), first_snapshot.get("sim_time_ns")),
            "clock.end_sim_time_ns": (clock.get("end_sim_time_ns"), last_snapshot.get("sim_time_ns")),
            "clock.duration_ns": (duration_ns, frames[-1].relative_time_ns),
        }
        for field_name, (declared, observed) in expected.items():
            if declared != observed:
                raise SimulationReplayError(
                    f"recording {field_name} does not match the timeline"
                )
        event_order, terminal_state, observed_content = _content_summary(frames)
        declared_content = manifest.get("content")
        if declared_content is not None:
            _validate_content_manifest(
                declared_content,
                observed_content,
                terminal_state=terminal_state,
            )
        tolerances = _continuous_tolerance_manifest(
            manifest.get("continuous_tolerances", {})
        )
        run_allocation = _validated_run_allocation(
            manifest.get("run_allocation"),
            run_id=manifest.get("run_id"),
            session_id=session_id,
        )
        descriptors = _validated_descriptors(
            manifest.get("descriptors"),
            session_id=session_id,
            model_generation=model_generation,
            reset_generation=start_reset,
        )
        _validate_sensor_payload_store(
            manifest.get("sensor_payloads"),
            _sensor_payload_summary(frames),
        )
        _validate_typed_dds_payload_store(
            manifest.get("typed_dds_payloads"),
            _typed_dds_payload_summary(frames),
        )
        return cls(
            root=recording_root,
            run_id=_text(manifest.get("run_id"), "run_id"),
            session_id=session_id,
            model_generation=model_generation,
            start_reset_generation=start_reset,
            end_reset_generation=end_reset,
            duration_ns=duration_ns,
            frames=frames,
            event_order=event_order,
            terminal_state=terminal_state,
            continuous_tolerances=MappingProxyType(tolerances),
            run_allocation=(
                MappingProxyType(run_allocation) if run_allocation is not None else None
            ),
            descriptors=(
                MappingProxyType(
                    {
                        name: MappingProxyType(document)
                        for name, document in descriptors.items()
                    }
                )
                if descriptors is not None
                else None
            ),
        )


def replay_snapshots(
    replay: SimulationReplay,
    sink: Callable[[dict[str, Any]], object],
    *,
    pace: bool = True,
    rate: float = 1.0,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> ReplayReport:
    """Present recorded snapshots while preserving every recorded timestamp."""

    if not isinstance(replay, SimulationReplay):
        raise TypeError("replay must be a SimulationReplay")
    if not callable(sink):
        raise TypeError("sink must be callable")
    if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(float(rate)) or rate <= 0:
        raise ValueError("rate must be positive finite numeric data")
    wall_origin_ns = monotonic_ns()
    presented = 0
    dropped = 0
    for frame in replay.frames:
        if pace:
            deadline_ns = wall_origin_ns + int(frame.relative_time_ns / float(rate))
            remaining_ns = deadline_ns - monotonic_ns()
            if remaining_ns > 0:
                sleep(remaining_ns / 1_000_000_000)
        result = sink(copy.deepcopy(dict(frame.snapshot)))
        if result == 0:
            dropped += 1
        else:
            presented += 1
    return ReplayReport(
        session_id=replay.session_id,
        frames_presented=presented,
        frames_dropped=dropped,
        duration_ns=replay.duration_ns,
        event_order=replay.event_order,
        generations=tuple(
            (
                int(frame.snapshot["model_generation"]),
                int(frame.snapshot["reset_generation"]),
            )
            for frame in replay.frames
        ),
        terminal_state=replay.terminal_state,
    )


def replay_sensor_payloads(
    replay: SimulationReplay,
    sink: Callable[[dict[str, Any], bytes], object],
    *,
    pace: bool = True,
    rate: float = 1.0,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> SensorPayloadReplayReport:
    """Present validated sensor blobs at their containing recorded frame times."""

    if not isinstance(replay, SimulationReplay):
        raise TypeError("replay must be a SimulationReplay")
    if not callable(sink):
        raise TypeError("sink must be callable")
    if (
        isinstance(rate, bool)
        or not isinstance(rate, (int, float))
        or not math.isfinite(float(rate))
        or rate <= 0
    ):
        raise ValueError("rate must be positive finite numeric data")
    wall_origin_ns = monotonic_ns()
    presented = 0
    dropped = 0
    presented_bytes = 0
    for frame in replay.frames:
        if not frame.sensor_payloads:
            continue
        if pace:
            deadline_ns = wall_origin_ns + int(frame.relative_time_ns / float(rate))
            remaining_ns = deadline_ns - monotonic_ns()
            if remaining_ns > 0:
                sleep(remaining_ns / 1_000_000_000)
        for reference in frame.sensor_payloads:
            payload = replay.read_sensor_payload(reference)
            result = sink(copy.deepcopy(dict(reference)), payload)
            if result == 0:
                dropped += 1
            else:
                presented += 1
                presented_bytes += len(payload)
    return SensorPayloadReplayReport(
        session_id=replay.session_id,
        samples_presented=presented,
        samples_dropped=dropped,
        bytes_presented=presented_bytes,
        duration_ns=replay.duration_ns,
    )


def replay_typed_dds_payloads(
    replay: SimulationReplay,
    sink: Callable[[dict[str, Any], bytes], object],
    *,
    pace: bool = True,
    rate: float = 1.0,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> SensorPayloadReplayReport:
    """Present validated typed-DDS bytes in recorded frame and sample order."""

    if not isinstance(replay, SimulationReplay):
        raise TypeError("replay must be a SimulationReplay")
    if not callable(sink):
        raise TypeError("sink must be callable")
    if (
        isinstance(rate, bool)
        or not isinstance(rate, (int, float))
        or not math.isfinite(float(rate))
        or rate <= 0
    ):
        raise ValueError("rate must be positive finite numeric data")
    wall_origin_ns = monotonic_ns()
    presented = 0
    dropped = 0
    presented_bytes = 0
    for frame in replay.frames:
        if not frame.typed_dds_payloads:
            continue
        if pace:
            deadline_ns = wall_origin_ns + int(frame.relative_time_ns / float(rate))
            remaining_ns = deadline_ns - monotonic_ns()
            if remaining_ns > 0:
                sleep(remaining_ns / 1_000_000_000)
        for reference in frame.typed_dds_payloads:
            payload = replay.read_typed_dds_payload(reference)
            result = sink(copy.deepcopy(dict(reference)), payload)
            if result == 0:
                dropped += 1
            else:
                presented += 1
                presented_bytes += len(payload)
    return SensorPayloadReplayReport(
        session_id=replay.session_id,
        samples_presented=presented,
        samples_dropped=dropped,
        bytes_presented=presented_bytes,
        duration_ns=replay.duration_ns,
    )


def compare_replays(
    reference: SimulationReplay,
    candidate: SimulationReplay,
) -> ReplayComparisonReport:
    """Compare deterministic state while applying recording-declared tolerances."""

    if not isinstance(reference, SimulationReplay) or not isinstance(
        candidate, SimulationReplay
    ):
        raise TypeError("reference and candidate must be SimulationReplay values")
    if (
        reference.tolerance_policy_version
        != TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION
        or candidate.tolerance_policy_version
        != TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION
    ):
        return _rejected_comparison("replay tolerance policy version is not trusted")
    try:
        reference_tolerances = _continuous_tolerance_manifest(
            reference.continuous_tolerances
        )
        candidate_tolerances = _continuous_tolerance_manifest(
            candidate.continuous_tolerances
        )
    except SimulationReplayError as exc:
        return _rejected_comparison(str(exc))
    mismatches: list[str] = []
    discrete_checks = {
        "run_id": reference.run_id == candidate.run_id,
        "session_id": reference.session_id == candidate.session_id,
        "frame_count": reference.frame_count == candidate.frame_count,
        "duration_ns": reference.duration_ns == candidate.duration_ns,
        "event_order": reference.event_order == candidate.event_order,
        "generations": _generations(reference) == _generations(candidate),
        "terminal_state": reference.terminal_state == candidate.terminal_state,
        "continuous_tolerances": reference_tolerances == candidate_tolerances,
        "run_allocation": _exact_json_equal(
            reference.run_allocation,
            candidate.run_allocation,
        ),
        "descriptors": _exact_json_equal(reference.descriptors, candidate.descriptors),
    }
    for field_name, matches in discrete_checks.items():
        if not matches:
            mismatches.append(f"discrete:{field_name}")
    for index, (left, right) in enumerate(zip(reference.frames, candidate.frames)):
        for field_name, left_value, right_value in (
            ("command", left.command, right.command),
            ("metadata", left.metadata, right.metadata),
            ("scenario_events", left.scenario_events, right.scenario_events),
            ("sensor_metadata", left.sensor_metadata, right.sensor_metadata),
            ("sensor_payloads", left.sensor_payloads, right.sensor_payloads),
            (
                "typed_dds_payloads",
                left.typed_dds_payloads,
                right.typed_dds_payloads,
            ),
            ("lifecycle_evidence", left.lifecycle_evidence, right.lifecycle_evidence),
        ):
            if not _exact_json_equal(left_value, right_value):
                mismatches.append(f"discrete:frames/{index}/{field_name}")

    continuous_mismatches: list[str] = []
    continuous_fields_compared = 0
    if reference.frame_count == candidate.frame_count:
        for index, (left, right) in enumerate(zip(reference.frames, candidate.frames)):
            continuous_fields_compared += _compare_snapshot_values(
                left.snapshot,
                right.snapshot,
                path="",
                tolerances=reference_tolerances,
                mismatches=continuous_mismatches,
                frame_index=index,
            )
    mismatches.extend(continuous_mismatches)
    discrete_match = not any(item.startswith("discrete:") for item in mismatches)
    continuous_match = not continuous_mismatches
    return ReplayComparisonReport(
        equivalent=discrete_match and continuous_match,
        discrete_match=discrete_match,
        continuous_match=continuous_match,
        continuous_fields_compared=continuous_fields_compared,
        mismatches=tuple(mismatches),
    )


def _rejected_comparison(reason: str) -> ReplayComparisonReport:
    return ReplayComparisonReport(
        equivalent=False,
        discrete_match=False,
        continuous_match=False,
        continuous_fields_compared=0,
        mismatches=(f"tolerance_policy:{reason}",),
    )


def _generations(replay: SimulationReplay) -> tuple[tuple[int, int], ...]:
    return tuple(
        (
            int(frame.snapshot["model_generation"]),
            int(frame.snapshot["reset_generation"]),
        )
        for frame in replay.frames
    )


def _exact_json_equal(left: object, right: object) -> bool:
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        if set(left) != set(right):
            return False
        return all(_exact_json_equal(left[key], right[key]) for key in left)
    if (
        isinstance(left, (list, tuple))
        and isinstance(right, (list, tuple))
    ):
        return len(left) == len(right) and all(
            _exact_json_equal(left_value, right_value)
            for left_value, right_value in zip(left, right)
        )
    return type(left) is type(right) and left == right


def _compare_snapshot_values(
    left: object,
    right: object,
    *,
    path: str,
    tolerances: Mapping[str, float],
    mismatches: list[str],
    frame_index: int,
) -> int:
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        if set(left) != set(right):
            mismatches.append(f"continuous:frames/{frame_index}{path}:shape")
            return 0
        return sum(
            _compare_snapshot_values(
                left[key],
                right[key],
                path=f"{path}/{_json_pointer_token(str(key))}",
                tolerances=tolerances,
                mismatches=mismatches,
                frame_index=frame_index,
            )
            for key in sorted(left)
        )
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            mismatches.append(f"continuous:frames/{frame_index}{path}:shape")
            return 0
        return sum(
            _compare_snapshot_values(
                left_value,
                right_value,
                path=f"{path}/{index}",
                tolerances=tolerances,
                mismatches=mismatches,
                frame_index=frame_index,
            )
            for index, (left_value, right_value) in enumerate(zip(left, right))
        )
    if (
        not isinstance(left, bool)
        and not isinstance(right, bool)
        and isinstance(left, (int, float))
        and isinstance(right, (int, float))
        and (isinstance(left, float) or isinstance(right, float))
    ):
        tolerance = _declared_tolerance(path, tolerances)
        if abs(float(left) - float(right)) > tolerance:
            mismatches.append(
                f"continuous:frames/{frame_index}{path}:delta={abs(float(left) - float(right))}"
            )
        return 1
    if not _exact_json_equal(left, right):
        mismatches.append(f"discrete:frames/{frame_index}{path}")
    return 0


def _declared_tolerance(path: str, tolerances: Mapping[str, float]) -> float:
    exact = tolerances.get(path)
    if exact is not None:
        return exact
    matches = [
        (pattern.count("*"), -len(pattern), tolerance)
        for pattern, tolerance in tolerances.items()
        if fnmatchcase(path, pattern)
    ]
    if not matches:
        return 0.0
    return sorted(matches)[0][2]


def _json_pointer_token(value: str) -> str:
    return value.replace("~", "~0").replace("/", "~1")


def _read_frames(
    payload: bytes,
    *,
    session_id: str,
    recording_root: Path,
) -> tuple[ReplayFrame, ...]:
    frames: list[ReplayFrame] = []
    payload_cache: dict[str, bytes] = {}
    typed_dds_payload_cache: dict[str, bytes] = {}
    previous_relative = -1
    previous_model_generation: int | None = None
    previous_reset_generation: int | None = None
    previous_sequence: int | None = None
    previous_physics_step: int | None = None
    previous_sim_time_ns: int | None = None
    expected_relative_time_ns = 0
    try:
        text = payload.decode("utf-8")
    except UnicodeError as exc:
        raise SimulationReplayError("recording timeline is not UTF-8") from exc
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line:
            raise SimulationReplayError(f"recording timeline line {line_number} is empty")
        value = _strict_json(line)
        if not isinstance(value, dict) or value.get("schema") != RECORDING_FRAME_SCHEMA:
            raise SimulationReplayError(f"recording frame {line_number} schema is invalid")
        _keys_with_optional(
            value,
            {"schema", "frame_index", "relative_time_ns", "snapshot", "command", "metadata"},
            {"evidence"},
            f"recording frame {line_number}",
        )
        frame_index = _integer(value.get("frame_index"), "frame_index")
        relative_time_ns = _integer(value.get("relative_time_ns"), "relative_time_ns")
        if frame_index != len(frames):
            raise SimulationReplayError("recording frame indices are not contiguous")
        if relative_time_ns < previous_relative:
            raise SimulationReplayError("recording relative time moved backwards")
        snapshot = value.get("snapshot")
        if not isinstance(snapshot, dict) or snapshot.get("schema") != "lingtu.sim.truth-snapshot.v1":
            raise SimulationReplayError("recording frame snapshot is invalid")
        if snapshot.get("session_id") != session_id:
            raise SimulationReplayError("recording frame session_id does not match")
        model_generation = _integer(snapshot.get("model_generation"), "snapshot.model_generation")
        reset_generation = _integer(snapshot.get("reset_generation"), "snapshot.reset_generation")
        sequence = _integer(snapshot.get("sequence"), "snapshot.sequence")
        physics_step = _integer(snapshot.get("physics_step"), "snapshot.physics_step")
        sim_time_ns = _integer(snapshot.get("sim_time_ns"), "snapshot.sim_time_ns")
        if previous_model_generation is not None:
            if model_generation != previous_model_generation:
                raise SimulationReplayError("recording model_generation changed inside the timeline")
            if reset_generation == previous_reset_generation:
                if previous_sequence is not None and sequence <= previous_sequence:
                    raise SimulationReplayError("recording snapshot sequence did not increase")
                if previous_physics_step is not None and physics_step <= previous_physics_step:
                    raise SimulationReplayError("recording physics_step did not increase")
                if previous_sim_time_ns is not None:
                    if sim_time_ns < previous_sim_time_ns:
                        raise SimulationReplayError("recording sim_time_ns moved backwards")
                    expected_relative_time_ns += sim_time_ns - previous_sim_time_ns
            elif previous_reset_generation is None or reset_generation != previous_reset_generation + 1:
                raise SimulationReplayError(
                    "recording reset_generation must remain stable or increase by one"
                )
        if relative_time_ns != expected_relative_time_ns:
            raise SimulationReplayError("recording relative time does not match snapshot time")
        command = value.get("command")
        metadata = value.get("metadata")
        if command is not None and not isinstance(command, dict):
            raise SimulationReplayError("recording frame command must be an object or null")
        if not isinstance(metadata, dict):
            raise SimulationReplayError("recording frame metadata must be an object")
        evidence = value.get("evidence", {})
        if not isinstance(evidence, dict):
            raise SimulationReplayError("recording frame evidence must be an object")
        _keys_with_optional(
            evidence,
            {"scenario_events", "sensor_metadata", "lifecycle_evidence"}
            if "evidence" in value
            else set(),
            {"sensor_payloads", "typed_dds_payloads"}
            if "evidence" in value
            else set(),
            "recording frame evidence",
        )
        scenario_events = _evidence_array(
            evidence.get("scenario_events", []),
            field="scenario_events",
            snapshot=snapshot,
        )
        sensor_metadata = _evidence_array(
            evidence.get("sensor_metadata", []),
            field="sensor_metadata",
            snapshot=snapshot,
        )
        sensor_payloads = _sensor_payload_reference_array(
            evidence.get("sensor_payloads", []),
            snapshot=snapshot,
            recording_root=recording_root,
            payload_cache=payload_cache,
        )
        typed_dds_payloads = _typed_dds_payload_reference_array(
            evidence.get("typed_dds_payloads", []),
            snapshot=snapshot,
            recording_root=recording_root,
            payload_cache=typed_dds_payload_cache,
        )
        lifecycle_evidence = _evidence_array(
            evidence.get("lifecycle_evidence", []),
            field="lifecycle_evidence",
            snapshot=snapshot,
        )
        frames.append(
            ReplayFrame(
                frame_index=frame_index,
                relative_time_ns=relative_time_ns,
                snapshot=MappingProxyType(snapshot),
                command=MappingProxyType(command) if command is not None else None,
                metadata=MappingProxyType(metadata),
                scenario_events=tuple(MappingProxyType(item) for item in scenario_events),
                sensor_metadata=tuple(MappingProxyType(item) for item in sensor_metadata),
                sensor_payloads=tuple(
                    MappingProxyType(item) for item in sensor_payloads
                ),
                typed_dds_payloads=tuple(
                    MappingProxyType(item) for item in typed_dds_payloads
                ),
                lifecycle_evidence=tuple(
                    MappingProxyType(item) for item in lifecycle_evidence
                ),
            )
        )
        previous_relative = relative_time_ns
        previous_model_generation = model_generation
        previous_reset_generation = reset_generation
        previous_sequence = sequence
        previous_physics_step = physics_step
        previous_sim_time_ns = sim_time_ns
    return tuple(frames)


def _evidence_array(
    value: object,
    *,
    field: str,
    snapshot: Mapping[str, Any],
) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise SimulationReplayError(f"recording frame {field} must be an array")
    records: list[dict[str, Any]] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise SimulationReplayError(
                f"recording frame {field}[{index}] must be an object"
            )
        for identity_field in (
            "session_id",
            "model_generation",
            "reset_generation",
        ):
            if item.get(identity_field) != snapshot.get(identity_field):
                raise SimulationReplayError(
                    f"recording frame {field}[{index}] {identity_field} does not match its truth snapshot"
                )
        if field == "lifecycle_evidence":
            state = item.get("state")
            if not isinstance(state, str) or not state or state != state.strip():
                raise SimulationReplayError(
                    "recording lifecycle evidence state must be non-empty trimmed text"
                )
        records.append(item)
    return records


def _sensor_payload_reference_array(
    value: object,
    *,
    snapshot: Mapping[str, Any],
    recording_root: Path,
    payload_cache: dict[str, bytes],
) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise SimulationReplayError(
            "recording frame sensor_payloads must be an array"
        )
    references: list[dict[str, Any]] = []
    for index, item in enumerate(value):
        if not isinstance(item, Mapping):
            raise SimulationReplayError(
                f"recording frame sensor_payloads[{index}] must be an object"
            )
        reference, _payload = _validated_sensor_payload_reference(
            recording_root,
            item,
            payload_cache=payload_cache,
        )
        for identity_field in (
            "session_id",
            "model_generation",
            "reset_generation",
        ):
            if reference[identity_field] != snapshot.get(identity_field):
                raise SimulationReplayError(
                    f"recording frame sensor_payloads[{index}] {identity_field} "
                    "does not match its truth snapshot"
                )
        references.append(reference)
    return references


def _typed_dds_payload_reference_array(
    value: object,
    *,
    snapshot: Mapping[str, Any],
    recording_root: Path,
    payload_cache: dict[str, bytes],
) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise SimulationReplayError(
            "recording frame typed_dds_payloads must be an array"
        )
    references: list[dict[str, Any]] = []
    for index, item in enumerate(value):
        if not isinstance(item, Mapping):
            raise SimulationReplayError(
                f"recording frame typed_dds_payloads[{index}] must be an object"
            )
        reference, _payload = _validated_typed_dds_payload_reference(
            recording_root,
            item,
            payload_cache=payload_cache,
        )
        for identity_field in ("session_id", "model_generation", "reset_generation"):
            if reference[identity_field] != snapshot.get(identity_field):
                raise SimulationReplayError(
                    f"recording frame typed_dds_payloads[{index}] {identity_field} "
                    "does not match its truth snapshot"
                )
        references.append(reference)
    return references


def _read_sensor_payload_reference(
    recording_root: Path,
    reference: Mapping[str, Any],
) -> bytes:
    _validated, payload = _validated_sensor_payload_reference(
        recording_root,
        reference,
        payload_cache={},
    )
    return payload


def _read_typed_dds_payload_reference(
    recording_root: Path,
    reference: Mapping[str, Any],
) -> bytes:
    _validated, payload = _validated_typed_dds_payload_reference(
        recording_root,
        reference,
        payload_cache={},
    )
    return payload


def _validated_sensor_payload_reference(
    recording_root: Path,
    value: Mapping[str, Any],
    *,
    payload_cache: dict[str, bytes],
) -> tuple[dict[str, Any], bytes]:
    reference = dict(value)
    _keys(
        reference,
        {
            "schema",
            "session_id",
            "model_generation",
            "reset_generation",
            "sensor_id",
            "stream_kind",
            "encoding",
            "media_type",
            "sample_sequence",
            "sample_time_ns",
            "path",
            "sha256",
            "bytes",
            "metadata",
        },
        "sensor payload reference",
    )
    if reference.get("schema") != SENSOR_PAYLOAD_REFERENCE_SCHEMA:
        raise SimulationReplayError("sensor payload reference schema is invalid")
    digest = reference.get("sha256")
    if not isinstance(digest, str) or _DIGEST_RE.fullmatch(digest) is None:
        raise SimulationReplayError("sensor payload SHA-256 is invalid")
    declared_bytes = _integer(reference.get("bytes"), "sensor payload bytes")
    if declared_bytes < 1 or declared_bytes > MAX_SENSOR_PAYLOAD_BYTES:
        raise SimulationReplayError(
            "sensor payload bytes exceed the trusted per-sample bound"
        )
    relative = reference.get("path")
    expected_path = f"{SENSOR_PAYLOAD_ROOT}/{digest[:2]}/{digest}.bin"
    if (
        not isinstance(relative, str)
        or "\\" in relative
        or PurePosixPath(relative).is_absolute()
        or any(part in {"", ".", ".."} for part in PurePosixPath(relative).parts)
        or relative != expected_path
    ):
        raise SimulationReplayError("sensor payload path is invalid")
    for field_name in ("sensor_id", "stream_kind", "encoding", "media_type"):
        _text(reference.get(field_name), f"sensor payload {field_name}")
    _integer(reference.get("sample_sequence"), "sensor payload sample_sequence")
    _integer(reference.get("sample_time_ns"), "sensor payload sample_time_ns")
    _integer(reference.get("model_generation"), "sensor payload model_generation")
    _integer(reference.get("reset_generation"), "sensor payload reset_generation")
    session_id = reference.get("session_id")
    if (
        not isinstance(session_id, str)
        or not session_id.strip()
    ):
        raise SimulationReplayError("sensor payload session_id is invalid")
    if not isinstance(reference.get("metadata"), dict):
        raise SimulationReplayError("sensor payload metadata must be an object")

    path = Path(recording_root)
    for part in PurePosixPath(relative).parts:
        path /= part
        if _is_reparse(path):
            raise SimulationReplayError(
                "sensor payload paths must not contain reparse points"
            )
    cached = payload_cache.get(digest)
    if cached is None:
        payload = _read_bytes(path)
        if len(payload) != declared_bytes:
            raise SimulationReplayError("sensor payload byte count does not match")
        if hashlib.sha256(payload).hexdigest() != digest:
            raise SimulationReplayError("sensor payload SHA-256 does not match")
        payload_cache[digest] = payload
    else:
        payload = cached
        if len(payload) != declared_bytes:
            raise SimulationReplayError(
                "sensor payload references disagree on byte count"
            )
    return reference, payload


def _validated_typed_dds_payload_reference(
    recording_root: Path,
    value: Mapping[str, Any],
    *,
    payload_cache: dict[str, bytes],
) -> tuple[dict[str, Any], bytes]:
    reference = dict(value)
    _keys(
        reference,
        {
            "schema",
            "session_id",
            "model_generation",
            "reset_generation",
            "topic",
            "type_name",
            "encoding",
            "sequence",
            "sim_time_ns",
            "path",
            "sha256",
            "bytes",
            "metadata",
        },
        "typed DDS payload reference",
    )
    if reference.get("schema") != TYPED_DDS_PAYLOAD_REFERENCE_SCHEMA:
        raise SimulationReplayError("typed DDS payload reference schema is invalid")
    digest = reference.get("sha256")
    if not isinstance(digest, str) or _DIGEST_RE.fullmatch(digest) is None:
        raise SimulationReplayError("typed DDS payload SHA-256 is invalid")
    declared_bytes = _integer(reference.get("bytes"), "typed DDS payload bytes")
    if declared_bytes < 1 or declared_bytes > MAX_SENSOR_PAYLOAD_BYTES:
        raise SimulationReplayError(
            "typed DDS payload bytes exceed the trusted per-sample bound"
        )
    relative = reference.get("path")
    expected_path = f"{TYPED_DDS_PAYLOAD_ROOT}/{digest[:2]}/{digest}.bin"
    if (
        not isinstance(relative, str)
        or "\\" in relative
        or PurePosixPath(relative).is_absolute()
        or any(part in {"", ".", ".."} for part in PurePosixPath(relative).parts)
        or relative != expected_path
    ):
        raise SimulationReplayError("typed DDS payload path is invalid")
    for field_name in ("topic", "type_name", "encoding"):
        _text(reference.get(field_name), f"typed DDS payload {field_name}")
    _integer(reference.get("model_generation"), "typed DDS payload model_generation")
    _integer(reference.get("reset_generation"), "typed DDS payload reset_generation")
    _integer(reference.get("sequence"), "typed DDS payload sequence")
    _integer(reference.get("sim_time_ns"), "typed DDS payload sim_time_ns")
    session_id = reference.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        raise SimulationReplayError("typed DDS payload session_id is invalid")
    if not isinstance(reference.get("metadata"), dict):
        raise SimulationReplayError("typed DDS payload metadata must be an object")
    path = Path(recording_root)
    for part in PurePosixPath(relative).parts:
        path /= part
        if _is_reparse(path):
            raise SimulationReplayError(
                "typed DDS payload paths must not contain reparse points"
            )
    cached = payload_cache.get(digest)
    if cached is None:
        payload = _read_bytes(path)
        if len(payload) != declared_bytes:
            raise SimulationReplayError("typed DDS payload byte count does not match")
        if hashlib.sha256(payload).hexdigest() != digest:
            raise SimulationReplayError("typed DDS payload SHA-256 does not match")
        payload_cache[digest] = payload
    else:
        payload = cached
        if len(payload) != declared_bytes:
            raise SimulationReplayError(
                "typed DDS payload references disagree on byte count"
            )
    return reference, payload


def _sensor_payload_summary(frames: tuple[ReplayFrame, ...]) -> dict[str, int]:
    references = [reference for frame in frames for reference in frame.sensor_payloads]
    unique: dict[str, int] = {}
    referenced_bytes = 0
    for reference in references:
        digest = str(reference["sha256"])
        size = int(reference["bytes"])
        referenced_bytes += size
        previous = unique.setdefault(digest, size)
        if previous != size:
            raise SimulationReplayError(
                "sensor payload references disagree on content-addressed size"
            )
    return {
        "reference_count": len(references),
        "unique_blob_count": len(unique),
        "referenced_bytes": referenced_bytes,
        "unique_bytes": sum(unique.values()),
    }


def _typed_dds_payload_summary(frames: tuple[ReplayFrame, ...]) -> dict[str, int]:
    references = [
        reference for frame in frames for reference in frame.typed_dds_payloads
    ]
    unique: dict[str, int] = {}
    referenced_bytes = 0
    for reference in references:
        digest = str(reference["sha256"])
        size = int(reference["bytes"])
        referenced_bytes += size
        previous = unique.setdefault(digest, size)
        if previous != size:
            raise SimulationReplayError(
                "typed DDS payload references disagree on content-addressed size"
            )
    return {
        "reference_count": len(references),
        "unique_blob_count": len(unique),
        "referenced_bytes": referenced_bytes,
        "unique_bytes": sum(unique.values()),
    }


def _validate_sensor_payload_store(
    value: object,
    observed: Mapping[str, int],
) -> None:
    if observed["reference_count"] == 0:
        if value is not None:
            raise SimulationReplayError(
                "recording sensor payload store is declared without references"
            )


def _validate_typed_dds_payload_store(
    value: object,
    observed: Mapping[str, int],
) -> None:
    if observed["reference_count"] == 0:
        if value is not None:
            raise SimulationReplayError(
                "recording typed DDS payload store is declared without references"
            )
        return
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording typed DDS payload store is missing")
    _keys(
        value,
        {
            "schema",
            "root",
            "reference_count",
            "unique_blob_count",
            "referenced_bytes",
            "unique_bytes",
        },
        "recording typed DDS payload store",
    )
    if value.get("schema") != TYPED_DDS_PAYLOAD_STORE_SCHEMA:
        raise SimulationReplayError("recording typed DDS payload store schema is invalid")
    if value.get("root") != TYPED_DDS_PAYLOAD_ROOT:
        raise SimulationReplayError("recording typed DDS payload store root is invalid")
    for field_name, expected in observed.items():
        if _integer(
            value.get(field_name), f"typed DDS payload store {field_name}"
        ) != expected:
            raise SimulationReplayError(
                f"recording typed DDS payload store {field_name} does not match"
            )
        return
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording sensor payload store is missing")
    _keys(
        value,
        {
            "schema",
            "root",
            "reference_count",
            "unique_blob_count",
            "referenced_bytes",
            "unique_bytes",
        },
        "recording sensor payload store",
    )
    if value.get("schema") != SENSOR_PAYLOAD_STORE_SCHEMA:
        raise SimulationReplayError("recording sensor payload store schema is invalid")
    if value.get("root") != SENSOR_PAYLOAD_ROOT:
        raise SimulationReplayError("recording sensor payload store root is invalid")
    for field_name, expected in observed.items():
        if _integer(value.get(field_name), f"sensor payload store {field_name}") != expected:
            raise SimulationReplayError(
                f"recording sensor payload store {field_name} does not match"
            )


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _document_digest(value: object) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _content_summary(
    frames: tuple[ReplayFrame, ...],
) -> tuple[tuple[str, ...], str | None, dict[str, Any]]:
    stream_hashers = {name: hashlib.sha256() for name in _CONTENT_STREAMS}
    stream_counts = dict.fromkeys(_CONTENT_STREAMS, 0)
    event_order_hasher = hashlib.sha256()
    event_order: list[str] = []
    event_count = 0
    terminal_state: str | None = None

    def track(frame_index: int, kind: str, value: Mapping[str, Any]) -> None:
        nonlocal event_count
        order_record = {
            "event_index": event_count,
            "frame_index": frame_index,
            "kind": kind,
        }
        content_record = {
            **order_record,
            "value": dict(value),
        }
        event_order_hasher.update(_canonical_json(order_record) + b"\n")
        stream_hashers[kind].update(_canonical_json(content_record) + b"\n")
        stream_counts[kind] += 1
        event_order.append(kind)
        event_count += 1

    for frame in frames:
        if frame.command is not None:
            track(frame.frame_index, "command", frame.command)
        for event in frame.scenario_events:
            track(frame.frame_index, "scenario_event", event)
        track(frame.frame_index, "truth_snapshot", frame.snapshot)
        for sample in frame.sensor_metadata:
            track(frame.frame_index, "sensor_metadata", sample)
        for reference in frame.sensor_payloads:
            track(frame.frame_index, "sensor_payload", reference)
        for reference in frame.typed_dds_payloads:
            track(frame.frame_index, "typed_dds_payload", reference)
        for lifecycle in frame.lifecycle_evidence:
            terminal_state = str(lifecycle["state"])
            track(frame.frame_index, "lifecycle_evidence", lifecycle)

    return (
        tuple(event_order),
        terminal_state,
        {
            "event_order": {
                "count": event_count,
                "sha256": event_order_hasher.hexdigest(),
            },
            "streams": {
                name: {
                    "count": stream_counts[name],
                    "sha256": stream_hashers[name].hexdigest(),
                }
                for name in _CONTENT_STREAMS
            },
        },
    )


def _validate_content_manifest(
    value: object,
    observed: Mapping[str, Any],
    *,
    terminal_state: str | None,
) -> None:
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording content manifest must be an object")
    _keys(value, {"required_streams", "event_order", "streams"}, "recording content")
    required = value.get("required_streams")
    if (
        not isinstance(required, list)
        or any(not isinstance(item, str) or item not in _CONTENT_STREAMS for item in required)
        or required != sorted(set(required))
    ):
        raise SimulationReplayError("recording required content streams are invalid")
    event_order = value.get("event_order")
    streams = value.get("streams")
    if not isinstance(event_order, Mapping) or not isinstance(streams, Mapping):
        raise SimulationReplayError("recording content hashes are invalid")
    _keys(event_order, {"count", "sha256"}, "recording event order")
    declared_streams = set(streams)
    if declared_streams not in {
        frozenset(_LEGACY_CONTENT_STREAMS),
        frozenset(_CONTENT_STREAMS),
        frozenset((*_LEGACY_CONTENT_STREAMS, "sensor_payload")),
    }:
        raise SimulationReplayError("recording content streams are invalid")
    if "sensor_payload" in required and "sensor_payload" not in declared_streams:
        raise SimulationReplayError(
            "recording required sensor_payload stream is not declared"
        )
    if "typed_dds_payload" in required and "typed_dds_payload" not in declared_streams:
        raise SimulationReplayError(
            "recording required typed_dds_payload stream is not declared"
        )
    if dict(event_order) != observed["event_order"]:
        raise SimulationReplayError("recording event order content hash does not match")
    observed_streams = observed["streams"]
    for name in declared_streams:
        stream = streams.get(name)
        if not isinstance(stream, Mapping):
            raise SimulationReplayError(f"recording {name} content hash is invalid")
        _keys(stream, {"count", "sha256"}, f"recording {name} content")
        if dict(stream) != observed_streams[name]:
            raise SimulationReplayError(
                f"recording {name} content hash does not match"
            )
        if name in required and stream.get("count") == 0:
            raise SimulationReplayError(f"recording required content is empty: {name}")
    for name in set(_CONTENT_STREAMS) - declared_streams:
        if observed_streams[name]["count"] != 0:
            raise SimulationReplayError(
                f"recording {name} content exists without a declared stream"
            )
    if (
        "lifecycle_evidence" in required
        and terminal_state not in _TERMINAL_STATES
    ):
        raise SimulationReplayError(
            "recording lifecycle evidence has no terminal STOPPED or FAILED state"
        )


def _continuous_tolerance_manifest(value: object) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording continuous_tolerances must be an object")
    result: dict[str, float] = {}
    for path, tolerance in sorted(value.items()):
        if (
            not isinstance(path, str)
            or not path.startswith("/")
            or path != path.strip()
        ):
            raise SimulationReplayError("recording continuous tolerance path is invalid")
        if (
            isinstance(tolerance, bool)
            or not isinstance(tolerance, (int, float))
            or not math.isfinite(float(tolerance))
            or float(tolerance) < 0.0
        ):
            raise SimulationReplayError(
                f"recording continuous tolerance for {path} is invalid"
            )
        trusted_maximum = TRUSTED_REPLAY_TOLERANCE_MAXIMA.get(path)
        if trusted_maximum is None:
            raise SimulationReplayError(
                f"recording continuous tolerance path is not trusted: {path}"
            )
        normalized = float(tolerance)
        if normalized > trusted_maximum:
            raise SimulationReplayError(
                "recording continuous tolerance exceeds trusted maximum: "
                f"{path}={normalized} > {trusted_maximum}"
            )
        result[path] = normalized
    return result


def _validated_run_allocation(
    value: object,
    *,
    run_id: object,
    session_id: str,
) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording run_allocation reference must be an object")
    _keys(value, {"sha256", "document"}, "recording run_allocation")
    document = value.get("document")
    if not isinstance(document, dict):
        raise SimulationReplayError("recording run_allocation document must be an object")
    if value.get("sha256") != _document_digest(document):
        raise SimulationReplayError("recording run_allocation SHA-256 does not match")
    if document.get("schema") != "lingtu.sim.run-allocation.v1":
        raise SimulationReplayError("recording run_allocation schema is invalid")
    if document.get("run_id") != run_id:
        raise SimulationReplayError("recording run_allocation run_id does not match")
    if document.get("session_id") != session_id:
        raise SimulationReplayError(
            "recording run_allocation session_id does not match"
        )
    return document


def _validated_descriptors(
    value: object,
    *,
    session_id: str,
    model_generation: int,
    reset_generation: int,
) -> dict[str, dict[str, Any]] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise SimulationReplayError("recording descriptors reference must be an object")
    _keys(value, {"sha256", "documents"}, "recording descriptors")
    documents = value.get("documents")
    if not isinstance(documents, dict) or not documents:
        raise SimulationReplayError("recording descriptors documents must be non-empty")
    if value.get("sha256") != _document_digest(documents):
        raise SimulationReplayError("recording descriptors SHA-256 does not match")
    for name, document in documents.items():
        if not isinstance(name, str) or not name or name != name.strip():
            raise SimulationReplayError("recording descriptor name is invalid")
        if not isinstance(document, dict):
            raise SimulationReplayError(f"recording descriptor {name} must be an object")
        if document.get("session_id") not in (None, session_id):
            raise SimulationReplayError(
                f"recording descriptor {name} session_id does not match"
            )
        for field_name, expected in (
            ("model_generation", model_generation),
            ("reset_generation", reset_generation),
        ):
            if document.get(field_name) not in (None, expected):
                raise SimulationReplayError(
                    f"recording descriptor {name} {field_name} does not match"
                )
    return documents


def _strict_json(text: str) -> Any:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise SimulationReplayError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def reject_constant(value: str) -> Any:
        raise SimulationReplayError(f"non-finite JSON value: {value}")

    try:
        return json.loads(text, object_pairs_hook=object_from_pairs, parse_constant=reject_constant)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SimulationReplayError("recording JSON is invalid") from exc


def _read_object(path: Path) -> dict[str, Any]:
    try:
        value = _strict_json(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SimulationReplayError(f"cannot read recording artifact: {path.name}") from exc
    if not isinstance(value, dict):
        raise SimulationReplayError(f"recording artifact must be an object: {path.name}")
    return value


def _read_bytes(path: Path) -> bytes:
    try:
        return _native_filesystem_path(path).read_bytes()
    except OSError as exc:
        raise SimulationReplayError(f"cannot read recording artifact: {path.name}") from exc


def _native_filesystem_path(path: Path) -> Path:
    """Use the Win32 extended path form for deep run-owned artifacts."""

    absolute = Path(path).absolute()
    if os.name != "nt":
        return absolute
    value = str(absolute)
    if value.startswith("\\\\?\\"):
        return absolute
    if value.startswith("\\\\"):
        return Path("\\\\?\\UNC\\" + value[2:])
    return Path("\\\\?\\" + value)


def _integer(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SimulationReplayError(f"recording {field} must be a non-negative integer")
    return value


def _keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    missing = sorted(expected - set(value))
    unknown = sorted(set(value) - expected)
    if missing:
        raise SimulationReplayError(f"{context} is missing required field(s): {', '.join(missing)}")
    if unknown:
        raise SimulationReplayError(f"{context} contains unknown field(s): {', '.join(unknown)}")


def _keys_with_optional(
    value: Mapping[str, Any],
    required: set[str],
    optional: set[str],
    context: str,
) -> None:
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing:
        raise SimulationReplayError(
            f"{context} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise SimulationReplayError(
            f"{context} contains unknown field(s): {', '.join(unknown)}"
        )


def _text(value: object, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SimulationReplayError(f"recording {field} must be non-empty text")
    return value


def _is_reparse(path: Path) -> bool:
    try:
        metadata = os.lstat(_native_filesystem_path(path))
    except OSError:
        return False
    return stat.S_ISLNK(metadata.st_mode) or bool(
        getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
    )


__all__ = [
    "TRUSTED_REPLAY_TOLERANCE_MAXIMA",
    "TRUSTED_REPLAY_TOLERANCE_POLICY_VERSION",
    "ReplayComparisonReport",
    "ReplayFrame",
    "ReplayReport",
    "SensorPayloadReplayReport",
    "SimulationReplay",
    "SimulationReplayError",
    "compare_replays",
    "replay_sensor_payloads",
    "replay_snapshots",
]
