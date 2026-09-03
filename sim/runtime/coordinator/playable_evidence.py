"""Post-close evidence assembly for the fixed RobotSimUE playable product.

This module has no live-runtime authority.  It consumes only immutable files
and process shutdown snapshots after the interactive owner has stopped.  It
never fills a missing fact with an inferred success value.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import subprocess
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast

from sim.runtime.process_owner import ProcessShutdownSnapshot
from sim.runtime.replay.timeline import SimulationReplay, SimulationReplayError

PLAYABLE_EVIDENCE_MANIFEST = "recording/recording.manifest.json"
PLAYABLE_PRODUCT: Mapping[str, str] = MappingProxyType(
    {
        "world_package": "factory_park_hf@1.2.1",
        "robot_package": "thunderv4@1.0.3",
        "robot_instance_id": "thunder_01",
        "robot_color": "black_graphite",
        "controller_package": "thunderv4_locomotion@1.0.0",
    }
)

_DIGEST_RE = re.compile(r"[0-9a-f]{64}\Z")
_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_FRAME_RE = re.compile(r"frame_([0-9]{6})\.png\Z")
_CAPTURE_LOG_RE = re.compile(
    r"LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED "
    r"capture_index=(?P<frame>[0-9]+) "
    r"model_generation=(?P<model>[0-9]+) "
    r"reset_generation=(?P<reset>[0-9]+) "
    r"sequence=(?P<sequence>[0-9]+) "
    r"sim_time_ns=(?P<time>[0-9]+) "
    r"path=(?P<path>.+?) requested=(?P<requested>[0-9]+) max=(?P<max>[0-9]+)"
)
_MANEUVERS = ("forward", "backward", "left", "right", "turn_left", "turn_right")
_REQUIRED_RECORDING_REQUESTS = (
    "record_start",
    "pause",
    "resume",
    "record_stop_commit",
    "pause",
    "exit",
)
_ALLOWED_ACCEPTED_RUNTIME_REQUESTS = frozenset(
    {
        "record_start",
        "record_stop_commit",
        "exit",
        "pause",
        "resume",
        "ui_state_update",
        "control_claim",
    }
)
_ALLOWED_PENDING_RUNTIME_REQUESTS = frozenset({"control_release"})
_RUNTIME_REQUEST_TRACE_COMMON_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "source_monotonic_ns",
        "arrival_monotonic_ns",
        "datagram_sha256",
        "request",
    }
)
_RUNTIME_REQUEST_TRACE_RECEIVED_FIELDS = (
    _RUNTIME_REQUEST_TRACE_COMMON_FIELDS | frozenset({"event"})
)
_RUNTIME_REQUEST_TRACE_TERMINAL_FIELDS = (
    _RUNTIME_REQUEST_TRACE_COMMON_FIELDS
    | frozenset({"event", "status", "reason"})
)
_RUNTIME_REQUEST_TRACE_JOIN_FIELDS = tuple(
    sorted(_RUNTIME_REQUEST_TRACE_COMMON_FIELDS - frozenset({"schema"}))
)
_QUALIFYING_SAFETY_ZERO_REASONS = frozenset(
    {"deadman_released", "runtime_request:control_release"}
)
_ALLOWED_ZERO_AUDIT_REASONS = _QUALIFYING_SAFETY_ZERO_REASONS | frozenset(
    {"cleared:pause"}
)
_SENSOR_RATES = {
    "thunder_01.front_depth": 30,
    "thunder_01.front_rgb": 30,
    "thunder_01.imu": 200,
    "thunder_01.mid360": 10,
    "thunder_01.truth_odom": 100,
}
_CAMERA_STREAMS = frozenset(
    {"thunder_01.front_depth", "thunder_01.front_rgb"}
)
_WIDTH = 1920
_HEIGHT = 1080
_CONTROLLER_MANIFEST_PATH = "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml"
_CONTROLLER_COMMAND_CALIBRATION: Mapping[str, Any] = {
    "schema": "lingtu.sim.controller-command-calibration.v1",
    "scope": "quadruped_him_observation_only",
    "provenance": {
        "source_id": "factory_park_turn_truth_qa_20260809",
        "audit_note": (
            "Source identifier records the calibration rationale only; no warehouse "
            "artifact is claimed as yaw-turn qualification proof."
        ),
        "qualification_claim": False,
    },
    "external_yaw_cap_radps": 0.35,
    "policy_yaw_observation_gain": 1.2857142857142858,
    "policy_yaw_observation_limit_radps": 0.45,
    "leaves_base_twist_unchanged": True,
    "affected_axes": ["angular_z"],
}


class PlayableEvidenceError(RuntimeError):
    """Completed raw evidence cannot be assembled without inventing facts."""


@dataclass(frozen=True, slots=True)
class PlayableEvidenceInputs:
    """Immutable post-close inputs captured by the owning runner."""

    run_dir: Path
    run_id: str
    boot_id: str
    session_id: str
    unreal_log_path: Path
    media_toolchain: PinnedPlayableMediaToolchain
    unreal_shutdown: ProcessShutdownSnapshot | None
    mujoco_shutdown: ProcessShutdownSnapshot | None
    resources_closed: Mapping[str, bool]

    def __post_init__(self) -> None:
        if not isinstance(self.run_id, str) or _RUN_ID_RE.fullmatch(self.run_id) is None:
            raise ValueError("run_id is invalid")
        if not isinstance(self.boot_id, str) or _RUN_ID_RE.fullmatch(self.boot_id) is None:
            raise ValueError("boot_id is invalid")
        if not isinstance(self.session_id, str) or not self.session_id.strip():
            raise ValueError("session_id is invalid")
        object.__setattr__(self, "run_dir", Path(self.run_dir).resolve())
        object.__setattr__(
            self,
            "unreal_log_path",
            _authoritative_unreal_log_path(
                self.run_dir,
                self.unreal_log_path,
            ),
        )
        if not isinstance(self.media_toolchain, PinnedPlayableMediaToolchain):
            raise TypeError("media_toolchain must be a pre-launch pinned toolchain")


@dataclass(frozen=True, slots=True)
class _Identity:
    run_id: str
    boot_id: str
    session_id: str
    model_generation: int
    reset_generation: int

    def fields(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "session_id": self.session_id,
            "boot_id": self.boot_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
        }


@dataclass(frozen=True, slots=True)
class _FrameCapture:
    frame_sequence: int
    truth_sequence: int
    sim_time_ns: int
    model_generation: int
    reset_generation: int
    path: Path
    sha256: str
    bytes: int
    nonblack_pixel_fraction: float


@dataclass(frozen=True, slots=True)
class _RuntimeRequestTraceSnapshot:
    source_path: Path
    source_identity: tuple[int, int, int, int, int]
    payload: bytes
    records: tuple[Mapping[str, Any], ...]


class PinnedPlayableMediaToolchain:
    """Pre-launch paths, hashes, byte sizes, and version outputs."""

    def __init__(self, *, ffmpeg: Path, ffprobe: Path) -> None:
        self.paths: Mapping[str, Path] = MappingProxyType(
            {
                "ffmpeg": _absolute_regular_file(ffmpeg, "ffmpeg"),
                "ffprobe": _absolute_regular_file(ffprobe, "ffprobe"),
            }
        )
        mutable_descriptors = {
            name: {"path": str(path), **_stable_file_descriptor(path)}
            for name, path in self.paths.items()
        }
        self.descriptors: Mapping[str, Mapping[str, Any]] = mutable_descriptors
        for name in ("ffmpeg", "ffprobe"):
            completed = self.run(name, ("-version",), timeout_s=30.0)
            output = (completed.stdout or completed.stderr).decode(
                "utf-8", errors="strict"
            )
            version = output.splitlines()[0].strip() if output.splitlines() else ""
            if not version:
                raise PlayableEvidenceError(f"{name} did not report a version")
            mutable_descriptors[name]["version"] = version
        from sim.runtime.qualification.playable import (
            create_pinned_ffmpeg_media_probe,
        )

        self.trusted_probe, trusted_descriptor = (
            create_pinned_ffmpeg_media_probe(
                ffprobe_path=self.paths["ffprobe"],
                ffmpeg_path=self.paths["ffmpeg"],
                ffprobe_version=str(self.descriptors["ffprobe"]["version"]),
                ffmpeg_version=str(self.descriptors["ffmpeg"]["version"]),
            )
        )
        if trusted_descriptor != self.descriptors:
            raise PlayableEvidenceError(
                "qualification media probe did not preserve the pinned toolchain identity"
            )
        self.descriptors = MappingProxyType(
            {
                name: MappingProxyType(dict(descriptor))
                for name, descriptor in mutable_descriptors.items()
            }
        )
        self.trusted_descriptor: Mapping[str, Mapping[str, Any]] = (
            MappingProxyType(
                {
                    name: MappingProxyType(dict(descriptor))
                    for name, descriptor in trusted_descriptor.items()
                }
            )
        )

    def run(
        self,
        name: str,
        arguments: Sequence[str],
        *,
        timeout_s: float,
    ) -> subprocess.CompletedProcess[bytes]:
        """Run one hash-pinned tool and reject replacement or failure."""

        self._require_unchanged()
        path = self.paths[name]
        try:
            completed = subprocess.run(  # noqa: S603 - exact hash-pinned argv.
                [str(path), *arguments],
                check=False,
                capture_output=True,
                timeout=timeout_s,
                shell=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise PlayableEvidenceError(f"pinned {name} execution failed") from exc
        self._require_unchanged()
        if completed.returncode != 0:
            stderr = completed.stderr.decode("utf-8", errors="replace").strip()
            raise PlayableEvidenceError(
                f"pinned {name} rejected evidence: {stderr[-500:]}"
            )
        return completed

    def _require_unchanged(self) -> None:
        for name, expected in self.descriptors.items():
            actual = _stable_file_descriptor(self.paths[name])
            if (
                actual["sha256"] != expected["sha256"]
                or actual["bytes"] != expected["bytes"]
            ):
                raise PlayableEvidenceError(
                    f"pinned {name} executable changed during evidence assembly"
                )

    def require_unchanged(self) -> None:
        """Reject replacement of either external executable."""

        self._require_unchanged()


def snapshot_playable_media_toolchain(
    *,
    ffmpeg: Path,
    ffprobe: Path,
) -> PinnedPlayableMediaToolchain:
    """Pin both media tools before any simulation process is constructed."""

    return PinnedPlayableMediaToolchain(ffmpeg=ffmpeg, ffprobe=ffprobe)


def finalize_playable_evidence(inputs: PlayableEvidenceInputs) -> Path:
    """Build all derived evidence after close and publish the manifest last."""

    if not isinstance(inputs, PlayableEvidenceInputs):
        raise TypeError("inputs must be PlayableEvidenceInputs")
    root = _safe_existing_directory(inputs.run_dir, "run_dir")
    runtime = _read_json(root / "session.runtime.json")
    episode = _read_json(root / "episode_result.json")
    allocation = _read_json(root / "run-allocation.json")
    identity = _identity(inputs, runtime, episode, allocation)
    replay = _open_replay(root, identity)
    raw_recording = _read_json(root / "simulation-recording.json")
    clock = _recording_clock(raw_recording, replay)

    accepted = _read_jsonl(root / "control-intent-accepted.jsonl")
    origins = _read_ue_control_origins(root)
    source_runtime_request_trace_path = root / "runtime-request-trace.jsonl"
    runtime_request_trace_snapshot = _snapshot_runtime_request_trace(
        source_runtime_request_trace_path,
        origins,
        identity,
    )
    zero_audit = _build_control_zero_audit(
        _read_jsonl(root / "control-command-zero-audit.jsonl"),
        origins,
        identity,
    )

    trajectory = _build_trajectory(replay, identity)
    captures = _normalize_recording_captures(
        _select_recording_captures(
            _load_frame_captures(
                root,
                identity,
                unreal_log_path=inputs.unreal_log_path,
            ),
            start_sim_time_ns=int(clock["start_sim_time_ns"]),
            end_sim_time_ns=int(clock["end_sim_time_ns"]),
        )
    )
    if len(captures) < 600:
        raise PlayableEvidenceError("fewer than 600 real 1920x1080 UE frames were captured")
    correlations = _build_correlations(
        accepted,
        origins,
        trajectory,
        captures,
        identity,
    )
    _require_control_zero_audit_covers_maneuvers(zero_audit, correlations, accepted)

    _write_jsonl_new(root / "motion-trajectory.jsonl", trajectory)
    _write_jsonl_new(root / "control-truth-correlation.jsonl", correlations)

    recording_dir = root / "recording"
    videos_dir = root / "videos"
    recording_dir.mkdir(exist_ok=False)
    videos_dir.mkdir(exist_ok=False)
    recording_frames_dir = recording_dir / "frames"
    _materialize_recording_frames(
        captures,
        root,
    )
    capture_map_path = recording_dir / "frame-capture-map.jsonl"
    _write_jsonl_new(
        capture_map_path,
        tuple(
            _frame_capture_document(
                item,
                root,
                identity,
                frame_root=recording_frames_dir,
            )
            for item in captures
        ),
    )

    sensor_summary = _build_sensor_summary(
        runtime,
        episode,
        captures,
        identity,
    )
    _write_json_new(root / "sensor-stream-summary.json", sensor_summary)

    zero_audit_path = recording_dir / "control-command-zero-audit.jsonl"
    _write_jsonl_new(zero_audit_path, zero_audit)

    recording_timeline_path = recording_dir / "recording.timeline.jsonl"
    _write_jsonl_new(
        recording_timeline_path,
        (
            {
                "schema": "lingtu.sim.playable-recording-event.v1",
                **identity.fields(),
                "sequence": 1,
                "sim_time_ns": clock["start_sim_time_ns"],
                "event": "record_start",
            },
            {
                "schema": "lingtu.sim.playable-recording-event.v1",
                **identity.fields(),
                "sequence": 2,
                "sim_time_ns": clock["end_sim_time_ns"],
                "event": "record_stop_commit",
            },
        ),
    )

    tools = inputs.media_toolchain
    tools.require_unchanged()
    media_toolchain_path = recording_dir / "media-toolchain.json"
    _write_json_new(
        media_toolchain_path,
        {
            "schema": "lingtu.sim.playable-media-toolchain.v1",
            **identity.fields(),
            "ffmpeg": tools.descriptors["ffmpeg"],
            "ffprobe": tools.descriptors["ffprobe"],
        },
    )

    controller_calibration_path = recording_dir / "controller-calibration.json"
    _write_json_new(
        controller_calibration_path,
        _controller_calibration_document(identity),
    )

    source_status_path = root / "control-status-authority.jsonl"
    status_payload = _read_stable_bytes(source_status_path)
    _validate_status_authority(status_payload, identity)
    status_path = recording_dir / "control-status-authority.jsonl"
    _write_bytes_new(status_path, status_payload)
    runtime_request_trace_descriptor = _publish_runtime_request_trace(
        runtime_request_trace_snapshot,
        run_root=root,
    )

    duration_ns = int(clock["duration_ns"])
    raw_video = videos_dir / "playable-raw.mp4"
    labeled_video = videos_dir / "playable-labeled.mp4"
    _encode_videos(
        tools,
        frames_dir=recording_frames_dir,
        frame_count=len(captures),
        duration_ns=duration_ns,
        raw_path=raw_video,
        labeled_path=labeled_video,
    )
    raw_probe = _probe_video(tools, raw_video)
    labeled_probe = _probe_video(tools, labeled_video)

    statuses = _strict_jsonl_payload(status_payload, "control status authority")
    hud = _build_hud_descriptors(root, statuses, identity, clock)
    shutdown = _build_shutdown_evidence(inputs, identity, accepted)
    _write_json_new(root / "shutdown-evidence.json", shutdown)

    manifest = {
        "schema": "lingtu.sim.playable-recording-manifest.v1",
        **identity.fields(),
        "state": "COMMITTED",
        "product": dict(PLAYABLE_PRODUCT),
        "clock": clock,
        "timeline": {"path": "recording/recording.timeline.jsonl", **_stable_file_descriptor(recording_timeline_path)},
        "media_toolchain": {"path": "recording/media-toolchain.json", **_stable_file_descriptor(media_toolchain_path)},
        "controller_calibration": {
            "path": "recording/controller-calibration.json",
            **_stable_file_descriptor(controller_calibration_path),
        },
        "control_status_authority": {"path": "recording/control-status-authority.jsonl", **_stable_file_descriptor(status_path)},
        "runtime_request_trace": {
            **runtime_request_trace_descriptor,
        },
        "control_zero_audit": {
            "path": "recording/control-command-zero-audit.jsonl",
            **_stable_file_descriptor(zero_audit_path),
        },
        "frame_capture_map": {"path": "recording/frame-capture-map.jsonl", **_stable_file_descriptor(capture_map_path)},
        "frames": _recording_frames_descriptor(captures),
        "videos": {
            "raw": _video_descriptor(root, raw_video, duration_ns, raw_probe),
            "labeled": _video_descriptor(root, labeled_video, duration_ns, labeled_probe),
        },
        "hud": hud,
    }
    tools.require_unchanged()
    if _read_stable_bytes(source_status_path) != status_payload:
        raise PlayableEvidenceError("control status authority changed after post-close snapshot")
    _require_runtime_request_trace_source_unchanged(
        runtime_request_trace_snapshot,
    )
    _require_runtime_request_trace_archive_unchanged(
        runtime_request_trace_snapshot,
        run_root=root,
        descriptor=runtime_request_trace_descriptor,
    )
    manifest_path = root / PLAYABLE_EVIDENCE_MANIFEST
    _write_json_new(manifest_path, manifest)
    return manifest_path


def _controller_calibration_document(identity: _Identity) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.playable-controller-calibration.v1",
        **identity.fields(),
        "controller_package_id": "thunderv4_locomotion",
        "controller_package_version": "1.0.0",
        "controller_manifest_path": _CONTROLLER_MANIFEST_PATH,
        "command_calibration": _CONTROLLER_COMMAND_CALIBRATION,
        "qualification_schedule": [
            {"maneuver": "turn_left", "key": "Q", "hold_s": 5.3},
            {"maneuver": "turn_right", "key": "E", "hold_s": 5.3},
        ],
        "first_probe_not_prior_qualification": True,
    }


def _identity(
    inputs: PlayableEvidenceInputs,
    runtime: Mapping[str, Any],
    episode: Mapping[str, Any],
    allocation: Mapping[str, Any],
) -> _Identity:
    for document, label in (
        (runtime, "session runtime"),
        (episode, "episode result"),
        (allocation, "run allocation"),
    ):
        for field, expected in (
            ("run_id", inputs.run_id),
            ("session_id", inputs.session_id),
        ):
            if document.get(field) != expected:
                raise PlayableEvidenceError(f"{label} {field} mismatch")
    if allocation.get("boot_id") != inputs.boot_id:
        raise PlayableEvidenceError("run allocation boot_id mismatch")
    model = _non_negative_int(runtime.get("model_generation"), "model_generation")
    reset = _non_negative_int(runtime.get("reset_generation"), "reset_generation")
    if episode.get("model_generation") != model or episode.get("reset_generation") != reset:
        raise PlayableEvidenceError("episode generation does not match final runtime")
    return _Identity(
        run_id=inputs.run_id,
        boot_id=inputs.boot_id,
        session_id=inputs.session_id,
        model_generation=model,
        reset_generation=reset,
    )


def _open_replay(root: Path, identity: _Identity) -> SimulationReplay:
    try:
        replay = SimulationReplay.open(root)
    except SimulationReplayError as exc:
        raise PlayableEvidenceError(f"raw simulation recording is invalid: {exc}") from exc
    if (
        replay.run_id != identity.run_id
        or replay.session_id != identity.session_id
        or replay.model_generation != identity.model_generation
        or replay.end_reset_generation != identity.reset_generation
    ):
        raise PlayableEvidenceError("raw recording identity does not match the closed run")
    return replay


def _recording_clock(
    manifest: Mapping[str, Any], replay: SimulationReplay
) -> dict[str, Any]:
    raw = _mapping(manifest.get("clock"), "simulation recording clock")
    start = _non_negative_int(raw.get("start_sim_time_ns"), "recording start")
    end = _non_negative_int(raw.get("end_sim_time_ns"), "recording end")
    duration = _non_negative_int(raw.get("duration_ns"), "recording duration")
    if (
        raw.get("authority") != "mujoco"
        or end - start != duration
        or duration != replay.duration_ns
        or duration < 20_000_000_000
    ):
        raise PlayableEvidenceError("raw recording clock is not a valid >=20 second MuJoCo interval")
    return {
        "authority": "mujoco",
        "start_sim_time_ns": start,
        "end_sim_time_ns": end,
        "duration_ns": duration,
    }


def _build_trajectory(
    replay: SimulationReplay,
    identity: _Identity,
) -> tuple[dict[str, Any], ...]:
    result: list[dict[str, Any]] = []
    previous_sequence = -1
    previous_time = -1
    for frame in replay.frames:
        snapshot = frame.snapshot
        sequence = _non_negative_int(snapshot.get("sequence"), "truth sequence")
        sim_time_ns = _non_negative_int(snapshot.get("sim_time_ns"), "truth sim_time_ns")
        if sequence <= previous_sequence or sim_time_ns <= previous_time:
            raise PlayableEvidenceError("raw truth recording is not strictly monotonic")
        previous_sequence = sequence
        previous_time = sim_time_ns
        body = _base_body(snapshot)
        position = _finite_vector(body.get("position_m"), 3, "base position")
        quaternion = _normalized_quaternion(body.get("quaternion_wxyz"))
        yaw = _quaternion_yaw(quaternion)
        result.append(
            {
                "schema": "lingtu.sim.motion-trajectory-sample.v1",
                **identity.fields(),
                "sequence": sequence,
                "sim_time_ns": sim_time_ns,
                "position_m": position,
                "quaternion_wxyz": quaternion,
                "yaw_rad": yaw,
            }
        )
    if not result:
        raise PlayableEvidenceError("raw truth recording is empty")
    return tuple(result)


def _base_body(snapshot: Mapping[str, Any]) -> Mapping[str, Any]:
    bodies = snapshot.get("bodies")
    if not isinstance(bodies, Sequence) or isinstance(bodies, (str, bytes)):
        raise PlayableEvidenceError("truth snapshot bodies are invalid")
    matches = [
        item
        for item in bodies
        if isinstance(item, Mapping)
        and item.get("stable_id") == "thunder_01/base_link"
    ]
    if len(matches) != 1:
        raise PlayableEvidenceError("truth snapshot lacks exactly one ThunderV4 base body")
    return matches[0]


def _load_frame_captures(
    root: Path,
    identity: _Identity,
    *,
    unreal_log_path: Path,
) -> tuple[_FrameCapture, ...]:
    log_path = _authoritative_unreal_log_path(root, unreal_log_path)
    text = _read_stable_bytes(log_path).decode("utf-8", errors="strict")
    by_index: dict[int, _FrameCapture] = {}
    for match in _CAPTURE_LOG_RE.finditer(text):
        index = int(match.group("frame"))
        if index in by_index:
            raise PlayableEvidenceError("UE frame capture log repeats a frame index")
        logged_path = Path(match.group("path"))
        expected = root / "frames" / f"frame_{index:06d}.png"
        if not logged_path.is_absolute() or logged_path != expected:
            raise PlayableEvidenceError("UE frame capture path escapes the allocated frame directory")
        try:
            path = _absolute_regular_file(logged_path, f"UE frame {index}")
        except PlayableEvidenceError as exc:
            raise PlayableEvidenceError(
                "UE frame capture path contains a link/reparse point"
            ) from exc
        if path != expected:
            raise PlayableEvidenceError("UE frame capture path is not canonical")
        payload = _read_stable_bytes(path)
        width, height, nonblack = _decode_png(payload, f"UE frame {index}")
        if width != _WIDTH or height != _HEIGHT or nonblack <= 0.0:
            raise PlayableEvidenceError("UE frame is black or not 1920x1080")
        capture = _FrameCapture(
            frame_sequence=index,
            truth_sequence=int(match.group("sequence")),
            sim_time_ns=int(match.group("time")),
            model_generation=int(match.group("model")),
            reset_generation=int(match.group("reset")),
            path=path,
            sha256=hashlib.sha256(payload).hexdigest(),
            bytes=len(payload),
            nonblack_pixel_fraction=nonblack,
        )
        if (
            capture.model_generation != identity.model_generation
            or capture.reset_generation != identity.reset_generation
        ):
            raise PlayableEvidenceError("UE frame capture generation is stale")
        by_index[index] = capture
    captures = tuple(by_index[index] for index in sorted(by_index))
    if not captures or [item.frame_sequence for item in captures] != list(range(len(captures))):
        raise PlayableEvidenceError("UE frame capture log/files are missing or non-contiguous")
    directory_entries = sorted((root / "frames").iterdir(), key=lambda path: path.name)
    if [path.name for path in directory_entries] != [
        f"frame_{index:06d}.png" for index in range(len(captures))
    ]:
        raise PlayableEvidenceError("frame directory differs from the UE capture log")
    return captures


def _authoritative_unreal_log_path(root: Path, path: Path) -> Path:
    candidate = Path(path)
    if (
        not candidate.is_absolute()
        or candidate.parent != root / "logs"
        or candidate.name not in {"Unreal.log", "RobotSimUE.log"}
    ):
        raise PlayableEvidenceError(
            "authoritative UE log must be Unreal.log or RobotSimUE.log under run/logs"
        )
    _reject_link_components(candidate)
    return candidate


def _materialize_recording_frames(
    captures: Sequence[_FrameCapture],
    run_root: Path,
) -> None:
    if not captures:
        raise PlayableEvidenceError("refusing to materialize an empty recording frame set")
    root = _safe_existing_directory(run_root, "recording run directory")
    raw_root = _safe_existing_directory(root / "frames", "raw UE frame directory")
    recording_root = _safe_existing_directory(
        root / "recording",
        "recording directory",
    )
    destination = recording_root / "frames"
    validated_sources: list[tuple[_FrameCapture, bytes]] = []
    source_file_ids: set[tuple[int, int]] = set()
    for expected_sequence, item in enumerate(captures):
        if item.frame_sequence != expected_sequence:
            raise PlayableEvidenceError(
                "recording frames must be normalized to a zero-based contiguous sequence"
            )
        source = Path(item.path)
        try:
            _reject_link_components(source)
            source_parent = source.parent.resolve(strict=True)
        except (OSError, PlayableEvidenceError) as exc:
            raise PlayableEvidenceError(
                "recording frame source contains a link/reparse point"
            ) from exc
        if (
            source_parent != raw_root
            or _FRAME_RE.fullmatch(source.name) is None
        ):
            raise PlayableEvidenceError(
                "recording frame source is outside the allocated raw frame directory"
            )
        payload, source_identity = _read_stable_bytes_with_identity(source)
        source_file_id = source_identity[:2]
        if source_file_id[1] != 0 and source_file_id in source_file_ids:
            raise PlayableEvidenceError(
                "recording frames reuse the same source file identity"
            )
        source_file_ids.add(source_file_id)
        if (
            hashlib.sha256(payload).hexdigest() != item.sha256
            or len(payload) != item.bytes
        ):
            raise PlayableEvidenceError(
                "recording frame source changed after UE capture validation"
            )
        validated_sources.append((item, payload))
    try:
        destination.mkdir(exist_ok=False)
    except OSError as exc:
        raise PlayableEvidenceError(
            "recording frame directory already exists or cannot be created"
        ) from exc
    for item, payload in validated_sources:
        frame_path = destination / f"frame_{item.frame_sequence:06d}.png"
        _write_bytes_new(frame_path, payload)
        copied_payload = _read_stable_bytes(frame_path)
        if copied_payload != payload:
            raise PlayableEvidenceError("recording frame copy does not match the UE frame source")
        if _read_stable_bytes(Path(item.path)) != payload:
            raise PlayableEvidenceError(
                "recording frame source changed while its immutable copy was created"
            )


def _frame_capture_document(
    item: _FrameCapture,
    root: Path,
    identity: _Identity,
    *,
    frame_root: Path | None = None,
) -> dict[str, Any]:
    frame_path = (
        (frame_root / f"frame_{item.frame_sequence:06d}.png").resolve()
        if frame_root is not None
        else item.path
    )
    return {
        "schema": "lingtu.sim.playable-frame-capture.v1",
        **identity.fields(),
        "frame_sequence": item.frame_sequence,
        "truth_sequence": item.truth_sequence,
        "sim_time_ns": item.sim_time_ns,
        "path": frame_path.relative_to(root).as_posix(),
        "sha256": item.sha256,
        "bytes": item.bytes,
    }


def _select_recording_captures(
    captures: Sequence[_FrameCapture],
    *,
    start_sim_time_ns: int,
    end_sim_time_ns: int,
) -> tuple[_FrameCapture, ...]:
    selected = tuple(
        item for item in captures if start_sim_time_ns <= item.sim_time_ns <= end_sim_time_ns
    )
    if not selected:
        raise PlayableEvidenceError("no UE frame capture falls inside the recording interval")
    expected = list(range(selected[0].frame_sequence, selected[-1].frame_sequence + 1))
    if [item.frame_sequence for item in selected] != expected:
        raise PlayableEvidenceError("recording UE frame capture interval is not contiguous")
    return selected


def _normalize_recording_captures(
    captures: Sequence[_FrameCapture],
) -> tuple[_FrameCapture, ...]:
    """Assign isolated recording-frame indices without changing source provenance."""

    if not captures:
        raise PlayableEvidenceError("cannot normalize an empty recording frame set")
    expected = list(
        range(captures[0].frame_sequence, captures[-1].frame_sequence + 1)
    )
    if [item.frame_sequence for item in captures] != expected:
        raise PlayableEvidenceError("raw recording frame interval is not contiguous")
    return tuple(
        _FrameCapture(
            frame_sequence=recording_sequence,
            truth_sequence=item.truth_sequence,
            sim_time_ns=item.sim_time_ns,
            model_generation=item.model_generation,
            reset_generation=item.reset_generation,
            path=item.path,
            sha256=item.sha256,
            bytes=item.bytes,
            nonblack_pixel_fraction=item.nonblack_pixel_fraction,
        )
        for recording_sequence, item in enumerate(captures)
    )


def _recording_frames_descriptor(
    captures: Sequence[_FrameCapture],
) -> dict[str, Any]:
    if not captures or [item.frame_sequence for item in captures] != list(
        range(len(captures))
    ):
        raise PlayableEvidenceError(
            "recording frame descriptor requires a zero-based contiguous set"
        )
    return {
        "directory": "recording/frames",
        "count": len(captures),
        "first_sequence": 0,
        "last_sequence": len(captures) - 1,
        "width": _WIDTH,
        "height": _HEIGHT,
    }


def _build_correlations(
    records: Sequence[Mapping[str, Any]],
    origins: Sequence[Mapping[str, Any]],
    trajectory: Sequence[Mapping[str, Any]],
    captures: Sequence[_FrameCapture],
    identity: _Identity,
) -> tuple[dict[str, Any], ...]:
    origins_by_event = _successful_origins(origins, identity)
    motions: list[tuple[str, Mapping[str, Any]]] = []
    final_zero: Mapping[str, Any] | None = None
    for item in records:
        _require_identity(item, identity, "accepted control")
        event_id = _text(item.get("event_id"), "accepted event_id")
        origin = origins_by_event.get(event_id)
        if origin is None or any(
            item.get(field) != origin.get(field)
            for field in ("source_id", "source_epoch", "source_sequence", "datagram_sha256")
        ):
            raise PlayableEvidenceError("accepted control lacks exact UE successful-send origin")
        twist = _twist(item.get("admitted_twist"))
        if item.get("schema") == "lingtu.sim.control-command-zero.v1":
            if item.get("reason") != "cleared:exit" or any(twist.values()):
                raise PlayableEvidenceError("accepted zero is not the final cleared:exit zero")
            if final_zero is not None:
                raise PlayableEvidenceError("accepted control has more than one final exit zero")
            final_zero = item
            continue
        if item.get("schema") != "lingtu.sim.control-intent-accepted.v1":
            raise PlayableEvidenceError("accepted control schema is invalid")
        motions.append((_maneuver_for_twist(twist), item))
    if final_zero is None:
        raise PlayableEvidenceError("accepted control lacks the final exit zero")
    groups: list[tuple[str, list[Mapping[str, Any]]]] = []
    for maneuver, item in motions:
        if not groups or groups[-1][0] != maneuver:
            groups.append((maneuver, []))
        groups[-1][1].append(item)
    if tuple(name for name, _items in groups) != _MANEUVERS:
        raise PlayableEvidenceError("accepted control does not contain exactly W/S/A/D/Q/E segments")

    trajectory_by_time = {int(item["sim_time_ns"]): index for index, item in enumerate(trajectory)}
    correlations: list[dict[str, Any]] = []
    for index, (maneuver, segment) in enumerate(groups):
        first = segment[0]
        last = segment[-1]
        start_time = _non_negative_int(first.get("apply_time_ns"), "segment start apply time")
        start_index = trajectory_by_time.get(start_time)
        if start_index is None:
            raise PlayableEvidenceError("accepted command has no exact MuJoCo truth sample")
        if index + 1 < len(groups):
            next_time = _non_negative_int(
                groups[index + 1][1][0].get("apply_time_ns"),
                "next segment apply time",
            )
            next_index = trajectory_by_time.get(next_time)
            if next_index is None or next_index <= start_index:
                raise PlayableEvidenceError("maneuver truth intervals are reordered")
            end_index = next_index - 1
        else:
            end_index = len(trajectory) - 1
        end_time = int(trajectory[end_index]["sim_time_ns"])
        if int(last["apply_time_ns"]) > end_time:
            raise PlayableEvidenceError("maneuver accepted interval exceeds truth")
        mapped = [
            frame
            for frame in captures
            if start_time <= frame.sim_time_ns <= end_time
        ]
        if not mapped:
            raise PlayableEvidenceError("maneuver has no real UE frame mapping")
        digest_records = [
            {
                field: item[field]
                for field in (
                    "event_id",
                    "source_id",
                    "source_epoch",
                    "source_sequence",
                    "datagram_sha256",
                    "controller_sequence",
                    "apply_time_ns",
                )
            }
            for item in segment
        ]
        first_truth = trajectory[start_index]
        last_truth = trajectory[end_index]
        correlations.append(
            {
                "schema": "lingtu.sim.control-truth-correlation.v2",
                **identity.fields(),
                "source_id": first["source_id"],
                "maneuver": maneuver,
                "start_event_id": first["event_id"],
                "end_event_id": last["event_id"],
                "start_source_epoch": first["source_epoch"],
                "start_source_sequence": first["source_sequence"],
                "end_source_epoch": last["source_epoch"],
                "end_source_sequence": last["source_sequence"],
                "start_datagram_sha256": first["datagram_sha256"],
                "end_datagram_sha256": last["datagram_sha256"],
                "start_controller_sequence": first["controller_sequence"],
                "end_controller_sequence": last["controller_sequence"],
                "start_apply_time_ns": first["apply_time_ns"],
                "end_apply_time_ns": last["apply_time_ns"],
                "accepted_event_count": len(segment),
                "accepted_events_sha256": hashlib.sha256(
                    _canonical_json(digest_records)
                ).hexdigest(),
                "truth_sequence_start": first_truth["sequence"],
                "truth_sequence_end": last_truth["sequence"],
                "truth_sim_time_ns_start": first_truth["sim_time_ns"],
                "truth_sim_time_ns_end": last_truth["sim_time_ns"],
                "truth_position_m_start": first_truth["position_m"],
                "truth_position_m_end": last_truth["position_m"],
                "truth_quaternion_wxyz_start": first_truth["quaternion_wxyz"],
                "truth_quaternion_wxyz_end": last_truth["quaternion_wxyz"],
                "frame_sequence_start": mapped[0].frame_sequence,
                "frame_sequence_end": mapped[-1].frame_sequence,
            }
        )
    return tuple(correlations)


def _successful_origins(
    origins: Sequence[Mapping[str, Any]], identity: _Identity
) -> dict[str, Mapping[str, Any]]:
    result: dict[str, Mapping[str, Any]] = {}
    for item in origins:
        _require_identity(item, identity, "UE origin")
        if item.get("schema") != "lingtu.sim.ue-control-origin.v1" or item.get("successful_send") is not True:
            continue
        event_id = _text(item.get("event_id"), "UE origin event_id")
        if event_id in result:
            raise PlayableEvidenceError("UE successful-send origin repeats event_id")
        result[event_id] = item
    return result


def _require_recording_requests(
    traces: Sequence[Mapping[str, Any]],
    origins: Sequence[Mapping[str, Any]],
    identity: _Identity,
) -> None:
    if not traces or len(traces) % 2 != 0:
        raise PlayableEvidenceError(
            "runtime request trace is missing a contiguous received/terminal pair"
        )
    successful = _successful_origins(origins, identity)
    key_requests: list[str] = []
    required_request_kinds = frozenset(_REQUIRED_RECORDING_REQUESTS)
    seen_event_ids: set[str] = set()
    seen_datagrams: set[str] = set()
    previous_order: tuple[int, int] | None = None
    previous_source_time = -1
    previous_arrival_time = -1
    for pair_offset in range(0, len(traces), 2):
        received = traces[pair_offset]
        terminal = traces[pair_offset + 1]
        if set(received) != _RUNTIME_REQUEST_TRACE_RECEIVED_FIELDS or (
            set(terminal) != _RUNTIME_REQUEST_TRACE_TERMINAL_FIELDS
        ):
            raise PlayableEvidenceError(
                "runtime request trace pair fields are invalid"
            )
        for item, expected_event in (
            (received, "runtime_request_received"),
            (terminal, "runtime_request_accepted"),
        ):
            _require_identity(item, identity, "runtime request")
            if item.get("schema") != "lingtu.sim.runtime-request-trace.v1":
                raise PlayableEvidenceError("runtime request trace schema is invalid")
            if item.get("event") != expected_event:
                raise PlayableEvidenceError(
                    "runtime request trace is not a contiguous received/terminal pair"
                )
            if item.get("source_id") != "robotsimue.local_player.0":
                raise PlayableEvidenceError(
                    "runtime request source is not RobotSimUE"
                )
            epoch = _positive_int(
                item.get("source_epoch"),
                "runtime request source_epoch",
            )
            sequence = _positive_int(
                item.get("source_sequence"),
                "runtime request source_sequence",
            )
            if item.get("event_id") != f"{identity.boot_id}:{epoch}:{sequence}":
                raise PlayableEvidenceError(
                    "runtime request event_id/sequence join is invalid"
                )
            _non_negative_int(
                item.get("source_monotonic_ns"),
                "runtime request source_monotonic_ns",
            )
            _non_negative_int(
                item.get("arrival_monotonic_ns"),
                "runtime request arrival_monotonic_ns",
            )
        if any(
            received.get(field) != terminal.get(field)
            for field in _RUNTIME_REQUEST_TRACE_JOIN_FIELDS
        ):
            raise PlayableEvidenceError(
                "runtime request received/terminal correlation was altered"
            )
        request = _text(received.get("request"), "runtime request kind")
        status = terminal.get("status")
        reason = terminal.get("reason")
        if status == "accepted":
            if request not in _ALLOWED_ACCEPTED_RUNTIME_REQUESTS:
                raise PlayableEvidenceError(
                    f"unexpected accepted runtime request: {request}"
                )
            if reason != "":
                raise PlayableEvidenceError(
                    "accepted runtime request contains a non-empty reason"
                )
        elif status == "pending":
            if request not in _ALLOWED_PENDING_RUNTIME_REQUESTS:
                raise PlayableEvidenceError(
                    f"unexpected pending runtime request: {request}"
                )
            if reason != f"safe_zero_pending:{request}":
                raise PlayableEvidenceError(
                    "pending runtime request reason is not the fixed safe-zero transition"
                )
        else:
            raise PlayableEvidenceError("runtime request trace status is invalid")
        event_id = _text(received.get("event_id"), "runtime request event_id")
        if event_id in seen_event_ids:
            raise PlayableEvidenceError("runtime request trace repeats event_id")
        seen_event_ids.add(event_id)
        datagram_sha256 = _text(
            received.get("datagram_sha256"),
            "runtime request datagram_sha256",
        )
        if (
            _DIGEST_RE.fullmatch(datagram_sha256) is None
            or datagram_sha256 in seen_datagrams
        ):
            raise PlayableEvidenceError(
                "runtime request trace reuses an invalid datagram hash"
            )
        seen_datagrams.add(datagram_sha256)
        origin = successful.get(event_id)
        if origin is None or any(
            received.get(field) != origin.get(field)
            for field in (
                "source_id",
                "source_epoch",
                "source_sequence",
                "datagram_sha256",
            )
        ):
            raise PlayableEvidenceError("recording request lacks its UE successful-send hash")
        order = (
            int(received["source_epoch"]),
            int(received["source_sequence"]),
        )
        source_time = int(received["source_monotonic_ns"])
        arrival_time = int(received["arrival_monotonic_ns"])
        if previous_order is not None and order <= previous_order:
            raise PlayableEvidenceError(
                "runtime request source sequence is missing, duplicated, or reordered"
            )
        if source_time < previous_source_time or arrival_time < previous_arrival_time:
            raise PlayableEvidenceError(
                "runtime request monotonic time moved backward"
            )
        previous_order = order
        previous_source_time = source_time
        previous_arrival_time = arrival_time
        if status == "accepted" and request in required_request_kinds:
            key_requests.append(request)
    if key_requests != list(_REQUIRED_RECORDING_REQUESTS):
        raise PlayableEvidenceError(
            "runtime request core must be exactly record_start, pause, resume, "
            "record_stop_commit, pause, exit"
        )


def _snapshot_runtime_request_trace(
    source_path: Path,
    origins: Sequence[Mapping[str, Any]],
    identity: _Identity,
) -> _RuntimeRequestTraceSnapshot:
    """Read and validate one byte-stable post-close runtime-request trace."""

    source = _absolute_regular_file(source_path, "runtime request trace source")
    payload, source_identity = _read_stable_bytes_with_identity(source)
    records = tuple(
        _strict_jsonl_payload(payload, "runtime request trace")
    )
    _require_recording_requests(records, origins, identity)
    return _RuntimeRequestTraceSnapshot(
        source_path=source,
        source_identity=source_identity,
        payload=payload,
        records=records,
    )


def _require_runtime_request_trace_source_unchanged(
    snapshot: _RuntimeRequestTraceSnapshot,
) -> None:
    try:
        current, source_identity = _read_stable_bytes_with_identity(
            snapshot.source_path
        )
    except PlayableEvidenceError as exc:
        raise PlayableEvidenceError(
            "runtime request trace source changed after validation"
        ) from exc
    if (
        current != snapshot.payload
        or source_identity != snapshot.source_identity
    ):
        raise PlayableEvidenceError(
            "runtime request trace source changed after validation"
        )


def _publish_runtime_request_trace(
    snapshot: _RuntimeRequestTraceSnapshot,
    *,
    run_root: Path,
) -> dict[str, Any]:
    """Copy the validated bytes into the run-owned recording snapshot."""

    root = _safe_existing_directory(run_root, "recording run directory")
    expected_source = root / "runtime-request-trace.jsonl"
    if snapshot.source_path != expected_source:
        raise PlayableEvidenceError(
            "runtime request trace source is not owned by the allocated run"
        )
    recording_root = _safe_existing_directory(
        root / "recording",
        "recording directory",
    )
    destination_path = recording_root / "runtime-request-trace.jsonl"
    _require_runtime_request_trace_source_unchanged(snapshot)
    _write_bytes_new(destination_path, snapshot.payload)
    copied = _read_stable_bytes(destination_path)
    if copied != snapshot.payload:
        raise PlayableEvidenceError(
            "runtime request trace archive differs from the validated source"
        )
    _require_runtime_request_trace_source_unchanged(snapshot)
    return {
        "schema": "lingtu.sim.runtime-request-trace-descriptor.v1",
        "content_schema": "lingtu.sim.runtime-request-trace.v1",
        "path": "recording/runtime-request-trace.jsonl",
        "sha256": hashlib.sha256(copied).hexdigest(),
        "bytes": len(copied),
        "record_count": len(snapshot.records),
    }


def _require_runtime_request_trace_archive_unchanged(
    snapshot: _RuntimeRequestTraceSnapshot,
    *,
    run_root: Path,
    descriptor: Mapping[str, Any],
) -> None:
    root = _safe_existing_directory(run_root, "recording run directory")
    archived_path = root / "recording" / "runtime-request-trace.jsonl"
    try:
        archived = _read_stable_bytes(archived_path)
    except PlayableEvidenceError as exc:
        raise PlayableEvidenceError(
            "runtime request trace archive changed before manifest commit"
        ) from exc
    if (
        archived != snapshot.payload
        or descriptor.get("sha256") != hashlib.sha256(archived).hexdigest()
        or descriptor.get("bytes") != len(archived)
        or descriptor.get("record_count") != len(snapshot.records)
    ):
        raise PlayableEvidenceError(
            "runtime request trace archive changed before manifest commit"
        )


def _build_sensor_summary(
    runtime: Mapping[str, Any],
    episode: Mapping[str, Any],
    captures: Sequence[_FrameCapture],
    identity: _Identity,
) -> dict[str, Any]:
    sensor_runtime = _mapping(runtime.get("sensor_streams"), "runtime sensor_streams")
    raw = _mapping(sensor_runtime.get("summary"), "runtime sensor summary")
    streams = _mapping(raw.get("streams"), "runtime sensor summary streams")
    if list(sorted(streams)) != list(sorted(_SENSOR_RATES)):
        raise PlayableEvidenceError("runtime sensor summary does not contain the exact five streams")
    episode_end = _non_negative_int(episode.get("end_sim_time_ns"), "episode end")
    output: list[dict[str, Any]] = []
    blockers: dict[str, str] = {}
    for stream_id in sorted(_SENSOR_RATES):
        item = _mapping(streams.get(stream_id), f"sensor {stream_id}")
        state = item.get("state")
        count = _non_negative_int(item.get("sample_count"), f"sensor {stream_id} count")
        last_truth_sequence = _non_negative_int(
            item.get("last_sample_truth_sequence"),
            f"sensor {stream_id} last truth sequence",
        )
        last = _non_negative_int(
            item.get("last_sample_sim_time_ns"),
            f"sensor {stream_id} last sample time",
        )
        age = episode_end - last
        if state != "ACTIVE" or count <= 0 or age < 0:
            blockers[stream_id] = "runtime stream is not current-generation ACTIVE evidence"
        mapped_frame_count: int | None = None
        if stream_id in _CAMERA_STREAMS:
            mapped_frame_count = len(captures)
            if mapped_frame_count > count:
                raise PlayableEvidenceError(
                    f"sensor {stream_id} camera frames exceed published samples"
                )
        output.append(
            {
                "stream_id": stream_id,
                "state": state,
                "sample_count": count,
                "rate_hz": _SENSOR_RATES[stream_id],
                "clock_domain": "mujoco_sim_time",
                "last_sample_truth_sequence": last_truth_sequence,
                "last_sample_sim_time_ns": last,
                "sample_age_ns": age,
                "mapped_frame_count": mapped_frame_count,
            }
        )
    return {
        "schema": "lingtu.sim.playable-sensor-stream-summary.v1",
        **identity.fields(),
        "is_ready": not blockers,
        "required_stream_ids": sorted(_SENSOR_RATES),
        "blocking_reasons": blockers,
        "streams": output,
    }


def _build_control_zero_audit(
    documents: Sequence[Mapping[str, Any]],
    origins: Sequence[Mapping[str, Any]],
    identity: _Identity,
) -> tuple[Mapping[str, Any], ...]:
    successful = _successful_origins(origins, identity)
    if not documents:
        raise PlayableEvidenceError("control-command-zero-audit evidence is empty")
    seen_events: set[str] = set()
    qualifying_segment_zeros = 0
    seen_qualifying_reasons: set[str] = set()
    previous_apply_time = -1
    previous_controller_sequence = -1
    output: list[Mapping[str, Any]] = []
    for index, item in enumerate(documents):
        _require_identity(item, identity, f"control zero audit line {index + 1}")
        if item.get("schema") != "lingtu.sim.control-command-zero.v1":
            raise PlayableEvidenceError("control zero audit schema is invalid")
        if item.get("event") != "control_command_zero":
            raise PlayableEvidenceError("control zero audit event is invalid")
        if item.get("submit_result") != "accepted":
            raise PlayableEvidenceError("control zero audit was not accepted by the controller")
        reason = _text(item.get("reason"), "control zero audit reason")
        if reason == "cleared:exit":
            raise PlayableEvidenceError("control zero audit contains a second cleared:exit zero")
        if reason not in _ALLOWED_ZERO_AUDIT_REASONS:
            raise PlayableEvidenceError(f"unexpected control zero audit reason: {reason}")
        twist = _twist(item.get("admitted_twist"))
        if any(value != 0.0 for value in twist.values()):
            raise PlayableEvidenceError("control zero audit command is not zero")
        event_id = _text(item.get("event_id"), "control zero audit event_id")
        if event_id in seen_events:
            raise PlayableEvidenceError("control zero audit repeats event_id")
        seen_events.add(event_id)
        origin = successful.get(event_id)
        if origin is None or any(
            item.get(field) != origin.get(field)
            for field in (
                "source_id",
                "source_epoch",
                "source_sequence",
                "datagram_sha256",
            )
        ):
            raise PlayableEvidenceError("control zero audit lacks its UE successful-send hash")
        controller_sequence = _positive_int(
            item.get("controller_sequence"),
            "control zero audit controller_sequence",
        )
        apply_time = _non_negative_int(
            item.get("apply_time_ns"),
            "control zero audit apply_time_ns",
        )
        if controller_sequence <= previous_controller_sequence or apply_time <= previous_apply_time:
            raise PlayableEvidenceError("control zero audit is not monotonic")
        previous_controller_sequence = controller_sequence
        previous_apply_time = apply_time
        if reason in _QUALIFYING_SAFETY_ZERO_REASONS:
            qualifying_segment_zeros += 1
            seen_qualifying_reasons.add(reason)
        output.append(item)
    if qualifying_segment_zeros < len(_MANEUVERS) or seen_qualifying_reasons != set(
        _QUALIFYING_SAFETY_ZERO_REASONS
    ):
        raise PlayableEvidenceError(
            "control zero audit does not prove each released maneuver safety zero"
        )
    return tuple(output)


def _require_control_zero_audit_covers_maneuvers(
    zero_audit: Sequence[Mapping[str, Any]],
    correlations: Sequence[Mapping[str, Any]],
    accepted: Sequence[Mapping[str, Any]],
) -> None:
    qualifying = [
        item
        for item in zero_audit
        if item.get("reason") in _QUALIFYING_SAFETY_ZERO_REASONS
    ]
    final_zero = next(
        (
            item
            for item in accepted
            if item.get("schema") == "lingtu.sim.control-command-zero.v1"
            and item.get("reason") == "cleared:exit"
        ),
        None,
    )
    if final_zero is None:
        raise PlayableEvidenceError("accepted control lacks the final exit zero")
    final_exit_time = _non_negative_int(
        final_zero.get("apply_time_ns"),
        "final exit zero apply_time_ns",
    )
    for index, correlation in enumerate(correlations):
        lower = _non_negative_int(
            correlation.get("end_apply_time_ns"),
            f"correlation {index + 1} end_apply_time_ns",
        )
        upper = (
            _non_negative_int(
                correlations[index + 1].get("start_apply_time_ns"),
                f"correlation {index + 2} start_apply_time_ns",
            )
            if index + 1 < len(correlations)
            else final_exit_time
        )
        if upper <= lower:
            raise PlayableEvidenceError("maneuver release interval is reordered")
        if not any(
            lower < _non_negative_int(item.get("apply_time_ns"), "control zero audit apply_time_ns") < upper
            for item in qualifying
        ):
            raise PlayableEvidenceError(
                "control zero audit does not prove each released maneuver safety zero"
            )


def _encode_videos(
    tools: PinnedPlayableMediaToolchain,
    *,
    frames_dir: Path,
    frame_count: int,
    duration_ns: int,
    raw_path: Path,
    labeled_path: Path,
) -> None:
    if frame_count < 600 or duration_ns < 20_000_000_000:
        raise PlayableEvidenceError("video input does not meet the playable duration/frame minimum")
    frame_rate = f"{frame_count * 1_000_000_000}/{duration_ns}"
    input_arguments = (
        "-hide_banner",
        "-loglevel",
        "error",
        "-xerror",
        "-nostdin",
        "-framerate",
        frame_rate,
        "-start_number",
        "0",
        "-i",
        str(frames_dir / "frame_%06d.png"),
        "-frames:v",
        str(frame_count),
    )
    encoding = (
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
    )
    tools.run(
        "ffmpeg",
        (*input_arguments, *encoding, "-y", str(raw_path)),
        timeout_s=600.0,
    )
    label_filter = (
        "drawbox=x=0:y=0:w=iw:h=72:color=black@0.55:t=fill,"
        "drawtext=text='LingTu UE5  FactoryPark  ThunderV4':"
        "x=24:y=22:fontsize=28:fontcolor=white"
    )
    tools.run(
        "ffmpeg",
        (*input_arguments, "-vf", label_filter, *encoding, "-y", str(labeled_path)),
        timeout_s=600.0,
    )


def _probe_video(
    tools: PinnedPlayableMediaToolchain,
    path: Path,
) -> dict[str, int]:
    completed = tools.run(
        "ffprobe",
        (
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_frames",
            "-show_entries",
            "stream=width,height,duration,nb_read_frames,nb_frames:format=duration",
            "-of",
            "json",
            str(path),
        ),
        timeout_s=120.0,
    )
    document = _strict_json_object(completed.stdout, "ffprobe output")
    streams = document.get("streams")
    if not isinstance(streams, Sequence) or isinstance(streams, (str, bytes)) or len(streams) != 1:
        raise PlayableEvidenceError("ffprobe did not find exactly one video stream")
    stream = _mapping(streams[0], "ffprobe stream")
    width = _positive_int(stream.get("width"), "video width")
    height = _positive_int(stream.get("height"), "video height")
    raw_frames = stream.get("nb_read_frames")
    if raw_frames in {None, "N/A"}:
        raw_frames = stream.get("nb_frames")
    frames = int(str(raw_frames), 10)
    raw_duration = stream.get("duration")
    if raw_duration in {None, "N/A"}:
        raw_duration = _mapping(document.get("format"), "ffprobe format").get("duration")
    duration_ns = round(float(str(raw_duration)) * 1_000_000_000)
    if width != _WIDTH or height != _HEIGHT or frames < 600 or duration_ns <= 0:
        raise PlayableEvidenceError("encoded video geometry, duration, or frame count is invalid")
    tools.run(
        "ffmpeg",
        (
            "-hide_banner",
            "-loglevel",
            "error",
            "-xerror",
            "-nostdin",
            "-i",
            str(path),
            "-map",
            "0:v:0",
            "-f",
            "null",
            "-",
        ),
        timeout_s=600.0,
    )
    black = tools.run(
        "ffmpeg",
        (
            "-hide_banner",
            "-nostdin",
            "-i",
            str(path),
            "-vf",
            "blackdetect=d=0.01:pic_th=0.98",
            "-an",
            "-f",
            "null",
            "-",
        ),
        timeout_s=600.0,
    )
    if b"black_start:" in black.stderr:
        raise PlayableEvidenceError("encoded video contains a detected black interval")
    return {
        "width": width,
        "height": height,
        "duration_ns": duration_ns,
        "frame_count": frames,
        "decode_error_count": 0,
        "black_frame_count": 0,
    }


def _video_descriptor(
    root: Path,
    path: Path,
    duration_ns: int,
    probe: Mapping[str, int],
) -> dict[str, Any]:
    if abs(int(probe["duration_ns"]) - duration_ns) > 100_000_000:
        raise PlayableEvidenceError("encoded video duration differs from the recording clock")
    return {
        "path": path.relative_to(root).as_posix(),
        **_stable_file_descriptor(path),
        "duration_ns": duration_ns,
        "width": probe["width"],
        "height": probe["height"],
        "decoded": True,
        "decode_error_count": probe["decode_error_count"],
        "black_frame_count": probe["black_frame_count"],
    }


def _validate_status_authority(payload: bytes, identity: _Identity) -> None:
    from .control_status import encode_control_status

    previous = 0
    for item in _strict_jsonl_payload(payload, "control status authority"):
        _require_identity(item, identity, "control status authority")
        try:
            encode_control_status(item)
        except Exception as exc:
            raise PlayableEvidenceError("control status authority is not exact wire data") from exc
        sequence = _positive_int(item.get("server_status_sequence"), "status sequence")
        if sequence <= previous:
            raise PlayableEvidenceError("control status sequence is not strictly increasing")
        previous = sequence


def _build_hud_descriptors(
    root: Path,
    statuses: Sequence[Mapping[str, Any]],
    identity: _Identity,
    clock: Mapping[str, Any],
) -> dict[str, Any]:
    by_sequence = {
        _positive_int(item.get("server_status_sequence"), "status sequence"): item
        for item in statuses
    }
    paths = {
        "drive": "screenshots/hud-drive.png",
        "tactical": "screenshots/hud-tactical.png",
        "menu_recording": "screenshots/hud-menu-recording.png",
    }
    output: dict[str, Any] = {}
    for mode, relative in paths.items():
        path = root / relative
        payload = _read_stable_bytes(path)
        width, height, nonblack = _decode_png(payload, f"HUD {mode}")
        sidecar_path = path.with_suffix(".evidence.json")
        sidecar_payload = _read_stable_bytes(sidecar_path)
        sidecar = _strict_json_object(sidecar_payload, f"HUD {mode} sidecar")
        _require_identity(sidecar, identity, f"HUD {mode} sidecar")
        control = _mapping(sidecar.get("control_status"), f"HUD {mode} control status")
        server_sequence = _positive_int(control.get("server_status_sequence"), "HUD status sequence")
        authority = by_sequence.get(server_sequence)
        if authority is None:
            raise PlayableEvidenceError(f"HUD {mode} has no authoritative status")
        captured = _non_negative_int(authority.get("sim_time_ns"), "HUD capture sim time")
        if not int(clock["start_sim_time_ns"]) <= captured <= int(clock["end_sim_time_ns"]):
            raise PlayableEvidenceError(f"HUD {mode} was captured outside the recording")
        recording = _mapping(authority.get("recording"), "HUD recording status")
        sidecar_recording = _mapping(
            sidecar.get("recording"),
            f"HUD {mode} sidecar recording status",
        )
        if (
            recording.get("state") != "recording"
            or dict(sidecar_recording) != dict(recording)
        ):
            raise PlayableEvidenceError(f"HUD {mode} was not captured while recording")
        camera_mode = _text(control.get("camera_mode"), "HUD camera_mode")
        output[mode] = {
            "path": relative,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "bytes": len(payload),
            "sidecar_path": sidecar_path.relative_to(root).as_posix(),
            "sidecar_sha256": hashlib.sha256(sidecar_payload).hexdigest(),
            "sidecar_bytes": len(sidecar_payload),
            "server_status_sequence": server_sequence,
            "camera_mode": camera_mode,
            "width": width,
            "height": height,
            "captured_sim_time_ns": captured,
            "snapshot_age_ns": _non_negative_int(sidecar.get("status_age_ns"), "HUD status age"),
            "snapshot_fresh": sidecar.get("qualification_ready") is True,
            "motion_fields": ["requested", "admitted", "observed"],
            "recording_state": sidecar_recording["state"],
            "nonblack_pixel_fraction": nonblack,
        }
    return output


def _build_shutdown_evidence(
    inputs: PlayableEvidenceInputs,
    identity: _Identity,
    accepted: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    final_zeros = [
        item
        for item in accepted
        if item.get("schema") == "lingtu.sim.control-command-zero.v1"
        and item.get("reason") == "cleared:exit"
    ]
    if len(final_zeros) != 1:
        raise PlayableEvidenceError("shutdown lacks exactly one final cleared:exit zero")
    expected_resources = {
        "control_intent_udp",
        "control_status_udp",
        "sensors",
        "recording",
    }
    resources = dict(inputs.resources_closed)
    if set(resources) != expected_resources or not all(value is True for value in resources.values()):
        raise PlayableEvidenceError("runtime resources did not all close successfully")
    processes = []
    for kind, snapshot in (
        ("robotsimue", inputs.unreal_shutdown),
        ("mujoco", inputs.mujoco_shutdown),
    ):
        if snapshot is None:
            raise PlayableEvidenceError(f"{kind} has no immutable shutdown snapshot")
        processes.append(
            {
                "kind": kind,
                "pid": snapshot.pid,
                "owned": True,
                "exit_code": snapshot.exit_code,
                "direct_child_running_after_close": snapshot.direct_child_running_after_close,
                "process_owner_closed": snapshot.process_owner_closed,
                "termination_mode": snapshot.termination_mode,
            }
        )
    return {
        "schema": "lingtu.sim.playable-shutdown-evidence.v1",
        **identity.fields(),
        "natural_shutdown": all(
            item["termination_mode"] == "natural"
            and item["exit_code"] == 0
            and item["direct_child_running_after_close"] is False
            and item["process_owner_closed"] is True
            for item in processes
        ),
        "final_zero_event_id": final_zeros[0]["event_id"],
        "resources_closed": resources,
        "owned_processes": processes,
        "scan_monotonic_ns": max(1, time.monotonic_ns()),
    }


def _maneuver_for_twist(twist: Mapping[str, float]) -> str:
    nonzero = [field for field, value in twist.items() if value != 0.0]
    if len(nonzero) != 1:
        raise PlayableEvidenceError("accepted playable motion is not single-axis")
    field = nonzero[0]
    value = twist[field]
    mapping = {
        ("linear_x", 1): "forward",
        ("linear_x", -1): "backward",
        ("linear_y", 1): "left",
        ("linear_y", -1): "right",
        ("angular_z", 1): "turn_left",
        ("angular_z", -1): "turn_right",
    }
    return mapping[(field, 1 if value > 0 else -1)]


def _twist(value: object) -> dict[str, float]:
    raw = _mapping(value, "admitted_twist")
    if set(raw) != {"linear_x", "linear_y", "angular_z"}:
        raise PlayableEvidenceError("admitted_twist fields are invalid")
    return {
        field: _finite_number(raw[field], f"admitted_twist.{field}")
        for field in ("linear_x", "linear_y", "angular_z")
    }


def _require_identity(
    document: Mapping[str, Any], identity: _Identity, label: str
) -> None:
    for field, expected in identity.fields().items():
        if document.get(field) != expected:
            raise PlayableEvidenceError(f"{label} {field} mismatch")


def _decode_png(payload: bytes, label: str) -> tuple[int, int, float]:
    # The builder decodes before publishing descriptors; the qualification
    # process later performs a second decode from its own immutable snapshot.
    from sim.runtime.qualification.playable import _strict_png_decode

    try:
        return cast(
            tuple[int, int, float],
            _strict_png_decode(
                payload,
                label,
                expected_width=_WIDTH,
                expected_height=_HEIGHT,
            ),
        )
    except Exception as exc:
        raise PlayableEvidenceError(f"{label} is not a strict 1920x1080 PNG") from exc


def _normalized_quaternion(value: object) -> list[float]:
    quaternion = _finite_vector(value, 4, "base quaternion")
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 0.0:
        raise PlayableEvidenceError("base quaternion is zero")
    normalized = [component / norm for component in quaternion]
    if abs(norm - 1.0) > 1e-6:
        raise PlayableEvidenceError("base quaternion is not normalized truth")
    return normalized


def _quaternion_yaw(value: Sequence[float]) -> float:
    w, x, y, z = value
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _read_json(path: Path) -> Mapping[str, Any]:
    return _strict_json_object(_read_stable_bytes(path), path.name)


def _read_jsonl(path: Path) -> tuple[Mapping[str, Any], ...]:
    return read_stable_playable_jsonl(path)


def read_stable_playable_jsonl(path: Path) -> tuple[Mapping[str, Any], ...]:
    """Read one completed link-free JSONL artifact under a stable identity."""

    path = Path(path)
    return tuple(_strict_jsonl_payload(_read_stable_bytes(path), path.name))


def _read_ue_control_origins(root: Path) -> tuple[Mapping[str, Any], ...]:
    """Read the C++ transport's post-close origin journal without fallback."""

    return _read_jsonl(root / "logs" / "ue-control-origin.jsonl")


def _strict_jsonl_payload(payload: bytes, label: str) -> list[Mapping[str, Any]]:
    if not payload or not payload.endswith(b"\n"):
        raise PlayableEvidenceError(f"{label} is empty or partial")
    lines = payload.splitlines()
    if not lines or any(not line for line in lines):
        raise PlayableEvidenceError(f"{label} contains an empty record")
    return [
        _strict_json_object(line, f"{label} line {index}")
        for index, line in enumerate(lines, 1)
    ]


def _strict_json_object(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite constant {token}")
            ),
        )
    except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise PlayableEvidenceError(f"{label} is not strict JSON") from exc
    if type(value) is not dict:
        raise PlayableEvidenceError(f"{label} must be a JSON object")
    return value


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate JSON key {key}")
        value[key] = item
    return value


def _read_stable_bytes(path: Path) -> bytes:
    payload, _identity = _read_stable_bytes_with_identity(path)
    return payload


def _read_stable_bytes_with_identity(
    path: Path,
) -> tuple[bytes, tuple[int, int, int, int, int]]:
    _reject_link_components(path)
    try:
        before = os.lstat(path)
        with path.open("rb") as stream:
            handle_before = os.fstat(stream.fileno())
            payload = stream.read()
            handle_after = os.fstat(stream.fileno())
        after = os.lstat(path)
    except OSError as exc:
        raise PlayableEvidenceError(f"cannot read completed evidence: {path}") from exc
    identities = {
        _stat_identity(before),
        _stat_identity(handle_before),
        _stat_identity(handle_after),
        _stat_identity(after),
    }
    if len(identities) != 1 or not payload:
        raise PlayableEvidenceError(f"completed evidence changed while reading: {path}")
    return payload, next(iter(identities))


def _stable_file_descriptor(path: Path) -> dict[str, Any]:
    payload = _read_stable_bytes(path)
    return {"sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}


def _absolute_regular_file(path: Path, label: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise PlayableEvidenceError(f"{label} path is not absolute")
    _reject_link_components(candidate)
    resolved = candidate.resolve(strict=True)
    if resolved != candidate or not resolved.is_file():
        raise PlayableEvidenceError(f"{label} path is not canonical/link-free")
    return resolved


def _safe_existing_directory(path: Path, label: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise PlayableEvidenceError(f"{label} is not absolute")
    _reject_link_components(candidate)
    resolved = candidate.resolve(strict=True)
    if resolved != candidate or not resolved.is_dir():
        raise PlayableEvidenceError(f"{label} is not a canonical directory")
    return resolved


def _reject_link_components(path: Path) -> None:
    absolute = path if path.is_absolute() else Path.cwd() / path
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            return
        except OSError as exc:
            raise PlayableEvidenceError(f"cannot inspect path component: {current}") from exc
        if current.is_symlink() or (
            getattr(metadata, "st_file_attributes", 0) & 0x400
        ):
            raise PlayableEvidenceError(f"path contains a link/reparse point: {current}")


def _stat_identity(value: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        int(value.st_dev),
        int(value.st_ino),
        int(value.st_size),
        int(value.st_mtime_ns),
        int(value.st_ctime_ns),
    )


def _write_json_new(path: Path, document: Mapping[str, Any]) -> None:
    _write_bytes_new(path, _canonical_json(document, indent=2) + b"\n")


def _write_jsonl_new(path: Path, documents: Sequence[Mapping[str, Any]]) -> None:
    if not documents:
        raise PlayableEvidenceError(f"refusing to write empty JSONL evidence: {path.name}")
    payload = b"".join(_canonical_json(item) + b"\n" for item in documents)
    _write_bytes_new(path, payload)


def _write_bytes_new(path: Path, payload: bytes) -> None:
    if not payload:
        raise PlayableEvidenceError(f"refusing to write empty evidence: {path.name}")
    _reject_link_components(path.parent)
    try:
        with path.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as exc:
        raise PlayableEvidenceError(f"cannot publish evidence atomically: {path}") from exc


def _canonical_json(value: object, *, indent: int | None = None) -> bytes:
    try:
        return json.dumps(
            value,
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":") if indent is None else None,
            indent=indent,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise PlayableEvidenceError("evidence contains non-finite/non-JSON data") from exc


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise PlayableEvidenceError(f"{label} must be an object")
    return value


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise PlayableEvidenceError(f"{label} must be non-empty trimmed text")
    return value


def _non_negative_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PlayableEvidenceError(f"{label} must be a non-negative integer")
    return value


def _positive_int(value: object, label: str) -> int:
    result = _non_negative_int(value, label)
    if result <= 0:
        raise PlayableEvidenceError(f"{label} must be positive")
    return result


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlayableEvidenceError(f"{label} must be finite numeric data")
    result = float(value)
    if not math.isfinite(result):
        raise PlayableEvidenceError(f"{label} must be finite numeric data")
    return result


def _finite_vector(value: object, length: int, label: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        raise PlayableEvidenceError(f"{label} must contain {length} values")
    return [_finite_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


__all__ = [
    "PLAYABLE_EVIDENCE_MANIFEST",
    "PinnedPlayableMediaToolchain",
    "PlayableEvidenceError",
    "PlayableEvidenceInputs",
    "finalize_playable_evidence",
    "read_stable_playable_jsonl",
    "snapshot_playable_media_toolchain",
]
