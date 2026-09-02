"""Pure, fail-closed qualification for the UE5 playable vertical slice.

This module never starts, stops, or inspects a live runtime.  It independently
joins artifacts written by UE, the Python control owner, MuJoCo truth, sensors,
recording, HUD capture, and post-close process ownership into one product
verdict.  Both PASS and rejection use the same schema; there is no weaker
component-qualification envelope that can be mistaken for this product gate.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import struct
import subprocess
import tempfile
import zlib
from collections.abc import Callable, Mapping, Sequence
from decimal import ROUND_HALF_UP, Decimal, InvalidOperation
from pathlib import Path
from typing import Any, TypeGuard, cast

PLAYABLE_QUALIFICATION_SCHEMA = "lingtu.sim.ue5-playable-vertical-slice.v1"
PLAYABLE_QUALIFICATION_FILENAME = "playable-qualification.json"

_MINIMUM_DURATION_NS = 20_000_000_000
_MINIMUM_TRANSLATION_M = 0.08
_MINIMUM_ROTATION_RAD = 0.35
_MAXIMUM_TURN_DRIFT_M = 0.10
_MAXIMUM_FRESH_AGE_NS = 100_000_000
_MINIMUM_RENDER_FRAMES = 600
# A half-open scheduler window can differ by one sample at an episode boundary.
_SAMPLE_BOUNDARY_TOLERANCE = 1
_MAXIMUM_MEDIA_DURATION_DELTA_NS = 100_000_000
_MAXIMUM_FRAME_TRUTH_BOUNDARY_GAP_NS = 33_333_334 + 16_000_000
_WIDTH = 1920
_HEIGHT = 1080
_MANEUVERS = ("forward", "backward", "left", "right", "turn_left", "turn_right")
_STREAM_RATES = {
    "thunder_01.front_depth": 30,
    "thunder_01.front_rgb": 30,
    "thunder_01.imu": 200,
    "thunder_01.mid360": 10,
    "thunder_01.truth_odom": 100,
}
_RENDER_STREAMS = frozenset({"thunder_01.front_depth", "thunder_01.front_rgb"})
_CHECK_NAMES = (
    "identity",
    "ue_control_origin",
    "control_acceptance",
    "truth_correlation",
    "maneuvers",
    "sensor_streams",
    "recording",
    "runtime_request_trace",
    "hud",
    "lifecycle",
    "artifact_integrity",
)
_SAFE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_DIGEST_RE = re.compile(r"[0-9a-f]{64}\Z")
_FRAME_RE = re.compile(r"frame_([0-9]{6})\.png\Z")
_THUNDERV4_CONTROLLER_MANIFEST_PATH = "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml"

_ALLOCATION_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "artifact_root",
        "boot_id",
        "dds_domain",
        "ports",
        "shm",
        "log_dir",
    }
)
_RUNTIME_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "model_generation",
        "reset_generation",
        "state",
        "bindings",
        "sensor_streams",
        "bundle_dir",
        "allocation",
        "clock",
    }
)
_RUNTIME_ALLOCATION_FIELDS = frozenset(
    {
        "run_dir",
        "log_dir",
        "boot_id",
        "physics_pid",
        "dds_domain",
        "ports",
        "shm",
        "shared_memory",
    }
)
_BINDING_FIELDS = frozenset(
    {
        "required",
        "state",
        "source_id",
        "failure_reason",
        "model_generation",
        "reset_generation",
    }
)
_ORIGIN_FIELDS = frozenset(
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
        "datagram_sha256",
        "datagram_bytes",
        "successful_send",
    }
)
_ACCEPTED_FIELDS = frozenset(
    {
        "schema",
        "event",
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
        "controller_id",
        "channel_id",
        "controller_sequence",
        "apply_time_ns",
        "submit_result",
        "admitted_twist",
    }
)
_ZERO_FIELDS = frozenset(
    {
        "schema",
        "event",
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
        "controller_sequence",
        "apply_time_ns",
        "submit_result",
        "admitted_twist",
        "reason",
    }
)
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
    _RUNTIME_REQUEST_TRACE_COMMON_FIELDS | frozenset({"event", "status", "reason"})
)
_RUNTIME_REQUEST_TRACE_JOIN_FIELDS = tuple(
    sorted(_RUNTIME_REQUEST_TRACE_COMMON_FIELDS - frozenset({"schema"}))
)
_RUNTIME_REQUEST_LIFECYCLE = (
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
_CORRELATION_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "maneuver",
        "start_event_id",
        "end_event_id",
        "start_source_epoch",
        "start_source_sequence",
        "end_source_epoch",
        "end_source_sequence",
        "start_datagram_sha256",
        "end_datagram_sha256",
        "start_controller_sequence",
        "end_controller_sequence",
        "start_apply_time_ns",
        "end_apply_time_ns",
        "accepted_event_count",
        "accepted_events_sha256",
        "truth_sequence_start",
        "truth_sequence_end",
        "truth_sim_time_ns_start",
        "truth_sim_time_ns_end",
        "truth_position_m_start",
        "truth_position_m_end",
        "truth_quaternion_wxyz_start",
        "truth_quaternion_wxyz_end",
        "frame_sequence_start",
        "frame_sequence_end",
    }
)
_TRAJECTORY_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "sequence",
        "sim_time_ns",
        "position_m",
        "quaternion_wxyz",
        "yaw_rad",
    }
)
_FRAME_CAPTURE_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "frame_sequence",
        "truth_sequence",
        "sim_time_ns",
        "path",
        "sha256",
        "bytes",
    }
)
_SEGMENT_DIGEST_FIELDS = (
    "event_id",
    "source_id",
    "source_epoch",
    "source_sequence",
    "datagram_sha256",
    "controller_sequence",
    "apply_time_ns",
)
_MEDIA_PROBE_FIELDS = frozenset(
    {
        "width",
        "height",
        "duration_ns",
        "frame_count",
        "decode_error_count",
    }
)
_HUD_SIDECAR_FIELDS = frozenset(
    {
        "schema",
        "state",
        "capture_id",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "captured_monotonic_ns",
        "status_age_ns",
        "control_status",
        "motion",
        "readiness",
        "sensors",
        "recording",
        "screenshot",
        "qualification_ready",
    }
)
_HUD_CONTROL_STATUS_FIELDS = frozenset(
    {
        "status",
        "event_id",
        "source_id",
        "source_epoch",
        "source_sequence",
        "server_status_sequence",
        "server_monotonic_ns",
        "received_monotonic_ns",
        "ui_mode",
        "camera_mode",
    }
)
_HUD_SCREENSHOT_FIELDS = frozenset({"basename", "bytes", "width", "height", "show_ui"})
_QUALIFIED_CAMERA_MODES = frozenset({"follow", "inspection", "free"})

MediaProbe = Callable[[Path], Mapping[str, Any]]


class PlayableQualificationError(ValueError):
    """Raised internally when completed evidence cannot qualify safely."""


class _PinnedFfmpegMediaProbe:
    def __init__(
        self,
        *,
        ffprobe_path: Path,
        ffmpeg_path: Path,
        ffprobe_version: str,
        ffmpeg_version: str,
    ) -> None:
        self.ffprobe_path = _absolute_file_path(ffprobe_path, "ffprobe_path")
        self.ffmpeg_path = _absolute_file_path(ffmpeg_path, "ffmpeg_path")
        versions = {
            "ffprobe": _nonempty_text(ffprobe_version, "ffprobe_version"),
            "ffmpeg": _nonempty_text(ffmpeg_version, "ffmpeg_version"),
        }
        self._expected = {
            name: {
                "path": path,
                **_external_file_descriptor(path),
                "version": versions[name],
            }
            for name, path in (
                ("ffprobe", self.ffprobe_path),
                ("ffmpeg", self.ffmpeg_path),
            )
        }

    def descriptor(self) -> dict[str, dict[str, Any]]:
        return {
            name: {**value, "path": str(value["path"])}
            for name, value in self._expected.items()
        }

    def require_toolchain(self, expected: Mapping[str, Mapping[str, Any]]) -> None:
        for name, path in (("ffprobe", self.ffprobe_path), ("ffmpeg", self.ffmpeg_path)):
            value = expected.get(name)
            if value is None or str(value.get("path", "")) != str(path):
                raise PlayableQualificationError(f"pinned {name} path does not match media-toolchain evidence")
            descriptor = _external_file_descriptor(path)
            pinned = self._expected.get(name)
            if pinned is None or descriptor["sha256"] != pinned["sha256"] or descriptor["bytes"] != pinned["bytes"]:
                raise PlayableQualificationError(f"pinned {name} binary changed after probe creation")
            if descriptor.get("sha256") != value.get("sha256") or descriptor.get("bytes") != value.get("bytes"):
                raise PlayableQualificationError(f"pinned {name} binary does not match media-toolchain evidence")
            if pinned.get("version") != value.get("version"):
                raise PlayableQualificationError(f"pinned {name} version does not match media-toolchain evidence")

    def __call__(self, path: Path) -> Mapping[str, Any]:
        self.require_toolchain(self._expected)
        result = _probe_video_with_ffmpeg(
            path,
            ffprobe_path=self.ffprobe_path,
            ffmpeg_path=self.ffmpeg_path,
        )
        self.require_toolchain(self._expected)
        return result


def create_pinned_ffmpeg_media_probe(
    *,
    ffprobe_path: Path,
    ffmpeg_path: Path,
    ffprobe_version: str,
    ffmpeg_version: str,
) -> tuple[_PinnedFfmpegMediaProbe, dict[str, dict[str, Any]]]:
    """Create a trusted probe and the immutable descriptor its caller must pin."""

    probe = _PinnedFfmpegMediaProbe(
        ffprobe_path=ffprobe_path,
        ffmpeg_path=ffmpeg_path,
        ffprobe_version=ffprobe_version,
        ffmpeg_version=ffmpeg_version,
    )
    return probe, probe.descriptor()


def build_playable_qualification(
    run_dir: Path,
    *,
    trusted_media_probe: object | None = None,
    trusted_media_toolchain: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return the verdict using only a caller-pinned production media toolchain."""

    return _build_playable_qualification(
        run_dir,
        media_probe=trusted_media_probe,
        trusted_media_toolchain=trusted_media_toolchain,
        allow_test_probe=False,
    )


def _build_playable_qualification_for_test(
    run_dir: Path,
    *,
    media_probe: MediaProbe,
    trusted_media_toolchain: Mapping[str, Any],
) -> dict[str, Any]:
    """Private deterministic seam for verifier tests; never a production API."""

    return _build_playable_qualification(
        run_dir,
        media_probe=media_probe,
        trusted_media_toolchain=trusted_media_toolchain,
        allow_test_probe=True,
    )


def _build_playable_qualification(
    run_dir: Path,
    *,
    media_probe: object | None,
    trusted_media_toolchain: Mapping[str, Any] | None,
    allow_test_probe: bool,
) -> dict[str, Any]:
    """Shared envelope builder with an explicitly private arbitrary-probe seam."""

    root = _safe_directory(Path(run_dir), "run_dir")
    verifier = _PlayableVerifier(root, None, None)
    try:
        if trusted_media_toolchain is None:
            raise PlayableQualificationError("trusted pinned media toolchain descriptor is required")
        trusted = _normalize_trusted_media_toolchain(trusted_media_toolchain, root)
        if allow_test_probe:
            if not callable(media_probe):
                raise PlayableQualificationError("private test media probe must be callable")
            selected_probe = cast(MediaProbe, media_probe)
            if type(media_probe) is _PinnedFfmpegMediaProbe:
                cast(_PinnedFfmpegMediaProbe, media_probe).require_toolchain(trusted)
        elif type(media_probe) is not _PinnedFfmpegMediaProbe:
            raise PlayableQualificationError("trusted pinned media probe is required")
        else:
            pinned_probe = cast(_PinnedFfmpegMediaProbe, media_probe)
            pinned_probe.require_toolchain(trusted)
            selected_probe = pinned_probe
        verifier._media_probe = selected_probe
        verifier._trusted_media_toolchain = trusted
        verifier.verify()
    # Evidence is an untrusted, completed artifact set.  A malformed value must
    # produce the one rejection envelope, never escape as a verifier exception.
    except Exception as exc:
        message = str(exc).strip() or "playable evidence is invalid"
        verifier.reasons.append(message)
    qualified = not verifier.reasons and all(verifier.checks.values())
    return {
        "schema": PLAYABLE_QUALIFICATION_SCHEMA,
        "result": "PASS" if qualified else "EVIDENCE_REJECTED",
        "qualified": qualified,
        "identity": dict(verifier.identity) if verifier.identity is not None else None,
        "checks": dict(verifier.checks),
        "maneuvers": list(verifier.maneuvers),
        "artifacts": dict(sorted(verifier.artifacts.items())),
        "reasons": list(dict.fromkeys(verifier.reasons)),
    }


def write_playable_qualification(
    run_dir: Path,
    *,
    trusted_media_probe: object | None = None,
    trusted_media_toolchain: Mapping[str, Any] | None = None,
) -> Path:
    """Build and atomically commit the sole playable verdict."""

    root = _safe_directory(Path(run_dir), "run_dir")
    destination = root / PLAYABLE_QUALIFICATION_FILENAME
    _reject_existing_link(destination)
    verdict = build_playable_qualification(
        root,
        trusted_media_probe=trusted_media_probe,
        trusted_media_toolchain=trusted_media_toolchain,
    )
    _atomic_write_json(destination, verdict)
    return destination


def _write_playable_qualification_for_test(
    run_dir: Path,
    *,
    media_probe: MediaProbe,
    trusted_media_toolchain: Mapping[str, Any],
) -> Path:
    """Private atomic-write seam used only by deterministic verifier tests."""

    root = _safe_directory(Path(run_dir), "run_dir")
    destination = root / PLAYABLE_QUALIFICATION_FILENAME
    _reject_existing_link(destination)
    verdict = _build_playable_qualification_for_test(
        root,
        media_probe=media_probe,
        trusted_media_toolchain=trusted_media_toolchain,
    )
    _atomic_write_json(destination, verdict)
    return destination


class _PlayableVerifier:
    def __init__(
        self,
        root: Path,
        media_probe: MediaProbe | None,
        trusted_media_toolchain: Mapping[str, Mapping[str, Any]] | None,
    ) -> None:
        self.root = root
        self._media_probe = media_probe
        self._trusted_media_toolchain = trusted_media_toolchain
        self.identity: dict[str, Any] | None = None
        self.checks = {name: False for name in _CHECK_NAMES}
        self.maneuvers: list[dict[str, Any]] = []
        self.artifacts: dict[str, dict[str, Any]] = {}
        self.reasons: list[str] = []
        self._runtime: dict[str, Any] | None = None
        self._episode: dict[str, Any] | None = None
        self._origins: dict[str, dict[str, Any]] = {}
        self._accepted: dict[str, dict[str, Any]] = {}
        self._final_zero: dict[str, Any] | None = None
        self._correlations: list[dict[str, Any]] = []
        self._trajectory: dict[int, dict[str, Any]] = {}
        self._frame_sequences: frozenset[int] = frozenset()
        self._frame_descriptors: dict[int, dict[str, Any]] = {}
        self._frame_capture_map: dict[int, dict[str, Any]] = {}
        self._decoded_pngs: dict[str, tuple[int, int, float]] = {}
        self._control_statuses: dict[int, dict[str, Any]] = {}

    def verify(self) -> None:
        self._verify_identity()
        self._verify_control_chain()
        self._verify_truth_and_maneuvers()
        self._verify_lifecycle()
        self._verify_sensors()
        self._verify_recording_and_hud()
        self._verify_artifact_stability()
        self.checks["artifact_integrity"] = True

    def _verify_identity(self) -> None:
        allocation = self._read_json("run-allocation.json")
        _exact_fields(allocation, _ALLOCATION_FIELDS, "run-allocation.json")
        _expect_schema(allocation, "lingtu.sim.run-allocation.v1", "run-allocation.json")
        run_id = _safe_id(allocation.get("run_id"), "run-allocation.json.run_id")
        session_id = _safe_id(
            allocation.get("session_id"),
            "run-allocation.json.session_id",
        )
        boot_id = _safe_id(allocation.get("boot_id"), "run-allocation.json.boot_id")
        _non_negative_int(allocation.get("dds_domain"), "run-allocation.json.dds_domain")
        ports = _mapping(allocation.get("ports"), "run-allocation.json.ports")
        required_ports = (
            "visual_snapshot_udp",
            "control_intent_udp",
            "control_status_udp",
        )
        if set(ports) != set(required_ports):
            raise PlayableQualificationError("playable run must allocate exactly the three production ports")
        port_values = []
        for name in required_ports:
            value = _positive_int(ports.get(name), f"run-allocation.json.ports.{name}")
            if value > 65535:
                raise PlayableQualificationError(f"run allocation port {name} is invalid")
            port_values.append(value)
        if len(set(port_values)) != len(port_values):
            raise PlayableQualificationError("playable run ports must be distinct")
        _mapping(allocation.get("shm"), "run-allocation.json.shm")
        log_dir = _absolute_path(allocation.get("log_dir"), "run-allocation.json.log_dir")
        if log_dir.resolve() != (self.root / "logs").resolve():
            raise PlayableQualificationError("run allocation log_dir is outside this run")
        artifact_root = _absolute_path(
            allocation.get("artifact_root"),
            "run-allocation.json.artifact_root",
        )
        if artifact_root != self.root:
            raise PlayableQualificationError("run-allocation artifact_root does not bind the current run_dir")

        runtime = self._read_json("session.runtime.json")
        _exact_fields(runtime, _RUNTIME_FIELDS, "session.runtime.json")
        _expect_schema(runtime, "lingtu.sim.session-runtime.v1", "session.runtime.json")
        model_generation = _non_negative_int(
            runtime.get("model_generation"),
            "session.runtime.json.model_generation",
        )
        reset_generation = _non_negative_int(
            runtime.get("reset_generation"),
            "session.runtime.json.reset_generation",
        )
        identity = {
            "run_id": run_id,
            "session_id": session_id,
            "boot_id": boot_id,
            "model_generation": model_generation,
            "reset_generation": reset_generation,
        }
        self.identity = identity
        self._verify_document_identity(runtime, "session.runtime.json", boot=False)
        if runtime.get("state") != "STOPPED":
            raise PlayableQualificationError("session runtime did not stop naturally")
        bindings = _mapping(runtime.get("bindings"), "session.runtime.json.bindings")
        if set(bindings) != {"physics", "control", "visual", "sensors"}:
            raise PlayableQualificationError("session runtime must contain exactly Physics/Control/Visual/Sensors")
        for name, raw in bindings.items():
            binding = _mapping(raw, f"session.runtime.json.bindings.{name}")
            _exact_fields(binding, _BINDING_FIELDS, f"binding {name}")
            if binding.get("required") is not True or binding.get("state") != "ACTIVE":
                raise PlayableQualificationError(f"required binding {name} was not ACTIVE")
            if binding.get("failure_reason") is not None:
                raise PlayableQualificationError(f"required binding {name} has a blocker")
            _safe_id(binding.get("source_id"), f"binding {name}.source_id")
            for field in ("model_generation", "reset_generation"):
                if binding.get(field) != identity[field]:
                    raise PlayableQualificationError(f"required binding {name} has stale {field}")
        runtime_allocation = _mapping(runtime.get("allocation"), "session.runtime.json.allocation")
        _exact_fields(
            runtime_allocation,
            _RUNTIME_ALLOCATION_FIELDS,
            "session.runtime.json.allocation",
        )
        if runtime_allocation.get("boot_id") != boot_id:
            raise PlayableQualificationError("session runtime boot_id mismatch")
        if runtime_allocation.get("ports") != ports:
            raise PlayableQualificationError("session runtime port allocation mismatch")
        if runtime_allocation.get("dds_domain") != allocation.get("dds_domain"):
            raise PlayableQualificationError("session runtime DDS allocation mismatch")
        if runtime_allocation.get("shm") != allocation.get("shm"):
            raise PlayableQualificationError("session runtime SHM allocation mismatch")
        _mapping(runtime_allocation.get("shared_memory"), "session.runtime.json.allocation.shared_memory")
        runtime_run_dir = _absolute_path(
            runtime_allocation.get("run_dir"),
            "session.runtime.json.allocation.run_dir",
        )
        if runtime_run_dir != self.root:
            raise PlayableQualificationError("session runtime run_dir mismatch")
        runtime_log_dir = _absolute_path(
            runtime_allocation.get("log_dir"),
            "session.runtime.json.allocation.log_dir",
        )
        if runtime_log_dir != log_dir:
            raise PlayableQualificationError("session runtime log_dir mismatch")
        _positive_int(runtime_allocation.get("physics_pid"), "physics_pid")
        _absolute_path(runtime.get("bundle_dir"), "session.runtime.json.bundle_dir")
        _mapping(runtime.get("sensor_streams"), "session.runtime.json.sensor_streams")
        _mapping(runtime.get("clock"), "session.runtime.json.clock")
        self._runtime = runtime
        self.checks["identity"] = True

    def _verify_control_chain(self) -> None:
        origins = self._read_jsonl("logs/ue-control-origin.jsonl")
        if not origins:
            raise PlayableQualificationError("UE successful-send origin evidence is empty")
        last_order: tuple[int, int] | None = None
        for index, item in enumerate(origins):
            label = f"ue-control-origin.jsonl line {index + 1}"
            _exact_fields(item, _ORIGIN_FIELDS, label)
            _expect_schema(item, "lingtu.sim.ue-control-origin.v1", label)
            self._verify_document_identity(item, label)
            order = self._event_order(item, label)
            if last_order is not None and order <= last_order:
                raise PlayableQualificationError("UE source sequence is not monotonic")
            last_order = order
            if item.get("successful_send") is not True:
                raise PlayableQualificationError("UE origin line was not a successful send")
            _digest(item.get("datagram_sha256"), f"{label}.datagram_sha256")
            datagram_bytes = _positive_int(item.get("datagram_bytes"), f"{label}.datagram_bytes")
            if datagram_bytes > 4096:
                raise PlayableQualificationError(f"{label}.datagram_bytes exceeds the UE wire limit")
            event_id = str(item["event_id"])
            if event_id in self._origins:
                raise PlayableQualificationError("UE origin contains duplicate event_id")
            self._origins[event_id] = item
        self.checks["ue_control_origin"] = True

        accepted = self._read_jsonl("control-intent-accepted.jsonl")
        if not accepted:
            raise PlayableQualificationError("control acceptance evidence is empty")
        last_accepted_order: tuple[int, int] | None = None
        last_controller_sequence = -1
        last_apply_time_ns = -1
        for index, item in enumerate(accepted):
            label = f"control-intent-accepted.jsonl line {index + 1}"
            schema = item.get("schema")
            is_final_zero = schema == "lingtu.sim.control-command-zero.v1"
            if is_final_zero:
                if index != len(accepted) - 1 or self._final_zero is not None:
                    raise PlayableQualificationError("final zero must be the unique last control trace")
                _exact_fields(item, _ZERO_FIELDS, label)
                if item.get("event") != "control_command_zero":
                    raise PlayableQualificationError("final zero trace event is invalid")
            else:
                _exact_fields(item, _ACCEPTED_FIELDS, label)
                _expect_schema(item, "lingtu.sim.control-intent-accepted.v1", label)
                if item.get("event") != "control_command_accepted":
                    raise PlayableQualificationError("accepted control trace event is invalid")
            self._verify_document_identity(item, label)
            order = self._event_order(item, label)
            if is_final_zero:
                if last_accepted_order is not None and order < last_accepted_order:
                    raise PlayableQualificationError("final zero is correlated to an older control event")
            else:
                if last_accepted_order is not None and order <= last_accepted_order:
                    raise PlayableQualificationError("accepted source sequence is not monotonic")
                last_accepted_order = order
            controller_sequence = _positive_int(item.get("controller_sequence"), f"{label}.controller_sequence")
            if controller_sequence <= last_controller_sequence:
                raise PlayableQualificationError("controller sequence is not monotonic")
            last_controller_sequence = controller_sequence
            apply_time_ns = _non_negative_int(item.get("apply_time_ns"), f"{label}.apply_time_ns")
            if apply_time_ns <= last_apply_time_ns:
                raise PlayableQualificationError("accepted command apply time is not monotonic")
            last_apply_time_ns = apply_time_ns
            _non_negative_int(item.get("source_monotonic_ns"), f"{label}.source_monotonic_ns")
            _non_negative_int(item.get("arrival_monotonic_ns"), f"{label}.arrival_monotonic_ns")
            if item.get("submit_result") != "accepted":
                raise PlayableQualificationError("playable command was not accepted")
            twist = _twist(item.get("admitted_twist"), f"{label}.admitted_twist")
            _validate_twist_limit(twist, label)
            event_id = str(item["event_id"])
            origin = self._origins.get(event_id)
            if origin is None:
                raise PlayableQualificationError("accepted control event lacks UE successful-send origin")
            for field in (
                "source_id",
                "source_epoch",
                "source_sequence",
                "datagram_sha256",
            ):
                if item.get(field) != origin.get(field):
                    raise PlayableQualificationError(f"UE origin to receiver hash/event join mismatch for {field}")
            if is_final_zero:
                if any(value != 0.0 for value in twist.values()):
                    raise PlayableQualificationError("final accepted command is not zero")
                reason = item.get("reason")
                if reason != "cleared:exit":
                    raise PlayableQualificationError("final zero trace is not the natural cleared:exit command")
                self._final_zero = item
                continue
            if item.get("controller_id") != "thunder_01.thunderv4_locomotion":
                raise PlayableQualificationError("accepted command does not target the production ThunderV4 controller")
            if item.get("channel_id") != "thunder_01.control.base_twist":
                raise PlayableQualificationError("accepted command does not target the production base_twist channel")
            if all(value == 0.0 for value in twist.values()):
                raise PlayableQualificationError("non-final accepted control trace must contain non-zero motion")
            if event_id in self._accepted:
                raise PlayableQualificationError("control acceptance has duplicate event_id")
            self._accepted[event_id] = item
        if not self._accepted:
            raise PlayableQualificationError("control acceptance contains no admitted motion")
        if self._final_zero is None:
            raise PlayableQualificationError("control acceptance lacks a correlated final zero")
        self.checks["control_acceptance"] = True

    def _verify_truth_and_maneuvers(self) -> None:
        trajectory = self._read_jsonl("motion-trajectory.jsonl")
        if not trajectory:
            raise PlayableQualificationError("MuJoCo truth trajectory is empty")
        previous_sequence = -1
        previous_time = -1
        for index, item in enumerate(trajectory):
            label = f"motion-trajectory.jsonl line {index + 1}"
            _exact_fields(item, _TRAJECTORY_FIELDS, label)
            _expect_schema(item, "lingtu.sim.motion-trajectory-sample.v1", label)
            self._verify_document_identity(item, label)
            sequence = _positive_int(item.get("sequence"), f"{label}.sequence")
            sim_time_ns = _non_negative_int(item.get("sim_time_ns"), f"{label}.sim_time_ns")
            if sequence <= previous_sequence or sim_time_ns <= previous_time:
                raise PlayableQualificationError("truth trajectory is not strictly monotonic")
            previous_sequence = sequence
            previous_time = sim_time_ns
            position = _finite_vector(item.get("position_m"), 3, f"{label}.position_m")
            item["position_m"] = position
            quaternion = _normalized_quaternion(
                item.get("quaternion_wxyz"),
                f"{label}.quaternion_wxyz",
            )
            item["quaternion_wxyz"] = quaternion
            yaw_rad = _finite_number(item.get("yaw_rad"), f"{label}.yaw_rad")
            if abs(_wrapped_angle_delta(yaw_rad, _quaternion_yaw(quaternion))) > 1e-6:
                raise PlayableQualificationError(f"{label} yaw_rad does not match quaternion_wxyz")
            item["yaw_rad"] = yaw_rad
            self._trajectory[sequence] = item

        correlations = self._read_jsonl("control-truth-correlation.jsonl")
        if len(correlations) != len(_MANEUVERS):
            raise PlayableQualificationError("truth correlation must contain exactly the six playable maneuvers")
        accepted_events = list(self._accepted.values())
        accepted_offset = 0
        previous_truth_end = -1
        previous_frame_end = -1
        for index, (item, expected_maneuver) in enumerate(zip(correlations, _MANEUVERS, strict=True)):
            label = f"control-truth-correlation.jsonl line {index + 1}"
            _exact_fields(item, _CORRELATION_FIELDS, label)
            _expect_schema(item, "lingtu.sim.control-truth-correlation.v2", label)
            self._verify_document_identity(item, label)
            if item.get("source_id") != "robotsimue.local_player.0":
                raise PlayableQualificationError("truth correlation source_id is not RobotSimUE")
            if item.get("maneuver") != expected_maneuver:
                raise PlayableQualificationError("truth correlation maneuver sequence is incomplete or reordered")
            accepted_count = _positive_int(
                item.get("accepted_event_count"),
                f"{label}.accepted_event_count",
            )
            segment = accepted_events[accepted_offset : accepted_offset + accepted_count]
            if len(segment) != accepted_count:
                raise PlayableQualificationError(
                    "six segments do not exactly partition every accepted non-zero event"
                )
            self._verify_accepted_segment(segment, item, expected_maneuver, label)
            accepted_offset += accepted_count

            start_sequence = _positive_int(item.get("truth_sequence_start"), f"{label}.truth_sequence_start")
            end_sequence = _positive_int(item.get("truth_sequence_end"), f"{label}.truth_sequence_end")
            if end_sequence <= start_sequence:
                raise PlayableQualificationError("truth interval is empty or reversed")
            if start_sequence <= previous_truth_end:
                raise PlayableQualificationError("truth correlation intervals overlap or are reordered")
            previous_truth_end = end_sequence
            samples = [
                self._trajectory[sequence]
                for sequence in range(start_sequence, end_sequence + 1)
                if sequence in self._trajectory
            ]
            if len(samples) != end_sequence - start_sequence + 1:
                raise PlayableQualificationError("truth interval has a missing sequence")
            if item.get("start_apply_time_ns") != samples[0]["sim_time_ns"]:
                raise PlayableQualificationError("accepted command apply_time does not start its truth interval")
            if int(item["end_apply_time_ns"]) > int(samples[-1]["sim_time_ns"]):
                raise PlayableQualificationError("accepted segment ends after its truth interval")
            self._verify_truth_segment_boundaries(item, samples, label)
            frame_start = _non_negative_int(item.get("frame_sequence_start"), f"{label}.frame_sequence_start")
            frame_end = _non_negative_int(item.get("frame_sequence_end"), f"{label}.frame_sequence_end")
            if frame_end < frame_start:
                raise PlayableQualificationError("mapped UE frame interval is reversed")
            if frame_start <= previous_frame_end:
                raise PlayableQualificationError("mapped UE frame intervals overlap or are reordered")
            previous_frame_end = frame_end
            truth_duration_ns = int(samples[-1]["sim_time_ns"]) - int(samples[0]["sim_time_ns"])
            minimum_segment_frames = max(
                1,
                truth_duration_ns * 30 // 1_000_000_000 - _SAMPLE_BOUNDARY_TOLERANCE,
            )
            if frame_end - frame_start + 1 < minimum_segment_frames:
                raise PlayableQualificationError("mapped UE frame interval is too short for the truth segment")
            metric = _maneuver_metric(
                expected_maneuver,
                str(item["start_event_id"]),
                samples,
                frame_start,
                frame_end,
            )
            metric["end_event_id"] = str(item["end_event_id"])
            metric["accepted_event_count"] = accepted_count
            metric["accepted_events_sha256"] = str(item["accepted_events_sha256"])
            self.maneuvers.append(metric)
        if accepted_offset != len(accepted_events):
            raise PlayableQualificationError(
                "six segments do not exactly partition every accepted non-zero event"
            )
        self._correlations = correlations
        self.checks["truth_correlation"] = True
        self.checks["maneuvers"] = True

    def _verify_accepted_segment(
        self,
        segment: Sequence[Mapping[str, Any]],
        correlation: Mapping[str, Any],
        maneuver: str,
        label: str,
    ) -> None:
        first = segment[0]
        last = segment[-1]
        boundary_fields = {
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
        }
        for field, expected in boundary_fields.items():
            if correlation.get(field) != expected:
                raise PlayableQualificationError(f"segment boundary mismatch for {field}")
        if first["source_epoch"] != last["source_epoch"]:
            raise PlayableQualificationError("accepted maneuver segment crosses a UE source epoch")
        previous_controller_sequence: int | None = None
        previous_apply_time_ns: int | None = None
        for event in segment:
            if event.get("source_epoch") != first["source_epoch"]:
                raise PlayableQualificationError("accepted maneuver segment crosses a UE source epoch")
            _validate_expected_maneuver_twist(
                maneuver,
                _twist(event.get("admitted_twist"), f"accepted {event['event_id']} admitted_twist"),
            )
            controller_sequence = int(event["controller_sequence"])
            apply_time_ns = int(event["apply_time_ns"])
            if previous_controller_sequence is not None and controller_sequence != previous_controller_sequence + 1:
                raise PlayableQualificationError("accepted maneuver controller sequence is not contiguous")
            if previous_apply_time_ns is not None and (
                apply_time_ns <= previous_apply_time_ns
                or apply_time_ns - previous_apply_time_ns > _MAXIMUM_FRESH_AGE_NS
            ):
                raise PlayableQualificationError("accepted maneuver cadence is stale or non-monotonic")
            previous_controller_sequence = controller_sequence
            previous_apply_time_ns = apply_time_ns
        expected_digest = _accepted_segment_sha256(segment)
        digest = _digest(
            correlation.get("accepted_events_sha256"),
            f"{label}.accepted_events_sha256",
        )
        if digest != expected_digest:
            raise PlayableQualificationError("accepted-event digest mismatch")

    @staticmethod
    def _verify_truth_segment_boundaries(
        correlation: Mapping[str, Any],
        samples: Sequence[Mapping[str, Any]],
        label: str,
    ) -> None:
        first = samples[0]
        last = samples[-1]
        scalar_boundaries = {
            "truth_sim_time_ns_start": first["sim_time_ns"],
            "truth_sim_time_ns_end": last["sim_time_ns"],
        }
        for field, expected in scalar_boundaries.items():
            value = _non_negative_int(correlation.get(field), f"{label}.{field}")
            if value != expected:
                raise PlayableQualificationError(f"truth segment boundary mismatch for {field}")
        vector_boundaries = {
            "truth_position_m_start": (first["position_m"], 3),
            "truth_position_m_end": (last["position_m"], 3),
            "truth_quaternion_wxyz_start": (first["quaternion_wxyz"], 4),
            "truth_quaternion_wxyz_end": (last["quaternion_wxyz"], 4),
        }
        for field, (expected, length) in vector_boundaries.items():
            vector_value = _finite_vector(correlation.get(field), length, f"{label}.{field}")
            if vector_value != expected:
                raise PlayableQualificationError(f"truth segment boundary mismatch for {field}")

    def _verify_lifecycle(self) -> None:
        episode = self._read_json("episode_result.json")
        _exact_fields(
            episode,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "model_generation",
                    "reset_generation",
                    "start_sim_time_ns",
                    "end_sim_time_ns",
                    "status",
                    "failure_reason",
                    "artifact_references",
                }
            ),
            "episode_result.json",
        )
        _expect_schema(episode, "lingtu.sim.episode-result.v1", "episode_result.json")
        self._verify_document_identity(episode, "episode_result.json", boot=False)
        start = _non_negative_int(episode.get("start_sim_time_ns"), "episode_result.start_sim_time_ns")
        end = _non_negative_int(episode.get("end_sim_time_ns"), "episode_result.end_sim_time_ns")
        if end - start < _MINIMUM_DURATION_NS:
            raise PlayableQualificationError("episode duration is shorter than 20 seconds")
        if episode.get("status") != "SUCCEEDED" or episode.get("failure_reason") is not None:
            raise PlayableQualificationError("episode did not SUCCEED")
        references = _mapping(episode.get("artifact_references"), "episode_result.artifact_references")
        expected_references = {
            "run_allocation": "run-allocation.json",
            "runtime_manifest": "session.runtime.json",
            "recording_manifest": "recording/recording.manifest.json",
            "shutdown_evidence": "shutdown-evidence.json",
        }
        _exact_fields(references, frozenset(expected_references), "episode_result.artifact_references")
        for name, expected in expected_references.items():
            if references.get(name) != expected:
                raise PlayableQualificationError(f"episode artifact reference {name} is missing or stale")
        self._episode = episode

        if self._runtime is None:
            raise PlayableQualificationError("session runtime identity is unavailable")
        clock = _mapping(self._runtime.get("clock"), "session.runtime.json.clock")
        _exact_fields(
            clock,
            frozenset({"sequence", "physics_step", "sim_time_ns"}),
            "session.runtime.json.clock",
        )
        _non_negative_int(clock.get("sequence"), "session.runtime.json.clock.sequence")
        _non_negative_int(clock.get("physics_step"), "session.runtime.json.clock.physics_step")
        if _non_negative_int(clock.get("sim_time_ns"), "session.runtime.json.clock.sim_time_ns") != end:
            raise PlayableQualificationError("session runtime clock is outside the completed episode")
        for accepted in self._accepted.values():
            apply_time = _non_negative_int(accepted.get("apply_time_ns"), "accepted command apply_time_ns")
            if not start <= apply_time <= end:
                raise PlayableQualificationError("accepted command timestamp is outside the episode")
        for sample in self._trajectory.values():
            sim_time = _non_negative_int(sample.get("sim_time_ns"), "truth sample sim_time_ns")
            if not start <= sim_time <= end:
                raise PlayableQualificationError("truth artifact timestamp is outside the episode")
        if self._final_zero is None:
            raise PlayableQualificationError("final zero evidence is unavailable")
        final_zero_time = _non_negative_int(
            self._final_zero.get("apply_time_ns"),
            "final zero apply_time_ns",
        )
        if final_zero_time != end:
            raise PlayableQualificationError("final zero timestamp is outside the episode completion")

        shutdown = self._read_json("shutdown-evidence.json")
        _exact_fields(
            shutdown,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "boot_id",
                    "model_generation",
                    "reset_generation",
                    "natural_shutdown",
                    "final_zero_event_id",
                    "resources_closed",
                    "owned_processes",
                    "scan_monotonic_ns",
                }
            ),
            "shutdown-evidence.json",
        )
        _expect_schema(
            shutdown,
            "lingtu.sim.playable-shutdown-evidence.v1",
            "shutdown-evidence.json",
        )
        self._verify_document_identity(shutdown, "shutdown-evidence.json")
        if shutdown.get("natural_shutdown") is not True:
            raise PlayableQualificationError("runtime shutdown was not natural")
        final_event_id = shutdown.get("final_zero_event_id")
        if final_event_id != self._final_zero.get("event_id"):
            raise PlayableQualificationError("shutdown final-zero event does not match the correlated zero trace")
        resources = _mapping(shutdown.get("resources_closed"), "shutdown-evidence.resources_closed")
        expected_resources = {
            "control_intent_udp",
            "control_status_udp",
            "sensors",
            "recording",
        }
        if set(resources) != expected_resources or not all(value is True for value in resources.values()):
            raise PlayableQualificationError("owned runtime resources were not all closed")
        processes = _sequence(shutdown.get("owned_processes"), "shutdown-evidence.owned_processes")
        if len(processes) != 2:
            raise PlayableQualificationError("shutdown evidence must name two owned processes")
        by_kind: dict[str, Mapping[str, Any]] = {}
        process_fields = frozenset(
            {
                "kind",
                "pid",
                "owned",
                "exit_code",
                "direct_child_running_after_close",
                "process_owner_closed",
                "termination_mode",
            }
        )
        for index, raw in enumerate(processes):
            process = _mapping(raw, f"shutdown-evidence.owned_processes[{index}]")
            _exact_fields(process, process_fields, f"owned process {index}")
            kind = process.get("kind")
            if kind not in {"robotsimue", "mujoco"} or kind in by_kind:
                raise PlayableQualificationError("shutdown process kind is missing or duplicated")
            _positive_int(process.get("pid"), f"owned process {kind}.pid")
            exit_code = _non_negative_int(process.get("exit_code"), f"owned process {kind}.exit_code")
            if (
                process.get("owned") is not True
                or exit_code != 0
                or process.get("direct_child_running_after_close") is not False
                or process.get("process_owner_closed") is not True
                or process.get("termination_mode") != "natural"
            ):
                raise PlayableQualificationError(f"owned process {kind} leaked or failed")
            by_kind[str(kind)] = process
        if set(by_kind) != {"robotsimue", "mujoco"}:
            raise PlayableQualificationError("shutdown process evidence is incomplete")
        if len({int(process["pid"]) for process in by_kind.values()}) != len(by_kind):
            raise PlayableQualificationError("shutdown process PIDs must be distinct")
        runtime_allocation = _mapping(self._runtime["allocation"], "session.runtime.json.allocation")
        if by_kind["mujoco"].get("pid") != runtime_allocation.get("physics_pid"):
            raise PlayableQualificationError("shutdown MuJoCo PID is not allocation-owned")
        _positive_int(shutdown.get("scan_monotonic_ns"), "shutdown scan_monotonic_ns")
        self.checks["lifecycle"] = True

    def _verify_sensors(self) -> None:
        if self._episode is None:
            raise PlayableQualificationError("episode identity is unavailable")
        summary = self._read_json("sensor-stream-summary.json")
        _exact_fields(
            summary,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "boot_id",
                    "model_generation",
                    "reset_generation",
                    "is_ready",
                    "required_stream_ids",
                    "blocking_reasons",
                    "streams",
                }
            ),
            "sensor-stream-summary.json",
        )
        _expect_schema(
            summary,
            "lingtu.sim.playable-sensor-stream-summary.v1",
            "sensor-stream-summary.json",
        )
        self._verify_document_identity(summary, "sensor-stream-summary.json")
        expected_ids = sorted(_STREAM_RATES)
        if summary.get("is_ready") is not True:
            raise PlayableQualificationError("sensor summary is not ready")
        blockers = _mapping(summary.get("blocking_reasons"), "sensor-stream-summary.blocking_reasons")
        if blockers:
            raise PlayableQualificationError("sensor summary contains blockers")
        required_ids = _sequence(
            summary.get("required_stream_ids"),
            "sensor-stream-summary.required_stream_ids",
        )
        if list(required_ids) != expected_ids:
            raise PlayableQualificationError("playable qualification requires the exact five sensor streams")
        streams = _sequence(summary.get("streams"), "sensor-stream-summary.streams")
        if len(streams) != len(expected_ids):
            raise PlayableQualificationError("sensor stream evidence is incomplete")
        episode_end = int(self._episode["end_sim_time_ns"])
        stream_fields = frozenset(
            {
                "stream_id",
                "state",
                "sample_count",
                "rate_hz",
                "clock_domain",
                "last_sample_truth_sequence",
                "last_sample_sim_time_ns",
                "sample_age_ns",
                "mapped_frame_count",
            }
        )
        seen: list[str] = []
        for index, raw in enumerate(streams):
            stream = _mapping(raw, f"sensor-stream-summary.streams[{index}]")
            _exact_fields(stream, stream_fields, f"sensor stream {index}")
            stream_id = stream.get("stream_id")
            if stream_id not in _STREAM_RATES or stream_id in seen:
                raise PlayableQualificationError("sensor stream is unknown or duplicated")
            seen.append(str(stream_id))
            if stream.get("state") != "ACTIVE":
                raise PlayableQualificationError(f"sensor {stream_id} is not ACTIVE")
            sample_count = _positive_int(stream.get("sample_count"), f"sensor {stream_id}.sample_count")
            if stream.get("rate_hz") != _STREAM_RATES[str(stream_id)]:
                raise PlayableQualificationError(f"sensor {stream_id} rate is stale")
            if stream.get("clock_domain") != "mujoco_sim_time":
                raise PlayableQualificationError(
                    f"sensor {stream_id} clock_domain must be mujoco_sim_time"
                )
            episode_start = int(self._episode["start_sim_time_ns"])
            episode_duration = episode_end - episode_start
            canonical_rate = _STREAM_RATES[str(stream_id)]
            scheduled_samples = episode_duration * canonical_rate // 1_000_000_000
            minimum_samples = max(1, scheduled_samples - _SAMPLE_BOUNDARY_TOLERANCE)
            if sample_count < minimum_samples:
                raise PlayableQualificationError(
                    f"sensor {stream_id} sample_count is below the canonical-rate minimum "
                    f"{minimum_samples} (one scheduling-boundary sample tolerance)"
                )
            last_sample = _non_negative_int(
                stream.get("last_sample_sim_time_ns"),
                f"sensor {stream_id}.last_sample_sim_time_ns",
            )
            _non_negative_int(
                stream.get("last_sample_truth_sequence"),
                f"sensor {stream_id}.last_sample_truth_sequence",
            )
            age = _non_negative_int(stream.get("sample_age_ns"), f"sensor {stream_id}.sample_age_ns")
            if episode_end - last_sample != age or age > _MAXIMUM_FRESH_AGE_NS:
                raise PlayableQualificationError(f"sensor {stream_id} evidence is stale")
            mapped = stream.get("mapped_frame_count")
            if stream_id in _RENDER_STREAMS:
                mapped_count = _non_negative_int(mapped, f"sensor {stream_id}.mapped_frame_count")
                if mapped_count < _MINIMUM_RENDER_FRAMES:
                    raise PlayableQualificationError(
                        f"30 Hz render stream {stream_id} has fewer than 600 mapped frames"
                    )
                if mapped_count > sample_count:
                    raise PlayableQualificationError(
                        f"30 Hz render stream {stream_id} maps more frames than it sampled"
                    )
            elif mapped is not None:
                raise PlayableQualificationError(f"non-render sensor {stream_id} must not claim mapped UE frames")
        if seen != expected_ids:
            raise PlayableQualificationError("sensor stream order/set is not canonical")
        self.checks["sensor_streams"] = True

    def _verify_recording_and_hud(self) -> None:
        if self._episode is None:
            raise PlayableQualificationError("episode identity is unavailable")
        manifest = self._read_json("recording/recording.manifest.json")
        _exact_fields(
            manifest,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "boot_id",
                    "model_generation",
                    "reset_generation",
                    "state",
                    "product",
                    "clock",
                    "timeline",
                    "media_toolchain",
                    "controller_calibration",
                    "control_status_authority",
                    "runtime_request_trace",
                    "control_zero_audit",
                    "frames",
                    "frame_capture_map",
                    "videos",
                    "hud",
                }
            ),
            "recording.manifest.json",
        )
        _expect_schema(
            manifest,
            "lingtu.sim.playable-recording-manifest.v1",
            "recording.manifest.json",
        )
        self._verify_document_identity(manifest, "recording.manifest.json")
        if manifest.get("state") != "COMMITTED":
            raise PlayableQualificationError("recording was not committed")
        product = _mapping(manifest.get("product"), "recording.manifest.product")
        expected_product = {
            "world_package": "factory_park_hf@1.2.1",
            "robot_package": "thunderv4@1.0.3",
            "robot_instance_id": "thunder_01",
            "robot_color": "black_graphite",
            "controller_package": "thunderv4_locomotion@1.0.0",
        }
        if dict(product) != expected_product:
            raise PlayableQualificationError(
                "recording product is not FactoryPark + black ThunderV4 + declared controller"
            )
        clock = _mapping(manifest.get("clock"), "recording.manifest.clock")
        _exact_fields(
            clock,
            frozenset({"authority", "start_sim_time_ns", "end_sim_time_ns", "duration_ns"}),
            "recording clock",
        )
        start = _non_negative_int(clock.get("start_sim_time_ns"), "recording start")
        end = _non_negative_int(clock.get("end_sim_time_ns"), "recording end")
        duration = _non_negative_int(clock.get("duration_ns"), "recording duration")
        episode_start = int(self._episode["start_sim_time_ns"])
        episode_end = int(self._episode["end_sim_time_ns"])
        if (
            clock.get("authority") != "mujoco"
            or end - start != duration
            or duration < _MINIMUM_DURATION_NS
            or not episode_start <= start < end <= episode_end
        ):
            raise PlayableQualificationError(
                "recording is not a >=20 second MuJoCo-clock interval contained by the episode"
            )
        for correlation in self._correlations:
            if not (
                start <= int(correlation["truth_sim_time_ns_start"])
                <= int(correlation["truth_sim_time_ns_end"])
                <= end
            ):
                raise PlayableQualificationError("maneuver truth interval is outside the recording")
        self._verify_recording_timeline(manifest.get("timeline"), start, end)
        self._verify_media_toolchain(manifest.get("media_toolchain"))
        self._verify_controller_calibration(manifest.get("controller_calibration"))
        self._verify_control_status_authority(manifest.get("control_status_authority"))
        self._verify_runtime_request_trace(manifest.get("runtime_request_trace"))
        self._verify_control_zero_audit(manifest.get("control_zero_audit"))
        self._verify_frames(manifest.get("frames"))
        self._verify_frame_capture_map(manifest.get("frame_capture_map"), start, end)
        self._verify_videos(manifest.get("videos"), duration)
        self.checks["recording"] = True
        self._verify_hud(manifest.get("hud"), start, end)
        self.checks["hud"] = True

    def _verify_media_toolchain(self, raw: object) -> None:
        trusted = self._trusted_media_toolchain
        if trusted is None:
            raise PlayableQualificationError("trusted pinned media toolchain descriptor is required")
        descriptor = _mapping(raw, "recording.manifest.media_toolchain")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "recording media toolchain descriptor",
        )
        if descriptor.get("path") != "recording/media-toolchain.json":
            raise PlayableQualificationError("recording media toolchain path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "recording media toolchain")
        document = _strict_json_object(payload, "recording/media-toolchain.json")
        _exact_fields(
            document,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "boot_id",
                    "model_generation",
                    "reset_generation",
                    "ffmpeg",
                    "ffprobe",
                }
            ),
            "recording/media-toolchain.json",
        )
        _expect_schema(
            document,
            "lingtu.sim.playable-media-toolchain.v1",
            "recording/media-toolchain.json",
        )
        self._verify_document_identity(document, "recording/media-toolchain.json")
        tool_fields = frozenset({"path", "sha256", "bytes", "version"})
        for name in ("ffmpeg", "ffprobe"):
            tool = _mapping(document.get(name), f"media toolchain {name}")
            _exact_fields(tool, tool_fields, f"media toolchain {name}")
            # Artifact bytes are untrusted.  They may echo the externally
            # pinned selection, but may never select a path for this process.
            if dict(tool) != dict(trusted[name]):
                raise PlayableQualificationError(
                    f"media toolchain {name} does not match the externally trusted descriptor"
                )

    def _verify_controller_calibration(self, raw: object) -> None:
        descriptor = _mapping(raw, "recording.manifest.controller_calibration")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "controller calibration descriptor",
        )
        if descriptor.get("path") != "recording/controller-calibration.json":
            raise PlayableQualificationError("controller calibration path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "controller calibration")
        document = _strict_json_object(payload, "recording/controller-calibration.json")
        _exact_fields(
            document,
            frozenset(
                {
                    "schema",
                    "run_id",
                    "session_id",
                    "boot_id",
                    "model_generation",
                    "reset_generation",
                    "controller_package_id",
                    "controller_package_version",
                    "controller_manifest_path",
                    "command_calibration",
                    "qualification_schedule",
                    "first_probe_not_prior_qualification",
                }
            ),
            "recording/controller-calibration.json",
        )
        _expect_schema(
            document,
            "lingtu.sim.playable-controller-calibration.v1",
            "recording/controller-calibration.json",
        )
        self._verify_document_identity(document, "recording/controller-calibration.json")
        if (
            document.get("controller_package_id") != "thunderv4_locomotion"
            or document.get("controller_package_version") != "1.0.0"
            or document.get("controller_manifest_path") != _THUNDERV4_CONTROLLER_MANIFEST_PATH
        ):
            raise PlayableQualificationError("controller calibration package provenance is stale")
        expected_calibration = {
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
        calibration = _mapping(
            document.get("command_calibration"),
            "controller calibration command_calibration",
        )
        if dict(calibration) != expected_calibration:
            raise PlayableQualificationError(
                "controller command_calibration is missing, changed, or makes a qualification claim"
            )
        expected_schedule = [
            {"maneuver": "turn_left", "key": "Q", "hold_s": 5.3},
            {"maneuver": "turn_right", "key": "E", "hold_s": 5.3},
        ]
        schedule = _sequence(
            document.get("qualification_schedule"),
            "controller calibration qualification_schedule",
        )
        if list(schedule) != expected_schedule:
            raise PlayableQualificationError("controller calibration Q/E 5.3 second schedule is stale")
        if document.get("first_probe_not_prior_qualification") is not True:
            raise PlayableQualificationError(
                "controller calibration must declare first_probe_not_prior_qualification"
            )

    def _verify_control_status_authority(self, raw: object) -> None:
        descriptor = _mapping(raw, "recording.manifest.control_status_authority")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "control-status authority descriptor",
        )
        if descriptor.get("path") != "recording/control-status-authority.jsonl":
            raise PlayableQualificationError("control-status authority path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "control-status authority")
        documents = _strict_jsonl(payload, "control-status authority")
        if not documents:
            raise PlayableQualificationError("control-status authority evidence is empty")
        from sim.runtime.coordinator.control_status import encode_control_status

        previous_server_sequence = 0
        previous_server_time = -1
        previous_sim_time = -1
        previous_source_order: tuple[int, int] | None = None
        for index, document in enumerate(documents):
            label = f"control-status authority line {index + 1}"
            try:
                encode_control_status(document)
            except Exception as exc:
                raise PlayableQualificationError(f"{label} is not an exact full status") from exc
            self._verify_document_identity(document, label)
            server_sequence = _positive_int(
                document.get("server_status_sequence"),
                f"{label}.server_status_sequence",
            )
            server_time = _non_negative_int(
                document.get("server_monotonic_ns"),
                f"{label}.server_monotonic_ns",
            )
            sim_time = _non_negative_int(document.get("sim_time_ns"), f"{label}.sim_time_ns")
            if (
                server_sequence <= previous_server_sequence
                or server_time <= previous_server_time
                or sim_time < previous_sim_time
            ):
                raise PlayableQualificationError("control-status authority is not monotonic")
            source_order = self._event_order(document, label)
            if previous_source_order is not None and source_order < previous_source_order:
                raise PlayableQualificationError("control-status source correlation moved backward")
            origin = self._origins.get(str(document["event_id"]))
            if origin is None:
                raise PlayableQualificationError("control-status authority lacks UE successful-send origin")
            for field in ("source_id", "source_epoch", "source_sequence"):
                if document.get(field) != origin.get(field):
                    raise PlayableQualificationError(f"control-status origin join mismatch for {field}")
            if document.get("intent_datagram_sha256") != origin.get("datagram_sha256"):
                raise PlayableQualificationError("control-status origin join mismatch for datagram SHA-256")
            if server_sequence in self._control_statuses:
                raise PlayableQualificationError("control-status authority duplicates server_status_sequence")
            self._control_statuses[server_sequence] = document
            previous_server_sequence = server_sequence
            previous_server_time = server_time
            previous_sim_time = sim_time
            previous_source_order = source_order

    def _verify_runtime_request_trace(self, raw: object) -> None:
        descriptor = _mapping(raw, "recording.manifest.runtime_request_trace")
        _exact_fields(
            descriptor,
            frozenset(
                {
                    "schema",
                    "content_schema",
                    "path",
                    "sha256",
                    "bytes",
                    "record_count",
                }
            ),
            "runtime request trace descriptor",
        )
        _expect_schema(
            descriptor,
            "lingtu.sim.runtime-request-trace-descriptor.v1",
            "runtime request trace descriptor",
        )
        if descriptor.get("content_schema") != "lingtu.sim.runtime-request-trace.v1":
            raise PlayableQualificationError(
                "runtime request trace descriptor content_schema is stale"
            )
        if descriptor.get("path") != "recording/runtime-request-trace.jsonl":
            raise PlayableQualificationError("runtime request trace path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "runtime request trace")
        documents = _strict_jsonl(payload, "runtime request trace")
        if not documents:
            raise PlayableQualificationError("runtime request trace evidence is empty")
        record_count = _positive_int(
            descriptor.get("record_count"),
            "runtime request trace descriptor record_count",
        )
        if record_count != len(documents):
            raise PlayableQualificationError(
                "runtime request trace record_count does not match the archived JSONL"
            )
        if record_count % 2 != 0:
            raise PlayableQualificationError(
                "runtime request trace is missing a contiguous received/terminal pair"
            )

        terminal_requests: list[str] = []
        seen_event_ids: set[str] = set()
        seen_datagram_hashes: set[str] = set()
        previous_order: tuple[int, int] | None = None
        previous_source_time = -1
        previous_arrival_time = -1
        for pair_offset in range(0, len(documents), 2):
            received = documents[pair_offset]
            terminal = documents[pair_offset + 1]
            received_label = f"runtime request trace line {pair_offset + 1}"
            terminal_label = f"runtime request trace line {pair_offset + 2}"
            if (
                received.get("event") != "runtime_request_received"
                or terminal.get("event") != "runtime_request_accepted"
            ):
                raise PlayableQualificationError(
                    "runtime request trace is not a contiguous received/terminal pair"
                )
            _exact_fields(
                received,
                _RUNTIME_REQUEST_TRACE_RECEIVED_FIELDS,
                received_label,
            )
            _exact_fields(
                terminal,
                _RUNTIME_REQUEST_TRACE_TERMINAL_FIELDS,
                terminal_label,
            )
            for item, label, expected_event in (
                (received, received_label, "runtime_request_received"),
                (terminal, terminal_label, "runtime_request_accepted"),
            ):
                _expect_schema(item, "lingtu.sim.runtime-request-trace.v1", label)
                self._verify_document_identity(item, label)
                if item.get("event") != expected_event:
                    raise PlayableQualificationError(
                        "runtime request trace is not a contiguous received/terminal pair"
                    )
                self._event_order(item, label)
                _digest(item.get("datagram_sha256"), f"{label}.datagram_sha256")
                _non_negative_int(
                    item.get("source_monotonic_ns"),
                    f"{label}.source_monotonic_ns",
                )
                _non_negative_int(
                    item.get("arrival_monotonic_ns"),
                    f"{label}.arrival_monotonic_ns",
                )
                self._verify_runtime_request_origin(item, label)

            if any(
                received.get(field) != terminal.get(field)
                for field in _RUNTIME_REQUEST_TRACE_JOIN_FIELDS
            ):
                raise PlayableQualificationError(
                    "runtime request received/terminal correlation was altered"
                )
            request = received.get("request")
            if not isinstance(request, str) or not request:
                raise PlayableQualificationError("runtime request trace request is invalid")
            status = terminal.get("status")
            reason = terminal.get("reason")
            if status == "accepted":
                if request not in _ALLOWED_ACCEPTED_RUNTIME_REQUESTS:
                    raise PlayableQualificationError(
                        f"unexpected accepted runtime request: {request}"
                    )
                if reason != "":
                    raise PlayableQualificationError(
                        "accepted runtime request contains a non-empty reason"
                    )
            elif status == "pending":
                if request not in _ALLOWED_PENDING_RUNTIME_REQUESTS:
                    raise PlayableQualificationError(
                        f"unexpected pending runtime request: {request}"
                    )
                if reason != f"safe_zero_pending:{request}":
                    raise PlayableQualificationError(
                        "pending runtime request reason is not the fixed safe-zero transition"
                    )
            else:
                raise PlayableQualificationError("runtime request trace status is invalid")

            event_id = str(received["event_id"])
            if event_id in seen_event_ids:
                raise PlayableQualificationError(
                    "runtime request trace duplicates a request event_id"
                )
            seen_event_ids.add(event_id)
            datagram_sha256 = str(received["datagram_sha256"])
            if datagram_sha256 in seen_datagram_hashes:
                raise PlayableQualificationError(
                    "runtime request trace duplicates a request datagram SHA-256"
                )
            seen_datagram_hashes.add(datagram_sha256)
            order = self._event_order(received, received_label)
            if previous_order is not None and order <= previous_order:
                raise PlayableQualificationError(
                    "runtime request source sequence is missing, duplicated, or reordered"
                )
            source_time = int(received["source_monotonic_ns"])
            arrival_time = int(received["arrival_monotonic_ns"])
            if source_time < previous_source_time or arrival_time < previous_arrival_time:
                raise PlayableQualificationError(
                    "runtime request monotonic time moved backward"
                )
            previous_order = order
            previous_source_time = source_time
            previous_arrival_time = arrival_time
            terminal_requests.append(request)

        self._verify_runtime_request_transitions(terminal_requests)
        self.checks["runtime_request_trace"] = True

    def _verify_runtime_request_origin(
        self,
        item: Mapping[str, Any],
        label: str,
    ) -> None:
        event_id = str(item["event_id"])
        origin = self._origins.get(event_id)
        if origin is None:
            raise PlayableQualificationError(
                f"{label} lacks UE successful-send origin"
            )
        for field in (
            "source_id",
            "source_epoch",
            "source_sequence",
            "datagram_sha256",
        ):
            if item.get(field) != origin.get(field):
                raise PlayableQualificationError(
                    f"runtime request origin join mismatch for {field}"
                )

    def _verify_runtime_request_transitions(self, requests: Sequence[str]) -> None:
        lifecycle = tuple(
            request
            for request in requests
            if request
            in {"record_start", "pause", "resume", "record_stop_commit", "exit"}
        )
        if lifecycle != _RUNTIME_REQUEST_LIFECYCLE:
            raise PlayableQualificationError(
                "runtime lifecycle must be record_start, pause, resume, "
                "record_stop_commit, pause, exit in exact order"
            )
        if requests[0] != "ui_state_update" or requests[-1] != "exit":
            raise PlayableQualificationError(
                "runtime request flow lacks its initial UI echo or terminal Menu exit"
            )

        record_start = requests.index("record_start")
        pause_one = requests.index("pause")
        resume = requests.index("resume")
        record_stop = requests.index("record_stop_commit")
        pause_two = requests.index("pause", pause_one + 1)
        exit_index = requests.index("exit")
        ui_updates = [
            index for index, request in enumerate(requests) if request == "ui_state_update"
        ]
        if len(ui_updates) < 7:
            raise PlayableQualificationError(
                "runtime request flow lacks real Drive/Tactical/Menu UI state updates"
            )
        if (
            not any(index < record_start for index in ui_updates)
            or sum(record_start < index < pause_one for index in ui_updates) < 3
            or requests[pause_one + 1 : resume] != ["ui_state_update"]
            or requests[resume + 1 : record_stop] != ["ui_state_update"]
            or requests[pause_two + 1 : exit_index] != ["ui_state_update"]
        ):
            raise PlayableQualificationError(
                "runtime UI state updates do not prove the fixed Drive/Tactical/Menu transitions"
            )

        claimed = False
        claim_count = 0
        matched_release_count = 0
        for index, request in enumerate(requests):
            if request == "control_claim":
                if claimed or not record_start < index < pause_one:
                    raise PlayableQualificationError(
                        "runtime control_claim transition is duplicated or outside Drive recording"
                    )
                claimed = True
                claim_count += 1
            elif request == "control_release":
                if claimed:
                    claimed = False
                    matched_release_count += 1
                elif index + 1 >= len(requests) or requests[index + 1] not in {
                    "ui_state_update",
                    "pause",
                }:
                    raise PlayableQualificationError(
                        "runtime control_release is neither paired nor a safe UI transition"
                    )
            elif request in {
                "pause",
                "resume",
                "record_stop_commit",
                "exit",
            } and claimed:
                raise PlayableQualificationError(
                    "runtime lifecycle advanced while UE still owned motion control"
                )
        if claimed or claim_count != len(_MANEUVERS) or matched_release_count != len(
            _MANEUVERS
        ):
            raise PlayableQualificationError(
                "runtime control_claim/control_release does not cover all six maneuvers"
            )

    def _verify_control_zero_audit(self, raw: object) -> None:
        descriptor = _mapping(raw, "recording.manifest.control_zero_audit")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "control zero audit descriptor",
        )
        if descriptor.get("path") != "recording/control-command-zero-audit.jsonl":
            raise PlayableQualificationError("control zero audit path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "control zero audit")
        documents = _strict_jsonl(payload, "control zero audit")
        if not documents:
            raise PlayableQualificationError("control zero audit evidence is empty")
        allowed_reasons = {
            "deadman_released",
            "runtime_request:control_release",
            "cleared:pause",
        }
        qualifying_reasons = {"deadman_released", "runtime_request:control_release"}
        seen_events: set[str] = set()
        seen_qualifying_reasons: set[str] = set()
        qualifying_zeros: list[dict[str, Any]] = []
        previous_controller_sequence = -1
        previous_apply_time = -1
        for index, item in enumerate(documents):
            label = f"control zero audit line {index + 1}"
            _exact_fields(item, _ZERO_FIELDS, label)
            _expect_schema(item, "lingtu.sim.control-command-zero.v1", label)
            self._verify_document_identity(item, label)
            if item.get("event") != "control_command_zero":
                raise PlayableQualificationError("control zero audit trace event is invalid")
            if item.get("submit_result") != "accepted":
                raise PlayableQualificationError("control zero audit command was not accepted")
            reason = item.get("reason")
            if reason == "cleared:exit":
                raise PlayableQualificationError("control zero audit contains a second cleared:exit zero")
            if reason not in allowed_reasons:
                raise PlayableQualificationError("control zero audit reason is not part of the fixed playable flow")
            twist = _twist(item.get("admitted_twist"), f"{label}.admitted_twist")
            if any(value != 0.0 for value in twist.values()):
                raise PlayableQualificationError("control zero audit command is not zero")
            controller_sequence = _positive_int(
                item.get("controller_sequence"),
                f"{label}.controller_sequence",
            )
            apply_time = _non_negative_int(item.get("apply_time_ns"), f"{label}.apply_time_ns")
            if controller_sequence <= previous_controller_sequence or apply_time <= previous_apply_time:
                raise PlayableQualificationError("control zero audit is not monotonic")
            previous_controller_sequence = controller_sequence
            previous_apply_time = apply_time
            event_id = str(item["event_id"])
            if event_id in seen_events:
                raise PlayableQualificationError("control zero audit duplicates event_id")
            seen_events.add(event_id)
            origin = self._origins.get(event_id)
            if origin is None:
                raise PlayableQualificationError("control zero audit lacks UE successful-send origin")
            for field in (
                "source_id",
                "source_epoch",
                "source_sequence",
                "datagram_sha256",
            ):
                if item.get(field) != origin.get(field):
                    raise PlayableQualificationError(f"control zero audit origin join mismatch for {field}")
            if reason in qualifying_reasons:
                qualifying_zeros.append(dict(item))
                seen_qualifying_reasons.add(str(reason))
        if len(qualifying_zeros) < len(_MANEUVERS) or seen_qualifying_reasons != qualifying_reasons:
            raise PlayableQualificationError(
                "control zero audit does not prove each released maneuver safety zero"
            )
        if self._final_zero is None:
            raise PlayableQualificationError("accepted control lacks the final exit zero")
        final_exit_time = _non_negative_int(
            self._final_zero.get("apply_time_ns"),
            "final exit zero apply_time_ns",
        )
        for index, correlation in enumerate(self._correlations):
            lower = _non_negative_int(
                correlation.get("end_apply_time_ns"),
                f"correlation {index + 1} end_apply_time_ns",
            )
            if index + 1 < len(self._correlations):
                upper = _non_negative_int(
                    self._correlations[index + 1].get("start_apply_time_ns"),
                    f"correlation {index + 2} start_apply_time_ns",
                )
            else:
                upper = final_exit_time
            if upper <= lower:
                raise PlayableQualificationError("maneuver release interval is reordered")
            if not any(
                lower < int(item["apply_time_ns"]) < upper
                for item in qualifying_zeros
            ):
                raise PlayableQualificationError(
                    "control zero audit does not prove each released maneuver safety zero"
                )

    def _verify_recording_timeline(self, raw: object, start_sim_time_ns: int, end_sim_time_ns: int) -> None:
        descriptor = _mapping(raw, "recording.manifest.timeline")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "recording timeline descriptor",
        )
        if descriptor.get("path") != "recording/recording.timeline.jsonl":
            raise PlayableQualificationError("recording timeline path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "recording timeline")
        events = _strict_jsonl(payload, "recording timeline")
        if len(events) != 2:
            raise PlayableQualificationError("recording timeline is partial")
        event_fields = frozenset(
            {
                "schema",
                "run_id",
                "session_id",
                "boot_id",
                "model_generation",
                "reset_generation",
                "sequence",
                "sim_time_ns",
                "event",
            }
        )
        expected = ((1, start_sim_time_ns, "record_start"), (2, end_sim_time_ns, "record_stop_commit"))
        for index, (event, expectation) in enumerate(zip(events, expected, strict=True)):
            _exact_fields(event, event_fields, f"recording timeline event {index}")
            _expect_schema(
                event,
                "lingtu.sim.playable-recording-event.v1",
                f"recording timeline event {index}",
            )
            self._verify_document_identity(event, f"recording timeline event {index}")
            if (event.get("sequence"), event.get("sim_time_ns"), event.get("event")) != expectation:
                raise PlayableQualificationError("recording timeline lifecycle is incomplete")

    def _verify_frames(self, raw: object) -> None:
        descriptor = _mapping(raw, "recording.manifest.frames")
        _exact_fields(
            descriptor,
            frozenset(
                {
                    "directory",
                    "count",
                    "first_sequence",
                    "last_sequence",
                    "width",
                    "height",
                }
            ),
            "recording frames descriptor",
        )
        if descriptor.get("directory") != "recording/frames":
            raise PlayableQualificationError("recording frame directory is stale")
        count = _non_negative_int(descriptor.get("count"), "recording frame count")
        first = _non_negative_int(descriptor.get("first_sequence"), "recording first frame")
        last = _non_negative_int(descriptor.get("last_sequence"), "recording last frame")
        if (
            count < _MINIMUM_RENDER_FRAMES
            or first != 0
            or last != count - 1
            or descriptor.get("width") != _WIDTH
            or descriptor.get("height") != _HEIGHT
        ):
            raise PlayableQualificationError("recording frame set is incomplete or not 1920x1080")
        frames_root = self._safe_subdirectory("recording/frames")
        expected_frame_names = {
            f"frame_{sequence:06d}.png" for sequence in range(first, last + 1)
        }
        actual_frame_names = set()
        for entry in frames_root.iterdir():
            if not entry.is_file() or _FRAME_RE.fullmatch(entry.name) is None:
                raise PlayableQualificationError("frame directory contains an unknown entry")
            actual_frame_names.add(entry.name)
        if actual_frame_names != expected_frame_names:
            raise PlayableQualificationError("recording frame sequence is missing or duplicated")
        sequences: list[int] = []
        for sequence in range(first, last + 1):
            entry = frames_root / f"frame_{sequence:06d}.png"
            if not entry.is_file():
                raise PlayableQualificationError("recording frame sequence is missing or duplicated")
            relative = f"recording/frames/{entry.name}"
            payload = self._read_bytes(relative)
            self._decode_png(payload, f"frame {sequence}")
            self._frame_descriptors[sequence] = {
                "path": relative,
                "sha256": hashlib.sha256(payload).hexdigest(),
                "bytes": len(payload),
            }
            sequences.append(sequence)
        if sequences != list(range(first, last + 1)):
            raise PlayableQualificationError("recording frame sequence is missing or duplicated")
        frame_set = frozenset(sequences)
        for correlation in self._correlations:
            expected = set(
                range(
                    int(correlation["frame_sequence_start"]),
                    int(correlation["frame_sequence_end"]) + 1,
                )
            )
            if not expected or not expected.issubset(frame_set):
                raise PlayableQualificationError("truth correlation maps to a missing UE frame")
        self._frame_sequences = frame_set

    def _verify_frame_capture_map(
        self,
        raw: object,
        recording_start_sim_time_ns: int,
        recording_end_sim_time_ns: int,
    ) -> None:
        descriptor = _mapping(raw, "recording.manifest.frame_capture_map")
        _exact_fields(
            descriptor,
            frozenset({"path", "sha256", "bytes"}),
            "frame capture map descriptor",
        )
        if descriptor.get("path") != "recording/frame-capture-map.jsonl":
            raise PlayableQualificationError("frame capture map path is stale")
        payload = self._read_bytes(str(descriptor["path"]))
        _verify_descriptor(payload, descriptor, "frame capture map")
        captures = _strict_jsonl(payload, "frame capture map")
        if len(captures) != len(self._frame_sequences):
            raise PlayableQualificationError("frame capture map does not cover every UE frame exactly once")
        previous_truth_sequence = -1
        previous_sim_time_ns = -1
        expected_sequences = sorted(self._frame_sequences)
        for index, (expected_frame_sequence, capture) in enumerate(
            zip(expected_sequences, captures, strict=True)
        ):
            label = f"frame capture map line {index + 1}"
            _exact_fields(capture, _FRAME_CAPTURE_FIELDS, label)
            _expect_schema(capture, "lingtu.sim.playable-frame-capture.v1", label)
            self._verify_document_identity(capture, label)
            frame_sequence = _non_negative_int(
                capture.get("frame_sequence"),
                f"{label}.frame_sequence",
            )
            if frame_sequence != expected_frame_sequence:
                raise PlayableQualificationError("frame capture map sequence is missing, duplicated, or reordered")
            expected_frame = self._frame_descriptors.get(frame_sequence)
            if expected_frame is None or any(
                capture.get(field) != expected_frame[field]
                for field in ("path", "sha256", "bytes")
            ):
                raise PlayableQualificationError("frame capture map does not match the captured PNG bytes")
            truth_sequence = _positive_int(
                capture.get("truth_sequence"),
                f"{label}.truth_sequence",
            )
            truth = self._trajectory.get(truth_sequence)
            sim_time_ns = _non_negative_int(capture.get("sim_time_ns"), f"{label}.sim_time_ns")
            if truth is None or sim_time_ns != truth["sim_time_ns"]:
                raise PlayableQualificationError("frame capture map truth sequence/time join is invalid")
            if not recording_start_sim_time_ns <= sim_time_ns <= recording_end_sim_time_ns:
                raise PlayableQualificationError("frame capture map frame is outside the recording interval")
            if truth_sequence < previous_truth_sequence or sim_time_ns < previous_sim_time_ns:
                raise PlayableQualificationError("frame capture map truth mapping moved backward")
            self._frame_capture_map[frame_sequence] = capture
            previous_truth_sequence = truth_sequence
            previous_sim_time_ns = sim_time_ns

        for correlation in self._correlations:
            frame_start = int(correlation["frame_sequence_start"])
            frame_end = int(correlation["frame_sequence_end"])
            mapped = [self._frame_capture_map[sequence] for sequence in range(frame_start, frame_end + 1)]
            truth_start = int(correlation["truth_sequence_start"])
            truth_end = int(correlation["truth_sequence_end"])
            time_start = int(correlation["truth_sim_time_ns_start"])
            time_end = int(correlation["truth_sim_time_ns_end"])
            if any(
                not truth_start <= int(item["truth_sequence"]) <= truth_end
                or not time_start <= int(item["sim_time_ns"]) <= time_end
                for item in mapped
            ):
                raise PlayableQualificationError("correlation frame range escapes its truth interval")
            eligible_sequences = [
                sequence
                for sequence, item in self._frame_capture_map.items()
                if truth_start <= int(item["truth_sequence"]) <= truth_end
                and time_start <= int(item["sim_time_ns"]) <= time_end
            ]
            if eligible_sequences != list(range(frame_start, frame_end + 1)):
                raise PlayableQualificationError(
                    "correlation frame range omits or reuses an available segment frame"
                )
            first_truth = int(mapped[0]["truth_sequence"])
            last_truth = int(mapped[-1]["truth_sequence"])
            previous = self._frame_capture_map.get(frame_start - 1)
            following = self._frame_capture_map.get(frame_end + 1)
            if (
                first_truth < truth_start
                or last_truth > truth_end
                or (previous is not None and int(previous["truth_sequence"]) >= truth_start)
                or (following is not None and int(following["truth_sequence"]) <= truth_end)
            ):
                raise PlayableQualificationError(
                    "correlation does not use the first/last available capture for its truth interval"
                )
            first_time = int(mapped[0]["sim_time_ns"])
            last_time = int(mapped[-1]["sim_time_ns"])
            if (
                first_time - time_start > _MAXIMUM_FRAME_TRUTH_BOUNDARY_GAP_NS
                or time_end - last_time > _MAXIMUM_FRAME_TRUTH_BOUNDARY_GAP_NS
            ):
                raise PlayableQualificationError(
                    "correlation frame boundary is more than one render period plus one physics step from truth"
                )

    def _verify_videos(self, raw: object, recording_duration_ns: int) -> None:
        media_probe = self._media_probe
        if media_probe is None:
            raise PlayableQualificationError("pinned ffprobe/FFmpeg media probe is unavailable")
        videos = _mapping(raw, "recording.manifest.videos")
        if set(videos) != {"raw", "labeled"}:
            raise PlayableQualificationError("raw and labeled videos are both required")
        expected_paths = {
            "raw": "videos/playable-raw.mp4",
            "labeled": "videos/playable-labeled.mp4",
        }
        fields = frozenset(
            {
                "path",
                "sha256",
                "bytes",
                "duration_ns",
                "width",
                "height",
                "decoded",
                "decode_error_count",
                "black_frame_count",
            }
        )
        for kind in ("raw", "labeled"):
            descriptor = _mapping(videos[kind], f"recording video {kind}")
            _exact_fields(descriptor, fields, f"recording video {kind}")
            if descriptor.get("path") != expected_paths[kind]:
                raise PlayableQualificationError(f"{kind} video path is stale")
            payload = self._read_bytes(str(descriptor["path"]))
            _verify_descriptor(payload, descriptor, f"{kind} video")
            if len(payload) < 12 or payload[4:8] != b"ftyp":
                raise PlayableQualificationError(f"{kind} video has no MP4 ftyp header")
            probe = _mapping(
                self._probe_media_snapshot(kind, payload, media_probe),
                f"independent {kind} media probe",
            )
            _exact_fields(probe, _MEDIA_PROBE_FIELDS, f"independent {kind} media probe")
            probe_width = _positive_int(probe.get("width"), f"independent {kind} width")
            probe_height = _positive_int(probe.get("height"), f"independent {kind} height")
            probe_duration = _positive_int(probe.get("duration_ns"), f"independent {kind} duration")
            probe_frames = _positive_int(probe.get("frame_count"), f"independent {kind} frame_count")
            probe_decode_errors = _non_negative_int(
                probe.get("decode_error_count"),
                f"independent {kind} decode_error_count",
            )
            self._read_bytes(str(descriptor["path"]))
            manifest_duration = _positive_int(
                descriptor.get("duration_ns"),
                f"{kind} video manifest duration",
            )
            manifest_decode_errors = _non_negative_int(
                descriptor.get("decode_error_count"),
                f"{kind} video manifest decode_error_count",
            )
            manifest_black_frames = _non_negative_int(
                descriptor.get("black_frame_count"),
                f"{kind} video manifest black_frame_count",
            )
            if (
                manifest_duration != recording_duration_ns
                or manifest_duration < _MINIMUM_DURATION_NS
                or descriptor.get("width") != _WIDTH
                or descriptor.get("height") != _HEIGHT
                or descriptor.get("decoded") is not True
                or manifest_decode_errors != 0
                or manifest_black_frames != 0
            ):
                raise PlayableQualificationError(f"{kind} video is short, undecodable, black, or not 1920x1080")
            expected_video_frames = len(self._frame_sequences)
            if (
                probe_width != _WIDTH
                or probe_height != _HEIGHT
                or probe_duration < _MINIMUM_DURATION_NS
                or abs(probe_duration - recording_duration_ns) > _MAXIMUM_MEDIA_DURATION_DELTA_NS
                or probe_frames != expected_video_frames
                or probe_decode_errors != 0
            ):
                raise PlayableQualificationError(
                    f"independent media probe rejected {kind} video geometry, duration, frames, or decode"
                )

    def _verify_hud(self, raw: object, start_sim_time_ns: int, end_sim_time_ns: int) -> None:
        hud = _mapping(raw, "recording.manifest.hud")
        expected_paths = {
            "drive": "screenshots/hud-drive.png",
            "tactical": "screenshots/hud-tactical.png",
            "menu_recording": "screenshots/hud-menu-recording.png",
        }
        if set(hud) != set(expected_paths):
            raise PlayableQualificationError("fresh Drive/Tactical/Menu-recording HUD evidence is required")
        fields = frozenset(
            {
                "path",
                "sha256",
                "bytes",
                "sidecar_path",
                "sidecar_sha256",
                "sidecar_bytes",
                "server_status_sequence",
                "camera_mode",
                "width",
                "height",
                "captured_sim_time_ns",
                "snapshot_age_ns",
                "snapshot_fresh",
                "motion_fields",
                "recording_state",
                "nonblack_pixel_fraction",
            }
        )
        expected_ui_modes = {
            "drive": "drive",
            "tactical": "tactical",
            "menu_recording": "menu",
        }
        camera_modes: list[str] = []
        capture_ids: set[str] = set()
        previous_status_sequence = 0
        for mode in ("drive", "tactical", "menu_recording"):
            descriptor = _mapping(hud[mode], f"HUD {mode}")
            _exact_fields(descriptor, fields, f"HUD {mode}")
            if descriptor.get("path") != expected_paths[mode]:
                raise PlayableQualificationError(f"HUD {mode} path is stale")
            payload = self._read_bytes(str(descriptor["path"]))
            _verify_descriptor(payload, descriptor, f"HUD {mode}")
            _width, _height, computed_nonblack = self._decode_png(payload, f"HUD {mode}")
            expected_sidecar_path = str(Path(expected_paths[mode]).with_suffix(".evidence.json")).replace("\\", "/")
            if descriptor.get("sidecar_path") != expected_sidecar_path:
                raise PlayableQualificationError(f"HUD {mode} sidecar path is stale")
            sidecar_payload = self._read_bytes(expected_sidecar_path)
            _verify_descriptor_fields(
                {"sha256": hashlib.sha256(sidecar_payload).hexdigest(), "bytes": len(sidecar_payload)},
                {
                    "sha256": descriptor.get("sidecar_sha256"),
                    "bytes": descriptor.get("sidecar_bytes"),
                },
                f"HUD {mode} sidecar",
            )
            sidecar = _strict_json_object(sidecar_payload, f"HUD {mode} sidecar")
            _exact_fields(sidecar, _HUD_SIDECAR_FIELDS, f"HUD {mode} sidecar")
            _expect_schema(
                sidecar,
                "lingtu.sim.ue-hud-screenshot-evidence.v1",
                f"HUD {mode} sidecar",
            )
            self._verify_document_identity(sidecar, f"HUD {mode} sidecar")
            if sidecar.get("state") != "CAPTURED" or sidecar.get("qualification_ready") is not True:
                raise PlayableQualificationError(f"HUD {mode} sidecar is not qualification-ready CAPTURED evidence")
            capture_id = _safe_id(sidecar.get("capture_id"), f"HUD {mode} sidecar.capture_id")
            if capture_id in capture_ids:
                raise PlayableQualificationError("HUD sidecars reuse a capture_id")
            capture_ids.add(capture_id)
            status_echo = _mapping(sidecar.get("control_status"), f"HUD {mode} sidecar.control_status")
            _exact_fields(status_echo, _HUD_CONTROL_STATUS_FIELDS, f"HUD {mode} sidecar.control_status")
            server_sequence = _positive_int(
                status_echo.get("server_status_sequence"),
                f"HUD {mode} server_status_sequence",
            )
            if server_sequence <= previous_status_sequence:
                raise PlayableQualificationError("HUD full-status echoes are not ordered")
            previous_status_sequence = server_sequence
            authority = self._control_statuses.get(server_sequence)
            if authority is None:
                raise PlayableQualificationError(f"HUD {mode} lacks its authoritative full-status record")
            authority_ui = _mapping(authority.get("ui"), f"HUD {mode} authority.ui")
            expected_echo = {
                "status": authority["status"],
                "event_id": authority["event_id"],
                "source_id": authority["source_id"],
                "source_epoch": authority["source_epoch"],
                "source_sequence": authority["source_sequence"],
                "server_status_sequence": authority["server_status_sequence"],
                "server_monotonic_ns": authority["server_monotonic_ns"],
                "ui_mode": authority_ui["ui_mode"],
                "camera_mode": authority_ui["camera_mode"],
            }
            for field, expected in expected_echo.items():
                if status_echo.get(field) != expected:
                    raise PlayableQualificationError(f"HUD {mode} full-status echo mismatch for {field}")
            if status_echo.get("status") not in {"accepted", "confirmed"}:
                raise PlayableQualificationError(f"HUD {mode} status was not admitted")
            received_monotonic_ns = _non_negative_int(
                status_echo.get("received_monotonic_ns"),
                f"HUD {mode} received_monotonic_ns",
            )
            captured_monotonic_ns = _non_negative_int(
                sidecar.get("captured_monotonic_ns"),
                f"HUD {mode} captured_monotonic_ns",
            )
            sidecar_age = _non_negative_int(sidecar.get("status_age_ns"), f"HUD {mode} status_age_ns")
            if (
                received_monotonic_ns < int(authority["server_monotonic_ns"])
                or captured_monotonic_ns - received_monotonic_ns != sidecar_age
                or sidecar_age > _MAXIMUM_FRESH_AGE_NS
            ):
                raise PlayableQualificationError(f"HUD {mode} full-status echo is stale")
            for field in ("motion", "readiness", "sensors", "recording"):
                if sidecar.get(field) != authority.get(field):
                    raise PlayableQualificationError(f"HUD {mode} full-status echo mismatch for {field}")
            authority_recording = _mapping(authority.get("recording"), f"HUD {mode} authority.recording")
            authority_sim_time_ns = _non_negative_int(
                authority.get("sim_time_ns"),
                f"HUD {mode} authority.sim_time_ns",
            )
            elapsed_sim_time_ns = _non_negative_int(
                authority_recording.get("elapsed_sim_time_ns"),
                f"HUD {mode} authority.recording.elapsed_sim_time_ns",
            )
            artifact_id = authority_recording.get("artifact_id")
            if (
                authority_recording.get("state") != "recording"
                or not isinstance(artifact_id, str)
                or not artifact_id.strip()
                or artifact_id != artifact_id.strip()
                or authority_recording.get("blocker") != ""
                or not start_sim_time_ns <= authority_sim_time_ns <= end_sim_time_ns
                or elapsed_sim_time_ns != authority_sim_time_ns - start_sim_time_ns
            ):
                raise PlayableQualificationError(
                    f"HUD {mode} authority does not prove an active recording interval/artifact"
                )
            screenshot = _mapping(sidecar.get("screenshot"), f"HUD {mode} sidecar.screenshot")
            _exact_fields(screenshot, _HUD_SCREENSHOT_FIELDS, f"HUD {mode} sidecar.screenshot")
            if screenshot != {
                "basename": Path(expected_paths[mode]).name,
                "bytes": len(payload),
                "width": _WIDTH,
                "height": _HEIGHT,
                "show_ui": True,
            }:
                raise PlayableQualificationError(f"HUD {mode} sidecar screenshot echo is stale")
            camera_mode = status_echo.get("camera_mode")
            if camera_mode not in _QUALIFIED_CAMERA_MODES:
                raise PlayableQualificationError(f"HUD {mode} camera_mode is unavailable or invalid")
            if status_echo.get("ui_mode") != expected_ui_modes[mode]:
                raise PlayableQualificationError(f"HUD {mode} ui_mode echo is invalid")
            if descriptor.get("server_status_sequence") != server_sequence or descriptor.get("camera_mode") != camera_mode:
                raise PlayableQualificationError(f"HUD {mode} manifest/status echo is stale")
            camera_modes.append(str(camera_mode))
            captured = _non_negative_int(descriptor.get("captured_sim_time_ns"), f"HUD {mode} capture time")
            age = _non_negative_int(descriptor.get("snapshot_age_ns"), f"HUD {mode} snapshot age")
            nonblack = _finite_number(
                descriptor.get("nonblack_pixel_fraction"),
                f"HUD {mode} nonblack pixel fraction",
            )
            if (
                not start_sim_time_ns <= captured <= end_sim_time_ns
                or captured != authority.get("sim_time_ns")
                or age > _MAXIMUM_FRESH_AGE_NS
                or age != sidecar_age
                or descriptor.get("width") != _WIDTH
                or descriptor.get("height") != _HEIGHT
                or descriptor.get("snapshot_fresh") is not True
                or descriptor.get("motion_fields") != ["requested", "admitted", "observed"]
                or descriptor.get("recording_state") != authority_recording.get("state")
                or not math.isclose(nonblack, computed_nonblack, rel_tol=0.0, abs_tol=1e-6)
                or not 0.01 <= computed_nonblack <= 1.0
            ):
                raise PlayableQualificationError(
                    f"HUD {mode} is stale, lacks three-state motion truth, or its computed nonblack pixel fraction "
                    "does not match"
                )
        if len(set(camera_modes)) < 2:
            raise PlayableQualificationError("HUD evidence does not prove a real camera mode transition")

    def _decode_png(self, payload: bytes, label: str) -> tuple[int, int, float]:
        digest = hashlib.sha256(payload).hexdigest()
        decoded = self._decoded_pngs.get(digest)
        if decoded is None:
            decoded = _strict_png_decode(payload, label, expected_width=_WIDTH, expected_height=_HEIGHT)
            self._decoded_pngs[digest] = decoded
        return decoded

    @staticmethod
    def _probe_media_snapshot(
        kind: str,
        payload: bytes,
        media_probe: MediaProbe,
    ) -> Mapping[str, Any]:
        """Probe an immutable verifier-owned copy, never the mutable evidence path."""

        descriptor, name = tempfile.mkstemp(prefix=f"lingtu-playable-{kind}-", suffix=".mp4")
        snapshot = Path(name)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            probe = media_probe(snapshot)
            try:
                after = snapshot.read_bytes()
            except OSError as exc:
                raise PlayableQualificationError(
                    f"{kind} media probe snapshot changed while decoding"
                ) from exc
            if after != payload:
                raise PlayableQualificationError(
                    f"{kind} media probe snapshot changed while decoding"
                )
            return probe
        finally:
            snapshot.unlink(missing_ok=True)

    def _verify_artifact_stability(self) -> None:
        """Re-read every input after all decoding to close the qualification window."""

        for relative in tuple(self.artifacts):
            self._read_bytes(relative)

    def _verify_document_identity(self, document: Mapping[str, Any], label: str, *, boot: bool = True) -> None:
        if self.identity is None:
            raise PlayableQualificationError("canonical run identity is unavailable")
        fields = ["run_id", "session_id", "model_generation", "reset_generation"]
        if boot:
            fields.insert(2, "boot_id")
        for field in fields:
            if document.get(field) != self.identity[field]:
                raise PlayableQualificationError(f"{label} {field} is stale or mismatched")

    def _event_order(self, item: Mapping[str, Any], label: str) -> tuple[int, int]:
        if self.identity is None:
            raise PlayableQualificationError("canonical run identity is unavailable")
        if item.get("source_id") != "robotsimue.local_player.0":
            raise PlayableQualificationError(f"{label} source_id is not RobotSimUE")
        epoch = _positive_int(item.get("source_epoch"), f"{label}.source_epoch")
        sequence = _positive_int(item.get("source_sequence"), f"{label}.source_sequence")
        expected = f"{self.identity['boot_id']}:{epoch}:{sequence}"
        if item.get("event_id") != expected:
            raise PlayableQualificationError(f"{label} event_id/sequence join is invalid")
        return epoch, sequence

    def _read_json(self, relative: str) -> dict[str, Any]:
        payload = self._read_bytes(relative)
        return _strict_json_object(payload, relative)

    def _read_jsonl(self, relative: str) -> list[dict[str, Any]]:
        payload = self._read_bytes(relative)
        return _strict_jsonl(payload, relative)

    def _read_bytes(self, relative: str) -> bytes:
        path = self._resolve_file(relative)
        _reject_link_components(path)
        try:
            path_before = os.lstat(path)
            with path.open("rb") as stream:
                handle_before = os.fstat(stream.fileno())
                payload = stream.read()
                handle_after = os.fstat(stream.fileno())
            path_after = os.lstat(path)
        except OSError as exc:
            raise PlayableQualificationError(f"cannot read required artifact: {relative}") from exc
        _reject_link_components(path)
        if not payload:
            raise PlayableQualificationError(f"required artifact is empty: {relative}")
        identities = {
            _stable_stat_identity(path_before),
            _stable_stat_identity(handle_before),
            _stable_stat_identity(handle_after),
            _stable_stat_identity(path_after),
        }
        if len(identities) != 1 or handle_before.st_size != len(payload):
            raise PlayableQualificationError(f"artifact changed while qualifying: {relative}")
        descriptor = {"sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}
        existing = self.artifacts.get(relative)
        if existing is not None and existing != descriptor:
            raise PlayableQualificationError(f"artifact changed between reads: {relative}")
        self.artifacts[relative] = descriptor
        return payload

    def _resolve_file(self, relative: str) -> Path:
        relative_path = _safe_relative_path(relative)
        candidate = self.root / relative_path
        _reject_link_components(candidate)
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as exc:
            raise PlayableQualificationError(f"required artifact is missing: {relative}") from exc
        if self.root != resolved and self.root not in resolved.parents:
            raise PlayableQualificationError(f"artifact escapes run_dir: {relative}")
        if not resolved.is_file():
            raise PlayableQualificationError(f"required artifact is not a file: {relative}")
        return candidate

    def _safe_subdirectory(self, relative: str) -> Path:
        relative_path = _safe_relative_path(relative)
        candidate = self.root / relative_path
        _reject_link_components(candidate)
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as exc:
            raise PlayableQualificationError(f"required directory is missing: {relative}") from exc
        if self.root not in resolved.parents or not resolved.is_dir():
            raise PlayableQualificationError(f"required directory is unsafe: {relative}")
        return resolved


def _maneuver_metric(
    name: str,
    event_id: str,
    samples: Sequence[Mapping[str, Any]],
    frame_start: int,
    frame_end: int,
) -> dict[str, Any]:
    start = samples[0]
    end = samples[-1]
    start_position = start["position_m"]
    end_position = end["position_m"]
    if not isinstance(start_position, list) or not isinstance(end_position, list):
        raise PlayableQualificationError("maneuver truth positions are unavailable")
    dx = float(end_position[0]) - float(start_position[0])
    dy = float(end_position[1]) - float(start_position[1])
    start_yaw = float(start["yaw_rad"])
    body_forward = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
    body_left = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
    yaw_delta = 0.0
    previous_yaw = start_yaw
    for sample in samples[1:]:
        current_yaw = float(sample["yaw_rad"])
        delta = (current_yaw - previous_yaw + math.pi) % (2.0 * math.pi) - math.pi
        yaw_delta += delta
        previous_yaw = current_yaw
    signed_translation = {
        "forward": body_forward,
        "backward": -body_forward,
        "left": body_left,
        "right": -body_left,
    }.get(name, 0.0)
    signed_yaw = yaw_delta if name == "turn_left" else -yaw_delta if name == "turn_right" else 0.0
    drift = 0.0
    if name.startswith("turn_"):
        drift = max(
            math.hypot(
                float(sample["position_m"][0]) - float(start_position[0]),
                float(sample["position_m"][1]) - float(start_position[1]),
            )
            for sample in samples
        )
    if name in {"forward", "backward", "left", "right"}:
        if signed_translation < _MINIMUM_TRANSLATION_M:
            raise PlayableQualificationError(
                f"maneuver {name} translated less than 0.08 m in its signed body direction"
            )
    elif signed_yaw < _MINIMUM_ROTATION_RAD:
        raise PlayableQualificationError(f"maneuver {name} rotated less than 0.35 rad in its command direction")
    elif drift > _MAXIMUM_TURN_DRIFT_M:
        raise PlayableQualificationError(f"maneuver {name} drifted more than 0.10 m while turning")
    return {
        "name": name,
        "event_id": event_id,
        "truth_sequence_start": int(start["sequence"]),
        "truth_sequence_end": int(end["sequence"]),
        "mapped_frame_count": frame_end - frame_start + 1,
        "signed_translation_m": signed_translation,
        "signed_yaw_rad": signed_yaw,
        "horizontal_drift_m": drift,
    }


def _twist(value: object, label: str) -> dict[str, float]:
    mapping = _mapping(value, label)
    _exact_fields(
        mapping,
        frozenset({"linear_x", "linear_y", "angular_z"}),
        label,
    )
    return {
        name: _finite_number(mapping.get(name), f"{label}.{name}") for name in ("linear_x", "linear_y", "angular_z")
    }


def _validate_twist_limit(twist: Mapping[str, float], label: str) -> None:
    if math.hypot(twist["linear_x"], twist["linear_y"]) > 0.100000001:
        raise PlayableQualificationError(f"{label} exceeds server translation limit")
    if abs(twist["angular_z"]) > 0.350000001:
        raise PlayableQualificationError(f"{label} exceeds server yaw limit")


def _validate_expected_maneuver_twist(name: str, twist: Mapping[str, float]) -> None:
    expected_axis = {
        "forward": ("linear_x", 1.0),
        "backward": ("linear_x", -1.0),
        "left": ("linear_y", 1.0),
        "right": ("linear_y", -1.0),
        "turn_left": ("angular_z", 1.0),
        "turn_right": ("angular_z", -1.0),
    }[name]
    axis, sign = expected_axis
    if sign * twist[axis] <= 0.0:
        raise PlayableQualificationError(f"accepted command is not aligned with maneuver {name}")
    if any(abs(value) > 1e-12 for field, value in twist.items() if field != axis):
        raise PlayableQualificationError(f"accepted command for maneuver {name} mixes motion axes")


def _accepted_segment_sha256(events: Sequence[Mapping[str, Any]]) -> str:
    canonical = [
        {field: event[field] for field in _SEGMENT_DIGEST_FIELDS}
        for event in events
    ]
    payload = json.dumps(
        canonical,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _normalized_quaternion(value: object, label: str) -> list[float]:
    quaternion = _finite_vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise PlayableQualificationError(f"{label} must be normalized")
    return quaternion


def _quaternion_yaw(quaternion_wxyz: Sequence[float]) -> float:
    w, x, y, z = quaternion_wxyz
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _wrapped_angle_delta(left: float, right: float) -> float:
    return (left - right + math.pi) % (2.0 * math.pi) - math.pi


def _strict_json_object(payload: bytes, label: str) -> dict[str, Any]:
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise PlayableQualificationError(f"{label} is not UTF-8") from exc

    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise PlayableQualificationError(f"{label} contains duplicate key {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise PlayableQualificationError(f"{label} contains non-finite value {value}")

    try:
        document = json.loads(
            text,
            object_pairs_hook=object_from_pairs,
            parse_constant=reject_constant,
        )
    except PlayableQualificationError:
        raise
    except (json.JSONDecodeError, TypeError) as exc:
        raise PlayableQualificationError(f"{label} is not strict JSON") from exc
    if type(document) is not dict:
        raise PlayableQualificationError(f"{label} must be a JSON object")
    return document


def _strict_jsonl(payload: bytes, label: str) -> list[dict[str, Any]]:
    if not payload.endswith(b"\n"):
        raise PlayableQualificationError(f"{label} is a partial JSONL file")
    lines = payload.splitlines()
    if any(not line for line in lines):
        raise PlayableQualificationError(f"{label} contains an empty JSONL record")
    return [_strict_json_object(line, f"{label} line {index + 1}") for index, line in enumerate(lines)]


def _probe_video_with_ffmpeg(
    path: Path,
    *,
    ffprobe_path: Path,
    ffmpeg_path: Path,
) -> Mapping[str, Any]:
    """Probe metadata with ffprobe and require a separate full FFmpeg decode."""

    probe_command = [
        str(ffprobe_path),
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
    ]
    try:
        completed = subprocess.run(  # noqa: S603 - argv only; executable is hash-pinned.
            probe_command,
            check=False,
            capture_output=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PlayableQualificationError("independent ffprobe execution failed") from exc
    if completed.returncode != 0:
        raise PlayableQualificationError("independent ffprobe rejected the MP4 container")
    document = _strict_json_object(completed.stdout, "ffprobe output")
    streams = _sequence(document.get("streams"), "ffprobe streams")
    if len(streams) != 1:
        raise PlayableQualificationError("ffprobe did not find exactly one primary video stream")
    stream = _mapping(streams[0], "ffprobe primary video stream")
    width = _positive_int(stream.get("width"), "ffprobe width")
    height = _positive_int(stream.get("height"), "ffprobe height")
    raw_frames = stream.get("nb_read_frames")
    if raw_frames is None or raw_frames == "N/A":
        raw_frames = stream.get("nb_frames")
    frame_count = _positive_decimal_integer(raw_frames, "ffprobe decoded frame count")
    raw_duration = stream.get("duration")
    if raw_duration in {None, "N/A"}:
        raw_duration = _mapping(document.get("format"), "ffprobe format").get("duration")
    duration_ns = _seconds_as_nanoseconds(raw_duration, "ffprobe duration")

    decode_command = [
        str(ffmpeg_path),
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
    ]
    try:
        decoded = subprocess.run(  # noqa: S603 - argv only; executable is hash-pinned.
            decode_command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PlayableQualificationError("independent FFmpeg full decode failed") from exc
    if decoded.returncode != 0:
        raise PlayableQualificationError("independent FFmpeg full decode reported an error")
    return {
        "width": width,
        "height": height,
        "duration_ns": duration_ns,
        "frame_count": frame_count,
        "decode_error_count": 0,
    }


def _positive_decimal_integer(value: object, label: str) -> int:
    if isinstance(value, bool):
        raise PlayableQualificationError(f"{label} must be positive")
    try:
        result = int(str(value), 10)
    except (TypeError, ValueError) as exc:
        raise PlayableQualificationError(f"{label} must be positive") from exc
    if result <= 0 or str(result) != str(value):
        raise PlayableQualificationError(f"{label} must be a canonical positive integer")
    return result


def _seconds_as_nanoseconds(value: object, label: str) -> int:
    if isinstance(value, bool):
        raise PlayableQualificationError(f"{label} must be positive finite seconds")
    try:
        seconds = Decimal(str(value))
    except (InvalidOperation, ValueError) as exc:
        raise PlayableQualificationError(f"{label} must be positive finite seconds") from exc
    if not seconds.is_finite() or seconds <= 0:
        raise PlayableQualificationError(f"{label} must be positive finite seconds")
    nanoseconds = (seconds * Decimal(1_000_000_000)).to_integral_value(rounding=ROUND_HALF_UP)
    return int(nanoseconds)


def _verify_descriptor(payload: bytes, descriptor: Mapping[str, Any], label: str) -> None:
    if descriptor.get("sha256") != hashlib.sha256(payload).hexdigest():
        raise PlayableQualificationError(f"{label} SHA-256 mismatch")
    if descriptor.get("bytes") != len(payload):
        raise PlayableQualificationError(f"{label} byte count mismatch")


def _verify_descriptor_fields(
    actual: Mapping[str, Any],
    expected: Mapping[str, Any],
    label: str,
) -> None:
    if actual.get("sha256") != expected.get("sha256"):
        raise PlayableQualificationError(f"{label} SHA-256 mismatch")
    if actual.get("bytes") != expected.get("bytes"):
        raise PlayableQualificationError(f"{label} byte count mismatch")


def _external_file_descriptor(path: Path) -> dict[str, Any]:
    _reject_link_components(path)
    try:
        path_before = os.lstat(path)
        with path.open("rb") as stream:
            handle_before = os.fstat(stream.fileno())
            hasher = hashlib.sha256()
            while chunk := stream.read(1024 * 1024):
                hasher.update(chunk)
            digest = hasher.hexdigest()
            handle_after = os.fstat(stream.fileno())
        path_after = os.lstat(path)
    except OSError as exc:
        raise PlayableQualificationError(f"cannot read pinned media tool: {path}") from exc
    _reject_link_components(path)
    identities = {
        _stable_stat_identity(path_before),
        _stable_stat_identity(handle_before),
        _stable_stat_identity(handle_after),
        _stable_stat_identity(path_after),
    }
    if len(identities) != 1 or handle_before.st_size <= 0:
        raise PlayableQualificationError(f"pinned media tool changed while hashing: {path}")
    return {"sha256": digest, "bytes": int(handle_before.st_size)}


def _normalize_trusted_media_toolchain(
    raw: Mapping[str, Any],
    run_root: Path,
) -> dict[str, dict[str, Any]]:
    """Validate caller-owned tool selection without consulting run artifacts."""

    mapping = _mapping(raw, "trusted media toolchain")
    _exact_fields(mapping, frozenset({"ffmpeg", "ffprobe"}), "trusted media toolchain")
    normalized: dict[str, dict[str, Any]] = {}
    fields = frozenset({"path", "sha256", "bytes", "version"})
    for name in ("ffmpeg", "ffprobe"):
        descriptor = _mapping(mapping.get(name), f"trusted media toolchain {name}")
        _exact_fields(descriptor, fields, f"trusted media toolchain {name}")
        path = _absolute_file_path(descriptor.get("path"), f"trusted media toolchain {name}.path")
        if path == run_root or run_root in path.parents:
            raise PlayableQualificationError(
                f"trusted media toolchain {name} must be outside run_dir"
            )
        actual = _external_file_descriptor(path)
        _verify_descriptor_fields(actual, descriptor, f"trusted media toolchain {name}")
        version = _nonempty_text(
            descriptor.get("version"),
            f"trusted media toolchain {name}.version",
        )
        normalized[name] = {
            "path": str(path),
            **actual,
            "version": version,
        }
    return normalized


class _PngDecodeError(ValueError):
    pass


_PNG_CHANNELS = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}
_PNG_BIT_DEPTHS = {
    0: frozenset({1, 2, 4, 8, 16}),
    2: frozenset({8, 16}),
    3: frozenset({1, 2, 4, 8}),
    4: frozenset({8, 16}),
    6: frozenset({8, 16}),
}
_MAXIMUM_PNG_BYTES = 64 * 1024 * 1024


def _strict_png_decode(
    payload: bytes,
    label: str,
    *,
    expected_width: int,
    expected_height: int,
) -> tuple[int, int, float]:
    try:
        return _decode_png_payload(
            payload,
            expected_width=expected_width,
            expected_height=expected_height,
        )
    except (_PngDecodeError, struct.error, zlib.error, OverflowError) as exc:
        raise PlayableQualificationError(f"{label} strict PNG decode failed: {exc}") from exc


def _decode_png_payload(
    payload: bytes,
    *,
    expected_width: int,
    expected_height: int,
) -> tuple[int, int, float]:
    if len(payload) > _MAXIMUM_PNG_BYTES:
        raise _PngDecodeError("file exceeds the bounded PNG size")
    if payload[:8] != b"\x89PNG\r\n\x1a\n":
        raise _PngDecodeError("signature is invalid")

    offset = 8
    chunk_index = 0
    ihdr: tuple[int, int, int, int] | None = None
    palette_payload: bytes | None = None
    transparency_payload: bytes | None = None
    idat_payloads: list[bytes] = []
    saw_idat = False
    closed_idat = False
    saw_iend = False
    while offset < len(payload):
        if len(payload) - offset < 12:
            raise _PngDecodeError("chunk is truncated")
        chunk_length = struct.unpack(">I", payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        chunk_end = offset + 12 + chunk_length
        if chunk_end > len(payload):
            raise _PngDecodeError("chunk length escapes the file")
        if len(chunk_type) != 4 or any(not (65 <= byte <= 90 or 97 <= byte <= 122) for byte in chunk_type):
            raise _PngDecodeError("chunk type is invalid")
        if chunk_type[2] & 0x20:
            raise _PngDecodeError("chunk reserved bit is set")
        chunk_payload = payload[offset + 8 : offset + 8 + chunk_length]
        recorded_crc = struct.unpack(">I", payload[offset + 8 + chunk_length : chunk_end])[0]
        computed_crc = zlib.crc32(chunk_type + chunk_payload) & 0xFFFFFFFF
        if recorded_crc != computed_crc:
            raise _PngDecodeError(f"{chunk_type.decode('ascii')} CRC mismatch")
        if chunk_index == 0 and chunk_type != b"IHDR":
            raise _PngDecodeError("IHDR is not the first chunk")
        if saw_idat and chunk_type != b"IDAT":
            closed_idat = True

        if chunk_type == b"IHDR":
            if chunk_index != 0 or ihdr is not None or chunk_length != 13:
                raise _PngDecodeError("IHDR is duplicated or malformed")
            width, height, bit_depth, color_type, compression, filter_method, interlace = struct.unpack(
                ">IIBBBBB", chunk_payload
            )
            if width != expected_width or height != expected_height:
                raise _PngDecodeError("pixel geometry is not the required 1920x1080")
            if color_type not in _PNG_CHANNELS or bit_depth not in _PNG_BIT_DEPTHS[color_type]:
                raise _PngDecodeError("IHDR color type/bit depth is unsupported")
            if compression != 0 or filter_method != 0 or interlace != 0:
                raise _PngDecodeError("IHDR compression/filter/interlace method is unsupported")
            ihdr = (width, height, bit_depth, color_type)
        elif chunk_type == b"PLTE":
            if ihdr is None or saw_idat or palette_payload is not None:
                raise _PngDecodeError("PLTE ordering is invalid")
            if not chunk_payload or len(chunk_payload) % 3 or len(chunk_payload) > 768:
                raise _PngDecodeError("PLTE length is invalid")
            palette_payload = chunk_payload
        elif chunk_type == b"tRNS":
            if ihdr is None or saw_idat or transparency_payload is not None:
                raise _PngDecodeError("tRNS ordering is invalid")
            if ihdr[3] == 3 and palette_payload is None:
                raise _PngDecodeError("indexed tRNS precedes PLTE")
            transparency_payload = chunk_payload
        elif chunk_type == b"IDAT":
            if ihdr is None or closed_idat:
                raise _PngDecodeError("IDAT ordering is invalid")
            saw_idat = True
            idat_payloads.append(chunk_payload)
        elif chunk_type == b"IEND":
            if not saw_idat or chunk_length != 0:
                raise _PngDecodeError("IEND is premature or malformed")
            saw_iend = True
        elif chunk_type[0] & 0x20 == 0:
            raise _PngDecodeError(f"unknown critical chunk {chunk_type.decode('ascii')}")

        offset = chunk_end
        chunk_index += 1
        if saw_iend:
            if offset != len(payload):
                raise _PngDecodeError("bytes trail IEND")
            break

    if ihdr is None or not idat_payloads or not saw_iend:
        raise _PngDecodeError("IHDR, IDAT, or IEND is missing")
    width, height, bit_depth, color_type = ihdr
    palette, palette_alpha, transparent_gray, transparent_rgb = _validate_png_palette_and_transparency(
        bit_depth,
        color_type,
        palette_payload,
        transparency_payload,
    )
    channels = _PNG_CHANNELS[color_type]
    bits_per_pixel = channels * bit_depth
    row_bytes = (width * bits_per_pixel + 7) // 8
    expected_decoded_bytes = height * (row_bytes + 1)
    compressed = b"".join(idat_payloads)
    decoder = zlib.decompressobj()
    decoded = decoder.decompress(compressed, expected_decoded_bytes + 1)
    if decoder.unconsumed_tail or len(decoded) > expected_decoded_bytes:
        raise _PngDecodeError("decompressed pixels exceed IHDR geometry")
    decoded += decoder.flush()
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail or len(decoded) != expected_decoded_bytes:
        raise _PngDecodeError("decompressed pixel size does not match IHDR geometry")
    rows = _unfilter_png_rows(
        decoded,
        height=height,
        row_bytes=row_bytes,
        filter_bytes_per_pixel=max(1, (bits_per_pixel + 7) // 8),
    )
    nonblack = _count_nonblack_png_pixels(
        rows,
        width=width,
        bit_depth=bit_depth,
        color_type=color_type,
        palette=palette,
        palette_alpha=palette_alpha,
        transparent_gray=transparent_gray,
        transparent_rgb=transparent_rgb,
    )
    return width, height, nonblack / (width * height)


def _validate_png_palette_and_transparency(
    bit_depth: int,
    color_type: int,
    palette_payload: bytes | None,
    transparency_payload: bytes | None,
) -> tuple[
    tuple[tuple[int, int, int], ...],
    tuple[int, ...],
    int | None,
    tuple[int, int, int] | None,
]:
    palette: tuple[tuple[int, int, int], ...] = ()
    if palette_payload is not None:
        palette = tuple(
            (palette_payload[index], palette_payload[index + 1], palette_payload[index + 2])
            for index in range(0, len(palette_payload), 3)
        )
    if color_type == 3:
        if not palette or len(palette) > 1 << bit_depth:
            raise _PngDecodeError("indexed PNG palette is missing or too large")
    elif color_type in {0, 4} and palette:
        raise _PngDecodeError("grayscale PNG must not contain PLTE")

    palette_alpha: tuple[int, ...] = ()
    transparent_gray: int | None = None
    transparent_rgb: tuple[int, int, int] | None = None
    if transparency_payload is None:
        return palette, palette_alpha, transparent_gray, transparent_rgb
    if color_type == 0:
        if len(transparency_payload) != 2:
            raise _PngDecodeError("grayscale tRNS length is invalid")
        transparent_gray = struct.unpack(">H", transparency_payload)[0]
        if transparent_gray >= 1 << bit_depth:
            raise _PngDecodeError("grayscale tRNS sample exceeds bit depth")
    elif color_type == 2:
        if len(transparency_payload) != 6:
            raise _PngDecodeError("truecolor tRNS length is invalid")
        transparent_rgb = struct.unpack(">HHH", transparency_payload)
        if any(sample >= 1 << bit_depth for sample in transparent_rgb):
            raise _PngDecodeError("truecolor tRNS sample exceeds bit depth")
    elif color_type == 3:
        if not transparency_payload or len(transparency_payload) > len(palette):
            raise _PngDecodeError("indexed tRNS length is invalid")
        palette_alpha = tuple(transparency_payload)
    else:
        raise _PngDecodeError("alpha PNG must not contain tRNS")
    return palette, palette_alpha, transparent_gray, transparent_rgb


def _unfilter_png_rows(
    decoded: bytes,
    *,
    height: int,
    row_bytes: int,
    filter_bytes_per_pixel: int,
) -> list[bytes]:
    rows: list[bytes] = []
    previous = bytearray(row_bytes)
    offset = 0
    for _row_index in range(height):
        filter_type = decoded[offset]
        source = decoded[offset + 1 : offset + 1 + row_bytes]
        offset += row_bytes + 1
        reconstructed = bytearray(row_bytes)
        for index, value in enumerate(source):
            left = reconstructed[index - filter_bytes_per_pixel] if index >= filter_bytes_per_pixel else 0
            above = previous[index]
            upper_left = previous[index - filter_bytes_per_pixel] if index >= filter_bytes_per_pixel else 0
            if filter_type == 0:
                predictor = 0
            elif filter_type == 1:
                predictor = left
            elif filter_type == 2:
                predictor = above
            elif filter_type == 3:
                predictor = (left + above) // 2
            elif filter_type == 4:
                predictor = _paeth_predictor(left, above, upper_left)
            else:
                raise _PngDecodeError("scanline filter type is invalid")
            reconstructed[index] = (value + predictor) & 0xFF
        rows.append(bytes(reconstructed))
        previous = reconstructed
    return rows


def _paeth_predictor(left: int, above: int, upper_left: int) -> int:
    estimate = left + above - upper_left
    left_distance = abs(estimate - left)
    above_distance = abs(estimate - above)
    upper_left_distance = abs(estimate - upper_left)
    if left_distance <= above_distance and left_distance <= upper_left_distance:
        return left
    if above_distance <= upper_left_distance:
        return above
    return upper_left


def _png_sample(row: bytes, sample_index: int, bit_depth: int) -> int:
    if bit_depth == 8:
        return row[sample_index]
    if bit_depth == 16:
        offset = sample_index * 2
        return (row[offset] << 8) | row[offset + 1]
    samples_per_byte = 8 // bit_depth
    byte = row[sample_index // samples_per_byte]
    shift = 8 - bit_depth * ((sample_index % samples_per_byte) + 1)
    return (byte >> shift) & ((1 << bit_depth) - 1)


def _count_nonblack_png_pixels(
    rows: Sequence[bytes],
    *,
    width: int,
    bit_depth: int,
    color_type: int,
    palette: Sequence[tuple[int, int, int]],
    palette_alpha: Sequence[int],
    transparent_gray: int | None,
    transparent_rgb: tuple[int, int, int] | None,
) -> int:
    if color_type == 0 and bit_depth == 1:
        if transparent_gray == 1:
            return 0
        full_bytes, remaining_bits = divmod(width, 8)
        total = 0
        for row in rows:
            total += sum(byte.bit_count() for byte in row[:full_bytes])
            if remaining_bits:
                total += (row[full_bytes] >> (8 - remaining_bits)).bit_count()
        return total
    channels = _PNG_CHANNELS[color_type]
    maximum_alpha = (1 << bit_depth) - 1
    nonblack = 0
    for row in rows:
        for column in range(width):
            base = column * channels
            samples = tuple(_png_sample(row, base + index, bit_depth) for index in range(channels))
            if color_type == 0:
                colors = samples
                alpha = 0 if transparent_gray is not None and samples[0] == transparent_gray else maximum_alpha
            elif color_type == 2:
                colors = samples
                alpha = 0 if transparent_rgb is not None and samples == transparent_rgb else maximum_alpha
            elif color_type == 3:
                palette_index = samples[0]
                if palette_index >= len(palette):
                    raise _PngDecodeError("pixel references a missing palette entry")
                colors = palette[palette_index]
                alpha = palette_alpha[palette_index] if palette_index < len(palette_alpha) else 255
            elif color_type == 4:
                colors = samples[:1]
                alpha = samples[1]
            else:
                colors = samples[:3]
                alpha = samples[3]
            if alpha > 0 and any(component > 0 for component in colors):
                nonblack += 1
    return nonblack


def _safe_directory(path: Path, label: str) -> Path:
    absolute = path if path.is_absolute() else Path.cwd() / path
    _reject_link_components(absolute)
    if not absolute.is_dir():
        raise PlayableQualificationError(f"{label} must be an existing directory")
    return absolute.resolve(strict=True)


def _safe_relative_path(value: str) -> Path:
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise PlayableQualificationError(f"artifact path is unsafe: {value!r}")
    path = Path(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise PlayableQualificationError(f"artifact path is unsafe: {value!r}")
    return path


def _reject_link_components(path: Path) -> None:
    absolute = path if path.is_absolute() else Path.cwd() / path
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        try:
            stat_result = os.lstat(current)
        except FileNotFoundError:
            return
        except OSError as exc:
            raise PlayableQualificationError(f"cannot inspect path component: {current}") from exc
        if current.is_symlink() or bool(getattr(stat_result, "st_file_attributes", 0) & 0x400):
            raise PlayableQualificationError(f"path component is symlink or reparse point: {current}")


def _reject_existing_link(path: Path) -> None:
    if not path.exists() and not path.is_symlink():
        return
    stat_result = os.lstat(path)
    if path.is_symlink() or bool(getattr(stat_result, "st_file_attributes", 0) & 0x400):
        raise PlayableQualificationError("playable verdict destination is a link")


def _stable_stat_identity(value: os.stat_result) -> tuple[int, int, int]:
    return (int(value.st_dev), int(value.st_ino), int(value.st_size))


def _absolute_path(value: object, label: str) -> Path:
    if not isinstance(value, str) or not value or value != value.strip():
        raise PlayableQualificationError(f"{label} must be absolute text")
    path = Path(value)
    if not path.is_absolute():
        raise PlayableQualificationError(f"{label} must be absolute")
    _reject_link_components(path)
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise PlayableQualificationError(f"{label} must exist") from exc
    if path != resolved:
        raise PlayableQualificationError(f"{label} must be canonical and link-free")
    return resolved


def _absolute_file_path(value: object, label: str) -> Path:
    if isinstance(value, Path):
        value = str(value)
    path = _absolute_path(value, label)
    if not path.is_file():
        raise PlayableQualificationError(f"{label} must be a file")
    return path


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise PlayableQualificationError(f"{label} must be an object")
    return value


def _sequence(value: object, label: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes, bytearray, Mapping)) or not isinstance(value, Sequence):
        raise PlayableQualificationError(f"{label} must be an array")
    return value


def _exact_fields(value: Mapping[str, Any], expected: frozenset[str], label: str) -> None:
    missing = sorted(expected - set(value))
    unknown = sorted(set(value) - expected)
    if missing:
        raise PlayableQualificationError(f"{label} is missing required field(s): {', '.join(missing)}")
    if unknown:
        raise PlayableQualificationError(f"{label} has unknown field(s): {', '.join(unknown)}")


def _expect_schema(document: Mapping[str, Any], schema: str, label: str) -> None:
    if document.get("schema") != schema:
        raise PlayableQualificationError(f"{label} schema is invalid")


def _safe_id(value: object, label: str) -> str:
    if not isinstance(value, str) or _SAFE_ID_RE.fullmatch(value) is None:
        raise PlayableQualificationError(f"{label} is not a safe identifier")
    return value


def _nonempty_text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise PlayableQualificationError(f"{label} must be non-empty trimmed text")
    return value


def _digest(value: object, label: str) -> str:
    if not isinstance(value, str) or _DIGEST_RE.fullmatch(value) is None:
        raise PlayableQualificationError(f"{label} is not a lowercase SHA-256 digest")
    return value


def _non_negative_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PlayableQualificationError(f"{label} must be a non-negative integer")
    return value


def _positive_int(value: object, label: str) -> int:
    result = _non_negative_int(value, label)
    if result == 0:
        raise PlayableQualificationError(f"{label} must be positive")
    return result


def _finite_number(value: object, label: str) -> float:
    if not _is_finite_number(value):
        raise PlayableQualificationError(f"{label} must be finite")
    return float(value)


def _is_finite_number(value: object) -> TypeGuard[int | float]:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite_vector(value: object, length: int, label: str) -> list[float]:
    sequence = _sequence(value, label)
    if len(sequence) != length:
        raise PlayableQualificationError(f"{label} must contain {length} values")
    return [_finite_number(item, f"{label}[{index}]") for index, item in enumerate(sequence)]


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    payload = (
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor, name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        from sim.runtime.coordinator.atomic_file import replace_file_with_retry

        replace_file_with_retry(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


__all__ = [
    "PLAYABLE_QUALIFICATION_FILENAME",
    "PLAYABLE_QUALIFICATION_SCHEMA",
    "PlayableQualificationError",
    "build_playable_qualification",
    "create_pinned_ffmpeg_media_probe",
    "write_playable_qualification",
]
