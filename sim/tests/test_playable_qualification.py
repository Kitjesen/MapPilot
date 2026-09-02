# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import math
import os
import struct
import subprocess
import zlib
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable

import pytest

from sim.runtime.qualification.playable import (
    PLAYABLE_QUALIFICATION_FILENAME,
    PLAYABLE_QUALIFICATION_SCHEMA,
    PlayableQualificationError,
    _build_playable_qualification_for_test,
    _write_playable_qualification_for_test,
    build_playable_qualification,
    create_pinned_ffmpeg_media_probe,
    write_playable_qualification,
)

_IDENTITY: dict[str, Any] = {
    "run_id": "run-playable-001",
    "session_id": "thunderv4_factory_park_hf",
    "boot_id": "boot-playable-001",
    "model_generation": 3,
    "reset_generation": 2,
}
_MANEUVERS = ("forward", "backward", "left", "right", "turn_left", "turn_right")
_STREAM_RATES = {
    "thunder_01.front_depth": 30,
    "thunder_01.front_rgb": 30,
    "thunder_01.imu": 200,
    "thunder_01.mid360": 10,
    "thunder_01.truth_odom": 100,
}
_VALID_MEDIA_RESULT = {
    "width": 1920,
    "height": 1080,
    "duration_ns": 21_000_000_000,
    "frame_count": 600,
    "decode_error_count": 0,
}


def _valid_media_probe(_path: Path) -> dict[str, Any]:
    return dict(_VALID_MEDIA_RESULT)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, documents: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(document, ensure_ascii=False, sort_keys=True) + "\n" for document in documents),
        encoding="utf-8",
    )


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    values = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert all(isinstance(value, dict) for value in values)
    return values


def _assert_rejected(
    run_dir: Path,
    message: str,
    *,
    media_probe: Callable[[Path], dict[str, Any]] = _valid_media_probe,
) -> dict[str, Any]:
    verdict = _build_playable_qualification_for_test(
        run_dir,
        media_probe=media_probe,
        trusted_media_toolchain=_trusted_media_toolchain(run_dir),
    )
    assert verdict["schema"] == PLAYABLE_QUALIFICATION_SCHEMA
    assert verdict["result"] == "EVIDENCE_REJECTED"
    assert verdict["qualified"] is False
    assert any(message in reason for reason in verdict["reasons"]), verdict["reasons"]
    return verdict


def _trusted_media_toolchain(run_dir: Path) -> dict[str, Any]:
    document = _read_json(run_dir / "recording" / "media-toolchain.json")
    return {name: deepcopy(document[name]) for name in ("ffmpeg", "ffprobe")}


def _build_for_test(
    run_dir: Path,
    media_probe: Callable[[Path], dict[str, Any]] = _valid_media_probe,
) -> dict[str, Any]:
    return _build_playable_qualification_for_test(
        run_dir,
        media_probe=media_probe,
        trusted_media_toolchain=_trusted_media_toolchain(run_dir),
    )


def _rewrite_runtime_request_trace(
    run_dir: Path,
    documents: list[dict[str, Any]],
    *,
    record_count: int | None = None,
) -> None:
    trace_path = run_dir / "recording" / "runtime-request-trace.jsonl"
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    _write_jsonl(trace_path, documents)
    payload = trace_path.read_bytes()
    manifest = _read_json(manifest_path)
    descriptor = manifest["runtime_request_trace"]
    descriptor["sha256"] = _sha256(payload)
    descriptor["bytes"] = len(payload)
    descriptor["record_count"] = (
        len(documents) if record_count is None else record_count
    )
    _write_json(manifest_path, manifest)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def _test_png(width: int = 1920, height: int = 1080, *, packed_pixel_byte: int = 0xAA) -> bytes:
    # A tiny, valid, non-uniform 1-bit greyscale PNG.  Media decoding belongs
    # to the runner; the offline verifier independently checks its container.
    row = bytes([packed_pixel_byte]) * ((width + 7) // 8)
    pixels = b"".join(b"\x00" + row for _ in range(height))
    return b"".join(
        (
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 1, 0, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(pixels, level=9)),
            _png_chunk(b"IEND", b""),
        )
    )


_PNG_1920_1080 = _test_png()
_BLACK_PNG_1920_1080 = _test_png(packed_pixel_byte=0)


def _corrupt_idat_with_valid_crc(payload: bytes) -> bytes:
    corrupted = bytearray(payload)
    chunk_type = payload.index(b"IDAT")
    length = struct.unpack(">I", payload[chunk_type - 4 : chunk_type])[0]
    data_start = chunk_type + 4
    corrupted[data_start] ^= 0xFF
    crc_start = data_start + length
    crc = zlib.crc32(b"IDAT" + corrupted[data_start:crc_start]) & 0xFFFFFFFF
    corrupted[crc_start : crc_start + 4] = struct.pack(">I", crc)
    return bytes(corrupted)


def _corrupt_idat_crc(payload: bytes) -> bytes:
    corrupted = bytearray(payload)
    chunk_type = payload.index(b"IDAT")
    corrupted[chunk_type + 4] ^= 0x01
    return bytes(corrupted)


def _identity_fields() -> dict[str, Any]:
    return dict(_IDENTITY)


def _event_identity(sequence: int) -> dict[str, Any]:
    return {
        **_identity_fields(),
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": sequence,
        "event_id": f"boot-playable-001:1:{sequence}",
    }


def _accepted_segment_sha256(events: list[dict[str, Any]]) -> str:
    canonical = [
        {
            field: event[field]
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
        for event in events
    ]
    payload = json.dumps(
        canonical,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return _sha256(payload)


def _set_correlation_segment(correlation: dict[str, Any], events: list[dict[str, Any]]) -> None:
    first = events[0]
    last = events[-1]
    correlation.update(
        {
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
            "accepted_event_count": len(events),
            "accepted_events_sha256": _accepted_segment_sha256(events),
        }
    )


def _control_status_document(
    event: dict[str, Any],
    *,
    server_status_sequence: int,
    sim_time_ns: int,
    truth_sequence: int,
    ui_mode: str,
    camera_mode: str,
) -> dict[str, Any]:
    readiness = {
        name: {
            "state": "ACTIVE",
            "required": True,
            "source_id": f"runtime.{name}",
            "blocker": "",
        }
        for name in ("physics", "control", "visual", "sensors")
    }
    sensors = [
        {
            "stream_id": stream_id,
            "state": "ACTIVE",
            "sample_count": rate * 20,
            "blocker": "",
        }
        for stream_id, rate in sorted(_STREAM_RATES.items())
    ]
    return {
        "schema": "lingtu.sim.ue-control-status.v1",
        **_identity_fields(),
        "server_status_sequence": server_status_sequence,
        "server_monotonic_ns": 80_000_000_000 + server_status_sequence,
        "sim_time_ns": sim_time_ns,
        "truth_sequence": truth_sequence,
        "source_id": event["source_id"],
        "source_epoch": event["source_epoch"],
        "source_sequence": event["source_sequence"],
        "event_id": event["event_id"],
        "intent_datagram_sha256": event["datagram_sha256"],
        "status": "accepted",
        "reason": "",
        "runtime": {
            "runtime_state": "RUNNING",
            "control_owner": "robotsimue.local_player.0",
            "deadman": True,
            "sample_age_ns": 10_000_000,
            "safe_stop_state": "clear",
        },
        "motion": {
            "requested_axes": {
                "available": True,
                "forward": event["admitted_twist"]["linear_x"] / 0.1,
                "left": event["admitted_twist"]["linear_y"] / 0.1,
                "yaw_left": event["admitted_twist"]["angular_z"] / 0.35,
            },
            "admitted_twist_mps_radps": {
                "available": True,
                **event["admitted_twist"],
            },
            "observed_base_velocity_mps_radps": {
                "available": True,
                "linear_x": event["admitted_twist"]["linear_x"] * 0.8,
                "linear_y": event["admitted_twist"]["linear_y"] * 0.8,
                "angular_z": event["admitted_twist"]["angular_z"] * 0.8,
            },
        },
        "readiness": readiness,
        "sensors": sensors,
        "recording": {
            "state": "recording",
            "elapsed_sim_time_ns": sim_time_ns,
            "artifact_id": "simulation-recording.json",
            "blocker": "",
        },
        "ui": {"ui_mode": ui_mode, "camera_mode": camera_mode},
    }


def _write_valid_run(root: Path) -> Path:
    run_dir = root / "run-playable-001"
    run_dir.mkdir(parents=True)
    (run_dir / "logs").mkdir()
    (run_dir / "bundle").mkdir()
    media_tools_dir = root / "pinned-media-tools"
    media_tools_dir.mkdir()
    ffmpeg_path = media_tools_dir / "ffmpeg.exe"
    ffprobe_path = media_tools_dir / "ffprobe.exe"
    ffmpeg_payload = b"fixture pinned ffmpeg binary\n"
    ffprobe_payload = b"fixture pinned ffprobe binary\n"
    ffmpeg_path.write_bytes(ffmpeg_payload)
    ffprobe_path.write_bytes(ffprobe_payload)

    _write_json(
        run_dir / "run-allocation.json",
        {
            "schema": "lingtu.sim.run-allocation.v1",
            "run_id": _IDENTITY["run_id"],
            "session_id": _IDENTITY["session_id"],
            "artifact_root": str(run_dir.resolve()),
            "boot_id": _IDENTITY["boot_id"],
            "dds_domain": 83,
            "ports": {
                "visual_snapshot_udp": 25123,
                "control_intent_udp": 25124,
                "control_status_udp": 25125,
            },
            "shm": {},
            "log_dir": str((run_dir / "logs").resolve()),
        },
    )
    bindings = {
        name: {
            "required": True,
            "state": "ACTIVE",
            "source_id": f"fixture.{name}",
            "failure_reason": None,
            "model_generation": _IDENTITY["model_generation"],
            "reset_generation": _IDENTITY["reset_generation"],
        }
        for name in ("physics", "control", "visual", "sensors")
    }
    _write_json(
        run_dir / "session.runtime.json",
        {
            "schema": "lingtu.sim.session-runtime.v1",
            "run_id": _IDENTITY["run_id"],
            "session_id": _IDENTITY["session_id"],
            "model_generation": _IDENTITY["model_generation"],
            "reset_generation": _IDENTITY["reset_generation"],
            "state": "STOPPED",
            "bindings": bindings,
            "sensor_streams": {"summary_path": "sensor-stream-summary.json"},
            "bundle_dir": str((run_dir / "bundle").resolve()),
            "allocation": {
                "run_dir": str(run_dir.resolve()),
                "log_dir": str((run_dir / "logs").resolve()),
                "boot_id": _IDENTITY["boot_id"],
                "physics_pid": 4101,
                "dds_domain": 83,
                "ports": {
                    "visual_snapshot_udp": 25123,
                    "control_intent_udp": 25124,
                    "control_status_udp": 25125,
                },
                "shm": {},
                "shared_memory": {},
            },
            "clock": {"sequence": 12, "physics_step": 21_000, "sim_time_ns": 21_000_000_000},
        },
    )

    origin: list[dict[str, Any]] = []
    accepted: list[dict[str, Any]] = []
    twists = (
        {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": -0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.1, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": -0.1, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.35},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.35},
    )
    segment_events: list[list[dict[str, Any]]] = []
    source_sequence = 0
    for maneuver_index, twist in enumerate(twists):
        current_segment: list[dict[str, Any]] = []
        segment_start_ns = (maneuver_index * 2 + 1) * 1_000_000_000
        for periodic_index in range(3):
            source_sequence += 1
            datagram_sha256 = _sha256(f"ue-datagram-{source_sequence}".encode())
            common = _event_identity(source_sequence)
            origin.append(
                {
                    "schema": "lingtu.sim.ue-control-origin.v1",
                    **common,
                    "datagram_sha256": datagram_sha256,
                    "datagram_bytes": 512,
                    "successful_send": True,
                }
            )
            event = {
                "schema": "lingtu.sim.control-intent-accepted.v1",
                "event": "control_command_accepted",
                **common,
                "source_monotonic_ns": source_sequence * 1_000_000,
                "arrival_monotonic_ns": source_sequence * 1_000_000 + 1_000,
                "datagram_sha256": datagram_sha256,
                "controller_id": "thunder_01.thunderv4_locomotion",
                "channel_id": "thunder_01.control.base_twist",
                "controller_sequence": 100 + source_sequence,
                "apply_time_ns": segment_start_ns + periodic_index * 100_000_000,
                "submit_result": "accepted",
                "admitted_twist": twist,
            }
            accepted.append(event)
            current_segment.append(event)
        segment_events.append(current_segment)

    source_sequence += 1
    exit_hash = _sha256(f"ue-datagram-{source_sequence}".encode())
    origin.append(
        {
            "schema": "lingtu.sim.ue-control-origin.v1",
            **_event_identity(source_sequence),
            "datagram_sha256": exit_hash,
            "datagram_bytes": 384,
            "successful_send": True,
        }
    )
    accepted.append(
        {
            "schema": "lingtu.sim.control-command-zero.v1",
            "event": "control_command_zero",
            **_event_identity(source_sequence),
            "source_monotonic_ns": source_sequence * 1_000_000,
            "arrival_monotonic_ns": source_sequence * 1_000_000 + 1_000,
            "datagram_sha256": exit_hash,
            "controller_sequence": 100 + source_sequence,
            "apply_time_ns": 21_000_000_000,
            "submit_result": "accepted",
            "admitted_twist": {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0,
            },
            "reason": "cleared:exit",
        }
    )
    zero_audit: list[dict[str, Any]] = []
    zero_reasons = [
        "deadman_released",
        "runtime_request:control_release",
        "deadman_released",
        "runtime_request:control_release",
        "deadman_released",
        "runtime_request:control_release",
        "cleared:pause",
    ]
    for offset, reason in enumerate(zero_reasons, start=1):
        source_sequence += 1
        zero_hash = _sha256(f"ue-datagram-{source_sequence}".encode())
        origin.append(
            {
                "schema": "lingtu.sim.ue-control-origin.v1",
                **_event_identity(source_sequence),
                "datagram_sha256": zero_hash,
                "datagram_bytes": 384,
                "successful_send": True,
            }
        )
        zero_audit.append(
            {
                "schema": "lingtu.sim.control-command-zero.v1",
                "event": "control_command_zero",
                **_event_identity(source_sequence),
                "source_monotonic_ns": source_sequence * 1_000_000,
                "arrival_monotonic_ns": source_sequence * 1_000_000 + 1_000,
                "datagram_sha256": zero_hash,
                "controller_sequence": 200 + source_sequence,
                "apply_time_ns": offset * 2_000_000_000,
                "submit_result": "accepted",
                "admitted_twist": {
                    "linear_x": 0.0,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                },
                "reason": reason,
            }
        )
    runtime_requests: list[dict[str, Any]] = []
    fixed_runtime_requests = [
        ("ui_state_update", "accepted"),
        ("record_start", "accepted"),
    ]
    for _maneuver in _MANEUVERS:
        fixed_runtime_requests.extend(
            (("control_claim", "accepted"), ("control_release", "pending"))
        )
    fixed_runtime_requests.extend(
        (
            ("ui_state_update", "accepted"),  # C: camera mode
            ("control_release", "pending"),  # Tab: leave Drive safely
            ("ui_state_update", "accepted"),  # Tactical
            ("ui_state_update", "accepted"),  # Tab: return to Drive
            ("control_release", "pending"),  # Esc: enter recording Menu
            ("pause", "accepted"),
            ("ui_state_update", "accepted"),
            ("resume", "accepted"),
            ("ui_state_update", "accepted"),
            ("record_stop_commit", "accepted"),
            ("control_release", "pending"),  # Esc: enter committed Menu
            ("pause", "accepted"),
            ("ui_state_update", "accepted"),
            ("exit", "accepted"),
        )
    )
    for request, request_status in fixed_runtime_requests:
        source_sequence += 1
        request_hash = _sha256(f"ue-datagram-{source_sequence}".encode())
        common = _event_identity(source_sequence)
        origin.append(
            {
                "schema": "lingtu.sim.ue-control-origin.v1",
                **common,
                "datagram_sha256": request_hash,
                "datagram_bytes": 384,
                "successful_send": True,
            }
        )
        request_common = {
            "schema": "lingtu.sim.runtime-request-trace.v1",
            **common,
            "source_monotonic_ns": source_sequence * 1_000_000,
            "arrival_monotonic_ns": source_sequence * 1_000_000 + 1_000,
            "datagram_sha256": request_hash,
            "request": request,
        }
        runtime_requests.extend(
            (
                {"event": "runtime_request_received", **request_common},
                {
                    "event": "runtime_request_accepted",
                    **request_common,
                    "status": request_status,
                    "reason": (
                        "safe_zero_pending:control_release"
                        if request_status == "pending"
                        else ""
                    ),
                },
            )
        )
    _write_jsonl(run_dir / "logs" / "ue-control-origin.jsonl", origin)
    _write_jsonl(run_dir / "control-intent-accepted.jsonl", accepted)
    zero_audit_path = run_dir / "recording" / "control-command-zero-audit.jsonl"
    _write_jsonl(zero_audit_path, zero_audit)
    zero_audit_payload = zero_audit_path.read_bytes()
    runtime_request_trace_path = run_dir / "recording" / "runtime-request-trace.jsonl"
    _write_jsonl(runtime_request_trace_path, runtime_requests)
    runtime_request_trace_payload = runtime_request_trace_path.read_bytes()

    segment_endpoints = (
        ((0.0, 0.0, 0.0), (0.09, 0.0, 0.0)),
        ((0.09, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.09, 0.0)),
        ((0.0, 0.09, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.05, 0.0, 0.36)),
        ((0.05, 0.0, 0.36), (0.10, 0.0, 0.0)),
    )
    trajectory: list[dict[str, Any]] = []
    for segment_index, (start_point, end_point) in enumerate(segment_endpoints):
        segment_start_ns = (segment_index * 2 + 1) * 1_000_000_000
        for offset_ns, fraction in ((0, 0.0), (500_000_000, 0.5), (1_000_000_000, 1.0)):
            x = start_point[0] + (end_point[0] - start_point[0]) * fraction
            y = start_point[1] + (end_point[1] - start_point[1]) * fraction
            yaw = start_point[2] + (end_point[2] - start_point[2]) * fraction
            sequence = len(trajectory) + 1
            trajectory.append(
                {
                    "schema": "lingtu.sim.motion-trajectory-sample.v1",
                    **_identity_fields(),
                    "sequence": sequence,
                    "sim_time_ns": segment_start_ns + offset_ns,
                    "position_m": [x, y, 0.0],
                    "quaternion_wxyz": [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
                    "yaw_rad": yaw,
                }
            )
    _write_jsonl(run_dir / "motion-trajectory.jsonl", trajectory)

    correlations = []
    for index, maneuver in enumerate(_MANEUVERS):
        events = segment_events[index]
        start_event = events[0]
        end_event = events[-1]
        start_truth = trajectory[index * 3]
        end_truth = trajectory[index * 3 + 2]
        correlations.append(
            {
                "schema": "lingtu.sim.control-truth-correlation.v2",
                **_identity_fields(),
                "source_id": "robotsimue.local_player.0",
                "maneuver": maneuver,
                "start_event_id": start_event["event_id"],
                "end_event_id": end_event["event_id"],
                "start_source_epoch": start_event["source_epoch"],
                "start_source_sequence": start_event["source_sequence"],
                "end_source_epoch": end_event["source_epoch"],
                "end_source_sequence": end_event["source_sequence"],
                "start_datagram_sha256": start_event["datagram_sha256"],
                "end_datagram_sha256": end_event["datagram_sha256"],
                "start_controller_sequence": start_event["controller_sequence"],
                "end_controller_sequence": end_event["controller_sequence"],
                "start_apply_time_ns": start_event["apply_time_ns"],
                "end_apply_time_ns": end_event["apply_time_ns"],
                "accepted_event_count": len(events),
                "accepted_events_sha256": _accepted_segment_sha256(events),
                "truth_sequence_start": index * 3 + 1,
                "truth_sequence_end": index * 3 + 3,
                "truth_sim_time_ns_start": start_truth["sim_time_ns"],
                "truth_sim_time_ns_end": end_truth["sim_time_ns"],
                "truth_position_m_start": start_truth["position_m"],
                "truth_position_m_end": end_truth["position_m"],
                "truth_quaternion_wxyz_start": start_truth["quaternion_wxyz"],
                "truth_quaternion_wxyz_end": end_truth["quaternion_wxyz"],
                "frame_sequence_start": index * 100,
                "frame_sequence_end": index * 100 + 99,
            }
        )
    _write_jsonl(run_dir / "control-truth-correlation.jsonl", correlations)

    sensor_streams = [
        {
            "stream_id": stream_id,
            "state": "ACTIVE",
            "sample_count": rate * 21,
            "rate_hz": rate,
            "clock_domain": "mujoco_sim_time",
            "last_sample_truth_sequence": rate * 21 - 1,
            "last_sample_sim_time_ns": 21_000_000_000,
            "sample_age_ns": 0,
            "mapped_frame_count": 600 if rate == 30 else None,
        }
        for stream_id, rate in sorted(_STREAM_RATES.items())
    ]
    _write_json(
        run_dir / "sensor-stream-summary.json",
        {
            "schema": "lingtu.sim.playable-sensor-stream-summary.v1",
            **_identity_fields(),
            "is_ready": True,
            "required_stream_ids": sorted(_STREAM_RATES),
            "blocking_reasons": {},
            "streams": sensor_streams,
        },
    )

    frames_dir = run_dir / "frames"
    frames_dir.mkdir()
    recording_frames_dir = run_dir / "recording" / "frames"
    recording_frames_dir.mkdir()
    for sequence in range(600):
        (frames_dir / f"frame_{sequence:06d}.png").write_bytes(_PNG_1920_1080)
        (recording_frames_dir / f"frame_{sequence:06d}.png").write_bytes(_PNG_1920_1080)
    frame_capture_map: list[dict[str, Any]] = []
    for frame_sequence in range(600):
        segment_index, local_frame = divmod(frame_sequence, 100)
        truth_offset = 0 if local_frame < 33 else 1 if local_frame < 66 else 2
        truth = trajectory[segment_index * 3 + truth_offset]
        frame_capture_map.append(
            {
                "schema": "lingtu.sim.playable-frame-capture.v1",
                **_identity_fields(),
                "frame_sequence": frame_sequence,
                "truth_sequence": truth["sequence"],
                "sim_time_ns": truth["sim_time_ns"],
                "path": f"recording/frames/frame_{frame_sequence:06d}.png",
                "sha256": _sha256(_PNG_1920_1080),
                "bytes": len(_PNG_1920_1080),
            }
        )
    frame_capture_map_path = run_dir / "recording" / "frame-capture-map.jsonl"
    _write_jsonl(frame_capture_map_path, frame_capture_map)
    frame_capture_map_payload = frame_capture_map_path.read_bytes()

    videos_dir = run_dir / "videos"
    videos_dir.mkdir()
    raw_video = b"\x00\x00\x00\x18ftypisom\x00\x00\x02\x00isomiso2raw"
    labeled_video = b"\x00\x00\x00\x18ftypisom\x00\x00\x02\x00isomiso2labeled"
    (videos_dir / "playable-raw.mp4").write_bytes(raw_video)
    (videos_dir / "playable-labeled.mp4").write_bytes(labeled_video)

    screenshots_dir = run_dir / "screenshots"
    screenshots_dir.mkdir()
    hud_paths = {
        "drive": "screenshots/hud-drive.png",
        "tactical": "screenshots/hud-tactical.png",
        "menu_recording": "screenshots/hud-menu-recording.png",
    }
    for path in hud_paths.values():
        (run_dir / path).write_bytes(_PNG_1920_1080)

    timeline = [
        {
            "schema": "lingtu.sim.playable-recording-event.v1",
            **_identity_fields(),
            "sequence": 1,
            "sim_time_ns": 0,
            "event": "record_start",
        },
        {
            "schema": "lingtu.sim.playable-recording-event.v1",
            **_identity_fields(),
            "sequence": 2,
            "sim_time_ns": 21_000_000_000,
            "event": "record_stop_commit",
        },
    ]
    timeline_path = run_dir / "recording" / "recording.timeline.jsonl"
    _write_jsonl(timeline_path, timeline)
    timeline_payload = timeline_path.read_bytes()

    media_toolchain_path = run_dir / "recording" / "media-toolchain.json"
    _write_json(
        media_toolchain_path,
        {
            "schema": "lingtu.sim.playable-media-toolchain.v1",
            **_identity_fields(),
            "ffmpeg": {
                "path": str(ffmpeg_path.resolve()),
                "sha256": _sha256(ffmpeg_payload),
                "bytes": len(ffmpeg_payload),
                "version": "ffmpeg fixture 1.0",
            },
            "ffprobe": {
                "path": str(ffprobe_path.resolve()),
                "sha256": _sha256(ffprobe_payload),
                "bytes": len(ffprobe_payload),
                "version": "ffprobe fixture 1.0",
            },
        },
    )
    media_toolchain_payload = media_toolchain_path.read_bytes()

    controller_calibration_path = run_dir / "recording" / "controller-calibration.json"
    _write_json(
        controller_calibration_path,
        {
            "schema": "lingtu.sim.playable-controller-calibration.v1",
            **_identity_fields(),
            "controller_package_id": "thunderv4_locomotion",
            "controller_package_version": "1.0.0",
            "controller_manifest_path": "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml",
            "command_calibration": {
                "schema": "lingtu.sim.controller-command-calibration.v1",
                "scope": "quadruped_him_observation_only",
                "provenance": {
                    "source_id": "factory_park_turn_truth_qa_20260809",
                    "audit_note": (
                        "Source identifier records the calibration rationale only; no warehouse artifact is "
                        "claimed as yaw-turn qualification proof."
                    ),
                    "qualification_claim": False,
                },
                "external_yaw_cap_radps": 0.35,
                "policy_yaw_observation_gain": 1.2857142857142858,
                "policy_yaw_observation_limit_radps": 0.45,
                "leaves_base_twist_unchanged": True,
                "affected_axes": ["angular_z"],
            },
            "qualification_schedule": [
                {"maneuver": "turn_left", "key": "Q", "hold_s": 5.3},
                {"maneuver": "turn_right", "key": "E", "hold_s": 5.3},
            ],
            "first_probe_not_prior_qualification": True,
        },
    )
    controller_calibration_payload = controller_calibration_path.read_bytes()

    def video_descriptor(path: str, payload: bytes) -> dict[str, Any]:
        return {
            "path": path,
            "sha256": _sha256(payload),
            "bytes": len(payload),
            "duration_ns": 21_000_000_000,
            "width": 1920,
            "height": 1080,
            "decoded": True,
            "decode_error_count": 0,
            "black_frame_count": 0,
        }

    hud_specs: tuple[tuple[str, str, str, dict[str, Any], int, int], ...] = (
        ("drive", "drive", "follow", accepted[0], 18_000_000_000, 501),
        ("tactical", "tactical", "inspection", accepted[6], 19_000_000_000, 502),
        ("menu_recording", "menu", "free", accepted[12], 20_000_000_000, 503),
    )
    control_statuses: list[dict[str, Any]] = []
    hud: dict[str, dict[str, Any]] = {}
    for mode, ui_mode, camera_mode, event, sim_time_ns, server_sequence in hud_specs:
        status = _control_status_document(
            event,
            server_status_sequence=server_sequence,
            sim_time_ns=sim_time_ns,
            truth_sequence=1_000 + server_sequence,
            ui_mode=ui_mode,
            camera_mode=camera_mode,
        )
        control_statuses.append(status)
        received_monotonic_ns = status["server_monotonic_ns"] + 5_000_000
        captured_monotonic_ns = received_monotonic_ns + 10_000_000
        screenshot_path = hud_paths[mode]
        sidecar_path = str(Path(screenshot_path).with_suffix(".evidence.json")).replace("\\", "/")
        sidecar = {
            "schema": "lingtu.sim.ue-hud-screenshot-evidence.v1",
            "state": "CAPTURED",
            "capture_id": f"capture-{mode}-{server_sequence}",
            **_identity_fields(),
            "captured_monotonic_ns": captured_monotonic_ns,
            "status_age_ns": 10_000_000,
            "control_status": {
                "status": status["status"],
                "event_id": status["event_id"],
                "source_id": status["source_id"],
                "source_epoch": status["source_epoch"],
                "source_sequence": status["source_sequence"],
                "server_status_sequence": status["server_status_sequence"],
                "server_monotonic_ns": status["server_monotonic_ns"],
                "received_monotonic_ns": received_monotonic_ns,
                "ui_mode": ui_mode,
                "camera_mode": camera_mode,
            },
            "motion": status["motion"],
            "readiness": status["readiness"],
            "sensors": status["sensors"],
            "recording": status["recording"],
            "screenshot": {
                "basename": Path(screenshot_path).name,
                "bytes": len(_PNG_1920_1080),
                "width": 1920,
                "height": 1080,
                "show_ui": True,
            },
            "qualification_ready": True,
        }
        _write_json(run_dir / sidecar_path, sidecar)
        sidecar_payload = (run_dir / sidecar_path).read_bytes()
        hud[mode] = {
            "path": screenshot_path,
            "sha256": _sha256(_PNG_1920_1080),
            "bytes": len(_PNG_1920_1080),
            "sidecar_path": sidecar_path,
            "sidecar_sha256": _sha256(sidecar_payload),
            "sidecar_bytes": len(sidecar_payload),
            "server_status_sequence": server_sequence,
            "camera_mode": camera_mode,
            "width": 1920,
            "height": 1080,
            "captured_sim_time_ns": sim_time_ns,
            "snapshot_age_ns": 10_000_000,
            "snapshot_fresh": True,
            "motion_fields": ["requested", "admitted", "observed"],
            "recording_state": "recording",
            "nonblack_pixel_fraction": 0.5,
        }
    control_status_path = run_dir / "recording" / "control-status-authority.jsonl"
    _write_jsonl(control_status_path, control_statuses)
    control_status_payload = control_status_path.read_bytes()
    _write_json(
        run_dir / "recording" / "recording.manifest.json",
        {
            "schema": "lingtu.sim.playable-recording-manifest.v1",
            **_identity_fields(),
            "state": "COMMITTED",
            "product": {
                "world_package": "factory_park_hf@1.2.1",
                "robot_package": "thunderv4@1.0.3",
                "robot_instance_id": "thunder_01",
                "robot_color": "black_graphite",
                "controller_package": "thunderv4_locomotion@1.0.0",
            },
            "clock": {
                "authority": "mujoco",
                "start_sim_time_ns": 0,
                "end_sim_time_ns": 21_000_000_000,
                "duration_ns": 21_000_000_000,
            },
            "timeline": {
                "path": "recording/recording.timeline.jsonl",
                "sha256": _sha256(timeline_payload),
                "bytes": len(timeline_payload),
            },
            "media_toolchain": {
                "path": "recording/media-toolchain.json",
                "sha256": _sha256(media_toolchain_payload),
                "bytes": len(media_toolchain_payload),
            },
            "controller_calibration": {
                "path": "recording/controller-calibration.json",
                "sha256": _sha256(controller_calibration_payload),
                "bytes": len(controller_calibration_payload),
            },
            "control_status_authority": {
                "path": "recording/control-status-authority.jsonl",
                "sha256": _sha256(control_status_payload),
                "bytes": len(control_status_payload),
            },
            "runtime_request_trace": {
                "schema": "lingtu.sim.runtime-request-trace-descriptor.v1",
                "content_schema": "lingtu.sim.runtime-request-trace.v1",
                "path": "recording/runtime-request-trace.jsonl",
                "sha256": _sha256(runtime_request_trace_payload),
                "bytes": len(runtime_request_trace_payload),
                "record_count": len(runtime_requests),
            },
            "control_zero_audit": {
                "path": "recording/control-command-zero-audit.jsonl",
                "sha256": _sha256(zero_audit_payload),
                "bytes": len(zero_audit_payload),
            },
            "frame_capture_map": {
                "path": "recording/frame-capture-map.jsonl",
                "sha256": _sha256(frame_capture_map_payload),
                "bytes": len(frame_capture_map_payload),
            },
            "frames": {
                "directory": "recording/frames",
                "count": 600,
                "first_sequence": 0,
                "last_sequence": 599,
                "width": 1920,
                "height": 1080,
            },
            "videos": {
                "raw": video_descriptor("videos/playable-raw.mp4", raw_video),
                "labeled": video_descriptor("videos/playable-labeled.mp4", labeled_video),
            },
            "hud": hud,
        },
    )

    _write_json(
        run_dir / "shutdown-evidence.json",
        {
            "schema": "lingtu.sim.playable-shutdown-evidence.v1",
            **_identity_fields(),
            "natural_shutdown": True,
            "final_zero_event_id": "boot-playable-001:1:19",
            "resources_closed": {
                "control_intent_udp": True,
                "control_status_udp": True,
                "sensors": True,
                "recording": True,
            },
            "owned_processes": [
                {
                    "kind": "robotsimue",
                    "pid": 4100,
                    "owned": True,
                    "exit_code": 0,
                    "direct_child_running_after_close": False,
                    "process_owner_closed": True,
                    "termination_mode": "natural",
                },
                {
                    "kind": "mujoco",
                    "pid": 4101,
                    "owned": True,
                    "exit_code": 0,
                    "direct_child_running_after_close": False,
                    "process_owner_closed": True,
                    "termination_mode": "natural",
                },
            ],
            "scan_monotonic_ns": 99_000_000_000,
        },
    )
    _write_json(
        run_dir / "episode_result.json",
        {
            "schema": "lingtu.sim.episode-result.v1",
            "run_id": _IDENTITY["run_id"],
            "session_id": _IDENTITY["session_id"],
            "model_generation": _IDENTITY["model_generation"],
            "reset_generation": _IDENTITY["reset_generation"],
            "start_sim_time_ns": 0,
            "end_sim_time_ns": 21_000_000_000,
            "status": "SUCCEEDED",
            "failure_reason": None,
            "artifact_references": {
                "run_allocation": "run-allocation.json",
                "runtime_manifest": "session.runtime.json",
                "recording_manifest": "recording/recording.manifest.json",
                "shutdown_evidence": "shutdown-evidence.json",
            },
        },
    )
    return run_dir


def test_complete_playable_evidence_returns_and_atomically_writes_the_only_verdict_schema(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)

    verdict = _build_for_test(run_dir)

    assert verdict["schema"] == PLAYABLE_QUALIFICATION_SCHEMA
    assert verdict["schema"] == "lingtu.sim.ue5-playable-vertical-slice.v1"
    assert verdict["result"] == "PASS"
    assert verdict["qualified"] is True
    assert verdict["identity"] == _IDENTITY
    assert all(verdict["checks"].values())
    assert [item["name"] for item in verdict["maneuvers"]] == list(_MANEUVERS)
    assert len(verdict["artifacts"]) >= 616
    assert verdict["reasons"] == []

    destination = _write_playable_qualification_for_test(
        run_dir,
        media_probe=_valid_media_probe,
        trusted_media_toolchain=_trusted_media_toolchain(run_dir),
    )

    assert destination == run_dir / PLAYABLE_QUALIFICATION_FILENAME
    assert destination.name == "playable-qualification.json"
    committed = json.loads(destination.read_text(encoding="utf-8"))
    assert committed == verdict
    assert committed["schema"] == PLAYABLE_QUALIFICATION_SCHEMA
    assert list(run_dir.glob(".playable-qualification.json.*.tmp")) == []


def test_ue_origin_evidence_is_required_at_the_production_logs_path(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    canonical_origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    root_origin_path = run_dir / "ue-control-origin.jsonl"
    root_origin_path.write_bytes(canonical_origin_path.read_bytes())
    canonical_origin_path.unlink()

    _assert_rejected(run_dir, "required artifact is missing")


def test_runtime_request_trace_descriptor_pairs_and_ue_hash_join_are_independent_gates(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    trace_path = run_dir / "recording" / "runtime-request-trace.jsonl"
    origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    original_manifest = _read_json(manifest_path)
    original_trace = _read_jsonl(trace_path)
    original_origins = _read_jsonl(origin_path)

    trace_path.unlink()
    _assert_rejected(run_dir, "required artifact is missing")
    _write_jsonl(trace_path, original_trace)

    missing_descriptor_schema = deepcopy(original_manifest)
    del missing_descriptor_schema["runtime_request_trace"]["schema"]
    _write_json(manifest_path, missing_descriptor_schema)
    _assert_rejected(run_dir, "missing required field")
    _write_json(manifest_path, original_manifest)

    _rewrite_runtime_request_trace(
        run_dir,
        original_trace,
        record_count=len(original_trace) - 1,
    )
    _assert_rejected(run_dir, "record_count does not match")
    _write_json(manifest_path, original_manifest)

    _rewrite_runtime_request_trace(run_dir, original_trace[:-1])
    _assert_rejected(run_dir, "missing a contiguous received/terminal pair")
    _write_jsonl(trace_path, original_trace)
    _write_json(manifest_path, original_manifest)

    noncontiguous = deepcopy(original_trace)
    noncontiguous[1], noncontiguous[2] = noncontiguous[2], noncontiguous[1]
    _rewrite_runtime_request_trace(run_dir, noncontiguous)
    _assert_rejected(run_dir, "not a contiguous received/terminal pair")
    _write_jsonl(trace_path, original_trace)
    _write_json(manifest_path, original_manifest)

    duplicate_pair = [*deepcopy(original_trace), *deepcopy(original_trace[-2:])]
    _rewrite_runtime_request_trace(run_dir, duplicate_pair)
    _assert_rejected(run_dir, "duplicates a request event_id")
    _write_jsonl(trace_path, original_trace)
    _write_json(manifest_path, original_manifest)

    duplicate_hash = deepcopy(original_trace)
    duplicated_digest = duplicate_hash[0]["datagram_sha256"]
    second_event_id = duplicate_hash[2]["event_id"]
    duplicate_hash[2]["datagram_sha256"] = duplicated_digest
    duplicate_hash[3]["datagram_sha256"] = duplicated_digest
    duplicate_hash_origins = deepcopy(original_origins)
    second_origin = next(
        item for item in duplicate_hash_origins if item["event_id"] == second_event_id
    )
    second_origin["datagram_sha256"] = duplicated_digest
    _write_jsonl(origin_path, duplicate_hash_origins)
    _rewrite_runtime_request_trace(run_dir, duplicate_hash)
    _assert_rejected(run_dir, "duplicates a request datagram SHA-256")
    _write_jsonl(origin_path, original_origins)
    _write_jsonl(trace_path, original_trace)
    _write_json(manifest_path, original_manifest)

    broken_hash = deepcopy(original_trace)
    broken_hash[0]["datagram_sha256"] = "f" * 64
    broken_hash[1]["datagram_sha256"] = "f" * 64
    _rewrite_runtime_request_trace(run_dir, broken_hash)
    _assert_rejected(run_dir, "origin join mismatch for datagram_sha256")


def test_runtime_request_trace_requires_the_real_fixed_ui_control_and_lifecycle_transitions(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    trace_path = run_dir / "recording" / "runtime-request-trace.jsonl"
    original_manifest = _read_json(manifest_path)
    original_trace = _read_jsonl(trace_path)

    def restore() -> None:
        _write_jsonl(trace_path, original_trace)
        _write_json(manifest_path, original_manifest)

    unknown = deepcopy(original_trace)
    ui_offset = next(
        offset
        for offset in range(0, len(unknown), 2)
        if unknown[offset]["request"] == "ui_state_update"
    )
    unknown[ui_offset]["request"] = "debug_unlock"
    unknown[ui_offset + 1]["request"] = "debug_unlock"
    _rewrite_runtime_request_trace(run_dir, unknown)
    _assert_rejected(run_dir, "unexpected accepted runtime request")
    restore()

    reordered = deepcopy(original_trace)
    pause_offsets = [
        offset
        for offset in range(0, len(reordered), 2)
        if reordered[offset]["request"] == "pause"
    ]
    resume_offset = next(
        offset
        for offset in range(0, len(reordered), 2)
        if reordered[offset]["request"] == "resume"
    )
    for pair_offset, request in ((pause_offsets[0], "resume"), (resume_offset, "pause")):
        reordered[pair_offset]["request"] = request
        reordered[pair_offset + 1]["request"] = request
    _rewrite_runtime_request_trace(run_dir, reordered)
    _assert_rejected(run_dir, "runtime lifecycle must be")
    restore()

    no_ui = [
        item
        for pair_offset in range(0, len(original_trace), 2)
        if original_trace[pair_offset]["request"] != "ui_state_update"
        for item in deepcopy(original_trace[pair_offset : pair_offset + 2])
    ]
    _rewrite_runtime_request_trace(run_dir, no_ui)
    _assert_rejected(run_dir, "initial UI echo")
    restore()

    missing_claim_cycle = deepcopy(original_trace)
    first_claim = next(
        offset
        for offset in range(0, len(missing_claim_cycle), 2)
        if missing_claim_cycle[offset]["request"] == "control_claim"
    )
    del missing_claim_cycle[first_claim : first_claim + 4]
    _rewrite_runtime_request_trace(run_dir, missing_claim_cycle)
    _assert_rejected(run_dir, "does not cover all six maneuvers")
    restore()

    duplicate_claim = deepcopy(original_trace)
    first_release = next(
        offset
        for offset in range(0, len(duplicate_claim), 2)
        if duplicate_claim[offset]["request"] == "control_release"
    )
    for item in duplicate_claim[first_release : first_release + 2]:
        item["request"] = "control_claim"
    duplicate_claim[first_release + 1]["status"] = "accepted"
    duplicate_claim[first_release + 1]["reason"] = ""
    _rewrite_runtime_request_trace(run_dir, duplicate_claim)
    _assert_rejected(run_dir, "control_claim transition is duplicated")
    restore()

    without_committed_menu_pause = deepcopy(original_trace)
    second_pause = [
        offset
        for offset in range(0, len(without_committed_menu_pause), 2)
        if without_committed_menu_pause[offset]["request"] == "pause"
    ][1]
    del without_committed_menu_pause[second_pause : second_pause + 4]
    _rewrite_runtime_request_trace(run_dir, without_committed_menu_pause)
    _assert_rejected(run_dir, "runtime lifecycle must be")


def test_recording_frame_gate_is_isolated_from_raw_pre_and_post_recording_captures(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    raw_frames = run_dir / "frames"
    (raw_frames / "frame_999998.png").write_bytes(_PNG_1920_1080)
    (raw_frames / "frame_999999.png").write_bytes(_PNG_1920_1080)

    assert _build_for_test(run_dir)["result"] == "PASS"

    manifest_path = run_dir / "recording" / "recording.manifest.json"
    original_manifest = _read_json(manifest_path)
    nonzero_manifest = deepcopy(original_manifest)
    nonzero_manifest["frames"]["first_sequence"] = 1
    nonzero_manifest["frames"]["last_sequence"] = 600
    _write_json(manifest_path, nonzero_manifest)
    _assert_rejected(run_dir, "recording frame set is incomplete")
    _write_json(manifest_path, original_manifest)

    isolated_frames = run_dir / "recording" / "frames"
    (isolated_frames / "frame_000600.png").write_bytes(_PNG_1920_1080)
    _assert_rejected(run_dir, "recording frame sequence is missing or duplicated")


def test_ue_origin_receiver_acceptance_and_truth_join_fail_closed_for_empty_spoofed_or_partial_logs(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    accepted_path = run_dir / "control-intent-accepted.jsonl"
    correlation_path = run_dir / "control-truth-correlation.jsonl"
    original_origin = origin_path.read_bytes()
    original_accepted = accepted_path.read_bytes()
    original_correlation = correlation_path.read_bytes()

    accepted = _read_jsonl(accepted_path)
    accepted[0]["datagram_sha256"] = "b" * 64
    _write_jsonl(accepted_path, accepted)
    _assert_rejected(run_dir, "origin to receiver hash/event join mismatch")
    accepted_path.write_bytes(original_accepted)

    origin = _read_jsonl(origin_path)
    del origin[0]
    _write_jsonl(origin_path, origin)
    _assert_rejected(run_dir, "lacks UE successful-send origin")
    origin_path.write_bytes(original_origin)

    origin = _read_jsonl(origin_path)
    origin[0]["model_generation"] = 4
    _write_jsonl(origin_path, origin)
    _assert_rejected(run_dir, "model_generation is stale or mismatched")
    origin_path.write_bytes(original_origin)

    origin = _read_jsonl(origin_path)
    origin[0]["unknown_future_field"] = True
    _write_jsonl(origin_path, origin)
    _assert_rejected(run_dir, "unknown field")
    origin_path.write_bytes(original_origin)

    accepted_path.write_bytes(original_accepted.rstrip(b"\n"))
    _assert_rejected(run_dir, "partial JSONL")
    accepted_path.write_bytes(original_accepted)

    correlation = _read_jsonl(correlation_path)
    correlation[0]["start_datagram_sha256"] = "c" * 64
    _write_jsonl(correlation_path, correlation)
    _assert_rejected(run_dir, "segment boundary mismatch for start_datagram_sha256")
    correlation_path.write_bytes(original_correlation)

    correlation_path.write_text("", encoding="utf-8")
    _assert_rejected(run_dir, "required artifact is empty")
    correlation_path.write_bytes(original_correlation)

    origin = _read_jsonl(origin_path)
    accepted = _read_jsonl(accepted_path)
    next_sequence = max(int(item["source_sequence"]) for item in origin) + 1
    extra_hash = _sha256(f"ue-datagram-extra-{next_sequence}".encode())
    exit_sequence = next_sequence + 1
    exit_hash = _sha256(f"ue-datagram-exit-{exit_sequence}".encode())
    origin.append(
        {
            "schema": "lingtu.sim.ue-control-origin.v1",
            **_event_identity(next_sequence),
            "datagram_sha256": extra_hash,
            "datagram_bytes": 512,
            "successful_send": True,
        },
    )
    origin.append(
        {
            "schema": "lingtu.sim.ue-control-origin.v1",
            **_event_identity(exit_sequence),
            "datagram_sha256": exit_hash,
            "datagram_bytes": 384,
            "successful_send": True,
        }
    )
    accepted.insert(
        -1,
        {
            "schema": "lingtu.sim.control-intent-accepted.v1",
            "event": "control_command_accepted",
            **_event_identity(next_sequence),
            "source_monotonic_ns": next_sequence * 1_000_000,
            "arrival_monotonic_ns": next_sequence * 1_000_000 + 1_000,
            "datagram_sha256": extra_hash,
            "controller_id": "thunder_01.thunderv4_locomotion",
            "channel_id": "thunder_01.control.base_twist",
            "controller_sequence": 300 + next_sequence,
            "apply_time_ns": 12_500_000_000,
            "submit_result": "accepted",
            "admitted_twist": {
                "linear_x": 0.1,
                "linear_y": 0.0,
                "angular_z": 0.0,
            },
        },
    )
    accepted[-1].update(
        {
            **_event_identity(exit_sequence),
            "source_monotonic_ns": exit_sequence * 1_000_000,
            "arrival_monotonic_ns": exit_sequence * 1_000_000 + 1_000,
            "datagram_sha256": exit_hash,
            "controller_sequence": 301 + exit_sequence,
        }
    )
    shutdown_path = run_dir / "shutdown-evidence.json"
    shutdown = _read_json(shutdown_path)
    shutdown["final_zero_event_id"] = _event_identity(exit_sequence)["event_id"]
    _write_jsonl(origin_path, origin)
    _write_jsonl(accepted_path, accepted)
    _write_json(shutdown_path, shutdown)
    _assert_rejected(run_dir, "six segments do not exactly partition every accepted non-zero event")


def test_six_segments_reject_omission_movement_overlap_and_digest_tampering(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    correlation_path = run_dir / "control-truth-correlation.jsonl"
    accepted = _read_jsonl(run_dir / "control-intent-accepted.jsonl")[:-1]
    original = _read_jsonl(correlation_path)

    omitted = deepcopy(original)
    _set_correlation_segment(omitted[0], accepted[:2])
    _write_jsonl(correlation_path, omitted)
    _assert_rejected(run_dir, "segment boundary mismatch for start_event_id")

    moved = deepcopy(original)
    _set_correlation_segment(moved[0], accepted[:4])
    _write_jsonl(correlation_path, moved)
    _assert_rejected(run_dir, "accepted command is not aligned with maneuver forward")

    overlap = deepcopy(original)
    overlap[1]["start_event_id"] = overlap[0]["end_event_id"]
    _write_jsonl(correlation_path, overlap)
    _assert_rejected(run_dir, "segment boundary mismatch for start_event_id")

    digest_tamper = deepcopy(original)
    digest_tamper[0]["accepted_events_sha256"] = "b" * 64
    _write_jsonl(correlation_path, digest_tamper)
    _assert_rejected(run_dir, "accepted-event digest mismatch")

    _write_jsonl(correlation_path, original)


def test_exact_five_current_generation_positive_sensor_streams_are_mandatory(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    summary_path = run_dir / "sensor-stream-summary.json"
    original = _read_json(summary_path)

    cases: list[tuple[dict[str, Any], str]] = []
    missing = deepcopy(original)
    missing["streams"].pop()
    cases.append((missing, "sensor stream evidence is incomplete"))
    zero = deepcopy(original)
    zero["streams"][0]["sample_count"] = 0
    cases.append((zero, "must be positive"))
    severely_under_sampled = deepcopy(original)
    imu = next(item for item in severely_under_sampled["streams"] if item["stream_id"] == "thunder_01.imu")
    imu["sample_count"] = 1
    cases.append((severely_under_sampled, "below the canonical-rate minimum"))
    stale = deepcopy(original)
    stale["streams"][0]["last_sample_sim_time_ns"] = 20_899_999_999
    stale["streams"][0]["sample_age_ns"] = 100_000_001
    cases.append((stale, "evidence is stale"))
    under_mapped = deepcopy(original)
    under_mapped["streams"][0]["mapped_frame_count"] = 599
    cases.append((under_mapped, "fewer than 600 mapped frames"))
    unknown = deepcopy(original)
    unknown["required_stream_ids"].append("thunder_01.fake")
    cases.append((unknown, "exact five sensor streams"))
    missing_truth_sequence = deepcopy(original)
    del missing_truth_sequence["streams"][0]["last_sample_truth_sequence"]
    cases.append((missing_truth_sequence, "missing required field"))
    negative_truth_sequence = deepcopy(original)
    negative_truth_sequence["streams"][0]["last_sample_truth_sequence"] = -1
    cases.append((negative_truth_sequence, "must be a non-negative"))
    wall_clock_mixed_into_sim_summary = deepcopy(original)
    wall_clock_mixed_into_sim_summary["streams"][0]["clock_domain"] = "unix_realtime"
    cases.append((wall_clock_mixed_into_sim_summary, "clock_domain must be mujoco_sim_time"))

    for document, message in cases:
        _write_json(summary_path, document)
        _assert_rejected(run_dir, message)

    one_boundary_sample_short = deepcopy(original)
    imu = next(item for item in one_boundary_sample_short["streams"] if item["stream_id"] == "thunder_01.imu")
    imu["sample_count"] = 4_199
    _write_json(summary_path, one_boundary_sample_short)
    assert _build_for_test(run_dir)["result"] == "PASS"
    _write_json(summary_path, original)


def test_six_maneuvers_are_recomputed_from_truth_with_fixed_thresholds(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    trajectory_path = run_dir / "motion-trajectory.jsonl"
    correlation_path = run_dir / "control-truth-correlation.jsonl"
    original_trajectory = _read_jsonl(trajectory_path)
    original_correlation = _read_jsonl(correlation_path)

    short_translation = deepcopy(original_trajectory)
    short_translation[2]["position_m"][0] = 0.079
    _write_jsonl(trajectory_path, short_translation)
    short_translation_correlation = deepcopy(original_correlation)
    short_translation_correlation[0]["truth_position_m_end"] = short_translation[2]["position_m"]
    _write_jsonl(correlation_path, short_translation_correlation)
    _assert_rejected(run_dir, "less than 0.08 m")
    _write_jsonl(trajectory_path, original_trajectory)
    _write_jsonl(correlation_path, original_correlation)

    short_turn = deepcopy(original_trajectory)
    short_turn[14]["yaw_rad"] = 0.349
    short_turn[14]["quaternion_wxyz"] = [math.cos(0.349 / 2.0), 0.0, 0.0, math.sin(0.349 / 2.0)]
    _write_jsonl(trajectory_path, short_turn)
    short_turn_correlation = deepcopy(original_correlation)
    short_turn_correlation[4]["truth_quaternion_wxyz_end"] = short_turn[14]["quaternion_wxyz"]
    _write_jsonl(correlation_path, short_turn_correlation)
    _assert_rejected(run_dir, "less than 0.35 rad")
    _write_jsonl(trajectory_path, original_trajectory)
    _write_jsonl(correlation_path, original_correlation)

    drifting_turn = deepcopy(original_trajectory)
    drifting_turn[17]["position_m"][0] = 0.151
    _write_jsonl(trajectory_path, drifting_turn)
    drifting_turn_correlation = deepcopy(original_correlation)
    drifting_turn_correlation[5]["truth_position_m_end"] = drifting_turn[17]["position_m"]
    _write_jsonl(correlation_path, drifting_turn_correlation)
    _assert_rejected(run_dir, "more than 0.10 m")
    _write_jsonl(trajectory_path, original_trajectory)
    _write_jsonl(correlation_path, original_correlation)

    transient_drift = deepcopy(original_trajectory)
    transient_drift[13]["position_m"][0] = 0.101
    _write_jsonl(trajectory_path, transient_drift)
    _assert_rejected(run_dir, "more than 0.10 m")
    _write_jsonl(trajectory_path, original_trajectory)

    missing_truth = deepcopy(original_trajectory)
    del missing_truth[1]
    _write_jsonl(trajectory_path, missing_truth)
    _assert_rejected(run_dir, "missing sequence")
    _write_jsonl(trajectory_path, original_trajectory)

    outside_episode = deepcopy(original_trajectory)
    outside_sample = deepcopy(outside_episode[-1])
    outside_sample["sequence"] = 19
    outside_sample["sim_time_ns"] = 22_000_000_000
    outside_episode.append(outside_sample)
    _write_jsonl(trajectory_path, outside_episode)
    _assert_rejected(run_dir, "truth artifact timestamp is outside the episode")
    _write_jsonl(trajectory_path, original_trajectory)

    reordered = deepcopy(original_correlation)
    reordered[0], reordered[1] = reordered[1], reordered[0]
    _write_jsonl(correlation_path, reordered)
    _assert_rejected(run_dir, "incomplete or reordered")
    _write_jsonl(correlation_path, original_correlation)

    verdict = _build_for_test(run_dir)
    assert verdict["result"] == "PASS"
    assert verdict["maneuvers"][0]["signed_translation_m"] == pytest.approx(0.09)
    assert verdict["maneuvers"][4]["signed_yaw_rad"] == pytest.approx(0.36)
    assert verdict["maneuvers"][4]["horizontal_drift_m"] == pytest.approx(0.05)


def test_media_hud_episode_final_zero_and_clean_shutdown_are_all_required(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    episode_path = run_dir / "episode_result.json"
    shutdown_path = run_dir / "shutdown-evidence.json"
    accepted_path = run_dir / "control-intent-accepted.jsonl"
    original_manifest = _read_json(manifest_path)
    original_episode = _read_json(episode_path)
    original_shutdown = _read_json(shutdown_path)
    original_accepted = _read_jsonl(accepted_path)

    episode = deepcopy(original_episode)
    episode["status"] = "FAILED"
    episode["failure_reason"] = "fixture failure"
    _write_json(episode_path, episode)
    _assert_rejected(run_dir, "episode did not SUCCEED")
    _write_json(episode_path, original_episode)

    accepted = deepcopy(original_accepted)
    accepted[-1]["admitted_twist"]["linear_x"] = 0.01
    _write_jsonl(accepted_path, accepted)
    _assert_rejected(run_dir, "final accepted command is not zero")
    _write_jsonl(accepted_path, original_accepted)

    shutdown = deepcopy(original_shutdown)
    shutdown["owned_processes"][0]["direct_child_running_after_close"] = True
    _write_json(shutdown_path, shutdown)
    _assert_rejected(run_dir, "leaked or failed")
    _write_json(shutdown_path, original_shutdown)

    forced_termination = deepcopy(original_shutdown)
    forced_termination["owned_processes"][0]["termination_mode"] = "owned_terminate"
    _write_json(shutdown_path, forced_termination)
    _assert_rejected(run_dir, "leaked or failed")
    _write_json(shutdown_path, original_shutdown)

    duplicate_pid = deepcopy(original_shutdown)
    duplicate_pid["owned_processes"][0]["pid"] = duplicate_pid["owned_processes"][1]["pid"]
    _write_json(shutdown_path, duplicate_pid)
    _assert_rejected(run_dir, "process PIDs must be distinct")
    _write_json(shutdown_path, original_shutdown)

    short = deepcopy(original_manifest)
    short["clock"]["end_sim_time_ns"] = 19_000_000_000
    short["clock"]["duration_ns"] = 19_000_000_000
    _write_json(manifest_path, short)
    _assert_rejected(run_dir, ">=20 second")
    _write_json(manifest_path, original_manifest)

    missing_frame = run_dir / "recording" / "frames" / "frame_000321.png"
    frame_payload = missing_frame.read_bytes()
    missing_frame.unlink()
    _assert_rejected(run_dir, "frame sequence is missing")
    missing_frame.write_bytes(frame_payload)

    duplicate_frame = run_dir / "recording" / "frames" / "frame_000600.png"
    duplicate_frame.write_bytes(frame_payload)
    duplicate_manifest = deepcopy(original_manifest)
    duplicate_manifest["frames"]["count"] = 601
    duplicate_manifest["frames"]["last_sequence"] = 600
    _write_json(manifest_path, duplicate_manifest)
    _assert_rejected(run_dir, "frame capture map does not cover every UE frame exactly once")
    _write_json(manifest_path, original_manifest)
    duplicate_frame.unlink()

    black_video = deepcopy(original_manifest)
    black_video["videos"]["raw"]["black_frame_count"] = 1
    _write_json(manifest_path, black_video)
    _assert_rejected(run_dir, "undecodable, black")
    _write_json(manifest_path, original_manifest)

    def failed_independent_probe(_path: Path) -> dict[str, Any]:
        return {
            **_VALID_MEDIA_RESULT,
            "frame_count": 1,
            "decode_error_count": 1,
        }

    _assert_rejected(
        run_dir,
        "independent media probe rejected",
        media_probe=failed_independent_probe,
    )

    def extra_frame_probe(_path: Path) -> dict[str, Any]:
        return {
            **_VALID_MEDIA_RESULT,
            "frame_count": 601,
        }

    _assert_rejected(
        run_dir,
        "independent media probe rejected",
        media_probe=extra_frame_probe,
    )

    default_probe_verdict = build_playable_qualification(run_dir)
    assert default_probe_verdict["result"] == "EVIDENCE_REJECTED"
    assert any(
        "trusted pinned media toolchain descriptor is required" in reason
        for reason in default_probe_verdict["reasons"]
    )

    stale_hud = deepcopy(original_manifest)
    stale_hud["hud"]["drive"]["snapshot_age_ns"] = 100_000_001
    _write_json(manifest_path, stale_hud)
    _assert_rejected(run_dir, "HUD drive is stale")
    _write_json(manifest_path, original_manifest)

    outside_episode_hud = deepcopy(original_manifest)
    outside_episode_hud["hud"]["drive"]["captured_sim_time_ns"] = 21_000_000_001
    _write_json(manifest_path, outside_episode_hud)
    _assert_rejected(run_dir, "HUD drive is stale")
    _write_json(manifest_path, original_manifest)

    wrong_hud_hash = deepcopy(original_manifest)
    wrong_hud_hash["hud"]["drive"]["sha256"] = "b" * 64
    _write_json(manifest_path, wrong_hud_hash)
    _assert_rejected(run_dir, "HUD drive SHA-256 mismatch")
    _write_json(manifest_path, original_manifest)

    wrong_hud_geometry = deepcopy(original_manifest)
    wrong_hud_geometry["hud"]["drive"]["width"] = 1
    _write_json(manifest_path, wrong_hud_geometry)
    _assert_rejected(run_dir, "HUD drive is stale")
    _write_json(manifest_path, original_manifest)

    drive_path = run_dir / "screenshots" / "hud-drive.png"
    original_drive = drive_path.read_bytes()
    invalid_pngs = (
        b"\x89PNG\r\n\x1a\n" + bytes(33),
        original_drive[:-5],
        original_drive + b"trailing",
        _corrupt_idat_crc(original_drive),
        _corrupt_idat_with_valid_crc(original_drive),
    )
    for invalid_png in invalid_pngs:
        drive_path.write_bytes(invalid_png)
        invalid_manifest = deepcopy(original_manifest)
        invalid_manifest["hud"]["drive"]["sha256"] = _sha256(invalid_png)
        invalid_manifest["hud"]["drive"]["bytes"] = len(invalid_png)
        _write_json(manifest_path, invalid_manifest)
        _assert_rejected(run_dir, "strict PNG decode failed")
    drive_path.write_bytes(original_drive)
    _write_json(manifest_path, original_manifest)

    drive_path.write_bytes(_BLACK_PNG_1920_1080)
    black_hud = deepcopy(original_manifest)
    black_hud["hud"]["drive"]["sha256"] = _sha256(_BLACK_PNG_1920_1080)
    black_hud["hud"]["drive"]["bytes"] = len(_BLACK_PNG_1920_1080)
    drive_sidecar_path = run_dir / black_hud["hud"]["drive"]["sidecar_path"]
    original_drive_sidecar = _read_json(drive_sidecar_path)
    black_drive_sidecar = deepcopy(original_drive_sidecar)
    black_drive_sidecar["screenshot"]["bytes"] = len(_BLACK_PNG_1920_1080)
    _write_json(drive_sidecar_path, black_drive_sidecar)
    black_sidecar_payload = drive_sidecar_path.read_bytes()
    black_hud["hud"]["drive"]["sidecar_sha256"] = _sha256(black_sidecar_payload)
    black_hud["hud"]["drive"]["sidecar_bytes"] = len(black_sidecar_payload)
    _write_json(manifest_path, black_hud)
    _assert_rejected(run_dir, "computed nonblack pixel fraction")
    drive_path.write_bytes(original_drive)
    _write_json(drive_sidecar_path, original_drive_sidecar)
    _write_json(manifest_path, original_manifest)

    assert _build_for_test(run_dir)["result"] == "PASS"


def test_control_zero_audit_rejects_missing_forged_nonzero_duplicate_and_bad_segment_timing(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    zero_path = run_dir / "recording" / "control-command-zero-audit.jsonl"
    origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    original_manifest = _read_json(manifest_path)
    original_zero_audit = _read_jsonl(zero_path)
    original_origins = _read_jsonl(origin_path)

    def write_zero_case(documents: list[dict[str, Any]]) -> None:
        manifest = deepcopy(original_manifest)
        _write_jsonl(zero_path, documents)
        payload = zero_path.read_bytes()
        manifest["control_zero_audit"]["sha256"] = _sha256(payload)
        manifest["control_zero_audit"]["bytes"] = len(payload)
        _write_json(manifest_path, manifest)

    missing_release = deepcopy(original_zero_audit)
    missing_release[0]["reason"] = "cleared:pause"
    write_zero_case(missing_release)
    _assert_rejected(run_dir, "does not prove each released maneuver safety zero")

    forged_nonzero = deepcopy(original_zero_audit)
    forged_nonzero[1]["admitted_twist"]["linear_x"] = 0.01
    write_zero_case(forged_nonzero)
    _assert_rejected(run_dir, "control zero audit command is not zero")

    duplicate_event = deepcopy(original_zero_audit)
    for field in (
        "event_id",
        "source_id",
        "source_epoch",
        "source_sequence",
        "datagram_sha256",
    ):
        duplicate_event[1][field] = duplicate_event[0][field]
    write_zero_case(duplicate_event)
    _assert_rejected(run_dir, "control zero audit duplicates event_id")

    second_exit_zero = deepcopy(original_zero_audit)
    second_exit_zero[2]["reason"] = "cleared:exit"
    write_zero_case(second_exit_zero)
    _assert_rejected(run_dir, "contains a second cleared:exit zero")

    origin_mismatch = deepcopy(original_zero_audit)
    origin_mismatch[3]["datagram_sha256"] = "b" * 64
    write_zero_case(origin_mismatch)
    _assert_rejected(run_dir, "control zero audit origin join mismatch")

    bad_timing = deepcopy(original_zero_audit)
    for index, item in enumerate(bad_timing[:6]):
        item["apply_time_ns"] = 13_000_000_000 + index * 100_000_000
    write_zero_case(bad_timing)
    _assert_rejected(run_dir, "does not prove each released maneuver safety zero")

    _write_jsonl(origin_path, original_origins)
    write_zero_case(original_zero_audit)
    assert _build_for_test(run_dir)["result"] == "PASS"


def test_artifact_root_and_all_artifact_paths_are_bound_to_the_current_run(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    allocation_path = run_dir / "run-allocation.json"
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    original_allocation = _read_json(allocation_path)
    original_manifest = _read_json(manifest_path)

    foreign_root = tmp_path / "foreign-run"
    foreign_root.mkdir()
    allocation = deepcopy(original_allocation)
    allocation["artifact_root"] = str(foreign_root.resolve())
    _write_json(allocation_path, allocation)
    _assert_rejected(run_dir, "artifact_root does not bind the current run_dir")
    _write_json(allocation_path, original_allocation)

    lexical_escape = deepcopy(original_allocation)
    lexical_escape["artifact_root"] = str(run_dir / "frames" / "..")
    _write_json(allocation_path, lexical_escape)
    _assert_rejected(run_dir, "must be canonical and link-free")
    _write_json(allocation_path, original_allocation)

    for escaped_path in (
        "screenshots/other.png",
        "screenshots/../outside.png",
        str((tmp_path / "outside.png").resolve()),
    ):
        manifest = deepcopy(original_manifest)
        manifest["hud"]["drive"]["path"] = escaped_path
        _write_json(manifest_path, manifest)
        _assert_rejected(run_dir, "HUD drive path is stale")
    _write_json(manifest_path, original_manifest)


def test_hud_camera_modes_require_authoritative_nonunavailable_status_echo_and_transition(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    status_path = run_dir / "recording" / "control-status-authority.jsonl"
    original_manifest = _read_json(manifest_path)
    original_statuses = _read_jsonl(status_path)
    sidecar_paths = {
        mode: run_dir / descriptor["sidecar_path"]
        for mode, descriptor in original_manifest["hud"].items()
    }
    original_sidecars = {mode: _read_json(path) for mode, path in sidecar_paths.items()}

    def write_case(
        statuses: list[dict[str, Any]],
        sidecars: dict[str, dict[str, Any]],
        camera_modes: dict[str, str],
    ) -> None:
        manifest = deepcopy(original_manifest)
        _write_jsonl(status_path, statuses)
        status_payload = status_path.read_bytes()
        manifest["control_status_authority"]["sha256"] = _sha256(status_payload)
        manifest["control_status_authority"]["bytes"] = len(status_payload)
        for mode, sidecar in sidecars.items():
            _write_json(sidecar_paths[mode], sidecar)
            payload = sidecar_paths[mode].read_bytes()
            manifest["hud"][mode]["sidecar_sha256"] = _sha256(payload)
            manifest["hud"][mode]["sidecar_bytes"] = len(payload)
            manifest["hud"][mode]["camera_mode"] = camera_modes[mode]
        _write_json(manifest_path, manifest)

    unavailable_statuses = deepcopy(original_statuses)
    unavailable_sidecars = deepcopy(original_sidecars)
    unavailable_statuses[0]["ui"]["camera_mode"] = "unavailable"
    unavailable_sidecars["drive"]["control_status"]["camera_mode"] = "unavailable"
    write_case(
        unavailable_statuses,
        unavailable_sidecars,
        {"drive": "unavailable", "tactical": "inspection", "menu_recording": "free"},
    )
    _assert_rejected(run_dir, "camera_mode is unavailable or invalid")

    mismatched_sidecars = deepcopy(original_sidecars)
    mismatched_sidecars["drive"]["control_status"]["camera_mode"] = "free"
    write_case(
        deepcopy(original_statuses),
        mismatched_sidecars,
        {"drive": "free", "tactical": "inspection", "menu_recording": "free"},
    )
    _assert_rejected(run_dir, "full-status echo mismatch for camera_mode")

    no_transition_statuses = deepcopy(original_statuses)
    no_transition_sidecars = deepcopy(original_sidecars)
    for status in no_transition_statuses:
        status["ui"]["camera_mode"] = "follow"
    for sidecar in no_transition_sidecars.values():
        sidecar["control_status"]["camera_mode"] = "follow"
    write_case(
        no_transition_statuses,
        no_transition_sidecars,
        {mode: "follow" for mode in original_sidecars},
    )
    _assert_rejected(run_dir, "does not prove a real camera mode transition")


def test_hud_requires_authoritative_active_recording_artifact_and_elapsed_interval(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    status_path = run_dir / "recording" / "control-status-authority.jsonl"
    original_manifest = _read_json(manifest_path)
    original_statuses = _read_jsonl(status_path)

    def write_case(field: str, value: object) -> None:
        manifest = deepcopy(original_manifest)
        statuses = deepcopy(original_statuses)
        statuses[0]["recording"][field] = value
        _write_jsonl(status_path, statuses)
        status_payload = status_path.read_bytes()
        manifest["control_status_authority"]["sha256"] = _sha256(status_payload)
        manifest["control_status_authority"]["bytes"] = len(status_payload)
        drive_sidecar_path = run_dir / manifest["hud"]["drive"]["sidecar_path"]
        drive_sidecar = _read_json(drive_sidecar_path)
        drive_sidecar["recording"] = statuses[0]["recording"]
        _write_json(drive_sidecar_path, drive_sidecar)
        sidecar_payload = drive_sidecar_path.read_bytes()
        manifest["hud"]["drive"]["sidecar_sha256"] = _sha256(sidecar_payload)
        manifest["hud"]["drive"]["sidecar_bytes"] = len(sidecar_payload)
        _write_json(manifest_path, manifest)

    for field, value in (
        ("state", "idle"),
        ("artifact_id", ""),
        ("elapsed_sim_time_ns", 1),
    ):
        write_case(field, value)
        _assert_rejected(run_dir, "does not prove an active recording interval/artifact")


def test_controller_calibration_provenance_is_exact_and_cannot_self_qualify(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    calibration_path = run_dir / "recording" / "controller-calibration.json"
    original_manifest = _read_json(manifest_path)
    original = _read_json(calibration_path)

    def write_case(document: dict[str, Any]) -> None:
        manifest = deepcopy(original_manifest)
        _write_json(calibration_path, document)
        payload = calibration_path.read_bytes()
        manifest["controller_calibration"]["sha256"] = _sha256(payload)
        manifest["controller_calibration"]["bytes"] = len(payload)
        _write_json(manifest_path, manifest)

    self_claim = deepcopy(original)
    self_claim["command_calibration"]["provenance"]["qualification_claim"] = True
    write_case(self_claim)
    _assert_rejected(run_dir, "makes a qualification claim")

    wrong_schedule = deepcopy(original)
    wrong_schedule["qualification_schedule"][0]["hold_s"] = 5.2
    write_case(wrong_schedule)
    _assert_rejected(run_dir, "Q/E 5.3 second schedule is stale")

    prior_claim = deepcopy(original)
    prior_claim["first_probe_not_prior_qualification"] = False
    write_case(prior_claim)
    _assert_rejected(run_dir, "first_probe_not_prior_qualification")


def test_frame_capture_map_binds_every_png_to_monotonic_truth_without_requiring_equal_rates(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    map_path = run_dir / "recording" / "frame-capture-map.jsonl"
    trajectory_path = run_dir / "motion-trajectory.jsonl"
    correlation_path = run_dir / "control-truth-correlation.jsonl"
    original_manifest = _read_json(manifest_path)
    original_map = _read_jsonl(map_path)
    original_trajectory = _read_jsonl(trajectory_path)
    original_correlations = _read_jsonl(correlation_path)

    def write_map(documents: list[dict[str, Any]]) -> None:
        manifest = deepcopy(original_manifest)
        _write_jsonl(map_path, documents)
        payload = map_path.read_bytes()
        manifest["frame_capture_map"]["sha256"] = _sha256(payload)
        manifest["frame_capture_map"]["bytes"] = len(payload)
        _write_json(manifest_path, manifest)

    fake_index = deepcopy(original_map)
    fake_index[10]["frame_sequence"] = 11
    write_map(fake_index)
    _assert_rejected(run_dir, "sequence is missing, duplicated, or reordered")

    fake_hash = deepcopy(original_map)
    fake_hash[10]["sha256"] = "b" * 64
    write_map(fake_hash)
    _assert_rejected(run_dir, "does not match the captured PNG bytes")

    truncated = deepcopy(original_map[:-1])
    write_map(truncated)
    _assert_rejected(run_dir, "does not cover every UE frame exactly once")

    wrong_interval = deepcopy(original_map)
    for capture in wrong_interval[50:100]:
        capture["truth_sequence"] = 4
        capture["sim_time_ns"] = 3_000_000_000
    write_map(wrong_interval)
    _assert_rejected(run_dir, "escapes its truth interval")

    # Add a truth sample that lies between two captured truth snapshots.  A
    # 62.5 Hz physics stream naturally has samples a 30 Hz renderer will not
    # capture; the join must stay exact without demanding rate equality.
    asynchronous_trajectory = deepcopy(original_trajectory)
    inserted = deepcopy(asynchronous_trajectory[0])
    inserted.update(
        {
            "sequence": 2,
            "sim_time_ns": 1_250_000_000,
            "position_m": [0.0225, 0.0, 0.0],
        }
    )
    for sample in asynchronous_trajectory[1:]:
        sample["sequence"] += 1
    asynchronous_trajectory.insert(1, inserted)
    asynchronous_correlations = deepcopy(original_correlations)
    for correlation in asynchronous_correlations:
        if correlation["truth_sequence_start"] >= 2:
            correlation["truth_sequence_start"] += 1
        if correlation["truth_sequence_end"] >= 2:
            correlation["truth_sequence_end"] += 1
    asynchronous_map = deepcopy(original_map)
    for capture in asynchronous_map:
        if capture["truth_sequence"] >= 2:
            capture["truth_sequence"] += 1
    _write_jsonl(trajectory_path, asynchronous_trajectory)
    _write_jsonl(correlation_path, asynchronous_correlations)
    write_map(asynchronous_map)
    assert _build_for_test(run_dir)["result"] == "PASS"


def test_recording_clock_is_a_contained_twenty_second_interval_not_the_episode_boundary(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    timeline_path = run_dir / "recording" / "recording.timeline.jsonl"
    status_path = run_dir / "recording" / "control-status-authority.jsonl"
    manifest = _read_json(manifest_path)
    timeline = _read_jsonl(timeline_path)
    statuses = _read_jsonl(status_path)
    start_ns = 500_000_000
    end_ns = 20_500_000_000
    duration_ns = end_ns - start_ns
    manifest["clock"].update(
        {
            "start_sim_time_ns": start_ns,
            "end_sim_time_ns": end_ns,
            "duration_ns": duration_ns,
        }
    )
    timeline[0]["sim_time_ns"] = start_ns
    timeline[1]["sim_time_ns"] = end_ns
    _write_jsonl(timeline_path, timeline)
    timeline_payload = timeline_path.read_bytes()
    manifest["timeline"]["sha256"] = _sha256(timeline_payload)
    manifest["timeline"]["bytes"] = len(timeline_payload)
    for status in statuses:
        status["recording"]["elapsed_sim_time_ns"] = status["sim_time_ns"] - start_ns
    _write_jsonl(status_path, statuses)
    status_payload = status_path.read_bytes()
    manifest["control_status_authority"]["sha256"] = _sha256(status_payload)
    manifest["control_status_authority"]["bytes"] = len(status_payload)
    status_by_sequence = {status["server_status_sequence"]: status for status in statuses}
    for descriptor in manifest["hud"].values():
        sidecar_path = run_dir / descriptor["sidecar_path"]
        sidecar = _read_json(sidecar_path)
        sidecar["recording"] = status_by_sequence[descriptor["server_status_sequence"]]["recording"]
        _write_json(sidecar_path, sidecar)
        sidecar_payload = sidecar_path.read_bytes()
        descriptor["sidecar_sha256"] = _sha256(sidecar_payload)
        descriptor["sidecar_bytes"] = len(sidecar_payload)
    for video in manifest["videos"].values():
        video["duration_ns"] = duration_ns
    _write_json(manifest_path, manifest)

    def twenty_second_probe(_path: Path) -> dict[str, Any]:
        return {**_VALID_MEDIA_RESULT, "duration_ns": duration_ns, "frame_count": 600}

    verdict = _build_for_test(run_dir, twenty_second_probe)
    assert verdict["result"] == "PASS"


def test_media_probe_reads_a_verifier_owned_stable_copy_and_cannot_mutate_it(
    tmp_path: Path,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    source_paths = {
        (run_dir / "videos" / "playable-raw.mp4").resolve(),
        (run_dir / "videos" / "playable-labeled.mp4").resolve(),
    }
    probed_paths: list[Path] = []

    def stable_probe(path: Path) -> dict[str, Any]:
        resolved = path.resolve()
        assert resolved not in source_paths
        assert path.read_bytes()[4:8] == b"ftyp"
        probed_paths.append(resolved)
        return dict(_VALID_MEDIA_RESULT)

    verdict = _build_for_test(run_dir, stable_probe)
    assert verdict["result"] == "PASS"
    assert len(probed_paths) == 2
    assert all(not path.exists() for path in probed_paths)

    second_run = _write_valid_run(tmp_path / "mutating-probe")

    def mutating_probe(path: Path) -> dict[str, Any]:
        path.write_bytes(b"mutated by media probe")
        return dict(_VALID_MEDIA_RESULT)

    _assert_rejected(
        second_run,
        "media probe snapshot changed while decoding",
        media_probe=mutating_probe,
    )


def test_pinned_media_probe_runs_ffprobe_and_a_separate_full_ffmpeg_decode(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ffprobe_path = tmp_path / "ffprobe.exe"
    ffmpeg_path = tmp_path / "ffmpeg.exe"
    video_path = tmp_path / "video.mp4"
    ffprobe_path.write_bytes(b"pinned ffprobe")
    ffmpeg_path.write_bytes(b"pinned ffmpeg")
    video_path.write_bytes(b"\x00\x00\x00\x18ftypisomfixture")
    calls: list[list[str]] = []

    def successful_run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[bytes]:
        calls.append(command)
        if command[0] == str(ffprobe_path.resolve()):
            return subprocess.CompletedProcess(
                command,
                0,
                stdout=json.dumps(
                    {
                        "streams": [
                            {
                                "width": 1920,
                                "height": 1080,
                                "duration": "21.0",
                                "nb_read_frames": "600",
                            }
                        ],
                        "format": {"duration": "21.0"},
                    }
                ).encode(),
                stderr=b"",
            )
        return subprocess.CompletedProcess(command, 0, stdout=b"", stderr=b"")

    monkeypatch.setattr(subprocess, "run", successful_run)
    probe, descriptor = create_pinned_ffmpeg_media_probe(
        ffprobe_path=ffprobe_path,
        ffmpeg_path=ffmpeg_path,
        ffprobe_version="ffprobe fixture 1.0",
        ffmpeg_version="ffmpeg fixture 1.0",
    )
    result = probe(video_path)

    assert result == _VALID_MEDIA_RESULT
    assert descriptor == {
        "ffmpeg": {
            "path": str(ffmpeg_path.resolve()),
            "sha256": _sha256(b"pinned ffmpeg"),
            "bytes": len(b"pinned ffmpeg"),
            "version": "ffmpeg fixture 1.0",
        },
        "ffprobe": {
            "path": str(ffprobe_path.resolve()),
            "sha256": _sha256(b"pinned ffprobe"),
            "bytes": len(b"pinned ffprobe"),
            "version": "ffprobe fixture 1.0",
        },
    }
    assert len(calls) == 2
    assert "-count_frames" in calls[0]
    assert "-xerror" in calls[1]
    assert calls[1][-2:] == ["null", "-"]

    def failed_decode(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[bytes]:
        if command[0] == str(ffprobe_path.resolve()):
            return successful_run(command)
        return subprocess.CompletedProcess(command, 1, stdout=b"", stderr=b"decode failed")

    monkeypatch.setattr(subprocess, "run", failed_decode)
    with pytest.raises(PlayableQualificationError, match="full decode reported an error"):
        probe(video_path)

    ffmpeg_path.write_bytes(b"mutated pinned ffmpeg")
    with pytest.raises(PlayableQualificationError, match="changed after probe creation"):
        probe(video_path)


def test_production_verifier_requires_external_pinned_media_tools_and_artifact_can_only_compare(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    artifact_toolchain_path = run_dir / "recording" / "media-toolchain.json"
    manifest_path = run_dir / "recording" / "recording.manifest.json"
    artifact_toolchain = _read_json(artifact_toolchain_path)
    trusted = _trusted_media_toolchain(run_dir)
    probe, trusted_from_factory = create_pinned_ffmpeg_media_probe(
        ffprobe_path=Path(trusted["ffprobe"]["path"]),
        ffmpeg_path=Path(trusted["ffmpeg"]["path"]),
        ffprobe_version=trusted["ffprobe"]["version"],
        ffmpeg_version=trusted["ffmpeg"]["version"],
    )
    assert trusted_from_factory == trusted
    calls: list[list[str]] = []

    def successful_run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[bytes]:
        calls.append(command)
        if command[0] == trusted["ffprobe"]["path"]:
            return subprocess.CompletedProcess(
                command,
                0,
                stdout=json.dumps(
                    {
                        "streams": [
                            {
                                "width": 1920,
                                "height": 1080,
                                "duration": "21.0",
                                "nb_read_frames": "600",
                            }
                        ],
                        "format": {"duration": "21.0"},
                    }
                ).encode(),
                stderr=b"",
            )
        return subprocess.CompletedProcess(command, 0, stdout=b"", stderr=b"")

    monkeypatch.setattr(subprocess, "run", successful_run)
    assert build_playable_qualification(
        run_dir,
        trusted_media_probe=probe,
        trusted_media_toolchain=trusted,
    )["result"] == "PASS"

    arbitrary_called = False

    def arbitrary_probe(_path: Path) -> dict[str, Any]:
        nonlocal arbitrary_called
        arbitrary_called = True
        return dict(_VALID_MEDIA_RESULT)

    arbitrary = build_playable_qualification(
        run_dir,
        trusted_media_probe=arbitrary_probe,
        trusted_media_toolchain=trusted,
    )
    assert arbitrary["result"] == "EVIDENCE_REJECTED"
    assert any("trusted pinned media probe is required" in reason for reason in arbitrary["reasons"])
    assert arbitrary_called is False

    forged_called = False

    def forged_call(_self: object, _path: Path) -> dict[str, Any]:
        nonlocal forged_called
        forged_called = True
        return dict(_VALID_MEDIA_RESULT)

    forged_type = type("ForgedPinnedProbe", (type(probe),), {"__call__": forged_call})
    forged_probe = forged_type(
        ffprobe_path=Path(trusted["ffprobe"]["path"]),
        ffmpeg_path=Path(trusted["ffmpeg"]["path"]),
        ffprobe_version=trusted["ffprobe"]["version"],
        ffmpeg_version=trusted["ffmpeg"]["version"],
    )
    forged = build_playable_qualification(
        run_dir,
        trusted_media_probe=forged_probe,
        trusted_media_toolchain=trusted,
    )
    assert forged["result"] == "EVIDENCE_REJECTED"
    assert any("trusted pinned media probe is required" in reason for reason in forged["reasons"])
    assert forged_called is False

    missing_probe = build_playable_qualification(
        run_dir,
        trusted_media_toolchain=trusted,
    )
    assert missing_probe["result"] == "EVIDENCE_REJECTED"
    assert any("trusted pinned media probe is required" in reason for reason in missing_probe["reasons"])

    mismatched_trusted = deepcopy(trusted)
    mismatched_trusted["ffprobe"]["sha256"] = "b" * 64
    mismatch = build_playable_qualification(
        run_dir,
        trusted_media_probe=probe,
        trusted_media_toolchain=mismatched_trusted,
    )
    assert mismatch["result"] == "EVIDENCE_REJECTED"
    assert any("trusted media toolchain ffprobe SHA-256 mismatch" in reason for reason in mismatch["reasons"])

    calls_before_artifact_redirect = len(calls)
    redirected_artifact = deepcopy(artifact_toolchain)
    redirected_artifact["ffprobe"]["path"] = str((run_dir / "videos" / "playable-raw.mp4").resolve())
    _write_json(artifact_toolchain_path, redirected_artifact)
    redirected_payload = artifact_toolchain_path.read_bytes()
    redirected_manifest = _read_json(manifest_path)
    redirected_manifest["media_toolchain"]["sha256"] = _sha256(redirected_payload)
    redirected_manifest["media_toolchain"]["bytes"] = len(redirected_payload)
    _write_json(manifest_path, redirected_manifest)
    redirected = build_playable_qualification(
        run_dir,
        trusted_media_probe=probe,
        trusted_media_toolchain=trusted,
    )
    assert redirected["result"] == "EVIDENCE_REJECTED"
    assert any("does not match the externally trusted descriptor" in reason for reason in redirected["reasons"])
    assert len(calls) == calls_before_artifact_redirect
    _write_json(artifact_toolchain_path, artifact_toolchain)
    original_toolchain_payload = artifact_toolchain_path.read_bytes()
    original_manifest = _read_json(manifest_path)
    original_manifest["media_toolchain"]["sha256"] = _sha256(original_toolchain_payload)
    original_manifest["media_toolchain"]["bytes"] = len(original_toolchain_payload)
    _write_json(manifest_path, original_manifest)

    in_run_tools = run_dir / "recording" / "tools"
    in_run_tools.mkdir()
    in_run_ffprobe = in_run_tools / "ffprobe.exe"
    in_run_ffmpeg = in_run_tools / "ffmpeg.exe"
    in_run_ffprobe.write_bytes(b"inside run ffprobe")
    in_run_ffmpeg.write_bytes(b"inside run ffmpeg")
    in_run_probe, in_run_descriptor = create_pinned_ffmpeg_media_probe(
        ffprobe_path=in_run_ffprobe,
        ffmpeg_path=in_run_ffmpeg,
        ffprobe_version="inside run ffprobe",
        ffmpeg_version="inside run ffmpeg",
    )
    in_run = build_playable_qualification(
        run_dir,
        trusted_media_probe=in_run_probe,
        trusted_media_toolchain=in_run_descriptor,
    )
    assert in_run["result"] == "EVIDENCE_REJECTED"
    assert any("must be outside run_dir" in reason for reason in in_run["reasons"])

    destination = write_playable_qualification(
        run_dir,
        trusted_media_probe=probe,
        trusted_media_toolchain=trusted,
    )
    assert _read_json(destination)["result"] == "PASS"


def test_offline_verification_does_not_use_filesystem_mtime_as_freshness(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    real_stat = Path.stat
    calls = 0

    class _ChangingMtimeStat:
        def __init__(self, delegate: os.stat_result, mtime_ns: int) -> None:
            self._delegate = delegate
            self.st_mtime_ns = mtime_ns

        def __getattr__(self, name: str) -> Any:
            return getattr(self._delegate, name)

    def changing_mtime_stat(path: Path, *args: Any, **kwargs: Any) -> os.stat_result | _ChangingMtimeStat:
        nonlocal calls
        result = real_stat(path, *args, **kwargs)
        if path == origin_path:
            calls += 1
            return _ChangingMtimeStat(result, result.st_mtime_ns + calls)
        return result

    monkeypatch.setattr(Path, "stat", changing_mtime_stat)
    verdict = _build_for_test(run_dir)

    assert verdict["result"] == "PASS"
    assert calls >= 2


def test_symlink_reparse_artifacts_and_linked_verdict_destination_are_rejected(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_dir = _write_valid_run(tmp_path)
    origin_path = run_dir / "logs" / "ue-control-origin.jsonl"
    real_is_symlink = Path.is_symlink

    monkeypatch.setattr(
        Path,
        "is_symlink",
        lambda path: path == origin_path or real_is_symlink(path),
    )
    _assert_rejected(run_dir, "path component is symlink or reparse point")
    monkeypatch.setattr(Path, "is_symlink", real_is_symlink)

    real_lstat = os.lstat

    class _ReparseStat:
        def __init__(self, delegate: os.stat_result) -> None:
            self._delegate = delegate
            self.st_file_attributes = getattr(delegate, "st_file_attributes", 0) | 0x400

        def __getattr__(self, name: str) -> Any:
            return getattr(self._delegate, name)

    def artifact_reparse_lstat(path: os.PathLike[str] | str) -> os.stat_result | _ReparseStat:
        result = real_lstat(path)
        return _ReparseStat(result) if Path(path) == origin_path else result

    monkeypatch.setattr(os, "lstat", artifact_reparse_lstat)
    _assert_rejected(run_dir, "path component is symlink or reparse point")
    monkeypatch.setattr(os, "lstat", real_lstat)

    destination = run_dir / PLAYABLE_QUALIFICATION_FILENAME
    destination.write_text("occupied\n", encoding="utf-8")

    def destination_reparse_lstat(path: os.PathLike[str] | str) -> os.stat_result | _ReparseStat:
        result = real_lstat(path)
        return _ReparseStat(result) if Path(path) == destination else result

    monkeypatch.setattr(os, "lstat", destination_reparse_lstat)
    with pytest.raises(PlayableQualificationError, match="destination is a link"):
        _write_playable_qualification_for_test(
            run_dir,
            media_probe=_valid_media_probe,
            trusted_media_toolchain=_trusted_media_toolchain(run_dir),
        )
