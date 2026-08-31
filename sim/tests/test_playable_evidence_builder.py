"""Focused contracts for post-close playable evidence assembly."""

# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.coordinator import playable_evidence as playable_evidence_module
from sim.runtime.coordinator.playable_evidence import (
    PinnedPlayableMediaToolchain,
    PlayableEvidenceError,
    PlayableEvidenceInputs,
    _build_correlations,
    _build_sensor_summary,
    _build_shutdown_evidence,
    _controller_calibration_document,
    _encode_videos,
    _frame_capture_document,
    _FrameCapture,
    _Identity,
    _load_frame_captures,
    _materialize_recording_frames,
    _normalize_recording_captures,
    _publish_runtime_request_trace,
    _read_ue_control_origins,
    _recording_frames_descriptor,
    _require_recording_requests,
    _require_runtime_request_trace_archive_unchanged,
    _select_recording_captures,
    _snapshot_runtime_request_trace,
)
from sim.runtime.process_owner import ProcessShutdownSnapshot

_SESSION_ID = "playable-evidence-session"
_STREAM_RATES = {
    "thunder_01.front_depth": 30,
    "thunder_01.front_rgb": 30,
    "thunder_01.imu": 200,
    "thunder_01.mid360": 10,
    "thunder_01.truth_odom": 100,
}


def test_playable_evidence_product_uses_the_active_thunderv4_package() -> None:
    assert dict(playable_evidence_module.PLAYABLE_PRODUCT) == {
        "world_package": "factory_park_hf@1.2.1",
        "robot_package": "thunderv4@1.0.3",
        "robot_instance_id": "thunder_01",
        "robot_color": "black_graphite",
        "controller_package": "thunderv4_locomotion@1.0.0",
    }


def _identity() -> _Identity:
    return _Identity(
        run_id="playable-evidence-test",
        boot_id="boot-playable-evidence-test",
        session_id=_SESSION_ID,
        model_generation=3,
        reset_generation=2,
    )


def _capture(index: int, *, root: Path) -> _FrameCapture:
    return _FrameCapture(
        frame_sequence=index,
        truth_sequence=index + 1,
        sim_time_ns=(index + 1) * 20_000_000,
        model_generation=3,
        reset_generation=2,
        path=root / "frames" / f"frame_{index:06d}.png",
        sha256=f"{index + 1:064x}",
        bytes=100,
        nonblack_pixel_fraction=1.0,
    )


def _runtime_request_pair(
    request: str,
    sequence: int,
    *,
    status: str = "accepted",
) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = _identity()
    common = {
        **identity.fields(),
        "schema": "lingtu.sim.runtime-request-trace.v1",
        "event_id": f"{identity.boot_id}:1:{sequence}",
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": sequence,
        "request": request,
        "source_monotonic_ns": sequence * 10,
        "arrival_monotonic_ns": sequence * 10 + 1,
        "datagram_sha256": f"{sequence:064x}",
    }
    return (
        {**common, "event": "runtime_request_received"},
        {
            **common,
            "event": "runtime_request_accepted",
            "status": status,
            "reason": "" if status == "accepted" else f"safe_zero_pending:{request}",
        },
    )


def _flatten_pairs(
    pairs: Sequence[tuple[dict[str, Any], dict[str, Any]]],
) -> list[dict[str, Any]]:
    return [record for pair in pairs for record in pair]


def _origin_for_trace(trace: Mapping[str, Any]) -> dict[str, Any]:
    identity = _identity()
    return {
        **identity.fields(),
        "schema": "lingtu.sim.ue-control-origin.v1",
        "event_id": trace["event_id"],
        "source_id": trace["source_id"],
        "source_epoch": trace["source_epoch"],
        "source_sequence": trace["source_sequence"],
        "datagram_sha256": trace["datagram_sha256"],
        "successful_send": True,
    }


def test_builder_reads_cpp_origin_only_from_the_run_log_directory(
    tmp_path: Path,
) -> None:
    logs = tmp_path / "logs"
    logs.mkdir()
    expected = _origin_for_trace(_runtime_request_pair("exit", 1)[0])
    (logs / "ue-control-origin.jsonl").write_text(
        json.dumps(expected) + "\n",
        encoding="utf-8",
    )
    (tmp_path / "ue-control-origin.jsonl").write_text(
        "legacy-root-origin-must-not-be-read\n",
        encoding="utf-8",
    )

    assert _read_ue_control_origins(tmp_path) == (expected,)

    (logs / "ue-control-origin.jsonl").unlink()
    with pytest.raises(PlayableEvidenceError, match="cannot read completed evidence"):
        _read_ue_control_origins(tmp_path)


def test_recording_request_trace_requires_the_exact_real_two_pause_core_subsequence() -> None:
    pairs = [
        _runtime_request_pair("ui_state_update", 1),
        _runtime_request_pair("control_claim", 2),
        _runtime_request_pair("record_start", 3),
        _runtime_request_pair("control_release", 4, status="pending"),
        _runtime_request_pair("ui_state_update", 5),
        _runtime_request_pair("pause", 6),
        _runtime_request_pair("ui_state_update", 7),
        _runtime_request_pair("resume", 8),
        _runtime_request_pair("control_claim", 9),
        _runtime_request_pair("record_stop_commit", 10),
        _runtime_request_pair("pause", 11),
        _runtime_request_pair("ui_state_update", 12),
        _runtime_request_pair("exit", 13),
    ]
    traces = _flatten_pairs(pairs)
    origins = [_origin_for_trace(pair[0]) for pair in pairs]

    _require_recording_requests(traces, origins, _identity())

    duplicate_pair = _runtime_request_pair("record_start", 14)
    duplicate = [*traces, *duplicate_pair]
    with pytest.raises(PlayableEvidenceError, match="runtime request core"):
        _require_recording_requests(
            duplicate,
            [*origins, _origin_for_trace(duplicate_pair[0])],
            _identity(),
        )

    reused_pause_hash = [dict(trace) for trace in traces]
    first_pause_offset = 5 * 2
    second_pause_offset = 10 * 2
    for pair_member in (0, 1):
        reused_pause_hash[second_pause_offset + pair_member][
            "datagram_sha256"
        ] = reused_pause_hash[first_pause_offset]["datagram_sha256"]
    reused_origins = [_origin_for_trace(pair[0]) for pair in pairs]
    reused_origins[10]["datagram_sha256"] = reused_origins[5][
        "datagram_sha256"
    ]
    with pytest.raises(PlayableEvidenceError, match="datagram hash"):
        _require_recording_requests(
            reused_pause_hash,
            reused_origins,
            _identity(),
        )

    reordered_pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("exit", 3),
        _runtime_request_pair("resume", 4),
        _runtime_request_pair("record_stop_commit", 5),
        _runtime_request_pair("pause", 6),
    ]
    with pytest.raises(PlayableEvidenceError, match="runtime request core"):
        _require_recording_requests(
            _flatten_pairs(reordered_pairs),
            [_origin_for_trace(pair[0]) for pair in reordered_pairs],
            _identity(),
        )


def test_recording_request_trace_rejects_unknown_or_originless_accepted_requests() -> None:
    pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("resume", 3),
        _runtime_request_pair("debug_unlock", 4),
        _runtime_request_pair("record_stop_commit", 5),
        _runtime_request_pair("pause", 6),
        _runtime_request_pair("exit", 7),
    ]
    with pytest.raises(PlayableEvidenceError, match="unexpected accepted runtime request"):
        _require_recording_requests(
            _flatten_pairs(pairs),
            [_origin_for_trace(pair[0]) for pair in pairs],
            _identity(),
        )

    originless_pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("resume", 3),
        _runtime_request_pair("record_stop_commit", 4),
        _runtime_request_pair("pause", 5),
        _runtime_request_pair("exit", 6),
    ]
    origins = [_origin_for_trace(pair[0]) for pair in originless_pairs[:-1]]
    with pytest.raises(PlayableEvidenceError, match="lacks its UE successful-send hash"):
        _require_recording_requests(
            _flatten_pairs(originless_pairs),
            origins,
            _identity(),
        )


def test_recording_request_trace_rejects_nonadjacent_or_altered_pairs() -> None:
    pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("resume", 3),
        _runtime_request_pair("record_stop_commit", 4),
        _runtime_request_pair("pause", 5),
        _runtime_request_pair("exit", 6),
    ]
    origins = [_origin_for_trace(pair[0]) for pair in pairs]
    missing_terminal = _flatten_pairs(pairs)[:-1]
    with pytest.raises(PlayableEvidenceError, match="received/terminal pair"):
        _require_recording_requests(missing_terminal, origins, _identity())

    altered = _flatten_pairs(pairs)
    altered[1] = {**altered[1], "request": "resume"}
    with pytest.raises(PlayableEvidenceError, match="correlation was altered"):
        _require_recording_requests(altered, origins, _identity())


def test_recording_capture_selection_uses_only_contiguous_frames_inside_recording_clock(
    tmp_path: Path,
) -> None:
    captures = tuple(_capture(index, root=tmp_path) for index in range(10))

    selected = _select_recording_captures(
        captures,
        start_sim_time_ns=60_000_000,
        end_sim_time_ns=140_000_000,
    )

    assert [item.frame_sequence for item in selected] == [2, 3, 4, 5, 6]


def test_recording_frames_are_isolated_from_raw_pre_and_post_frames_and_renumbered(
    tmp_path: Path,
) -> None:
    raw_root = tmp_path / "frames"
    raw_root.mkdir()
    raw_captures = []
    for sequence, payload in enumerate((b"pre", b"selected-a", b"selected-b", b"post")):
        path = raw_root / f"frame_{sequence:06d}.png"
        path.write_bytes(payload)
        raw_captures.append(
            _FrameCapture(
                frame_sequence=sequence,
                truth_sequence=sequence + 1,
                sim_time_ns=(sequence + 1) * 20_000_000,
                model_generation=3,
                reset_generation=2,
                path=path,
                sha256=hashlib.sha256(payload).hexdigest(),
                bytes=len(payload),
                nonblack_pixel_fraction=1.0,
            )
        )

    selected = _select_recording_captures(
        raw_captures,
        start_sim_time_ns=40_000_000,
        end_sim_time_ns=60_000_000,
    )
    recording = _normalize_recording_captures(selected)
    destination = tmp_path / "recording" / "frames"
    destination.parent.mkdir()

    _materialize_recording_frames(
        recording,
        tmp_path,
    )

    assert [item.frame_sequence for item in recording] == [0, 1]
    assert [entry.name for entry in sorted(destination.iterdir())] == [
        "frame_000000.png",
        "frame_000001.png",
    ]
    assert (destination / "frame_000000.png").read_bytes() == b"selected-a"
    assert (destination / "frame_000001.png").read_bytes() == b"selected-b"
    documents = [
        _frame_capture_document(
            item,
            tmp_path,
            _identity(),
            frame_root=destination,
        )
        for item in recording
    ]
    assert [item["path"] for item in documents] == [
        "recording/frames/frame_000000.png",
        "recording/frames/frame_000001.png",
    ]
    assert [item["frame_sequence"] for item in documents] == [0, 1]
    assert _recording_frames_descriptor(recording) == {
        "directory": "recording/frames",
        "count": 2,
        "first_sequence": 0,
        "last_sequence": 1,
        "width": 1920,
        "height": 1080,
    }
    (raw_root / "frame_000001.png").write_bytes(b"mutated-after-copy")
    assert (destination / "frame_000000.png").read_bytes() == b"selected-a"


def test_video_encoding_reads_only_zero_based_isolated_recording_frames(
    tmp_path: Path,
) -> None:
    class _Toolchain(PinnedPlayableMediaToolchain):
        def __init__(self) -> None:
            self.calls: list[tuple[str, tuple[str, ...], float]] = []

        def run(
            self,
            name: str,
            arguments: Sequence[str],
            *,
            timeout_s: float,
        ) -> subprocess.CompletedProcess[bytes]:
            self.calls.append((name, tuple(arguments), timeout_s))
            return subprocess.CompletedProcess([], 0, b"", b"")

    recording_frames = tmp_path / "recording" / "frames"
    toolchain = _Toolchain()

    _encode_videos(
        toolchain,
        frames_dir=recording_frames,
        frame_count=600,
        duration_ns=20_000_000_000,
        raw_path=tmp_path / "videos" / "raw.mp4",
        labeled_path=tmp_path / "videos" / "labeled.mp4",
    )

    assert len(toolchain.calls) == 2
    for name, arguments, timeout_s in toolchain.calls:
        assert name == "ffmpeg"
        assert timeout_s == 600.0
        start_index = arguments.index("-start_number")
        input_index = arguments.index("-i")
        assert arguments[start_index + 1] == "0"
        assert arguments[input_index + 1] == str(
            recording_frames / "frame_%06d.png"
        )


def test_recording_frame_materialization_rejects_foreign_links_and_overwrite(
    tmp_path: Path,
) -> None:
    raw_root = tmp_path / "frames"
    raw_root.mkdir()
    foreign_root = tmp_path / "foreign"
    foreign_root.mkdir()
    payload = b"frame"
    foreign = foreign_root / "frame_000000.png"
    foreign.write_bytes(payload)
    capture = _FrameCapture(
        frame_sequence=0,
        truth_sequence=1,
        sim_time_ns=20_000_000,
        model_generation=3,
        reset_generation=2,
        path=foreign,
        sha256=hashlib.sha256(payload).hexdigest(),
        bytes=len(payload),
        nonblack_pixel_fraction=1.0,
    )
    destination_parent = tmp_path / "recording"
    destination_parent.mkdir()
    destination = destination_parent / "frames"

    with pytest.raises(PlayableEvidenceError, match="allocated raw frame directory"):
        _materialize_recording_frames(
            (capture,),
            tmp_path,
        )

    source = raw_root / "frame_000000.png"
    source.write_bytes(payload)
    linked = raw_root / "frame_000001.png"
    link_created = True
    try:
        os.symlink(source, linked)
    except OSError:
        link_created = False
    if link_created:
        linked_capture = _FrameCapture(
            frame_sequence=0,
            truth_sequence=1,
            sim_time_ns=20_000_000,
            model_generation=3,
            reset_generation=2,
            path=linked,
            sha256=hashlib.sha256(payload).hexdigest(),
            bytes=len(payload),
            nonblack_pixel_fraction=1.0,
        )
        with pytest.raises(PlayableEvidenceError, match="link/reparse"):
            _materialize_recording_frames(
                (linked_capture,),
                tmp_path,
            )

    destination.mkdir()
    with pytest.raises(PlayableEvidenceError, match="recording frame directory"):
        _materialize_recording_frames(
            (
                _FrameCapture(
                    frame_sequence=0,
                    truth_sequence=1,
                    sim_time_ns=20_000_000,
                    model_generation=3,
                    reset_generation=2,
                    path=source,
                    sha256=hashlib.sha256(payload).hexdigest(),
                    bytes=len(payload),
                    nonblack_pixel_fraction=1.0,
                ),
            ),
            tmp_path,
        )


def test_frame_capture_loader_rejects_a_logged_link_before_resolving_it(
    tmp_path: Path,
) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    logs = tmp_path / "logs"
    logs.mkdir()
    foreign = tmp_path / "foreign.png"
    foreign.write_bytes(b"foreign frame")
    linked = frames / "frame_000000.png"
    try:
        os.symlink(foreign, linked)
    except OSError:
        pytest.skip("symbolic-link creation is unavailable on this Windows host")
    (logs / "Unreal.log").write_text(
        "LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED "
        "capture_index=0 model_generation=3 reset_generation=2 "
        "sequence=1 sim_time_ns=20000000 "
        f"path={linked} requested=1 max=1\n",
        encoding="utf-8",
    )

    with pytest.raises(PlayableEvidenceError, match="link/reparse"):
        _load_frame_captures(
            tmp_path,
            _identity(),
            unreal_log_path=logs / "Unreal.log",
        )


@pytest.mark.parametrize(
    ("log_name", "wrong_name"),
    (("Unreal.log", "RobotSimUE.log"), ("RobotSimUE.log", "Unreal.log")),
)
def test_frame_capture_loader_accepts_only_the_exact_direct_allocated_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    log_name: str,
    wrong_name: str,
) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    logs = tmp_path / "logs"
    logs.mkdir()
    frame = frames / "frame_000000.png"
    payload = b"direct frame"
    frame.write_bytes(payload)
    (logs / log_name).write_text(
        "LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED "
        "capture_index=0 model_generation=3 reset_generation=2 "
        "sequence=1 sim_time_ns=20000000 "
        f"path={frame} requested=1 max=1\n",
        encoding="utf-8",
    )
    (logs / wrong_name).write_text(
        "wrong surface log must not be read\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        playable_evidence_module,
        "_decode_png",
        lambda _payload, _label: (1920, 1080, 1.0),
    )

    captures = _load_frame_captures(
        tmp_path,
        _identity(),
        unreal_log_path=logs / log_name,
    )

    assert len(captures) == 1
    assert captures[0].path == frame
    assert captures[0].sha256 == hashlib.sha256(payload).hexdigest()

    (logs / log_name).unlink()
    with pytest.raises(PlayableEvidenceError, match="cannot read completed evidence"):
        _load_frame_captures(
            tmp_path,
            _identity(),
            unreal_log_path=logs / log_name,
        )


def test_recording_frame_materialization_rejects_duplicate_hardlink_sources(
    tmp_path: Path,
) -> None:
    frames = tmp_path / "frames"
    frames.mkdir()
    recording = tmp_path / "recording"
    recording.mkdir()
    first = frames / "frame_000000.png"
    second = frames / "frame_000001.png"
    payload = b"shared frame"
    first.write_bytes(payload)
    try:
        os.link(first, second)
    except OSError:
        pytest.skip("hard-link creation is unavailable on this Windows host")
    digest = hashlib.sha256(payload).hexdigest()
    captures = tuple(
        _FrameCapture(
            frame_sequence=sequence,
            truth_sequence=sequence + 1,
            sim_time_ns=(sequence + 1) * 20_000_000,
            model_generation=3,
            reset_generation=2,
            path=path,
            sha256=digest,
            bytes=len(payload),
            nonblack_pixel_fraction=1.0,
        )
        for sequence, path in enumerate((first, second))
    )

    with pytest.raises(PlayableEvidenceError, match="source file identity"):
        _materialize_recording_frames(captures, tmp_path)


def test_runtime_request_trace_is_archived_byte_exact_with_complete_descriptor(
    tmp_path: Path,
) -> None:
    pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("resume", 3),
        _runtime_request_pair("record_stop_commit", 4),
        _runtime_request_pair("pause", 5),
        _runtime_request_pair("exit", 6),
    ]
    traces = _flatten_pairs(pairs)
    origins = [_origin_for_trace(pair[0]) for pair in pairs]
    source = tmp_path / "runtime-request-trace.jsonl"
    payload = b"".join(
        (b"  " + json.dumps(trace).encode("utf-8") + b"\n")
        for trace in traces
    )
    source.write_bytes(payload)
    snapshot = _snapshot_runtime_request_trace(source, origins, _identity())
    destination_parent = tmp_path / "recording"
    destination_parent.mkdir()
    destination = destination_parent / "runtime-request-trace.jsonl"

    descriptor = _publish_runtime_request_trace(
        snapshot,
        run_root=tmp_path,
    )

    assert destination.read_bytes() == payload
    assert descriptor == {
        "schema": "lingtu.sim.runtime-request-trace-descriptor.v1",
        "content_schema": "lingtu.sim.runtime-request-trace.v1",
        "path": "recording/runtime-request-trace.jsonl",
        "sha256": hashlib.sha256(payload).hexdigest(),
        "bytes": len(payload),
        "record_count": 12,
    }
    _require_runtime_request_trace_archive_unchanged(
        snapshot,
        run_root=tmp_path,
        descriptor=descriptor,
    )
    destination.write_text("{}\n", encoding="utf-8")
    with pytest.raises(PlayableEvidenceError, match="archive changed"):
        _require_runtime_request_trace_archive_unchanged(
            snapshot,
            run_root=tmp_path,
            descriptor=descriptor,
        )


@pytest.mark.parametrize("mutation", ["delete", "replace", "recreate_same"])
def test_runtime_request_trace_archive_fails_closed_if_validated_source_changes(
    tmp_path: Path,
    mutation: str,
) -> None:
    pairs = [
        _runtime_request_pair("record_start", 1),
        _runtime_request_pair("pause", 2),
        _runtime_request_pair("resume", 3),
        _runtime_request_pair("record_stop_commit", 4),
        _runtime_request_pair("pause", 5),
        _runtime_request_pair("exit", 6),
    ]
    traces = _flatten_pairs(pairs)
    origins = [_origin_for_trace(pair[0]) for pair in pairs]
    source = tmp_path / "runtime-request-trace.jsonl"
    source.write_text(
        "".join(json.dumps(trace) + "\n" for trace in traces),
        encoding="utf-8",
    )
    snapshot = _snapshot_runtime_request_trace(source, origins, _identity())
    original_payload = source.read_bytes()
    if mutation == "delete":
        source.unlink()
    elif mutation == "recreate_same":
        source.unlink()
        source.write_bytes(original_payload)
    else:
        source.write_text("{}\n", encoding="utf-8")
    destination_parent = tmp_path / "recording"
    destination_parent.mkdir()
    destination = destination_parent / "runtime-request-trace.jsonl"

    with pytest.raises(PlayableEvidenceError, match="changed after validation"):
        _publish_runtime_request_trace(
            snapshot,
            run_root=tmp_path,
        )

    assert not destination.exists()


def test_sensor_summary_preserves_actual_truth_stamp_for_exact_five_streams(
    tmp_path: Path,
) -> None:
    captures = tuple(_capture(index, root=tmp_path) for index in range(630))
    streams = {
        stream_id: {
            "state": "ACTIVE",
            "sample_count": rate * 21,
            "last_sample_truth_sequence": 1050,
            "last_sample_sim_time_ns": 21_000_000_000,
        }
        for stream_id, rate in _STREAM_RATES.items()
    }
    runtime = {"sensor_streams": {"summary": {"streams": streams}}}
    episode = {"end_sim_time_ns": 21_000_000_000}

    summary = _build_sensor_summary(runtime, episode, captures, _identity())

    assert summary["is_ready"] is True
    assert [item["stream_id"] for item in summary["streams"]] == sorted(
        _STREAM_RATES
    )
    assert all(
        item["last_sample_truth_sequence"] == 1050
        and item["clock_domain"] == "mujoco_sim_time"
        and item["last_sample_sim_time_ns"] == 21_000_000_000
        and item["sample_age_ns"] == 0
        for item in summary["streams"]
    )
    assert {
        item["mapped_frame_count"]
        for item in summary["streams"]
        if item["stream_id"] in {"thunder_01.front_depth", "thunder_01.front_rgb"}
    } == {630}

    del streams["thunder_01.front_rgb"]["last_sample_truth_sequence"]
    with pytest.raises(PlayableEvidenceError, match="last truth sequence"):
        _build_sensor_summary(runtime, episode, captures, _identity())


def test_sensor_summary_rejects_more_mapped_camera_frames_than_published_samples(
    tmp_path: Path,
) -> None:
    captures = tuple(_capture(index, root=tmp_path) for index in range(630))
    streams = {
        stream_id: {
            "state": "ACTIVE",
            "sample_count": rate * 21,
            "last_sample_truth_sequence": 1050,
            "last_sample_sim_time_ns": 21_000_000_000,
        }
        for stream_id, rate in _STREAM_RATES.items()
    }
    streams["thunder_01.front_rgb"]["sample_count"] = 629

    with pytest.raises(
        PlayableEvidenceError,
        match="camera frames exceed published samples",
    ):
        _build_sensor_summary(
            {"sensor_streams": {"summary": {"streams": streams}}},
            {"end_sim_time_ns": 21_000_000_000},
            captures,
            _identity(),
        )


def test_controller_calibration_records_first_probe_without_self_qualification() -> None:
    document = _controller_calibration_document(_identity())

    assert document["controller_manifest_path"].endswith("controller.package.yaml")
    assert document["command_calibration"]["provenance"][
        "qualification_claim"
    ] is False
    assert document["qualification_schedule"] == [
        {"maneuver": "turn_left", "key": "Q", "hold_s": 5.3},
        {"maneuver": "turn_right", "key": "E", "hold_s": 5.3},
    ]
    assert document["first_probe_not_prior_qualification"] is True


def test_correlation_builder_partitions_every_motion_into_exact_six_segments(
    tmp_path: Path,
) -> None:
    identity = _identity()
    twists = (
        {"linear_x": 0.5, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": -0.5, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.5, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": -0.5, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.35},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.35},
    )
    trajectory = tuple(
        {
            "sequence": index + 1,
            "sim_time_ns": (index + 1) * 20_000_000,
            "position_m": [float(index), 0.0, 0.0],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        }
        for index in range(18)
    )
    captures = tuple(_capture(index, root=tmp_path) for index in range(18))
    records: list[dict[str, Any]] = []
    origins: list[dict[str, Any]] = []
    sequence = 1
    for segment_index, twist in enumerate(twists):
        for offset in range(2):
            truth_index = segment_index * 3 + offset
            event_id = f"event-{sequence}"
            digest = f"{sequence:064x}"
            event = {
                **identity.fields(),
                "schema": "lingtu.sim.control-intent-accepted.v1",
                "event_id": event_id,
                "source_id": "robotsimue.local_player.0",
                "source_epoch": 1,
                "source_sequence": sequence,
                "datagram_sha256": digest,
                "controller_sequence": sequence,
                "apply_time_ns": trajectory[truth_index]["sim_time_ns"],
                "admitted_twist": twist,
            }
            records.append(event)
            origins.append(
                {
                    **identity.fields(),
                    "schema": "lingtu.sim.ue-control-origin.v1",
                    "event_id": event_id,
                    "source_id": event["source_id"],
                    "source_epoch": 1,
                    "source_sequence": sequence,
                    "datagram_sha256": digest,
                    "successful_send": True,
                }
            )
            sequence += 1
    records.append(
        {
            **identity.fields(),
            "schema": "lingtu.sim.control-command-zero.v1",
            "event_id": "exit-zero",
            "source_id": "robotsimue.local_player.0",
            "source_epoch": 1,
            "source_sequence": sequence,
            "datagram_sha256": f"{sequence:064x}",
            "controller_sequence": sequence,
            "apply_time_ns": trajectory[-1]["sim_time_ns"],
            "admitted_twist": {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0,
            },
            "reason": "cleared:exit",
        }
    )
    origins.append(
        {
            **identity.fields(),
            "schema": "lingtu.sim.ue-control-origin.v1",
            "event_id": "exit-zero",
            "source_id": "robotsimue.local_player.0",
            "source_epoch": 1,
            "source_sequence": sequence,
            "datagram_sha256": f"{sequence:064x}",
            "successful_send": True,
        }
    )

    correlations = _build_correlations(
        records,
        origins,
        trajectory,
        captures,
        identity,
    )

    assert [item["maneuver"] for item in correlations] == [
        "forward",
        "backward",
        "left",
        "right",
        "turn_left",
        "turn_right",
    ]
    assert [item["accepted_event_count"] for item in correlations] == [2] * 6
    assert correlations[0]["frame_sequence_start"] == 0
    assert correlations[-1]["frame_sequence_end"] == 17

    asynchronous = tuple(captures[index] for index in range(1, 18, 2))
    asynchronous_correlations = _build_correlations(
        records,
        origins,
        trajectory,
        asynchronous,
        identity,
    )
    assert asynchronous_correlations[0]["frame_sequence_start"] == 1
    assert asynchronous_correlations[0]["truth_sequence_start"] == 1


def test_shutdown_builder_uses_only_immutable_owned_process_facts(
    tmp_path: Path,
) -> None:
    toolchain = object.__new__(PinnedPlayableMediaToolchain)
    natural = ProcessShutdownSnapshot(
        pid=101,
        exit_code=0,
        direct_child_running_after_close=False,
        process_owner_closed=True,
        termination_mode="natural",
    )
    inputs = PlayableEvidenceInputs(
        run_dir=tmp_path,
        run_id=_identity().run_id,
        boot_id=_identity().boot_id,
        session_id=_SESSION_ID,
        unreal_log_path=tmp_path / "logs" / "Unreal.log",
        media_toolchain=toolchain,
        unreal_shutdown=natural,
        mujoco_shutdown=ProcessShutdownSnapshot(
            pid=202,
            exit_code=0,
            direct_child_running_after_close=False,
            process_owner_closed=True,
            termination_mode="natural",
        ),
        resources_closed={
            "control_intent_udp": True,
            "control_status_udp": True,
            "sensors": True,
            "recording": True,
        },
    )
    accepted = (
        {
            "schema": "lingtu.sim.control-command-zero.v1",
            "reason": "cleared:exit",
            "event_id": "exit-zero",
        },
    )

    evidence = _build_shutdown_evidence(inputs, _identity(), accepted)

    assert evidence["natural_shutdown"] is True
    assert [item["pid"] for item in evidence["owned_processes"]] == [101, 202]

    forced = ProcessShutdownSnapshot(
        pid=101,
        exit_code=1,
        direct_child_running_after_close=False,
        process_owner_closed=True,
        termination_mode="owned_terminate",
    )
    forced_inputs = PlayableEvidenceInputs(
        run_dir=tmp_path,
        run_id=_identity().run_id,
        boot_id=_identity().boot_id,
        session_id=_SESSION_ID,
        unreal_log_path=tmp_path / "logs" / "Unreal.log",
        media_toolchain=toolchain,
        unreal_shutdown=forced,
        mujoco_shutdown=inputs.mujoco_shutdown,
        resources_closed=inputs.resources_closed,
    )
    assert _build_shutdown_evidence(
        forced_inputs,
        _identity(),
        accepted,
    )["natural_shutdown"] is False


def test_evidence_inputs_require_one_link_free_authoritative_run_log(
    tmp_path: Path,
) -> None:
    toolchain = object.__new__(PinnedPlayableMediaToolchain)

    def construct(path: Path) -> PlayableEvidenceInputs:
        return PlayableEvidenceInputs(
            run_dir=tmp_path,
            run_id=_identity().run_id,
            boot_id=_identity().boot_id,
            session_id=_SESSION_ID,
            unreal_log_path=path,
            media_toolchain=toolchain,
            unreal_shutdown=None,
            mujoco_shutdown=None,
            resources_closed={},
        )

    with pytest.raises(PlayableEvidenceError, match="under run/logs"):
        construct(tmp_path / "Unreal.log")
    with pytest.raises(PlayableEvidenceError, match="under run/logs"):
        construct(tmp_path / "logs" / "Editor.log")

    logs = tmp_path / "logs"
    logs.mkdir()
    foreign = tmp_path / "foreign.log"
    foreign.write_text("foreign\n", encoding="utf-8")
    linked = logs / "Unreal.log"
    try:
        os.symlink(foreign, linked)
    except OSError:
        pytest.skip("symbolic-link creation is unavailable on this Windows host")
    with pytest.raises(PlayableEvidenceError, match="link/reparse"):
        construct(linked)
