# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from pathlib import Path
from typing import Any

import pytest

import sim.runtime.coordinator.atomic_file as atomic_file_module
from sim.runtime.qualification.session_record import QualificationRecordError
from sim.runtime.recording import (
    SimulationRecordingWriter,
    write_qualification_result,
)


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")


def _completed_run(tmp_path: Path) -> tuple[Path, Path]:
    bundle_dir = tmp_path / "bundle"
    run_dir = tmp_path / "run"
    bundle_dir.mkdir()
    run_dir.mkdir()
    session_id = "recording-qualification-session"
    run_id = "recording-qualification"
    _write_json(
        bundle_dir / "session.yaml",
        {
            "schema": "lingtu.sim.session.v1",
            "session_id": session_id,
            "runtime": {"required_bindings": ["physics"]},
        },
    )
    _write_json(
        run_dir / "run-allocation.json",
        {
            "schema": "lingtu.sim.run-allocation.v1",
            "run_id": run_id,
            "session_id": session_id,
            "dds_domain": 79,
            "ports": {},
            "shm": {},
        },
    )
    _write_json(
        run_dir / "session.runtime.json",
        {
            "schema": "lingtu.sim.session-runtime.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 2,
            "reset_generation": 3,
            "state": "STOPPED",
            "bindings": {"physics": {"required": True, "state": "ACTIVE"}},
            "sensor_streams": {
                "is_ready": True,
                "required_stream_ids": [],
                "streams": {},
            },
            "allocation": {
                "run_dir": str(run_dir.resolve()),
                "dds_domain": 79,
                "ports": {},
                "shm": {},
            },
        },
    )
    with SimulationRecordingWriter(
        run_dir,
        run_id=run_id,
        session_id=session_id,
    ) as recorder:
        recorder.append(
            {
                "event": "snapshot",
                "session_id": session_id,
                "model_generation": 2,
                "reset_generation": 3,
                "sequence": 0,
                "physics_step": 0,
                "sim_time_ns": 0,
                "bodies": [],
                "joints": [],
                "actuators": [],
            }
        )
    _write_json(
        run_dir / "motion-evidence.json",
        {
            "schema": "lingtu.sim.motion-recording-evidence.v2",
            "run_id": run_id,
            "session_id": session_id,
            "frames": {"captured_count": 1, "minimum_frames": 1},
            "maneuvers": [
                {
                    "name": "forward",
                    "command": {
                        "linear_x": 0.1,
                        "linear_y": 0.0,
                        "angular_z": 0.0,
                    },
                    "translation_commanded": True,
                    "rotation_commanded": False,
                    "signed_translation_m": 0.1,
                    "minimum_displacement_m": 0.05,
                    "signed_yaw_rad": 0.0,
                    "horizontal_drift_m": 0.0,
                    "minimum_rotation_rad": 0.35,
                    "maximum_turn_drift_m": 0.1,
                    "motion_verified": True,
                    "translation_met": True,
                    "rotation_met": True,
                    "turn_drift_met": True,
                    "expectation_met": True,
                }
            ],
        },
    )
    _write_json(
        run_dir / "episode_result.json",
        {
            "schema": "lingtu.sim.episode-result.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 2,
            "reset_generation": 3,
            "status": "SUCCEEDED",
            "failure_reason": None,
            "artifact_references": {
                "runtime_manifest": "session.runtime.json",
                "run_allocation": "run-allocation.json",
                "simulation_recording": "simulation-recording.json",
                "simulation_timeline": "simulation-timeline.jsonl",
            },
        },
    )
    return bundle_dir, run_dir


def test_qualification_result_atomically_commits_the_public_builder_verdict(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    episode_before = (run_dir / "episode_result.json").read_bytes()

    path = write_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert path == run_dir / "qualification_result.json"
    assert result["schema"] == "lingtu.sim.qualification-result.v1"
    assert result["qualified"] is True
    assert result["verdict"] == "EVIDENCE_QUALIFIED"
    assert result["record"]["schema"] == "lingtu.sim.e2e-qualification-record.v1"
    assert result["record"]["identity"] == {
        "run_id": "recording-qualification",
        "session_id": "recording-qualification-session",
        "model_generation": 2,
        "reset_generation": 3,
    }
    assert result["error"] is None
    assert (run_dir / "episode_result.json").read_bytes() == episode_before


def test_qualification_result_rejects_rehashed_timeline_generation_tampering(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    first_path = write_qualification_result(bundle_dir, run_dir)
    assert json.loads(first_path.read_text(encoding="utf-8"))["qualified"] is True
    episode_before = (run_dir / "episode_result.json").read_bytes()

    timeline_path = run_dir / "simulation-timeline.jsonl"
    frame = json.loads(timeline_path.read_text(encoding="utf-8"))
    frame["snapshot"]["model_generation"] = 9
    payload = (
        json.dumps(frame, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    timeline_path.write_bytes(payload)
    manifest_path = run_dir / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["timeline"].update(
        {
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    )
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    rejected_path = write_qualification_result(bundle_dir, run_dir)

    rejected = json.loads(rejected_path.read_text(encoding="utf-8"))
    assert rejected["qualified"] is False
    assert rejected["verdict"] == "EVIDENCE_REJECTED"
    assert rejected["record"] is None
    assert rejected["record_sha256"] is None
    assert rejected["error"]["code"] == "QUALIFICATION_EVIDENCE_INVALID"
    assert "model_generation" in rejected["error"]["message"]
    assert (run_dir / "episode_result.json").read_bytes() == episode_before


def test_qualification_rejects_recording_self_reported_wide_tolerance(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    manifest_path = run_dir / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["continuous_tolerances"] = {
        "/bodies/*/position_m/*": 1000.0,
    }
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    path = write_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert result["qualified"] is False
    assert result["record"] is None
    assert result["verdict"] == "EVIDENCE_REJECTED"
    assert "trusted maximum" in result["error"]["message"]


@pytest.mark.parametrize("mutation", ["missing", "empty", "truncated"])
def test_qualification_result_rejects_incomplete_required_timeline(
    tmp_path: Path,
    mutation: str,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    timeline = run_dir / "simulation-timeline.jsonl"
    if mutation == "missing":
        timeline.unlink()
    elif mutation == "empty":
        timeline.write_bytes(b"")
    else:
        timeline.write_bytes(timeline.read_bytes()[:-7])

    path = write_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert result["qualified"] is False
    assert result["verdict"] == "EVIDENCE_REJECTED"
    assert result["record"] is None
    assert result["error"]["code"] == "QUALIFICATION_EVIDENCE_INVALID"
    assert "timeline" in result["error"]["message"]


@pytest.mark.parametrize("mutation", ["empty_maneuvers", "truncated_json"])
def test_qualification_result_rejects_empty_or_truncated_motion_evidence(
    tmp_path: Path,
    mutation: str,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion_path = run_dir / "motion-evidence.json"
    if mutation == "empty_maneuvers":
        motion = json.loads(motion_path.read_text(encoding="utf-8"))
        motion["maneuvers"] = []
        motion_path.write_text(json.dumps(motion, sort_keys=True), encoding="utf-8")
    else:
        motion_path.write_text('{"schema":', encoding="utf-8")

    path = write_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert result["qualified"] is False
    assert result["verdict"] == "EVIDENCE_REJECTED"
    if mutation == "empty_maneuvers":
        assert result["record"] is not None
        assert "motion evidence is not qualified" in result["record"]["reasons"]
    else:
        assert result["record"] is None
        assert result["error"]["code"] == "QUALIFICATION_EVIDENCE_INVALID"


def test_qualification_atomic_replace_failure_preserves_previous_verdict(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    path = write_qualification_result(bundle_dir, run_dir)
    previous = path.read_bytes()

    def fail_replace(_source: Path, _destination: Path) -> None:
        raise OSError("qualification replace unavailable")

    monkeypatch.setattr(atomic_file_module, "replace_file_with_retry", fail_replace)

    with pytest.raises(OSError, match="qualification replace unavailable"):
        write_qualification_result(bundle_dir, run_dir)

    assert path.read_bytes() == previous
    assert list(run_dir.glob(".qualification_result.json.*.tmp")) == []


def test_qualification_result_rejects_bundle_symlink_before_builder_resolve(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    actual_bundle = tmp_path / "actual-bundle"
    bundle_dir.rename(actual_bundle)
    try:
        bundle_dir.symlink_to(actual_bundle, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with pytest.raises(QualificationRecordError, match=r"reparse|symlink"):
        write_qualification_result(bundle_dir, run_dir)


def test_qualification_result_rejects_run_junction_before_builder_resolve(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    actual_run = tmp_path / "actual-run"
    run_dir.rename(actual_run)
    cmd = os.environ.get("ComSpec", r"C:\Windows\System32\cmd.exe")
    result = subprocess.run(  # noqa: S603
        [cmd, "/c", "mklink", "/J", str(run_dir), str(actual_run)],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.skip(f"junction creation is unavailable: {result.stderr or result.stdout}")

    with pytest.raises(QualificationRecordError, match=r"reparse|symlink"):
        write_qualification_result(bundle_dir, run_dir)
