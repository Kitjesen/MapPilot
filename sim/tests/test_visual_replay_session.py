"""End-to-end contract for presenting a committed recording in RobotSimUE."""

# ruff: noqa: S101

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.recording import SimulationRecordingWriter
from sim.runtime.replay.visual import (
    VisualReplayConfig,
    VisualReplayError,
    run_visual_replay,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim"
    / "scenarios"
    / "catalog"
    / "thunderv4_factory_park_motion"
    / "session.yaml"
)
PNG = b"\x89PNG\r\n\x1a\nfixture"


def _snapshot(
    session_id: str,
    *,
    sequence: int,
    sim_time_ns: int,
    x: float,
) -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": session_id,
        "model_generation": 0,
        "reset_generation": 0,
        "sequence": sequence,
        "physics_step": sequence * 8,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "instance_id": "thunder_01",
                "frame_id": "base_link",
                "position_m": [x, 0.0, 0.5],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        ],
        "joints": [],
        "actuators": [],
    }


def _bundle_and_recording(tmp_path: Path) -> tuple[Path, Path, str]:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    shutil.copyfile(SESSION, bundle / "session.yaml")
    session_id = resolved.session_id
    recording = tmp_path / "recording"
    with SimulationRecordingWriter(
        recording,
        run_id="source-motion",
        session_id=session_id,
    ) as writer:
        for index, x in enumerate((0.0, 0.1, 0.2), start=1):
            writer.append(
                _snapshot(
                    session_id,
                    sequence=index,
                    sim_time_ns=index * 20_000_000,
                    x=x,
                )
            )
    return bundle, recording, session_id


class _Harness:
    def __init__(self, *, exit_code: int | None = None) -> None:
        self.exit_code = exit_code
        self.allocation: Any | None = None
        self.process_started = False
        self.process_terminated = False
        self.publisher_closed = False
        self.published: list[dict[str, Any]] = []

    def unreal_factory(self, *args: object, **kwargs: object) -> _FakeUnreal:
        del args, kwargs
        return _FakeUnreal(self)

    def publisher_factory(self, port: int) -> _FakePublisher:
        assert port == 25401
        return _FakePublisher(self)

    def emit_visual_evidence(self, event: dict[str, Any]) -> None:
        assert self.allocation is not None
        log_dir = self.allocation.log_dir
        evidence = {
            "schema": "lingtu.sim.sensor-readiness-evidence.v1",
            "session_id": event["session_id"],
            "model_generation": event["model_generation"],
            "reset_generation": event["reset_generation"],
            "source_id": "robotsimue-visual",
            "basis": "truth_snapshot_applied_to_visual_bindings",
            "visual": {"state": "ACTIVE"},
            "sensors": {"camera_streams": "PREPARING", "overall": "PREPARING"},
            "streams": [],
        }
        (log_dir / "visual-readiness.json").write_text(
            json.dumps(evidence), encoding="utf-8"
        )
        (log_dir / "visual-first-frame.png").write_bytes(PNG)
        capture_dir = log_dir / "replay-frames"
        capture_dir.mkdir(exist_ok=True)
        frame_index = min(len(self.published) - 1, 2)
        (capture_dir / f"frame_{frame_index:06d}.png").write_bytes(PNG)


class _FakeUnreal:
    def __init__(self, harness: _Harness) -> None:
        self._harness = harness

    @property
    def pid(self) -> int | None:
        return 43210 if self._harness.process_started else None

    def start(self, **kwargs: Any) -> None:
        self._harness.allocation = kwargs["allocation"]
        self._harness.process_started = True

    def poll(self) -> int | None:
        return self._harness.exit_code

    def terminate(self) -> None:
        self._harness.process_terminated = True


class _FakePublisher:
    def __init__(self, harness: _Harness) -> None:
        self._harness = harness

    def publish(self, event: dict[str, Any]) -> int:
        self._harness.published.append(event)
        self._harness.emit_visual_evidence(event)
        return 256

    def close(self) -> None:
        self._harness.publisher_closed = True


def _config(tmp_path: Path, bundle: Path, recording: Path) -> VisualReplayConfig:
    return VisualReplayConfig(
        bundle_dir=bundle,
        recording_dir=recording,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        unreal_editor=tmp_path / "UnrealEditor.exe",
        uproject=tmp_path / "RobotSimUE.uproject",
        run_id="visual-replay-01",
        snapshot_port=25401,
        pace=False,
        ready_timeout_s=1.0,
        screenshot_timeout_s=1.0,
        frame_timeout_s=1.0,
        minimum_frames=2,
        maximum_frames=4,
    )


def test_visual_replay_launches_only_unreal_and_commits_terminal_evidence(
    tmp_path: Path,
) -> None:
    bundle, recording, session_id = _bundle_and_recording(tmp_path)
    harness = _Harness()

    result = run_visual_replay(
        _config(tmp_path, bundle, recording),
        unreal_factory=harness.unreal_factory,
        publisher_factory=harness.publisher_factory,
    )

    assert result.status == "SUCCEEDED"
    assert result.session_id == session_id
    assert result.replay_frames_presented == 3
    assert result.physics_process_launched is False
    assert result.captured_frames >= 2
    assert result.first_frame_screenshot.read_bytes().startswith(b"\x89PNG")
    assert harness.process_started is True
    assert harness.process_terminated is True
    assert harness.publisher_closed is True
    assert [event["bodies"][0]["position_m"][0] for event in harness.published[-3:]] == [
        0.0,
        0.1,
        0.2,
    ]

    runtime = json.loads(result.runtime_manifest.read_text(encoding="utf-8"))
    assert runtime["state"] == "STOPPED"
    assert runtime["allocation"]["physics_pid"] is None
    assert runtime["replay"]["clock_authority"] == "recorded_mujoco"
    assert runtime["replay"]["source_run_id"] == "source-motion"

    committed = json.loads(result.result_path.read_text(encoding="utf-8"))
    assert committed["schema"] == "lingtu.sim.visual-replay-result.v1"
    assert committed["status"] == "SUCCEEDED"
    assert committed["physics_process_launched"] is False


def test_visual_replay_rejects_a_recording_from_another_session_before_allocation(
    tmp_path: Path,
) -> None:
    bundle, recording, _session_id = _bundle_and_recording(tmp_path)
    manifest_path = recording / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["session_id"] = "another-session"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(VisualReplayError, match="session_id"):
        run_visual_replay(
            _config(tmp_path, bundle, recording),
            unreal_factory=_Harness().unreal_factory,
            publisher_factory=_Harness().publisher_factory,
        )

    assert not (tmp_path / "runs" / "visual-replay-01").exists()


def test_visual_replay_records_failed_terminal_state_when_unreal_exits(
    tmp_path: Path,
) -> None:
    bundle, recording, _session_id = _bundle_and_recording(tmp_path)
    harness = _Harness(exit_code=7)

    with pytest.raises(VisualReplayError, match="exited"):
        run_visual_replay(
            _config(tmp_path, bundle, recording),
            unreal_factory=harness.unreal_factory,
            publisher_factory=harness.publisher_factory,
        )

    runtime_path = tmp_path / "runs" / "visual-replay-01" / "session.runtime.json"
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    assert runtime["state"] == "FAILED"
    assert runtime["allocation"]["physics_pid"] is None
    assert "exited" in runtime["failure_reason"]
    assert harness.process_terminated is True
    assert harness.publisher_closed is True
