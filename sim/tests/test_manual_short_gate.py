"""Offline tests for the one-shot manual SDK-quiet short gate."""

# ruff: noqa: S101

from __future__ import annotations

import json
import subprocess
import time
from functools import partial
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.runtime.coordinator import playable_vertical_slice as playable_module
from sim.runtime.coordinator.manual_short_gate import (
    ManualShortGateConfig,
    ManualShortGateError,
    supervise_manual_short_gate,
)

RUN_ID = "manual-20260811-200000-aabbccdd"
SESSION_ID = "manual-short-session"


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _ready_runtime(run_dir: Path) -> dict[str, Any]:
    streams = {
        sensor_id: {
            "stream_id": sensor_id,
            "stream_kind": kind,
            "state": "ACTIVE",
            "owner": "visual",
            "source": "unreal_camera",
            "transport": "camera_shm",
            "model_generation": 0,
            "reset_generation": 0,
            "session_id": SESSION_ID,
            "sample_count": 3,
        }
        for sensor_id, kind in (
            ("thunder_01.front_rgb", "rgb"),
            ("thunder_01.front_depth", "depth"),
        )
    }
    return {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "state": "RUNNING",
        "model_generation": 0,
        "reset_generation": 0,
        "allocation": {"run_dir": str(run_dir)},
        "sensor_streams": {
            "is_ready": True,
            "model_generation": 0,
            "reset_generation": 0,
            "summary": {
                "schema": "lingtu.sim.sensor-stream-summary.v1",
                "is_ready": True,
                "session_id": SESSION_ID,
                "model_generation": 0,
                "reset_generation": 0,
                "streams": streams,
            },
        },
    }


def _write_ready_artifacts(run_dir: Path) -> None:
    _write_json(run_dir / "session.runtime.json", _ready_runtime(run_dir))
    _write_json(
        run_dir / "manual-runtime-health.json",
        {
            "schema": "lingtu.sim.manual-runtime-health.v1",
            "qualification": False,
            "evidence_class": "manual_diagnostic_only",
            "run_id": RUN_ID,
            "session_id": SESSION_ID,
            "session_state": "RUNNING",
            "owner_thread_alive": True,
            "monitor_sequence": 2,
            "non_running_transition_count": 0,
            "owner_thread_stop_count": 0,
            "model_generation": 0,
            "reset_generation": 0,
            "updated_unix_ns": time.time_ns(),
        },
    )
    _write_json(
        run_dir / "logs/sensor-readiness.json",
        {
            "schema": "lingtu.sim.sensor-readiness-evidence.v1",
            "session_id": SESSION_ID,
            "model_generation": 0,
            "reset_generation": 0,
            "streams": [
                {
                    "sensor_id": "thunder_01.front_rgb",
                    "state": "ACTIVE",
                    "published_frames": 3,
                },
                {
                    "sensor_id": "thunder_01.front_depth",
                    "state": "ACTIVE",
                    "published_frames": 3,
                },
            ],
        },
    )
    (run_dir / "logs/Unreal.log").write_text(
        "LogLingTuSimUI: Display: LINGTU_RUNTIME_UI_ATTACHED modes=drive,build,tactical,menu\n",
        encoding="utf-8",
    )


class _Proof:
    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "lingtu.sim.owned-robotsimue-window-presence-proof.v1",
            "qualification": False,
            "evidence_class": "manual_diagnostic_only",
            "owned_unreal_pid": 4321,
            "candidates": [
                {
                    "hwnd": 99,
                    "pid": 4321,
                    "title": "RobotSimUE",
                    "title_redacted": False,
                    "visible": True,
                    "enabled": True,
                    "owner_hwnd": None,
                    "window_area": 100,
                    "client_area": 90,
                    "eligible": True,
                }
            ],
            "selected_hwnd": 99,
            "selected_pid": 4321,
            "observed_at_unix_ns": 123,
        }


class _DocumentProof:
    def __init__(self, document: dict[str, Any]) -> None:
        self.document = document

    def to_dict(self) -> dict[str, Any]:
        return self.document


def _config(run_dir: Path, probe_script: Path, **changes: Any) -> ManualShortGateConfig:
    values: dict[str, Any] = {
        "run_dir": run_dir,
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "unreal_pid": 4321,
        "profile_deadline_monotonic": 130.0,
        "performance_probe_script": probe_script,
        # Static fixtures do not refresh their heartbeat while the full Windows
        # suite is running.  Keep the explicit stale-heartbeat case meaningful
        # without making unrelated cases depend on host filesystem latency.
        "maximum_health_age_s": 600.0,
    }
    values.update(changes)
    return ManualShortGateConfig(**values)


def _write_probe_result(run_dir: Path, *, passed: bool) -> None:
    median_realtime_factor = 0.9 if passed else 0.5
    _write_json(
        run_dir / "runtime-performance-probe.json",
        {
            "schema": "lingtu.sim.runtime-performance-probe.v1",
            "qualification": False,
            "evidence_class": "manual_diagnostic_only",
            "passed": passed,
            "run_id": RUN_ID,
            "session_id": SESSION_ID,
            "run_dir": str(run_dir),
            "duration_seconds": 12.0,
            "session_state": "RUNNING",
            "model_generation": 0,
            "reset_generation": 0,
            "monitor_sequence_start": 1,
            "monitor_sequence_end": 2,
            "non_running_transition_count": 0,
            "owner_thread_stop_count": 0,
            "median_realtime_factor": median_realtime_factor,
            "minimum_median_realtime_factor": 0.8,
            "camera_rates_hz": {
                "thunder_01.front_rgb": 30.0,
                "thunder_01.front_depth": 30.0,
            },
            "minimum_camera_rate_hz": 29.0,
            "checks": {
                "continuous_running": True,
                "duration": True,
                "realtime_factor": passed,
                "camera_rates": True,
            },
        },
    )


class _AdvancingClock:
    def __init__(self, value: float = 100.0) -> None:
        self.value = value

    def __call__(self) -> float:
        self.value += 1.0
        return self.value


def test_ready_runtime_proves_presence_before_and_after_one_bound_12s_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe_runtime_performance.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[tuple[str, ...]] = []
    events: list[str] = []

    def prove_window(pid: int) -> _Proof:
        assert pid == 4321
        events.append("presence")
        return _Proof()

    def run_command(command: tuple[str, ...], timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        calls.append(command)
        assert timeout_s == 28.0
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="{}", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_window,
        command_runner=run_command,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is True
    assert result.failure_reason is None
    assert events == ["presence", "probe", "presence"]
    assert len(calls) == 1
    command = calls[0]
    assert command.count("-DurationSeconds") == 1
    assert command[command.index("-DurationSeconds") + 1] == "12"
    assert command[command.index("-RunDir") + 1] == str(run_dir)
    assert command[command.index("-ExpectedRunId") + 1] == RUN_ID
    assert command[command.index("-ProductSessionId") + 1] == SESSION_ID
    assert "60" not in command
    assert not any("probe_drive_input" in argument for argument in command)

    window = json.loads((run_dir / "manual-window-presence-proof.json").read_text())
    assert window["schema"] == "lingtu.sim.manual-window-presence-proof.v1"
    assert window["run_id"] == RUN_ID
    assert window["pre_probe"]["selected_hwnd"] == 99
    assert window["post_probe"]["selected_hwnd"] == 99
    assert "foreground_hwnd" not in json.dumps(window)
    assert "foreground_pid" not in json.dumps(window)
    summary = json.loads((run_dir / "manual-short-gate.json").read_text())
    assert summary["passed"] is True
    assert summary["probe_invocation_count"] == 1
    assert summary["qualification"] is False
    assert summary["evidence_class"] == "manual_diagnostic_only"
    assert summary["input_foreground_status"] == "not_evaluated"
    assert summary["input_ready"] is False
    assert summary["input_sent"] is False
    assert summary["long_probe_run"] is False
    assert "window_foreground_proof" not in summary["evidence"]


def test_post_probe_presence_failure_preserves_single_probe_and_fails_without_presence_artifact(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        if events.count("presence") == 2:
            raise RuntimeError("post-probe presence disappeared")
        return _Proof()

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_presence,
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert result.probe_invocation_count == 1
    assert result.window_proof_path is None
    assert result.performance_probe_path == run_dir / "runtime-performance-probe.json"
    assert events == ["presence", "probe", "presence"]
    summary = json.loads(result.summary_path.read_text())
    assert summary["window_presence_failure"]["phase"] == "window_presence_proof"
    assert summary["window_presence_failure"]["role"] == "primary"
    assert summary["input_foreground_status"] == "not_evaluated"
    assert summary["input_ready"] is False
    assert summary["input_sent"] is False
    assert not (run_dir / "manual-window-presence-proof.json").exists()


def test_post_probe_presence_identity_drift_fails_closed_without_retry(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _DocumentProof:
        events.append("presence")
        document = _Proof().to_dict()
        if events.count("presence") == 2:
            document["selected_hwnd"] = 100
            document["candidates"][0]["hwnd"] = 100
        return _DocumentProof(document)

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_presence,
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "presence identity changed" in str(result.failure_reason)
    assert result.probe_invocation_count == 1
    assert result.window_proof_path is None
    assert events == ["presence", "probe", "presence"]
    summary = json.loads(result.summary_path.read_text())
    assert summary["retry_count"] == 0
    assert summary["window_presence_failure"]["role"] == "primary"
    assert summary["input_ready"] is False
    assert summary["input_sent"] is False
    assert not (run_dir / "manual-window-presence-proof.json").exists()


def test_honest_failed_probe_remains_primary_when_post_presence_fails(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        if events.count("presence") == 2:
            raise RuntimeError("post-probe presence disappeared")
        return _Proof()

    def fail_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=prove_presence,
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert first.passed is False
    assert "performance probe did not pass" in str(first.failure_reason)
    assert first.performance_probe_path == run_dir / "runtime-performance-probe.json"
    assert first.window_proof_path is None
    assert events == ["presence", "probe", "presence"]
    diagnostic = json.loads(first.summary_path.read_text())["window_presence_failure"]
    assert diagnostic["role"] == "secondary_to_probe_failure"
    assert diagnostic["message"] == "post-probe presence disappeared"

    second = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            AssertionError("presence callback must not run on re-entry")
        ),
        command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
            AssertionError("probe callback must not run on re-entry")
        ),
    )
    assert second.passed is False
    assert second.failure_reason == first.failure_reason


@pytest.mark.parametrize(
    ("artifact_kind", "expected_failure"),
    [
        ("missing", "did not publish a readable artifact"),
        ("invalid", "schema is invalid"),
    ],
)
def test_invalid_probe_artifact_remains_primary_when_post_presence_fails(
    tmp_path: Path,
    artifact_kind: str,
    expected_failure: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        if events.count("presence") == 2:
            raise RuntimeError("post-probe presence unavailable")
        return _Proof()

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        if artifact_kind == "invalid":
            _write_probe_result(run_dir, passed=True)
            probe_path = run_dir / "runtime-performance-probe.json"
            payload = json.loads(probe_path.read_text())
            payload["schema"] = "invalid"
            _write_json(probe_path, payload)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_presence,
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert expected_failure in str(result.failure_reason)
    assert result.performance_probe_path is None
    assert result.window_proof_path is None
    assert events == ["presence", "probe", "presence"]
    diagnostic = json.loads(result.summary_path.read_text())["window_presence_failure"]
    assert diagnostic["role"] == "secondary_to_probe_failure"
    assert diagnostic["message"] == "post-probe presence unavailable"


def test_post_probe_runtime_generation_drift_rejects_presence_artifact(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=True)
        runtime_path = run_dir / "session.runtime.json"
        runtime = json.loads(runtime_path.read_text())
        runtime["model_generation"] = 1
        runtime["reset_generation"] = 1
        runtime["sensor_streams"]["model_generation"] = 1
        runtime["sensor_streams"]["reset_generation"] = 1
        summary = runtime["sensor_streams"]["summary"]
        summary["model_generation"] = 1
        summary["reset_generation"] = 1
        for stream in summary["streams"].values():
            stream["model_generation"] = 1
            stream["reset_generation"] = 1
        _write_json(runtime_path, runtime)
        health_path = run_dir / "manual-runtime-health.json"
        health = json.loads(health_path.read_text())
        health["model_generation"] = 1
        health["reset_generation"] = 1
        health["updated_unix_ns"] = time.time_ns()
        _write_json(health_path, health)
        sensors_path = run_dir / "logs/sensor-readiness.json"
        sensors = json.loads(sensors_path.read_text())
        sensors["model_generation"] = 1
        sensors["reset_generation"] = 1
        _write_json(sensors_path, sensors)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: _Proof(),
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "runtime generation changed" in str(result.failure_reason)
    assert result.probe_invocation_count == 1
    assert result.window_proof_path is None
    assert not (run_dir / "manual-window-presence-proof.json").exists()


def test_identity_mismatch_fails_before_window_or_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    runtime = json.loads((run_dir / "session.runtime.json").read_text())
    runtime["run_id"] = "manual-substituted"
    _write_json(run_dir / "session.runtime.json", runtime)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "run_id identity mismatch" in str(result.failure_reason)
    assert result.probe_invocation_count == 0
    assert calls == []
    summary = json.loads(result.summary_path.read_text())
    assert summary["evidence"]["window_presence_proof"]["present"] is False
    assert summary["evidence"]["runtime_performance_probe"]["present"] is False


@pytest.mark.parametrize(
    "marker",
    [
        "ValidatePlatforms",
        "VerifySdk",
        "BuildCookRun",
        "RunUAT",
        "AutomationTool",
        "UnrealBuildTool",
        "Build.bat",
        "MSBuild.exe",
        "cl.exe",
        "link.exe",
        "CookCommandlet",
        "-run=Cook",
    ],
)
def test_forbidden_sdk_or_build_marker_prevents_window_and_probe(
    tmp_path: Path,
    marker: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    with (run_dir / "logs/Unreal.log").open("a", encoding="utf-8") as stream:
        stream.write(f"forbidden startup: {marker}\n")
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert marker in str(result.failure_reason)
    assert result.probe_invocation_count == 0
    assert calls == []


def test_window_presence_failure_does_not_invoke_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    probe_calls: list[tuple[str, ...]] = []

    def reject_window(_pid: int) -> object:
        raise RuntimeError("RobotSimUE game window is unavailable")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=reject_window,
        command_runner=lambda command, _timeout: probe_calls.append(command),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "window presence proof dependency failed closed" in str(result.failure_reason)
    assert "game window is unavailable" not in str(result.failure_reason)
    assert result.probe_invocation_count == 0
    assert probe_calls == []
    summary = json.loads(result.summary_path.read_text())
    assert summary["window_presence_failure"] == {
        "phase": "window_presence_proof",
        "role": "primary",
        "exception_type": "RuntimeError",
        "message": "RobotSimUE game window is unavailable",
        "cause_type": None,
        "cause_message": None,
        "cause_chain": [],
        "cause_chain_truncated": False,
    }
    missing = summary["evidence"]["runtime_performance_probe"]
    assert missing["present"] is False
    assert "not invoked" in missing["missing_reason"]


def test_public_window_presence_never_focuses_and_runs_exactly_twice(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    probe_calls: list[tuple[str, ...]] = []
    presence_calls = 0

    class FocusDeniedBackend:
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert title_fragment == "RobotSimUE"
            return (
                playable_module._TopLevelWindowSnapshot(
                    hwnd=7001,
                    pid=pid,
                    title="RobotSimUE (64 bit Development PCD3D_SM5)",
                    visible=True,
                    enabled=True,
                    owner_hwnd=None,
                    window_area=2_000_000,
                    client_area=1_900_000,
                ),
            )

        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("performance presence proof must not focus")

        def foreground_window(self) -> int | None:
            raise AssertionError("performance presence proof must not query foreground")

        def window_process_id(self, _hwnd: int) -> int | None:
            return 4321

        def send_key(self, _key: str, *, pressed: bool) -> None:
            raise AssertionError(f"unexpected SendInput pressed={pressed}")

    backend = FocusDeniedBackend()

    def prove_presence(pid: int) -> object:
        nonlocal presence_calls
        presence_calls += 1
        return playable_module.prove_owned_robotsimue_window_presence(
            pid,
            backend=backend,
            sleep=lambda _seconds: None,
            monotonic=lambda: 0.0,
            timeout_s=0.5,
        )

    def run_probe(command: tuple[str, ...], _timeout: float) -> SimpleNamespace:
        probe_calls.append(command)
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_presence,
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is True
    assert result.probe_invocation_count == 1
    assert len(probe_calls) == 1
    assert presence_calls == 2
    summary = json.loads(result.summary_path.read_text())
    assert summary["window_presence_failure"] is None
    assert summary["input_foreground_status"] == "not_evaluated"
    assert summary["input_ready"] is False


def test_public_window_rejection_diagnostic_aggregates_without_title_or_input(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    probe_calls: list[tuple[str, ...]] = []
    sensitive_title = r"D:\private\RobotSimUE\UnrealEditor.exe"

    class RejectedWindowBackend:
        def top_level_windows(
            self,
            pid: int,
            *,
            title_fragment: str,
        ) -> tuple[Any, ...]:
            assert title_fragment == "RobotSimUE"
            return (
                playable_module._TopLevelWindowSnapshot(
                    hwnd=7001,
                    pid=pid,
                    title=sensitive_title,
                    visible=True,
                    enabled=True,
                    owner_hwnd=None,
                    window_area=2_000_000,
                    client_area=1_900_000,
                ),
            )

        def focus_window(self, _hwnd: int) -> None:
            raise AssertionError("ineligible window must not be focused")

        def foreground_window(self) -> int | None:
            return None

        def window_process_id(self, _hwnd: int) -> int | None:
            return 4321

        def send_key(self, _key: str, *, pressed: bool) -> None:
            raise AssertionError(f"unexpected SendInput pressed={pressed}")

    proof_clock = iter((0.0, 1.0))
    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=partial(
            playable_module.prove_owned_robotsimue_window_presence,
            backend=RejectedWindowBackend(),
            sleep=lambda _seconds: None,
            monotonic=lambda: next(proof_clock),
            timeout_s=0.5,
        ),
        command_runner=lambda command, _timeout: probe_calls.append(command),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    diagnostic = json.loads(result.summary_path.read_text())["window_presence_failure"]
    assert result.probe_invocation_count == 0
    assert probe_calls == []
    assert "observed_candidates=1" in diagnostic["message"]
    assert "title_path_or_executable=1" in diagnostic["message"]
    assert "title_mismatch=1" in diagnostic["message"]
    assert sensitive_title not in json.dumps(diagnostic)


def test_window_proof_diagnostic_is_single_line_and_bounded(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def reject_window(_pid: int) -> object:
        cause = OSError("cause\n\x00" + "c" * 1_000)
        try:
            raise cause
        except OSError as exc:
            raise RuntimeError("outer\r\n\t" + "m" * 1_000) from exc

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=reject_window,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    diagnostic = json.loads(result.summary_path.read_text())["window_presence_failure"]
    assert diagnostic["exception_type"] == "RuntimeError"
    assert diagnostic["cause_type"] == "OSError"
    assert "\n" not in diagnostic["message"]
    assert "\r" not in diagnostic["message"]
    assert "\n" not in diagnostic["cause_message"]
    assert "\x00" not in diagnostic["cause_message"]
    assert len(diagnostic["message"]) <= 512
    assert len(diagnostic["cause_message"]) <= 512


def test_window_proof_diagnostic_preserves_bounded_nested_cause_chain(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def reject_window(_pid: int) -> object:
        try:
            raise OSError(5, "injected Win32 access denial")
        except OSError as native_error:
            try:
                raise playable_module.PlayableLifecycleError(
                    "failed to focus RobotSimUE window"
                ) from native_error
            except playable_module.PlayableLifecycleError as focus_error:
                raise playable_module.PlayableLifecycleError(
                    "timed out focusing owned RobotSimUE game window"
                ) from focus_error

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=reject_window,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.probe_invocation_count == 0
    diagnostic = json.loads(result.summary_path.read_text())["window_presence_failure"]
    assert diagnostic["cause_chain"] == [
        {
            "exception_type": "PlayableLifecycleError",
            "message": "failed to focus RobotSimUE window",
        },
        {
            "exception_type": "OSError",
            "message": "[Errno 5] injected Win32 access denial",
        },
    ]
    assert diagnostic["cause_chain_truncated"] is False


def test_low_realtime_failed_probe_runs_once_and_second_supervision_is_read_only(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[tuple[str, ...]] = []

    def fail_probe(command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        calls.append(command)
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="{}", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    second = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            AssertionError("window proof must not repeat")
        ),
        command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
            AssertionError("probe must not repeat")
        ),
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert first.passed is False
    assert second.passed is False
    assert first.probe_invocation_count == second.probe_invocation_count == 1
    assert len(calls) == 1
    probe = json.loads((run_dir / "runtime-performance-probe.json").read_text())
    assert probe["passed"] is False
    assert probe["median_realtime_factor"] == 0.5
    assert probe["checks"]["realtime_factor"] is False
    summary = json.loads(first.summary_path.read_text())
    assert summary["retry_count"] == 0
    assert summary["launcher_lifecycle_action"] == (
        "return_and_wait_for_natural_profile_deadline"
    )


def test_existing_failed_summary_rejects_presence_artifact_tamper_without_callbacks(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def fail_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.window_proof_path is not None
    artifact = json.loads(first.window_proof_path.read_text())
    artifact["post_probe"]["selected_hwnd"] = 100
    _write_json(first.window_proof_path, artifact)

    with pytest.raises(ManualShortGateError, match="presence artifact"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("presence callback must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe callback must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_existing_failed_summary_rejects_presence_generation_tamper_without_callbacks(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def fail_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.window_proof_path is not None
    artifact = json.loads(first.window_proof_path.read_text())
    artifact["model_generation"] = 1
    artifact["reset_generation"] = 1
    _write_json(first.window_proof_path, artifact)

    with pytest.raises(ManualShortGateError, match="generation"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("presence callback must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe callback must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_existing_failed_summary_rejects_probe_artifact_tamper_without_callbacks(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def fail_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.performance_probe_path is not None
    probe = json.loads(first.performance_probe_path.read_text())
    probe["schema"] = "tampered"
    _write_json(first.performance_probe_path, probe)

    with pytest.raises(ManualShortGateError, match="probe schema"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("presence callback must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe callback must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_existing_failed_summary_rejects_probe_file_when_invocation_count_is_zero(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            RuntimeError("expected pre-probe presence failure")
        ),
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.probe_invocation_count == 0
    _write_probe_result(run_dir, passed=False)

    with pytest.raises(ManualShortGateError, match="invocation count is zero"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("presence callback must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe callback must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_existing_failed_summary_rejects_missing_probe_after_one_invocation(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def fail_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=False)
        return SimpleNamespace(returncode=1, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=fail_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.performance_probe_path is not None
    first.performance_probe_path.unlink()

    with pytest.raises(ManualShortGateError, match="probe artifact is missing"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("presence callback must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe callback must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_existing_passing_summary_cannot_bypass_live_proof_or_probe(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def pass_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: _Proof(),
        command_runner=pass_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert first.passed is True

    with pytest.raises(
        ManualShortGateError,
        match="cannot authorize re-entry",
    ):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("window proof must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("unreal_pid", 9999, "Unreal PID identity mismatch"),
        ("probe_duration_seconds", 13, "probe duration mismatch"),
    ],
)
def test_existing_failed_summary_rejects_identity_tamper_without_callbacks(
    tmp_path: Path,
    field: str,
    value: object,
    expected: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            RuntimeError("expected window failure")
        ),
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    summary = json.loads(first.summary_path.read_text())
    summary[field] = value
    _write_json(first.summary_path, summary)

    with pytest.raises(ManualShortGateError, match=expected):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("window proof must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


@pytest.mark.parametrize(
    "mutate",
    [
        lambda diagnostic: diagnostic.update(message="tampered\nmessage"),
        lambda diagnostic: diagnostic.update(role="secondary_to_probe_failure"),
        lambda diagnostic: diagnostic.update(unknown="field"),
        lambda diagnostic: diagnostic.update(cause_chain=[{"bad": "shape"}]),
    ],
)
def test_existing_window_failure_rejects_diagnostic_tamper_without_callbacks(
    tmp_path: Path,
    mutate: Any,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            RuntimeError("expected window failure")
        ),
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    summary = json.loads(first.summary_path.read_text())
    mutate(summary["window_presence_failure"])
    _write_json(first.summary_path, summary)

    with pytest.raises(ManualShortGateError, match="window presence"):
        supervise_manual_short_gate(
            config,
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                AssertionError("window proof must not run on re-entry")
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe must not run on re-entry")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )


def test_insufficient_trigger_budget_fails_before_window_and_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script, profile_deadline_monotonic=119.9),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "20-second trigger budget" in str(result.failure_reason)
    assert calls == []


def test_budget_is_rechecked_after_window_proof_before_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    clock = iter((100.0, 100.0, 117.0))
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: _Proof(),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: next(clock),
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "no longer has enough budget" in str(result.failure_reason)
    assert calls == []
    assert result.window_proof_path is None
    assert result.probe_invocation_count == 0


def test_readiness_waits_for_exactly_one_ui_marker_before_window(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    log_path = run_dir / "logs/Unreal.log"
    reads = 0
    events: list[str] = []

    def read_text(path: Path) -> str:
        nonlocal reads
        assert path == log_path
        reads += 1
        if reads == 1:
            return "LogInit: startup\n"
        return log_path.read_text(encoding="utf-8")

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: events.append("window") or _Proof(),
        command_runner=run_probe,
        read_text=read_text,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: events.append("wait"),
    )

    assert result.passed is True
    assert reads >= 3  # not-ready, ready, post-window, post-probe
    assert events[:3] == ["wait", "window", "probe"]


def test_readiness_waits_for_first_truth_generation_before_window(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    health_path = run_dir / "manual-runtime-health.json"
    health = json.loads(health_path.read_text())
    health["model_generation"] = None
    health["reset_generation"] = None
    _write_json(health_path, health)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def publish_first_generation(_seconds: float) -> None:
        events.append("wait")
        current = json.loads(health_path.read_text())
        current["model_generation"] = 0
        current["reset_generation"] = 0
        current["monitor_sequence"] += 1
        current["updated_unix_ns"] = time.time_ns()
        _write_json(health_path, current)

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: events.append("window") or _Proof(),
        command_runner=run_probe,
        monotonic=_AdvancingClock(),
        sleep=publish_first_generation,
    )

    assert result.passed is True
    assert events[:3] == ["wait", "window", "probe"]


def test_duplicate_ui_marker_fails_closed_without_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    log_path = run_dir / "logs/Unreal.log"
    log_path.write_text(log_path.read_text() * 2, encoding="utf-8")
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "exactly once" in str(result.failure_reason)
    assert calls == []


def test_probe_verdict_requires_a_real_boolean(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    def invalid_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=True)
        payload = json.loads((run_dir / "runtime-performance-probe.json").read_text())
        payload["passed"] = 1
        _write_json(run_dir / "runtime-performance-probe.json", payload)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: _Proof(),
        command_runner=invalid_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "verdict is invalid" in str(result.failure_reason)


def test_probe_timeout_uses_remaining_profile_budget_and_never_retries(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    timeouts: list[float] = []
    events: list[str] = []

    def timeout_probe(command: tuple[str, ...], timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        timeouts.append(timeout_s)
        raise subprocess.TimeoutExpired(command, timeout_s)

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        return _Proof()

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=prove_presence,
        command_runner=timeout_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert result.probe_invocation_count == 1
    assert events == ["presence", "probe", "presence"]
    assert timeouts == [28.0]
    assert "exceeded its profile deadline" in str(result.failure_reason)
    assert result.performance_probe_path is None
    summary = json.loads(result.summary_path.read_text())
    assert summary["retry_count"] == 0
    assert summary["evidence"]["runtime_performance_probe"]["present"] is False


def test_probe_timeout_keeps_post_presence_failure_secondary_and_reentry_read_only(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        if events.count("presence") == 2:
            raise RuntimeError("post-probe RobotSimUE window disappeared")
        return _Proof()

    def timeout_probe(command: tuple[str, ...], timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        _write_probe_result(run_dir, passed=False)
        raise subprocess.TimeoutExpired(command, timeout_s)

    config = _config(run_dir, probe_script)
    first = supervise_manual_short_gate(
        config,
        prove_presence=prove_presence,
        command_runner=timeout_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert first.passed is False
    assert first.probe_invocation_count == 1
    assert first.performance_probe_path == run_dir / "runtime-performance-probe.json"
    assert first.window_proof_path is None
    assert "exceeded its profile deadline" in str(first.failure_reason)
    assert events == ["presence", "probe", "presence"]
    summary = json.loads(first.summary_path.read_text())
    diagnostic = summary["window_presence_failure"]
    assert diagnostic["role"] == "secondary_to_probe_failure"
    assert diagnostic["message"] == "post-probe RobotSimUE window disappeared"

    second = supervise_manual_short_gate(
        config,
        prove_presence=lambda _pid: (_ for _ in ()).throw(
            AssertionError("presence callback must not run on re-entry")
        ),
        command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
            AssertionError("probe callback must not run on re-entry")
        ),
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )
    assert second.passed is False
    assert second.failure_reason == first.failure_reason
    assert second.performance_probe_path == first.performance_probe_path


def test_probe_keyboard_interrupt_runs_post_presence_then_propagates(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    events: list[str] = []

    def prove_presence(_pid: int) -> _Proof:
        events.append("presence")
        return _Proof()

    def interrupt_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        events.append("probe")
        raise KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        supervise_manual_short_gate(
            _config(run_dir, probe_script),
            prove_presence=prove_presence,
            command_runner=interrupt_probe,
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )

    assert events == ["presence", "probe", "presence"]
    assert not (run_dir / "manual-short-gate.json").exists()
    assert not (run_dir / "manual-window-presence-proof.json").exists()


def test_window_artifact_publish_does_not_overwrite_a_racing_file(
    tmp_path: Path,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    window_path = run_dir / "manual-window-presence-proof.json"
    racing_bytes = b'{"owner":"other"}\n'
    probe_calls: list[str] = []

    def race_window(_pid: int) -> _Proof:
        window_path.write_bytes(racing_bytes)
        return _Proof()

    def run_probe(_command: tuple[str, ...], _timeout: float) -> SimpleNamespace:
        probe_calls.append("probe")
        _write_probe_result(run_dir, passed=True)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=race_window,
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "artifact already exists" in str(result.failure_reason)
    assert window_path.read_bytes() == racing_bytes
    assert probe_calls == ["probe"]


def test_keyboard_interrupt_is_not_swallowed_as_a_gate_verdict(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")

    with pytest.raises(KeyboardInterrupt):
        supervise_manual_short_gate(
            _config(run_dir, probe_script),
            prove_presence=lambda _pid: (_ for _ in ()).throw(
                KeyboardInterrupt()
            ),
            command_runner=lambda _command, _timeout: (_ for _ in ()).throw(
                AssertionError("probe must not run")
            ),
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )

    assert not (run_dir / "manual-short-gate.json").exists()


@pytest.mark.parametrize(
    ("artifact", "field", "value", "expected"),
    [
        ("session.runtime.json", "session_id", "b" * 64, "session_id"),
        ("manual-runtime-health.json", "run_id", "manual-other", "run_id"),
        (
            "manual-runtime-health.json",
            "session_id",
            "b" * 64,
            "session_id",
        ),
        (
            "logs/sensor-readiness.json",
            "session_id",
            "b" * 64,
            "sensor readiness identity",
        ),
    ],
)
def test_each_runtime_evidence_source_is_identity_bound_before_window(
    tmp_path: Path,
    artifact: str,
    field: str,
    value: object,
    expected: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    path = run_dir / artifact
    payload = json.loads(path.read_text())
    payload[field] = value
    _write_json(path, payload)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=_AdvancingClock(),
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert expected in str(result.failure_reason)
    assert calls == []


def test_runtime_allocation_run_directory_is_identity_bound(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    other = (tmp_path / "other-run").resolve()
    other.mkdir()
    _write_ready_artifacts(run_dir)
    path = run_dir / "session.runtime.json"
    payload = json.loads(path.read_text())
    payload["allocation"]["run_dir"] = str(other)
    _write_json(path, payload)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=_AdvancingClock(),
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "allocation run_dir identity mismatch" in str(result.failure_reason)
    assert calls == []


@pytest.mark.parametrize(
    ("artifact", "mutate", "expected"),
    [
        (
            "session.runtime.json",
            lambda payload: payload.update(schema="wrong"),
            "runtime schema is invalid",
        ),
        (
            "session.runtime.json",
            lambda payload: payload.update(state="STOPPED"),
            "terminal state STOPPED",
        ),
        (
            "session.runtime.json",
            lambda payload: payload["sensor_streams"]["summary"]["streams"][
                "thunder_01.front_rgb"
            ].update(state="INACTIVE"),
            "not ACTIVE",
        ),
        (
            "session.runtime.json",
            lambda payload: payload["sensor_streams"]["summary"]["streams"][
                "thunder_01.front_depth"
            ].update(sample_count=0),
            "has no samples",
        ),
        (
            "logs/sensor-readiness.json",
            lambda payload: payload["streams"][0].update(state="INACTIVE"),
            "not ACTIVE",
        ),
        (
            "logs/sensor-readiness.json",
            lambda payload: payload["streams"][1].update(published_frames=0),
            "no rendered frames",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(schema="wrong"),
            "manual runtime health schema is invalid",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(session_state="PAUSED"),
            "not RUNNING",
        ),
        (
            "logs/sensor-readiness.json",
            lambda payload: payload.update(schema="wrong"),
            "sensor readiness schema is invalid",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(owner_thread_alive=False),
            "owner is not alive",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(non_running_transition_count=1),
            "non_running_transition_count is not zero",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(owner_thread_stop_count=1),
            "owner_thread_stop_count is not zero",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(model_generation=1),
            "generation mismatch",
        ),
        (
            "manual-runtime-health.json",
            lambda payload: payload.update(updated_unix_ns=1),
            "heartbeat is stale",
        ),
    ],
)
def test_stale_or_inactive_camera_and_health_evidence_never_reaches_window(
    tmp_path: Path,
    artifact: str,
    mutate: Any,
    expected: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    path = run_dir / artifact
    payload = json.loads(path.read_text())
    mutate(payload)
    _write_json(path, payload)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=_AdvancingClock(),
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert expected in str(result.failure_reason)
    assert calls == []


def test_missing_ui_marker_times_out_without_window_or_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    (run_dir / "logs/Unreal.log").write_text("LogInit: startup\n", encoding="utf-8")
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: calls.append("window"),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=_AdvancingClock(),
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "timed out" in str(result.failure_reason)
    assert "attach marker has not appeared" in str(result.failure_reason)
    assert calls == []


@pytest.mark.parametrize(
    ("mutate", "expected"),
    [
        (
            lambda document: document.update(schema="wrong"),
            "window presence proof schema is invalid",
        ),
        (
            lambda document: document.update(qualification=True),
            "window proof classification is invalid",
        ),
        (
            lambda document: document.update(owned_unreal_pid=9999),
            "PID identity mismatch",
        ),
        (
            lambda document: document.update(selected_hwnd=100),
            "exactly one eligible selected candidate",
        ),
        (
            lambda document: document.update(selected_pid=9999),
            "selected PID mismatch",
        ),
        (
            lambda document: document.update(candidates={"not": "an array"}),
            "candidates must be an array",
        ),
        (
            lambda document: document["candidates"].append(
                dict(document["candidates"][0])
            ),
            "candidates are not deduplicated",
        ),
        (
            lambda document: document["candidates"][0].update(visible=False),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(enabled=False),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(owner_hwnd=77),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(window_area=0),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(client_area=0),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(
                title=r"D:\private\RobotSimUE\UnrealEditor.exe"
            ),
            "candidate eligibility is inconsistent",
        ),
        (
            lambda document: document["candidates"][0].update(pid=9999),
            "candidate eligibility is inconsistent",
        ),
    ],
)
def test_invalid_window_proof_identity_or_shape_prevents_probe(
    tmp_path: Path,
    mutate: Any,
    expected: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    proof_document = _Proof().to_dict()
    mutate(proof_document)
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: _DocumentProof(proof_document),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert expected in str(result.failure_reason)
    assert calls == []


def test_window_proof_without_to_dict_prevents_probe(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls: list[str] = []

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: object(),
        command_runner=lambda _command, _timeout: calls.append("probe"),  # type: ignore[arg-type,return-value]
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert "must expose to_dict" in str(result.failure_reason)
    assert calls == []


@pytest.mark.parametrize(
    ("mutation", "returncode", "expected"),
    [
        ("missing", 0, "did not publish a readable artifact"),
        ("schema", 0, "schema is invalid"),
        ("classification", 0, "classification is invalid"),
        ("run_id", 0, "run_id identity mismatch"),
        ("session_id", 0, "session_id identity mismatch"),
        ("run_dir", 0, "run_dir identity mismatch"),
        ("generation", 0, "generation mismatch"),
        ("duration", 0, "checks contradict its metrics"),
        ("realtime", 0, "checks contradict its metrics"),
        ("camera_rate", 0, "checks contradict its metrics"),
        ("checks", 0, "checks contradict its metrics"),
        ("passed_with_nonzero", 1, "did not pass"),
    ],
)
def test_probe_artifact_and_exit_code_are_strictly_correlated(
    tmp_path: Path,
    mutation: str,
    returncode: int,
    expected: str,
) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    other = (tmp_path / "other-run").resolve()
    other.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    calls = 0

    def run_probe(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        nonlocal calls
        calls += 1
        if mutation != "missing":
            _write_probe_result(run_dir, passed=True)
            path = run_dir / "runtime-performance-probe.json"
            payload = json.loads(path.read_text())
            if mutation == "schema":
                payload["schema"] = "wrong"
            elif mutation == "classification":
                payload["qualification"] = True
            elif mutation == "run_id":
                payload["run_id"] = "manual-other"
            elif mutation == "session_id":
                payload["session_id"] = "b" * 64
            elif mutation == "run_dir":
                payload["run_dir"] = str(other)
            elif mutation == "generation":
                payload["reset_generation"] = 1
            elif mutation == "duration":
                payload["duration_seconds"] = 1.0
            elif mutation == "realtime":
                payload["median_realtime_factor"] = 0.79
            elif mutation == "camera_rate":
                payload["camera_rates_hz"]["thunder_01.front_depth"] = 28.9
            elif mutation == "checks":
                payload["checks"]["camera_rates"] = False
            _write_json(path, payload)
        return SimpleNamespace(returncode=returncode, stdout="", stderr="")

    result = supervise_manual_short_gate(
        _config(run_dir, probe_script),
        prove_presence=lambda _pid: _Proof(),
        command_runner=run_probe,
        monotonic=lambda: 100.0,
        sleep=lambda _seconds: None,
    )

    assert result.passed is False
    assert result.probe_invocation_count == 1
    assert calls == 1
    assert expected in str(result.failure_reason)


def test_summary_publish_race_is_create_new_and_never_overwrites(tmp_path: Path) -> None:
    run_dir = (tmp_path / RUN_ID).resolve()
    run_dir.mkdir()
    _write_ready_artifacts(run_dir)
    probe_script = (tmp_path / "probe.ps1").resolve()
    probe_script.write_text("# fake", encoding="utf-8")
    summary_path = run_dir / "manual-short-gate.json"
    racing_bytes = b'{"owner":"other"}\n'

    def race_summary(_command: tuple[str, ...], _timeout_s: float) -> SimpleNamespace:
        _write_probe_result(run_dir, passed=True)
        summary_path.write_bytes(racing_bytes)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    with pytest.raises(ManualShortGateError, match="artifact already exists"):
        supervise_manual_short_gate(
            _config(run_dir, probe_script),
            prove_presence=lambda _pid: _Proof(),
            command_runner=race_summary,
            monotonic=lambda: 100.0,
            sleep=lambda _seconds: None,
        )

    assert summary_path.read_bytes() == racing_bytes
