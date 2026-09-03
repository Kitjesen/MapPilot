# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

import sim.runtime.coordinator.atomic_file as atomic_file_module
from sim.runtime.recording import EpisodeRecorder, EpisodeResult, EpisodeStatus


def test_successful_episode_writes_complete_v1_result(tmp_path: Path) -> None:
    result = EpisodeResult(
        run_id="run-001",
        session_id="a" * 64,
        model_generation=3,
        reset_generation=1,
        start_sim_time_ns=100,
        end_sim_time_ns=900,
        status=EpisodeStatus.SUCCEEDED,
        failure_reason=None,
        artifact_references={
            "runtime_manifest": "session.runtime.json",
            "sensor_recording": "recordings/sensors.mcap",
        },
    )

    path = EpisodeRecorder(tmp_path).write(result)

    expected = {
        "schema": "lingtu.sim.episode-result.v1",
        "run_id": "run-001",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 1,
        "start_sim_time_ns": 100,
        "end_sim_time_ns": 900,
        "status": "SUCCEEDED",
        "failure_reason": None,
        "artifact_references": {
            "runtime_manifest": "session.runtime.json",
            "sensor_recording": "recordings/sensors.mcap",
        },
    }
    assert path == tmp_path / "episode_result.json"
    assert json.loads(path.read_text(encoding="utf-8")) == expected
    assert path.read_text(encoding="utf-8") == (
        json.dumps(
            expected,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )


def test_failed_episode_requires_and_writes_failure_reason(tmp_path: Path) -> None:
    common: dict[str, Any] = {
        "run_id": "run-failed",
        "session_id": "b" * 64,
        "model_generation": 4,
        "reset_generation": 2,
        "start_sim_time_ns": 50,
        "end_sim_time_ns": 75,
        "status": EpisodeStatus.FAILED,
        "artifact_references": {},
    }
    with pytest.raises(
        ValueError,
        match="FAILED episode requires a non-empty failure_reason",
    ):
        EpisodeResult(**common)

    result = EpisodeResult(
        **common,
        failure_reason="controller process exited with code 7",
    )
    path = EpisodeRecorder(tmp_path).write(result)

    assert json.loads(path.read_text(encoding="utf-8")) == {
        "schema": "lingtu.sim.episode-result.v1",
        "run_id": "run-failed",
        "session_id": "b" * 64,
        "model_generation": 4,
        "reset_generation": 2,
        "start_sim_time_ns": 50,
        "end_sim_time_ns": 75,
        "status": "FAILED",
        "failure_reason": "controller process exited with code 7",
        "artifact_references": {},
    }


def test_result_serialization_is_stable_after_artifact_input_mutates(
    tmp_path: Path,
) -> None:
    references = {
        "zeta": "evidence/zeta.json",
        "alpha": "evidence/alpha.json",
    }
    first = EpisodeResult(
        run_id="run-deterministic",
        session_id="c" * 64,
        model_generation=0,
        reset_generation=0,
        start_sim_time_ns=0,
        end_sim_time_ns=1,
        status=EpisodeStatus.SUCCEEDED,
        artifact_references=references,
    )
    second = EpisodeResult(
        run_id="run-deterministic",
        session_id="c" * 64,
        model_generation=0,
        reset_generation=0,
        start_sim_time_ns=0,
        end_sim_time_ns=1,
        status=EpisodeStatus.SUCCEEDED,
        artifact_references={
            "alpha": "evidence/alpha.json",
            "zeta": "evidence/zeta.json",
        },
    )
    references["late_mutation"] = "evidence/unstable.json"

    first_path = EpisodeRecorder(tmp_path / "first").write(first)
    second_path = EpisodeRecorder(tmp_path / "second").write(second)

    assert first_path.read_bytes() == second_path.read_bytes()
    assert "late_mutation" not in first_path.read_text(encoding="utf-8")


def test_episode_result_requires_non_empty_session_id() -> None:
    with pytest.raises(
        ValueError,
        match="session_id must be non-empty",
    ):
        EpisodeResult(
            run_id="run-invalid-digest",
            session_id=" ",
            model_generation=0,
            reset_generation=0,
            start_sim_time_ns=0,
            end_sim_time_ns=1,
            status=EpisodeStatus.SUCCEEDED,
        )


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"model_generation": -1}, "model_generation"),
        ({"model_generation": True}, "model_generation"),
        ({"reset_generation": -1}, "reset_generation"),
        ({"start_sim_time_ns": -1}, "start_sim_time_ns"),
        ({"end_sim_time_ns": -1}, "end_sim_time_ns"),
        (
            {"start_sim_time_ns": 2, "end_sim_time_ns": 1},
            "end_sim_time_ns must not precede start_sim_time_ns",
        ),
    ],
)
def test_episode_result_validates_generation_and_sim_time_schema(
    override: dict[str, object],
    message: str,
) -> None:
    values: dict[str, Any] = {
        "run_id": "run-invalid-stamp",
        "session_id": "d" * 64,
        "model_generation": 0,
        "reset_generation": 0,
        "start_sim_time_ns": 0,
        "end_sim_time_ns": 1,
        "status": EpisodeStatus.SUCCEEDED,
    }
    values.update(override)

    with pytest.raises(ValueError, match=message):
        EpisodeResult(**values)


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"run_id": ""}, "run_id"),
        ({"status": "SUCCEEDED"}, "status"),
        ({"artifact_references": []}, "artifact_references"),
        ({"artifact_references": {"": "evidence.json"}}, "artifact reference name"),
        ({"artifact_references": {"manifest": "  "}}, "artifact reference path"),
    ],
)
def test_episode_result_validates_identity_status_and_artifact_schema(
    override: dict[str, object],
    message: str,
) -> None:
    values: dict[str, Any] = {
        "run_id": "run-schema",
        "session_id": "e" * 64,
        "model_generation": 0,
        "reset_generation": 0,
        "start_sim_time_ns": 0,
        "end_sim_time_ns": 1,
        "status": EpisodeStatus.SUCCEEDED,
        "artifact_references": {},
    }
    values.update(override)

    with pytest.raises(ValueError, match=message):
        EpisodeResult(**values)


def test_atomic_write_keeps_previous_result_when_replace_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = EpisodeRecorder(tmp_path)
    previous = EpisodeResult(
        run_id="run-atomic",
        session_id="f" * 64,
        model_generation=1,
        reset_generation=0,
        start_sim_time_ns=0,
        end_sim_time_ns=10,
        status=EpisodeStatus.SUCCEEDED,
    )
    destination = recorder.write(previous)
    previous_bytes = destination.read_bytes()

    replacement = EpisodeResult(
        run_id="run-atomic",
        session_id="f" * 64,
        model_generation=1,
        reset_generation=1,
        start_sim_time_ns=0,
        end_sim_time_ns=5,
        status=EpisodeStatus.FAILED,
        failure_reason="simulated writer failure",
    )

    def fail_replace(_: object, __: object) -> None:
        raise OSError("replace unavailable")

    monkeypatch.setattr(
        atomic_file_module,
        "replace_file_with_retry",
        fail_replace,
    )
    with pytest.raises(OSError, match="replace unavailable"):
        recorder.write(replacement)

    assert destination.read_bytes() == previous_bytes
    assert list(tmp_path.glob(".episode_result.json.*.tmp")) == []


class _WindowsReplaceError(OSError):
    def __init__(self, winerror: int) -> None:
        super().__init__(f"Windows replace error {winerror}")
        self.winerror = winerror


def test_episode_recorder_retries_windows_replace_conflict(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_retry = atomic_file_module.replace_file_with_retry
    replace_calls = 0
    sleeps: list[float] = []

    def flaky_replace(source: Path, destination: Path) -> None:
        nonlocal replace_calls
        replace_calls += 1
        if replace_calls < 3:
            raise _WindowsReplaceError(32)
        source.replace(destination)

    def retry_episode_file(source: Path, destination: Path) -> None:
        real_retry(
            source,
            destination,
            attempts=3,
            delay_s=0.001,
            replace=flaky_replace,
            sleep=sleeps.append,
        )

    monkeypatch.setattr(
        atomic_file_module,
        "replace_file_with_retry",
        retry_episode_file,
    )
    result = EpisodeResult(
        run_id="run-windows-retry",
        session_id="1" * 64,
        model_generation=0,
        reset_generation=0,
        start_sim_time_ns=0,
        end_sim_time_ns=10,
        status=EpisodeStatus.SUCCEEDED,
    )

    path = EpisodeRecorder(tmp_path).write(result)

    assert path.is_file()
    assert replace_calls == 3
    assert sleeps == [0.001, 0.001]
