from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CLI_PATH = ROOT / "tools" / "datasets" / "capture_slam_status_jsonl.py"
CLI_SPEC = importlib.util.spec_from_file_location("capture_slam_status_jsonl", CLI_PATH)
assert CLI_SPEC is not None and CLI_SPEC.loader is not None
CLI = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(CLI)


class _FakeTime:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, duration_s: float) -> None:
        self.now_s += duration_s


def _snapshot(runtime_id: str, sequence: int, stamp_s: float) -> dict:
    return {
        "schema_version": "lingtu.slam.status_snapshot.v1",
        "runtime_instance_id": runtime_id,
        "observation_sequence": sequence,
        "stamp_s": stamp_s,
        "state": "TRACKING",
    }


def test_capture_deduplicates_frozen_snapshot_and_keeps_runtime_restart(tmp_path: Path):
    events = [
        CLI.SnapshotUnavailable("missing"),
        _snapshot("runtime-a", 10, 1.0),
        _snapshot("runtime-a", 10, 1.0),
        _snapshot("runtime-a", 11, 1.1),
        _snapshot("runtime-b", 0, 1.2),
    ]

    def read_snapshot() -> dict:
        event = events.pop(0)
        if isinstance(event, Exception):
            raise event
        return event

    output = tmp_path / "slam-status.jsonl"
    fake_time = _FakeTime()
    summary = CLI.capture_snapshot_history(
        tmp_path / "status.json",
        output,
        duration_s=0.4,
        poll_hz=10.0,
        min_samples=3,
        monotonic=fake_time.monotonic,
        sleeper=fake_time.sleep,
        reader=read_snapshot,
    )

    records = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert [(item["runtime_instance_id"], item["observation_sequence"]) for item in records] == [
        ("runtime-a", 10),
        ("runtime-a", 11),
        ("runtime-b", 0),
    ]
    assert summary["status"] == "COMPLETE"
    assert summary["unique_samples"] == 3
    assert summary["duplicate_reads"] == 1
    assert summary["unavailable_reads"] == {"missing": 1}
    assert summary["motion_authorization"] is False


def test_capture_is_incomplete_when_source_never_advances(tmp_path: Path):
    frozen = _snapshot("runtime-a", 10, 1.0)
    fake_time = _FakeTime()

    summary = CLI.capture_snapshot_history(
        tmp_path / "status.json",
        tmp_path / "history.jsonl",
        duration_s=0.2,
        poll_hz=10.0,
        min_samples=2,
        monotonic=fake_time.monotonic,
        sleeper=fake_time.sleep,
        reader=lambda: frozen,
    )

    assert summary["status"] == "INCOMPLETE"
    assert summary["unique_samples"] == 1
    assert summary["duplicate_reads"] == 2


def test_capture_is_incomplete_when_source_advances_then_freezes(tmp_path: Path):
    advancing = [
        _snapshot("runtime-a", 10, 1.0),
        _snapshot("runtime-a", 11, 1.1),
    ]
    frozen = advancing[-1]

    def read_snapshot() -> dict:
        return advancing.pop(0) if advancing else frozen

    fake_time = _FakeTime()
    summary = CLI.capture_snapshot_history(
        tmp_path / "status.json",
        tmp_path / "history.jsonl",
        duration_s=2.0,
        poll_hz=10.0,
        min_samples=2,
        monotonic=fake_time.monotonic,
        sleeper=fake_time.sleep,
        reader=read_snapshot,
    )

    assert summary["status"] == "INCOMPLETE"
    assert summary["unique_samples"] == 2
    assert summary["observed_duration_s"] == 0.1
    assert summary["min_observed_duration_s"] == 1.0


def test_capture_rejects_wrong_schema_without_calling_it_evidence(tmp_path: Path):
    fake_time = _FakeTime()

    try:
        CLI.capture_snapshot_history(
            tmp_path / "status.json",
            tmp_path / "history.jsonl",
            duration_s=0.1,
            poll_hz=10.0,
            min_samples=1,
            monotonic=fake_time.monotonic,
            sleeper=fake_time.sleep,
            reader=lambda: {"schema_version": "unexpected.v1"},
        )
    except CLI.SnapshotContractError as exc:
        assert "schema_version" in str(exc)
    else:
        raise AssertionError("wrong snapshot schema must fail capture")


def test_capture_refuses_to_overwrite_existing_evidence(tmp_path: Path):
    output = tmp_path / "history.jsonl"
    output.write_text("existing\n", encoding="utf-8")

    try:
        CLI.capture_snapshot_history(
            tmp_path / "status.json",
            output,
            duration_s=0.1,
            poll_hz=10.0,
            min_samples=1,
            reader=lambda: _snapshot("runtime-a", 1, 1.0),
        )
    except FileExistsError:
        pass
    else:
        raise AssertionError("existing evidence must require explicit overwrite")


def test_main_returns_incomplete_code_and_writes_summary(tmp_path: Path):
    source = tmp_path / "status.json"
    output = tmp_path / "history.jsonl"
    summary_path = tmp_path / "capture-summary.json"
    source.write_text(json.dumps(_snapshot("runtime-a", 1, 1.0)), encoding="utf-8")

    rc = CLI.main(
        [
            str(source),
            str(output),
            "--duration-s",
            "0",
            "--min-samples",
            "2",
            "--summary",
            str(summary_path),
        ]
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert rc == 2
    assert summary["status"] == "INCOMPLETE"
    assert summary["motion_authorization"] is False
