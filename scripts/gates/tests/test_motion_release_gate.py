from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from scripts.gates import motion_release_gate
from scripts.gates.motion_release_gate import GateStep, run_gate


def test_run_gate_records_successful_step_output(tmp_path: Path) -> None:
    report_path = tmp_path / "run" / "report.json"
    step = GateStep(
        name="smoke",
        command=[
            sys.executable,
            "-c",
            "import sys; print('ready'); print('quiet', file=sys.stderr)",
        ],
        timeout_s=5.0,
    )

    exit_code = run_gate([step], report_path=report_path, cwd=tmp_path)

    assert exit_code == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema_version"] == "lingtu.motion_release_gate.v1"
    assert report["ok"] is True
    assert len(report["steps"]) == 1
    recorded = report["steps"][0]
    assert recorded["name"] == "smoke"
    assert recorded["timeout_s"] == 5.0
    assert recorded["exit_code"] == 0
    assert recorded["status"] == "passed"
    assert Path(recorded["stdout_path"]).read_text(encoding="utf-8") == "ready\n"
    assert Path(recorded["stderr_path"]).read_text(encoding="utf-8") == "quiet\n"
    assert not list(report_path.parent.glob("*.tmp"))


def test_run_gate_reports_missing_command_and_returns_nonzero(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"
    step = GateStep(
        name="missing",
        command=[str(tmp_path / "definitely-not-a-command")],
        timeout_s=5.0,
    )

    exit_code = run_gate([step], report_path=report_path, cwd=tmp_path)

    assert exit_code != 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert report["steps"][0]["status"] == "missing"
    assert report["steps"][0]["exit_code"] is None
    assert "not found" in Path(report["steps"][0]["stderr_path"]).read_text(
        encoding="utf-8"
    ).lower()


def test_run_gate_reports_timeout_and_returns_nonzero(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"
    step = GateStep(
        name="timeout",
        command=[
            sys.executable,
            "-c",
            "import time; print('started', flush=True); time.sleep(60)",
        ],
        timeout_s=0.1,
    )

    exit_code = run_gate([step], report_path=report_path, cwd=tmp_path)

    assert exit_code != 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    recorded = report["steps"][0]
    assert recorded["status"] == "timeout"
    assert recorded["exit_code"] is None
    assert Path(recorded["stdout_path"]).is_file()
    assert "timed out" in Path(recorded["stderr_path"]).read_text(encoding="utf-8")


def test_run_gate_reports_failed_exit_code_and_returns_nonzero(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"
    step = GateStep(
        name="failure",
        command=[
            sys.executable,
            "-c",
            "import sys; print('broken', file=sys.stderr); raise SystemExit(7)",
        ],
        timeout_s=5.0,
    )

    exit_code = run_gate([step], report_path=report_path, cwd=tmp_path)

    assert exit_code != 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    recorded = report["steps"][0]
    assert recorded["status"] == "failed"
    assert recorded["exit_code"] == 7
    assert Path(recorded["stderr_path"]).read_text(encoding="utf-8") == "broken\n"


def test_report_write_is_atomic_and_preserves_previous_report(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "report.json"
    report_path.write_text('{"previous": true}\n', encoding="utf-8")
    step = GateStep(
        name="smoke",
        command=[sys.executable, "-c", "print('ok')"],
        timeout_s=5.0,
    )

    def reject_replace(source: str | Path, destination: str | Path) -> None:
        assert Path(destination) == report_path
        pending = json.loads(Path(source).read_text(encoding="utf-8"))
        assert pending["schema_version"] == "lingtu.motion_release_gate.v1"
        raise OSError("simulated replace failure")

    monkeypatch.setattr(motion_release_gate.os, "replace", reject_replace)

    with pytest.raises(OSError, match="simulated replace failure"):
        run_gate([step], report_path=report_path, cwd=tmp_path)

    assert json.loads(report_path.read_text(encoding="utf-8")) == {"previous": True}
    assert not list(tmp_path.glob(".report.json.*.tmp"))
