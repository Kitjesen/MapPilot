from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from sim.scripts import pct_runtime_preflight

pytestmark = [pytest.mark.sim]
REPO_ROOT = Path(__file__).resolve().parents[2]


def test_pct_runtime_preflight_reports_selected_planner_runtime(monkeypatch):
    monkeypatch.setattr(
        pct_runtime_preflight,
        "inspect_pct_runtime",
        lambda repo_root, machine=None: {
            "ok": False,
            "planner_runtime": {
                "requested": "rust_process",
                "resolved": "rust_process",
                "supported": True,
                "error": "",
            },
            "runtime": {
                "ok": False,
                "binary": "",
                "searched": ["pct/runtime/rust/gpmp_optimize"],
                "error": "Rust PCT planner process is unavailable",
            },
            "recommended_build_command": "cargo build --release --bin gpmp_optimize",
        },
    )

    report = pct_runtime_preflight.build_report()

    assert report["schema_version"] == "lingtu.pct_runtime_preflight.v1"
    assert report["ok"] is False
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["pct_planner_runtime"]["runtime"] == "rust_process"
    assert report["pct_planner_runtime_ok"] is False
    assert set(report["checks"]) == {"pct_planner_runtime"}
    assert report["checks"]["pct_planner_runtime"]["ok"] is False
    assert report["blockers"] == ["PCT planner runtime unavailable"]
    assert report["claim_boundary"] == "environment_blocked_no_algorithm_claim"
    assert any("cargo build" in item for item in report["recommended_setup_commands"])
    assert "native_runtime" not in report
    assert "runtime_fingerprint" not in report


def test_pct_runtime_preflight_cli_writes_json_without_importing_numpy(tmp_path: Path):
    report_path = tmp_path / "pct_runtime_preflight.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/pct_runtime_preflight.py"),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=20,
    )

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema_version"] == "lingtu.pct_runtime_preflight.v1"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["pct_planner_runtime"]["runtime"] == "rust_process"
    assert report["pct_planner_runtime_ok"] is report["pct_planner_runtime"]["ok"]
    assert report["execution_mode"] == "pct_runtime_preflight"
    assert probe.returncode == (0 if report["ok"] else 1)
