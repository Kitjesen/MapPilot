from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts.blocked_route_replan_gate import SCHEMA_VERSION
from sim.scripts.blocked_route_replan_gate import run_gate


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_blocked_route_replan_gate_writes_non_motion_replan_report(tmp_path: Path):
    pytest.importorskip("fastapi")

    report = run_gate(
        map_name="pytest_map",
        x=2.0,
        y=0.0,
        planner="pct",
        json_out=tmp_path / "blocked_route" / "report.json",
        client_id="pytest",
    )

    assert report["schema_version"] == SCHEMA_VERSION
    assert report["ok"] is True
    assert report["mode"] == "blocked_route_replan_non_motion"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["published"] == {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0}
    assert report["baseline_path_intersects_block"] is True
    assert report["candidate_path_avoids_block"] is True
    assert report["candidate_route_changed"] is True
    assert report["blocked_route_replanned"] is True
    assert report["phases"]["baseline"]["intersects_block"] is True
    assert report["phases"]["blocked_candidate"]["intersects_block"] is False
    assert report["phases"]["blocked_candidate"]["min_block_clearance_m"] >= 0.25


def test_blocked_route_replan_gate_cli_writes_report(tmp_path: Path):
    report_path = tmp_path / "report.json"

    probe = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "sim/scripts/blocked_route_replan_gate.py"),
            "--json-out",
            str(report_path),
            "--strict",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=30,
    )

    assert probe.returncode == 0, probe.stderr
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["candidate_path_avoids_block"] is True
