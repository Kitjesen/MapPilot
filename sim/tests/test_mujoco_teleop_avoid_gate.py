from __future__ import annotations

import json
from pathlib import Path

from sim.scripts.mujoco import teleop_avoid_gate


def test_mujoco_teleop_avoid_gate(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"

    rc = teleop_avoid_gate.main(
        [
            "--artifact-dir",
            str(tmp_path),
            "--json-out",
            str(report_path),
            "--strict",
        ]
    )

    assert rc == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["ok"] is True
    cases = {case["case"]: case for case in report["cases"]}
    assert cases["no_obstacle"]["first_decision"]["reason"] == "accepted"
    assert cases["no_obstacle"]["final_pose"]["x"] > 0.30
    assert cases["slow_obstacle"]["first_decision"]["reason"] == "obstacle_slow"
    assert cases["slow_obstacle"]["first_decision"]["cmd"]["vx"] < 0.10
    assert cases["stop_obstacle"]["first_decision"]["reason"] == "obstacle_stop"
    assert cases["stop_obstacle"]["final_pose"]["x"] <= 0.01
    assert cases["side_obstacle"]["first_decision"]["reason"] == "accepted"
    assert cases["low_obstacle"]["first_decision"]["reason"] == "accepted"
    assert cases["terrain_hard"]["first_decision"]["reason"] == "terrain_stop"
