from __future__ import annotations

import json
from pathlib import Path

from sim.scripts.mujoco import saved_map_tracking_gate as gate


def test_select_target_index_uses_lookahead_and_cursor() -> None:
    path = [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.8, 0.0, 0.0], [1.4, 0.0, 0.0]]

    idx = gate._select_target_index(
        path,
        [0.0, 0.0, 0.0],
        cursor=0,
        lookahead_m=1.0,
        waypoint_reached_m=0.35,
    )

    assert idx == 3


def test_run_tracking_gate_reports_global_plan_blocker(monkeypatch, tmp_path: Path) -> None:
    def fake_run_gate(_args):
        return {
            "ok": False,
            "map_pcd": str(tmp_path / "missing.pcd"),
            "plan": {"ok": False, "path": [], "error": "test failure"},
        }

    monkeypatch.setattr(gate.plan_gate, "run_gate", fake_run_gate)
    args = gate.build_parser().parse_args(["--out-dir", str(tmp_path / "gate")])

    report = gate.run_tracking_gate(args)

    assert report["ok"] is False
    assert report["stage"] == "global_plan"
    assert "global_plan_failed_or_empty" in report["blockers"]
    written = json.loads((tmp_path / "gate" / "report.json").read_text(encoding="utf-8"))
    assert written["stage"] == "global_plan"


def test_write_preview_svg_contains_path_and_trajectory(tmp_path: Path) -> None:
    out = tmp_path / "preview.svg"

    gate._write_preview_svg(
        out,
        pcd_points=[[0.0, 0.0, 0.0], [1.0, 0.4, 0.0]],
        global_path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        trajectory=[[0.0, 0.0, 0.0], [0.8, 0.1, 0.0]],
        start=[0.0, 0.0, 0.0],
        goal=[1.0, 0.0, 0.0],
    )

    text = out.read_text(encoding="utf-8")
    assert "<svg" in text
    assert "global_path" in text
    assert "MuJoCo trajectory" in text
