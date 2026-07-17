from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

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


def test_tracking_gate_defaults_to_policy_drive_mode() -> None:
    args = gate.build_parser().parse_args([])

    assert args.drive_mode == "policy"
    assert args.policy_path == ""


def test_tracking_gate_allows_explicit_kinematic_smoke_mode() -> None:
    args = gate.build_parser().parse_args(["--drive-mode", "kinematic"])

    assert args.drive_mode == "kinematic"


def test_multilevel_scene_traversability_grid_marks_blocked_stairs_as_hard_risk() -> None:
    payload = gate._make_scene_traversability_grid("stair_blocked_30cm")

    assert payload["ok"] is True
    assert payload["source"] == "builtin_stair_blocked_30cm_traversability_grid"
    assert float(payload["grid"].min()) <= 95.0
    assert (payload["grid"] >= 92.0).any()


def test_multilevel_scene_traversability_grid_marks_easy_stairs_as_soft_cost() -> None:
    payload = gate._make_scene_traversability_grid("stair_easy")

    assert payload["ok"] is True
    assert payload["source"] == "builtin_stair_easy_traversability_grid"
    assert float(payload["grid"].min()) == 0.0
    assert ((payload["grid"] > 20.0) & (payload["grid"] < 70.0)).any()


def test_product_acceptance_rejects_ground_support_bypass() -> None:
    args = SimpleNamespace(
        map_source="mujoco_lidar",
        drive_mode="policy",
        planner_no_ground_support=True,
        acceptance_radius_m=0.35,
    )
    report = {
        "uses_ros": False,
        "policy_loaded": True,
        "artifacts": {
            "map_pcd": "map.pcd",
            "map_dir": "map",
            "scene_xml": "scene.xml",
            "trajectory_csv": "trajectory.csv",
            "preview_svg": "preview.svg",
        },
        "plan": {
            "path_count": 3,
            "diagnostics": {"constraints": {"require_ground_support": False}},
        },
        "tracking": {
            "arrived": True,
            "final_error_m": 0.1,
            "nonzero_cmd_count": 5,
        },
    }

    blockers = gate._product_acceptance_blockers(args, report)

    assert "product acceptance forbids planner-no-ground-support" in blockers
    assert "product acceptance requires OctoPlanner3D ground support" in blockers


def test_tracking_gate_appends_requested_goal_to_execution_path() -> None:
    path, source = gate._append_requested_terminal_goal(
        [[0.0, 0.0, 0.3], [0.9, -0.1, 0.3]],
        {"source": "saved_map_plan_gate_report"},
        [1.0, 0.0, 0.0],
    )

    assert source["terminal_goal_appended"] is True
    assert source["terminal_goal_xy"] == [1.0, 0.0]
    assert path[-1] == [1.0, 0.0, 0.3]


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
