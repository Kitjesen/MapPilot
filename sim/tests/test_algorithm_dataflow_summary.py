from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import json
from pathlib import Path

from sim.scripts.algorithm_dataflow_summary import resolve_report_path, summarize_report


def _live_report() -> dict:
    return {
        "ok": False,
        "outputs": {
            "fastlio2_cloud_registered": 201,
            "fastlio2_cloud_map": 201,
            "fastlio2_odometry": 229,
            "nav_odometry": 229,
            "nav_map_cloud": 201,
            "nav_registered_cloud": 201,
            "nav_cmd_vel": 231,
            "nav_cmd_vel_nonzero": 231,
        },
        "lingtu_inspection": {
            "goal_count": 3,
            "successful_navigation_goal_count": 1,
            "min_required_checkpoints": 3,
            "global_path_count": 2,
            "local_path_count": 225,
            "patrol_state": "PATROLLING",
        },
        "fastlio2_motion_consistency": {"ok": True},
        "fastlio2_z_consistency": {"ok": True},
        "fastlio2_yaw_consistency": {"ok": False, "yaw_delta_error_rad": 0.887},
        "remaining_gaps": ["Fast-LIO odometry yaw drift exceeded limit"],
    }


def test_dataflow_summary_distinguishes_live_flow_from_yaw_blocker():
    summary = summarize_report(_live_report())

    assert summary["ok"] is False
    assert summary["primary_blocker"] == "fastlio_yaw_consistency"
    flow = {edge["id"]: edge["ok"] for edge in summary["flow"]}
    assert flow["raw_lidar_to_fastlio"] is True
    assert flow["fastlio_odometry_to_navigation"] is True
    assert flow["global_planner_to_local_planner"] is True
    assert flow["path_follower_to_cmd_vel"] is True
    assert flow["fastlio_yaw_consistency"] is False


def test_dataflow_summary_passes_when_all_core_edges_pass():
    report = _live_report()
    report["ok"] = True
    report["lingtu_inspection"]["successful_navigation_goal_count"] = 3
    report["fastlio2_yaw_consistency"] = {"ok": True}

    summary = summarize_report(report)

    assert summary["ok"] is True
    assert summary["primary_blocker"] == ""


def test_resolve_report_path_accepts_latest_txt(tmp_path: Path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "report.json").write_text(json.dumps(_live_report()), encoding="utf-8")
    latest = tmp_path / "latest.txt"
    latest.write_text(f"latest_run_dir={run_dir}\n", encoding="utf-8")

    assert resolve_report_path(str(latest)) == run_dir / "report.json"
