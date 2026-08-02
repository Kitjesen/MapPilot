from __future__ import annotations

import pytest

from sim.scripts import large_terrain_nav_validation as gate

pytestmark = [pytest.mark.sim]


def test_large_terrain_preflight_writes_host_guard_without_importing_runtime(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    def fail_load_runtime() -> None:
        raise AssertionError("host guard should run before large-terrain runtime import")

    monkeypatch.setattr(gate, "_load_runtime", fail_load_runtime)
    monkeypatch.setattr(gate, "numpy_import_is_safe", lambda: False)
    monkeypatch.setattr(
        gate,
        "_pct_planner_runtime_evidence",
        lambda: {
            "runtime": "rust_process",
            "ok": False,
            "error": "Rust PCT planner process is unavailable",
        },
    )

    report = gate._environment_preflight_report(
        tmp_path,
        routes=("terrain_short",),
        planners=("pct", "astar"),
        platform_system="Windows",
    )

    assert report is not None
    assert report["ok"] is False
    assert report["execution_mode"] == "host_guard"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["environment"]["accepted_host"] is False
    assert report["environment"]["blocked_reason"] == "windows_mingw_numpy_not_accepted"
    assert report["environment"]["claim_boundary"] == "environment_blocked_no_algorithm_claim"
    assert report["pct_planner_runtime"]["runtime"] == "rust_process"
    assert report["pct_planner_runtime_ok"] is False
    assert "PCT planner runtime unavailable" in report["environment_blockers"]
    plan = report["cases"][0]["planning"][0]
    assert plan["failure_category"] == "environment_runtime"
    assert plan["pct_planner_runtime"]["runtime"] == "rust_process"
    assert plan["pct_planner_runtime_ok"] is False
    assert "native_runtime" not in report
    assert "native_backend_used" not in plan


def test_large_terrain_preflight_blocks_unavailable_pct_on_any_host(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(gate, "numpy_import_is_safe", lambda: True)
    monkeypatch.setattr(
        gate,
        "_pct_planner_runtime_evidence",
        lambda: {
            "runtime": "rust_process",
            "ok": False,
            "error": "Rust PCT planner process is unavailable",
        },
    )

    report = gate._environment_preflight_report(
        tmp_path,
        routes=("terrain_short",),
        planners=("pct", "astar"),
        platform_system="Linux",
    )

    assert report is not None
    assert report["environment"]["blocked_reason"] == "pct_planner_runtime_unavailable"
    assert report["environment_blockers"] == ["PCT planner runtime unavailable"]
    assert report["pct_planner_runtime_ok"] is False
    assert "accepted_platforms" not in report["environment"]

def test_large_terrain_preflight_accepts_ready_rust_runtime_on_windows(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(gate, "numpy_import_is_safe", lambda: True)
    monkeypatch.setattr(
        gate,
        "_pct_planner_runtime_evidence",
        lambda: {"runtime": "rust_process", "ok": True, "error": ""},
    )

    report = gate._environment_preflight_report(
        tmp_path,
        routes=("terrain_short",),
        planners=("pct",),
        platform_system="Windows",
    )

    assert report is None
