from __future__ import annotations

import subprocess
from types import SimpleNamespace


def test_large_terrain_records_pct_child_process_crash_without_numpy(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    assets = SimpleNamespace(metadata=tmp_path / "metadata.json")
    route = SimpleNamespace(
        name="terrain_short",
        start=(-9.5, -5.6, 0.0),
        goal=(-5.9, -4.2, 0.0),
    )

    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=-11,
            stdout="planner stdout",
            stderr="Segmentation fault (core dumped)",
        )

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    plan = mod._plan_with_backend_subprocess(
        "pct",
        assets,
        route,
        obstacle_thr=49.9,
        pct_planner_runtime={"runtime": "rust_process", "ok": True},
    )

    assert plan["feasible"] is False
    assert plan["pct_planner_runtime"] == {"runtime": "rust_process", "ok": True}
    assert plan["pct_planner_runtime_ok"] is True
    assert "native_runtime" not in plan
    assert "native_backend_used" not in plan
    assert plan["status"] == "failed"
    assert plan["failure_category"] == "planner_process_crash"
    assert plan["returncode"] == -11
    assert "signal 11" in plan["error"]
    assert "SIGSEGV" in plan["error"]
    assert "Segmentation fault" in plan["stderr_tail"]


def test_large_terrain_pct_child_disables_optimizer_by_default(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    assets = SimpleNamespace(metadata=tmp_path / "metadata.json")
    route = SimpleNamespace(
        name="terrain_short",
        start=(-9.5, -5.6, 0.0),
        goal=(-5.9, -4.2, 0.0),
    )
    captured_env = {}

    def fake_run(*args, **kwargs):
        captured_env.update(kwargs["env"])
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=-11,
            stdout="",
            stderr="Segmentation fault",
        )

    monkeypatch.delenv(mod.PCT_OPTIMIZE_TRAJECTORY_ENV, raising=False)
    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    plan = mod._plan_with_backend_subprocess(
        "pct",
        assets,
        route,
        obstacle_thr=49.9,
        pct_planner_runtime={"runtime": "rust_process", "ok": True},
    )

    assert captured_env[mod.PCT_OPTIMIZE_TRAJECTORY_ENV] == "0"
    assert plan["pct_optimizer_enabled"] is False
    assert plan["pct_planner_path_mode"] == "astar_raw_path"

def test_large_terrain_pct_child_preserves_explicit_optimizer_env(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    assets = SimpleNamespace(metadata=tmp_path / "metadata.json")
    route = SimpleNamespace(
        name="terrain_short",
        start=(-9.5, -5.6, 0.0),
        goal=(-5.9, -4.2, 0.0),
    )
    captured_env = {}

    def fake_run(*args, **kwargs):
        captured_env.update(kwargs["env"])
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=-11,
            stdout="",
            stderr="Segmentation fault",
        )

    monkeypatch.setenv(mod.PCT_OPTIMIZE_TRAJECTORY_ENV, "1")
    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    plan = mod._plan_with_backend_subprocess(
        "pct",
        assets,
        route,
        obstacle_thr=49.9,
        pct_planner_runtime={"runtime": "rust_process", "ok": True},
    )

    assert captured_env[mod.PCT_OPTIMIZE_TRAJECTORY_ENV] == "1"
    assert plan["pct_optimizer_enabled"] is True
    assert plan["pct_planner_path_mode"] == "optimized_trajectory"


def test_large_terrain_pct_mode_fields_prefer_backend_diagnostics():
    from sim.scripts import large_terrain_nav_validation as mod

    fields = mod._pct_planning_mode_fields(
        "pct",
        optimizer_enabled=True,
        planner_diagnostics={
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_optimizer_raw_blocked_sample_count": 0,
            "pct_planner_path_mode": "astar_raw_path",
        },
    )

    assert fields["pct_optimizer_enabled"] is True
    assert fields["pct_optimizer_attempted"] is True
    assert fields["pct_optimizer_accepted"] is False
    assert fields["pct_optimizer_reject_reason"] == "optimized_trajectory_hard_obstacle"
    assert fields["pct_optimizer_blocked_sample_count"] == 3
    assert fields["pct_optimizer_raw_blocked_sample_count"] == 0
    assert fields["pct_planner_path_mode"] == "astar_raw_path"

def test_large_terrain_pct_mode_fields_do_not_accept_legacy_path_mode_alias():
    from sim.scripts import large_terrain_nav_validation as mod

    fields = mod._pct_planning_mode_fields(
        "pct",
        optimizer_enabled=True,
        planner_diagnostics={"pct_planner_path_mode": "native_astar_raw_path"},
    )

    assert fields["pct_planner_path_mode"] == "optimized_trajectory"
