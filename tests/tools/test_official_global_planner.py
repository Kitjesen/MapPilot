from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "bench" / "official_global_planner.py"
SPEC = importlib.util.spec_from_file_location("official_global_planner", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _evaluate(*, returncode: int, result: object, expected: dict[str, object]) -> list[str]:
    return MODULE.evaluate_planner_result(
        returncode=returncode,
        result=result,
        expected=expected,
        start=[0.0, 0.0, 0.35],
        goal=[2.0, 0.0, 0.35],
        start_snap_tolerance_m=0.45,
        goal_tolerance_m=0.2,
    )


def test_accepts_a_finite_path_that_reaches_the_requested_goal() -> None:
    result = {
        "ok": True,
        "reached_goal": True,
        "path": [[0.2, 0.0, 0.35], [1.0, 0.0, 0.35], [1.9, 0.0, 0.35]],
        "diagnostics": {"goal_error_m": 0.1, "goal_xy_error_m": 0.1, "goal_z_error_m": 0.0},
    }

    assert _evaluate(returncode=0, result=result, expected={"ok": True}) == []


def test_rejects_ok_result_whose_path_does_not_reach_goal() -> None:
    result = {
        "ok": True,
        "reached_goal": True,
        "path": [[0.0, 0.0, 0.35], [1.0, 0.0, 0.35]],
        "diagnostics": {"goal_error_m": 0.0, "goal_xy_error_m": 0.0, "goal_z_error_m": 0.0},
    }

    errors = _evaluate(returncode=0, result=result, expected={"ok": True})

    assert any("last path point" in error for error in errors)
    assert any("reported goal_error_m" in error for error in errors)


def test_does_not_treat_a_crashed_process_as_an_expected_negative_plan() -> None:
    errors = _evaluate(
        returncode=139,
        result={"ok": False, "error": "empty_path", "path": []},
        expected={"ok": False, "error": "empty_path"},
    )

    assert errors == ["planner process failed with exit code 139"]


def test_rejects_negative_result_that_carries_a_stale_path() -> None:
    errors = _evaluate(
        returncode=2,
        result={
            "ok": False,
            "error": "goal_snap_exhausted",
            "path": [[0.0, 0.0, 0.35]],
            "reached_goal": True,
        },
        expected={"ok": False, "error": "goal_snap_exhausted"},
    )

    assert "negative plan returned a non-empty or invalid path" in errors
    assert "negative plan did not report reached_goal=false" in errors


def test_route_requires_full_sweep_and_metric_motion(monkeypatch, tmp_path):
    import json
    import subprocess

    diagnostic = {
        "route_valid": True,
        "route_reason": "ok",
        "sweep_valid": True,
        "motion_diagnostic_valid": False,
        "valid": False,
        "reason": "segment_motion_failed",
    }
    monkeypatch.setattr(
        MODULE, "_run", lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 2, json.dumps(diagnostic).encode(), b""
        )
    )
    arguments = dict(
        verifier=tmp_path / "verify", map_path=tmp_path / "map.ot",
        xyz_path=tmp_path / "path.xyz", options={}, output_dir=tmp_path, timeout_s=1,
    )
    assert "motion_diagnostic_valid=true" in MODULE._verify_path(**arguments)[0]
    saved = json.loads((tmp_path / "verifier.stdout.json").read_bytes())
    assert saved["valid"] is False
    assert saved["reason"] == "segment_motion_failed"

    diagnostic.update(motion_diagnostic_valid=True, valid=True, reason="ok")
    assert MODULE._verify_path(**arguments) == []

    diagnostic.update(sweep_valid=False, sweep_failures=[{
        "segment_index": 32, "reason": "occupied_body", "at": [3.05, 0.25, 2.85],
    }])
    errors = MODULE._verify_path(**arguments)
    assert "sweep_valid=true" in errors[0]
    assert "occupied_body" in errors[0]
    assert "32" in errors[0]

    diagnostic.update(route_valid=False, route_reason="non_adjacent_route")
    assert "non_adjacent_route" in MODULE._verify_path(**arguments)[0]

    monkeypatch.setattr(
        MODULE, "_run", lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 139, b"", b"crashed"
        )
    )
    assert "exit code 139" in MODULE._verify_path(**arguments)[0]
