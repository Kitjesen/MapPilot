from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "bench" / "semantic_goal_preview.py"
SPEC = importlib.util.spec_from_file_location("semantic_goal_preview", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_admitted_requires_native_goal_reached_and_path() -> None:
    assert MODULE._admitted(0, {"ok": True, "reached_goal": True, "path": [[0, 0, 0]]}) == (True, "accepted")
    assert MODULE._admitted(0, {"ok": True, "reached_goal": False, "path": [[0, 0, 0]]}) == (
        False,
        "goal_not_reached",
    )
    assert MODULE._admitted(0, {"ok": True, "reached_goal": True, "path": []}) == (False, "empty_path")
    assert MODULE._admitted(2, {"ok": False, "error": "goal_snap_exhausted"}) == (False, "planner_exit_2")


def test_last_json_ignores_native_diagnostics() -> None:
    assert MODULE._last_json(b"planner log\n{\"ok\": true, \"path\": []}\n") == {
        "ok": True,
        "path": [],
    }


def test_preview_marks_bound_target_unreachable_when_all_candidates_fail() -> None:
    status, admitted, reasons = MODULE._summarize_results(
        [
            {"admitted": False, "reason": "planner_exit_2"},
            {"admitted": False, "reason": "goal_not_reached"},
        ]
    )
    assert status == "target_unreachable"
    assert admitted == 0
    assert reasons == {"planner_exit_2": 1, "goal_not_reached": 1}


def test_preview_keeps_candidate_admission_separate_from_motion() -> None:
    status, admitted, reasons = MODULE._summarize_results(
        [{"admitted": True, "reason": "accepted"}]
    )
    assert status == "candidate_admitted"
    assert admitted == 1
    assert reasons == {"accepted": 1}


def test_preview_reports_verifier_block_after_discrete_planner_admission() -> None:
    status, admitted, reasons = MODULE._summarize_results(
        [{"admitted": False, "planner_admitted": True, "reason": "verification_segment_sweep_failed"}]
    )
    assert status == "verifier_blocked"
    assert admitted == 0
    assert reasons == {"verification_segment_sweep_failed": 1}
