"""Point coverage follows native completion and skip facts."""

from __future__ import annotations

import pytest

from gateway.schemas import InspectionTaskReportResponse
from gateway.services.inspection_report import build_inspection_task_report


def _route(*, loops: int = 1, failure_policy: str = "stop") -> dict:
    return {
        "id": "route-a",
        "map_id": "map-a",
        "map_content_epoch": 1,
        "revision": 1,
        "loop_count": loops,
        "failure_policy": failure_policy,
        "points": [{"id": f"point-{index}", "action": "", "enabled": True} for index in range(3)],
    }


def _event(point: int, *, state: int, reason: str, kind: int = 2, loop: int = 0) -> dict:
    return {"point_index": point, "loop_index": loop, "state": state, "kind": kind, "reason": reason}


def _task(events: list[dict], *, state: str = "EXECUTING", complete: bool = True) -> dict:
    return {
        "task_id": "task-a",
        "current_state": state,
        "terminal": state in {"SUCCESS", "FAILED", "CANCELLED"},
        "execution_confirmed": bool(events),
        "delivery": {"history_complete": complete},
        "timeline": events,
        "latest_event": events[-1] if events else None,
    }


@pytest.mark.parametrize(("state", "native_state"), [("FAILED", 8), ("CANCELLED", 9)])
def test_terminal_route_does_not_complete_unvisited_points(state, native_state):
    events = [
        _event(0, state=4, reason="point_completed", kind=3),
        _event(1, state=2, reason="next_point"),
        _event(1, state=native_state, reason="operator_or_navigation_stop"),
    ]
    report = build_inspection_task_report(_task(events, state=state), _route(), {})

    assert report["coverage"]["completed_points"] == 1
    assert [point["status"] for point in report["points"]] == [
        "COMPLETED",
        state,
        "NOT_VISITED",
    ]
    InspectionTaskReportResponse.model_validate(report)


def test_skip_fact_is_separate_from_successful_completion():
    events = [
        _event(0, state=2, reason="point_skipped:goal_unreachable", kind=3),
        _event(1, state=2, reason="next_point"),
        _event(1, state=4, reason="point_completed", kind=3),
        _event(2, state=2, reason="next_point"),
        _event(2, state=4, reason="point_completed", kind=3),
        _event(3, state=7, reason="route_complete", loop=1),
    ]
    report = build_inspection_task_report(
        _task(events, state="SUCCESS"),
        _route(failure_policy="skip"),
        {},
    )

    assert report["coverage"]["completed_points"] == 2
    assert report["coverage"]["skipped_points"] == 1
    assert [point["status"] for point in report["points"]] == [
        "SKIPPED",
        "COMPLETED",
        "COMPLETED",
    ]
    assert report["points"][0]["reason"] == "goal_unreachable"
    assert report["report_status"] == "PARTIAL"
    assert report["acceptance"] == "REVIEW_REQUIRED"
    InspectionTaskReportResponse.model_validate(report)


def test_skip_route_with_no_point_outcomes_does_not_guess_completion():
    report = build_inspection_task_report(
        _task([_event(3, state=7, reason="route_complete", loop=1)], state="SUCCESS"),
        _route(failure_policy="skip"),
        {},
    )

    assert report["coverage"]["completed_points"] == 0
    assert all(point["status"] == "UNKNOWN" for point in report["points"])
    assert report["acceptance"] == "REVIEW_REQUIRED"


def test_completion_fact_stays_bound_to_its_loop():
    events = [
        _event(0, state=4, reason="point_completed", kind=3),
        _event(1, state=2, reason="next_point"),
        _event(1, state=8, reason="goal_unreachable"),
    ]
    report = build_inspection_task_report(
        _task(events, state="FAILED"),
        _route(loops=2),
        {},
    )

    assert report["coverage"]["completed_points"] == 1
    assert [point["status"] for point in report["points"][3:]] == ["NOT_VISITED"] * 3


def test_retained_completion_fact_remains_visible_without_guessing_missing_history():
    events = [
        _event(0, state=4, reason="point_completed", kind=3),
        _event(2, state=3, reason="leg_path_ready"),
    ]
    report = build_inspection_task_report(_task(events, complete=False), _route(), {})

    assert report["coverage"]["completed_points"] == 1
    assert [point["status"] for point in report["points"]] == [
        "COMPLETED",
        "UNKNOWN",
        "IN_PROGRESS",
    ]


def test_complete_non_skip_native_advance_history_preserves_legacy_completion():
    events = [
        _event(1, state=2, reason="next_point"),
        _event(1, state=8, reason="goal_unreachable"),
    ]
    report = build_inspection_task_report(_task(events, state="FAILED"), _route(), {})

    assert report["coverage"]["completed_points"] == 1
    assert [point["status"] for point in report["points"]] == [
        "COMPLETED",
        "FAILED",
        "NOT_VISITED",
    ]
