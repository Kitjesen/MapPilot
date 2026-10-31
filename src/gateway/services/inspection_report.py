"""Build one user-facing inspection result from authoritative stored facts."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

_PUBLIC_TASK_STATES = frozenset(
    {
        "PLANNING",
        "EXECUTING",
        "PAUSED",
        "RECOVERING",
        "SUCCESS",
        "FAILED",
        "CANCELLED",
    }
)


def build_inspection_task_report(
    task: Mapping[str, Any],
    route: Mapping[str, Any],
    evidence_by_id: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Return a read-only product report without inferring task execution."""

    task_id = str(task.get("task_id") or "")
    identity = task.get("identity")
    if not isinstance(identity, Mapping):
        identity = {}
    timeline = task.get("timeline")
    if not isinstance(timeline, list):
        timeline = []
    execution_state = _public_execution_state(task)
    terminal = bool(task.get("terminal", False))
    execution_confirmed = bool(task.get("execution_confirmed", False))
    execution_reason = str(task.get("reason") or "")
    delivery = task.get("delivery")
    if not isinstance(delivery, Mapping):
        delivery = {}
    history_complete = bool(delivery.get("history_complete", False))
    history_reason = str(delivery.get("reason") or "")
    latest_event = task.get("latest_event")
    current_visit: tuple[int, int] | None = None
    if isinstance(latest_event, Mapping):
        current_visit = (
            max(0, int(latest_event.get("loop_index", 0) or 0)),
            max(0, int(latest_event.get("point_index", 0) or 0)),
        )

    evidence_events: dict[tuple[int, int], Mapping[str, Any]] = {}
    for raw_event in timeline:
        if not isinstance(raw_event, Mapping):
            continue
        if int(raw_event.get("kind", 0) or 0) != 5:
            continue
        evidence_id = str(raw_event.get("evidence_id") or "").strip()
        if not evidence_id:
            continue
        key = (
            max(0, int(raw_event.get("loop_index", 0) or 0)),
            max(0, int(raw_event.get("point_index", 0) or 0)),
        )
        evidence_events[key] = raw_event

    route_points = route.get("points")
    if not isinstance(route_points, list):
        route_points = []
    loop_count = max(1, int(route.get("loop_count", 1) or 1))
    point_outcomes = _native_point_outcomes(
        timeline, route, route_points, loop_count, history_complete=history_complete,
    )
    points: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    required_evidence = 0
    verified_evidence = 0
    missing_evidence = 0
    invalid_evidence = 0
    unavailable_evidence = 0
    unknown_evidence = 0

    if not history_complete:
        issues.append(
            {
                "code": "task_history_incomplete",
                "reason": history_reason or "task_history_incomplete",
            }
        )

    for loop_index in range(loop_count):
        for point_index, raw_point in enumerate(route_points):
            if not isinstance(raw_point, Mapping) or raw_point.get("enabled") is False:
                continue
            point_id = str(raw_point.get("id") or "")
            action = str(raw_point.get("action") or "").strip()
            visit = (loop_index, point_index)
            event = evidence_events.get(visit)
            evidence_id = str(event.get("evidence_id") or "") if event else ""
            evidence_status = "NOT_REQUIRED" if not action else "MISSING"
            native_status, native_reason = point_outcomes.get(visit, ("", ""))
            if not native_status:
                if visit == current_visit and execution_confirmed:
                    native_status = (
                        execution_state if execution_state in {"FAILED", "CANCELLED"}
                        else "UNKNOWN" if terminal else "IN_PROGRESS"
                    )
                    native_reason = execution_reason if terminal else ""
                elif current_visit is None or visit > current_visit:
                    native_status = "NOT_VISITED" if terminal else "PENDING"
                else:
                    native_status = "UNKNOWN"
                    native_reason = history_reason or "point_outcome_not_recorded"
            point_status = native_status
            reason = native_reason

            if action:
                required_evidence += 1
                if evidence_id:
                    evidence = evidence_by_id.get(evidence_id)
                    evidence_status, reason = _verified_evidence_status(
                        evidence,
                        task_id=task_id,
                        route=route,
                        point_id=point_id,
                        point_index=point_index,
                        action=action,
                    )
                elif native_status == "UNKNOWN":
                    evidence_status = "UNKNOWN"
                    point_status = "UNKNOWN"
                elif native_status in {"PENDING", "IN_PROGRESS"}:
                    evidence_status = "PENDING"
                if evidence_status == "VERIFIED":
                    verified_evidence += 1
                    point_status = "COMPLETED"
                elif evidence_status == "INVALID":
                    invalid_evidence += 1
                    point_status = "INVALID_EVIDENCE"
                elif evidence_status == "UNAVAILABLE":
                    unavailable_evidence += 1
                    point_status = "UNAVAILABLE_EVIDENCE"
                elif evidence_status == "PENDING":
                    pass
                elif evidence_status == "UNKNOWN":
                    unknown_evidence += 1
                else:
                    missing_evidence += 1
                    point_status = "MISSING_EVIDENCE"
                if native_status in {"SKIPPED", "FAILED", "CANCELLED", "NOT_VISITED"}:
                    point_status = native_status
                    reason = native_reason
                if point_status in {
                    "MISSING_EVIDENCE",
                    "INVALID_EVIDENCE",
                    "UNAVAILABLE_EVIDENCE",
                }:
                    issues.append(
                        {
                            "code": point_status.lower(),
                            "loop_index": loop_index,
                            "point_index": point_index,
                            "point_id": point_id,
                            "action": action,
                            "evidence_id": evidence_id,
                            "reason": reason,
                        }
                    )

            if point_status == "SKIPPED" or (point_status == "UNKNOWN" and history_complete):
                issues.append(
                    {
                        "code": "point_skipped" if point_status == "SKIPPED" else "point_outcome_unknown",
                        "loop_index": loop_index,
                        "point_index": point_index,
                        "point_id": point_id,
                        "action": action,
                        "evidence_id": evidence_id,
                        "reason": reason,
                    }
                )

            points.append(
                {
                    "loop_index": loop_index,
                    "point_index": point_index,
                    "point_id": point_id,
                    "action": action,
                    "status": point_status,
                    "evidence_status": evidence_status,
                    "evidence_id": evidence_id,
                    "reason": reason,
                    "inspection_result": _point_inspection_result(
                        action, point_status, evidence_status, evidence_by_id.get(evidence_id)
                    ),
                }
            )

    report_status, acceptance = _report_outcome(
        execution_state=execution_state,
        terminal=terminal,
        execution_confirmed=execution_confirmed,
        history_complete=history_complete,
        issues=issues,
    )
    completed_points = sum(1 for point in points if point["status"] == "COMPLETED")

    return {
        "schema_version": "lingtu.inspection.report.v1",
        "task_id": task_id,
        "report_status": report_status,
        "acceptance": acceptance,
        "terminal": terminal,
        "execution": {
            "state": execution_state,
            "terminal": terminal,
            "confirmed": execution_confirmed,
            "reason": execution_reason,
            "history_complete": history_complete,
            "history_reason": history_reason,
        },
        "identity": {
            "route_id": str(identity.get("route_id") or route.get("id") or ""),
            "route_revision": int(
                identity.get("route_revision") or route.get("revision") or 0
            ),
            "map_id": str(identity.get("map_id") or route.get("map_id") or ""),
            "map_content_epoch": int(
                identity.get("map_content_epoch") or route.get("map_content_epoch") or 0
            ),
        },
        "coverage": {
            "required_points": len(points),
            "completed_points": completed_points,
            "skipped_points": sum(point["status"] == "SKIPPED" for point in points),
            "failed_points": sum(point["status"] == "FAILED" for point in points),
            "cancelled_points": sum(point["status"] == "CANCELLED" for point in points),
            "not_visited_points": sum(point["status"] == "NOT_VISITED" for point in points),
            "unknown_points": sum(point["status"] == "UNKNOWN" for point in points),
            "required_evidence": required_evidence,
            "verified_evidence": verified_evidence,
            "missing_evidence": missing_evidence,
            "invalid_evidence": invalid_evidence,
            "unavailable_evidence": unavailable_evidence,
            "unknown_evidence": unknown_evidence,
        },
        "inspection_result": _inspection_summary(points, terminal=terminal),
        "points": points,
        "issues": issues,
    }


def _point_inspection_result(
    action: str,
    point_status: str,
    evidence_status: str,
    evidence: Mapping[str, Any] | None,
) -> str:
    """Capture completion and equipment condition are independent facts."""

    if not action:
        return "NOT_REQUIRED"
    if point_status != "COMPLETED" or evidence_status != "VERIFIED" or evidence is None:
        return "NOT_INSPECTED"
    summary = evidence.get("summary")
    if not isinstance(summary, Mapping):
        return "NOT_INSPECTED"
    review = summary.get("review")
    if isinstance(review, Mapping):
        verdict = review.get("verdict")
    else:
        analysis = summary.get("analysis")
        if not isinstance(analysis, Mapping) or analysis.get("support") in {
            "unavailable", "not_applicable"
        }:
            return "NEEDS_REVIEW"
        verdict = analysis.get("verdict")
    return {
        "normal": "NORMAL", "clear": "NORMAL",
        "anomaly": "ANOMALY", "violation": "ANOMALY",
        "inconclusive": "INCONCLUSIVE",
    }.get(verdict, "NEEDS_REVIEW")


def _inspection_summary(points: list[dict[str, Any]], *, terminal: bool) -> dict[str, Any]:
    states = [point["inspection_result"] for point in points if point["action"]]
    counts = {
        "normal_points": states.count("NORMAL"),
        "anomaly_points": states.count("ANOMALY"),
        "inconclusive_points": states.count("INCONCLUSIVE"),
        "unreviewed_points": states.count("NEEDS_REVIEW"),
        "not_inspected_points": states.count("NOT_INSPECTED"),
    }
    if not states:
        status = "NOT_REQUIRED"
    elif counts["anomaly_points"]:
        status = "ANOMALY"
    elif not terminal:
        status = "IN_PROGRESS"
    elif counts["not_inspected_points"] or counts["unreviewed_points"]:
        status = "NEEDS_REVIEW"
    elif counts["inconclusive_points"]:
        status = "INCONCLUSIVE"
    else:
        status = "NORMAL"
    return {"status": status, **counts}


def _native_point_outcomes(
    timeline: list[Any],
    route: Mapping[str, Any],
    route_points: list[Any],
    loop_count: int,
    *,
    history_complete: bool,
) -> dict[tuple[int, int], tuple[str, str]]:
    """Read native point facts; moving a cursor alone does not prove a skipped point."""

    outcomes: dict[tuple[int, int], tuple[str, str]] = {}
    advanced_to: tuple[int, int] | None = None
    for event in timeline:
        if not isinstance(event, Mapping):
            continue
        visit = (
            max(0, int(event.get("loop_index", 0) or 0)),
            max(0, int(event.get("point_index", 0) or 0)),
        )
        kind = int(event.get("kind", 0) or 0)
        reason = str(event.get("reason") or "")
        if kind == 3 and reason == "point_completed":
            outcomes[visit] = ("COMPLETED", "")
        elif kind == 3 and reason.startswith("point_skipped:"):
            outcomes[visit] = ("SKIPPED", reason.removeprefix("point_skipped:"))
        elif kind == 5 and event.get("evidence_id"):
            outcomes[visit] = ("COMPLETED", "")
        elif kind == 2 and reason in {"next_point", "next_loop", "route_complete"}:
            advanced_to = max(advanced_to or visit, visit)

    # Existing non-skip routes advance only after a successful visit. A complete
    # retained history therefore preserves their completion fact without
    # deriving success from failed/cancelled snapshots or guessing skip outcomes.
    if history_complete and route.get("failure_policy", "stop") != "skip" and advanced_to:
        for loop_index in range(loop_count):
            for point_index, point in enumerate(route_points):
                visit = (loop_index, point_index)
                if isinstance(point, Mapping) and point.get("enabled") is not False and visit < advanced_to:
                    outcomes.setdefault(visit, ("COMPLETED", ""))
    return outcomes


def _verified_evidence_status(
    evidence: Mapping[str, Any] | None,
    *,
    task_id: str,
    route: Mapping[str, Any],
    point_id: str,
    point_index: int,
    action: str,
) -> tuple[str, str]:
    if evidence is None:
        return "MISSING", "evidence_not_found"
    status = str(evidence.get("status") or "MISSING").upper()
    if status != "VERIFIED":
        return status, str(evidence.get("reason") or "")
    summary = evidence.get("summary")
    if not isinstance(summary, Mapping):
        return "INVALID", "evidence_summary_invalid"
    request = summary.get("request")
    if not isinstance(request, Mapping):
        return "INVALID", "evidence_request_invalid"
    expected = {
        "task_id": task_id,
        "route_id": str(route.get("id") or ""),
        "route_revision": int(route.get("revision") or 0),
        "map_id": str(route.get("map_id") or ""),
        "map_content_epoch": int(route.get("map_content_epoch") or 0),
        "point_index": point_index,
        "action": action,
    }
    for field, expected_value in expected.items():
        if request.get(field) != expected_value:
            return "INVALID", f"evidence_identity_mismatch:{field}"
    return "VERIFIED", ""


def _report_outcome(
    *,
    execution_state: str | None,
    terminal: bool,
    execution_confirmed: bool,
    history_complete: bool,
    issues: list[dict[str, Any]],
) -> tuple[str, str]:
    if terminal and execution_confirmed:
        if execution_state == "SUCCESS":
            if not history_complete:
                return "UNKNOWN", "REVIEW_REQUIRED"
            if issues:
                return "PARTIAL", "REVIEW_REQUIRED"
            return "COMPLETE", "ACCEPTABLE"
        if execution_state == "CANCELLED":
            return "CANCELLED", "NOT_ACCEPTABLE"
        if execution_state == "FAILED":
            return "FAILED", "NOT_ACCEPTABLE"
        return "UNKNOWN", "UNKNOWN"
    if not history_complete:
        return "UNKNOWN", "REVIEW_REQUIRED"
    if not terminal or not execution_confirmed:
        if execution_state is None:
            return "UNKNOWN", "UNKNOWN"
        return "IN_PROGRESS", "PENDING"
    return "UNKNOWN", "UNKNOWN"


def _public_execution_state(task: Mapping[str, Any]) -> str | None:
    state = task.get("current_state")
    if not isinstance(state, str) or state not in _PUBLIC_TASK_STATES:
        return None
    return state


__all__ = ["build_inspection_task_report"]
