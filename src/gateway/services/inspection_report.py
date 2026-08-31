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
            point_status = "COMPLETED" if not action else "MISSING_EVIDENCE"
            reason = ""

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
                elif not history_complete:
                    evidence_status = "UNKNOWN"
                    point_status = "UNKNOWN"
                    reason = history_reason or "task_history_incomplete"
                elif not terminal and (
                    current_visit is None or visit >= current_visit
                ):
                    evidence_status = "PENDING"
                    point_status = (
                        "IN_PROGRESS" if visit == current_visit else "PENDING"
                    )
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
            elif not history_complete:
                point_status = "UNKNOWN"
                reason = history_reason or "task_history_incomplete"
            elif not terminal:
                if current_visit is None or visit > current_visit:
                    point_status = "PENDING"
                elif visit == current_visit:
                    point_status = "IN_PROGRESS"

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
            "required_evidence": required_evidence,
            "verified_evidence": verified_evidence,
            "missing_evidence": missing_evidence,
            "invalid_evidence": invalid_evidence,
            "unavailable_evidence": unavailable_evidence,
            "unknown_evidence": unknown_evidence,
        },
        "points": points,
        "issues": issues,
    }


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
        "run_id": task_id,
        "route_id": str(route.get("id") or ""),
        "route_revision": int(route.get("revision") or 0),
        "map_id": str(route.get("map_id") or ""),
        "map_content_epoch": int(route.get("map_content_epoch") or 0),
        "point_id": point_id,
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
