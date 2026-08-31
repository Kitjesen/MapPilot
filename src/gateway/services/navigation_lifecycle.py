"""Read-only Gateway projections of native navigation request lifecycle."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any, Final

from runtime.msgs.nav import NavigationLifecycle


class NavigationTaskProjectionError(ValueError):
    """Raised when a native goal phase cannot be projected to the public lifecycle."""


_GOAL_PHASE_TO_LIFECYCLE: Final[dict[str, str]] = {
    "PLANNING": "PLANNING",
    "PATH_ACTIVE": "EXECUTING",
    "EXECUTING": "EXECUTING",
    "PAUSED": "PAUSED",
    "RECOVERING": "RECOVERING",
    "REACHED": "SUCCESS",
    "SUCCESS": "SUCCESS",
    "FAILED": "FAILED",
    "CANCELLED": "CANCELLED",
}
_TERMINAL_LIFECYCLES: Final[frozenset[str]] = frozenset(
    {"SUCCESS", "FAILED", "CANCELLED"}
)


def project_navigation_goal_status(status: Mapping[str, Any]) -> dict[str, Any]:
    """Project one native navigation phase onto the seven-state public contract."""

    projected = dict(status)
    phase = str(
        projected.get("phase")
        or projected.get("native_state_name")
        or projected.get("state_name")
        or ""
    ).strip().upper()
    lifecycle_name = _GOAL_PHASE_TO_LIFECYCLE.get(phase)
    if lifecycle_name is None:
        raise NavigationTaskProjectionError(
            f"unsupported_navigation_goal_phase:{phase or 'missing'}"
        )

    native_state = projected.get("native_state", projected.get("state"))
    lifecycle_state = int(NavigationLifecycle[lifecycle_name])
    projected.update(
        {
            "native_state": native_state,
            "phase": phase,
            "state": lifecycle_state,
            "state_name": lifecycle_name,
            "lifecycle_state": lifecycle_state,
            "lifecycle_state_name": lifecycle_name,
            "terminal": lifecycle_name in _TERMINAL_LIFECYCLES,
        }
    )
    return projected


def query_navigation_goal_status(gw: Any, request_id: str) -> dict[str, Any]:
    normalized = str(request_id or "").strip()
    if not normalized:
        return {
            "schema_version": 1,
            "found": False,
            "request_id": "",
            "status": None,
            "reason": "request_id_required",
            "ts": time.time(),
        }
    with gw._state_lock:
        status = gw._navigation_goal_status_by_request.get(normalized)
        snapshot = project_navigation_goal_status(status) if isinstance(status, dict) else None
    return {
        "schema_version": 1,
        "found": snapshot is not None,
        "request_id": normalized,
        "status": snapshot,
        "reason": "" if snapshot is not None else "request_status_unknown",
        "ts": time.time(),
    }


def query_navigation_task_status(gw: Any, task_id: str) -> dict[str, Any]:
    normalized = str(task_id or "").strip()
    if not normalized:
        return {
            "schema_version": 1,
            "found": False,
            "task_id": "",
            "request_id": "",
            "status": None,
            "source": "",
            "evidence_status": "",
            "reason": "task_id_required",
            "ts": time.time(),
        }
    with gw._state_lock:
        status = gw._navigation_goal_status_by_task.get(normalized)
        retained = dict(status) if isinstance(status, dict) else None

    if retained is not None:
        snapshot = project_navigation_goal_status(retained)
        source = "live_gateway_cache"
        evidence_status = "live"
    else:
        record, lookup_reason = _read_durable_task_record(gw, normalized)
        if record is None:
            return {
                "schema_version": 1,
                "found": False,
                "task_id": normalized,
                "request_id": "",
                "status": None,
                "source": "durable_task_ledger" if lookup_reason else "",
                "evidence_status": "",
                "reason": lookup_reason or "task_status_unknown",
                "ts": time.time(),
            }
        evidence_status = str(record.get("evidence_status") or "")
        native_status = record.get("last_goal_status")
        if not isinstance(native_status, Mapping):
            return {
                "schema_version": 1,
                "found": False,
                "task_id": normalized,
                "request_id": "",
                "status": None,
                "source": "durable_task_ledger",
                "evidence_status": evidence_status,
                "reason": "native_task_status_not_observed",
                "ts": time.time(),
            }
        if str(native_status.get("task_id") or "").strip() != normalized:
            return {
                "schema_version": 1,
                "found": False,
                "task_id": normalized,
                "request_id": "",
                "status": None,
                "source": "durable_task_ledger",
                "evidence_status": evidence_status,
                "reason": "native_task_status_identity_mismatch",
                "ts": time.time(),
            }
        try:
            snapshot = project_navigation_goal_status(native_status)
        except NavigationTaskProjectionError:
            return {
                "schema_version": 1,
                "found": False,
                "task_id": normalized,
                "request_id": "",
                "status": None,
                "source": "durable_task_ledger",
                "evidence_status": evidence_status,
                "reason": "native_task_status_invalid",
                "ts": time.time(),
            }
        source = "durable_native_goal_status"
    return {
        "schema_version": 1,
        "found": True,
        "task_id": normalized,
        "request_id": str(snapshot.get("request_id") or ""),
        "status": snapshot,
        "source": source,
        "evidence_status": evidence_status,
        "reason": "",
        "ts": time.time(),
    }


def _read_durable_task_record(
    gw: Any,
    task_id: str,
) -> tuple[Mapping[str, Any] | None, str]:
    goals = getattr(gw, "_goals", None)
    if goals is None:
        modules = getattr(gw, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, Mapping) else None
    get_task = getattr(goals, "get_task", None)
    if not callable(get_task):
        return None, ""
    try:
        record = get_task(task_id)
    except Exception:
        return None, "task_status_unavailable"
    if record is None:
        return None, "task_status_unknown"
    if not isinstance(record, Mapping):
        return None, "task_status_unavailable"
    if str(record.get("task_id") or "").strip() != task_id:
        return None, "task_status_identity_mismatch"
    return record, ""
