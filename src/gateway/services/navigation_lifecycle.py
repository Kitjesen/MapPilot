"""Read-only Gateway projections of native navigation request lifecycle."""

from __future__ import annotations

import time
from typing import Any


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
        snapshot = dict(status) if isinstance(status, dict) else None
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
            "reason": "task_id_required",
            "ts": time.time(),
        }
    with gw._state_lock:
        status = gw._navigation_goal_status_by_task.get(normalized)
        snapshot = dict(status) if isinstance(status, dict) else None
    return {
        "schema_version": 1,
        "found": snapshot is not None,
        "task_id": normalized,
        "request_id": str(snapshot.get("request_id") or "") if snapshot else "",
        "status": snapshot,
        "reason": "" if snapshot is not None else "task_status_unknown",
        "ts": time.time(),
    }
