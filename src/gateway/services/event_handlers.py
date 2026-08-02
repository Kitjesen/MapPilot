"""Gateway module event serialization helpers."""

from __future__ import annotations

import json
import logging
import time
from typing import Any

logger = logging.getLogger(__name__)


def json_payload(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, default=str))
    except Exception as exc:
        logger.debug("json_payload round-trip failed: %s", exc)
        return {"raw": str(value)}


def handle_scene_graph(gw: Any, sg: Any) -> None:
    with gw._state_lock:
        gw._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
    gw._sg_throttle += 1
    if gw._sg_throttle % 5 != 0:
        return
    try:
        objects = [
            {
                "label": obj.label,
                "x": round(float(obj.position.x), 2),
                "y": round(float(obj.position.y), 2),
                "conf": round(float(obj.confidence), 2),
            }
            for obj in sg.objects
            if obj.label and float(obj.confidence) > 0.25
        ]
        if objects:
            gw.push_event({"type": "scene_graph", "objects": objects})
    except Exception as exc:
        logger.debug("_on_scene_graph: failed to build objects list: %s", exc)


def handle_safety(gw: Any, state: Any) -> None:
    data = {"level": getattr(state, "level", 0), "ts": time.time()}
    with gw._state_lock:
        gw._safety = data
    gw.push_event({"type": "safety", "data": data})


def handle_mission(gw: Any, status: Any) -> None:
    data = status if isinstance(status, dict) else {"raw": str(status)}
    with gw._state_lock:
        gw._mission = data
    gw.push_event({"type": "mission", "data": data})
    try:
        from gateway.services.runtime_status import build_navigation_status

        gw.push_event(
            {
                "type": "navigation_status",
                "data": build_navigation_status(gw),
            }
        )
    except Exception as exc:
        logger.debug("_on_mission: build_navigation_status failed: %s", exc)


def handle_navigation_state(gw: Any, state: Any) -> None:
    data = state.to_dict() if hasattr(state, "to_dict") else json_payload(state)
    if not isinstance(data, dict):
        data = {"raw": str(data)}
    with gw._state_lock:
        gw._navigation_state = data
    gw.push_event({"type": "navigation_state", "data": data})
    try:
        from gateway.services.runtime_status import build_navigation_status

        gw.push_event(
            {
                "type": "navigation_status",
                "data": build_navigation_status(gw),
            }
        )
    except Exception as exc:
        logger.debug("_on_navigation_state: build_navigation_status failed: %s", exc)


def handle_navigation_goal_status(gw: Any, status: Any) -> None:
    data = status.to_dict() if hasattr(status, "to_dict") else json_payload(status)
    if not isinstance(data, dict):
        return
    boot_id = str(data.get("boot_id") or "")
    sequence = int(data.get("sequence") or 0)
    task_id = str(data.get("task_id") or "")
    request_id = str(data.get("request_id") or "")
    if not boot_id or sequence <= 0 or not task_id or not request_id:
        return
    with gw._state_lock:
        sequences = gw._navigation_goal_status_sequences
        if sequence <= int(sequences.get(boot_id, 0)):
            return
        sequences[boot_id] = sequence
        tasks = gw._navigation_goal_status_by_task
        tasks.pop(task_id, None)
        tasks[task_id] = dict(data)
        while len(tasks) > 256:
            tasks.popitem(last=False)
        statuses = gw._navigation_goal_status_by_request
        statuses.pop(request_id, None)
        statuses[request_id] = dict(data)
        while len(statuses) > 256:
            statuses.popitem(last=False)
        gw._latest_navigation_goal_status = dict(data)
    gw.push_event({"type": "navigation_goal_status", "data": data})


def handle_inspection_task_event(gw: Any, event: Any) -> None:
    """Project one native inspection fact and forward that exact fact to SSE."""

    from gateway.services.inspection_task_lifecycle import (
        InspectionTaskJournalUnavailable,
        ensure_inspection_task_timeline,
    )

    timeline = ensure_inspection_task_timeline(gw)
    data = event.to_dict() if hasattr(event, "to_dict") else event
    task_id = str(data.get("task_id") or "") if isinstance(data, dict) else ""
    try:
        accepted = timeline.observe(event)
    except InspectionTaskJournalUnavailable as exc:
        journal = timeline.health()["journal"]
        gw.push_event(
            {
                "type": "inspection_task_journal_error",
                "data": {
                    "task_id": task_id,
                    "status": journal["status"],
                    "error": str(exc),
                },
            }
        )
        return
    if not accepted:
        return
    if not task_id:
        return
    projected = timeline.query(task_id).get("latest_event")
    if not isinstance(projected, dict):
        return
    gw.push_event({"type": "inspection_task_event", "data": projected})


def _request_exploration_stop_after_projection_failure(
    gw: Any,
    data: dict[str, Any],
) -> dict[str, Any]:
    """Issue one run-addressed fail-safe stop without claiming it completed."""

    from gateway.services.command_boundary import navigation_commands
    from gateway.services.explore_runs import new_request_id

    run_id = str(data.get("exploration_run_id") or "").strip()
    session_id = str(data.get("product_session_id") or "").strip()
    if not run_id or not session_id:
        return {"attempted": False, "accepted": False, "reason": "identity_missing"}
    requested = getattr(gw, "_explore_projection_stop_requests", None)
    if not isinstance(requested, set):
        requested = set()
        gw._explore_projection_stop_requests = requested
    if run_id in requested:
        return {"attempted": False, "accepted": False, "reason": "already_requested"}
    requested.add(run_id)

    commands = navigation_commands(gw)
    stop = getattr(commands, "stop_exploration", None) if commands is not None else None
    if not callable(stop):
        return {
            "attempted": False,
            "accepted": False,
            "reason": "native_exploration_stop_unavailable",
        }
    request_id = new_request_id()
    try:
        receipt = stop(
            exploration_run_id=run_id,
            session_id=session_id,
            reason="exploration_run_projection_failed",
            request_id=request_id,
        )
    except Exception as exc:
        return {
            "attempted": True,
            "accepted": False,
            "request_id": request_id,
            "reason": str(exc) or type(exc).__name__,
        }
    accepted = receipt.get("accepted") is True if isinstance(receipt, dict) else receipt is True
    return {
        "attempted": True,
        "accepted": accepted,
        "request_id": request_id,
        "reason": (
            str(receipt.get("reason") or "")
            if isinstance(receipt, dict)
            else "native_stop_acknowledged" if accepted else "native_stop_not_acknowledged"
        ),
    }


def _request_exploration_stop_for_rejected_known_run(
    gw: Any,
    data: dict[str, Any],
) -> dict[str, Any]:
    """Stop only a retained nonterminal run whose native fact was rejected."""

    from gateway.services.explore_runs import ensure_explore_runs

    run_id = str(data.get("exploration_run_id") or "").strip()
    if not run_id:
        return {"attempted": False, "accepted": False, "reason": "identity_missing"}
    try:
        run = ensure_explore_runs(gw).query(run_id)
    except (TypeError, ValueError):
        return {"attempted": False, "accepted": False, "reason": "run_not_retained"}
    if not run.get("found") or run.get("terminal") is True:
        return {"attempted": False, "accepted": False, "reason": "run_not_active"}
    identity = run.get("identity")
    session_id = (
        str(identity.get("product_session_id") or "").strip()
        if isinstance(identity, dict)
        else ""
    )
    return _request_exploration_stop_after_projection_failure(
        gw,
        {
            "exploration_run_id": run_id,
            "product_session_id": session_id,
        },
    )


def handle_exploration_run_event(gw: Any, event: Any) -> None:
    """Persist one native Explore fact before exposing the same fact to clients."""

    from gateway.services.explore_runs import (
        ExploreRunConflict,
        ExploreRunJournalUnavailable,
        ensure_explore_runs,
    )

    data = event.to_dict() if hasattr(event, "to_dict") else event
    if not isinstance(data, dict):
        gw.push_event(
            {
                "type": "exploration_run_event_rejected",
                "data": {"reason": "native_exploration_event_not_an_object"},
            }
        )
        return
    try:
        projected = ensure_explore_runs(gw).observe_event(event)
    except ExploreRunJournalUnavailable as exc:
        stop = _request_exploration_stop_after_projection_failure(gw, data)
        gw.push_event(
            {
                "type": "exploration_run_journal_error",
                "data": {
                    "exploration_run_id": data.get("exploration_run_id"),
                    "error": str(exc),
                    "compensating_stop": stop,
                },
            }
        )
        return
    except (ExploreRunConflict, KeyError, TypeError, ValueError) as exc:
        stop = _request_exploration_stop_for_rejected_known_run(gw, data)
        gw.push_event(
            {
                "type": "exploration_run_event_rejected",
                "data": {
                    "exploration_run_id": data.get("exploration_run_id"),
                    "reason": str(exc),
                    "compensating_stop": stop,
                },
            }
        )
        return

    gw._exploring = str(projected.get("state") or "") in {
        "submitted",
        "admitted",
        "running",
        "pausing",
        "paused",
        "cancelling",
    }
    gw.push_event({"type": "exploration_run_event", "data": dict(data)})


def handle_map_event(gw: Any, event: Any) -> None:
    data = event if isinstance(event, dict) else {"raw": str(event)}
    gw.push_event({"type": "map_event", "data": data})


def handle_eval(gw: Any, ev: Any) -> None:
    data = ev.to_dict() if hasattr(ev, "to_dict") else {"raw": str(ev)}
    with gw._state_lock:
        gw._eval = data
    gw.push_event({"type": "eval", "data": data})


def handle_dialogue(gw: Any, state: Any) -> None:
    data = state if isinstance(state, dict) else {"raw": str(state)}
    with gw._state_lock:
        gw._dialogue = data
    gw.push_event({"type": "dialogue", "data": data})


def handle_gnss_fusion_health(gw: Any, state: Any) -> None:
    data = state if isinstance(state, dict) else {"raw": str(state)}
    gw._blackbox.record("gnss", data)
    gw.push_event({"type": "gnss_fusion", "data": data})


def handle_tare_stats(gw: Any, stats: Any) -> None:
    data = stats if isinstance(stats, dict) else {"raw": str(stats)}
    with gw._state_lock:
        gw._last_tare_stats = dict(data)
    gw.push_event({"type": "tare_stats", "data": data})


def handle_exploration_supervisor(gw: Any, state: Any) -> None:
    data = state if isinstance(state, dict) else {"raw": str(state)}
    with gw._state_lock:
        gw._exploration_supervisor_state = dict(data)
    gw.push_event({"type": "exploration_supervisor", "data": data})


def handle_traversable_frontiers(gw: Any, candidates: Any) -> None:
    data = json_payload(candidates if isinstance(candidates, list) else [])
    with gw._state_lock:
        gw._last_traversable_frontiers = list(data) if isinstance(data, list) else []
    gw.push_event({"type": "traversable_frontiers", "data": data})


def handle_frontier_candidate(gw: Any, candidate: Any) -> None:
    data = json_payload(candidate if isinstance(candidate, dict) else {"raw": str(candidate)})
    with gw._state_lock:
        gw._last_frontier_candidate = dict(data) if isinstance(data, dict) else {"raw": str(data)}
    gw.push_event({"type": "frontier_candidate", "data": data})


def handle_agent_message(gw: Any, msg: Any) -> None:
    try:
        payload = msg if isinstance(msg, dict) else {"text": str(msg)}
        gw.push_event(
            {
                "type": "agent_message",
                "role": str(payload.get("role", "assistant")),
                "text": str(payload.get("text", "")),
                "phase": str(payload.get("phase", "")),
                "ts": float(payload.get("ts", time.time())),
            }
        )
    except Exception as exc:
        logger.debug("_on_agent_message serialize failed: %s", exc)
