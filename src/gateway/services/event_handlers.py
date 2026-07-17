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
