"""Exploration runtime helpers for GatewayModule."""

from __future__ import annotations

import json
import logging
from typing import Any

logger = logging.getLogger(__name__)


def explorer_backend(gw: Any) -> str:
    if gw._frontier_explorer is not None:
        return "frontier"
    if gw._tare_explorer is not None:
        return "tare"
    return "none"


def explorer_available(gw: Any) -> bool:
    return explorer_backend(gw) != "none"


def explorer_unavailable_detail() -> dict[str, Any]:
    return {
        "reason": "explorer_backend_not_running",
        "required_profile": "explore_or_tare_explore",
        "supported_profiles": ["explore", "tare_explore"],
        "action": ("restart LingTu with the explore or tare_explore profile before starting exploration"),
    }


def coerce_explorer_result(result: Any) -> Any:
    if isinstance(result, str):
        try:
            return json.loads(result)
        except Exception as exc:
            logger.debug("_coerce_explorer_result JSON parse failed: %s", exc)
            return result
    return result


def begin_exploration(gw: Any) -> Any:
    if gw._frontier_explorer is not None:
        return coerce_explorer_result(gw._frontier_explorer.begin_exploration())
    if gw._tare_explorer is not None:
        starter = getattr(gw._tare_explorer, "start_tare_exploration", None)
        if starter is None:
            starter = getattr(gw._tare_explorer, "begin_exploration", None)
        if starter is None:
            raise RuntimeError("TARE explorer has no start method")
        return coerce_explorer_result(starter())
    raise RuntimeError("explorer_not_running")


def end_exploration(gw: Any) -> Any:
    if gw._frontier_explorer is not None:
        return coerce_explorer_result(gw._frontier_explorer.end_exploration())
    if gw._tare_explorer is not None:
        stopper = getattr(gw._tare_explorer, "stop_tare_exploration", None)
        if stopper is None:
            stopper = getattr(gw._tare_explorer, "end_exploration", None)
        if stopper is None:
            raise RuntimeError("TARE explorer has no stop method")
        return coerce_explorer_result(stopper())
    raise RuntimeError("explorer_not_running")


def tare_status_payload(gw: Any) -> dict[str, Any]:
    if gw._tare_explorer is None:
        return {}
    raw_status: Any = {}
    status_fn = getattr(gw._tare_explorer, "get_tare_status", None)
    if status_fn is not None:
        try:
            raw_status = coerce_explorer_result(status_fn())
        except Exception as exc:
            raw_status = {"error": str(exc)}
    if not isinstance(raw_status, dict):
        raw_status = {"raw": raw_status}
    return {
        "status": raw_status,
        "stats": gw._last_tare_stats or {},
    }


def exploration_status_payload(gw: Any) -> dict[str, Any]:
    backend = gw._explorer_backend()
    readiness = gw._exploration_start_readiness()
    if backend == "none":
        return {
            "available": False,
            "backend": "none",
            "exploring": False,
            "frontier_count": 0,
            **readiness,
            **gw._explorer_unavailable_detail(),
        }
    if backend == "frontier":
        health = {}
        if hasattr(gw._frontier_explorer, "health"):
            try:
                health = gw._frontier_explorer.health() or {}
            except Exception as exc:
                health = {"error": str(exc)}
        frontier_count = 0
        if isinstance(health, dict):
            try:
                frontier_count = int(health.get("frontier_count", 0) or 0)
            except (TypeError, ValueError):
                frontier_count = 0
        return {
            "available": True,
            "backend": "frontier",
            "exploring": gw._exploring,
            "frontier_count": frontier_count,
            **readiness,
        }

    tare = gw._tare_status_payload()
    tare_status = tare.get("status") if isinstance(tare, dict) else {}
    tare_started = bool(tare_status.get("started")) if isinstance(tare_status, dict) else False
    return {
        "available": True,
        "backend": "tare",
        "exploring": bool(gw._exploring or tare_started),
        "frontier_count": 0,
        "tare": tare,
        "supervisor": gw._exploration_supervisor_state or {},
        **readiness,
    }


def exploration_start_readiness(gw: Any) -> dict[str, Any]:
    blockers: list[str] = []
    advisories: list[str] = []
    navigation: dict[str, Any] = {}
    exploration_ignored_nav_blockers = {"navigation_session_inactive"}

    if not gw._explorer_available():
        blockers.append("explorer_backend_not_running")
    if gw._session_pending:
        blockers.append("session_transition_pending")
    if gw._exploring:
        blockers.append("exploration_already_active")
    if str(gw._session_mode or "idle").lower() != "idle":
        blockers.append("session_not_idle")

    try:
        from gateway.services.runtime_status import build_navigation_status

        navigation = build_navigation_status(gw)
        readiness = navigation.get("readiness") or {}
        nav_blockers = readiness.get("blockers") or []
        nav_advisories = readiness.get("advisories") or []
        if isinstance(nav_blockers, list):
            blockers.extend(str(code) for code in nav_blockers if str(code) not in exploration_ignored_nav_blockers)
        if isinstance(nav_advisories, list):
            advisories.extend(str(code) for code in nav_advisories)
        if not blockers and not bool(readiness.get("can_execute_autonomy", False)) and not nav_blockers:
            blockers.append("navigation_not_ready")
    except Exception as exc:
        logger.debug(
            "_exploration_start_readiness: build_navigation_status failed: %s",
            exc,
        )
        blockers.append("navigation_status_unavailable")

    blockers = list(dict.fromkeys(blockers))
    advisories = list(dict.fromkeys(advisories))
    return {
        "can_start": bool(gw._explorer_available()) and not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "navigation": {
            "state": navigation.get("state"),
            "can_accept_goal": navigation.get("can_accept_goal"),
            "readiness": navigation.get("readiness") or {},
        },
    }
