"""Stable state snapshot helpers for GatewayModule."""

from __future__ import annotations

from typing import Any

from gateway.navigation.status import build_navigation_status
from gateway.services.app_bootstrap import (
    CLIENT_LINKS,
    _map_summary,
    _media_summary,
)
from gateway.services.runtime_facts import capture_runtime_facts
from gateway.services.runtime_status import build_localization_status_from_parts, safe_lease, safe_session
from gateway.services.safety_status import safety_summary

STATE_SNAPSHOT_SCHEMA_VERSION = 4


def build_state_snapshot(gw: Any) -> dict[str, Any]:
    """Return the current session, localization, and navigation snapshot."""
    facts = capture_runtime_facts(gw)
    now = facts["ts"]
    teleop_clients = gw._teleop_client_count()

    session = safe_session(gw)
    localization = build_localization_status_from_parts(
        facts["odometry"],
        session,
        facts["icp_quality"],
        facts["localization_status"],
        gw=gw,
    )
    navigation = build_navigation_status(gw, facts=facts)
    return {
        "schema_version": STATE_SNAPSHOT_SCHEMA_VERSION,
        "ts": now,
        "server": {
            "api_version": "v1",
            "time": now,
        },
        "lease": safe_lease(gw),
        "teleop": {
            "active": facts["teleop_active"],
            "clients": int(teleop_clients),
        },
        "session": session,
        "safety": safety_summary(facts["navigation_state"]),
        "localization": localization,
        "navigation": navigation,
        "visual_servo": facts["visual_servo_status"],
        "map": _map_summary(gw, session),
        "scene": {
            "available": bool(facts["scene_graph_json"]) and facts["scene_graph_json"] != "{}",
            "endpoint": CLIENT_LINKS["scene_graph"],
        },
        "path": {
            "points": facts["path_len"],
            "endpoint": CLIENT_LINKS["path"],
        },
        "media": _media_summary(gw),
        "links": dict(CLIENT_LINKS),
    }
