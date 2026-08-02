"""Stable state snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
from typing import Any

from gateway.services.app_bootstrap import (
    CLIENT_LINKS,
    _map_summary,
    _media_summary,
)
from gateway.services.runtime_status import (
    build_localization_status_from_parts,
    build_navigation_status,
    safe_lease,
    safe_session,
)


STATE_SNAPSHOT_SCHEMA_VERSION = 1


def build_state_snapshot(gw: Any) -> dict[str, Any]:
    """Return /api/v1/state while preserving legacy top-level fields."""
    now = time.time()
    with gw._state_lock:
        odometry = gw._odom
        safety = gw._safety
        mission = gw._mission
        navigation_state = getattr(gw, "_navigation_state", None)
        eval_state = gw._eval
        dialogue = gw._dialogue
        mode = gw._mode
        teleop_active = gw._teleop_active
        scene_graph_json = gw._sg_json
        path_len = len(gw._last_path)
        localization_status = getattr(gw, "_localization_status", None)
    teleop_clients = gw._teleop_client_count()

    session = safe_session(gw)
    localization = build_localization_status_from_parts(
        odometry,
        session,
        float(getattr(gw, "_icp_quality", 0.0)),
        localization_status,
    )
    navigation = build_navigation_status(gw)

    return {
        "schema_version": STATE_SNAPSHOT_SCHEMA_VERSION,
        "ts": now,
        "server": {
            "api_version": "v1",
            "time": now,
        },
        # Legacy fields kept stable for existing dashboards/scripts.
        "odometry": odometry,
        "safety": safety,
        "mission": mission,
        "navigation_state": navigation_state,
        "eval": eval_state,
        "dialogue": dialogue,
        "mode": mode,
        "lease": safe_lease(gw),
        "teleop": {
            "active": bool(teleop_active),
            "clients": int(teleop_clients),
        },
        # New app/web-friendly summaries.
        "session": session,
        "localization": localization,
        "navigation": navigation,
        "map": _map_summary(gw, session),
        "scene": {
            "available": bool(scene_graph_json) and scene_graph_json != "{}",
            "endpoint": CLIENT_LINKS["scene_graph"],
        },
        "path": {
            "points": path_len,
            "endpoint": CLIENT_LINKS["path"],
        },
        "media": _media_summary(gw),
        "links": dict(CLIENT_LINKS),
    }
