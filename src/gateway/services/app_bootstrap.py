"""Lightweight App/Web bootstrap contract for GatewayModule."""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping
from typing import Any

from runtime.diagnostics.runtime_validation_gates import runtime_validation_gates
from gateway.services.media_status import build_media_status
from gateway.services.runtime_status import (
    build_localization_status_from_parts,
    build_navigation_status,
    safe_session as _safe_session,
)
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_clear_for_motion,
    safety_summary,
)
from gateway.services.traffic import (
    DEFAULT_SSE_RASTER_MIN_INTERVAL_S,
    DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED,
    SSE_DIAGNOSTIC_EVENT_TYPES,
    SSE_EVENT_SCHEMA_VERSION,
    SSE_EVENT_TYPES,
    SSE_LEGACY_EVENT_TYPES,
    SSE_RETRY_MS,
)


APP_BOOTSTRAP_SCHEMA_VERSION = 1
APP_CAPABILITIES_SCHEMA_VERSION = 1
APP_TRAFFIC_SCHEMA_VERSION = 1
_OPERATION_CONTRACT_CACHE_ATTR = "_app_capabilities_operation_contract_cache"
_OPERATION_CONTRACT_CACHE_LOCK = threading.RLock()

CLIENT_LINKS: dict[str, str] = {
    "bootstrap": "/api/v1/app/bootstrap",
    "capabilities": "/api/v1/app/capabilities",
    "traffic": "/api/v1/app/traffic",
    "state": "/api/v1/state",
    "scene_graph": "/api/v1/scene_graph",
    "locations": "/api/v1/locations",
    "location_detail": "/api/v1/locations/{name}",
    "path": "/api/v1/path",
    "localization_status": "/api/v1/localization/status",
    "navigation_status": "/api/v1/navigation/status",
    "runtime_dataflow": "/api/v1/runtime/dataflow",
    "runtime_dataflow_topic": "/api/v1/runtime/dataflow/topic",
    "runtime_dataflow_subscribe": "/api/v1/runtime/dataflow/subscribe",
    "runtime_switch_plan": "/api/v1/runtime/switch-plan",
    "runtime_switch": "/api/v1/runtime/switch",
    "devices": "/api/v1/devices",
    "health": "/api/v1/health",
    "readiness": "/api/v1/readiness",
    "auth_login": "/api/v1/auth/login",
    "auth_check": "/api/v1/auth/check",
    "session": "/api/v1/session",
    "session_start": "/api/v1/session/start",
    "session_end": "/api/v1/session/end",
    "events": "/api/v1/events",
    "teleop_ws": "/ws/teleop",
    "camera_ws": "/ws/camera",
    "cloud_ws": "/ws/cloud",
    "camera_snapshot": "/api/v1/camera/snapshot",
    "webrtc_stats": "/api/v1/webrtc/stats",
    "webrtc_offer": "/api/v1/webrtc/offer",
    "webrtc_bitrate": "/api/v1/webrtc/bitrate",
    "webrtc_whep": "/api/v1/webrtc/whep",
    "go2rtc_status": "/api/v1/webrtc/go2rtc/status",
    "goal": "/api/v1/goal",
    "navigate_click": "/api/v1/navigate/click",
    "navigation_goal_candidate": "/api/v1/navigation/goal_candidate",
    "navigation_plan": "/api/v1/navigation/plan",
    "inspection_acceptance": "/api/v1/inspection/acceptance",
    "navigation_cancel": "/api/v1/navigation/cancel",
    "stop": "/api/v1/stop",
    "instruction": "/api/v1/instruction",
    "visual_servo": "/api/v1/visual_servo",
    "mode": "/api/v1/mode",
    "lease": "/api/v1/lease",
    "maps": "/api/v1/slam/maps",
    "map_lifecycle": "/api/v1/maps",
    "map_activate": "/api/v1/map/activate",
    "map_rename": "/api/v1/map/rename",
    "map_save": "/api/v1/map/save",
    "map_import_pcd": "/api/v1/maps/import_pcd",
    "map_crop": "/api/v1/maps/{name}/crop",
    "map_mark_zone": "/api/v1/maps/{name}/mark_zone",
    "map_build_octomap": "/api/v1/maps/{name}/build_octomap",
    "map_validate_plan": "/api/v1/maps/{name}/validate_plan",
    "map_restore_predufo": "/api/v1/map/restore_predufo",
    "map_cloud_reset": "/api/v1/map_cloud/reset",
    "map_points": "/api/v1/map/points",
    "saved_map_points": "/api/v1/maps/{name}/points",
    "explore_status": "/api/v1/explore/status",
    "explore_start": "/api/v1/explore/start",
    "explore_stop": "/api/v1/explore/stop",
    "slam_status": "/api/v1/slam/status",
    "slam_switch": "/api/v1/slam/switch",
    "slam_auto_relocalize": "/api/v1/slam/auto_relocalize",
    "slam_relocalize": "/api/v1/slam/relocalize",
    "slam_track_against_map": "/api/v1/slam/track_against_map",
    "bag_start": "/api/v1/bag/start",
    "bag_stop": "/api/v1/bag/stop",
    "bag_status": "/api/v1/bag/status",
    "memory_temporal": "/api/v1/memory/temporal",
    "memory_temporal_semantic": "/api/v1/memory/temporal/semantic",
    "diagnostic_pack": "/api/v1/diagnostic_pack",
    "field_check": "/api/v1/diagnostics/field-check",
    "routecheck_latest": "/api/v1/diagnostics/routecheck/latest",
    "real_runtime_evidence_latest": "/api/v1/diagnostics/real-runtime-evidence/latest",
    "algorithm_benchmark_latest": "/api/v1/diagnostics/algorithm-benchmark/latest",
    "runtime_contract": "/api/v1/diagnostics/runtime-contract",
}

CLIENT_ENDPOINTS: dict[str, dict[str, dict[str, str]]] = {
    "app": {
        "bootstrap": {"method": "GET", "path": CLIENT_LINKS["bootstrap"]},
        "capabilities": {"method": "GET", "path": CLIENT_LINKS["capabilities"]},
        "traffic": {"method": "GET", "path": CLIENT_LINKS["traffic"]},
    },
    "auth": {
        "login": {"method": "POST", "path": CLIENT_LINKS["auth_login"]},
        "check": {"method": "GET", "path": CLIENT_LINKS["auth_check"]},
    },
    "state": {
        "snapshot": {"method": "GET", "path": CLIENT_LINKS["state"]},
        "scene_graph": {"method": "GET", "path": CLIENT_LINKS["scene_graph"]},
        "locations": {"method": "GET", "path": CLIENT_LINKS["locations"]},
        "location_create": {"method": "POST", "path": CLIENT_LINKS["locations"]},
        "location_update": {"method": "PUT", "path": CLIENT_LINKS["location_detail"]},
        "location_delete": {"method": "DELETE", "path": CLIENT_LINKS["location_detail"]},
        "path": {"method": "GET", "path": CLIENT_LINKS["path"]},
        "localization_status": {
            "method": "GET",
            "path": CLIENT_LINKS["localization_status"],
        },
        "navigation_status": {
            "method": "GET",
            "path": CLIENT_LINKS["navigation_status"],
        },
        "runtime_dataflow": {
            "method": "GET",
            "path": CLIENT_LINKS["runtime_dataflow"],
        },
        "runtime_dataflow_topic": {
            "method": "GET",
            "path": CLIENT_LINKS["runtime_dataflow_topic"],
        },
        "runtime_dataflow_subscribe": {
            "method": "POST",
            "path": CLIENT_LINKS["runtime_dataflow_subscribe"],
        },
        "devices": {"method": "GET", "path": CLIENT_LINKS["devices"]},
        "health": {"method": "GET", "path": CLIENT_LINKS["health"]},
        "readiness": {"method": "GET", "path": CLIENT_LINKS["readiness"]},
    },
    "realtime": {
        "events": {"method": "SSE", "path": CLIENT_LINKS["events"]},
        "teleop": {"method": "WS", "path": CLIENT_LINKS["teleop_ws"]},
        "camera": {"method": "WS", "path": CLIENT_LINKS["camera_ws"]},
        "cloud": {"method": "WS", "path": CLIENT_LINKS["cloud_ws"]},
    },
    "control": {
        "navigation_goal_candidate": {
            "method": "POST",
            "path": CLIENT_LINKS["navigation_goal_candidate"],
        },
        "navigation_plan": {"method": "POST", "path": CLIENT_LINKS["navigation_plan"]},
        "navigation_cancel": {"method": "POST", "path": CLIENT_LINKS["navigation_cancel"]},
        "goal": {"method": "POST", "path": CLIENT_LINKS["goal"]},
        "navigate_click": {"method": "POST", "path": CLIENT_LINKS["navigate_click"]},
        "stop": {"method": "POST", "path": CLIENT_LINKS["stop"]},
        "instruction": {"method": "POST", "path": CLIENT_LINKS["instruction"]},
        "visual_servo": {"method": "POST", "path": CLIENT_LINKS["visual_servo"]},
        "mode": {"method": "POST", "path": CLIENT_LINKS["mode"]},
        "lease": {"method": "POST", "path": CLIENT_LINKS["lease"]},
    },
    "media": {
        "camera_snapshot": {"method": "GET", "path": CLIENT_LINKS["camera_snapshot"]},
        "webrtc_stats": {"method": "GET", "path": CLIENT_LINKS["webrtc_stats"]},
        "webrtc_offer": {"method": "POST", "path": CLIENT_LINKS["webrtc_offer"]},
        "webrtc_bitrate": {"method": "POST", "path": CLIENT_LINKS["webrtc_bitrate"]},
        "webrtc_whep": {"method": "POST", "path": CLIENT_LINKS["webrtc_whep"]},
        "go2rtc_status": {"method": "GET", "path": CLIENT_LINKS["go2rtc_status"]},
    },
    "map": {
        "maps": {"method": "GET", "path": CLIENT_LINKS["maps"]},
        "map_lifecycle": {"method": "POST", "path": CLIENT_LINKS["map_lifecycle"]},
        "map_activate": {"method": "POST", "path": CLIENT_LINKS["map_activate"]},
        "map_rename": {"method": "POST", "path": CLIENT_LINKS["map_rename"]},
        "map_save": {"method": "POST", "path": CLIENT_LINKS["map_save"]},
        "map_import_pcd": {"method": "POST", "path": CLIENT_LINKS["map_import_pcd"]},
        "map_crop": {"method": "POST", "path": CLIENT_LINKS["map_crop"]},
        "map_mark_zone": {"method": "POST", "path": CLIENT_LINKS["map_mark_zone"]},
        "map_build_octomap": {
            "method": "POST",
            "path": CLIENT_LINKS["map_build_octomap"],
        },
        "map_validate_plan": {
            "method": "POST",
            "path": CLIENT_LINKS["map_validate_plan"],
        },
        "map_restore_predufo": {
            "method": "POST",
            "path": CLIENT_LINKS["map_restore_predufo"],
        },
        "map_cloud_reset": {"method": "POST", "path": CLIENT_LINKS["map_cloud_reset"]},
        "map_points": {"method": "GET", "path": CLIENT_LINKS["map_points"]},
        "saved_map_points": {"method": "GET", "path": CLIENT_LINKS["saved_map_points"]},
        "session": {"method": "GET", "path": CLIENT_LINKS["session"]},
        "session_start": {"method": "POST", "path": CLIENT_LINKS["session_start"]},
        "session_end": {"method": "POST", "path": CLIENT_LINKS["session_end"]},
        "explore_status": {"method": "GET", "path": CLIENT_LINKS["explore_status"]},
        "explore_start": {"method": "POST", "path": CLIENT_LINKS["explore_start"]},
        "explore_stop": {"method": "POST", "path": CLIENT_LINKS["explore_stop"]},
    },
    "ops": {
        "slam_status": {"method": "GET", "path": CLIENT_LINKS["slam_status"]},
        "slam_switch": {"method": "POST", "path": CLIENT_LINKS["slam_switch"]},
        "slam_auto_relocalize": {"method": "POST", "path": CLIENT_LINKS["slam_auto_relocalize"]},
        "slam_relocalize": {"method": "POST", "path": CLIENT_LINKS["slam_relocalize"]},
        "slam_track_against_map": {"method": "POST", "path": CLIENT_LINKS["slam_track_against_map"]},
        "bag_start": {"method": "POST", "path": CLIENT_LINKS["bag_start"]},
        "bag_stop": {"method": "POST", "path": CLIENT_LINKS["bag_stop"]},
        "bag_status": {"method": "GET", "path": CLIENT_LINKS["bag_status"]},
        "memory_temporal": {"method": "GET", "path": CLIENT_LINKS["memory_temporal"]},
        "memory_temporal_semantic": {"method": "POST", "path": CLIENT_LINKS["memory_temporal_semantic"]},
        "diagnostic_pack": {"method": "GET", "path": CLIENT_LINKS["diagnostic_pack"]},
        "field_check": {
            "method": "POST",
            "path": CLIENT_LINKS["field_check"],
        },
        "runtime_switch_plan": {
            "method": "POST",
            "path": CLIENT_LINKS["runtime_switch_plan"],
        },
        "runtime_switch": {"method": "POST", "path": CLIENT_LINKS["runtime_switch"]},
        "inspection_acceptance": {
            "method": "POST",
            "path": CLIENT_LINKS["inspection_acceptance"],
        },
        "routecheck_latest": {"method": "GET", "path": CLIENT_LINKS["routecheck_latest"]},
        "real_runtime_evidence_latest": {
            "method": "GET",
            "path": CLIENT_LINKS["real_runtime_evidence_latest"],
        },
        "algorithm_benchmark_latest": {
            "method": "GET",
            "path": CLIENT_LINKS["algorithm_benchmark_latest"],
        },
        "runtime_contract": {"method": "GET", "path": CLIENT_LINKS["runtime_contract"]},
    },
}

PROBE_ENDPOINTS: dict[str, dict[str, str]] = {
    "liveness": {"method": "GET", "path": "/health"},
    "readiness": {"method": "GET", "path": "/ready"},
}


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _mission_summary(mission: Any) -> dict[str, Any]:
    raw = _mapping(mission)
    return {
        "state": raw.get("state", raw.get("status", "idle")),
        "raw": raw,
    }


def _safety_summary(safety: Any) -> dict[str, Any]:
    return safety_summary(safety)


def _map_summary(gw: Any, session: Mapping[str, Any]) -> dict[str, Any]:
    lock = getattr(gw, "_map_cloud_lock", None)
    if lock is None:
        points = getattr(gw, "_map_points", None)
    else:
        with lock:
            points = getattr(gw, "_map_points", None)
    try:
        live_points = len(points) if points is not None else 0
    except TypeError:
        live_points = 0
    return {
        "active": session.get("active_map"),
        "has_live_cloud": live_points > 0,
        "live_points": live_points,
        "live_cloud_frames": int(getattr(gw, "_map_cloud_count", 0)),
        "has_manager": getattr(gw, "_map_mgr", None) is not None,
        "map_has_pcd": bool(session.get("map_has_pcd", False)),
        "map_has_tomogram": bool(session.get("map_has_tomogram", False)),
    }


def _traffic_summary(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._traffic_stats_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except (AttributeError, TypeError):
        pass
    return {
        "sse": {
            "clients": 0,
            "queue_maxsize": None,
            "drop_policy": "drop_oldest",
        },
        "cloud": {
            "clients": 0,
            "queue_maxsize": None,
            "drop_policy": "drop_oldest",
        },
        "recommended_client_rates_hz": {},
    }


def _int_value(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _max_depth(stats: Mapping[str, Any]) -> int:
    depths = stats.get("queue_depths", [])
    if isinstance(depths, list):
        candidates = [_int_value(item) for item in depths]
    else:
        candidates = []
    candidates.append(_int_value(stats.get("max_depth_seen")))
    return max(candidates, default=0)


def _traffic_warnings(traffic: Mapping[str, Any]) -> list[str]:
    warnings: list[str] = []
    sse = _mapping(traffic.get("sse"))
    cloud = _mapping(traffic.get("cloud"))

    if _int_value(sse.get("dropped_events")) > 0:
        warnings.append("sse_events_dropped")
    sse_maxsize = _int_value(sse.get("queue_maxsize"))
    if sse_maxsize > 0 and _max_depth(sse) >= max(1, int(sse_maxsize * 0.75)):
        warnings.append("sse_queue_pressure")

    if _int_value(cloud.get("dropped_frames")) > 0:
        warnings.append("cloud_frames_dropped_latest_only")
    cloud_maxsize = _int_value(cloud.get("queue_maxsize"))
    if cloud_maxsize > 0 and _max_depth(cloud) >= cloud_maxsize:
        warnings.append("cloud_queue_pressure")

    return warnings


def _large_event_policy(gw: Any) -> dict[str, Any]:
    slope_inline = bool(
        getattr(gw, "_sse_slope_payload_enabled", DEFAULT_SSE_SLOPE_PAYLOAD_ENABLED)
    )
    return {
        "raster_min_interval_s": float(
            getattr(gw, "_sse_raster_min_interval_s", DEFAULT_SSE_RASTER_MIN_INTERVAL_S)
        ),
        "costmap_payload": "inline_sse",
        "slope_grid_payload": "inline_sse" if slope_inline else "metadata_sse",
        "point_cloud_payload": "binary_websocket",
        "binary_cloud_endpoint": CLIENT_LINKS["cloud_ws"],
    }


def _client_traffic_policy(
    gw: Any,
    traffic: Mapping[str, Any],
    *,
    usage: str,
) -> dict[str, Any]:
    return {
        "usage": usage,
        "poll_rates_hz": traffic.get("recommended_client_rates_hz", {}),
        "events_endpoint": CLIENT_LINKS["events"],
        "traffic_endpoint": CLIENT_LINKS["traffic"],
        "cloud_endpoint": CLIENT_LINKS["cloud_ws"],
        "large_event_policy": _large_event_policy(gw),
        "backpressure": {
            "sse": _mapping(traffic.get("sse")).get("drop_policy"),
            "cloud": _mapping(traffic.get("cloud")).get("drop_policy"),
        },
    }


def _command_policy(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._command_stats_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except (AttributeError, TypeError):
        pass
    return {
        "idempotency_supported": False,
        "request_id_field": "request_id",
        "client_id_field": "client_id",
        "rate_policy_hz": {},
        "rate_policy_enforcement": "unavailable",
    }


def _auth_summary() -> dict[str, Any]:
    try:
        from gateway.auth import _get_configured_key

        enabled = _get_configured_key() is not None
    except Exception:
        enabled = False
    return {
        "enabled": enabled,
        "scheme": "api_key" if enabled else "none",
        "header": "X-API-Key",
        "query_param": "api_key",
        "cookie": "lingtu_api_key",
        "public_endpoints": [
            "/",
            "/api/v1/auth/login",
            "/api/v1/auth/check",
            "/docs",
            "/redoc",
            "/openapi.json",
        ],
    }


def _feature_flags(gw: Any) -> dict[str, bool]:
    explorer_available = getattr(gw, "_frontier_explorer", None) is not None
    if hasattr(gw, "_explorer_available"):
        explorer_available = bool(gw._explorer_available())
    return {
        "state": True,
        "health": True,
        "devices": True,
        "mapping": True,
        "localization": True,
        "navigation": True,
        "runtime_dataflow": True,
        "runtime_switch_plan": True,
        "runtime_switch": True,
        "algorithm_benchmark": True,
        "exploration": explorer_available,
        "teleop": True,
        "camera_snapshot": True,
        "webrtc": getattr(gw, "_webrtc", None) is not None,
        "scene_graph": True,
        "locations": True,
        "sessions": True,
    }


def _webrtc_summary(gw: Any) -> dict[str, Any]:
    return {
        "available": getattr(gw, "_webrtc", None) is not None,
        "stats": CLIENT_LINKS["webrtc_stats"],
        "offer": CLIENT_LINKS["webrtc_offer"],
        "bitrate": CLIENT_LINKS["webrtc_bitrate"],
        "whep": CLIENT_LINKS["webrtc_whep"],
        "go2rtc_status": CLIENT_LINKS["go2rtc_status"],
    }


def _media_summary(gw: Any) -> dict[str, Any]:
    media = build_media_status(gw)
    webrtc = _webrtc_summary(gw)
    return {
        "events": CLIENT_LINKS["events"],
        "teleop_ws": CLIENT_LINKS["teleop_ws"],
        "camera_ws": CLIENT_LINKS["camera_ws"],
        "cloud_ws": CLIENT_LINKS["cloud_ws"],
        "camera_snapshot": CLIENT_LINKS["camera_snapshot"],
        "webrtc_available": bool(webrtc["available"]),
        "webrtc_stats": CLIENT_LINKS["webrtc_stats"],
        "webrtc_offer": CLIENT_LINKS["webrtc_offer"],
        "webrtc_bitrate": CLIENT_LINKS["webrtc_bitrate"],
        "webrtc_whep": CLIENT_LINKS["webrtc_whep"],
        "go2rtc_status": CLIENT_LINKS["go2rtc_status"],
        "camera": media["camera"],
        "webrtc": webrtc,
    }


def _schema_name(schema: Any) -> str | None:
    raw = _mapping(schema)
    ref = raw.get("$ref")
    if isinstance(ref, str):
        return ref.rsplit("/", 1)[-1]
    for key in ("anyOf", "oneOf"):
        variants = raw.get(key)
        if isinstance(variants, list):
            names = [_schema_name(item) for item in variants]
            joined = "|".join(name for name in names if name)
            if joined:
                return joined
    if raw.get("type") == "object":
        title = raw.get("title")
        if isinstance(title, str) and title:
            return title
    typ = raw.get("type")
    return str(typ) if typ else None


def _openapi_route_signature(app: Any) -> tuple[tuple[str, tuple[str, ...]], ...]:
    routes = getattr(app, "routes", [])
    signature: list[tuple[str, tuple[str, ...]]] = []
    for route in routes:
        path = getattr(route, "path", None)
        if not path:
            continue
        methods = getattr(route, "methods", None) or []
        signature.append(
            (
                str(path),
                tuple(sorted(str(method).upper() for method in methods)),
            )
        )
    return tuple(signature)


def _operation_contracts(gw: Any) -> dict[tuple[str, str], dict[str, Any]]:
    app = getattr(gw, "_app", None)
    if app is None or not hasattr(app, "openapi"):
        return {}
    route_signature = _openapi_route_signature(app)
    with _OPERATION_CONTRACT_CACHE_LOCK:
        cached = getattr(gw, _OPERATION_CONTRACT_CACHE_ATTR, None)
        if (
            isinstance(cached, tuple)
            and len(cached) == 2
            and cached[0] == route_signature
            and isinstance(cached[1], dict)
        ):
            return cached[1]
        try:
            openapi = app.openapi()
        except Exception:
            return {}

        contracts: dict[tuple[str, str], dict[str, Any]] = {}
        for path, methods in _mapping(openapi.get("paths")).items():
            if not isinstance(methods, Mapping):
                continue
            for method, operation in methods.items():
                http_method = str(method).upper()
                if http_method not in {"GET", "POST", "PUT", "PATCH", "DELETE"}:
                    continue
                op = _mapping(operation)
                request_content = (
                    _mapping(op.get("requestBody")).get("content", {})
                    if isinstance(op.get("requestBody"), Mapping)
                    else {}
                )
                request_json = _mapping(
                    _mapping(request_content).get("application/json")
                )
                responses = _mapping(op.get("responses"))
                response_200 = _mapping(responses.get("200"))
                response_content = _mapping(response_200.get("content"))
                response_json = _mapping(response_content.get("application/json"))
                response_schema = _schema_name(response_json.get("schema"))
                if response_schema is None:
                    response_media = _mapping(
                        response_content.get("text/event-stream")
                    )
                    response_schema = _schema_name(response_media.get("schema"))
                contracts[(http_method, str(path))] = {
                    "operation_id": op.get("operationId"),
                    "request_schema": _schema_name(request_json.get("schema")),
                    "response_schema": response_schema,
                    "response_content_types": sorted(response_content.keys()),
                    "status_codes": sorted(str(code) for code in responses.keys()),
                }
        setattr(gw, _OPERATION_CONTRACT_CACHE_ATTR, (route_signature, contracts))
        return contracts


def prewarm_app_capability_contracts(gw: Any) -> bool:
    """Precompute App/Web capabilities metadata and response-model caches."""
    contracts_ready = bool(_operation_contracts(gw))
    try:
        from gateway.schemas import AppCapabilitiesResponse

        payload = build_app_capabilities(gw)
        AppCapabilitiesResponse.model_validate(payload).model_dump(mode="json")
    except (AttributeError, TypeError, ValueError):
        pass
    return contracts_ready


def _enrich_endpoint_specs(
    gw: Any,
    groups: Mapping[str, Mapping[str, Mapping[str, str]]],
) -> dict[str, dict[str, dict[str, Any]]]:
    contracts = _operation_contracts(gw)
    enriched: dict[str, dict[str, dict[str, Any]]] = {}
    for group, endpoints in groups.items():
        enriched[group] = {}
        for name, spec in endpoints.items():
            item: dict[str, Any] = dict(spec)
            method = str(item.get("method", "")).upper()
            lookup_method = "GET" if method == "SSE" else method
            lookup_path = str(item.get("path", "")).split("?", 1)[0]
            contract = contracts.get((lookup_method, lookup_path))
            if contract:
                for key, value in contract.items():
                    if value in (None, []):
                        continue
                    item[key] = list(value) if isinstance(value, list) else value
            enriched[group][name] = item
    return enriched


def build_app_capabilities(gw: Any) -> dict[str, Any]:
    """Return stable client-facing API capabilities and traffic policies."""
    traffic = _traffic_summary(gw)
    now = time.time()
    return {
        "schema_version": APP_CAPABILITIES_SCHEMA_VERSION,
        "ts": now,
        "server": {
            "api_version": "v1",
            "time": now,
        },
        "auth": _auth_summary(),
        "features": _feature_flags(gw),
        "endpoints": _enrich_endpoint_specs(gw, CLIENT_ENDPOINTS),
        "probes": _enrich_endpoint_specs(gw, {"probes": PROBE_ENDPOINTS})["probes"],
        "validation_gates": runtime_validation_gates(),
        "realtime": {
            "events": {
                "path": CLIENT_LINKS["events"],
                "transport": "sse",
                "initial_snapshot": True,
                "heartbeat_s": 1.0,
                "schema_version": SSE_EVENT_SCHEMA_VERSION,
                "event_schema": "SSEEventEnvelope",
                "event_id_field": "event_id",
                "timestamp_field": "ts",
                "heartbeat_type": "ping",
                "snapshot_type": "snapshot",
                "event_types": list(SSE_EVENT_TYPES),
                "diagnostic_event_types": list(SSE_DIAGNOSTIC_EVENT_TYPES),
                "legacy_event_types": list(SSE_LEGACY_EVENT_TYPES),
                "named_events": False,
                "browser_handler": "onmessage",
                "retry_ms": SSE_RETRY_MS,
                "replay_supported": False,
                "last_event_id_header": "Last-Event-ID",
                "drop_policy": traffic.get("sse", {}).get("drop_policy"),
                "large_event_policy": _large_event_policy(gw),
            },
            "teleop": {
                "path": CLIENT_LINKS["teleop_ws"],
                "transport": "websocket",
                "control_messages": ["joy", "stop"],
                "binary_camera_frames": False,
                "legacy_camera_query": "?video=1",
            },
            "camera": {
                "path": CLIENT_LINKS["camera_ws"],
                "transport": "websocket",
                "binary_camera_frames": True,
                "explicit_subscription": True,
            },
            "cloud": {
                "path": CLIENT_LINKS["cloud_ws"],
                "transport": "websocket",
                "binary_point_cloud_frames": True,
                "drop_policy": traffic.get("cloud", {}).get("drop_policy"),
            },
        },
        "client_policy": {
            "poll_rates_hz": traffic.get("recommended_client_rates_hz", {}),
            "commands": _command_policy(gw),
            "retry_safe_when_request_id_present": True,
        },
        "links": dict(CLIENT_LINKS),
    }


def build_app_traffic(gw: Any) -> dict[str, Any]:
    """Return low-frequency App/Web traffic and realtime backpressure status."""
    now = time.time()
    traffic = _traffic_summary(gw)
    warnings = _traffic_warnings(traffic)
    return {
        "schema_version": APP_TRAFFIC_SCHEMA_VERSION,
        "ts": now,
        "server": {
            "api_version": "v1",
            "time": now,
        },
        "status": "degraded" if warnings else "ok",
        "sse": _mapping(traffic.get("sse")),
        "cloud": _mapping(traffic.get("cloud")),
        "recommended_client_rates_hz": traffic.get("recommended_client_rates_hz", {}),
        "client_policy": _client_traffic_policy(
            gw,
            traffic,
            usage="low_frequency_monitoring",
        ),
        "warnings": warnings,
        "links": dict(CLIENT_LINKS),
    }


def build_app_bootstrap(gw: Any) -> dict[str, Any]:
    """Return the first payload a mobile app or web client needs after login."""
    now = time.time()
    with gw._state_lock:
        odometry = gw._odom
        mission = gw._mission
        safety = gw._safety
        mode = gw._mode
        scene_graph_json = gw._sg_json
        path_len = len(gw._last_path)
        teleop_active = gw._teleop_active
        localization_status = getattr(gw, "_localization_status", None)
    teleop_clients = gw._teleop_client_count()

    session = _safe_session(gw)
    icp_quality = float(getattr(gw, "_icp_quality", 0.0))
    localization = build_localization_status_from_parts(
        odometry,
        session,
        icp_quality,
        localization_status,
    )
    navigation = build_navigation_status(gw)
    traffic = _traffic_summary(gw)
    control = dict(navigation.get("control", {}))
    nav_readiness = navigation.get("readiness", {})
    safety_clear = safety_clear_for_motion(safety)
    goal_blockers = list(nav_readiness.get("blockers") or [])
    if not safety_clear and SAFETY_STOP_BLOCKER not in goal_blockers:
        goal_blockers.append(SAFETY_STOP_BLOCKER)
    control.update(
        {
            "teleop": {
                "active": bool(teleop_active),
                "clients": int(teleop_clients),
            },
            "estop_clear": mode != "estop",
            "safety_clear": safety_clear,
            "can_send_commands": (
                mode != "estop"
                and safety_clear
                and not bool(session.get("pending"))
            ),
            "can_send_goal": (
                safety_clear and bool(navigation.get("can_accept_goal", False))
            ),
            "goal_blockers": goal_blockers,
            "command_policy": _command_policy(gw),
        }
    )

    return {
        "schema_version": APP_BOOTSTRAP_SCHEMA_VERSION,
        "ts": now,
        "server": {
            "api_version": "v1",
            "time": now,
        },
        "robot": {
            "online": True,
            "has_odometry": odometry is not None,
        },
        "session": session,
        "mission": _mission_summary(mission),
        "safety": _safety_summary(safety),
        "localization": localization,
        "navigation": navigation,
        "control": control,
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
        "traffic": {
            **traffic,
            "client_policy": _client_traffic_policy(
                gw,
                traffic,
                usage="cold_start_only",
            ),
        },
        "capabilities": _feature_flags(gw),
        "capabilities_endpoint": CLIENT_LINKS["capabilities"],
        "links": dict(CLIENT_LINKS),
    }
