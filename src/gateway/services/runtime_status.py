"""App/Web runtime status builders for localization and navigation."""

from __future__ import annotations

import logging
import os
import json
import math
import time
from dataclasses import asdict
from collections.abc import Mapping
from typing import Any

from core.runtime_policy import (
    backend_capability_defaults as _backend_capability_defaults,
)
from core.runtime_interface import REAL_RUNTIME_CONTRACT, map_frame_id
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_stop_active,
    safety_summary,
)

logger = logging.getLogger(__name__)


LOCALIZATION_STATUS_SCHEMA_VERSION = 1
NAVIGATION_STATUS_SCHEMA_VERSION = 1
STATUS_MAP_FRAME_ID = map_frame_id()

PATH_ENDPOINT = "/api/v1/path"

MISSION_ACTIVE_STATES = {"PLANNING", "EXECUTING", "PATROLLING"}
MISSION_TERMINAL_STATES = {"SUCCESS", "FAILED", "CANCELLED"}

CONTROL_SOURCE_META: dict[str, dict[str, Any]] = {
    "teleop": {
        "label": "Teleop joystick",
        "category": "manual",
        "owner": "teleop",
        "preempts_autonomy": True,
    },
    "visual_servo": {
        "label": "Visual servo",
        "category": "autonomy_assist",
        "owner": "visual_servo",
        "preempts_autonomy": True,
    },
    "recovery": {
        "label": "Navigation recovery",
        "category": "autonomy_recovery",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
    "path_follower": {
        "label": "Path follower",
        "category": "autonomy",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
}

TRACKING_STATES = {"TRACKING", "OK", "READY"}
LOST_STATES = {"LOST", "UNINIT", "UNINITIALIZED"}
DEGRADED_STATES = {"DEGRADED", "FALLBACK_GNSS_ONLY"}
ADVISORY_DEGENERACY = {"MILD"}
BAD_DEGENERACY = {"MODERATE", "SEVERE", "CRITICAL"}
GOOD_LOCALIZER_HEALTH = {
    "",
    "UNKNOWN",
    "LOCKED",
    "RECOVERED",
    "OK",
    "READY",
    "LIO_TRACKING",
    "LIO_RECOVERED",
}
POSE_FRESH_MAX_ODOM_AGE_MS = 2000.0


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _finite_float(value: Any) -> float | None:
    parsed = _as_float(value, None)
    if parsed is None or not math.isfinite(parsed):
        return None
    return parsed


def _session_mode(session: Mapping[str, Any]) -> str:
    mode = str(session.get("mode") or "unknown").strip().lower()
    return mode or "unknown"


def _point_payload(
    value: Any,
    *,
    frame_id: str = STATUS_MAP_FRAME_ID,
    ts: float | None = None,
) -> dict[str, Any] | None:
    if value is None:
        return None
    metadata: dict[str, Any] = {}
    yaw: float | None = None
    point_frame = frame_id
    point_ts = ts

    if isinstance(value, Mapping):
        x = _finite_float(value.get("x"))
        y = _finite_float(value.get("y"))
        z = _finite_float(value.get("z", 0.0))
        yaw = _finite_float(value.get("yaw"))
        point_frame = str(value.get("frame_id") or frame_id)
        point_ts = _finite_float(value.get("ts")) or ts
        if isinstance(value.get("metadata"), Mapping):
            metadata = dict(value["metadata"])
    else:
        if hasattr(value, "tolist"):
            value = value.tolist()
        try:
            seq = list(value)
        except TypeError:
            return None
        if len(seq) < 2:
            return None
        x = _finite_float(seq[0])
        y = _finite_float(seq[1])
        z = _finite_float(seq[2]) if len(seq) > 2 else 0.0
        yaw = _finite_float(seq[3]) if len(seq) > 3 else None

    if x is None or y is None:
        return None
    if z is None:
        z = 0.0
    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "frame_id": point_frame,
        "ts": point_ts,
        "metadata": metadata,
    }


def _distance_xy(
    point_a: Mapping[str, Any] | None,
    point_b: Mapping[str, Any] | None,
) -> float | None:
    if not point_a or not point_b:
        return None
    ax = _finite_float(point_a.get("x"))
    ay = _finite_float(point_a.get("y"))
    bx = _finite_float(point_b.get("x"))
    by = _finite_float(point_b.get("y"))
    if ax is None or ay is None or bx is None or by is None:
        return None
    return round(math.hypot(ax - bx, ay - by), 3)


def _as_optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "y"}:
            return True
        if lowered in {"false", "0", "no", "n"}:
            return False
    return None


def backend_capability_defaults(backend_name: str | None) -> dict[str, Any]:
    return _backend_capability_defaults(backend_name)


def _reason_code_from_text(prefix: str, text: str) -> str:
    words: list[str] = []
    current: list[str] = []
    for ch in text.lower():
        if ch.isalnum():
            current.append(ch)
        elif current:
            words.append("".join(current))
            current = []
    if current:
        words.append("".join(current))
    if not words:
        return prefix
    return f"{prefix}_{'_'.join(words[:6])}"


def safe_session(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._session_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        logger.debug("safe_session_snapshot failed", exc_info=True)
    return {
        "mode": getattr(gw, "_session_mode", "unknown"),
        "active_map": getattr(gw, "_session_map", None),
        "pending": bool(getattr(gw, "_session_pending", False)),
        "error": "session_snapshot_unavailable",
        "can_start_mapping": False,
        "can_start_navigating": False,
        "can_start_exploring": False,
        "can_end": False,
    }


def safe_lease(gw: Any) -> dict[str, Any]:
    lease = getattr(gw, "_lease", None)
    if hasattr(lease, "to_dict"):
        try:
            data = lease.to_dict()
            if isinstance(data, Mapping):
                return dict(data)
        except (AttributeError, TypeError):
            pass
    return {}


def _reported_state(raw: Any) -> str:
    state = str(raw or "").strip()
    return state.upper() if state else ""


def _active_recovery_signal(raw: Any) -> str:
    signal = _reported_state(raw)
    if signal in {"", "NONE", "RECOVERED"}:
        return ""
    return signal


def localizer_algorithm_healthy(
    diagnostics: Mapping[str, Any],
    icp_quality: float,
) -> bool:
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    localizer_health = _reported_state(diagnostics.get("localizer_health"))
    recovery_signal = _active_recovery_signal(diagnostics.get("recovery_signal"))
    health_source = str(
        diagnostics.get("health_source")
        or diagnostics.get("localizer_health_source")
        or ""
    ).strip().lower()
    icp_fitness = _as_float(diagnostics.get("icp_fitness"), icp_quality)
    health_fitness = _as_float(diagnostics.get("localizer_health_fitness"), None)
    icp_ok = any(
        value is not None and 0.0 < value < 0.5
        for value in (icp_fitness, health_fitness)
    )
    pose_fresh, _ = _pose_freshness(diagnostics)
    cloud_fresh = _cloud_fresh(diagnostics)
    odom_cloud_ok = (
        health_source == "odom_map_cloud"
        and pose_fresh is not False
        and cloud_fresh
        and reported in {"", *TRACKING_STATES}
    )

    return (
        reported in {"", *TRACKING_STATES}
        and degeneracy not in BAD_DEGENERACY
        and localizer_health in GOOD_LOCALIZER_HEALTH
        and not recovery_signal
        and cloud_fresh
        and (icp_ok or odom_cloud_ok)
    )


def _localizer_algorithm_healthy(
    diagnostics: Mapping[str, Any],
    icp_quality: float,
) -> bool:
    return localizer_algorithm_healthy(diagnostics, icp_quality)


def classify_pose_freshness(diagnostics: Mapping[str, Any]) -> tuple[bool | None, str]:
    reported = _reported_state(diagnostics.get("state"))
    explicit = _as_optional_bool(diagnostics.get("pose_fresh"))
    odom_age_ms = _as_float(diagnostics.get("odom_age_ms"), None)
    confidence = diagnostics.get("confidence")

    if reported in LOST_STATES:
        return False, "lost"
    if explicit is not None:
        return explicit, "fresh" if explicit else "stale"
    if odom_age_ms is not None and odom_age_ms >= 0.0:
        fresh = odom_age_ms <= POSE_FRESH_MAX_ODOM_AGE_MS
        return fresh, "fresh" if fresh else "stale"
    if isinstance(confidence, (int, float)):
        return confidence >= 0.5, "fresh" if confidence >= 0.5 else "stale"
    return None, "unknown"


def _pose_freshness(diagnostics: Mapping[str, Any]) -> tuple[bool | None, str]:
    return classify_pose_freshness(diagnostics)


def _cloud_fresh(diagnostics: Mapping[str, Any]) -> bool:
    explicit = _as_optional_bool(diagnostics.get("map_cloud_fresh"))
    if explicit is not None:
        return explicit
    cloud_age_ms = _as_float(diagnostics.get("cloud_age_ms"), None)
    if cloud_age_ms is not None and cloud_age_ms >= 0.0:
        return cloud_age_ms <= POSE_FRESH_MAX_ODOM_AGE_MS
    return True


def _localization_state(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    diagnostics: Mapping[str, Any],
) -> tuple[str, list[str]]:
    reasons: list[str] = []
    mode = str(session.get("mode", "unknown"))
    ready = bool(session.get("localizer_ready", False))
    pending = bool(session.get("pending", False))
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    localizer_health = _reported_state(diagnostics.get("localizer_health"))
    recovery_signal = _active_recovery_signal(diagnostics.get("recovery_signal"))
    diagnostics.get("confidence")
    algorithm_healthy = _localizer_algorithm_healthy(diagnostics, icp_quality)
    pose_fresh, _ = _pose_freshness(diagnostics)

    if odometry is None:
        reasons.append("odometry_missing")
        return "no_odometry", reasons

    if "RELOCAL" in reported or (pending and mode == "navigating"):
        reasons.append("relocalization_pending")
        return "relocalizing", reasons

    if reported in LOST_STATES:
        reasons.append(f"reported_state:{reported.lower()}")
        return "lost", reasons

    if localizer_health == "LOST":
        reasons.append("localizer_health:lost")
        return "lost", reasons

    if recovery_signal:
        reasons.append(f"recovery_signal:{recovery_signal.lower()}")
        return "degraded", reasons

    if (
        reported in DEGRADED_STATES
        or degeneracy in BAD_DEGENERACY
        or localizer_health == "DEGRADED"
        or pose_fresh is False
    ):
        if reported:
            reasons.append(f"reported_state:{reported.lower()}")
        if degeneracy and degeneracy != "NONE":
            reasons.append(f"degeneracy:{degeneracy.lower()}")
        if localizer_health == "DEGRADED":
            reasons.append("localizer_health:degraded")
        if pose_fresh is False:
            reasons.append("stale_odometry" if algorithm_healthy else "low_confidence")
        return "degraded", reasons

    if ready:
        return "ready", reasons

    if mode == "navigating":
        reasons.append("localizer_not_ready")
        return "initializing" if icp_quality <= 0.0 else "degraded", reasons

    if reported in TRACKING_STATES:
        return "tracking", reasons

    return "tracking", reasons


def build_localization_status_from_parts(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    status: Any,
) -> dict[str, Any]:
    diagnostics = _mapping(status)
    diag_received_mono = _as_float(diagnostics.get("_gateway_received_mono"))
    diag_age_ms = (
        round(max(0.0, time.monotonic() - diag_received_mono) * 1000.0, 1)
        if diag_received_mono is not None
        else None
    )
    algorithm_healthy = _localizer_algorithm_healthy(diagnostics, float(icp_quality))
    pose_fresh, pose_freshness = _pose_freshness(diagnostics)
    state, reasons = _localization_state(
        odometry,
        session,
        float(icp_quality),
        diagnostics,
    )
    ready = state == "ready"
    backend = (
        diagnostics.get("backend")
        or diagnostics.get("slam_profile")
        or session.get("slam_profile")
    )
    backend_name = str(backend or "").strip().lower()
    capability_defaults = backend_capability_defaults(backend_name)
    relocalization_supported = _as_optional_bool(
        diagnostics.get("relocalization_supported")
    )
    if relocalization_supported is None:
        relocalization_supported = bool(
            capability_defaults["relocalization_supported"]
        )
    saved_map_relocalization_supported = _as_optional_bool(
        diagnostics.get("saved_map_relocalization_supported")
    )
    if saved_map_relocalization_supported is None:
        saved_map_relocalization_supported = relocalization_supported
    restart_recovery_supported = _as_optional_bool(
        diagnostics.get("restart_recovery_supported")
    )
    if restart_recovery_supported is None:
        restart_recovery_supported = bool(
            capability_defaults["restart_recovery_supported"]
        )
    recovery_method = diagnostics.get("recovery_method")
    if not recovery_method:
        recovery_method = capability_defaults["recovery_method"]
    map_save_supported = _as_optional_bool(diagnostics.get("map_save_supported"))
    if map_save_supported is None:
        map_save_supported = bool(capability_defaults["map_save_supported"])
    map_save_source = diagnostics.get("map_save_source")
    if map_save_source is None:
        map_save_source = capability_defaults["map_save_source"]
    runtime_boundary = _runtime_boundary_status()
    frames = _localization_frame_summary(
        odometry,
        diagnostics,
        runtime_boundary,
    )
    return {
        "schema_version": LOCALIZATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "ready": ready,
        "has_odometry": odometry is not None,
        "session_mode": session.get("mode"),
        "active_map": session.get("active_map"),
        "icp_quality": float(icp_quality),
        "reported_state": diagnostics.get("state"),
        "confidence": diagnostics.get("confidence"),
        "algorithm_healthy": algorithm_healthy,
        "backend": backend,
        "health_source": diagnostics.get("health_source"),
        "pose_fresh": pose_fresh,
        "pose_freshness": pose_freshness,
        "stale_odometry": pose_fresh is False and algorithm_healthy,
        "odom_age_ms": _as_float(diagnostics.get("odom_age_ms")),
        "cloud_age_ms": _as_float(diagnostics.get("cloud_age_ms")),
        "degeneracy": diagnostics.get("degeneracy"),
        "icp_fitness": _as_float(diagnostics.get("icp_fitness")),
        "effective_ratio": _as_float(diagnostics.get("effective_ratio")),
        "condition_number": _as_float(diagnostics.get("condition_number")),
        "degenerate_dof_count": _as_optional_int(
            diagnostics.get("degenerate_dof_count")
        ),
        "pos_cov_trace": _as_float(diagnostics.get("pos_cov_trace")),
        "ieskf_iter_num": _as_optional_int(diagnostics.get("ieskf_iter_num")),
        "ieskf_converged": _as_optional_bool(diagnostics.get("ieskf_converged")),
        "map_cloud_fresh": _as_optional_bool(diagnostics.get("map_cloud_fresh")),
        "map_state": diagnostics.get("map_state"),
        "map_save_supported": map_save_supported,
        "map_save_source": map_save_source,
        "relocalization_supported": relocalization_supported,
        "saved_map_relocalization_supported": saved_map_relocalization_supported,
        "restart_recovery_supported": restart_recovery_supported,
        "recovery_method": recovery_method,
        "relocalization_state": diagnostics.get("relocalization_state"),
        "recovery_signal": diagnostics.get("recovery_signal"),
        "recovery_action": diagnostics.get("recovery_action"),
        "localizer_health": diagnostics.get("localizer_health"),
        "localizer_health_raw": diagnostics.get("localizer_health_raw"),
        "localizer_health_source": diagnostics.get("localizer_health_source"),
        "localizer_health_topic_age_ms": _as_float(
            diagnostics.get("localizer_health_topic_age_ms")
        ),
        "localizer_health_fitness": _as_float(
            diagnostics.get("localizer_health_fitness")
        ),
        "localizer_health_iter": _as_optional_int(
            diagnostics.get("localizer_health_iter")
        ),
        "localizer_health_cov_trace": _as_float(
            diagnostics.get("localizer_health_cov_trace")
        ),
        "ts": diagnostics.get("ts"),
        "diag_received_ts": _as_float(diagnostics.get("_gateway_received_ts")),
        "diag_age_ms": diag_age_ms,
        "runtime": runtime_boundary,
        "frames": frames,
        "can_relocalize": (
            saved_map_relocalization_supported
            and state in {"degraded", "lost"}
            and odometry is not None
        ),
        "reasons": reasons,
        "raw": diagnostics,
    }


def build_localization_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        odometry = gw._odom
        localization_status = getattr(gw, "_localization_status", None)

    session = safe_session(gw)
    return build_localization_status_from_parts(
        odometry,
        session,
        float(getattr(gw, "_icp_quality", 0.0)),
        localization_status,
    )


def _cmd_vel_health(gw: Any) -> dict[str, Any]:
    mux = getattr(gw, "_cmd_vel_mux", None)
    modules = getattr(gw, "_all_modules", None) or {}
    if mux is None:
        mux = modules.get("CmdVelMux")
    if mux is None:
        for name, module in modules.items():
            token = str(name).lower()
            class_token = module.__class__.__name__.lower()
            if (
                "cmdvelmux" in token
                or "cmd_vel_mux" in token
                or "cmdvelmux" in class_token
                or "cmd_vel_mux" in class_token
            ):
                mux = module
                break
    if mux is None:
        return {"active_source": "unknown", "sources": {}, "available": False}
    try:
        health = mux.health() if hasattr(mux, "health") else {}
        if isinstance(health, Mapping):
            data = dict(health)
            data["available"] = True
            data.setdefault("active_source", "unknown")
            data.setdefault("sources", {})
            return data
    except Exception as exc:
        return {
            "active_source": "unknown",
            "sources": {},
            "available": False,
            "error": str(exc),
        }
    return {"active_source": "unknown", "sources": {}, "available": False}


def _navigation_module_status(gw: Any) -> dict[str, Any]:
    modules = getattr(gw, "_all_modules", None) or {}
    candidates = []
    injected = getattr(gw, "_navigation_module", None)
    if injected is not None:
        candidates.append(injected)
    direct = modules.get("NavigationModule")
    if direct is not None and not any(direct is candidate for candidate in candidates):
        candidates.append(direct)
    for name, module in modules.items():
        if module is direct or any(module is candidate for candidate in candidates):
            continue
        token = str(name).lower()
        class_token = module.__class__.__name__.lower()
        if "navigationmodule" in token or "navigation_module" in token:
            candidates.append(module)
            continue
        if "navigationmodule" in class_token or "navigation_module" in class_token:
            candidates.append(module)

    for module in candidates:
        get_status = getattr(module, "get_navigation_status", None)
        if not callable(get_status):
            continue
        try:
            raw = get_status()
            if isinstance(raw, str):
                raw = json.loads(raw)
            status = _mapping(raw)
            if status:
                return status
        except Exception:
            continue
    return {}


def _control_summary(
    *,
    mode: str,
    lease: Mapping[str, Any],
    mission_state: str,
    cmd_vel: Mapping[str, Any],
) -> dict[str, Any]:
    active_source = str(cmd_vel.get("active_source") or "none")
    sources = _mapping(cmd_vel.get("sources"))
    active_source_health = _mapping(sources.get(active_source))
    meta = CONTROL_SOURCE_META.get(active_source, {})
    if active_source == "none":
        meta = {
            "label": "No active command source",
            "category": "none",
            "owner": "none",
            "preempts_autonomy": False,
        }
    elif not meta:
        meta = {
            "label": active_source,
            "category": "unknown",
            "owner": "unknown",
            "preempts_autonomy": False,
        }

    autonomy_requested = mission_state in MISSION_ACTIVE_STATES
    manual_override = active_source == "teleop"
    preempting_autonomy = bool(
        autonomy_requested and meta.get("preempts_autonomy", False)
    )

    return {
        "mode": mode,
        "lease": dict(lease),
        "active_cmd_source": active_source,
        "command_owner": meta["owner"],
        "source_category": meta["category"],
        "manual_override": manual_override,
        "autonomy_requested": autonomy_requested,
        "preempting_autonomy": preempting_autonomy,
        "mux_available": bool(cmd_vel.get("available", False)),
        "active_source": {
            "name": active_source,
            "label": meta["label"],
            "category": meta["category"],
            "owner": meta["owner"],
            "priority": active_source_health.get("priority"),
            "active": active_source_health.get("active", active_source != "none"),
            "age_ms": active_source_health.get("age_ms"),
        },
        "sources": sources,
        "cmd_vel_mux": dict(cmd_vel),
    }


def _progress_summary(
    *,
    state: str,
    wp_index: int,
    wp_total: int,
    path_points: int,
    replan_count: int,
) -> dict[str, Any]:
    if state == "SUCCESS":
        fraction = 1.0
    elif wp_total > 0:
        fraction = max(0.0, min(1.0, wp_index / wp_total))
    else:
        fraction = 0.0

    return {
        "wp_index": wp_index,
        "wp_total": wp_total,
        "fraction": round(fraction, 4),
        "path_points": path_points,
        "replan_count": replan_count,
        "active": state in MISSION_ACTIVE_STATES,
        "terminal": state in MISSION_TERMINAL_STATES,
    }


def _target_summary(
    mission: Mapping[str, Any],
    odometry: Mapping[str, Any] | None,
    *,
    wp_index: int,
    wp_total: int,
    ts: float,
) -> dict[str, Any]:
    robot = _point_payload(odometry, ts=ts) if odometry else None
    goal = _point_payload(mission.get("goal"), ts=ts)
    current_waypoint = _point_payload(
        mission.get("current_waypoint") or mission.get("waypoint"),
        ts=ts,
    )
    live_goal_distance = _distance_xy(robot, goal)
    distance_to_goal = (
        live_goal_distance
        if live_goal_distance is not None
        else _finite_float(mission.get("distance_to_goal_m"))
    )
    live_waypoint_distance = _distance_xy(robot, current_waypoint)
    waypoint_distance = (
        live_waypoint_distance
        if live_waypoint_distance is not None
        else _finite_float(mission.get("active_waypoint_distance_m"))
    )
    remaining_waypoints = _as_optional_int(mission.get("remaining_waypoints"))
    if remaining_waypoints is None and wp_total:
        remaining_waypoints = max(0, wp_total - wp_index)

    return {
        "goal": goal,
        "current_waypoint": current_waypoint,
        "distance_to_goal_m": distance_to_goal,
        "active_waypoint_distance_m": waypoint_distance,
        "remaining_waypoints": remaining_waypoints,
    }


def _speed_policy_summary(
    mission: Mapping[str, Any],
    localization: Mapping[str, Any],
    speed_scale: float | None,
) -> dict[str, Any]:
    raw_policy = _mapping(mission.get("speed_policy"))
    scale = _finite_float(raw_policy.get("scale"))
    if scale is None:
        scale = speed_scale
    reason = (
        raw_policy.get("reason")
        or mission.get("speed_policy_reason")
        or mission.get("degeneracy")
        or localization.get("state")
    )
    if reason is not None:
        reason = str(reason)

    mode = str(raw_policy.get("mode") or "").strip().lower()
    if mode not in {"normal", "cautious", "restricted", "hold"}:
        if scale is None:
            mode = "unknown"
        elif scale <= 0.0:
            mode = "hold"
        elif scale < 0.5:
            mode = "restricted"
        elif scale < 1.0:
            mode = "cautious"
        else:
            mode = "normal"

    applied = raw_policy.get("applied")
    if not isinstance(applied, bool):
        applied = bool(scale is not None and scale < 1.0)

    return {
        "scale": scale,
        "mode": mode,
        "reason": reason,
        "source": str(raw_policy.get("source") or "mission_status"),
        "applied": applied,
    }


def _current_speed_mps(odometry: Mapping[str, Any] | None) -> float | None:
    if not odometry:
        return None
    vx = _finite_float(odometry.get("vx")) or 0.0
    vy = _finite_float(odometry.get("vy")) or 0.0
    return round(math.hypot(vx, vy), 3)


def _motion_summary(
    *,
    odometry: Mapping[str, Any] | None,
    speed_scale: float | None,
    speed_policy: Mapping[str, Any],
    control: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "current_speed_mps": _current_speed_mps(odometry),
        "speed_scale": speed_scale,
        "speed_policy": dict(speed_policy),
        "active_cmd_source": str(control.get("active_cmd_source") or "none"),
        "command_owner": str(control.get("command_owner") or "unknown"),
    }


def _feedback_summary(
    *,
    state: str,
    can_accept_goal: bool,
    readiness: Mapping[str, Any],
    reason_codes: list[str],
) -> dict[str, Any]:
    blockers = list(readiness.get("blockers") or [])
    advisories = list(readiness.get("advisories") or [])
    if blockers:
        next_action = "resolve_blockers"
        primary = "Navigation is blocked."
    elif state in {"EXECUTING", "PATROLLING"}:
        next_action = "monitor_progress"
        primary = "Navigation is following the planned path."
    elif state == "PLANNING":
        next_action = "wait_for_plan"
        primary = "Navigation is planning."
    elif state == "SUCCESS":
        next_action = "choose_goal"
        primary = "Destination reached."
    elif state == "CANCELLED":
        next_action = "choose_goal" if can_accept_goal else "resolve_blockers"
        primary = "Navigation mission was cancelled."
    elif state in {"FAILED", "STUCK"}:
        next_action = "inspect_failure"
        primary = "Navigation needs attention before retrying."
    elif can_accept_goal:
        next_action = "choose_goal"
        primary = "Ready for a navigation goal."
    else:
        next_action = "monitor_readiness"
        primary = "Navigation is not ready for a goal yet."
    return {
        "next_action": next_action,
        "primary": primary,
        "blockers": blockers,
        "advisories": advisories,
        "reason_codes": reason_codes,
    }


def _navigation_reason_codes(
    *,
    state: str,
    failure_reason: str,
    has_odometry: bool,
    mode: str,
    safety: Any,
    session: Mapping[str, Any],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
    frames: Mapping[str, Any],
    map_artifact_gate: Mapping[str, Any],
    real_runtime_evidence: Mapping[str, Any],
) -> list[str]:
    codes: list[str] = []
    if not has_odometry:
        codes.append("odometry_missing")
    for mismatch in frames.get("mismatches", []):
        if isinstance(mismatch, Mapping):
            source = str(mismatch.get("source") or "").strip()
            if source:
                codes.append(f"frame_mismatch_{source}")
    if mode == "estop":
        codes.append("estop_active")
    if safety_stop_active(safety):
        codes.append(SAFETY_STOP_BLOCKER)
    if bool(session.get("pending", False)):
        codes.append("session_transition_pending")
    if _session_mode(session) not in {"navigating", "exploring"}:
        codes.append("navigation_session_inactive")

    localization_state = str(localization.get("state") or "unknown")
    if localization_state in {"degraded", "lost", "relocalizing", "initializing"}:
        codes.append(f"localization_{localization_state}")
    if _active_recovery_signal(localization.get("recovery_signal")):
        codes.append("localization_recovery_active")
    localization_degeneracy = _reported_state(localization.get("degeneracy"))
    if localization_degeneracy in ADVISORY_DEGENERACY:
        codes.append(f"localization_{localization_degeneracy.lower()}_degeneracy")
    if (
        localization.get("pose_fresh") is False
        and bool(localization.get("algorithm_healthy", False))
    ):
        codes.append("pose_stale")

    if state == "STUCK":
        codes.append("mission_stuck")
    elif state == "FAILED":
        codes.append("mission_failed")
    elif state == "CANCELLED":
        codes.append("mission_cancelled")

    if failure_reason:
        codes.append(_reason_code_from_text("failure", failure_reason))

    if control.get("preempting_autonomy"):
        codes.append(f"control_preempted_by_{control.get('active_cmd_source')}")
    if not control.get("mux_available", False):
        codes.append("cmd_vel_mux_unavailable")
    if (
        map_artifact_gate.get("required") is True
        and map_artifact_gate.get("ok") is not True
    ):
        codes.append("map_artifact_gate_failed")
    if (
        real_runtime_evidence.get("required") is True
        and real_runtime_evidence.get("ok") is not True
    ):
        codes.append("real_runtime_evidence_missing_or_stale")

    return list(dict.fromkeys(codes))


_NAVIGATION_BLOCKER_CODES = {
    "odometry_missing",
    "frame_mismatch_odometry",
    "frame_mismatch_costmap",
    "frame_mismatch_goal",
    "estop_active",
    SAFETY_STOP_BLOCKER,
    "session_transition_pending",
    "navigation_session_inactive",
    "localization_lost",
    "localization_relocalizing",
    "localization_initializing",
    "localization_recovery_active",
    "pose_stale",
    "map_artifact_gate_failed",
    "real_runtime_evidence_missing_or_stale",
}


def _map_artifact_gate_status(nav_runtime: Mapping[str, Any]) -> dict[str, Any]:
    gate = _mapping(nav_runtime.get("map_artifact_gate"))
    if not gate:
        return {
            "required": False,
            "ok": True,
            "reason": "not_reported",
            "blockers": [],
        }
    gate.setdefault("required", False)
    gate.setdefault("ok", True if not gate.get("required") else False)
    gate.setdefault("blockers", [])
    return gate


def _real_runtime_evidence_status(session: Mapping[str, Any]) -> dict[str, Any]:
    from core.runtime_interface import canonical_data_source_name

    runtime_contract = (
        os.environ.get("LINGTU_RUNTIME_CONTRACT")
        or os.environ.get("LINGTU_DATA_SOURCE")
    )
    runtime_contract = canonical_data_source_name(runtime_contract)
    required = (
        runtime_contract == REAL_RUNTIME_CONTRACT
        and _session_mode(session) in {"navigating", "exploring"}
    )
    if not required:
        return {
            "required": False,
            "ok": None,
            "runtime_contract": runtime_contract,
            "reason": "not_required_for_current_runtime",
            "blockers": [],
        }
    try:
        from gateway.routes.diagnostics import (
            build_real_runtime_evidence_latest_summary,
        )

        evidence = build_real_runtime_evidence_latest_summary()
        evidence["required"] = True
        return evidence
    except Exception as exc:
        return {
            "required": True,
            "ok": False,
            "runtime_contract": runtime_contract,
            "reason": "real_runtime_evidence_status_error",
            "blockers": [f"real-runtime-evidence status error: {exc}"],
        }


def _frame_id(value: Any) -> str | None:
    if value is None:
        return None
    frame = str(value).strip()
    return frame or None


def _frame_from_payload(value: Any) -> str | None:
    if not isinstance(value, Mapping):
        return None
    frame = _frame_id(value.get("frame_id") or value.get("frame"))
    if frame:
        return frame
    header = value.get("header")
    if isinstance(header, Mapping):
        return _frame_id(header.get("frame_id") or header.get("frame"))
    return None


def _frame_mismatch(
    source: str,
    frame: str | None,
    expected_frames: tuple[str, ...],
) -> dict[str, str] | None:
    from core.runtime_interface import normalize_frame_id

    normalized = normalize_frame_id(frame)
    if not normalized or normalized == "unknown" or normalized in expected_frames:
        return None
    expected = ",".join(expected_frames) if expected_frames else "unknown"
    return {
        "source": source,
        "expected_frame": expected,
        "received_frame": normalized,
    }


def _navigation_frame_summary(
    mission: Mapping[str, Any],
    odometry: Any,
) -> dict[str, Any]:
    from core.runtime_interface import TOPICS
    from core.runtime_interface import canonical_data_source_name
    from core.runtime_interface import normalize_frame_id
    from core.runtime_interface import runtime_topic_default_frame_id

    runtime_contract = (
        os.environ.get("LINGTU_RUNTIME_CONTRACT")
        or os.environ.get("LINGTU_DATA_SOURCE")
    )
    runtime_contract = canonical_data_source_name(runtime_contract)
    default_planning_frame = runtime_topic_default_frame_id(
        runtime_contract,
        TOPICS.global_path,
    )
    planning_frame_id = (
        _frame_id(mission.get("planning_frame_id") or mission.get("frame_id"))
        or default_planning_frame
    )
    odom_frame_id = (
        _frame_id(mission.get("odom_frame_id"))
        or _frame_from_payload(odometry)
        or "unknown"
    )
    costmap_frame_id = _frame_id(mission.get("costmap_frame_id")) or "unknown"
    goal_frame_id = (
        _frame_id(mission.get("goal_frame_id"))
        or _frame_from_payload(mission.get("goal"))
    )
    planning_frame = normalize_frame_id(planning_frame_id) or default_planning_frame
    odometry_expected = (planning_frame,)
    planning_expected = (planning_frame,)
    mismatches: list[dict[str, str]] = []
    for source, frame, expected_frames in (
        ("odometry", odom_frame_id, odometry_expected),
        ("costmap", costmap_frame_id, planning_expected),
        ("goal", goal_frame_id, planning_expected),
    ):
        mismatch = _frame_mismatch(source, frame, expected_frames)
        if mismatch:
            mismatches.append(mismatch)
    return {
        "planning_frame_id": planning_frame_id,
        "odom_frame_id": odom_frame_id,
        "costmap_frame_id": costmap_frame_id,
        "goal_frame_id": goal_frame_id,
        "odometry_expected_frame_ids": list(odometry_expected),
        "ok": not mismatches,
        "mismatches": mismatches,
    }


def _runtime_boundary_status() -> dict[str, Any]:
    from core.blueprints.runtime_endpoint import canonical_runtime_endpoint_name
    from core.runtime_interface import FRAMES
    from core.runtime_interface import RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES
    from core.runtime_interface import canonical_data_source_name
    from core.runtime_interface import resolved_runtime_data_flow
    from core.runtime_interface import runtime_data_flow_topics
    from core.runtime_interface import runtime_contract_manifest
    from core.runtime_interface import runtime_required_topic_frame_ids
    from core.runtime_interface import runtime_topic_allowed_frame_ids
    from core.runtime_interface import runtime_topic_default_frame_ids

    env = {
        "profile": os.environ.get("LINGTU_PROFILE"),
        "endpoint": os.environ.get("LINGTU_ENDPOINT"),
        "data_source": os.environ.get("LINGTU_DATA_SOURCE"),
        "runtime_contract": os.environ.get("LINGTU_RUNTIME_CONTRACT"),
        "command_sink": os.environ.get("LINGTU_COMMAND_SINK"),
        "simulation_only": os.environ.get("LINGTU_SIMULATION_ONLY"),
    }
    declared = any(value not in (None, "") for value in env.values())
    endpoint = (
        canonical_runtime_endpoint_name(env["endpoint"])
        if env["endpoint"]
        else env["endpoint"]
    )
    data_source = canonical_data_source_name(env["data_source"])
    runtime_contract = canonical_data_source_name(env["runtime_contract"])
    command_sink = env["command_sink"]
    blockers: list[str] = []
    manifest: dict[str, Any] = {}
    data_sources: Mapping[str, Any] = {}
    source: dict[str, Any] = {}
    expected_command_sink: str | None = None
    resolved_flow: list[dict[str, Any]] = []
    stage_algorithm_interfaces: dict[str, list[str]] = {}
    data_flow_topics: list[str] = []
    topic_allowed_frames: dict[str, list[str]] = {}
    topic_default_frames: dict[str, str] = {}
    required_topic_frame_ids: list[str] = []

    if declared and not data_source:
        blockers.append("data_source_missing")
    if runtime_contract and data_source and runtime_contract != data_source:
        blockers.append("runtime_contract_data_source_mismatch")

    if data_source or runtime_contract:
        manifest = runtime_contract_manifest()
        raw_data_sources = manifest.get("data_sources", {})
        if isinstance(raw_data_sources, Mapping):
            data_sources = raw_data_sources

    if data_source:
        source = _mapping(data_sources.get(data_source))
        if source:
            expected_command_sink = str(source.get("command_sink") or "")
            resolved_flow = [
                asdict(stage)
                for stage in resolved_runtime_data_flow(data_source)
            ]
            stage_algorithm_interfaces = {
                name: list(interfaces)
                for name, interfaces in (
                    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
                )
            }
            data_flow_topics = list(runtime_data_flow_topics(data_source))
        else:
            blockers.append("data_source_unknown")

    topic_contract = runtime_contract or data_source
    if topic_contract:
        if topic_contract in data_sources:
            topic_allowed_frames = {
                topic: list(frame_ids)
                for topic, frame_ids in runtime_topic_allowed_frame_ids(
                    topic_contract
                ).items()
            }
            topic_default_frames = dict(runtime_topic_default_frame_ids(topic_contract))
            required_topic_frame_ids = list(
                runtime_required_topic_frame_ids(topic_contract)
            )
        else:
            blockers.append("topic_frame_contract_unavailable")

    if command_sink and expected_command_sink and command_sink != expected_command_sink:
        blockers.append("command_sink_mismatch")

    return {
        "ok": not blockers,
        "declared": declared,
        "profile": env["profile"],
        "endpoint": endpoint,
        "data_source": data_source,
        "runtime_contract": runtime_contract,
        "simulation_only": _as_optional_bool(env["simulation_only"]),
        "command_sink": command_sink or expected_command_sink,
        "expected_command_sink": expected_command_sink,
        "slam_source": source.get("slam_source"),
        "localization_source": source.get("localization_source"),
        "mapping_source": source.get("mapping_source"),
        "frames": manifest.get("frames", asdict(FRAMES)),
        "frame_links": manifest.get("frame_links", {}),
        "topic_allowed_frame_ids": topic_allowed_frames,
        "topic_default_frame_ids": topic_default_frames,
        "required_topic_frame_ids": required_topic_frame_ids,
        "runtime_data_flow_topics": data_flow_topics,
        "resolved_runtime_data_flow": resolved_flow,
        "runtime_data_flow_stage_algorithm_interfaces": stage_algorithm_interfaces,
        "blockers": blockers,
    }


def _diagnostic_frame_id(
    diagnostics: Mapping[str, Any],
    *keys: str,
) -> str | None:
    from core.runtime_interface import normalize_frame_id

    for key in keys:
        frame = normalize_frame_id(_frame_id(diagnostics.get(key)))
        if frame:
            return frame
    return None


def _localization_frame_summary(
    odometry: Any,
    diagnostics: Mapping[str, Any],
    runtime_boundary: Mapping[str, Any],
) -> dict[str, Any]:
    from core.runtime_interface import TOPICS
    from core.runtime_interface import canonical_data_source_name
    from core.runtime_interface import normalize_frame_id
    from core.runtime_interface import runtime_required_topic_frame_ids
    from core.runtime_interface import runtime_topic_expected_frame_ids

    runtime_contract = (
        runtime_boundary.get("runtime_contract")
        or runtime_boundary.get("data_source")
        or os.environ.get("LINGTU_RUNTIME_CONTRACT")
        or os.environ.get("LINGTU_DATA_SOURCE")
    )
    runtime_contract = canonical_data_source_name(runtime_contract)
    odometry_frame_id = (
        normalize_frame_id(_frame_from_payload(odometry))
        or _diagnostic_frame_id(diagnostics, "odometry_frame_id", "odom_frame_id")
        or "unknown"
    )
    registered_cloud_frame_id = _diagnostic_frame_id(
        diagnostics,
        "registered_cloud_frame_id",
        "registered_frame_id",
        "cloud_frame_id",
    )
    map_cloud_frame_id = _diagnostic_frame_id(
        diagnostics,
        "map_cloud_frame_id",
        "map_frame_id",
        "world_frame_id",
    )
    odometry_expected = runtime_topic_expected_frame_ids(
        runtime_contract,
        TOPICS.odometry,
    )
    registered_cloud_expected = runtime_topic_expected_frame_ids(
        runtime_contract,
        TOPICS.registered_cloud,
    )
    map_cloud_expected = runtime_topic_expected_frame_ids(
        runtime_contract,
        TOPICS.map_cloud,
    )
    observations = (
        (TOPICS.odometry, "odometry", odometry_frame_id, odometry_expected),
        (
            TOPICS.registered_cloud,
            "registered_cloud",
            registered_cloud_frame_id,
            registered_cloud_expected,
        ),
        (TOPICS.map_cloud, "map_cloud", map_cloud_frame_id, map_cloud_expected),
    )
    required_topics = set(
        runtime_boundary.get("required_topic_frame_ids")
        or runtime_required_topic_frame_ids(runtime_contract)
    )
    mismatches: list[dict[str, str]] = []
    missing_required_topic_frame_ids: list[str] = []
    observed_topic_frame_ids: dict[str, str] = {}
    for topic, source, frame_id, expected_frames in observations:
        normalized = normalize_frame_id(frame_id)
        if normalized and normalized != "unknown":
            observed_topic_frame_ids[topic] = normalized
        elif topic in required_topics:
            missing_required_topic_frame_ids.append(topic)
        mismatch = _frame_mismatch(source, normalized, expected_frames)
        if mismatch:
            mismatches.append(mismatch)
    return {
        "runtime_contract": runtime_contract,
        "odometry_frame_id": odometry_frame_id,
        "registered_cloud_frame_id": registered_cloud_frame_id,
        "map_cloud_frame_id": map_cloud_frame_id,
        "odometry_expected_frame_ids": list(odometry_expected),
        "registered_cloud_expected_frame_ids": list(registered_cloud_expected),
        "map_cloud_expected_frame_ids": list(map_cloud_expected),
        "observed_topic_frame_ids": observed_topic_frame_ids,
        "missing_required_topic_frame_ids": missing_required_topic_frame_ids,
        "ok": not mismatches and not missing_required_topic_frame_ids,
        "mismatches": mismatches,
    }


def _navigation_blockers(reason_codes: list[str]) -> list[str]:
    return [code for code in reason_codes if code in _NAVIGATION_BLOCKER_CODES]


def _readiness_summary(
    *,
    can_accept_goal: bool,
    reason_codes: list[str],
    session: Mapping[str, Any],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
    frames: Mapping[str, Any],
    map_artifact_gate: Mapping[str, Any],
    real_runtime_evidence: Mapping[str, Any],
) -> dict[str, Any]:
    blockers = _navigation_blockers(reason_codes)
    advisories = [code for code in reason_codes if code not in blockers]
    map_required = map_artifact_gate.get("required") is True
    real_required = real_runtime_evidence.get("required") is True
    return {
        "can_accept_goal": can_accept_goal,
        "can_execute_autonomy": not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "tf_ok": bool(frames.get("ok", False)),
        "map_artifacts_ok": (
            map_artifact_gate.get("ok") is True if map_required else True
        ),
        "real_runtime_evidence_ok": (
            real_runtime_evidence.get("ok") is True if real_required else None
        ),
        "planning_frame_id": frames.get("planning_frame_id"),
        "odom_frame_id": frames.get("odom_frame_id"),
        "observed_frame_links": _mapping(
            real_runtime_evidence.get("checked_frame_link_evidence")
        ),
        "map_artifact_gate": dict(map_artifact_gate),
        "real_runtime_evidence": dict(real_runtime_evidence),
        "localization_ready": bool(localization.get("ready", False)),
        "control_owner": control.get("command_owner", "unknown"),
        "session_mode": _session_mode(session),
    }


def build_navigation_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        mission = _mapping(gw._mission)
        odometry = gw._odom
        path_len = len(gw._last_path)
        mode = gw._mode
        safety = gw._safety

    session = safe_session(gw)
    lease = safe_lease(gw)
    localization = build_localization_status(gw)
    cmd_vel = _cmd_vel_health(gw)
    nav_runtime = _navigation_module_status(gw)
    state = str(mission.get("state", "IDLE"))
    wp_index = _as_int(mission.get("wp_index"), 0)
    wp_total = _as_int(mission.get("wp_total"), 0)
    replan_count = _as_int(mission.get("replan_count"), 0)
    speed_scale = _as_float(mission.get("speed_scale"), None)
    failure_reason = str(mission.get("failure_reason", "") or "")
    mission_ts = float(mission.get("ts", time.time()) or time.time())
    session_mode = _session_mode(session)
    degraded = localization.get("state") in {
        "no_odometry",
        "initializing",
        "degraded",
        "lost",
        "relocalizing",
    }
    base_can_accept_goal = (
        mode != "estop"
        and odometry is not None
        and not bool(session.get("pending", False))
        and session_mode in {"navigating", "exploring"}
    )
    control = _control_summary(
        mode=mode,
        lease=lease,
        mission_state=state,
        cmd_vel=cmd_vel,
    )
    control["safety_clear"] = not safety_stop_active(safety)
    progress = _progress_summary(
        state=state,
        wp_index=wp_index,
        wp_total=wp_total,
        path_points=path_len,
        replan_count=replan_count,
    )
    frame_mission = dict(mission)
    for key in (
        "frame_id",
        "planning_frame_id",
        "odom_frame_id",
        "costmap_frame_id",
        "goal_frame_id",
    ):
        value = _frame_id(nav_runtime.get(key))
        if value:
            frame_mission[key] = value
    frames = _navigation_frame_summary(frame_mission, odometry)
    map_artifact_gate = _map_artifact_gate_status(nav_runtime)
    real_runtime_evidence = _real_runtime_evidence_status(session)
    runtime_boundary = _runtime_boundary_status()
    reason_codes = _navigation_reason_codes(
        state=state,
        failure_reason=failure_reason,
        has_odometry=odometry is not None,
        mode=mode,
        safety=safety,
        session=session,
        localization=localization,
        control=control,
        frames=frames,
        map_artifact_gate=map_artifact_gate,
        real_runtime_evidence=real_runtime_evidence,
    )
    can_accept_goal = base_can_accept_goal and not _navigation_blockers(reason_codes)
    readiness = _readiness_summary(
        can_accept_goal=can_accept_goal,
        reason_codes=reason_codes,
        session=session,
        localization=localization,
        control=control,
        frames=frames,
        map_artifact_gate=map_artifact_gate,
        real_runtime_evidence=real_runtime_evidence,
    )
    target = _target_summary(
        mission,
        odometry,
        wp_index=wp_index,
        wp_total=wp_total,
        ts=mission_ts,
    )
    speed_policy = _speed_policy_summary(mission, localization, speed_scale)
    motion = _motion_summary(
        odometry=odometry,
        speed_scale=speed_scale,
        speed_policy=speed_policy,
        control=control,
    )
    feedback = _feedback_summary(
        state=state,
        can_accept_goal=can_accept_goal,
        readiness=readiness,
        reason_codes=reason_codes,
    )
    plan_safety_policy = (
        nav_runtime.get("plan_safety_policy")
        or mission.get("plan_safety_policy")
    )
    last_plan_report = (
        _mapping(nav_runtime.get("last_plan_report"))
        or _mapping(mission.get("last_plan_report"))
    )

    return {
        "schema_version": NAVIGATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "has_odometry": odometry is not None,
        "can_accept_goal": can_accept_goal,
        "wp_index": wp_index,
        "wp_total": wp_total,
        "replan_count": replan_count,
        "speed_scale": speed_scale,
        "failure_reason": failure_reason,
        "reason_codes": reason_codes,
        "readiness": readiness,
        "progress": progress,
        "path": {
            "points": path_len,
            "endpoint": PATH_ENDPOINT,
        },
        "runtime": runtime_boundary,
        "frames": frames,
        "control": control,
        "localization": {
            "state": localization.get("state"),
            "ready": localization.get("ready"),
            "degraded": degraded,
            "algorithm_healthy": localization.get("algorithm_healthy"),
            "pose_fresh": localization.get("pose_fresh"),
            "pose_freshness": localization.get("pose_freshness"),
            "degeneracy": localization.get("degeneracy"),
            "speed_scale": speed_scale,
            "reasons": localization.get("reasons", []),
        },
        "target": target,
        "motion": motion,
        "feedback": feedback,
        "diagnostics": {
            "reason_codes": reason_codes,
            "failure_reason": failure_reason,
            "localization_reasons": localization.get("reasons", []),
            "cmd_vel_mux_available": control.get("mux_available", False),
            "frame_mismatches": frames.get("mismatches", []),
            "safety": safety_summary(safety),
            "plan_safety_policy": plan_safety_policy,
            "last_plan_report": last_plan_report,
            "map_artifact_gate": map_artifact_gate,
            "real_runtime_evidence": real_runtime_evidence,
        },
        "mission": {
            "state": state,
            "raw": mission,
        },
        "ts": mission_ts,
    }
