"""App/Web runtime status builders for localization and navigation."""

from __future__ import annotations

import logging
import math
import os
import time
from collections.abc import Mapping
from dataclasses import asdict
from typing import Any

from gateway.services.native_control import (
    endpoint_only_enabled,
)
from gateway.services.native_control import (
    read_status as read_native_control_status,
)
from gateway.services.native_control import (
    status_is_fresh as native_control_status_is_fresh,
)
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_stop_active,
    safety_summary,
)
from runtime.runtime_interface import REAL_RUNTIME_CONTRACT, map_frame_id
from runtime.runtime_policy import (
    backend_capability_defaults as _backend_capability_defaults,
)
from runtime.tf import (
    map_from_odom_transform_from_mapping,
    map_from_odom_transform_to_dict,
)

logger = logging.getLogger(__name__)


LOCALIZATION_STATUS_SCHEMA_VERSION = 1
NAVIGATION_STATUS_SCHEMA_VERSION = 1
STATUS_MAP_FRAME_ID = map_frame_id()

PATH_ENDPOINT = "/api/v1/path"

MISSION_ACTIVE_STATES = {
    "PLANNING",
    "EXECUTING",
    "PAUSED",
    "RECOVERING",
    "PATROLLING",
}
MISSION_TERMINAL_STATES = {"SUCCESS", "FAILED", "CANCELLED"}

CONTROL_SOURCE_META: dict[str, dict[str, Any]] = {
    "teleop": {
        "label": "Teleop velocity",
        "category": "manual",
        "owner": "teleop",
        "preempts_autonomy": True,
    },
    "manual_hold": {
        "label": "Operator takeover hold",
        "category": "manual_hold",
        "owner": "operator",
        "preempts_autonomy": True,
    },
    "autonomy": {
        "label": "Native navigation autonomy",
        "category": "autonomy",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
    "estop": {
        "label": "Native emergency stop",
        "category": "safety",
        "owner": "safety",
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
LOST_STATES = {
    "LOST",
    "DIVERGED",
    "FAILED",
    "ERROR",
    "STALE",
    "UNINIT",
    "UNINITIALIZED",
}
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
    health_source = (
        str(diagnostics.get("health_source") or diagnostics.get("localizer_health_source") or "").strip().lower()
    )
    icp_fitness = _as_float(diagnostics.get("icp_fitness"), icp_quality)
    health_fitness = _as_float(diagnostics.get("localizer_health_fitness"), None)
    icp_ok = any(value is not None and 0.0 < value < 0.5 for value in (icp_fitness, health_fitness))
    pose_fresh, _ = classify_pose_freshness(diagnostics)
    cloud_fresh = _cloud_fresh(diagnostics)
    odom_cloud_ok = (
        health_source == "odom_map_cloud"
        and pose_fresh is not False
        and cloud_fresh
        and reported in {"", *TRACKING_STATES}
    )
    cpp_status_snapshot_ok = (
        health_source in {"slam_runtime", "cpp_slam_status_snapshot", "cpp_slam_status_json"}
        and pose_fresh is not False
        and reported in TRACKING_STATES
        and float(diagnostics.get("quality", diagnostics.get("confidence", 0.0)) or 0.0) >= 0.5
    )

    return (
        reported in {"", *TRACKING_STATES}
        and degeneracy not in BAD_DEGENERACY
        and localizer_health in GOOD_LOCALIZER_HEALTH
        and not recovery_signal
        and cloud_fresh
        and (icp_ok or odom_cloud_ok or cpp_status_snapshot_ok)
    )


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
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    localizer_health = _reported_state(diagnostics.get("localizer_health"))
    recovery_signal = _active_recovery_signal(diagnostics.get("recovery_signal"))
    diagnostics.get("confidence")
    algorithm_healthy = localizer_algorithm_healthy(diagnostics, icp_quality)
    pose_fresh, _ = classify_pose_freshness(diagnostics)

    if odometry is None:
        reasons.append("odometry_missing")
        return "no_odometry", reasons

    if "RELOCAL" in reported:
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
    *,
    gw: Any | None = None,
) -> dict[str, Any]:
    diagnostics = _mapping(status)
    diag_received_mono = _as_float(diagnostics.get("_gateway_received_mono"))
    diag_age_ms = (
        round(max(0.0, time.monotonic() - diag_received_mono) * 1000.0, 1) if diag_received_mono is not None else None
    )
    algorithm_healthy = localizer_algorithm_healthy(diagnostics, float(icp_quality))
    pose_fresh, pose_freshness = classify_pose_freshness(diagnostics)
    state, reasons = _localization_state(
        odometry,
        session,
        float(icp_quality),
        diagnostics,
    )
    ready = state == "ready"
    raw_backend = diagnostics.get("backend") or diagnostics.get("slam_profile") or session.get("slam_profile")
    backend_name = str(raw_backend or "").strip().lower()
    health_source = str(diagnostics.get("health_source") or "").strip().lower()
    algorithm_profile = backend_name or None
    if health_source == "slam_runtime":
        backend_name = "native_dds"
    backend = backend_name or raw_backend
    capability_defaults = backend_capability_defaults(backend_name)
    relocalization_supported = _as_optional_bool(diagnostics.get("relocalization_supported"))
    if relocalization_supported is None:
        relocalization_supported = bool(capability_defaults["relocalization_supported"])
    saved_map_relocalization_supported = _as_optional_bool(diagnostics.get("saved_map_relocalization_supported"))
    if saved_map_relocalization_supported is None:
        saved_map_relocalization_supported = relocalization_supported
    restart_recovery_supported = _as_optional_bool(diagnostics.get("restart_recovery_supported"))
    if restart_recovery_supported is None:
        restart_recovery_supported = bool(capability_defaults["restart_recovery_supported"])
    recovery_method = diagnostics.get("recovery_method")
    if not recovery_method:
        recovery_method = capability_defaults["recovery_method"]
    map_save_supported = _as_optional_bool(diagnostics.get("map_save_supported"))
    if map_save_supported is None:
        map_save_supported = bool(capability_defaults["map_save_supported"])
    map_save_source = diagnostics.get("map_save_source")
    if map_save_source is None:
        map_save_source = capability_defaults["map_save_source"]
    runtime_boundary = _runtime_boundary_status(gw)
    frames = _localization_frame_summary(
        odometry,
        diagnostics,
        runtime_boundary,
    )
    buffers = _mapping(diagnostics.get("buffers"))
    map_tracking = _mapping(diagnostics.get("track_against_map"))
    map_from_odom = map_from_odom_transform_from_mapping(diagnostics.get("map_odom_tf"))
    map_odom_tf = (
        map_from_odom_transform_to_dict(map_from_odom)
        if map_from_odom is not None
        else None
    )
    return {
        "schema_version": LOCALIZATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "ready": ready,
        "has_odometry": odometry is not None,
        "odometry": odometry,
        "session_mode": session.get("mode"),
        "active_map": session.get("active_map"),
        "icp_quality": float(icp_quality),
        "reported_state": diagnostics.get("state"),
        "reason": diagnostics.get("reason") or (reasons[0] if reasons else None),
        "backend_reason": diagnostics.get("reason"),
        "confidence": diagnostics.get("confidence"),
        "algorithm_healthy": algorithm_healthy,
        "backend": backend,
        "algorithm_profile": algorithm_profile,
        "native_mode": diagnostics.get("mode"),
        "health_source": diagnostics.get("health_source"),
        "pose_fresh": pose_fresh,
        "pose_freshness": pose_freshness,
        "stale_odometry": pose_fresh is False and algorithm_healthy,
        "odom_age_ms": _as_float(diagnostics.get("odom_age_ms")),
        "cloud_age_ms": _as_float(diagnostics.get("cloud_age_ms")),
        "degeneracy": diagnostics.get("degeneracy"),
        "icp_fitness": _as_float(diagnostics.get("icp_fitness")),
        "degeneracy_detected": _as_optional_bool(diagnostics.get("degeneracy_detected")),
        "effective_ratio": _as_float(diagnostics.get("effective_ratio")),
        "condition_number": _as_float(diagnostics.get("condition_number")),
        "min_eigenvalue": _as_float(diagnostics.get("min_eigenvalue")),
        "max_eigenvalue": _as_float(diagnostics.get("max_eigenvalue")),
        "degenerate_dof_count": _as_optional_int(diagnostics.get("degenerate_dof_count")),
        "pos_cov_trace": _as_float(diagnostics.get("pos_cov_trace")),
        "ieskf_iter_num": _as_optional_int(diagnostics.get("ieskf_iter_num")),
        "ieskf_converged": _as_optional_bool(diagnostics.get("ieskf_converged")),
        "map_cloud_fresh": _as_optional_bool(diagnostics.get("map_cloud_fresh")),
        "status_target_hz": _as_float(diagnostics.get("status_target_hz")),
        "imu_input_hz": _as_float(diagnostics.get("imu_input_hz")),
        "lidar_input_hz": _as_float(diagnostics.get("lidar_input_hz")),
        "slam_tick_hz": _as_float(diagnostics.get("slam_tick_hz")),
        "processed_scan_hz": _as_float(diagnostics.get("processed_scan_hz")),
        "fastlio_velocity": _mapping(diagnostics.get("fastlio_velocity")),
        "fastlio_speed_mps": _as_float(diagnostics.get("fastlio_speed_mps")),
        "max_reasonable_speed_mps": _as_float(diagnostics.get("max_reasonable_speed_mps")),
        "runtime_instance_id": str(diagnostics.get("runtime_instance_id") or "") or None,
        "observation_sequence": _as_optional_int(diagnostics.get("observation_sequence")),
        "registered_points": _as_optional_int(diagnostics.get("registered_points")),
        "map_points": _as_optional_int(diagnostics.get("map_points")),
        "imu_buffer": _as_optional_int(diagnostics.get("imu_buffer", buffers.get("imu"))),
        "lidar_buffer": _as_optional_int(diagnostics.get("lidar_buffer", buffers.get("lidar"))),
        "imu_batch": _as_optional_int(diagnostics.get("imu_batch", buffers.get("imu_batch"))),
        "dropped_lidar_frames": _as_optional_int(
            diagnostics.get("dropped_lidar_frames", buffers.get("dropped_lidar_frames"))
        ),
        "dropped_imu_frames": _as_optional_int(
            diagnostics.get("dropped_imu_frames", buffers.get("dropped_imu_frames"))
        ),
        "scan_start_s": _as_float(diagnostics.get("scan_start_s")),
        "scan_end_s": _as_float(diagnostics.get("scan_end_s")),
        "last_imu_s": _as_float(diagnostics.get("last_imu_s")),
        "sync_wait_count": _as_optional_int(diagnostics.get("sync_wait_count", buffers.get("sync_wait_count"))),
        "imu_rollback_count": _as_optional_int(
            diagnostics.get("imu_rollback_count", buffers.get("imu_rollback_count"))
        ),
        "lidar_rollback_count": _as_optional_int(
            diagnostics.get("lidar_rollback_count", buffers.get("lidar_rollback_count"))
        ),
        "map_loaded": _as_optional_bool(diagnostics.get("map_loaded")),
        "map_tracking": map_tracking,
        "map_frame_jump": _as_optional_bool(diagnostics.get("map_frame_jump")),
        "map_frame_jump_sequence": _as_optional_int(diagnostics.get("map_frame_jump_sequence")),
        "scene_mode": diagnostics.get("scene_mode"),
        "gnss_fusion_health": _mapping(diagnostics.get("gnss_fusion_health")),
        "map_odom_tf": map_odom_tf,
        "has_map_odom_tf": map_odom_tf is not None,
        "map_state": diagnostics.get("map_state"),
        "map_save_supported": map_save_supported,
        "map_save_source": map_save_source,
        "relocalization_supported": relocalization_supported,
        "saved_map_relocalization_supported": saved_map_relocalization_supported,
        "restart_recovery_supported": restart_recovery_supported,
        "recovery_method": recovery_method,
        "relocalization_state": diagnostics.get("relocalization_state"),
        "relocalization_quality": _as_float(diagnostics.get("relocalization_quality")),
        "relocalization_map_body": diagnostics.get("relocalization_map_body"),
        "relocalization_refine_backend": diagnostics.get("relocalization_refine_backend"),
        "relocalization_refine_iterations": _as_optional_int(diagnostics.get("relocalization_refine_iterations")),
        "relocalization_refine_inliers": _as_optional_int(diagnostics.get("relocalization_refine_inliers")),
        "relocalization_refine_input_points": _as_optional_int(diagnostics.get("relocalization_refine_input_points")),
        "relocalization_refine_evaluated_points": _as_optional_int(
            diagnostics.get("relocalization_refine_evaluated_points")
        ),
        "relocalization_min_inliers": _as_optional_int(diagnostics.get("relocalization_min_inliers")),
        "relocalization_min_evaluated_points": _as_optional_int(diagnostics.get("relocalization_min_evaluated_points")),
        "relocalization_refine_support_ratio": _as_float(diagnostics.get("relocalization_refine_support_ratio")),
        "relocalization_refine_overlap_inlier_ratio": _as_float(
            diagnostics.get("relocalization_refine_overlap_inlier_ratio")
        ),
        "relocalization_refine_converged": _as_optional_bool(diagnostics.get("relocalization_refine_converged")),
        "relocalization_refine_pos_cov_trace": _as_float(diagnostics.get("relocalization_refine_pos_cov_trace")),
        "recovery_signal": diagnostics.get("recovery_signal"),
        "recovery_action": diagnostics.get("recovery_action"),
        "localizer_health": diagnostics.get("localizer_health"),
        "localizer_health_raw": diagnostics.get("localizer_health_raw"),
        "localizer_health_source": diagnostics.get("localizer_health_source"),
        "localizer_health_topic_age_ms": _as_float(diagnostics.get("localizer_health_topic_age_ms")),
        "localizer_health_fitness": _as_float(diagnostics.get("localizer_health_fitness")),
        "localizer_health_iter": _as_optional_int(diagnostics.get("localizer_health_iter")),
        "localizer_health_cov_trace": _as_float(diagnostics.get("localizer_health_cov_trace")),
        "ts": diagnostics.get("ts"),
        "diag_received_ts": _as_float(diagnostics.get("_gateway_received_ts")),
        "diag_age_ms": diag_age_ms,
        "runtime": runtime_boundary,
        "frames": frames,
        "registered_cloud_frame_id": frames.get("registered_cloud_frame_id"),
        "map_cloud_frame_id": frames.get("map_cloud_frame_id"),
        "can_relocalize": (
            saved_map_relocalization_supported and state in {"degraded", "lost"} and odometry is not None
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
        gw=gw,
    )


def _control_summary(
    *,
    mode: str,
    lease: Mapping[str, Any],
    mission_state: str,
    native_endpoint: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    native_control = _mapping(native_endpoint)
    active_source = str(native_control.get("active_cmd_source") or "unknown")
    active = active_source not in {"none", "unknown"}
    sources = {active_source: {"active": active}} if active_source else {}
    source_available = native_control.get("status_available") is True
    active_source_health = _mapping(sources.get(active_source))
    authority = _mapping(native_control.get("control_authority"))
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
    manual_override = active_source in {"teleop", "manual_hold"}
    preempting_autonomy = bool(autonomy_requested and meta.get("preempts_autonomy", False))

    return {
        "mode": mode,
        "lease": dict(lease),
        "authority_source": "native_endpoint",
        "authority_available": source_available,
        "native_endpoint_available": source_available,
        "active_cmd_source": active_source,
        "command_owner": meta["owner"],
        "source_category": meta["category"],
        "manual_override": manual_override,
        "autonomy_requested": autonomy_requested,
        "preempting_autonomy": preempting_autonomy,
        "operator_takeover_latched": authority.get("operator_takeover_latched") is True,
        "resume_required": authority.get("resume_required") is True,
        "estop_latched": authority.get("estop_latched") is True,
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
        "native_endpoint_control": native_control,
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
        live_goal_distance if live_goal_distance is not None else _finite_float(mission.get("distance_to_goal_m"))
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
        "source": str(raw_policy.get("source") or "native_navigation_state"),
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
    elif state == "RECOVERING":
        next_action = "monitor_recovery"
        primary = "Navigation is running recovery."
    elif state == "PAUSED":
        next_action = "resume_or_cancel"
        primary = "Navigation is paused."
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
    if localization.get("pose_fresh") is False and bool(localization.get("algorithm_healthy", False)):
        codes.append("pose_stale")
    if _saved_map_relocalization_missing(localization):
        codes.append("saved_map_relocalization_missing")
    if _saved_map_tracking_unhealthy(localization):
        codes.append("saved_map_tracking_unhealthy")

    if state == "RECOVERING":
        codes.append("mission_recovering")
    elif state == "PAUSED":
        codes.append("mission_paused")
    elif state == "STUCK":
        codes.append("mission_stuck")
    elif state == "FAILED":
        codes.append("mission_failed")
    elif state == "CANCELLED":
        codes.append("mission_cancelled")

    if failure_reason:
        codes.append(_reason_code_from_text("failure", failure_reason))

    if control.get("preempting_autonomy"):
        codes.append(f"control_preempted_by_{control.get('active_cmd_source')}")
    if map_artifact_gate.get("required") is True and map_artifact_gate.get("ok") is not True:
        codes.append("map_artifact_gate_failed")
    if real_runtime_evidence.get("required") is True and real_runtime_evidence.get("ok") is not True:
        codes.append("real_runtime_evidence_missing_or_stale")

    return list(dict.fromkeys(codes))


def _saved_map_relocalization_missing(localization: Mapping[str, Any]) -> bool:
    if not localization.get("active_map"):
        return False
    if str(localization.get("backend") or "").lower() != "native_dds":
        return False
    if str(localization.get("native_mode") or "").lower() != "localization":
        return False
    if localization.get("map_loaded") is not True:
        return False
    if localization.get("saved_map_relocalization_supported") is not True:
        return True

    state = str(localization.get("relocalization_state") or "").strip().lower()
    if state in {"completed", "relocalized"}:
        return False
    if isinstance(localization.get("relocalization_map_body"), Mapping):
        return False
    return True


def _saved_map_tracking_unhealthy(localization: Mapping[str, Any]) -> bool:
    if not localization.get("active_map"):
        return False
    if str(localization.get("backend") or "").lower() != "native_dds":
        return False
    if str(localization.get("native_mode") or "").lower() != "localization":
        return False
    tracking = localization.get("map_tracking")
    if not isinstance(tracking, Mapping):
        return True
    successes = _as_optional_int(tracking.get("successes"))
    return (
        tracking.get("enabled") is not True
        or successes is None
        or successes <= 0
        or tracking.get("degraded") is not False
    )


_NAVIGATION_BLOCKER_CODES = {
    "odometry_missing",
    "frame_mismatch_odometry",
    "frame_mismatch_costmap",
    "frame_mismatch_goal",
    "estop_active",
    SAFETY_STOP_BLOCKER,
    "navigation_session_inactive",
    "localization_lost",
    "localization_relocalizing",
    "localization_initializing",
    "localization_recovery_active",
    "saved_map_relocalization_missing",
    "saved_map_tracking_unhealthy",
    "pose_stale",
    "map_artifact_gate_failed",
    "real_runtime_evidence_missing_or_stale",
    "native_endpoint_status_missing_or_stale",
    "native_input_gate_not_ready",
    "native_control_mode_mismatch",
    "native_control_authority_invalid",
    "native_active_cmd_source_invalid",
    "native_resume_required",
    "native_estop_latched",
    "native_product_expectation_unavailable",
    "native_product_mismatch",
    "native_product_parameters_mismatch",
    "native_teleop_local_planner_disabled",
    "native_obstacle_check_disabled",
    "native_traversability_cost_disabled",
    "native_global_planner_missing",
    "native_global_planner_mismatch",
    "native_planner_map_missing",
    "native_cmd_vel_publish_disabled",
    "native_control_loop_unhealthy",
    "native_control_loop_health_unavailable",
    "native_operator_motion_status_missing",
    "native_operator_motion_schema_mismatch",
    "native_operator_motion_interface_disabled",
    "native_operator_motion_authority_invalid",
    "native_operator_motion_control_mode_mismatch",
    "native_operator_motion_ack_scope_invalid",
    "native_operator_motion_sample_evidence_invalid",
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


def _with_active_map_artifact_consistency(
    gate: Mapping[str, Any],
    session: Mapping[str, Any],
    localization: Mapping[str, Any],
) -> dict[str, Any]:
    checked = dict(gate)
    active_map = str(session.get("active_map") or localization.get("active_map") or "").strip()
    gate_map_id = str(checked.get("map_id") or "").strip()
    if active_map and gate_map_id and gate_map_id != active_map:
        blockers = [str(item) for item in (checked.get("blockers") or []) if str(item)]
        if "active_map_artifact_gate_mismatch" not in blockers:
            blockers.append("active_map_artifact_gate_mismatch")
        checked["ok"] = False
        checked["reason"] = "active_map_artifact_gate_mismatch"
        checked["active_map"] = active_map
        checked["gate_map"] = gate_map_id
        checked["blockers"] = blockers
    return checked


def _real_runtime_evidence_status(session: Mapping[str, Any]) -> dict[str, Any]:
    from runtime.runtime_interface import canonical_data_source_name

    evidence_required = os.environ.get(
        "LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE",
        "1",
    ).strip().lower() not in {"0", "false", "no", "off"}
    runtime_contract = os.environ.get("LINGTU_RUNTIME_CONTRACT") or os.environ.get("LINGTU_DATA_SOURCE")
    runtime_contract = canonical_data_source_name(runtime_contract)
    required = runtime_contract == REAL_RUNTIME_CONTRACT and _session_mode(session) in {"navigating", "exploring"}
    if required and not evidence_required:
        return {
            "required": False,
            "ok": None,
            "runtime_contract": runtime_contract,
            "reason": "disabled_for_commissioning",
            "blockers": [],
        }
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
    from runtime.runtime_interface import normalize_frame_id

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
    from runtime.runtime_interface import (
        TOPICS,
        canonical_data_source_name,
        normalize_frame_id,
        runtime_topic_default_frame_id,
    )

    runtime_contract = os.environ.get("LINGTU_RUNTIME_CONTRACT") or os.environ.get("LINGTU_DATA_SOURCE")
    runtime_contract = canonical_data_source_name(runtime_contract)
    default_planning_frame = runtime_topic_default_frame_id(
        runtime_contract,
        TOPICS.global_path,
    )
    planning_frame_id = _frame_id(mission.get("planning_frame_id") or mission.get("frame_id")) or default_planning_frame
    odom_frame_id = _frame_id(mission.get("odom_frame_id")) or _frame_from_payload(odometry) or "unknown"
    costmap_frame_id = _frame_id(mission.get("costmap_frame_id")) or "unknown"
    goal_frame_id = _frame_id(mission.get("goal_frame_id")) or _frame_from_payload(mission.get("goal"))
    planning_frame = normalize_frame_id(planning_frame_id) or default_planning_frame
    map_from_odom = map_from_odom_transform_from_mapping(mission.get("map_odom_tf"))
    map_odom_tf = (
        map_from_odom_transform_to_dict(map_from_odom)
        if map_from_odom is not None
        else None
    )
    linked_odom_frame: str | None = None
    if map_odom_tf is not None:
        parent = map_odom_tf["frame_id"]
        child = map_odom_tf["child_frame_id"]
        if parent == planning_frame and child:
            linked_odom_frame = child
    odometry_expected = tuple(dict.fromkeys(frame for frame in (planning_frame, linked_odom_frame) if frame))
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
        "has_map_odom_tf": linked_odom_frame is not None,
        "map_odom_tf": map_odom_tf,
        "observed_frame_links": (
            {
                "map_to_odom": {
                    "frame_id": planning_frame,
                    "child_frame_id": linked_odom_frame,
                    "valid": True,
                }
            }
            if linked_odom_frame is not None
            else {}
        ),
        "ok": not mismatches,
        "mismatches": mismatches,
    }


def runtime_identity(gw: Any | None = None) -> dict[str, Any]:
    """Return the public Gateway runtime identity."""

    plan = getattr(gw, "_compiled_run_plan", None) if gw is not None else None
    env = str(getattr(gw, "_compiled_env", "real") or "real").strip()
    if env not in {"real", "sim"}:
        raise ValueError(f"Env must be 'real' or 'sim', received {env!r}")

    product = str(getattr(gw, "_compiled_product", "") or "").strip()
    return {
        "env": env,
        "product": product or None,
        "state": "active" if plan is not None else "standby",
        "product_session_id": (
            str(getattr(gw, "_compiled_product_session_id", "") or "").strip()
            or None
        ),
    }


def _runtime_boundary_status(gw: Any | None = None) -> dict[str, Any]:
    from runtime.runtime_interface import (
        DATA_SOURCE_CONTRACTS,
        FRAMES,
        REAL_RUNTIME_CONTRACT,
        RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
        canonical_data_source_name,
        resolved_runtime_data_flow,
        runtime_contract_data_source,
        runtime_contract_manifest,
        runtime_data_flow_topics,
        runtime_required_topic_frame_ids,
        runtime_topic_allowed_frame_ids,
        runtime_topic_default_frame_ids,
    )

    identity = runtime_identity(gw)
    runtime_settings = {
        "data_source": os.environ.get("LINGTU_DATA_SOURCE"),
        "runtime_contract": os.environ.get("LINGTU_RUNTIME_CONTRACT"),
        "command_sink": os.environ.get("LINGTU_COMMAND_SINK"),
    }
    declared = any(value not in (None, "") for value in runtime_settings.values())
    data_source = canonical_data_source_name(runtime_settings["data_source"])
    runtime_contract = canonical_data_source_name(runtime_settings["runtime_contract"])
    command_sink = runtime_settings["command_sink"]
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
    if (
        runtime_contract
        and data_source
        and runtime_contract_data_source(runtime_contract) != data_source
    ):
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
            resolved_flow = [asdict(stage) for stage in resolved_runtime_data_flow(data_source)]
            stage_algorithm_interfaces = {
                name: list(interfaces) for name, interfaces in (RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items())
            }
            data_flow_topics = list(runtime_data_flow_topics(data_source))
        else:
            blockers.append("data_source_unknown")

    topic_contract = runtime_contract or data_source
    if topic_contract:
        frame_contracts = {
            *DATA_SOURCE_CONTRACTS,
            REAL_RUNTIME_CONTRACT,
        }
        if topic_contract in frame_contracts:
            topic_allowed_frames = {
                topic: list(frame_ids) for topic, frame_ids in runtime_topic_allowed_frame_ids(topic_contract).items()
            }
            topic_default_frames = dict(runtime_topic_default_frame_ids(topic_contract))
            required_topic_frame_ids = list(runtime_required_topic_frame_ids(topic_contract))
        else:
            blockers.append("topic_frame_contract_unavailable")

    if command_sink and expected_command_sink and command_sink != expected_command_sink:
        blockers.append("command_sink_mismatch")

    return {
        "ok": not blockers,
        "declared": declared,
        **identity,
        "data_source": data_source,
        "runtime_contract": runtime_contract,
        "simulation_only": identity["env"] == "sim",
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
    from runtime.runtime_interface import normalize_frame_id

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
    from runtime.runtime_interface import (
        TOPICS,
        canonical_data_source_name,
        normalize_frame_id,
        runtime_required_topic_frame_ids,
        runtime_topic_expected_frame_ids,
    )

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
        runtime_boundary.get("required_topic_frame_ids") or runtime_required_topic_frame_ids(runtime_contract)
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


def _native_endpoint_readiness(
    session: Mapping[str, Any],
    gw: Any | None = None,
) -> dict[str, Any]:
    plan = getattr(gw, "_compiled_run_plan", None) if gw is not None else None
    raw_product = str(
        getattr(plan, "product", "") or session.get("product") or os.environ.get("LINGTU_PRODUCT") or ""
    ).strip()
    product = raw_product
    required_capabilities = tuple(getattr(plan, "required_capabilities", ()) if plan is not None else ())
    process_names = {
        str(getattr(process, "name", "") or "").strip()
        for process in getattr(plan, "processes", ())
        if str(getattr(process, "name", "") or "").strip()
    }
    operator_motion_required = bool(
        {
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
        }
        & set(required_capabilities)
    )
    native_nav = getattr(plan, "native_nav", {}) if plan is not None else {}
    expected_control_mode = (
        str(native_nav.get("control_mode", "autonomy") if isinstance(native_nav, Mapping) else "autonomy")
        .strip()
        .lower()
    )
    product_requires_native_endpoint = "nav" in process_names
    managed_run_plan_missing = bool(
        plan is None
        and gw is not None
        and getattr(gw, "_compiled_command_output_mode", "") == "endpoint_only"
    )
    legacy_runtime_requires_native_endpoint = (
        plan is None and endpoint_only_enabled(gw) and _session_mode(session) in {"navigating", "exploring"}
    )
    required = product_requires_native_endpoint or managed_run_plan_missing or legacy_runtime_requires_native_endpoint
    if not required:
        return {
            "required": False,
            "ok": None,
            "navigation_ready": None,
            "blockers": [],
            "far_input": {"required": False, "ready": True, "reason": "not_required"},
            "input_gate": {},
            "status_available": None,
            "active_cmd_source": None,
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": operator_motion_required,
                "status_available": None,
            },
        }
    if managed_run_plan_missing:
        return {
            "required": True,
            "ok": False,
            "navigation_ready": False,
            "blockers": ["run_plan_missing"],
            "far_input": {},
            "input_gate": {},
            "status_available": None,
            "active_cmd_source": "unknown",
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": False,
                "status_available": None,
            },
        }
    snapshot = read_native_control_status()
    if not native_control_status_is_fresh(snapshot):
        return {
            "required": True,
            "ok": False,
            "navigation_ready": False,
            "blockers": ["native_endpoint_status_missing_or_stale"],
            "far_input": {},
            "input_gate": {},
            "status_available": False,
            "active_cmd_source": "unknown",
            "control_authority": {},
            "expected_control_mode": expected_control_mode,
            "operator_motion": {
                "required": operator_motion_required,
                "status_available": False,
            },
        }
    input_gate = _mapping(snapshot.get("input_gate"))
    control_loop_health = _mapping(snapshot.get("control_loop_health"))
    blockers: list[str] = []
    control_loop_reason = str(control_loop_health.get("reason") or "").strip().lower()
    if not control_loop_health:
        blockers.append("native_control_loop_health_unavailable")
    elif control_loop_health.get("ready") is False:
        if control_loop_reason != "warming_up":
            blockers.append("native_control_loop_health_unavailable")
    elif control_loop_health.get("ready") is True:
        if control_loop_health.get("healthy") is not True:
            blockers.append("native_control_loop_unhealthy")
    else:
        blockers.append("native_control_loop_health_unavailable")
    if input_gate.get("ready") is not True:
        blockers.append("native_input_gate_not_ready")
    active_cmd_source = str(snapshot.get("active_cmd_source") or "").strip().lower()
    if active_cmd_source not in {"none", "autonomy", "teleop", "manual_hold", "estop"}:
        blockers.append("native_active_cmd_source_invalid")
    control_authority = _mapping(snapshot.get("control_authority"))
    if str(control_authority.get("owner") or "").strip().lower() != "native_endpoint":
        blockers.append("native_control_authority_invalid")
    estop_latched = control_authority.get("estop_latched") is True
    operator_takeover_latched = control_authority.get("operator_takeover_latched") is True
    resume_required = control_authority.get("resume_required") is True
    if estop_latched or active_cmd_source == "estop":
        blockers.append("native_estop_latched")
    if operator_takeover_latched or resume_required or active_cmd_source in {"teleop", "manual_hold"}:
        blockers.append("native_resume_required")
    control_mode = str(snapshot.get("control_mode") or "").strip().lower()
    parameter_mismatches: dict[str, dict[str, float | None]] = {}
    expected_parameters: dict[str, float] = {}
    observed_native_product = _mapping(snapshot.get("native_product"))
    native_product_required = plan is not None and "nav" in process_names
    if native_product_required:
        actual_product = str(observed_native_product.get("product") or "").strip()
        if actual_product != product:
            blockers.append("native_product_mismatch")

        expected_environment = getattr(plan, "native_process_environment", {})
        parameter_environment = (
            ("path_follower_max_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS"),
            ("path_follower_min_speed_mps", "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS"),
            ("path_follower_max_accel_mps2", "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2"),
            ("path_follower_lookahead_m", "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M"),
            ("path_follower_goal_tolerance_m", "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M"),
            ("waypoint_reached_m", "LINGTU_NAV_WAYPOINT_REACHED_M"),
            ("goal_reached_m", "LINGTU_NAV_GOAL_REACHED_M"),
            ("corridor_lookahead_m", "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M"),
            ("teleop_planner_horizon_m", "LINGTU_TELEOP_PLANNER_HORIZON_M"),
            ("teleop_planner_max_deviation_deg", "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG"),
        )
        for parameter, environment_name in parameter_environment:
            value = _finite_float(expected_environment.get(environment_name))
            if value is None:
                expected_parameters.clear()
                break
            expected_parameters[parameter] = value
        if not expected_parameters:
            blockers.append("native_product_expectation_unavailable")
        else:
            observed_path_follower = _mapping(snapshot.get("path_follower"))
            observed_nav_loop = _mapping(snapshot.get("nav_loop"))
            observed_parameters = (
                (
                    "path_follower.max_speed_mps",
                    observed_path_follower.get("max_speed_mps"),
                    "path_follower_max_speed_mps",
                ),
                (
                    "path_follower.min_speed_mps",
                    observed_path_follower.get("min_speed_mps"),
                    "path_follower_min_speed_mps",
                ),
                (
                    "path_follower.max_accel_mps2",
                    observed_path_follower.get("max_accel_mps2"),
                    "path_follower_max_accel_mps2",
                ),
                (
                    "path_follower.lookahead_m",
                    observed_path_follower.get("lookahead_m"),
                    "path_follower_lookahead_m",
                ),
                (
                    "path_follower.goal_tolerance_m",
                    observed_path_follower.get("goal_tolerance_m"),
                    "path_follower_goal_tolerance_m",
                ),
                (
                    "nav_loop.waypoint_reached_m",
                    observed_nav_loop.get("waypoint_reached_m"),
                    "waypoint_reached_m",
                ),
                (
                    "nav_loop.goal_reached_m",
                    observed_nav_loop.get("goal_reached_m"),
                    "goal_reached_m",
                ),
                (
                    "nav_loop.corridor_lookahead_m",
                    observed_nav_loop.get("corridor_lookahead_m"),
                    "corridor_lookahead_m",
                ),
            )
            for field, raw_actual, expected_key in observed_parameters:
                actual = _as_float(raw_actual, None)
                expected = float(expected_parameters[expected_key])
                if actual is None or not math.isclose(
                    actual,
                    expected,
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                ):
                    parameter_mismatches[field] = {
                        "actual": actual,
                        "expected": expected,
                    }
            if parameter_mismatches:
                blockers.append("native_product_parameters_mismatch")
    if control_mode != expected_control_mode:
        blockers.append("native_control_mode_mismatch")
    global_planner = str(snapshot.get("global_planner") or "").strip().lower()
    far_input = _mapping(snapshot.get("far_input"))
    assisted_teleop_required = plan is not None and bool(
        {
            "operator_assisted_local_planner_control",
            "operator_assisted_local_planner_takeover",
        }
        & set(required_capabilities)
    )
    teleop_local_planner = _as_optional_bool(snapshot.get("teleop_local_planner"))
    check_obstacle = _as_optional_bool(snapshot.get("check_obstacle"))
    use_traversability_cost = _as_optional_bool(snapshot.get("use_traversability_cost"))
    if assisted_teleop_required and expected_parameters:
        for field, raw_actual, expected_key in (
            (
                "teleop_planner_horizon_m",
                snapshot.get("teleop_planner_horizon_m"),
                "teleop_planner_horizon_m",
            ),
            (
                "teleop_planner_max_deviation_deg",
                snapshot.get("teleop_planner_max_deviation_deg"),
                "teleop_planner_max_deviation_deg",
            ),
        ):
            actual = _as_float(raw_actual, None)
            expected = float(expected_parameters[expected_key])
            if actual is None or not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9):
                parameter_mismatches[field] = {"actual": actual, "expected": expected}
        if parameter_mismatches and "native_product_parameters_mismatch" not in blockers:
            blockers.append("native_product_parameters_mismatch")
    if assisted_teleop_required:
        if teleop_local_planner is not True:
            blockers.append("native_teleop_local_planner_disabled")
        if check_obstacle is not True:
            blockers.append("native_obstacle_check_disabled")
        if use_traversability_cost is not True:
            blockers.append("native_traversability_cost_disabled")
    expected_global_planner = str(
        native_nav.get("global_planner") or ""
        if isinstance(native_nav, Mapping)
        else ""
    ).strip().lower()
    if not expected_global_planner:
        expected_global_planner = str(
            session.get("global_planner")
            or session.get("planner")
            or os.environ.get("NAV_GLOBAL_PLANNER")
            or "octoplanner3d"
        ).strip().lower()
    aliases = {"octo": "octoplanner3d", "octplanner": "octoplanner3d"}
    global_planner = aliases.get(global_planner, global_planner)
    expected_global_planner = aliases.get(expected_global_planner, expected_global_planner)
    if global_planner == "far":
        if not far_input:
            blockers.append("native_far_input_status_missing")
        elif far_input.get("required") is not True:
            blockers.append("native_far_input_contract_invalid")
        elif far_input.get("ready") is not True:
            blockers.append("native_far_input_not_ready")
        elif not str(far_input.get("map_id") or "").strip() or _as_int(
            far_input.get("content_epoch"), 0
        ) <= 0:
            blockers.append("native_far_input_identity_invalid")
    planner_map = str(snapshot.get("planner_map") or "").strip()
    operator_motion = _mapping(snapshot.get("operator_motion"))
    operator_motion_status_available = bool(operator_motion)
    if operator_motion_required:
        if not operator_motion_status_available:
            blockers.append("native_operator_motion_status_missing")
        else:
            if _as_int(operator_motion.get("schema_version"), -1) != 1:
                blockers.append("native_operator_motion_schema_mismatch")
            if operator_motion.get("interface_enabled") is not True:
                blockers.append("native_operator_motion_interface_disabled")
            if str(operator_motion.get("authority_owner") or "").strip().lower() != "native_endpoint":
                blockers.append("native_operator_motion_authority_invalid")
            if str(operator_motion.get("control_mode") or "").strip().lower() != expected_control_mode:
                blockers.append("native_operator_motion_control_mode_mismatch")
            if operator_motion.get("control_ack_scope") != "claim_hold_release":
                blockers.append("native_operator_motion_ack_scope_invalid")
            if operator_motion.get("sample_evidence") != "status_sequences":
                blockers.append("native_operator_motion_sample_evidence_invalid")
    lifecycle = getattr(plan, "lifecycle", {}) if plan is not None else {}
    requires_map = bool(
        isinstance(lifecycle, Mapping)
        and lifecycle.get("requires_map") is True
    )
    global_planner_required = (
        plan is None
        or requires_map
        or bool({"global_planning", "octoplanner3d_global_planning"} & set(required_capabilities))
    )
    planner_map_required = plan is None or requires_map
    if global_planner_required:
        if not global_planner:
            blockers.append("native_global_planner_missing")
        elif global_planner != expected_global_planner:
            blockers.append("native_global_planner_mismatch")
    if planner_map_required and not planner_map:
        blockers.append("native_planner_map_missing")
    publish_cmd_vel = _as_optional_bool(snapshot.get("publish_cmd_vel"))
    if publish_cmd_vel is not True:
        blockers.append("native_cmd_vel_publish_disabled")
    return {
        "required": True,
        "ok": not blockers,
        "navigation_ready": not blockers,
        "blockers": blockers,
        "status_available": True,
        "active_cmd_source": active_cmd_source,
        "control_authority": control_authority,
        "stamp_s": _as_float(snapshot.get("stamp_s"), None),
        "control_loop_health": control_loop_health,
        "input_gate": input_gate,
        "control_mode": control_mode,
        "expected_control_mode": expected_control_mode,
        "native_product": observed_native_product,
        "parameter_mismatches": parameter_mismatches,
        "global_planner": global_planner,
        "expected_global_planner": expected_global_planner,
        "publish_cmd_vel": publish_cmd_vel,
        "assisted_teleop_required": assisted_teleop_required,
        "teleop_local_planner": teleop_local_planner,
        "check_obstacle": check_obstacle,
        "use_traversability_cost": use_traversability_cost,
        "far_input": dict(far_input),
        "operator_motion": {
            "required": operator_motion_required,
            "status_available": operator_motion_status_available,
            **operator_motion,
        },
    }


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
    native_endpoint: Mapping[str, Any],
) -> dict[str, Any]:
    blockers = _navigation_blockers(reason_codes)
    advisories = [code for code in reason_codes if code not in blockers]
    map_required = map_artifact_gate.get("required") is True
    real_required = real_runtime_evidence.get("required") is True
    return {
        "navigation_ready": can_accept_goal and not blockers,
        "can_accept_goal": can_accept_goal,
        "can_execute_autonomy": not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "tf_ok": bool(frames.get("ok", False)),
        "map_artifacts_ok": (map_artifact_gate.get("ok") is True if map_required else True),
        "real_runtime_evidence_ok": (real_runtime_evidence.get("ok") is True if real_required else None),
        "planning_frame_id": frames.get("planning_frame_id"),
        "odom_frame_id": frames.get("odom_frame_id"),
        "observed_frame_links": _mapping(frames.get("observed_frame_links")),
        "map_artifact_gate": dict(map_artifact_gate),
        "real_runtime_evidence": dict(real_runtime_evidence),
        "native_endpoint": dict(native_endpoint),
        "localization_ready": bool(localization.get("ready", False)),
        "control_owner": control.get("command_owner", "unknown"),
        "session_mode": _session_mode(session),
    }


def _mission_from_navigation_state(state: Mapping[str, Any]) -> dict[str, Any]:
    lifecycle = str(state.get("lifecycle_state_name") or "UNKNOWN").upper()
    return {
        "state": lifecycle,
        "ts": state.get("ts"),
        "frame_id": state.get("frame_id"),
        "planning_frame_id": state.get("frame_id"),
        "failure_reason": state.get("failure_code") or "",
        "active_task_id": state.get("active_task_id") or "",
        "active_request_id": state.get("active_request_id") or "",
        "goal_epoch": state.get("goal_epoch"),
        "map_id": state.get("map_id") or "",
        "map_content_epoch": state.get("map_content_epoch"),
        "hold_reason": state.get("hold_reason") or "",
        "progress": state.get("progress"),
    }


def build_navigation_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        native_state = _mapping(getattr(gw, "_navigation_state", None))
        mission = _mission_from_navigation_state(native_state)
        odometry = gw._odom
        path_len = len(gw._last_path)
        mode = gw._mode
        safety = gw._navigation_state
        task_statuses = getattr(gw, "_navigation_goal_status_by_task", {})
        goal_statuses = getattr(gw, "_navigation_goal_status_by_request", {})
        latest_goal_status = getattr(gw, "_latest_navigation_goal_status", None)
    state_source = "native_navigation_state"
    active_task_id = str(mission.get("active_task_id") or "")
    active_request_id = str(mission.get("active_request_id") or "")
    active_goal_status = (
        task_statuses.get(active_task_id)
        if active_task_id and hasattr(task_statuses, "get")
        else goal_statuses.get(active_request_id)
        if active_request_id and hasattr(goal_statuses, "get")
        else None
    )
    goal_status = (
        dict(active_goal_status)
        if isinstance(active_goal_status, Mapping)
        else dict(latest_goal_status)
        if isinstance(latest_goal_status, Mapping)
        else None
    )

    session = safe_session(gw)
    lease = safe_lease(gw)
    localization = build_localization_status(gw)
    native_endpoint = _native_endpoint_readiness(session, gw=gw)
    nav_runtime: dict[str, Any] = {}
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
        and session_mode in {"navigating", "exploring"}
    )
    control = _control_summary(
        mode=mode,
        lease=lease,
        mission_state=state,
        native_endpoint=native_endpoint,
    )
    if native_state:
        control.update(
            {
                "authority_source": "native_navigation_state",
                "command_owner": str(native_state.get("authority") or "unknown"),
                "control_mode": str(native_state.get("control_mode_name") or "UNKNOWN"),
                "hold_reason": str(native_state.get("hold_reason") or ""),
            }
        )
    control["safety_clear"] = not safety_stop_active(safety)
    progress = _progress_summary(
        state=state,
        wp_index=wp_index,
        wp_total=wp_total,
        path_points=path_len,
        replan_count=replan_count,
    )
    native_progress = _as_float(native_state.get("progress"), None)
    if native_progress is not None and native_progress >= 0.0:
        progress["fraction"] = max(0.0, min(1.0, native_progress))
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
    if isinstance(localization.get("map_odom_tf"), Mapping):
        frame_mission["map_odom_tf"] = dict(localization["map_odom_tf"])
    frames = _navigation_frame_summary(frame_mission, odometry)
    map_artifact_gate = _with_active_map_artifact_consistency(
        _map_artifact_gate_status(nav_runtime),
        session,
        localization,
    )
    real_runtime_evidence = _real_runtime_evidence_status(session)
    runtime_boundary = _runtime_boundary_status(gw)
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
    reason_codes.extend(str(code) for code in native_endpoint.get("blockers", []))
    reason_codes = list(dict.fromkeys(reason_codes))
    native_endpoint_ready = native_endpoint.get("required") is not True or native_endpoint.get("ok") is True
    can_accept_goal = base_can_accept_goal and native_endpoint_ready and not _navigation_blockers(reason_codes)
    readiness = _readiness_summary(
        can_accept_goal=can_accept_goal,
        reason_codes=reason_codes,
        session=session,
        localization=localization,
        control=control,
        frames=frames,
        map_artifact_gate=map_artifact_gate,
        real_runtime_evidence=real_runtime_evidence,
        native_endpoint=native_endpoint,
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
    return {
        "schema_version": NAVIGATION_STATUS_SCHEMA_VERSION,
        "state_source": state_source,
        "state": state,
        "has_odometry": odometry is not None,
        "can_accept_goal": can_accept_goal,
        "navigation_ready": readiness["navigation_ready"],
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
            "fastlio_velocity": localization.get("fastlio_velocity", {}),
            "fastlio_speed_mps": localization.get("fastlio_speed_mps"),
            "max_reasonable_speed_mps": localization.get("max_reasonable_speed_mps"),
            "speed_scale": speed_scale,
            "reasons": localization.get("reasons", []),
        },
        "native_endpoint": native_endpoint,
        "target": target,
        "motion": motion,
        "feedback": feedback,
        "diagnostics": {
            "reason_codes": reason_codes,
            "failure_reason": failure_reason,
            "localization_reasons": localization.get("reasons", []),
            "frame_mismatches": frames.get("mismatches", []),
            "safety": safety_summary(safety),
            "map_artifact_gate": map_artifact_gate,
            "real_runtime_evidence": real_runtime_evidence,
        },
        "mission": {
            "state": state,
            "raw": mission,
        },
        "goal_status": goal_status,
        "navigation_state": dict(native_state),
        "ts": mission_ts,
    }
