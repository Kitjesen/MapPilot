"""Localization status, session context, and runtime identity for Gateway."""

from __future__ import annotations

import logging
import os
import time
from collections.abc import Mapping
from dataclasses import asdict
from typing import Any

from runtime.runtime_policy import (
    backend_capability_defaults as _backend_capability_defaults,
)
from runtime.tf.conversions import map_from_odom_transform_from_mapping, map_from_odom_transform_to_dict

logger = logging.getLogger(__name__)


LOCALIZATION_STATUS_SCHEMA_VERSION = 1

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


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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


def compiled_session_context(gw: Any) -> dict[str, Any] | None:
    """Read immutable Host session context without querying map resources."""

    plan = getattr(gw, "_compiled_run_plan", None)
    lifecycle = getattr(plan, "lifecycle", None) if plan is not None else None
    if not isinstance(lifecycle, Mapping):
        return None
    product = str(getattr(plan, "product", "") or "").strip()
    if str(lifecycle.get("product") or "").strip() != product:
        raise RuntimeError("compiled RunPlan lifecycle Product mismatch")
    mode = str(lifecycle.get("session_mode") or "none").strip().lower()
    return {
        **runtime_identity(gw),
        "mode": "idle" if mode == "none" else mode,
        "slam_profile": str(lifecycle.get("slam_mode") or "none").strip().lower(),
        "requires_map": lifecycle.get("requires_map") is True,
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


def build_localization_status(
    gw: Any,
    *,
    facts: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if facts is None:
        from gateway.services.runtime_facts import capture_runtime_facts

        facts = capture_runtime_facts(gw)

    session = safe_session(gw)
    return build_localization_status_from_parts(
        facts.get("odometry"),
        session,
        float(facts.get("icp_quality", 0.0)),
        facts.get("localization_status"),
        gw=gw,
    )


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
    from runtime.tf.frames import normalize_frame_id

    normalized = normalize_frame_id(frame)
    if not normalized or normalized == "unknown" or normalized in expected_frames:
        return None
    expected = ",".join(expected_frames) if expected_frames else "unknown"
    return {
        "source": source,
        "expected_frame": expected,
        "received_frame": normalized,
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
    from diagnostics.runtime_contract import (
        DATA_SOURCE_CONTRACTS,
        REAL_RUNTIME_CONTRACT,
        RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
        canonical_data_source_name,
        resolved_runtime_data_flow,
        runtime_contract_data_source,
        runtime_data_flow_topics,
    )
    from runtime.tf.frames import (
        FRAME_LINKS,
        FRAMES,
        runtime_frames_contract,
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

    if data_source:
        source_contract = DATA_SOURCE_CONTRACTS.get(data_source)
        if source_contract is not None:
            source = asdict(source_contract)
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
        "frames": runtime_frames_contract() if data_source or runtime_contract else asdict(FRAMES),
        "frame_links": (
            {name: asdict(link) for name, link in FRAME_LINKS.items()}
            if data_source or runtime_contract else {}
        ),
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
    from runtime.tf.frames import normalize_frame_id

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
    from diagnostics.runtime_contract import canonical_data_source_name
    from message.topics import TOPICS
    from runtime.tf.frames import normalize_frame_id, runtime_required_topic_frame_ids, runtime_topic_expected_frame_ids

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
