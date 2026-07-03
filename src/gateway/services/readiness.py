"""Readiness snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
import math
from dataclasses import asdict, is_dataclass
from collections.abc import Mapping
from numbers import Real
from typing import Any

from gateway.services.safety_status import safety_stop_active, safety_summary


READINESS_SCHEMA_VERSION = 1
_CALIBRATION_CACHE_TTL_S = 30.0
_CALIBRATION_CACHE: tuple[float, dict[str, Any]] | None = None

_LOCALIZATION_BLOCKING_STATES = {
    "no_odometry",
    "initializing",
    "degraded",
    "lost",
    "relocalizing",
}

_DATA_RELEVANT_REASON_PREFIXES = (
    "localization:",
    "navigation_blocked:odometry_missing",
    "navigation_blocked:localization_lost",
    "navigation_blocked:localization_relocalizing",
    "navigation_blocked:localization_initializing",
    "navigation_blocked:pose_stale",
    "runtime_blocked:",
    "localization:status_error",
    "navigation:status_error",
)

_MISSION_ACTIVE_STATES = {
    "EXECUTING",
    "NAVIGATING",
    "PLANNING",
    "PAUSED",
    "EXPLORING",
    "RECOVERY",
    "RECOVERING",
    "REPLANNING",
}


def _json_safe(value: Any) -> Any:
    """Return a JSON-compliant value for readiness diagnostics."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Real):
        number = float(value)
        return number if math.isfinite(number) else None
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if hasattr(value, "model_dump"):
        try:
            return _json_safe(value.model_dump())
        except Exception:
            return str(value)
    if isinstance(value, dict):
        return {str(_json_safe(key)): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]
    return str(value)


def _requires_runtime_readiness(gw: Any, modules: Mapping[str, Any]) -> bool:
    """Return True when /ready should include robot runtime readiness.

    Stub/dev stacks may intentionally have no SLAM or odometry. Real robot
    stacks load SLAM/localizer/navigation modules or expose localization
    evidence, so module liveness alone is too optimistic there.
    """
    names = " ".join(str(name).lower() for name in modules)
    if any(
        token in names
        for token in (
            "slam",
            "localizer",
            "navigation",
            "cmdvelmux",
            "cmd_vel_mux",
        )
    ):
        return True

    session_mode = str(getattr(gw, "_session_mode", "") or "").lower()
    if session_mode in {"mapping", "navigating", "exploring"}:
        return True

    lock = getattr(gw, "_state_lock", None)
    if lock is None:
        return False
    try:
        with lock:
            return (
                getattr(gw, "_odom", None) is not None
                or getattr(gw, "_localization_status", None) is not None
                or getattr(gw, "_mission", None) is not None
            )
    except Exception:
        return False


def _source_name(control: Mapping[str, Any]) -> str:
    source = control.get("active_cmd_source")
    if source in (None, "", "unknown"):
        source = control.get("active_source")
    if isinstance(source, Mapping):
        source = source.get("name") or source.get("source") or source.get("owner")
    if source in (None, ""):
        return "none"
    return str(source)


def _runtime_readiness_modes(
    *,
    failed_modules: list[str],
    reasons: list[str],
    runtime: Mapping[str, Any],
) -> dict[str, Any]:
    navigation = runtime.get("navigation")
    navigation = navigation if isinstance(navigation, Mapping) else {}
    data_reasons = [
        reason
        for reason in reasons
        if str(reason).startswith(_DATA_RELEVANT_REASON_PREFIXES)
    ]
    active_cmd_source = str(navigation.get("active_cmd_source") or "unknown")
    mission_state = str(navigation.get("state") or "unknown").upper()
    motion_active = mission_state in _MISSION_ACTIVE_STATES
    command_source_idle = active_cmd_source.lower() in {"", "none", "unknown", "null"}
    data_ready = not failed_modules and not data_reasons
    non_motion_safe = command_source_idle and not motion_active
    motion_ready = not failed_modules and not reasons
    return {
        "data_ready": data_ready,
        "motion_ready": motion_ready,
        "non_motion_safe": non_motion_safe,
        "active_cmd_source": active_cmd_source,
        "mission_state": mission_state,
        "data_blockers": data_reasons,
    }


def _runtime_readiness_reasons(gw: Any) -> tuple[list[str], dict[str, Any]]:
    from gateway.services.runtime_status import (
        build_localization_status,
        build_navigation_status,
    )

    runtime: dict[str, Any] = {}
    reasons: list[str] = []
    try:
        localization = build_localization_status(gw)
        localization_runtime = localization.get("runtime")
        localization_runtime = (
            localization_runtime if isinstance(localization_runtime, Mapping) else {}
        )
        localization_frames = localization.get("frames")
        localization_frames = (
            localization_frames if isinstance(localization_frames, Mapping) else {}
        )
        runtime["localization"] = {
            "state": localization.get("state"),
            "ready": localization.get("ready"),
            "backend": localization.get("backend"),
            "algorithm_profile": localization.get("algorithm_profile"),
            "health_source": localization.get("health_source"),
            "native_mode": localization.get("native_mode"),
            "active_map": localization.get("active_map"),
            "map_state": localization.get("map_state"),
            "map_loaded": localization.get("map_loaded"),
            "map_save_source": localization.get("map_save_source"),
            "relocalization_state": localization.get("relocalization_state"),
            "relocalization_supported": localization.get("relocalization_supported"),
            "saved_map_relocalization_supported": localization.get(
                "saved_map_relocalization_supported"
            ),
            "restart_recovery_supported": localization.get(
                "restart_recovery_supported"
            ),
            "recovery_method": localization.get("recovery_method"),
            "recovery_action": localization.get("recovery_action"),
            "pose_fresh": localization.get("pose_fresh"),
            "pose_freshness": localization.get("pose_freshness"),
            "algorithm_healthy": localization.get("algorithm_healthy"),
            "runtime_contract": (
                localization_frames.get("runtime_contract")
                or localization_runtime.get("runtime_contract")
                or localization_runtime.get("data_source")
            ),
            "frames": _json_safe(localization_frames),
            "topic_allowed_frame_ids": _json_safe(
                localization_runtime.get("topic_allowed_frame_ids") or {}
            ),
            "topic_default_frame_ids": _json_safe(
                localization_runtime.get("topic_default_frame_ids") or {}
            ),
            "required_topic_frame_ids": _json_safe(
                localization_runtime.get("required_topic_frame_ids") or []
            ),
            "runtime_data_flow_topics": _json_safe(
                localization_runtime.get("runtime_data_flow_topics") or []
            ),
            "runtime_data_flow_stage_algorithm_interfaces": _json_safe(
                localization_runtime.get(
                    "runtime_data_flow_stage_algorithm_interfaces"
                )
                or {}
            ),
            "reasons": localization.get("reasons", []),
        }
        state = str(localization.get("state") or "unknown").lower()
        if state in _LOCALIZATION_BLOCKING_STATES:
            reasons.append(f"localization:{state}")
        if localization.get("pose_fresh") is False:
            reasons.append("localization:pose_stale")
    except Exception as exc:
        runtime["localization"] = {"error": str(exc)}
        reasons.append("localization:status_error")

    try:
        navigation = build_navigation_status(gw)
        readiness = navigation.get("readiness", {})
        blockers = list(readiness.get("blockers") or [])
        control = navigation.get("control") or {}
        boundary = navigation.get("runtime")
        boundary = boundary if isinstance(boundary, Mapping) else {}
        boundary_blockers = list(boundary.get("blockers") or [])
        runtime["navigation"] = {
            "state": navigation.get("state"),
            "can_accept_goal": navigation.get("can_accept_goal"),
            "blockers": blockers,
            "advisories": list(readiness.get("advisories") or []),
            "tf_ok": readiness.get("tf_ok"),
            "map_artifacts_ok": readiness.get("map_artifacts_ok"),
            "real_runtime_evidence_ok": readiness.get("real_runtime_evidence_ok"),
            "planning_frame_id": readiness.get("planning_frame_id"),
            "odom_frame_id": readiness.get("odom_frame_id"),
            "map_artifact_gate": _json_safe(
                readiness.get("map_artifact_gate") or {}
            ),
            "observed_frame_links": _json_safe(
                readiness.get("observed_frame_links") or {}
            ),
            "active_cmd_source": _source_name(control),
        }
        if boundary:
            runtime["boundary"] = {
                "ok": boundary.get("ok"),
                "declared": boundary.get("declared"),
                "profile": boundary.get("profile"),
                "endpoint": boundary.get("endpoint"),
                "data_source": boundary.get("data_source"),
                "runtime_contract": boundary.get("runtime_contract"),
                "command_sink": boundary.get("command_sink"),
                "expected_command_sink": boundary.get("expected_command_sink"),
                "frames": _json_safe(boundary.get("frames") or {}),
                "frame_links": _json_safe(boundary.get("frame_links") or {}),
                "topic_allowed_frame_ids": _json_safe(
                    boundary.get("topic_allowed_frame_ids") or {}
                ),
                "topic_default_frame_ids": _json_safe(
                    boundary.get("topic_default_frame_ids") or {}
                ),
                "required_topic_frame_ids": _json_safe(
                    boundary.get("required_topic_frame_ids") or []
                ),
                "runtime_data_flow_topics": _json_safe(
                    boundary.get("runtime_data_flow_topics") or []
                ),
                "resolved_runtime_data_flow": _json_safe(
                    boundary.get("resolved_runtime_data_flow") or []
                ),
                "runtime_data_flow_stage_algorithm_interfaces": _json_safe(
                    boundary.get("runtime_data_flow_stage_algorithm_interfaces") or {}
                ),
                "blockers": boundary_blockers,
            }
        reasons.extend(f"navigation_blocked:{blocker}" for blocker in blockers)
        reasons.extend(f"runtime_blocked:{blocker}" for blocker in boundary_blockers)
    except Exception as exc:
        runtime["navigation"] = {"error": str(exc)}
        reasons.append("navigation:status_error")

    try:
        with gw._state_lock:
            safety = getattr(gw, "_safety", None)
        runtime["safety"] = safety_summary(safety)
        if safety_stop_active(safety):
            reasons.append("safety:stop")
    except Exception as exc:
        runtime["safety"] = {"error": str(exc)}
        reasons.append("safety:status_error")

    calibration = _calibration_status()
    runtime["calibration"] = calibration
    if calibration.get("errors"):
        reasons.append("calibration:error")

    return list(dict.fromkeys(reasons)), runtime


def _calibration_status(now: float | None = None) -> dict[str, Any]:
    """Return cached startup calibration diagnostics for client readiness."""
    global _CALIBRATION_CACHE
    ts = time.time() if now is None else now
    if _CALIBRATION_CACHE is not None:
        cached_ts, cached = _CALIBRATION_CACHE
        if ts - cached_ts < _CALIBRATION_CACHE_TTL_S:
            return cached
    try:
        from runtime.utils.calibration_check import run_calibration_check

        report = run_calibration_check(require_camera=False, require_slam=False)
        payload = {
            "ok": report.ok,
            "errors": list(report.errors),
            "warnings": list(report.warnings),
            "info_count": len(report.info),
            "summary": report.summary(),
        }
    except Exception as exc:
        payload = {
            "ok": False,
            "errors": [str(exc)],
            "warnings": [],
            "info_count": 0,
            "summary": "Calibration check unavailable",
        }
    _CALIBRATION_CACHE = (ts, payload)
    return payload


def build_readiness_snapshot(
    gw: Any,
    now: float | None = None,
    *,
    include_details: bool = True,
) -> tuple[dict[str, Any], int]:
    """Return a stable /ready payload plus the HTTP status code."""
    ts = time.time() if now is None else now
    modules = getattr(gw, "_all_modules", None) or {}
    if not modules:
        return (
            {
                "schema_version": READINESS_SCHEMA_VERSION,
                "status": "not_started",
                "ready": False,
                "data_ready": False,
                "motion_ready": False,
                "non_motion_safe": True,
                "modules": {},
                "module_count": 0,
                "failed_modules": [],
                "reasons": ["no_modules_loaded"],
                "advisories": [],
                "ts": ts,
            },
            503,
        )

    module_health: dict[str, Any] = {}
    failed_modules: list[str] = []

    for name, mod in modules.items():
        try:
            module_health[name] = {"ok": True}
            if include_details:
                detail = mod.health() if hasattr(mod, "health") else {}
                module_health[name]["detail"] = _json_safe(detail)
        except Exception as exc:
            failed_modules.append(str(name))
            module_health[name] = {"ok": False, "error": str(exc)}

    reasons = [
        f"module_failed:{name}"
        for name in failed_modules
    ]
    runtime: dict[str, Any] = {}
    if _requires_runtime_readiness(gw, modules):
        runtime_reasons, runtime = _runtime_readiness_reasons(gw)
        reasons.extend(runtime_reasons)

    ready = not failed_modules and not reasons
    modes = _runtime_readiness_modes(
        failed_modules=failed_modules,
        reasons=reasons,
        runtime=runtime,
    )
    if runtime:
        runtime["summary"] = modes
    calibration = runtime.get("calibration")
    advisories = []
    if isinstance(calibration, Mapping):
        advisories.extend(str(w) for w in (calibration.get("warnings") or []) if w)
    payload = {
        "schema_version": READINESS_SCHEMA_VERSION,
        "status": "ready" if ready else "degraded",
        "ready": ready,
        "data_ready": modes["data_ready"],
        "motion_ready": modes["motion_ready"],
        "non_motion_safe": modes["non_motion_safe"],
        "modules": module_health,
        "module_count": len(modules),
        "failed_modules": failed_modules,
        "reasons": reasons,
        "advisories": list(dict.fromkeys(advisories)),
        "runtime": runtime,
        "ts": ts,
    }
    return payload, 200 if ready else 503
