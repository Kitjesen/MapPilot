"""Readiness snapshot helpers for GatewayModule."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from numbers import Real
from typing import Any

from gateway.services.safety_status import safety_stop_active, safety_summary
from runtime.runtime_interface import TOPICS

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
    "maps:",
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

_NATIVE_OPERATOR_TOPICS = frozenset(
    {
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
    }
)
_NATIVE_OPERATOR_CAPABILITIES = frozenset(
    {
        "operator_motion_typed_dds_interface",
        "native_operator_motion_authority",
    }
)


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


def _compiled_product_contract(gw: Any) -> tuple[str, Any | None]:
    plan = getattr(gw, "_compiled_run_plan", None)
    if plan is not None:
        return str(getattr(plan, "product", "") or "").strip(), plan
    return str(getattr(gw, "_compiled_product", "") or "").strip(), None


def _run_plan_process_names(plan: Any | None) -> frozenset[str]:
    if plan is None:
        return frozenset()
    return frozenset(
        str(getattr(process, "name", "") or "").strip()
        for process in getattr(plan, "processes", ())
        if str(getattr(process, "name", "") or "").strip()
    )


def _managed_run_plan_missing(gw: Any) -> bool:
    return bool(
        getattr(gw, "_compiled_run_plan", None) is None
        and (
            getattr(gw, "_compiled_run_plan_fingerprint", "")
            or getattr(gw, "_compiled_command_output_mode", "") == "endpoint_only"
        )
    )


def _contract_requires_native_readiness(gw: Any, contract: Any | None) -> bool:
    if contract is None:
        return False
    return bool(
        getattr(gw, "_compiled_command_output_mode", "") == "endpoint_only"
        or "nav" in _run_plan_process_names(contract)
        or _NATIVE_OPERATOR_TOPICS
        & frozenset(getattr(contract, "required_topics", ()))
        or _NATIVE_OPERATOR_CAPABILITIES
        & frozenset(getattr(contract, "required_capabilities", ()))
    )


def _product_contract_summary(gw: Any) -> dict[str, Any]:
    product, contract = _compiled_product_contract(gw)
    return {
        "product": product or None,
        "fingerprint": (
            getattr(gw, "_compiled_run_plan_fingerprint", "") or None
        ),
        "command_output_mode": (
            getattr(gw, "_compiled_command_output_mode", "") or None
        ),
        "hardware_control_boundary": (
            getattr(gw, "_compiled_hardware_control_boundary", "") or None
        ),
        "processes": sorted(_run_plan_process_names(contract)),
        "required_topics": (
            sorted(getattr(contract, "required_topics", ()))
            if contract is not None
            else []
        ),
        "required_capabilities": (
            sorted(getattr(contract, "required_capabilities", ()))
            if contract is not None
            else []
        ),
        "native_readiness_required": _contract_requires_native_readiness(
            gw,
            contract,
        ),
    }


def _requires_runtime_readiness(gw: Any, modules: Mapping[str, Any]) -> bool:
    """Return True when /ready should include robot runtime readiness.

    Stub/dev stacks may intentionally have no SLAM or odometry. Real robot
    stacks load SLAM/localizer/navigation modules or expose localization
    evidence, so module liveness alone is too optimistic there.
    """
    _product, contract = _compiled_product_contract(gw)
    if _contract_requires_native_readiness(gw, contract):
        return True

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


def _native_maps_readiness(
    gw: Any,
    modules: Mapping[str, Any],
) -> tuple[list[str], dict[str, Any]]:
    """Enforce fresh/current mapd state and scene for products that require it."""

    _profile, contract = _compiled_product_contract(gw)
    if contract is None or not (
        {TOPICS.maps_state, TOPICS.maps_scene}
        & set(getattr(contract, "required_topics", ()))
    ):
        return [], {}
    host_bus = modules.get("host.bus")
    if host_bus is None:
        return ["maps:host_bus_missing"], {"required": True}
    try:
        reason = (
            host_bus.map_readiness()
            if hasattr(host_bus, "map_readiness")
            else "map_readiness_contract_missing"
        )
        health = host_bus.health() if hasattr(host_bus, "health") else {}
    except Exception as exc:
        return [f"maps:readiness_error:{exc}"], {
            "required": True,
            "error": str(exc),
        }
    detail = health.get("map_scene") if isinstance(health, Mapping) else {}
    return (
        [f"maps:{reason}"] if reason else [],
        {
            "required": True,
            "ready": reason is None,
            "reason": reason,
            "host_bus": _json_safe(detail),
        },
    )


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
                "env": boundary.get("env"),
                "product": boundary.get("product"),
                "run_plan_fingerprint": boundary.get(
                    "run_plan_fingerprint"
                ),
                "identity_source": boundary.get("identity_source"),
                "simulation_only": boundary.get("simulation_only"),
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


def _normalize_startup_status(source: Any, *, source_name: str) -> dict[str, Any]:
    """Return normalized runtime startup evidence for readiness."""

    if source is None:
        return {
            "startup_state": None,
            "critical_modules": [],
            "critical_failed_modules": [],
            "optional_failed_modules": set(),
            "advisories": [],
        }

    try:
        raw_state = source.startup_state
        raw_critical_modules = source.critical_modules
        raw_failed_modules = source.failed_modules
        raw_critical_failures = source.critical_failures
    except Exception as exc:
        return {
            "startup_state": "failed",
            "critical_modules": [],
            "critical_failed_modules": [source_name],
            "optional_failed_modules": set(),
            "advisories": [
                f"{source_name.lower()}_status_error:{type(exc).__name__}"
            ],
        }

    startup_state = str(raw_state)
    if isinstance(raw_critical_modules, (list, tuple, set)):
        critical_modules = list(
            dict.fromkeys(str(name) for name in raw_critical_modules)
        )
    else:
        critical_modules = []
    failed_modules = (
        {str(name): str(reason) for name, reason in raw_failed_modules.items()}
        if isinstance(raw_failed_modules, Mapping)
        else {}
    )
    critical_failures = (
        {str(name): str(reason) for name, reason in raw_critical_failures.items()}
        if isinstance(raw_critical_failures, Mapping)
        else {}
    )

    declared_critical = set(critical_modules)
    critical_failed_modules = [
        name
        for name in critical_modules
        if name in failed_modules or name in critical_failures
    ]
    critical_failed_modules.extend(
        name
        for name in critical_failures
        if name not in critical_failed_modules
    )
    optional_failures = {
        name: reason
        for name, reason in failed_modules.items()
        if name not in declared_critical and name not in critical_failures
    }
    advisories = [
        f"optional_module_failed:{name}:{reason}"
        for name, reason in optional_failures.items()
    ]
    try:
        raw_advisories = getattr(source, "advisories", ())
    except Exception:
        raw_advisories = ()
    if isinstance(raw_advisories, (list, tuple, set)):
        advisories.extend(str(item) for item in raw_advisories if item)
    return {
        "startup_state": startup_state,
        "critical_modules": critical_modules,
        "critical_failed_modules": critical_failed_modules,
        "optional_failed_modules": set(optional_failures),
        "advisories": advisories,
    }


def _system_startup_status(gw: Any) -> dict[str, Any]:
    """Return normalized runtime startup evidence for readiness."""

    provider = getattr(gw, "_runtime_status_provider", None)
    if provider is not None:
        return _normalize_startup_status(
            provider,
            source_name=type(provider).__name__ or "RuntimeStatusProvider",
        )
    return _normalize_startup_status(
        getattr(gw, "_system_handle", None),
        source_name="SystemHandle",
    )


def _runtime_modules(gw: Any) -> Mapping[str, Any]:
    provider = getattr(gw, "_runtime_status_provider", None)
    if provider is not None:
        try:
            modules = provider.modules
        except Exception:
            modules = None
        if isinstance(modules, Mapping):
            return modules
    modules = getattr(gw, "_all_modules", None)
    return modules if isinstance(modules, Mapping) else {}


def build_readiness_snapshot(
    gw: Any,
    now: float | None = None,
    *,
    include_details: bool = True,
) -> tuple[dict[str, Any], int]:
    """Return a stable /ready payload plus the HTTP status code."""
    ts = time.time() if now is None else now
    product_contract = _product_contract_summary(gw)
    startup = _system_startup_status(gw)
    startup_state = startup["startup_state"]
    critical_modules = startup["critical_modules"]
    critical_failed_modules = startup["critical_failed_modules"]
    startup_blocked = startup_state is not None and startup_state != "ready"
    startup_reasons = [
        *(f"critical_module_failed:{name}" for name in critical_failed_modules),
        *([f"startup_state:{startup_state}"] if startup_blocked else []),
    ]
    modules = _runtime_modules(gw)
    if not modules:
        reasons = ["no_modules_loaded", *startup_reasons]
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
                "failed_modules": list(critical_failed_modules),
                "critical_failed_modules": list(critical_failed_modules),
                "critical_modules": list(critical_modules),
                "startup_state": startup_state,
                "reasons": list(dict.fromkeys(reasons)),
                "advisories": list(startup["advisories"]),
                "product_contract": product_contract,
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
            module_name = str(name)
            if module_name not in startup["optional_failed_modules"]:
                failed_modules.append(module_name)
            module_health[name] = {"ok": False, "error": str(exc)}

    failed_modules.extend(
        name for name in critical_failed_modules if name not in failed_modules
    )
    reasons = [f"module_failed:{name}" for name in failed_modules]
    reasons.extend(startup_reasons)
    if _managed_run_plan_missing(gw):
        reasons.append("run_plan_missing")
    reasons = list(dict.fromkeys(reasons))
    runtime: dict[str, Any] = {}
    if _requires_runtime_readiness(gw, modules):
        runtime_reasons, runtime = _runtime_readiness_reasons(gw)
        reasons.extend(runtime_reasons)
    map_reasons, map_runtime = _native_maps_readiness(gw, modules)
    reasons.extend(map_reasons)
    if map_runtime:
        runtime["maps"] = map_runtime

    ready = not failed_modules and not reasons
    modes = _runtime_readiness_modes(
        failed_modules=failed_modules,
        reasons=reasons,
        runtime=runtime,
    )
    if startup_blocked:
        modes["data_ready"] = False
        modes["data_blockers"] = list(
            dict.fromkeys([*modes["data_blockers"], f"startup_state:{startup_state}"])
        )
    if runtime:
        runtime["summary"] = modes
    calibration = runtime.get("calibration")
    advisories = list(startup["advisories"])
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
        "critical_failed_modules": list(critical_failed_modules),
        "critical_modules": list(critical_modules),
        "startup_state": startup_state,
        "reasons": reasons,
        "advisories": list(dict.fromkeys(advisories)),
        "product_contract": product_contract,
        "runtime": runtime,
        "ts": ts,
    }
    return payload, 200 if ready else 503
