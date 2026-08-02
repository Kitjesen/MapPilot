"""Product-session view helpers for GatewayModule."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from gateway.services.runtime_status import (
    backend_capability_defaults,
    classify_pose_freshness,
    localizer_algorithm_healthy,
    runtime_identity,
)
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_clear_for_motion,
    safety_summary,
)


def recover_external_mapping_session(gw: Any) -> bool:
    """Restore the stateless mapping session after an external-runtime restart.

    The field ``map`` Product owns its SLAM process outside Gateway. Gateway's
    in-memory session cache therefore disappears on a Gateway restart even
    though the product graph and native SLAM stay in mapping mode.  Recovering
    from the immutable RunPlan keeps the first incoming cloud in the
    mapping lifecycle without attempting to start or stop robot services.
    """
    if str(getattr(gw, "_session_mode", "idle") or "idle").lower() != "idle":
        return False

    plan = getattr(gw, "_compiled_run_plan", None)
    if plan is None:
        return False
    product = str(getattr(plan, "product", "") or "").strip()
    lifecycle = getattr(plan, "lifecycle", None)
    if not isinstance(lifecycle, Mapping):
        return False
    if str(lifecycle.get("product") or "").strip() != product:
        return False
    if str(lifecycle.get("product_mode") or "").strip() != "mapping":
        return False

    gw._session_mode = "mapping"
    gw._session_product = product
    gw._session_product_session = str(lifecycle.get("product_session") or "").strip()
    gw._session_map = None
    gw._session_since = time.time()
    return True


def recover_external_explore_session(gw: Any) -> bool:
    """Recover only an exactly committed, native-backed Explore session."""

    if str(getattr(gw, "_session_mode", "idle") or "idle").lower() != "idle":
        return False

    from gateway.services.exploration import (
        ExplorationRunError,
        external_explore_binding,
        reconcile_exploration_runtime,
    )

    binding = external_explore_binding(gw)
    if not binding.get("valid"):
        return False

    plan = getattr(gw, "_compiled_run_plan", None)
    lifecycle = getattr(plan, "lifecycle", None)
    status = binding.get("native_status")
    if not isinstance(lifecycle, Mapping) or not isinstance(status, Mapping):
        return False

    try:
        reconcile_exploration_runtime(gw, binding)
    except ExplorationRunError:
        # Product session recovery remains truthful even when the durable run
        # projection is unavailable. Explore readiness exposes that blocker and
        # prevents new motion admission.
        pass

    gw._session_mode = "exploring"
    gw._session_product = "explore"
    gw._session_product_session = str(
        lifecycle.get("product_session") or "exploration"
    ).strip()
    gw._session_map = binding.get("map_name") or binding.get("map_id")
    gw._session_slam_profile = str(lifecycle.get("slam_mode") or "").strip()
    committed_at = binding.get("committed_at")
    try:
        gw._session_since = float(committed_at)
    except (TypeError, ValueError):
        gw._session_since = time.time()
    gw._exploring = status.get("active") is True
    return True


def detect_current_mode(gw: Any) -> tuple[str, str | None]:
    """Return logical session state without inferring it from process state."""
    recover_external_mapping_session(gw)
    recover_external_explore_session(gw)
    active_map = (
        gw._session_map
        if gw._session_mode in {"navigating", "exploring"} and gw._session_map
        else None
    )
    return gw._session_mode, active_map


def session_snapshot(gw: Any) -> dict[str, Any]:
    """Build the product session payload for HTTP, SSE, and bootstrap views."""
    saved_active_map = gw._session_active_map_name()
    active_map = (
        gw._session_map
        if gw._session_mode in {"navigating", "exploring"} and gw._session_map
        else None
    )
    artifact_map = active_map or saved_active_map
    has_octomap = False
    has_pcd = False
    if artifact_map:
        has_pcd = (
            gw._map_bundle_from_maps_service(
                artifact_map,
                "source_pointcloud",
            )
            is not None
        )
        has_octomap = (
            gw._map_bundle_from_maps_service(
                artifact_map,
                "navigation_safety_3d",
            )
            is not None
        )
    icp = gw._icp_quality
    localization_status = gw._localization_status or {}
    session_profile = str(gw._session_slam_profile or "").strip().lower()
    if gw._session_mode != "idle" and session_profile not in {"", "stopped"}:
        slam_profile = session_profile
    else:
        slam_profile = gw._get_slam_profile()
    backend = str(localization_status.get("backend") or slam_profile or gw._session_slam_profile or "stopped").lower()
    if str(localization_status.get("health_source") or "").lower() == "slam_runtime":
        backend = "native_dds"
    pose_fresh, pose_freshness = classify_pose_freshness(localization_status)
    algorithm_healthy = localizer_algorithm_healthy(localization_status, icp)
    loc_ready = algorithm_healthy and pose_fresh is not False
    capability_defaults = backend_capability_defaults(backend)
    relocalization_supported = localization_status.get("relocalization_supported")
    if relocalization_supported is None:
        relocalization_supported = capability_defaults["relocalization_supported"]
    saved_map_relocalization_supported = localization_status.get("saved_map_relocalization_supported")
    if saved_map_relocalization_supported is None:
        saved_map_relocalization_supported = relocalization_supported
    restart_recovery_supported = localization_status.get("restart_recovery_supported")
    if restart_recovery_supported is None:
        restart_recovery_supported = capability_defaults["restart_recovery_supported"]
    recovery_method = localization_status.get("recovery_method")
    if not recovery_method:
        recovery_method = capability_defaults["recovery_method"]
    map_save_supported = localization_status.get("map_save_supported")
    if map_save_supported is None:
        map_save_supported = capability_defaults["map_save_supported"]
    map_save_source = localization_status.get("map_save_source")
    if map_save_source is None:
        map_save_source = capability_defaults["map_save_source"]

    transition_pending = bool(gw._session_pending)
    idle = gw._session_mode == "idle" and not transition_pending
    can_start_mapping = idle
    can_start_navigating = idle and saved_active_map is not None and has_pcd and has_octomap
    explorer_backend = gw._explorer_backend()
    explorer_available = explorer_backend != "none"
    explorer_detail = {} if explorer_available else gw._explorer_unavailable_detail()
    explorer_unavailable_reason = None if explorer_available else explorer_detail.get("reason")
    explorer_required_product = None if explorer_available else explorer_detail.get("required_product")
    safety = safety_summary(gw._safety)
    safety_clear = safety_clear_for_motion(gw._safety)
    exploration_blockers = _exploration_blockers(
        gw,
        explorer_available=explorer_available,
        safety_clear=safety_clear,
        localization_status=localization_status,
        pose_fresh=pose_fresh,
        algorithm_healthy=algorithm_healthy,
    )
    identity = runtime_identity(gw)
    return {
        "mode": gw._session_mode,
        "env": identity["env"],
        "product": gw._session_product or identity["product"],
        "product_session": gw._session_product_session,
        "slam_profile": slam_profile,
        "localization_backend": backend,
        "health_source": localization_status.get("health_source"),
        "active_map": active_map,
        "saved_active_map": saved_active_map,
        "map_has_pcd": has_pcd,
        "map_has_octomap": has_octomap,
        "since": gw._session_since,
        "pending": transition_pending,
        "error": gw._session_error,
        "icp_quality": icp,
        "localizer_ready": loc_ready,
        "localizer_algorithm_healthy": algorithm_healthy,
        "pose_fresh": pose_fresh,
        "pose_freshness": pose_freshness,
        "map_state": localization_status.get("map_state"),
        "map_save_supported": bool(map_save_supported),
        "map_save_source": map_save_source,
        "relocalization_supported": bool(relocalization_supported),
        "saved_map_relocalization_supported": bool(saved_map_relocalization_supported),
        "restart_recovery_supported": bool(restart_recovery_supported),
        "recovery_method": recovery_method,
        "relocalization_state": localization_status.get("relocalization_state"),
        "recovery_signal": localization_status.get("recovery_signal"),
        "recovery_action": localization_status.get("recovery_action"),
        "can_start_mapping": can_start_mapping,
        "can_start_navigating": can_start_navigating,
        "can_start_exploring": not exploration_blockers,
        "exploration_blockers": exploration_blockers,
        "safety_clear": safety_clear,
        "safety": safety,
        "can_end": gw._session_mode != "idle" and not transition_pending,
        "explorer_backend": explorer_backend,
        "explorer_available": explorer_available,
        "explorer_unavailable_reason": explorer_unavailable_reason,
        "explorer_required_product": explorer_required_product,
    }


def _exploration_blockers(
    gw: Any,
    *,
    explorer_available: bool,
    safety_clear: bool,
    localization_status: dict[str, Any],
    pose_fresh: bool | None,
    algorithm_healthy: bool,
) -> list[str]:
    blockers: list[str] = []
    if not explorer_available:
        blockers.append("explorer_backend_not_running")
    if gw._session_pending:
        blockers.append("session_transition_pending")
    if gw._exploring:
        blockers.append("exploration_already_active")
    session_mode = str(gw._session_mode or "idle").lower()
    staged_field_explore = False
    if session_mode == "exploring" and not gw._exploring:
        try:
            from gateway.services.exploration import (
                _native_start_idle,
                external_explore_binding,
            )

            binding = external_explore_binding(gw)
            native_status = binding.get("native_status")
            staged_field_explore = bool(
                binding.get("valid")
                and isinstance(native_status, Mapping)
                and _native_start_idle(native_status)
            )
        except Exception:
            staged_field_explore = False
    if session_mode != "idle" and not staged_field_explore:
        blockers.append("session_not_idle")
    if gw._mode == "estop":
        blockers.append("estop_active")
    if not safety_clear:
        blockers.append(SAFETY_STOP_BLOCKER)
    if gw._odom is None:
        blockers.append("odometry_missing")
    localization_state = str(localization_status.get("state") or "").strip().lower()
    if localization_state in {"degraded", "lost", "relocalizing", "initializing"}:
        blockers.append(f"localization_{localization_state}")
    recovery_signal = str(localization_status.get("recovery_signal") or "").strip().upper()
    if recovery_signal not in {"", "NONE", "RECOVERED"}:
        blockers.append("localization_recovery_active")
    if pose_fresh is False and algorithm_healthy:
        blockers.append("pose_stale")
    return list(dict.fromkeys(blockers))
