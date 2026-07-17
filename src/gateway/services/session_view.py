"""Product-session view helpers for GatewayModule."""

from __future__ import annotations

import logging
import os
import time
from typing import Any

from gateway.services.runtime_status import (
    backend_capability_defaults,
    classify_pose_freshness,
    localizer_algorithm_healthy,
    runtime_switch_pending,
)
from gateway.services.safety_status import (
    SAFETY_STOP_BLOCKER,
    safety_clear_for_motion,
    safety_summary,
)
from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS
from runtime.profiles.resolver import canonical_profile_name

logger = logging.getLogger(__name__)


def recover_external_mapping_session(gw: Any) -> bool:
    """Restore the stateless mapping session after an external-runtime restart.

    The field ``map`` profile owns its SLAM process outside Gateway.  Gateway's
    in-memory session cache therefore disappears on a Gateway restart even
    though the product graph and native SLAM stay in mapping mode.  Recovering
    from the immutable runtime profile keeps the first incoming cloud in the
    mapping lifecycle without attempting to start or stop robot services.
    """
    if bool(getattr(gw, "_manage_session_services", True)):
        return False
    if str(getattr(gw, "_session_mode", "idle") or "idle").lower() != "idle":
        return False

    profile = canonical_profile_name(str(os.environ.get("LINGTU_PROFILE") or "").strip())
    contract = PRODUCT_MODE_CONTRACTS.get(profile)
    if contract is None or contract.product_mode != "mapping":
        return False

    gw._session_mode = "mapping"
    gw._session_product_profile = profile
    gw._session_product_session = contract.product_session
    gw._session_map = None
    gw._session_since = time.time()
    return True


def detect_current_mode(gw: Any) -> tuple[str, str | None]:
    """Reflect what's actually running into session state."""
    if not gw._manage_session_services:
        recover_external_mapping_session(gw)
        return gw._session_mode, gw._session_active_map_name()
    try:
        from runtime.service_manager import get_service_manager

        svc = get_service_manager()
        status = svc.status(
            "slam",
            "slam_pgo",
            "localizer",
            "genz_icp",
            "super_lio",
            "super_lio_relocation",
        )
        slam_active = status.get("slam") in ("running", "active")
        pgo_active = status.get("slam_pgo") in ("running", "active")
        loc_active = status.get("localizer") in ("running", "active")
        genz_active = status.get("genz_icp") in ("running", "active")
        super_lio_active = status.get("super_lio") in ("running", "active")
        super_lio_relocation_active = status.get("super_lio_relocation") in (
            "running",
            "active",
        )
        if super_lio_relocation_active:
            return "navigating", gw._session_active_map_name()
        if super_lio_active:
            if gw._exploring:
                return "exploring", None
            if gw._session_mode in ("mapping", "navigating"):
                return gw._session_mode, (gw._session_active_map_name() if gw._session_mode == "navigating" else None)
            return "mapping", None
        if genz_active:
            if gw._exploring:
                return "exploring", None
            if gw._session_mode in ("mapping", "navigating"):
                return gw._session_mode, (gw._session_active_map_name() if gw._session_mode == "navigating" else None)
            return "mapping", None
        if loc_active and slam_active:
            return "navigating", gw._session_active_map_name()
        if pgo_active and slam_active:
            if gw._exploring:
                return "exploring", None
            return "mapping", None
        if slam_active and gw._session_mode in {"mapping", "exploring", "navigating"}:
            if gw._exploring:
                return "exploring", None
            return gw._session_mode, (gw._session_active_map_name() if gw._session_mode == "navigating" else None)
        if gw._exploring and gw._session_uses_external_slam_none():
            return "exploring", None
        return "idle", None
    except Exception as exc:
        logger.warning(
            "session_view.detect_current_mode: service_manager lookup failed: %s",
            exc,
        )
        if gw._exploring and gw._session_uses_external_slam_none():
            return "exploring", None
        return "idle", None


def session_snapshot(gw: Any) -> dict[str, Any]:
    """Build the product session payload for HTTP, SSE, and bootstrap views."""
    saved_active_map = gw._session_active_map_name()
    active_map = gw._session_map if gw._session_mode == "navigating" and gw._session_map else None
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

    transition_pending = bool(gw._session_pending or runtime_switch_pending(gw))
    idle = gw._session_mode == "idle" and not transition_pending
    can_start_mapping = idle
    can_start_navigating = idle and saved_active_map is not None and has_pcd and has_octomap
    explorer_backend = gw._explorer_backend()
    explorer_available = explorer_backend != "none"
    explorer_detail = {} if explorer_available else gw._explorer_unavailable_detail()
    explorer_unavailable_reason = None if explorer_available else explorer_detail.get("reason")
    explorer_required_profile = None if explorer_available else explorer_detail.get("required_profile")
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
    return {
        "mode": gw._session_mode,
        "product_session": gw._session_product_session,
        "product_profile": gw._session_product_profile,
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
        "explorer_required_profile": explorer_required_profile,
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
    if str(gw._session_mode or "idle").lower() != "idle":
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
