"""Product-session view helpers for GatewayModule."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from gateway.services.mapd_transport import mapd_query
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


def _runtime_projection(gw: Any) -> dict[str, Any]:
    """Project Product session identity from RunPlan and native mapd."""

    identity = runtime_identity(gw)
    plan = getattr(gw, "_compiled_run_plan", None)
    lifecycle = getattr(plan, "lifecycle", None) if plan is not None else None
    mode = "idle"
    slam_profile = "stopped"
    requires_map = False
    if isinstance(lifecycle, Mapping):
        product = str(getattr(plan, "product", "") or "").strip()
        if str(lifecycle.get("product") or "").strip() != product:
            raise RuntimeError("compiled RunPlan lifecycle Product mismatch")
        declared_mode = str(lifecycle.get("session_mode") or "none").strip().lower()
        mode = "idle" if declared_mode == "none" else declared_mode
        slam_profile = str(lifecycle.get("slam_mode") or "none").strip().lower()
        requires_map = lifecycle.get("requires_map") is True

    saved_active_map = gw._session_active_map_name()
    active_map = (
        saved_active_map
        if mode in {"navigating", "exploring"} and requires_map
        else None
    )
    return {
        **identity,
        "mode": mode,
        "slam_profile": slam_profile,
        "active_map": active_map,
        "saved_active_map": saved_active_map,
    }


def refresh_session_projection(gw: Any) -> dict[str, Any]:
    """Refresh fields used by read-only Gateway consumers."""

    projection = _runtime_projection(gw)
    key = (
        projection["mode"],
        projection["product"],
        projection["active_map"],
        projection["product_session_id"],
    )
    if getattr(gw, "_session_projection_key", None) != key:
        gw._session_projection_key = key
        gw._session_since = time.time()
    gw._session_mode = projection["mode"]
    gw._session_product = projection["product"]
    gw._session_map = projection["active_map"]
    gw._session_slam_profile = projection["slam_profile"]
    return projection


def reconcile_native_explore(gw: Any) -> bool:
    """Reconcile native Explore state for an active Explore Product."""

    if refresh_session_projection(gw)["mode"] != "exploring":
        return False

    from gateway.services.exploration import (
        ExplorationRunError,
        external_explore_binding,
        reconcile_exploration_runtime,
    )

    binding = external_explore_binding(gw)
    if not binding.get("valid"):
        return False

    status = binding.get("native_status")
    if not isinstance(status, Mapping):
        return False

    try:
        reconcile_exploration_runtime(gw, binding)
    except ExplorationRunError:
        # Product session recovery remains truthful even when the durable run
        # projection is unavailable. Explore readiness exposes that blocker and
        # prevents new motion admission.
        pass

    gw._exploring = status.get("active") is True
    return True


def detect_current_mode(gw: Any) -> tuple[str, str | None]:
    """Return the current RunPlan-derived session mode and native map."""

    projection = refresh_session_projection(gw)
    return projection["mode"], projection["active_map"]


def session_snapshot(gw: Any) -> dict[str, Any]:
    """Build the product session payload for HTTP, SSE, and bootstrap views."""
    projection = refresh_session_projection(gw)
    saved_active_map = projection["saved_active_map"]
    active_map = projection["active_map"]
    artifact_map = active_map or saved_active_map
    has_octomap = False
    has_pcd = False
    can_activate = False
    if saved_active_map:
        try:
            maps_response = mapd_query(gw, {"action": "list_maps"})
        except Exception:
            pass
        else:
            maps = maps_response.get("maps") if isinstance(maps_response, Mapping) else None
            if isinstance(maps, list) and maps_response.get("success") is True:
                target = next(
                    (
                        item
                        for item in maps
                        if isinstance(item, Mapping)
                        and str(item.get("name") or "") == saved_active_map
                    ),
                    None,
                )
                if target is not None and type(target.get("can_activate")) is bool:
                    can_activate = target["can_activate"]
    if artifact_map:
        has_pcd = (
            gw._map_bundle_from_mapd(
                artifact_map,
                "source_pointcloud",
            )
            is not None
        )
        has_octomap = (
            gw._map_bundle_from_mapd(
                artifact_map,
                "navigation_safety_3d",
            )
            is not None
        )
    icp = gw._icp_quality
    localization_status = gw._localization_status or {}
    slam_profile = gw._get_slam_profile()
    backend = str(localization_status.get("backend") or slam_profile or "stopped").lower()
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

    explorer_backend = gw._explorer_backend()
    explorer_available = explorer_backend != "none"
    explorer_detail = {} if explorer_available else gw._explorer_unavailable_detail()
    explorer_unavailable_reason = None if explorer_available else explorer_detail.get("reason")
    explorer_required_product = None if explorer_available else explorer_detail.get("required_product")
    safety = safety_summary(gw._navigation_state)
    safety_clear = safety_clear_for_motion(gw._navigation_state)
    exploration_blockers = _exploration_blockers(
        gw,
        explorer_available=explorer_available,
        safety_clear=safety_clear,
        localization_status=localization_status,
        pose_fresh=pose_fresh,
        algorithm_healthy=algorithm_healthy,
    )
    return {
        "mode": projection["mode"],
        "env": projection["env"],
        "product": projection["product"],
        "product_session_id": projection["product_session_id"],
        "slam_profile": slam_profile,
        "localization_backend": backend,
        "health_source": localization_status.get("health_source"),
        "active_map": active_map,
        "saved_active_map": saved_active_map,
        "map_has_pcd": has_pcd,
        "map_has_octomap": has_octomap,
        "can_activate": can_activate,
        "since": gw._session_since,
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
        "exploration_blockers": exploration_blockers,
        "safety_clear": safety_clear,
        "safety": safety,
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
