"""SLAM runtime profile helpers for GatewayModule."""

from __future__ import annotations

import logging
import time
from typing import Any

logger = logging.getLogger(__name__)


def slam_profile_from_status(status: dict | None) -> str:
    """Return a live backend from localization_status when it is usable."""
    if not isinstance(status, dict):
        return ""
    profile = str(status.get("backend") or "").strip().lower()
    profile = {
        "genz-icp": "genz",
        "genz_icp": "genz",
        "super-lio": "super_lio",
        "superlio": "super_lio",
        "super_lio_reloc": "super_lio_relocation",
        "super-lio-reloc": "super_lio_relocation",
        "point-lio": "pointlio",
        "point_lio": "pointlio",
    }.get(profile, profile)
    if profile in {"", "none", "unknown", "stopped", "stop", "disabled"}:
        return ""
    state = str(status.get("reported_state") or status.get("state") or "").strip().upper()
    if state in {
        "STOPPED",
        "DISABLED",
        "UNINIT",
        "UNINITIALIZED",
        "LOST",
        "DIVERGED",
        "FAILED",
        "ERROR",
    }:
        return ""
    if str(status.get("health_source") or "").strip().lower() == "slam_runtime":
        return "native_dds"
    if profile in {
        "cpp_dds_slam",
        "lingtu_slam_dds",
        "lingtu-slam-dds",
        "native_slam",
        "native_dds",
    }:
        return "native_dds"
    return profile


def current_slam_profile(gw: Any) -> str:
    """Return current SLAM profile, with service-manager lookup cached by Gateway."""
    live_profile = slam_profile_from_status(gw._localization_status)
    if live_profile:
        gw._session_runtime.remember_slam_profile(live_profile)
        return live_profile
    if not gw._manage_session_services:
        profile = str(gw._session_slam_profile or gw._cached_slam_profile or "stopped").lower()
        gw._session_runtime.remember_slam_profile(profile)
        return profile
    now = time.time()
    if now - gw._slam_profile_ts < 5.0:
        return gw._cached_slam_profile
    gw._slam_profile_ts = now
    try:
        from lingtu.control import ProductControl

        services, _ = ProductControl().status(
            (
                "super_lio_relocation",
                "super_lio",
                "genz_icp",
                "slam_pgo",
                "localizer",
                "slam",
            )
        )
        if services.get("super_lio_relocation") in ("running", "active"):
            profile = "super_lio_relocation"
        elif services.get("super_lio") in ("running", "active"):
            profile = "super_lio"
        elif services.get("genz_icp") in ("running", "active"):
            profile = "genz"
        elif services.get("slam_pgo") in ("running", "active"):
            profile = "fastlio2"
        elif services.get("localizer") in ("running", "active"):
            profile = "localizer"
        elif services.get("slam") in ("running", "active"):
            profile = "native_dds"
        elif gw._session_uses_external_slam_none():
            profile = "none"
        else:
            profile = "stopped"
        gw._session_runtime.remember_slam_profile(profile, ts=gw._slam_profile_ts)
    except Exception as e:
        logger.debug("_get_slam_profile: service_manager lookup failed: %s", e)
    return gw._cached_slam_profile


def cached_slam_hz(gw: Any) -> float:
    """Estimate SLAM Hz from odometry already flowing through Gateway."""
    now = time.time()
    with gw._state_lock:
        timestamps = list(gw._odom_timestamps)
    if len(timestamps) < 2:
        return 0.0
    if now - timestamps[-1] > 2.0:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    if span <= 0.0:
        return 0.0
    return round((len(timestamps) - 1) / span, 1)
