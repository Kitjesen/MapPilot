"""SLAM runtime profile helpers for GatewayModule."""

from __future__ import annotations

import time
from typing import Any


def slam_profile_from_status(status: dict | None) -> str:
    """Return a live backend from localization_status when it is usable."""
    if not isinstance(status, dict):
        return ""
    profile = str(status.get("backend") or "").strip().lower()
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
        "fastlio2",
        "pointlio",
        "genz",
        "localizer",
        "native_dds",
    }:
        return profile
    return ""


def current_slam_profile(gw: Any) -> str:
    """Return the current SLAM profile from native telemetry or session cache."""
    live_profile = slam_profile_from_status(gw._localization_status)
    if live_profile:
        gw._session_runtime.remember_slam_profile(live_profile)
        return live_profile
    profile = str(gw._session_slam_profile or gw._cached_slam_profile or "").strip().lower()
    if not profile:
        profile = "none" if gw._session_uses_external_slam_none() else "stopped"
    gw._session_runtime.remember_slam_profile(profile)
    return profile


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
