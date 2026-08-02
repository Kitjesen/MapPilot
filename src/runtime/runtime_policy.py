"""Pure runtime policy for SLAM backend capabilities.

This module intentionally has no ROS, systemd, Gateway, or Module imports.
It is the shared source of truth for canonical profile normalization and
backend capability defaults used by Gateway and SLAM bridge code.
ProductControl owns field service transitions; this module does not describe
process operations.
"""

from __future__ import annotations

from typing import Any

SUPPORTED_SLAM_PROFILES = {
    "none",
    "native_dds",
    "fastlio2",
    "genz",
    "localizer",
}
SWITCHABLE_SLAM_PROFILES = SUPPORTED_SLAM_PROFILES | {"stop"}


def normalize_slam_profile(profile: Any) -> str:
    return str(profile or "").strip().lower()


def is_supported_slam_profile(profile: Any, *, allow_stop: bool = False) -> bool:
    normalized = normalize_slam_profile(profile)
    supported = SWITCHABLE_SLAM_PROFILES if allow_stop else SUPPORTED_SLAM_PROFILES
    return normalized in supported


def default_slam_profile_for_mode(mode: str) -> str:
    return "native_dds"


def backend_capability_defaults(backend_name: Any) -> dict[str, Any]:
    """Gateway-facing backend capability defaults.

    Keep this conservative for unknown profiles because Gateway status surfaces
    can be fed by partial diagnostics during startup.
    """

    backend = normalize_slam_profile(backend_name)
    if backend in {"", "none"}:
        return {
            "map_save_supported": False,
            "map_save_source": None,
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": False,
            "recovery_method": "external_or_disabled_slam",
        }
    if backend == "native_dds":
        return {
            "map_save_supported": True,
            "map_save_source": "native_dds_slam_runtime",
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "restart_native_dds_slam",
        }
    if backend == "localizer":
        return {
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "relocalize_service",
        }
    if backend == "genz":
        return {
            "map_save_supported": False,
            "map_save_source": None,
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_genz_icp",
        }
    if backend == "fastlio2":
        return {
            "map_save_supported": True,
            "map_save_source": "slam_save_maps",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_slam",
        }
    return {
        "map_save_supported": False,
        "map_save_source": None,
        "relocalization_supported": False,
        "saved_map_relocalization_supported": False,
        "restart_recovery_supported": False,
        "recovery_method": "unknown_backend",
    }


def slam_backend_contract(profile: Any) -> dict[str, Any]:
    """SLAM-bridge-facing backend contract including health/recovery action."""

    backend = normalize_slam_profile(profile) or "bridge"
    if backend == "none":
        return {
            "backend": "none",
            "health_source": "external_or_disabled_slam",
            **backend_capability_defaults("none"),
            "recovery_action": "none",
        }
    if backend == "native_dds":
        return {
            "backend": "native_dds",
            "health_source": "slam_runtime",
            **backend_capability_defaults("native_dds"),
            "recovery_action": "restart_native_dds_slam",
        }
    if backend == "localizer":
        return {
            "backend": "localizer",
            "health_source": "localizer_health_topic",
            **backend_capability_defaults("localizer"),
            "recovery_action": "relocalize_service",
        }
    if backend == "genz":
        return {
            "backend": "genz",
            "health_source": "odom_map_cloud",
            **backend_capability_defaults("genz"),
            "recovery_action": "restart_genz_icp",
        }
    if backend == "fastlio2":
        return {
            "backend": "fastlio2",
            "health_source": "odom_map_cloud",
            **backend_capability_defaults("fastlio2"),
            "recovery_action": "restart_slam",
        }
    return {
        "backend": backend,
        "health_source": "odom_map_cloud",
        "map_save_supported": True,
        "map_save_source": "slam_services_or_live_cloud",
        "relocalization_supported": False,
        "saved_map_relocalization_supported": False,
        "restart_recovery_supported": True,
        "recovery_method": "restart_backend",
        "recovery_action": "restart_backend",
    }
