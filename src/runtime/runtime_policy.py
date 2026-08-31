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
}


def normalize_slam_profile(profile: Any) -> str:
    return str(profile or "").strip().lower()


def is_supported_slam_profile(profile: Any) -> bool:
    normalized = normalize_slam_profile(profile)
    return normalized in SUPPORTED_SLAM_PROFILES


def default_slam_profile_for_mode(mode: str) -> str:
    return "none" if str(mode or "").strip().lower() == "none" else "native_dds"


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
            "map_save_source": "native_slam_dds_control",
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "restart_native_dds_slam",
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

    backend = normalize_slam_profile(profile) or "none"
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
    return {
        "backend": backend,
        "health_source": "unknown",
        **backend_capability_defaults(backend),
        "recovery_action": "none",
    }
