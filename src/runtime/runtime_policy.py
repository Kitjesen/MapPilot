"""Pure runtime policy for SLAM backend capabilities and service plans.

This module intentionally has no ROS, systemd, Gateway, or Module imports.
It is the shared source of truth for profile aliases, backend capability
defaults, and the service transition orders used by Gateway and SLAM bridge
code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


SLAM_PROFILE_ALIASES: dict[str, str] = {
    "super-lio": "super_lio",
    "superlio": "super_lio",
    "super_lio_reloc": "super_lio_relocation",
    "super-lio-reloc": "super_lio_relocation",
    "superlio-reloc": "super_lio_relocation",
    "super_lio_relocation": "super_lio_relocation",
    "super-lio-relocation": "super_lio_relocation",
    "superlio-relocation": "super_lio_relocation",
    "relocation": "super_lio_relocation",
}

SUPPORTED_SLAM_PROFILES = {
    "none",
    "fastlio2",
    "localizer",
    "super_lio",
    "super_lio_relocation",
}
SWITCHABLE_SLAM_PROFILES = SUPPORTED_SLAM_PROFILES | {"stop"}


@dataclass(frozen=True)
class ServiceTransitionPlan:
    """Service-manager operations for a runtime transition.

    The fields are ordered tuples because service stop/start ordering is part of
    the robot-side behavior contract.
    """

    stop: tuple[str, ...]
    ensure: tuple[str, ...] = ()
    wait_ready: tuple[str, ...] = ()
    clear_live_map: bool = False


def normalize_slam_profile(profile: Any) -> str:
    raw = str(profile or "").strip().lower()
    return SLAM_PROFILE_ALIASES.get(raw, raw)


def is_supported_slam_profile(profile: Any, *, allow_stop: bool = False) -> bool:
    normalized = normalize_slam_profile(profile)
    supported = SWITCHABLE_SLAM_PROFILES if allow_stop else SUPPORTED_SLAM_PROFILES
    return normalized in supported


def default_slam_profile_for_mode(mode: str) -> str:
    return "localizer" if str(mode).strip().lower() == "navigating" else "fastlio2"


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
    if backend == "super_lio":
        return {
            "map_save_supported": True,
            "map_save_source": "live_map_cloud_snapshot",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
        }
    if backend == "super_lio_relocation":
        return {
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
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
    if backend in {"fastlio2", "slam"}:
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
        "relocalization_supported": True,
        "saved_map_relocalization_supported": True,
        "restart_recovery_supported": False,
        "recovery_method": "relocalize_service",
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
    if backend == "super_lio":
        return {
            "backend": "super_lio",
            "health_source": "odom_map_cloud",
            **backend_capability_defaults("super_lio"),
            "recovery_action": "restart_super_lio",
        }
    if backend == "super_lio_relocation":
        return {
            "backend": "super_lio_relocation",
            "health_source": "odom_map_cloud",
            **backend_capability_defaults("super_lio_relocation"),
            "recovery_action": "restart_super_lio_relocation",
        }
    if backend == "localizer":
        return {
            "backend": "localizer",
            "health_source": "localizer_health_topic",
            **backend_capability_defaults("localizer"),
            "recovery_action": "relocalize_service",
        }
    if backend in {"fastlio2", "slam"}:
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


def slam_switch_plan(profile: Any) -> ServiceTransitionPlan:
    profile = normalize_slam_profile(profile)
    if profile == "none":
        return ServiceTransitionPlan(
            stop=(
                "super_lio_relocation",
                "super_lio",
                "slam_pgo",
                "localizer",
                "slam",
            ),
        )
    if profile == "fastlio2":
        return ServiceTransitionPlan(
            stop=("localizer", "super_lio", "super_lio_relocation"),
            ensure=("slam", "slam_pgo"),
            wait_ready=("slam", "slam_pgo"),
        )
    if profile == "localizer":
        return ServiceTransitionPlan(
            stop=("slam_pgo", "super_lio", "super_lio_relocation"),
            ensure=("slam", "localizer"),
            wait_ready=("slam", "localizer"),
        )
    if profile == "super_lio":
        return ServiceTransitionPlan(
            stop=("slam", "slam_pgo", "localizer", "super_lio_relocation"),
            ensure=("lidar", "super_lio"),
            wait_ready=("lidar", "super_lio"),
        )
    if profile == "super_lio_relocation":
        return ServiceTransitionPlan(
            stop=("slam", "slam_pgo", "localizer", "super_lio"),
            ensure=("lidar", "super_lio_relocation"),
            wait_ready=("lidar", "super_lio_relocation"),
        )
    if profile == "stop":
        return ServiceTransitionPlan(
            stop=(
                "super_lio_relocation",
                "super_lio",
                "slam_pgo",
                "localizer",
                "slam",
            ),
        )
    raise ValueError(f"Unknown SLAM profile: {profile!r}")


def session_transition_plan(mode: str, backend: Any) -> ServiceTransitionPlan:
    """Preserve the existing Gateway session start service behavior."""

    mode = str(mode or "").strip().lower()
    backend = normalize_slam_profile(backend)
    if backend in {"", "none"}:
        return ServiceTransitionPlan(
            stop=(),
            clear_live_map=mode in {"mapping", "exploring"},
        )
    if backend == "super_lio":
        return ServiceTransitionPlan(
            stop=("slam", "slam_pgo", "localizer", "super_lio_relocation"),
            ensure=("lidar", "super_lio"),
            wait_ready=("lidar", "super_lio"),
            clear_live_map=mode in {"mapping", "exploring"},
        )
    if backend == "super_lio_relocation":
        return ServiceTransitionPlan(
            stop=("slam", "slam_pgo", "localizer", "super_lio"),
            ensure=("lidar", "super_lio_relocation"),
            wait_ready=("lidar", "super_lio_relocation"),
        )
    if mode in {"mapping", "exploring"}:
        return ServiceTransitionPlan(
            stop=("localizer", "super_lio", "super_lio_relocation"),
            ensure=("slam", "slam_pgo"),
            wait_ready=("slam", "slam_pgo"),
            clear_live_map=True,
        )
    return ServiceTransitionPlan(
        stop=("slam_pgo", "super_lio", "super_lio_relocation"),
        ensure=("slam", "localizer"),
        wait_ready=("slam", "localizer"),
    )
