"""SLAM stack: native SlamModule by default, explicit compatibility adapters.

This stack only creates Module graph nodes. Starting external services belongs
to runtime.blueprints.stacks.system.external_services.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.blueprint import Blueprint
from runtime.adapters.mapping_slam import localization_adapter_module
from runtime.blueprints.stacks._registry import optional_stack_module

logger = logging.getLogger(__name__)


def slam(
    profile: str = "fastlio2",
    enable_visual_backup: bool = True,
    manage_services: bool = False,
    localization_adapter: str | None = None,
    endpoint_contract: str | None = None,
) -> Blueprint:
    """Build the SLAM/localization stack.

    Native ``SlamModule`` is the product path. ROS2/LCM bridges are only used
    when ``localization_adapter`` explicitly selects them.
    """

    bp = Blueprint()
    profile = normalize_slam_profile(profile)

    if not profile or profile == "none":
        return bp
    if profile == "bridge" and not _uses_compat_adapter(localization_adapter):
        logger.warning(
            "slam_profile='bridge' requires an explicit localization_adapter; "
            "skipping SLAM module"
        )
        return bp
    if manage_services:
        logger.debug(
            "slam(manage_services=True) is ignored; external service startup "
            "is handled by runtime.blueprints.stacks.system.external_services"
        )

    module_alias = "SlamModule"
    has_slam_module = False
    try:
        if _uses_compat_adapter(localization_adapter):
            module_cls = _localization_adapter_module(localization_adapter)
            module_alias = slam_adapter_module_name(localization_adapter)
        else:
            from localization.slam.module import SlamModule as module_cls

        kwargs = _read_gnss_fusion_kwargs()
        kwargs["backend_profile"] = profile
        if endpoint_contract:
            kwargs["endpoint_contract"] = endpoint_contract
        bp.add(module_cls, alias=module_alias, **kwargs)
        has_slam_module = True
    except ImportError as exc:
        logger.warning("SLAM module not available: %s", exc)

    if enable_visual_backup and has_slam_module:
        depth_visual_odom = optional_stack_module(
            "visual_odom",
            "depth",
            seed_group="slam",
            fallback="localization.depth_visual_odom_module.DepthVisualOdomModule",
        )
        if depth_visual_odom is not None:
            bp.add(depth_visual_odom, alias="DepthVisualOdomModule")
            logger.info("SLAM stack: DepthVisualOdomModule enabled")
        else:
            logger.debug("DepthVisualOdomModule not available")

    return bp


def slam_module_name(profile: str) -> str:
    """Return the default native SLAM module name for graph wiring."""

    if not profile or profile == "none":
        return ""
    return "SlamModule"


def _localization_adapter_module(adapter_name: str | None = None) -> type[Any]:
    """Resolve an explicit compatibility localization adapter."""

    return localization_adapter_module(adapter_name)


def _uses_compat_adapter(adapter_name: str | None) -> bool:
    adapter = str(adapter_name or "").strip().lower()
    return bool(adapter and adapter not in {"native", "native_slam", "slam"})


def slam_adapter_module_name(adapter_name: str | None) -> str:
    """Return the graph alias for an explicit localization adapter.

    Only the ROS2 adapter is a bridge. DDS endpoints are native transport
    adapters and should not appear in product graphs as ``SlamBridgeModule``.
    """

    adapter = str(adapter_name or "").strip().lower()
    if adapter in {"ros2", "ros2_slam_bridge"}:
        return "SlamBridgeModule"
    return "SlamAdapterModule"


def normalize_slam_profile(profile: str) -> str:
    """Return the canonical SLAM profile name used by stack factories."""

    raw = str(profile or "").strip().lower()
    aliases = {
        "super-lio": "super_lio",
        "superlio": "super_lio",
        "super_lio_reloc": "super_lio_relocation",
        "super-lio-reloc": "super_lio_relocation",
        "superlio-reloc": "super_lio_relocation",
        "super-lio-relocation": "super_lio_relocation",
        "superlio-relocation": "super_lio_relocation",
        "relocation": "super_lio_relocation",
    }
    return aliases.get(raw, raw)


def _normalize_slam_profile(profile: str) -> str:
    """Backward-compatible private alias for existing imports/tests."""

    return normalize_slam_profile(profile)


def _read_gnss_fusion_kwargs() -> dict:
    """Load GNSS fusion kwargs from the typed runtime config.

    Returns an empty dict on any failure so SLAM startup is not blocked by
    config quirks.
    """

    try:
        from runtime.config import get_config

        gnss = get_config().gnss
    except Exception as exc:
        logger.debug("GNSS fusion config unavailable: %s", exc)
        return {}

    ant = gnss.antenna_offset
    fusion = gnss.fusion
    return {
        "gnss_antenna_offset": (float(ant.x), float(ant.y), float(ant.z)),
        "gnss_fusion": bool(fusion.enabled),
        "gnss_alpha_healthy": float(fusion.alpha_healthy),
        "gnss_alpha_degraded": float(fusion.alpha_degraded),
        "gnss_rtk_float_scale": float(fusion.rtk_float_scale),
        "gnss_max_age_s": float(fusion.max_age_s),
        "gnss_max_std_m": float(fusion.max_std_m),
        "gnss_residual_warn_m": float(fusion.residual_warn_m),
        "gnss_residual_warn_duration_s": float(fusion.residual_warn_duration_s),
        "gnss_residual_warn_ratio": float(fusion.residual_warn_ratio),
    }
