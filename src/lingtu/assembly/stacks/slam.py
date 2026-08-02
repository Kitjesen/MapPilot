"""SLAM stack: native SlamModule or an explicit native transport adapter.

This stack only creates Module graph nodes. ProductControl owns field process
lifecycle through its internal SystemdRunner.
"""

from __future__ import annotations

import logging

from localization.adapters.resolver import localization_adapter_module
from runtime.blueprint import Blueprint
from runtime.plugin_resolution import optional_stack_module

logger = logging.getLogger(__name__)


def slam(
    profile: str = "fastlio2",
    enable_visual_backup: bool = True,
    localization_adapter: str | None = None,
    endpoint_contract: str | None = None,
) -> Blueprint:
    """Build the SLAM/localization stack.

    Native ``SlamModule`` is the managed path. Real-env Product Hosts use
    explicit native DDS/status adapters owned by the external SLAM endpoint.
    """

    bp = Blueprint()
    profile = normalize_slam_profile(profile)
    adapter = str(localization_adapter or "").strip()

    if (not profile or profile == "none") and not adapter:
        return bp
    if profile == "bridge" and not adapter:
        logger.warning("slam_profile='bridge' requires an explicit localization_adapter; skipping SLAM module")
        return bp
    module_alias = "SlamModule"
    has_slam_module = False
    try:
        if adapter:
            module_cls = localization_adapter_module(adapter)
            module_alias = slam_adapter_module_name(adapter)
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



def slam_adapter_module_name(adapter_name: str | None) -> str:
    """Return the graph alias for an explicit localization adapter."""

    return "SlamAdapterModule"


def normalize_slam_profile(profile: str) -> str:
    """Return the canonical SLAM profile name used by stack factories."""

    return str(profile or "").strip().lower()


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
