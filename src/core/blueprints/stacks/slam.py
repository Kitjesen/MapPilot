"""SLAM stack: bridge external/native SLAM data into Module ports.

Optionally includes DepthVisualOdomModule for degeneracy-resilient fusion.
When SLAM detects corridor/open-field degeneracy (SEVERE/CRITICAL),
visual odometry from the depth camera selectively fuses into degenerate DOFs.

External service startup belongs to the system stack
(`core.blueprints.stacks.system.external_services`) and runs through
ExternalServiceManagerModule during runtime startup. This stack factory must
remain side-effect free so non-ROS and non-systemd profiles can build the same
module graph.

GNSS fusion: when GnssModule is present in the system (from full_stack.py's
_gnss_bp), its ``gnss_odom`` port auto-wires into SlamBridgeModule.gnss_odom
via Blueprint._do_auto_wire (matches by port_name + msg_type). No explicit
wire() call needed — do not add one here unless the port names diverge.
"""

from __future__ import annotations

import importlib
import logging
from typing import Any

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import optional_stack_module
from core.plugin_seed import seed_registered_plugins
from core.registry import get

logger = logging.getLogger(__name__)


def slam(
    profile: str = "fastlio2",
    enable_visual_backup: bool = True,
    manage_services: bool = False,
    localization_adapter: str | None = None,
    endpoint_contract: str | None = None,
) -> Blueprint:
    """SLAM / localization stack.

    The external SLAM process may be ROS2, native, or disabled depending on the
    runtime endpoint. This factory only creates Module graph nodes.

    Args:
        profile: SLAM backend profile
        enable_visual_backup: Add DepthVisualOdomModule for degeneracy fallback
        manage_services: Deprecated compatibility flag. Service orchestration
            is now handled by the system stack, not this factory.

    Profiles:
      "fastlio2"  → start slam + slam_pgo services (mapping mode)
      "localizer" → start slam + localizer services (navigation mode)
      "super_lio" → start Super-LIO as an external experimental LIO backend
      "bridge"    → subscribe only, assume SLAM already running
      "none"/""   → empty (stub/dev mode)

    GNSS fusion config is read from robot_config.yaml gnss.* section and
    passed to SlamBridgeModule as kwargs:
      gnss.antenna_offset.{x,y,z}  → gnss_antenna_offset (lever-arm)
      gnss.fusion.enabled          → gnss_fusion
      gnss.fusion.alpha_{healthy,degraded}  → gnss_alpha_{healthy,degraded}
      gnss.fusion.rtk_float_scale  → gnss_rtk_float_scale
      gnss.fusion.max_{age_s,std_m} → gnss_max_{age_s,std_m}
      gnss.fusion.residual_{warn_m,warn_duration_s,warn_ratio}
                                    → gnss_residual_*
    """
    bp = Blueprint()
    profile = normalize_slam_profile(profile)

    if not profile or profile == "none":
        return bp
    if manage_services:
        logger.debug(
            "slam(manage_services=True) is ignored; external service startup "
            "is handled by core.blueprints.stacks.system.external_services"
        )

    # Resolve the localization role first; ROS2/DDS is only the current backend.
    try:
        LocalizationAdapterModule = _localization_adapter_module(localization_adapter)
        bridge_kwargs = _read_gnss_fusion_kwargs()
        bridge_kwargs["backend_profile"] = profile
        if endpoint_contract:
            bridge_kwargs["endpoint_contract"] = endpoint_contract
        # Keep the runtime alias stable until all downstream wires migrate.
        bp.add(LocalizationAdapterModule, alias="SlamBridgeModule", **bridge_kwargs)
    except ImportError as e:
        logger.warning("Localization adapter module not available: %s", e)

    # Depth camera visual odometry for degeneracy fallback
    if enable_visual_backup:
        DepthVisualOdomModule = optional_stack_module(
            "visual_odom",
            "depth",
            seed_group="slam",
            fallback="slam.depth_visual_odom_module.DepthVisualOdomModule",
        )
        if DepthVisualOdomModule is not None:
            bp.add(DepthVisualOdomModule, alias="DepthVisualOdomModule")
            logger.info("SLAM stack: DepthVisualOdomModule enabled for degeneracy backup")
        else:
            logger.debug("DepthVisualOdomModule not available")

    return bp


def slam_module_name(profile: str) -> str:
    """Return the Module class name for SLAM data wiring.

    Always SlamBridgeModule — it has the Out ports.
    """
    if not profile or profile == "none":
        return ""
    return "SlamBridgeModule"


def _localization_adapter_module(adapter_name: str | None = None) -> type[Any]:
    """Resolve the localization adapter while preserving legacy plugins."""
    adapter = str(adapter_name or "").strip().lower()
    if adapter in {"lcm", "lcm_endpoint", "thunder_field_lcm_v1"}:
        preferred = (("localization_adapter", "lcm_endpoint"),)
        fallback_module = "compat.lcm.localization_adapter"
        fallback_class = "LCMLocalizationAdapterModule"
        seed_group = "slam_lcm"
    elif adapter and adapter not in {"auto", "default", "ros2", "ros2_slam_bridge"}:
        preferred = (("localization_adapter", adapter),)
        fallback_module = ""
        fallback_class = ""
        seed_group = "slam"
    else:
        preferred = (
            ("localization_adapter", "ros2_slam_bridge"),
            ("slam_bridge", "default"),
        )
        fallback_module = "compat.ros2.slam_bridge"
        fallback_class = "SlamBridgeModule"
        seed_group = "slam_ros2"

    for category, name in preferred:
        try:
            return get(category, name)
        except KeyError:
            pass

    seed_registered_plugins(groups=(seed_group,), reload_loaded=False)
    for category, name in preferred:
        try:
            return get(category, name)
        except KeyError:
            pass

    if fallback_module and fallback_class:
        module = importlib.import_module(fallback_module)
        return getattr(module, fallback_class)
    raise ImportError(f"Localization adapter '{adapter_name}' not available")


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
    """Load SlamBridgeModule GNSS fusion kwargs from the typed GnssConfig.

    Returns an empty dict on any failure so SlamBridgeModule falls back to its
    own hardcoded defaults — never blocks SLAM startup because of config quirks.
    """
    try:
        from core.config import get_config
        gnss = get_config().gnss
    except Exception as e:
        logger.debug("GNSS fusion config unavailable: %s", e)
        return {}

    ant = gnss.antenna_offset
    fusion = gnss.fusion
    return {
        "gnss_antenna_offset": (float(ant.x), float(ant.y), float(ant.z)),
        "gnss_fusion":                   bool(fusion.enabled),
        "gnss_alpha_healthy":            float(fusion.alpha_healthy),
        "gnss_alpha_degraded":           float(fusion.alpha_degraded),
        "gnss_rtk_float_scale":          float(fusion.rtk_float_scale),
        "gnss_max_age_s":                float(fusion.max_age_s),
        "gnss_max_std_m":                float(fusion.max_std_m),
        "gnss_residual_warn_m":          float(fusion.residual_warn_m),
        "gnss_residual_warn_duration_s": float(fusion.residual_warn_duration_s),
        "gnss_residual_warn_ratio":      float(fusion.residual_warn_ratio),
    }
