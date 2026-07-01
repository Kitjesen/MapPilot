"""Adapter resolution helpers for mapping and SLAM stack boundaries."""

from __future__ import annotations

import importlib
from typing import Any

from runtime.blueprints.stacks._registry import optional_stack_module
from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get


def map_output_adapter_module(
    *,
    enable_ros2: bool = False,
    enable_dds: bool = False,
) -> type[Any] | None:
    """Resolve the optional map output adapter.

    The only built-in map output adapter is the ROS2 compatibility bridge, so
    callers must opt in explicitly.  Plain endpoint enablement should not
    resurrect ROS2 through this helper.
    """
    if enable_dds:
        return optional_stack_module(
            "map",
            "dds_map_output",
            seed_group="map_dds",
            fallback="runtime.adapters.dds.map_output.DDSMapOutModule",
        )
    if not enable_ros2:
        return None
    return optional_stack_module(
        "map",
        "ros2_map_output",
        seed_group="map_ros2",
        fallback="nav.adapters.ros2.nav.map_out.ROS2MapOutModule",
    )


def localization_adapter_module(adapter_name: str | None = None) -> type[Any]:
    """Resolve an explicit localization adapter.

    Product/runtime configs must name the adapter they want.  An empty adapter
    fails closed instead of resurrecting the legacy ROS2 bridge; compatibility
    callers can still request ``ros2`` or ``ros2_slam_bridge`` explicitly.
    """
    adapter = str(adapter_name or "").strip().lower()
    if not adapter or adapter in {"auto", "default"}:
        raise ImportError(
            "Localization adapter must be explicit; choose 'dds_endpoint', "
            "'lcm_endpoint', or explicit 'ros2_slam_bridge' compatibility"
        )
    if adapter in {"dds", "dds_endpoint"}:
        preferred = (("localization_adapter", "dds_endpoint"),)
        fallback_module = "runtime.adapters.dds.localization_adapter"
        fallback_class = "DDSLocalizationAdapterModule"
        seed_group = "slam"
    elif adapter in {"lcm", "lcm_endpoint", "thunder_field_lcm_v1"}:
        preferred = (("localization_adapter", "lcm_endpoint"),)
        fallback_module = "runtime.adapters.lcm.localization_adapter"
        fallback_class = "LCMLocalizationAdapterModule"
        seed_group = "slam_lcm"
    elif adapter not in {"ros2", "ros2_slam_bridge"}:
        preferred = (("localization_adapter", adapter),)
        fallback_module = ""
        fallback_class = ""
        seed_group = "slam"
    else:
        preferred = (
            ("localization_adapter", "ros2_slam_bridge"),
            ("slam_bridge", "default"),
        )
        fallback_module = "localization.adapters.ros2.slam_bridge"
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
