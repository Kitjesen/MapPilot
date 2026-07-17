"""Localization adapter resolution for Blueprint composition."""

from __future__ import annotations

import importlib
from typing import Any

from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get


def localization_adapter_module(adapter_name: str | None = None) -> type[Any]:
    """Resolve an explicit localization adapter.

    Product/runtime configs must name the adapter they want.  An empty adapter
    fails closed instead of guessing a transport.
    """
    adapter = str(adapter_name or "").strip().lower()
    if not adapter or adapter in {"auto", "default"}:
        raise ImportError("Localization adapter must be explicit; choose 'cpp_slam_status' or 'dds_endpoint'")
    if adapter in {"ros2", "ros2_slam_bridge"} or adapter.startswith("ros2_"):
        raise ImportError("ROS2 localization adapters were removed; use 'cpp_slam_status' or 'dds_endpoint'")
    if adapter in {"cpp_slam_status", "native_slam_status"}:
        preferred = (("localization_adapter", "cpp_slam_status"),)
        fallback_module = "runtime.adapters.native.localization_adapter"
        fallback_class = "CppSlamStatusAdapterModule"
        seed_group = "slam"
    elif adapter in {"dds", "dds_endpoint"}:
        preferred = (("localization_adapter", "dds_endpoint"),)
        fallback_module = "runtime.adapters.dds.localization_adapter"
        fallback_class = "DDSLocalizationAdapterModule"
        seed_group = "slam"
    else:
        preferred = (("localization_adapter", adapter),)
        fallback_module = ""
        fallback_class = ""
        seed_group = "slam"

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
