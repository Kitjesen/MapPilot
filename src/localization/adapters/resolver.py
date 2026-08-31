"""Resolve the native SLAM status adapter used by Product Hosts."""

from __future__ import annotations

import importlib
from typing import Any

from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get


def localization_adapter_module(adapter_name: str | None = None) -> type[Any]:
    """Return the sole supported Product Host localization adapter."""
    adapter = str(adapter_name or "").strip().lower()
    if adapter != "cpp_slam_status":
        raise ImportError(
            f"Unsupported localization adapter {adapter_name!r}; use 'cpp_slam_status'"
        )

    key = ("localization_adapter", "cpp_slam_status")
    try:
        return get(*key)
    except KeyError:
        seed_registered_plugins(groups=("slam",), reload_loaded=False)
    try:
        return get(*key)
    except KeyError:
        module = importlib.import_module("localization.adapters.status")
        return module.CppSlamStatusAdapterModule
