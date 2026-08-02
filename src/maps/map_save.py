"""Map-save adapter contract owned by the persistent maps domain.

Map lifecycle code should depend on this interface instead of importing a
specific transport or middleware implementation. Product entry points register
the default adapter in the runtime registry.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol


class MapSaveError(RuntimeError):
    """Base error for map-save adapter failures."""


class MapSaveUnavailable(MapSaveError):
    """Raised when the configured map-save runtime is unavailable."""


class MapSaveTimeout(MapSaveError):
    """Raised when a map-save operation times out."""


class MapSaveAdapter(Protocol):
    """Adapter contract for live-map and SLAM map-save operations."""

    def save_nav_map(
        self,
        pcd_path: str | Path,
        *,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        """Save the current live navigation map to ``pcd_path``."""

    def save_slam_map(
        self,
        file_path: str | Path,
        *,
        save_patches: bool = True,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        """Save the current SLAM map artifacts under ``file_path``."""


def default_map_save_adapter() -> MapSaveAdapter:
    """Return the configured default map-save adapter.

    Product/runtime layers must register the adapter first. The maps domain
    resolves it through the registry without importing a concrete transport.
    """

    import os

    from runtime.registry import get

    adapter_name = os.environ.get("LINGTU_MAP_SAVE_ADAPTER", "native_slam").strip() or "native_slam"
    try:
        adapter_cls = get("map_save_adapter", adapter_name)
    except KeyError as exc:
        raise MapSaveUnavailable(f"Map-save adapter '{adapter_name}' is not registered") from exc

    return adapter_cls()


def seed_default_map_save_adapter_plugins() -> None:
    """Seed optional map-save adapter plugins if the runtime provides them."""

    from runtime.plugin_seed import seed_registered_plugins

    try:
        seed_registered_plugins(
            groups=("map_save_adapter",),
            reload_loaded=False,
        )
    except ValueError as exc:
        if "map_save_adapter" not in str(exc):
            raise
        raise MapSaveUnavailable(
            "Map-save adapter plugin group is not available; inject a "
            "MapSaveAdapter or install a runtime package that provides one"
        ) from exc


def _result_dict(result: Any) -> dict[str, Any]:
    if not isinstance(result, dict) or not isinstance(result.get("success"), bool):
        raise MapSaveError("map-save adapter returned an invalid acknowledgement")
    return result


def save_nav_map_with_adapter(
    adapter: MapSaveAdapter | None,
    pcd_path: str | Path,
    *,
    timeout_sec: float = 30.0,
) -> dict[str, Any]:
    """Save a live navigation map through an injected or default adapter."""

    selected = adapter if adapter is not None else default_map_save_adapter()
    return _result_dict(selected.save_nav_map(pcd_path, timeout_sec=timeout_sec))


def save_slam_map_with_adapter(
    adapter: MapSaveAdapter | None,
    file_path: str | Path,
    *,
    save_patches: bool = True,
    timeout_sec: float = 30.0,
) -> dict[str, Any]:
    """Save a SLAM map through an injected or default native adapter."""

    selected = adapter if adapter is not None else default_map_save_adapter()
    result = selected.save_slam_map(
        file_path,
        save_patches=save_patches,
        timeout_sec=timeout_sec,
    )
    return _result_dict(result)
