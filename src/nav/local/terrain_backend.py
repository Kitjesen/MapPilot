"""Backend adapter helpers for nav.terrain.

These helpers own optional nav_kernel discovery and C++ parameter assembly so
Terrain can keep only lifecycle/dataflow state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from nav.kernel import nav_kernel_build_hint, try_import_nav_kernel


@dataclass(frozen=True)
class NanobindTerrainBackend:
    core: Any
    params: Any
    runtime: Any | None = None
    effective_params: dict[str, Any] = field(default_factory=dict)


def build_terrain_params(
    nav_kernel: Any,
    *,
    config_getter: Callable[[], Any] | None = None,
) -> tuple[Any, dict[str, Any]]:
    """Build and populate nav_kernel TerrainParams from terrain_analysis config."""
    params = nav_kernel.TerrainParams()
    effective: dict[str, Any] = {}

    try:
        if config_getter is None:
            from runtime.config import get_config

            config_getter = get_config
        cfg = config_getter()
        ta = cfg.raw.get("terrain_analysis", {})
        if ta:
            for attr in [
                "scan_voxel_size",
                "decay_time",
                "no_decay_dis",
                "obstacle_height_thre",
                "vehicle_height",
                "min_rel_z",
                "max_rel_z",
            ]:
                if attr in ta:
                    setattr(params, attr, ta[attr])
                    effective[attr] = getattr(params, attr)
    except ImportError:
        pass

    return params, effective


def create_nanobind_terrain_backend(
    *,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
    build_hint_provider: Callable[[], str] = nav_kernel_build_hint,
    config_getter: Callable[[], Any] | None = None,
) -> NanobindTerrainBackend:
    """Create the nanobind TerrainAnalysisCore backend or raise old errors."""
    nav_kernel = nav_kernel_importer(("TerrainParams", "TerrainAnalysisCore"))
    if nav_kernel is None:
        raise RuntimeError(
            f"Terrain [nanobind]: compatible LingTu native navigation kernel not found. "
            f"Build C++ backend or explicitly choose backend='simple' for passthrough testing.\n"
            f"  To build: {build_hint_provider()}"
        )
    try:
        params, effective = build_terrain_params(
            nav_kernel,
            config_getter=config_getter,
        )
        return NanobindTerrainBackend(
            core=nav_kernel.TerrainAnalysisCore(params),
            params=params,
            runtime=nav_kernel,
            effective_params=effective,
        )
    except RuntimeError:
        raise
    except Exception as e:
        raise RuntimeError(
            f"Terrain [nanobind]: LingTu native navigation kernel init failed: {e}. "
            f"Install lingtu_nav_kernel.so or explicitly choose backend='simple'."
        ) from e
