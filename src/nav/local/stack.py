"""add_autonomy_stack() -convenience wrapper for the 3 autonomy sub-modules.

The autonomy stack is 3 independent Modules, each independently pluggable via
Registry:
  - Terrain:       traversability analysis (In: odometry, map_cloud -Out: terrain_map)
  - LocalPlanner:  obstacle avoidance (In: terrain_map, waypoint -Out: local_path)
  - PathFollower:  path tracking (In: local_path -Out: cmd_vel)

`add_autonomy_stack()` is a convenience function that adds all 3 at once with a
single blueprint call; there is no wrapper Module class.

Usage::

    # Option A: add individually (more control)
    bp.add(Terrain, backend="nanobind")
    bp.add(LocalPlanner, backend="nanobind")
    bp.add(PathFollower, backend="nav_kernel")

    # Option B: convenience wrapper (same ROS-free defaults)
    from nav.local import add_autonomy_stack
    add_autonomy_stack(bp)
"""

from __future__ import annotations

import logging
import sys
from importlib import import_module, reload
from typing import Any

from nav.local.contracts import TERRAIN_BACKENDS
from runtime.blueprint import Blueprint
from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get

logger = logging.getLogger(__name__)

# The three class names are lazy-forward references resolved by __getattr__
# below -each name is exported via lazy import from its real module.
# noqa comments below: ruff F822 sees these as undefined but they are
# resolved by __getattr__ at runtime.
__all__ = [  # noqa: F822
    "nav.local_planner",
    "nav.path_follower",
    "nav.terrain",
    "add_autonomy_stack",
]


_EXPORTS = {
    "nav.terrain": "nav.local.terrain",
    "nav.local_planner": "nav.services.plan.local_planner.service",
    "nav.path_follower": "nav.local.path_follower",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)


def _module_for_backend(category: str, backend: str) -> type:
    try:
        return get(category, backend)
    except KeyError:
        seed_registered_plugins(groups=("autonomy",), reload_loaded=True)
        try:
            return get(category, backend)
        except KeyError:
            _import_builtin_category(category)
            return get(category, backend)


def _import_builtin_category(category: str) -> None:
    module_names = {
        "terrain": ("nav.local.terrain",),
        "local_planner": ("nav.services.plan.local_planner.service",),
        "path_follower": ("nav.local.path_follower",),
    }.get(category, ())
    for module_name in module_names:
        if module_name in sys.modules:
            reload(sys.modules[module_name])
        else:
            import_module(module_name)


def add_autonomy_stack(
    bp: Blueprint,
    backend: str = "nanobind",
    terrain_backend: str | None = None,
    path_follower_backend: str = "nav_kernel",
    terrain_config: dict[str, Any] | None = None,
    local_planner_config: dict[str, Any] | None = None,
    **kw,
) -> Blueprint:
    """Add the full autonomy stack (terrain + local_planner + path_follower) to a Blueprint.

    Args:
        bp: Blueprint to add modules to.
        backend: Backend for nav.local_planner.
        terrain_backend: Optional independent Terrain backend. Defaults to
            the local-planner backend only when that name is also a terrain
            backend, otherwise "nanobind".
        path_follower_backend: Backend for nav.path_follower.

    Returns:
        The same Blueprint (for chaining).
    """
    terrain_key = (
        terrain_backend
        if terrain_backend is not None
        else backend if backend in TERRAIN_BACKENDS else "nanobind"
    )
    TerrainCls = _module_for_backend("terrain", terrain_key)
    LocalPlannerCls = _module_for_backend("local_planner", backend)
    PathFollowerCls = _module_for_backend("path_follower", path_follower_backend)

    bp.add(
        TerrainCls,
        alias="nav.terrain",
        backend=terrain_key,
        **(terrain_config or {}),
    )
    bp.add(
        LocalPlannerCls,
        alias="nav.local_planner",
        backend=backend,
        **(local_planner_config or {}),
    )
    bp.add(
        PathFollowerCls,
        alias="nav.path_follower",
        backend=path_follower_backend,
        **kw,
    )
    return bp
