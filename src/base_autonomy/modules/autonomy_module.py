"""AutonomyModule — convenience wrapper for the 3 autonomy sub-modules.

Instead of a monolithic black box, the autonomy stack is now 3 independent Modules:
  - TerrainModule:       traversability analysis (In: odometry, map_cloud → Out: terrain_map)
  - LocalPlannerModule:  obstacle avoidance (In: terrain_map, waypoint → Out: local_path)
  - PathFollowerModule:  path tracking (In: local_path → Out: cmd_vel)

Each is independently pluggable via Registry. AutonomyModule remains as a convenience
to add all 3 at once with a single blueprint call.

Usage::

    # Option A: add individually (more control)
    bp.add(TerrainModule, backend="cmu")
    bp.add(LocalPlannerModule, backend="cmu")
    bp.add(PathFollowerModule, backend="pure_pursuit")

    # Option B: convenience wrapper (same result)
    from base_autonomy import add_autonomy_stack
    add_autonomy_stack(bp, backend="cmu")
"""

from __future__ import annotations

import logging
import sys
from importlib import import_module, reload
from typing import Any

from core.blueprint import Blueprint
from core.plugin_seed import seed_registered_plugins
from core.registry import get

logger = logging.getLogger(__name__)

# The three class names are lazy-forward references resolved by __getattr__
# below — each name is exported via lazy import from its real module.
# noqa comments below: ruff F822 sees these as undefined but they are
# resolved by __getattr__ at runtime.
__all__ = [  # noqa: F822
    "LocalPlannerModule",
    "PathFollowerModule",
    "TerrainModule",
    "add_autonomy_stack",
]


_EXPORTS = {
    "TerrainModule": "base_autonomy.modules.terrain_module",
    "LocalPlannerModule": "base_autonomy.modules.local_planner_module",
    "PathFollowerModule": "base_autonomy.modules.path_follower_module",
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
        "terrain": ("base_autonomy.modules.terrain_module",),
        "local_planner": ("base_autonomy.modules.local_planner_module",),
        "path_follower": ("base_autonomy.modules.path_follower_module",),
    }.get(category, ())
    for module_name in module_names:
        if module_name in sys.modules:
            reload(sys.modules[module_name])
        else:
            import_module(module_name)


def add_autonomy_stack(
    bp: Blueprint,
    backend: str = "cmu",
    terrain_backend: str | None = None,
    path_follower_backend: str = "pure_pursuit",
    local_planner_config: dict[str, Any] | None = None,
    **kw,
) -> Blueprint:
    """Add the full autonomy stack (terrain + local_planner + path_follower) to a Blueprint.

    Args:
        bp: Blueprint to add modules to.
        backend: Backend for TerrainModule and LocalPlannerModule ("cmu" or "simple").
        terrain_backend: Optional independent TerrainModule backend.
        path_follower_backend: Backend for PathFollowerModule ("pure_pursuit" or "pid").

    Returns:
        The same Blueprint (for chaining).
    """
    terrain_key = terrain_backend or backend
    TerrainCls = _module_for_backend("terrain", terrain_key)
    LocalPlannerCls = _module_for_backend("local_planner", backend)
    PathFollowerCls = _module_for_backend("path_follower", path_follower_backend)

    bp.add(TerrainCls, alias="TerrainModule", backend=terrain_key)
    bp.add(
        LocalPlannerCls,
        alias="LocalPlannerModule",
        backend=backend,
        **(local_planner_config or {}),
    )
    bp.add(
        PathFollowerCls,
        alias="PathFollowerModule",
        backend=path_follower_backend,
        **kw,
    )
    return bp
