"""Driver runtime registry resolution helpers."""

from __future__ import annotations

import sys
from importlib import import_module, reload

from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get


_LEGACY_DRIVER_KEYS = frozenset({"nova_dog"})
_SIM_DRIVER_KEYS = frozenset({"sim", "sim_mujoco", "mujoco_inproc", "sim_endpoint", "sim_gazebo"})
_DRIVER_KEY_ALIASES = {
    "thunder_remote": "thunder",
    "s100p": "thunder",
    "navigate": "thunder",
    "grpc_brainstem": "thunder",
    "sim": "sim_mujoco",
    "mujoco_inproc": "sim_mujoco",
    "sim_gazebo": "sim_endpoint",
}

_DRIVER_FALLBACK_MODULES: dict[str, tuple[str, ...]] = {
    "auto": ("drivers.sim.stub",),
    "stub": ("drivers.sim.stub",),
    "thunder": (
        "drivers.real.thunder.han_dog_module",
    ),
    "thunder_remote": (
        "drivers.real.thunder.han_dog_module",
    ),
    "s100p": (
        "drivers.real.thunder.han_dog_module",
    ),
    "navigate": (
        "drivers.real.thunder.han_dog_module",
    ),
    "grpc_brainstem": (
        "drivers.real.thunder.han_dog_module",
    ),
    "nova_dog": (
        "drivers.real.thunder.connection",
    ),
    "sim": ("drivers.sim.mujoco.driver",),
    "sim_mujoco": ("drivers.sim.mujoco.driver",),
    "mujoco_inproc": ("drivers.sim.mujoco.driver",),
    "sim_endpoint": ("drivers.sim.endpoint",),
    "sim_gazebo": ("drivers.sim.endpoint",),
}


def seed_driver_plugins_for_runtime(
    key: str | None = None,
    *,
    reload_loaded: bool = False,
) -> None:
    """Seed driver plugin groups needed by a runtime key or protocol."""

    groups = ["driver"]
    if key in _LEGACY_DRIVER_KEYS:
        groups.append("driver_legacy")
    if key in _SIM_DRIVER_KEYS:
        groups.append("driver_sim")
    seed_registered_plugins(groups=tuple(groups), reload_loaded=reload_loaded)


def _candidate_keys(category: str, key: str) -> tuple[str, ...]:
    if category != "driver":
        return (key,)
    alias = _DRIVER_KEY_ALIASES.get(key)
    if alias is None or alias == key:
        return (key,)
    return (key, alias)


def _registered_key(category: str, key: str) -> str | None:
    for candidate in _candidate_keys(category, key):
        try:
            get(category, candidate)
            return candidate
        except KeyError:
            pass
    return None


def ensure_driver_runtime_registered(category: str, key: str) -> str:
    """Ensure a driver registry key can be resolved.

    Existing registry entries win. Built-in product/runtime plugin catalogs are
    seeded next. Direct module fallback is reserved for local compatibility
    modules that may not be present in an installed catalog.
    """

    registered = _registered_key(category, key)
    if registered is not None:
        return registered

    seed_driver_plugins_for_runtime(key)
    registered = _registered_key(category, key)
    if registered is not None:
        return registered

    seed_driver_plugins_for_runtime(key, reload_loaded=True)
    registered = _registered_key(category, key)
    if registered is not None:
        return registered

    fallback_modules = list(_DRIVER_FALLBACK_MODULES.get(key, ()))
    for module_name in fallback_modules:
        if module_name in sys.modules:
            reload(sys.modules[module_name])
        else:
            import_module(module_name)

    registered = _registered_key(category, key)
    if registered is not None:
        return registered

    return key
