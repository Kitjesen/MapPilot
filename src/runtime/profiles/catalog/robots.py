"""Robot preset catalog.

Product profiles choose *what* to run. Robot presets describe hardware and
connection defaults. Compatibility aliases such as ``s100p`` remain available
for older scripts, but product-facing code should prefer ``thunder``.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.robot_archives import (
    robot_archive_canonical_driver_modules,
    robot_archive_canonical_driver_profiles,
    robot_archive_canonical_presets,
    robot_archive_compat_driver_modules,
    robot_archive_compat_driver_profiles,
    robot_archive_compat_presets,
)

_THUNDER_CANONICAL_PRESETS = robot_archive_canonical_presets("thunder")
_THUNDER_COMPAT_PRESETS = robot_archive_compat_presets("thunder")
_THUNDER_CANONICAL_DRIVER_PROFILES = robot_archive_canonical_driver_profiles("thunder")
_THUNDER_COMPAT_DRIVER_PROFILES = robot_archive_compat_driver_profiles("thunder")
_THUNDER_CANONICAL_DRIVER_MODULES = robot_archive_canonical_driver_modules("thunder")
_THUNDER_COMPAT_DRIVER_MODULES = robot_archive_compat_driver_modules("thunder")

CANONICAL_ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    "stub": dict(robot="stub"),
    "sim": dict(robot="sim_mujoco"),
    "sim_endpoint": dict(robot="sim_endpoint"),
    "sim_gazebo": dict(robot="sim_endpoint"),
    **_THUNDER_CANONICAL_PRESETS,
}

COMPAT_ROBOT_PRESETS: dict[str, dict[str, Any]] = dict(_THUNDER_COMPAT_PRESETS)

ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    **CANONICAL_ROBOT_PRESETS,
    **COMPAT_ROBOT_PRESETS,
}

CANONICAL_ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = {
    "sim": ("mujoco_inproc", {}),
    "sim_endpoint": ("sim_endpoint", {}),
    "sim_gazebo": ("sim_endpoint", {}),
    "stub": ("stub", {}),
    **_THUNDER_CANONICAL_DRIVER_PROFILES,
}

COMPAT_ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = dict(
    _THUNDER_COMPAT_DRIVER_PROFILES
)

ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = {
    **CANONICAL_ROBOT_DRIVER_PROFILES,
    **COMPAT_ROBOT_DRIVER_PROFILES,
}

CANONICAL_ROBOT_DRIVER_MODULES: dict[str, str] = {
    "auto": "StubDogModule",
    "stub": "StubDogModule",
    "sim": "MujocoDriverModule",
    "sim_mujoco": "MujocoDriverModule",
    "sim_endpoint": "SimEndpointDriverModule",
    "sim_gazebo": "SimEndpointDriverModule",
    **_THUNDER_CANONICAL_DRIVER_MODULES,
}

COMPAT_ROBOT_DRIVER_MODULES: dict[str, str] = dict(_THUNDER_COMPAT_DRIVER_MODULES)

ROBOT_DRIVER_MODULES: dict[str, str] = {
    **CANONICAL_ROBOT_DRIVER_MODULES,
    **COMPAT_ROBOT_DRIVER_MODULES,
}


def robot_preset_names(*, include_compat: bool = True) -> tuple[str, ...]:
    """Return canonical robot presets, optionally including compatibility names."""

    names = list(CANONICAL_ROBOT_PRESETS)
    if include_compat:
        names.extend(name for name in COMPAT_ROBOT_PRESETS if name not in names)
    return tuple(names)


def robot_preset(name: str) -> dict[str, Any]:
    """Return a copy of a robot preset config."""

    return dict(ROBOT_PRESETS[name])


def robot_driver_profile_names(*, include_compat: bool = True) -> tuple[str, ...]:
    """Return canonical driver profile names, optionally including compat names."""

    names = list(CANONICAL_ROBOT_DRIVER_PROFILES)
    if include_compat:
        names.extend(name for name in COMPAT_ROBOT_DRIVER_PROFILES if name not in names)
    return tuple(names)


def robot_driver_profile(name: str) -> tuple[str, dict[str, Any]]:
    """Return a driver protocol and copy of its default parameters."""

    protocol, params = ROBOT_DRIVER_PROFILES[name]
    return protocol, dict(params)


def robot_driver_module_name(name: str) -> str:
    """Return the blueprint driver module name for a robot value or preset alias."""

    try:
        return ROBOT_DRIVER_MODULES[name]
    except KeyError as exc:
        valid = ", ".join(robot_preset_names(include_compat=False))
        raise KeyError(
            f"unknown robot driver module mapping: {name}; "
            f"canonical robot presets: {valid}"
        ) from exc
