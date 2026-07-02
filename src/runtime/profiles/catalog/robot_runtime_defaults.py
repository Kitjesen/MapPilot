"""Robot-keyed runtime defaults that are not hardware presets.

Robot presets describe connection and hardware identity. These defaults keep
legacy profile resolution stable for perception and localization choices that
historically came from ``ROBOT_PRESETS``.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.robot_archives import (
    robot_archive_canonical_runtime_defaults,
    robot_archive_compat_runtime_defaults,
)


_STUB_RUNTIME_DEFAULTS = dict(
    slam_profile="none",
    detector="yoloe",
    encoder="mobileclip",
)

_SIM_RUNTIME_DEFAULTS = dict(
    slam_profile="bridge",
    detector="yoloe",
    encoder="mobileclip",
)

_SIM_GAZEBO_RUNTIME_DEFAULTS = dict(
    slam_profile="none",
    detector="yoloe",
    encoder="mobileclip",
)

_THUNDER_CANONICAL_RUNTIME_DEFAULTS = robot_archive_canonical_runtime_defaults(
    "thunder"
)
_THUNDER_COMPAT_RUNTIME_DEFAULTS = robot_archive_compat_runtime_defaults("thunder")


CANONICAL_ROBOT_RUNTIME_DEFAULTS: dict[str, dict[str, Any]] = {
    "stub": dict(_STUB_RUNTIME_DEFAULTS),
    "sim": dict(_SIM_RUNTIME_DEFAULTS),
    "sim_endpoint": dict(_SIM_GAZEBO_RUNTIME_DEFAULTS),
    "sim_gazebo": dict(_SIM_GAZEBO_RUNTIME_DEFAULTS),
    **_THUNDER_CANONICAL_RUNTIME_DEFAULTS,
}

COMPAT_ROBOT_RUNTIME_DEFAULTS: dict[str, dict[str, Any]] = dict(
    _THUNDER_COMPAT_RUNTIME_DEFAULTS
)

ROBOT_RUNTIME_DEFAULTS: dict[str, dict[str, Any]] = {
    **CANONICAL_ROBOT_RUNTIME_DEFAULTS,
    **COMPAT_ROBOT_RUNTIME_DEFAULTS,
}


def robot_runtime_defaults(name: str) -> dict[str, Any]:
    """Return non-hardware runtime defaults for a robot preset."""

    return dict(ROBOT_RUNTIME_DEFAULTS[name])
