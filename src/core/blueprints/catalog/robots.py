"""Robot preset catalog.

Product profiles choose *what* to run. Robot presets describe hardware and
connection defaults. Compatibility aliases such as ``s100p`` remain available
for older scripts, but product-facing code should prefer ``thunder``.
"""

from __future__ import annotations

from typing import Any

_THUNDER_LOCAL_PRESET = dict(
    robot="thunder",
    slam_profile="localizer",
    detector="bpu",
    encoder="mobileclip",
    dog_host="127.0.0.1",
    dog_port=13145,
    auto_enable=False,
    auto_standup=False,
)

_THUNDER_REMOTE_PRESET = dict(
    robot="thunder",
    slam_profile="localizer",
    detector="bpu",
    encoder="mobileclip",
    dog_host="192.168.66.190",
    dog_port=13145,
    auto_enable=False,
    auto_standup=False,
)

CANONICAL_ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    "stub": dict(robot="stub", slam_profile="none", detector="yoloe", encoder="mobileclip"),
    "sim": dict(robot="sim_mujoco", slam_profile="bridge", detector="yoloe", encoder="mobileclip"),
    "ros2": dict(robot="sim_ros2", slam_profile="bridge", detector="yoloe", encoder="mobileclip"),
    "sim_gazebo": dict(robot="sim_ros2", slam_profile="none", detector="yoloe", encoder="mobileclip"),
    "thunder": dict(_THUNDER_LOCAL_PRESET),
    "thunder_remote": dict(_THUNDER_REMOTE_PRESET),
}

COMPAT_ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    "s100p": dict(_THUNDER_LOCAL_PRESET),
    "navigate": dict(_THUNDER_LOCAL_PRESET),
}

ROBOT_PRESETS: dict[str, dict[str, Any]] = {
    **CANONICAL_ROBOT_PRESETS,
    **COMPAT_ROBOT_PRESETS,
}

_THUNDER_LOCAL_DRIVER_PARAMS = {
    "dog_host": "127.0.0.1",
    "dog_port": 13145,
    "auto_enable": False,
    "auto_standup": False,
}

_THUNDER_REMOTE_DRIVER_PARAMS = {
    "dog_host": "192.168.66.190",
    "dog_port": 13145,
    "auto_enable": False,
    "auto_standup": False,
}

CANONICAL_ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = {
    "thunder": (
        "grpc_brainstem",
        dict(_THUNDER_LOCAL_DRIVER_PARAMS),
    ),
    "thunder_remote": (
        "grpc_brainstem",
        dict(_THUNDER_REMOTE_DRIVER_PARAMS),
    ),
    "sim": ("mujoco_inproc", {}),
    "sim_gazebo": ("ros2_bridge", {}),
    "ros2": ("ros2_bridge", {}),
    "stub": ("stub", {}),
}

COMPAT_ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = {
    "s100p": ("grpc_brainstem", dict(_THUNDER_LOCAL_DRIVER_PARAMS)),
    "navigate": ("grpc_brainstem", dict(_THUNDER_LOCAL_DRIVER_PARAMS)),
}

ROBOT_DRIVER_PROFILES: dict[str, tuple[str, dict[str, Any]]] = {
    **CANONICAL_ROBOT_DRIVER_PROFILES,
    **COMPAT_ROBOT_DRIVER_PROFILES,
}

CANONICAL_ROBOT_DRIVER_MODULES: dict[str, str] = {
    "auto": "StubDogModule",
    "stub": "StubDogModule",
    "sim": "MujocoDriverModule",
    "sim_mujoco": "MujocoDriverModule",
    "sim_gazebo": "ROS2SimDriverModule",
    "sim_ros2": "ROS2SimDriverModule",
    "ros2": "ROS2SimDriverModule",
    "thunder": "ThunderDriver",
    "thunder_remote": "ThunderDriver",
}

COMPAT_ROBOT_DRIVER_MODULES: dict[str, str] = {
    "s100p": "ThunderDriver",
    "navigate": "ThunderDriver",
}

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
