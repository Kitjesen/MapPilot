"""Driver stack: robot connection + camera bridge (if needed).

Two resolution patterns are supported:

1. Legacy (robot string → @register("driver", ...)):
   driver("thunder") or driver(robot="thunder")
   → get("driver", "thunder") returns ThunderDriver

2. New (profile → RobotProfile → @register("driver_protocol", ...)):
   driver(profile="s100p")
   → RobotProfile("s100p") resolves protocol="grpc_brainstem"
   → get("driver_protocol", "grpc_brainstem") returns ThunderDriver

The profile-based pattern is preferred for new code because:
- It decouples the driver class from how it is connected
- The same driver_protocol can serve multiple robot presets
- Configuration defaults live in RobotProfile, not spread across call sites
"""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint
from core.config import get_config
from core.plugin_seed import seed_builtin_plugins
from core.registry import auto_select, get

logger = logging.getLogger(__name__)


# Registry of known robot presets → (protocol, default params).
# Kept in sync with ROBOT_PRESETS in core.runtime_profiles.
_ROBOT_PROFILE_REGISTRY: dict[str, tuple[str, dict[str, Any]]] = {
    "s100p": ("grpc_brainstem", {
        "dog_host": "127.0.0.1",
        "dog_port": 13145,
        "auto_enable": False,
        "auto_standup": False,
    }),
    "thunder": ("grpc_brainstem", {
        "dog_host": "192.168.66.190",
        "dog_port": 13145,
        "auto_enable": False,
        "auto_standup": False,
    }),
    "navigate": ("grpc_brainstem", {
        "dog_host": "127.0.0.1",
        "dog_port": 13145,
        "auto_enable": False,
        "auto_standup": False,
    }),
    "sim": ("mujoco_inproc", {}),
    "sim_gazebo": ("ros2_bridge", {}),
    "ros2": ("ros2_bridge", {}),
    "stub": ("stub", {}),
}


class RobotProfile:
    """Robot connection profile — resolves driver protocol and parameters.

    Maps a robot preset name (e.g. "s100p", "sim") to a driver protocol
    (registered under the "driver_protocol" registry category) and optional
    connection parameters.

    Usage::

        profile = RobotProfile("s100p")
        DriverCls = get("driver_protocol", profile.protocol)
        bp.add(DriverCls, **profile.params)

    The ``driver()`` factory accepts ``profile=`` as an alternative to
    ``robot=`` for the same purpose.
    """

    def __init__(self, name: str) -> None:
        """Resolve a robot preset name to its driver protocol and params.

        Args:
            name: Robot preset name (e.g. "s100p", "thunder", "sim").

        Raises:
            KeyError: if ``name`` is not a known preset.
        """
        entry = _ROBOT_PROFILE_REGISTRY.get(name)
        if entry is None:
            valid = ", ".join(sorted(_ROBOT_PROFILE_REGISTRY))
            raise KeyError(
                f"Unknown RobotProfile '{name}'. "
                f"Available presets: {valid}"
            )
        self.protocol: str = entry[0]
        self.params: dict[str, Any] = dict(entry[1])

    @classmethod
    def register(cls, name: str, protocol: str, **params: Any) -> None:
        """Register a custom robot profile at runtime.

        Usage::

            RobotProfile.register("my_robot", "grpc_brainstem",
                                  dog_host="10.0.0.1", dog_port=13145)
        """
        global _ROBOT_PROFILE_REGISTRY
        _ROBOT_PROFILE_REGISTRY[name] = (protocol, params)

    @classmethod
    def known_presets(cls) -> list[str]:
        """Return sorted list of all known preset names."""
        return sorted(_ROBOT_PROFILE_REGISTRY)


def _ensure_drivers_registered():
    seed_builtin_plugins(groups=("driver",))


def driver(robot: str = "thunder", **config) -> Blueprint:
    """Driver + optional camera bridge.

    ``robot`` is the legacy parameter — a registered "driver" name.
    ``config`` may contain ``profile=`` for the new RobotProfile pattern.

    New pattern::

        driver(profile="s100p")

    Legacy pattern::

        driver("thunder", dog_host="192.168.66.190")
    """
    cfg = get_config()
    bp = Blueprint()

    _ensure_drivers_registered()

    # New pattern: resolve via RobotProfile → driver_protocol registry
    if "profile" in config:
        profile_name = config.pop("profile")
        profile = RobotProfile(profile_name)
        DriverCls = get("driver_protocol", profile.protocol)
        driver_config = {**profile.params, **dict(config)}
        logger.debug(
            "Resolved profile=%s → protocol=%s, driver=%s",
            profile_name, profile.protocol, DriverCls.__name__,
        )
    # Legacy pattern: resolve robot string → driver registry
    else:
        if robot == "auto":
            import platform
            robot = auto_select("driver", platform=platform.machine().lower()) or "stub"
        DriverCls = get("driver", robot)
        driver_config = dict(config)

    driver_config.setdefault("dog_host", cfg.driver.dog_host)
    driver_config.setdefault("dog_port", cfg.driver.dog_port)
    driver_config.setdefault("auto_enable", cfg.driver.auto_enable)
    driver_config.setdefault("auto_standup", cfg.driver.auto_standup)
    bp.add(DriverCls, **driver_config)

    # Camera bridge moved to perception() stack — only loaded when needed
    return bp


def driver_name(robot: str) -> str:
    """Resolve driver class name for wiring."""
    _ensure_drivers_registered()
    if robot == "auto":
        import platform
        robot = auto_select("driver", platform=platform.machine().lower()) or "stub"
    return get("driver", robot).__name__


# Known driver class names (used by Blueprint as module names).
_KNOWN_DRIVER_CLASSES = frozenset({
    "ThunderDriver",
    "StubDogModule",
    "MujocoDriverModule",
    "ROS2SimDriverModule",
    "NovaDogConnection",
    "StubConnection",
})


def get_current_driver_name(system: Any) -> str:
    """Inspect a SystemHandle to find the active driver module name.

    Scans ``system.modules`` for known driver class names and returns the
    first match.  Raises ``KeyError`` if no known driver is found.

    Useful for wiring helpers that need to know which module is the robot
    driver without hardcoding a class name.
    """
    for name in system.modules:
        if name in _KNOWN_DRIVER_CLASSES:
            return name
    raise KeyError(
        f"No known driver module found in system. "
        f"Available modules: {list(system.modules.keys())}"
    )
