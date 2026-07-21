"""Driver stack: robot connection.

Two resolution patterns are supported:

1. Legacy (robot string → @register("driver", ...)):
   driver("thunder") or driver(robot="thunder")
   → get("driver", "thunder") returns ThunderDriver

2. New (profile → RobotProfile → @register("driver_protocol", ...)):
   driver(profile="thunder")
   → RobotProfile("thunder") resolves protocol="grpc_brainstem"
   → get("driver_protocol", "grpc_brainstem") returns ThunderDriver

The profile-based pattern is preferred for new code because:
- It decouples the driver class from how it is connected
- The same driver_protocol can serve multiple robot presets
- Configuration defaults live in RobotProfile, not spread across call sites
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.adapters.driver_runtime import (
    ensure_driver_runtime_registered,
    seed_driver_plugins_for_runtime,
)
from runtime.blueprint import Blueprint
from runtime.config import get_config
from runtime.profiles.catalog.robots import (
    ROBOT_DRIVER_PROFILES,
    robot_driver_profile_names,
)
from runtime.registry import auto_select, get

logger = logging.getLogger(__name__)


# Registry of known robot presets → (protocol, default params).
# The catalog owns defaults; this local registry remains mutable for tests.
_ROBOT_PROFILE_REGISTRY: dict[str, tuple[str, dict[str, Any]]] = {
    name: (protocol, dict(params)) for name, (protocol, params) in ROBOT_DRIVER_PROFILES.items()
}


class RobotProfile:
    """Robot connection profile — resolves driver protocol and parameters.

    Maps a robot preset name (e.g. "thunder", "sim") to a driver protocol
    (registered under the "driver_protocol" registry category) and optional
    connection parameters.

    Usage::

        profile = RobotProfile("thunder")
        DriverCls = get("driver_protocol", profile.protocol)
        bp.add(DriverCls, **profile.params)

    The ``driver()`` factory accepts ``profile=`` as an alternative to
    ``robot=`` for the same purpose.
    """

    def __init__(self, name: str) -> None:
        """Resolve a robot preset name to its driver protocol and params.

        Args:
            name: Robot preset name (e.g. "thunder", "sim").

        Raises:
            KeyError: if ``name`` is not a known preset.
        """
        entry = _ROBOT_PROFILE_REGISTRY.get(name)
        if entry is None:
            valid = ", ".join(robot_driver_profile_names(include_compat=False))
            raise KeyError(f"Unknown RobotProfile '{name}'. Canonical presets: {valid}")
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
    def known_presets(cls, *, include_compat: bool = True) -> list[str]:
        """Return known preset names, optionally hiding compatibility aliases."""

        if include_compat:
            return sorted(_ROBOT_PROFILE_REGISTRY)
        return list(robot_driver_profile_names(include_compat=False))


def driver(robot: str = "thunder", **config) -> Blueprint:
    """Driver connection stack.

    ``robot`` is the legacy parameter — a registered "driver" name.
    ``config`` may contain ``profile=`` for the new RobotProfile pattern.

    New pattern::

        driver(profile="thunder")

    Legacy pattern::

        driver("thunder", dog_host="192.168.66.190")
    """
    cfg = get_config()
    bp = Blueprint()

    seed_driver_plugins_for_runtime()

    # New pattern: resolve via RobotProfile → driver_protocol registry
    if "profile" in config:
        profile_name = config.pop("profile")
        profile = RobotProfile(profile_name)
        protocol_key = ensure_driver_runtime_registered(
            "driver_protocol",
            profile.protocol,
        )
        DriverCls = get("driver_protocol", protocol_key)
        driver_config = {**profile.params, **dict(config)}
        logger.debug(
            "Resolved profile=%s → protocol=%s, driver=%s",
            profile_name,
            profile.protocol,
            DriverCls.__name__,
        )
    # Legacy pattern: resolve robot string → driver registry
    else:
        if robot == "auto":
            import platform

            robot = auto_select("driver", platform=platform.machine().lower()) or "stub"
        driver_key = ensure_driver_runtime_registered("driver", robot)
        DriverCls = get("driver", driver_key)
        driver_config = dict(config)

    driver_config.setdefault("dog_host", cfg.driver.dog_host)
    driver_config.setdefault("dog_port", cfg.driver.dog_port)
    driver_config.setdefault("auto_enable", cfg.driver.auto_enable)
    driver_config.setdefault("auto_standup", cfg.driver.auto_standup)
    bp.add(DriverCls, **driver_config)

    # Camera is owned by perception() and loaded only when needed.
    return bp


def driver_name(robot: str) -> str:
    """Resolve driver class name for wiring."""
    seed_driver_plugins_for_runtime(robot)
    if robot == "auto":
        import platform

        robot = auto_select("driver", platform=platform.machine().lower()) or "stub"
    driver_key = ensure_driver_runtime_registered("driver", robot)
    return get("driver", driver_key).__name__


# Known driver class names (used by Blueprint as module names).
_KNOWN_DRIVER_CLASSES = frozenset(
    {
        "ThunderDriver",
        "StubDogModule",
        "MujocoDriverModule",
        "SimEndpointDriverModule",
        "NovaDogConnection",
    }
)


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
    raise KeyError(f"No known driver module found in system. Available modules: {list(system.modules.keys())}")
