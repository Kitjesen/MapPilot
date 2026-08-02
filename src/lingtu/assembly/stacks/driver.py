"""Driver stack: robot connection.

Two resolution patterns are supported:

1. Direct backend registry lookup (robot string → @register("driver", ...)):
   driver("thunder") or driver(robot="thunder")
   → get("driver", "thunder") returns ThunderDriver

2. Catalog lookup (driver backend → @register("driver_protocol", ...)):
   driver(driver_backend="thunder")
   → DriverBackend("thunder") resolves protocol="grpc_brainstem"
   → get("driver_protocol", "grpc_brainstem") returns ThunderDriver

The catalog-based pattern is preferred for new code because:
- It decouples the driver class from how it is connected
- The same driver protocol can serve multiple backends
- Configuration defaults live in the driver catalog, not across call sites
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
from runtime.profiles.catalog.driver_backends import (
    DRIVER_PROTOCOLS,
    driver_backend_names,
)
from runtime.registry import auto_select, get

logger = logging.getLogger(__name__)


# Registry of known driver backends → (protocol, default params).
# The catalog owns defaults; this local registry remains mutable for tests.
_DRIVER_BACKEND_REGISTRY: dict[str, tuple[str, dict[str, Any]]] = {
    name: (protocol, dict(params)) for name, (protocol, params) in DRIVER_PROTOCOLS.items()
}


class DriverBackend:
    """Local driver backend that resolves a protocol and parameters.

    Maps a backend name (for example, ``thunder`` or ``sim``) to a protocol
    (registered under the "driver_protocol" registry category) and optional
    connection parameters.

    Usage::

        backend = DriverBackend("thunder")
        DriverCls = get("driver_protocol", backend.protocol)
        bp.add(DriverCls, **backend.params)

    The ``driver()`` factory accepts ``driver_backend=`` as an alternative to
    ``robot=`` for the same purpose.
    """

    def __init__(self, name: str) -> None:
        """Resolve a driver backend to its protocol and parameters.

        Args:
            name: Driver backend name (for example, ``thunder`` or ``sim``).

        Raises:
            KeyError: if ``name`` is not a known backend.
        """
        entry = _DRIVER_BACKEND_REGISTRY.get(name)
        if entry is None:
            valid = ", ".join(driver_backend_names())
            raise KeyError(f"Unknown DriverBackend '{name}'. Canonical backends: {valid}")
        self.protocol: str = entry[0]
        self.params: dict[str, Any] = dict(entry[1])

    @classmethod
    def register(cls, name: str, protocol: str, **params: Any) -> None:
        """Register a custom driver backend at runtime.

        Usage::

            DriverBackend.register("my_driver", "grpc_brainstem",
                                   dog_host="10.0.0.1", dog_port=13145)
        """
        global _DRIVER_BACKEND_REGISTRY
        _DRIVER_BACKEND_REGISTRY[name] = (protocol, params)

    @classmethod
    def known_backends(cls) -> list[str]:
        """Return known canonical and runtime-registered backend names."""

        return sorted(_DRIVER_BACKEND_REGISTRY)


def driver(robot: str = "thunder", **config) -> Blueprint:
    """Driver connection stack.

    ``robot`` selects a registered ``driver`` name directly. ``config`` may
    contain ``driver_backend=`` to resolve through the driver catalog.

    Catalog pattern::

        driver(driver_backend="thunder")

    Direct pattern::

        driver("thunder", dog_host="192.168.66.190")
    """
    cfg = get_config()
    bp = Blueprint()

    seed_driver_plugins_for_runtime()

    # Resolve a catalog backend through the driver_protocol registry.
    if "driver_backend" in config:
        backend_name = config.pop("driver_backend")
        backend = DriverBackend(backend_name)
        protocol_key = ensure_driver_runtime_registered(
            "driver_protocol",
            backend.protocol,
        )
        DriverCls = get("driver_protocol", protocol_key)
        driver_config = {**backend.params, **dict(config)}
        logger.debug(
            "Resolved driver_backend=%s → protocol=%s, driver=%s",
            backend_name,
            backend.protocol,
            DriverCls.__name__,
        )
    # Resolve the direct robot key through the driver registry.
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
