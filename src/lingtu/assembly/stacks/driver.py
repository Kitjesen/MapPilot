"""Local and simulation Host driver stack.

Real Thunder motion is intentionally absent from this Module factory. Field
Products use ProductControl and the native ``lingtu-driver`` process.
"""

from __future__ import annotations

import logging
from typing import Any

from drivers.backends import (
    DRIVER_PROTOCOLS,
    driver_backend_names,
)
from runtime.adapters.driver_runtime import (
    ensure_driver_runtime_registered,
    seed_driver_plugins_for_runtime,
)
from runtime.blueprint import Blueprint
from runtime.config import get_config
from runtime.registry import auto_select, get

logger = logging.getLogger(__name__)

_RETIRED_FIELD_DRIVER_KEYS = frozenset(
    {"thunder", "thunder_remote", "grpc_brainstem"}
)


def _reject_retired_field_driver(name: object) -> None:
    key = str(name or "").strip()
    if key in _RETIRED_FIELD_DRIVER_KEYS:
        raise RuntimeError(
            f"Python driver {key!r} was removed; start a real Product through "
            "ProductControl so the native lingtu-driver owns motion"
        )


class DriverBackend:
    """Local driver backend that resolves a protocol and parameters.

    Maps a backend name (for example, ``stub`` or ``sim``) to a protocol
    (registered under the "driver_protocol" registry category) and optional
    connection parameters.

    Usage::

        backend = DriverBackend("sim")
        DriverCls = get("driver_protocol", backend.protocol)
        bp.add(DriverCls, **backend.params)

    The ``driver()`` factory accepts ``driver_backend=`` as an alternative to
    ``robot=`` for the same purpose.
    """

    def __init__(self, name: str) -> None:
        """Resolve a driver backend to its protocol and parameters.

        Args:
            name: Local or simulation driver backend name.

        Raises:
            KeyError: if ``name`` is not a known backend.
        """
        _reject_retired_field_driver(name)
        entry = DRIVER_PROTOCOLS.get(name)
        if entry is None:
            valid = ", ".join(driver_backend_names())
            raise KeyError(f"Unknown DriverBackend '{name}'. Canonical backends: {valid}")
        self.protocol: str = entry[0]
        self.params: dict[str, Any] = dict(entry[1])

def _driver_module(name: str, *, category: str = "driver") -> type:
    if category == "driver":
        _reject_retired_field_driver(name)
        if name == "auto":
            import platform

            seed_driver_plugins_for_runtime()
            name = auto_select("driver", platform=platform.machine().lower()) or "stub"
    key = ensure_driver_runtime_registered(category, name)
    return get(category, key)


def driver(robot: str = "stub", **config) -> Blueprint:
    """Driver connection stack.

    ``robot`` selects a registered ``driver`` name directly. ``config`` may
    contain ``driver_backend=`` to resolve through the driver catalog.

    Catalog pattern::

        driver(driver_backend="sim")

    Direct pattern::

        driver("stub")
    """
    cfg = get_config()
    bp = Blueprint()

    # Resolve a catalog backend through the driver_protocol registry.
    if "driver_backend" in config:
        backend_name = config.pop("driver_backend")
        backend = DriverBackend(backend_name)
        DriverCls = _driver_module(backend.protocol, category="driver_protocol")
        driver_config = {**backend.params, **dict(config)}
        logger.debug(
            "Resolved driver_backend=%s → protocol=%s, driver=%s",
            backend_name,
            backend.protocol,
            DriverCls.__name__,
        )
    # Resolve the direct robot key through the driver registry.
    else:
        DriverCls = _driver_module(robot)
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
    return _driver_module(robot).__name__
