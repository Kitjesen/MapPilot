"""Local and simulation Host driver stack.

Real Thunder motion is intentionally absent from this Module factory. Field
Products use ProductControl and the native ``lingtu-driver`` process.
"""

from __future__ import annotations

from runtime.adapters.driver_runtime import (
    ensure_driver_runtime_registered,
    seed_driver_plugins_for_runtime,
)
from runtime.blueprint import Blueprint
from runtime.config import get_config
from runtime.registry import auto_select, get

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


def _driver_module(name: str) -> type:
    _reject_retired_field_driver(name)
    if name == "auto":
        import platform

        seed_driver_plugins_for_runtime()
        name = auto_select("driver", platform=platform.machine().lower()) or "stub"
    key = ensure_driver_runtime_registered("driver", name)
    return get("driver", key)


def driver(robot: str = "stub", **config) -> Blueprint:
    """Driver connection stack.

    ``robot`` selects a registered ``driver`` name directly::

        driver("stub")
    """
    if "driver_backend" in config:
        raise TypeError(
            "driver_backend= was removed; pass a registered driver key as the "
            "first argument (stub, sim_mujoco, or sim_endpoint). Real Products "
            "must use ProductControl and the native lingtu-driver"
        )

    cfg = get_config()
    bp = Blueprint()
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
