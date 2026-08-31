"""Internal driver-backend catalog for local and simulated Modules."""

from __future__ import annotations

from typing import Any

from drivers.catalog import (
    driver_catalog_backends,
    driver_catalog_modules,
    driver_catalog_protocols,
)

_THUNDER_CANONICAL_BACKENDS = driver_catalog_backends("thunder")
_THUNDER_CANONICAL_PROTOCOLS = driver_catalog_protocols("thunder")
_THUNDER_CANONICAL_MODULES = driver_catalog_modules("thunder")

CANONICAL_DRIVER_BACKENDS: dict[str, dict[str, Any]] = {
    "stub": dict(robot="stub"),
    "sim": dict(robot="sim_mujoco"),
    "sim_endpoint": dict(robot="sim_endpoint"),

    **_THUNDER_CANONICAL_BACKENDS,
}

DRIVER_BACKENDS: dict[str, dict[str, Any]] = dict(CANONICAL_DRIVER_BACKENDS)

CANONICAL_DRIVER_PROTOCOLS: dict[str, tuple[str, dict[str, Any]]] = {
    "sim": ("mujoco_inproc", {}),
    "sim_endpoint": ("sim_endpoint", {}),

    "stub": ("stub", {}),
    **_THUNDER_CANONICAL_PROTOCOLS,
}

DRIVER_PROTOCOLS: dict[str, tuple[str, dict[str, Any]]] = dict(
    CANONICAL_DRIVER_PROTOCOLS
)

CANONICAL_DRIVER_MODULES: dict[str, str] = {
    "auto": "StubDogModule",
    "stub": "StubDogModule",
    "sim": "MujocoDriverModule",
    "sim_mujoco": "MujocoDriverModule",
    "sim_endpoint": "SimEndpointDriverModule",

    **_THUNDER_CANONICAL_MODULES,
}

DRIVER_MODULES: dict[str, str] = dict(CANONICAL_DRIVER_MODULES)


def driver_backend_names() -> tuple[str, ...]:
    """Return executable local Host driver backends."""

    return tuple(CANONICAL_DRIVER_PROTOCOLS)


def driver_backend_defaults(name: str) -> dict[str, Any]:
    """Return a copy of one local driver backend's defaults."""

    return dict(DRIVER_BACKENDS[name])


def driver_backend_protocol(name: str) -> tuple[str, dict[str, Any]]:
    """Return a driver protocol and copy of its default parameters."""

    protocol, params = DRIVER_PROTOCOLS[name]
    return protocol, dict(params)


def driver_backend_module_name(name: str) -> str:
    """Return the Blueprint driver module name for a driver backend."""

    try:
        return DRIVER_MODULES[name]
    except KeyError as exc:
        if name == "thunder":
            raise RuntimeError(
                "Python driver 'thunder' was removed; start a real Product "
                "through ProductControl so the native lingtu-driver owns motion"
            ) from exc
        valid = ", ".join(driver_backend_names())
        raise KeyError(
            f"unknown driver module mapping: {name}; driver backends: {valid}"
        ) from exc
