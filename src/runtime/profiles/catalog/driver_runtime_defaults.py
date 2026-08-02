"""Upper-stack Host defaults keyed by local driver backend."""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.driver_catalog import driver_catalog_runtime_defaults

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

_SIM_ENDPOINT_RUNTIME_DEFAULTS = dict(
    slam_profile="none",
    detector="yoloe",
    encoder="mobileclip",
)

_THUNDER_CANONICAL_RUNTIME_DEFAULTS = driver_catalog_runtime_defaults("thunder")

CANONICAL_DRIVER_RUNTIME_DEFAULTS: dict[str, dict[str, Any]] = {
    "stub": dict(_STUB_RUNTIME_DEFAULTS),
    "sim": dict(_SIM_RUNTIME_DEFAULTS),
    "sim_endpoint": dict(_SIM_ENDPOINT_RUNTIME_DEFAULTS),

    **_THUNDER_CANONICAL_RUNTIME_DEFAULTS,
}

DRIVER_RUNTIME_DEFAULTS: dict[str, dict[str, Any]] = dict(
    CANONICAL_DRIVER_RUNTIME_DEFAULTS
)


def driver_runtime_defaults(name: str) -> dict[str, Any]:
    """Return upper-stack defaults for a local driver backend."""

    return dict(DRIVER_RUNTIME_DEFAULTS[name])
