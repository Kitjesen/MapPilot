"""Named product blueprints."""

from __future__ import annotations

from typing import Any, Mapping

from runtime.blueprint import Blueprint

from .configuration import (
    ResolvedProductHostConfig,
    resolve_product_host_config,
    resolve_product_host_runtime,
)
from .host_defaults import FIELD_PRODUCT_HOST_DEFAULTS, FIELD_PRODUCT_NAMES
from .thunder import thunder_blueprint


def host_blueprint_for_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Build the Host graph for an already resolved local Profile."""

    if profile in FIELD_PRODUCT_NAMES:
        raise ValueError(
            f"{profile!r} is a Product; use host_blueprint_for_product(...)"
        )
    return thunder_blueprint(config)


def host_blueprint_for_product(
    product: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Build the Host graph for an already resolved field Product."""

    if product not in FIELD_PRODUCT_NAMES:
        raise ValueError(
            f"{product!r} is not a Product; use host_blueprint_for_profile(...)"
        )
    return thunder_blueprint(config)


__all__ = [
    "FIELD_PRODUCT_HOST_DEFAULTS",
    "FIELD_PRODUCT_NAMES",
    "ResolvedProductHostConfig",
    "host_blueprint_for_product",
    "host_blueprint_for_profile",
    "resolve_product_host_config",
    "resolve_product_host_runtime",
    "thunder_blueprint",
]
