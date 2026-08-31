"""Named product blueprints."""

from __future__ import annotations

from .configuration import (
    ResolvedProductHostConfig,
    resolve_product_host_config,
    resolve_product_host_runtime,
)
from .host import host_blueprint
from .host_defaults import FIELD_PRODUCT_HOST_DEFAULTS, FIELD_PRODUCT_NAMES

__all__ = [
    "FIELD_PRODUCT_HOST_DEFAULTS",
    "FIELD_PRODUCT_NAMES",
    "ResolvedProductHostConfig",
    "host_blueprint",
    "resolve_product_host_config",
    "resolve_product_host_runtime",
]
