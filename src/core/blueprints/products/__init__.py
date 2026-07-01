"""Named product blueprints."""

from __future__ import annotations

from typing import Any, Mapping

from core.blueprint import Blueprint
from core.blueprints.catalog.products import PRODUCT_PROFILES

from .thunder import (
    thunder_basic_blueprint,
    thunder_basic_config,
    thunder_blueprint,
    thunder_explore_blueprint,
    thunder_explore_config,
    thunder_lite_blueprint,
    thunder_lite_config,
    thunder_map_blueprint,
    thunder_map_config,
    thunder_nav_blueprint,
    thunder_nav_config,
)


def product_blueprint_for_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint | None:
    """Return the product-level blueprint for a canonical profile, if any."""

    if profile in PRODUCT_PROFILES:
        return thunder_blueprint(config)
    return None


__all__ = [
    "product_blueprint_for_profile",
    "thunder_basic_blueprint",
    "thunder_basic_config",
    "thunder_blueprint",
    "thunder_explore_blueprint",
    "thunder_explore_config",
    "thunder_lite_blueprint",
    "thunder_lite_config",
    "thunder_map_blueprint",
    "thunder_map_config",
    "thunder_nav_blueprint",
    "thunder_nav_config",
]
