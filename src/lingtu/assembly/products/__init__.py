"""Named product blueprints."""

from __future__ import annotations

from typing import Any, Mapping

from runtime.blueprint import Blueprint

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
) -> Blueprint:
    """Return the standard product blueprint for any resolved profile."""

    assembly_config = dict(config)
    assembly_config["_product_profile"] = profile
    return thunder_blueprint(assembly_config)


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
