"""Navigation IO adapter composition for the navigation stack."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.profiles.binding_policy import navigation_io_adapters_enabled


def add_navigation_io_adapter_stack(bp: Blueprint, **config) -> Blueprint:
    """Add navigation IO adapters when runtime binding requires them."""

    if not navigation_io_adapters_enabled(config):
        return bp

    from runtime.blueprints.adapters.navigation_io import (
        add_navigation_io_adapters,
    )

    return add_navigation_io_adapters(bp, **config)


def wire_navigation_output_adapter_stack(bp: Blueprint) -> Blueprint:
    """Wire navigation outputs after optional local autonomy modules exist."""

    from runtime.blueprints.adapters.navigation_io import (
        wire_navigation_output_adapter,
    )

    return wire_navigation_output_adapter(bp)
