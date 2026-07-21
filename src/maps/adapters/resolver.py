"""Map adapter resolution for Blueprint composition."""

from __future__ import annotations

from typing import Any

from runtime.plugin_resolution import optional_stack_module


def map_output_adapter_module(*, enable_dds: bool = False) -> type[Any] | None:
    """Resolve the optional typed-DDS map output adapter."""
    if not enable_dds:
        return None
    return optional_stack_module(
        "map",
        "dds_map_output",
        seed_group="map_dds",
        fallback="maps.adapters.dds.output.DDSMapOutModule",
    )
