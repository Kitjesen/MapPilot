"""Registry tokens for SLAM runtime profiles.

These entries describe selectable SLAM profiles for catalogs and validation.
Actual runtime IO is provided by explicit localization adapters, so importing
this module must not start ROS2 or native services.
"""

from __future__ import annotations

from dataclasses import dataclass

from runtime.registry import register


@dataclass(frozen=True)
class SlamRuntimeProfile:
    """Selectable native SLAM runtime identity."""

    name: str
    description: str


@register("slam", "native_dds", description="Native C++ SLAM DDS runtime")
class NativeDdsProfile(SlamRuntimeProfile):
    """Native C++ SLAM process reached through typed DDS adapters."""

    def __init__(self) -> None:
        super().__init__(
            name="native_dds",
            description="Native C++ SLAM DDS runtime",
        )
