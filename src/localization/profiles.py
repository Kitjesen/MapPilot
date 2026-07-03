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
    name: str
    mode: str
    description: str


@register("slam", "fastlio2", description="Fast-LIO2 mapping runtime profile")
class FastLio2Profile(SlamRuntimeProfile):
    def __init__(self) -> None:
        super().__init__(
            name="fastlio2",
            mode="mapping",
            description="Fast-LIO2 mapping runtime profile",
        )


@register("slam", "pointlio", description="Point-LIO mapping runtime profile")
class PointLioProfile(SlamRuntimeProfile):
    def __init__(self) -> None:
        super().__init__(
            name="pointlio",
            mode="mapping",
            description="Point-LIO mapping runtime profile",
        )


@register("slam", "localizer", description="Saved-map localization runtime profile")
class LocalizerProfile(SlamRuntimeProfile):
    def __init__(self) -> None:
        super().__init__(
            name="localizer",
            mode="localization",
            description="Saved-map localization runtime profile",
        )


@register("slam", "genz", description="GenZ-ICP localization runtime profile")
class GenZProfile(SlamRuntimeProfile):
    def __init__(self) -> None:
        super().__init__(
            name="genz",
            mode="localization",
            description="GenZ-ICP localization runtime profile",
        )
