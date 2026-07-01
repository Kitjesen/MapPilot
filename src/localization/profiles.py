"""Registry tokens for external SLAM runtime profiles.

These entries describe selectable SLAM profiles for catalogs and validation.
Actual runtime IO is provided by explicit localization adapters, so importing
this module must not start ROS2 or native services.
"""

from __future__ import annotations

from dataclasses import dataclass

from runtime.registry import register


@dataclass(frozen=True)
class ExternalSlamProfile:
    name: str
    mode: str
    description: str


@register("slam", "fastlio2", description="External Fast-LIO2 mapping profile")
class FastLio2Profile(ExternalSlamProfile):
    def __init__(self) -> None:
        super().__init__(
            name="fastlio2",
            mode="mapping",
            description="External Fast-LIO2 mapping profile",
        )


@register("slam", "pointlio", description="External Point-LIO mapping profile")
class PointLioProfile(ExternalSlamProfile):
    def __init__(self) -> None:
        super().__init__(
            name="pointlio",
            mode="mapping",
            description="External Point-LIO mapping profile",
        )


@register("slam", "localizer", description="External saved-map localization profile")
class LocalizerProfile(ExternalSlamProfile):
    def __init__(self) -> None:
        super().__init__(
            name="localizer",
            mode="localization",
            description="External saved-map localization profile",
        )


@register("slam", "genz", description="External GenZ-ICP localization profile")
class GenZProfile(ExternalSlamProfile):
    def __init__(self) -> None:
        super().__init__(
            name="genz",
            mode="localization",
            description="External GenZ-ICP localization profile",
        )
