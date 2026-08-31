"""Fail-closed Windows Cook, stage, and package interface for RobotSimUE."""

from .core import DistributionError, DistributionResult, WindowsDistribution
from .smoke import (
    VerifiedWindowsRelease,
    WindowsPackagedSmokeConfig,
    resolve_verified_windows_release,
    run_windows_packaged_smoke,
)

__all__ = [
    "DistributionError",
    "DistributionResult",
    "VerifiedWindowsRelease",
    "WindowsDistribution",
    "WindowsPackagedSmokeConfig",
    "resolve_verified_windows_release",
    "run_windows_packaged_smoke",
]
