"""Core relocalization capability contract.

Gateway code depends on this protocol only. Concrete ROS service calls live
behind compatibility adapters.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping, Protocol


@dataclass(frozen=True)
class RelocalizationResult:
    success: bool
    message: str
    quality: float | None = None
    stdout: str = ""
    stderr: str = ""
    returncode: int | None = None
    timed_out: bool = False

    @property
    def ok(self) -> bool:
        return self.success


class RelocalizationService(Protocol):
    def trigger_global_relocalize(
        self,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        ...

    def query_global_relocalize_status(
        self,
        *,
        timeout_s: float = 5.0,
    ) -> RelocalizationResult:
        ...

    def relocalize_saved_map(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 30.0,
    ) -> RelocalizationResult:
        ...

    def relocalize_saved_map_with_env(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 20.0,
        base_env: Mapping[str, str] | None = None,
    ) -> RelocalizationResult:
        ...
