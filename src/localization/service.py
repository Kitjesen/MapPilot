"""Public localization command contract.

Localization owns saved-map alignment and map-to-odometry updates.  Callers
provide a map artifact and pose seed; they do not know whether the command is
served by an in-process module or the native DDS endpoint.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol


@dataclass(frozen=True)
class RelocalizationResult:
    success: bool
    message: str
    quality: float | None = None
    stdout: str = ""
    stderr: str = ""
    returncode: int | None = None
    timed_out: bool = False
    details: Mapping[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.success


class RelocalizationService(Protocol):
    """Backend-neutral localization commands used by navigation and Gateway."""

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

    def track_against_map(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        ...


class Localization:
    """Stable localization API over an optional runtime backend.

    Module discovery and transport selection happen during product assembly.
    Consumers keep this object and never reach into the selected module or DDS
    adapter directly.
    """

    def __init__(self, backend: RelocalizationService | None = None) -> None:
        self._backend = backend

    @property
    def backend(self) -> RelocalizationService | None:
        return self._backend

    @property
    def available(self) -> bool:
        return self._backend is not None

    def bind(self, backend: RelocalizationService | None) -> None:
        self._backend = backend

    @staticmethod
    def _unavailable(command: str) -> RelocalizationResult:
        return RelocalizationResult(
            success=False,
            message="relocalization service unavailable",
            details={"code": "unavailable", "command": command},
        )

    def trigger_global_relocalize(self, *, timeout_s: float = 10.0) -> RelocalizationResult:
        if self._backend is None:
            return self._unavailable("global_relocalize")
        return self._backend.trigger_global_relocalize(timeout_s=timeout_s)

    def query_global_relocalize_status(self, *, timeout_s: float = 5.0) -> RelocalizationResult:
        if self._backend is None:
            return self._unavailable("relocalization_status")
        query = getattr(self._backend, "query_global_relocalize_status", None)
        if not callable(query):
            return RelocalizationResult(
                success=False,
                message="relocalization status is not supported by the active backend",
                details={"code": "unsupported", "command": "relocalization_status"},
            )
        return query(timeout_s=timeout_s)

    def relocalize_saved_map(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 30.0,
    ) -> RelocalizationResult:
        if self._backend is None:
            return self._unavailable("relocalize_saved_map")
        return self._backend.relocalize_saved_map(
            pcd_path,
            x,
            y,
            yaw,
            timeout_s=timeout_s,
        )

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
        if self._backend is None:
            return self._unavailable("relocalize_saved_map")
        return self._backend.relocalize_saved_map_with_env(
            pcd_path,
            x,
            y,
            yaw,
            timeout_s=timeout_s,
            base_env=base_env,
        )

    def track_against_map(
        self,
        pcd_path: str | os.PathLike[str],
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        if self._backend is None:
            return self._unavailable("track_against_map")
        track = getattr(self._backend, "track_against_map", None)
        if not callable(track):
            return RelocalizationResult(
                success=False,
                message="saved-map tracking is not supported by the active backend",
                details={"code": "unsupported", "command": "track_against_map"},
            )
        return track(pcd_path, x, y, yaw, timeout_s=timeout_s)
