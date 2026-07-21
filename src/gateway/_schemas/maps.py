"""Internal Gateway maps request and response models."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator

from gateway._schemas.common import (
    GATEWAY_MAP_FRAME_ID,
    GatewayResponseModel,
)


class MapRequest(BaseModel):
    action: str
    name: str | None = None
    new_name: str | None = None

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        allowed = {
            "list",
            "save",
            "use",
            "build",
            "delete",
            "rename",
            "set_active",
            "build_occupancy",
            "build_occupancy_snapshot",
            "build_octomap",
            "build_artifact",
        }
        if v not in allowed:
            raise ValueError(f"action must be one of {allowed}")
        return v


class MapNameRequest(BaseModel):
    name: str | None = Field(default=None, max_length=128)


class MapRenameRequest(BaseModel):
    old_name: str | None = Field(default=None, max_length=128)
    new_name: str | None = Field(default=None, max_length=128)


class MapSaveRequest(BaseModel):
    name: str | None = Field(default=None, max_length=128)
    optimization: str | None = Field(default=None, max_length=32)
    request_id: str | None = Field(
        default=None,
        max_length=128,
        pattern=r"^[A-Za-z0-9_][A-Za-z0-9_-]*$",
    )

    @field_validator("optimization")
    @classmethod
    def valid_optimization(cls, v: str | None) -> str | None:
        if v is None:
            return None
        value = str(v).strip().lower()
        allowed = {"pgo", "hba", "none", "auto"}
        if value not in allowed:
            raise ValueError(f"optimization must be one of {allowed}")
        return value


class MapInfo(GatewayResponseModel):
    name: str
    has_pcd: bool = False
    has_occupancy: bool = False
    has_octomap: bool = False
    navigation_ready: bool = False
    state: str | None = None
    is_active: bool = False
    size_mb: float | None = None
    patch_count: int = 0


class MapListResponse(GatewayResponseModel):
    schema_version: int = 1
    maps: list[MapInfo] = Field(default_factory=list)
    count: int = 0
    active: str = ""
    map_dir: str = ""
    ts: float = Field(default_factory=time.time)


class MapLifecycleResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool | None = None
    message: str | None = None
    name: str | None = None
    active: str | None = None
    old_name: str | None = None
    new_name: str | None = None
    path: str | None = None
    map_dir: str | None = None
    pcd: str | None = None
    octomap: str | None = None
    octomap_ok: bool | None = None
    octomap_message: str | None = None
    occupancy: str | None = None
    occupancy_ok: bool | None = None
    occupancy_message: str | None = None
    navigation_ready: bool | None = None
    size: str | None = None
    restored_size: int | None = None
    replaced_backups_kept: int | None = None
    replaced_backups_pruned: int | None = None
    note: str | None = None
    errors: list[Any] | None = None
    warnings: list[Any] | None = None
    slam_profile: str | None = None
    source: str | None = None
    map_save_source: str | None = None
    relocalization_supported: bool | None = None
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    dynamic_filter: Any = None
    map_optimization: Any = None
    map_optimization_ok: bool | None = None
    maps: list[Any] | None = None
    live_cloud_reset: bool | None = None
    ts: float = Field(default_factory=time.time)


class MapPointsResponse(GatewayResponseModel):
    schema_version: int = 1
    protocol_version: int | None = None
    count: int
    layout: Literal["flat_xyz", "xyz_rows"] = "xyz_rows"
    frame_id: str = GATEWAY_MAP_FRAME_ID
    epoch: int | None = None
    sequence: int | None = None
    stamp_s: float | None = None
    stream_kind: Literal["cloud", "map", "scan", "reset"] | None = None
    source: str = "unknown"
    name: str | None = None
    version_id: str | None = None
    map_pcd_sha256: str | None = None
    points: list[float] | list[tuple[float, float, float]] = Field(default_factory=list)
    bounds: dict[str, list[float]] | None = None
    ts: float = Field(default_factory=time.time)


__all__ = (
    "MapInfo",
    "MapLifecycleResponse",
    "MapListResponse",
    "MapNameRequest",
    "MapPointsResponse",
    "MapRenameRequest",
    "MapRequest",
    "MapSaveRequest",
)
