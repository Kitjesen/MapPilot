"""Internal Gateway session request and response models."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


class SessionStartRequest(BaseModel):
    mode: str | None = Field(default=None, max_length=32)
    map_name: str | None = Field(default=None, max_length=128)
    map: str | None = Field(default=None, max_length=128)
    profile: str | None = Field(default=None, max_length=64)
    product_profile: str | None = Field(default=None, max_length=64)
    product_session: str | None = Field(default=None, max_length=64)
    slam_profile: str | None = Field(default=None, max_length=64)
    slam_backend: str | None = Field(default=None, max_length=64)


class SessionResponse(GatewayResponseModel):
    mode: str
    product_session: str = "idle"
    product_profile: str | None = None
    slam_profile: str = "stopped"
    localization_backend: str | None = None
    health_source: str | None = None
    active_map: str | None = None
    saved_active_map: str | None = None
    map_has_pcd: bool = False
    map_has_octomap: bool = False
    since: float | None = None
    pending: bool = False
    error: str = ""
    icp_quality: float | None = None
    localizer_ready: bool = False
    localizer_algorithm_healthy: bool = False
    pose_fresh: bool | None = None
    pose_freshness: str = "unknown"
    map_state: str | None = None
    map_save_supported: bool = False
    map_save_source: str | None = None
    relocalization_supported: bool = True
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    relocalization_state: str | None = None
    recovery_signal: str | None = None
    recovery_action: str | None = None
    can_start_mapping: bool = False
    can_start_navigating: bool = False
    can_start_exploring: bool = False
    exploration_blockers: list[str] = Field(default_factory=list)
    safety_clear: bool = True
    safety: dict[str, Any] | None = None
    can_end: bool = False
    explorer_backend: Literal["none", "frontier", "tare"] | str = "none"
    explorer_available: bool = False
    explorer_unavailable_reason: str | None = None
    explorer_required_profile: str | None = None


class SessionTransitionResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool
    session: SessionResponse | None = None
    message: str | None = None
    detail: Any = None
    ts: float = Field(default_factory=time.time)


__all__ = (
    "SessionResponse",
    "SessionStartRequest",
    "SessionTransitionResponse",
)
