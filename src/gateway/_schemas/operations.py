"""Internal Gateway operations request and response models."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


class SlamSwitchRequest(BaseModel):
    profile: str | None = Field(default=None, max_length=64)


class SlamRelocalizeRequest(BaseModel):
    map_name: str | None = Field(default=None, max_length=128)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class BagStartRequest(BaseModel):
    duration: int = Field(default=600, ge=1, le=86400)
    prefix: str = Field(default="web", max_length=40)


class ExplorationCommandResponse(GatewayResponseModel):
    status: Any = None


class ExplorationStatusResponse(GatewayResponseModel):
    available: bool
    backend: Literal["none", "frontier", "tare"] | str = "none"
    exploring: bool = False
    frontier_count: int = 0
    can_start: bool = False
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    navigation: dict[str, Any] = Field(default_factory=dict)
    reason: str | None = None
    required_profile: str | None = None
    supported_profiles: list[str] | None = None
    action: str | None = None
    tare: dict[str, Any] | None = None
    supervisor: dict[str, Any] | None = None


class SlamStatusResponse(GatewayResponseModel):
    mode: str
    native_mode: str | None = None
    services: dict[str, str] = Field(default_factory=dict)
    service_details: dict[str, dict[str, Any]] = Field(default_factory=dict)
    service_groups: dict[str, list[str]] = Field(default_factory=dict)
    service_metadata: dict[str, dict[str, Any]] = Field(default_factory=dict)
    product_runtime: str = "native_dds"
    ros2_required: bool = False
    manual_systemctl_required: bool = False
    control_entrypoint: str = "lingtu svc restart slam"


class ServiceStatusResponse(GatewayResponseModel):
    schema_version: int = 1
    services: dict[str, str] = Field(default_factory=dict)
    service_details: dict[str, dict[str, Any]] = Field(default_factory=dict)
    readiness: dict[str, Any] = Field(default_factory=dict)
    field_readiness: dict[str, Any] = Field(default_factory=dict)
    service_groups: dict[str, list[str]] = Field(default_factory=dict)
    service_metadata: dict[str, dict[str, Any]] = Field(default_factory=dict)
    product_runtime: str = "native_dds"
    control_entrypoint: str = "lingtu svc status"


class SlamOperationResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool
    profile: str | None = None
    message: str | None = None
    quality: float | None = None
    details: dict[str, Any] | None = None
    ts: float = Field(default_factory=time.time)


class BagOperationResponse(GatewayResponseModel):
    status: str | None = None
    path: str | None = None
    pid: int | None = None
    duration: int | None = None
    prefix: str | None = None
    error: str | None = None
    detail: Any = None


class BagStatusResponse(GatewayResponseModel):
    recording: bool
    path: str | None = None
    duration_s: float = 0.0
    size_bytes: int = 0
    pid: int | None = None
    exit_code: int | None = None
    disk_free: int = 0
    disk_total: int = 0


class Go2RTCStatusResponse(GatewayResponseModel):
    available: bool
    reason: str | None = None
    status: int | None = None
    streams: list[str] = Field(default_factory=list)


__all__ = (
    "BagOperationResponse",
    "BagStartRequest",
    "BagStatusResponse",
    "ExplorationCommandResponse",
    "ExplorationStatusResponse",
    "Go2RTCStatusResponse",
    "ServiceStatusResponse",
    "SlamOperationResponse",
    "SlamRelocalizeRequest",
    "SlamStatusResponse",
    "SlamSwitchRequest",
)
