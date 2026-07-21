"""Internal Gateway inspection request and response models."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


class InspectionAcceptanceRequest(BaseModel):
    mode: Literal["non_motion", "simulation", "field"] = "simulation"
    points: list[str] = Field(default_factory=list)
    tag: str | None = Field(default=None, max_length=128)
    map_dir: str | None = None
    require_octomap: bool = False
    require_occupancy: bool = False
    expected_data_source: str | None = None
    expected_source_profile: str | None = None
    expected_frame_id: str | None = None
    client_id: str = Field(default="unknown", max_length=128)


class ProductFieldCheckRequest(BaseModel):
    mode: Literal["non_motion", "simulation", "field"] = "simulation"
    map_dir: str | None = None
    require_octomap: bool = False
    require_occupancy: bool = False
    expected_data_source: str | None = None
    expected_source_profile: str | None = None
    expected_frame_id: str | None = None


class ProductFieldCheckResponse(GatewayResponseModel):
    schema_version: str
    ok: bool
    mode: str
    summary: str
    map: dict[str, Any] = Field(default_factory=dict)
    runtime: dict[str, Any] = Field(default_factory=dict)
    stage_evidence: dict[str, Any] = Field(default_factory=dict)
    navigation: dict[str, Any] = Field(default_factory=dict)
    evidence: dict[str, Any] = Field(default_factory=dict)
    algorithm: dict[str, Any] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    commands: dict[str, str] = Field(default_factory=dict)


class InspectionAcceptanceTargetResult(GatewayResponseModel):
    name: str
    status: str
    ok: bool
    target_type: str | None = None
    source: str | None = None
    location_name: str | None = None
    preview_feasible: bool = False
    preview_count: int | None = None
    planner: str | None = None
    distance_m: float | None = None
    non_motion: bool = True
    command_published: bool = False
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None


class InspectionAcceptanceResponse(GatewayResponseModel):
    schema_version: str
    ok: bool
    summary: str
    gateway_url: str | None = None
    mode: str = "field"
    field_ready: bool = False
    field_summary: str = "UNKNOWN"
    target_count: int = 0
    pass_count: int = 0
    fail_count: int = 0
    locations_count: int | None = None
    motion_safety: dict[str, Any] = Field(default_factory=dict)
    targets: list[InspectionAcceptanceTargetResult] = Field(default_factory=list)
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    evidence: dict[str, Any] = Field(default_factory=dict)
    commands: dict[str, str] = Field(default_factory=dict)
    ts: float


InspectionFailurePolicy = Literal["stop", "retry", "skip"]


class InspectionRoutePointRequest(BaseModel):
    id: str = Field(min_length=1, max_length=128)
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    tolerance: float = Field(default=0.35, gt=0.0, le=5.0)
    dwell: float = Field(default=0.0, ge=0.0, le=3600.0)
    action: str = Field(default="", max_length=128)
    enabled: bool = True


_UINT64_MAX = (1 << 64) - 1


class InspectionRouteRequest(BaseModel):
    id: str = Field(min_length=1, max_length=128)
    name: str | None = Field(default=None, max_length=128)
    map_id: str = Field(min_length=1, max_length=128)
    map_version: int | None = Field(default=None, ge=0)
    revision: int = Field(ge=1, le=_UINT64_MAX)
    points: list[InspectionRoutePointRequest] = Field(min_length=1)
    loop_count: int = Field(default=1, ge=1)
    failure_policy: InspectionFailurePolicy = "stop"
    max_retries: int = Field(default=0, ge=0)


class InspectionStartRequest(BaseModel):
    map_id: str | None = Field(default=None, max_length=128)
    revision: int = Field(default=0, ge=0, le=_UINT64_MAX)
    request_id: str | None = Field(default=None, max_length=128)


class InspectionRunControlRequest(BaseModel):
    map_id: str | None = Field(default=None, max_length=128)
    reason: str = Field(default="operator", max_length=128)
    request_id: str | None = Field(default=None, max_length=128)


class InspectionRouteRecord(GatewayResponseModel):
    id: str
    name: str | None = None
    map_id: str | None = None
    map_version: int | None = None
    revision: int | None = None
    points: list[dict[str, Any]] = Field(default_factory=list)
    loop_count: int | None = None
    failure_policy: str | None = None
    max_retries: int | None = None


class InspectionRouteResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.v1"] = "lingtu.inspection.v1"
    ok: bool = True
    route: InspectionRouteRecord
    ts: float = Field(default_factory=time.time)


class InspectionRouteListResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.v1"] = "lingtu.inspection.v1"
    ok: bool = True
    map_id: str
    routes: list[InspectionRouteRecord] = Field(default_factory=list)
    count: int
    ts: float = Field(default_factory=time.time)


class InspectionCommandResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.v1"] = "lingtu.inspection.v1"
    ok: bool = True
    accepted: bool = True
    action: Literal["start", "pause", "resume", "cancel", "delete"]
    route_id: str | None = None
    map_id: str | None = None
    revision: int | None = None
    request_id: str | None = None
    ts: float = Field(default_factory=time.time)


class InspectionStatusResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.v1"] = "lingtu.inspection.v1"
    ok: bool = True
    status: dict[str, Any] = Field(default_factory=dict)
    ts: float = Field(default_factory=time.time)


__all__ = (
    "InspectionAcceptanceRequest",
    "InspectionAcceptanceResponse",
    "InspectionAcceptanceTargetResult",
    "InspectionCommandResponse",
    "InspectionFailurePolicy",
    "InspectionRouteListResponse",
    "InspectionRoutePointRequest",
    "InspectionRouteRecord",
    "InspectionRouteRequest",
    "InspectionRouteResponse",
    "InspectionRunControlRequest",
    "InspectionStartRequest",
    "InspectionStatusResponse",
    "ProductFieldCheckRequest",
    "ProductFieldCheckResponse",
)
