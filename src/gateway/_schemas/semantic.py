"""Internal Gateway semantic request and response models."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator, model_validator

from gateway._schemas.common import (
    GATEWAY_MAP_FRAME_ID,
    GatewayResponseModel,
)


class TemporalSemanticRequest(BaseModel):
    embedding: list[float] | None = None
    since: str | None = Field(default=None, max_length=64)
    top_k: int = Field(default=10, ge=1, le=1000)
    label: str | None = Field(default=None, max_length=128)


class SceneGraphObject(GatewayResponseModel):
    id: str | None = None
    label: str = ""
    x: float | None = None
    y: float | None = None
    z: float | None = None
    confidence: float | None = None
    distance: float | None = None
    bbox: Any = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphRelation(GatewayResponseModel):
    source: str | None = None
    target: str | None = None
    relation: str | None = None
    confidence: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphRegion(GatewayResponseModel):
    id: str | None = None
    name: str | None = None
    label: str | None = None
    x: float | None = None
    y: float | None = None
    z: float | None = None
    polygon: Any = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphResponse(GatewayResponseModel):
    schema_version: int = 1
    frame_id: str = GATEWAY_MAP_FRAME_ID
    ts: float | None = None
    objects: list[SceneGraphObject] = Field(default_factory=list)
    relations: list[SceneGraphRelation] = Field(default_factory=list)
    regions: list[SceneGraphRegion] = Field(default_factory=list)
    count: int = 0
    scene_graph: Any = None


class LocationEntry(GatewayResponseModel):
    name: str
    x: float
    y: float
    z: float
    yaw: float | None = None
    tags: list[str] = Field(default_factory=list)
    source: str | None = None
    ts: float | None = None
    map_id: str | None = None
    map_version: int | None = None
    frame_id: str = GATEWAY_MAP_FRAME_ID
    metadata: dict[str, Any] = Field(default_factory=dict)


class LocationUpsertRequest(BaseModel):
    name: str = Field(min_length=1, max_length=128)
    x: float | None = None
    y: float | None = None
    z: float = 0.0
    yaw: float | None = None
    tags: list[str] = Field(default_factory=list)
    source: str = Field(default="app", max_length=64)
    metadata: dict[str, Any] = Field(default_factory=dict)
    use_current_pose: bool = False
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z", "yaw")
    @classmethod
    def finite_optional(cls, v: float | None) -> float | None:
        import math

        if v is not None and not math.isfinite(v):
            raise ValueError("must be finite")
        return v

    @model_validator(mode="after")
    def require_coordinates_or_current_pose(self) -> LocationUpsertRequest:
        if not self.use_current_pose and (self.x is None or self.y is None):
            raise ValueError("x and y are required unless use_current_pose is true")
        return self


class LocationsResponse(GatewayResponseModel):
    schema_version: int = 1
    locations: list[LocationEntry] = Field(default_factory=list)
    count: int = 0
    frame_id: str = GATEWAY_MAP_FRAME_ID
    ts: float | None = None
    source: str = "tagged_locations"


class LocationOperationResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    status: Literal["saved", "deleted", "not_found", "unavailable", "invalid", "error"]
    action: Literal["create", "update", "delete"]
    location: LocationEntry | None = None
    locations: LocationsResponse
    message: str | None = None
    error: str | None = None
    request_id: str | None = None
    client_id: str = "unknown"
    ts: float = Field(default_factory=time.time)


class TemporalMemoryResponse(GatewayResponseModel):
    observations: list[Any] = Field(default_factory=list)
    count: int = 0


__all__ = (
    "LocationEntry",
    "LocationOperationResponse",
    "LocationUpsertRequest",
    "LocationsResponse",
    "SceneGraphObject",
    "SceneGraphRegion",
    "SceneGraphRelation",
    "SceneGraphResponse",
    "TemporalMemoryResponse",
    "TemporalSemanticRequest",
)
