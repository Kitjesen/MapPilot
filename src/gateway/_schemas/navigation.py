"""Internal Gateway navigation request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator, model_validator

from gateway._schemas.common import (
    GATEWAY_BODY_FRAME_ID,
    GATEWAY_MAP_FRAME_ID,
    GatewayResponseModel,
    MapFrameId,
)

GoalSource = Literal[
    "coordinate",
    "map_click",
    "saved_location",
    "semantic",
    "frontier",
    "api",
]


GoalTargetType = Literal[
    "coordinate",
    "map_point",
    "saved_location",
    "semantic_target",
    "frontier",
]


class GoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID
    instruction: str | None = None
    source: GoalSource = "coordinate"
    target_type: GoalTargetType = "coordinate"
    label: str | None = Field(default=None, max_length=128)
    acceptance_radius_m: float | None = Field(default=None, gt=0, le=20)
    max_speed_mps: float | None = Field(default=None, gt=0, le=5)
    metadata: dict[str, Any] = Field(default_factory=dict)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z", "yaw")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class ClickNavRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID
    source: GoalSource = "map_click"
    target_type: GoalTargetType = "map_point"
    label: str | None = Field(default=None, max_length=128)
    acceptance_radius_m: float | None = Field(default=None, gt=0, le=20)
    max_speed_mps: float | None = Field(default=None, gt=0, le=5)
    metadata: dict[str, Any] = Field(default_factory=dict)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class PlanPreviewRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID
    client_id: str = Field(default="unknown", max_length=128)
    planner_constraints: dict[str, Any] = Field(default_factory=dict)

    @field_validator("x", "y", "z")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class GoalCandidateRequest(BaseModel):
    x: float | None = None
    y: float | None = None
    z: float = 0.0
    yaw: float | None = None
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID
    source: GoalSource = "coordinate"
    target_type: GoalTargetType = "coordinate"
    label: str | None = Field(default=None, max_length=128)
    location_name: str | None = Field(default=None, min_length=1, max_length=128)
    acceptance_radius_m: float | None = Field(default=None, gt=0, le=20)
    max_speed_mps: float | None = Field(default=None, gt=0, le=5)
    metadata: dict[str, Any] = Field(default_factory=dict)
    preview: bool = True
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z", "yaw")
    @classmethod
    def finite_optional(cls, v: float | None) -> float | None:
        import math

        if v is not None and not math.isfinite(v):
            raise ValueError("must be finite")
        return v

    @model_validator(mode="after")
    def require_coordinates_or_location(self) -> GoalCandidateRequest:
        if self.location_name is None and (self.x is None or self.y is None):
            raise ValueError("x and y are required unless location_name is provided")
        return self


class ConstructedGoalTarget(GatewayResponseModel):
    schema_version: int = 1
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    frame_id: str = GATEWAY_MAP_FRAME_ID
    source: str = "coordinate"
    target_type: str = "coordinate"
    label: str | None = None
    location_name: str | None = None
    acceptance_radius_m: float | None = None
    max_speed_mps: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    ts: float | None = None


class PathPoint(GatewayResponseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    frame_id: str | None = None
    ts: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class RobotPoseSummary(GatewayResponseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    vx: float | None = None
    vy: float | None = None
    wz: float | None = None
    frame_id: str | None = None
    ts: float | None = None


class PathResponse(GatewayResponseModel):
    schema_version: int = 1
    path: list[PathPoint] = Field(default_factory=list)
    robot: RobotPoseSummary | None = None
    count: int = 0
    frame_id: str = GATEWAY_MAP_FRAME_ID
    ts: float | None = None
    source: str = "gateway_cache"


class DdsTwistSnapshot(GatewayResponseModel):
    frame_id: str = GATEWAY_BODY_FRAME_ID
    linear: dict[str, float] = Field(default_factory=dict)
    angular: dict[str, float] = Field(default_factory=dict)
    active_source: str = "none"
    ts: float | None = None


class NavigationDdsSnapshotResponse(GatewayResponseModel):
    schema_version: str = "lingtu.navigation.dds_snapshot.v1"
    global_path: PathResponse
    local_path: PathResponse
    cmd_vel: DdsTwistSnapshot | None = None
    nav_endpoint: dict[str, Any] | None = None
    traversability_endpoint: dict[str, Any] | None = None
    navigation: dict[str, Any] = Field(default_factory=dict)
    ts: float
    source: str = "gateway_navigation_cache"


class PlanPreviewResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    feasible: bool = False
    frame_id: str = GATEWAY_MAP_FRAME_ID
    start: PathPoint | None = None
    goal: PathPoint
    adjusted_goal: PathPoint | None = None
    path: list[PathPoint] = Field(default_factory=list)
    count: int = 0
    distance_m: float | None = None
    plan_ms: float | None = None
    planner: str | None = None
    selected_planner: str | None = None
    plan_safety_policy: str | None = None
    path_safety: dict[str, Any] | None = None
    fallback_reason: str = ""
    rejected_plans: list[dict[str, Any]] = Field(default_factory=list)
    snap_diagnostics: dict[str, Any] | None = None
    source: str = "navigation_preview"
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None
    ts: float


class GoalCandidateResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    target: ConstructedGoalTarget | None = None
    preview: PlanPreviewResponse | None = None
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None
    ts: float


__all__ = (
    "ClickNavRequest",
    "ConstructedGoalTarget",
    "DdsTwistSnapshot",
    "GoalCandidateRequest",
    "GoalCandidateResponse",
    "GoalRequest",
    "GoalSource",
    "GoalTargetType",
    "NavigationDdsSnapshotResponse",
    "PathPoint",
    "PathResponse",
    "PlanPreviewRequest",
    "PlanPreviewResponse",
    "RobotPoseSummary",
)
