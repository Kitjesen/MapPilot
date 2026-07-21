"""Internal Gateway runtime request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field, field_validator

from gateway._schemas.common import (
    GatewayResponseModel,
    NavigationFrameMismatch,
    TeleopSummary,
)
from gateway._schemas.runtime_contracts import (
    RuntimeDataFlowStageSummary,
    RuntimeFrameLinkSummary,
    RuntimeFrameSummary,
)


class ReadinessModuleStatus(GatewayResponseModel):
    ok: bool
    detail: dict[str, Any] | None = None
    error: str | None = None


class RuntimeDataflowSubscribeRequest(GatewayResponseModel):
    selector: str = Field(description="Canonical runtime topic or short alias")
    transport: Literal["gateway_sse"] = "gateway_sse"
    max_rate_hz: float | None = Field(default=None, ge=0.0)


class RuntimeDataflowSubscribeResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.runtime_dataflow_subscription.v1"] = "lingtu.runtime_dataflow_subscription.v1"
    ok: bool
    ts: float
    read_only: bool = True
    endpoint_topic_required: bool = False
    ros2_topic_required: bool = False
    arbitrary_publish_supported: bool = False
    publishes: list[str] = Field(default_factory=list)
    selector: str
    topic: str | None = None
    transport: Literal["gateway_sse"] = "gateway_sse"
    stream_url: str = ""
    event_types: list[str] = Field(default_factory=list)
    stream_interfaces: list[dict[str, Any]] = Field(default_factory=list)
    blockers: list[str] = Field(default_factory=list)
    links: dict[str, str] = Field(default_factory=dict)


class RuntimeSwitchPlanRequest(GatewayResponseModel):
    current_profile: str | None = None
    target_profile: str = "explore"
    current_endpoint: str | None = None
    target_endpoint: str | None = None
    endpoint: str | None = None


class RuntimeSwitchValidationSummary(GatewayResponseModel):
    ok: bool
    blockers: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


class RuntimeSwitchPlanResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.runtime_switch_plan.v1"] = "lingtu.runtime_switch_plan.v1"
    ok: bool
    ts: float
    read_only: bool = True
    dry_run: bool = True
    motion: bool = False
    publishes: list[str] = Field(default_factory=list)
    lifecycle: str = "dry_run_preflight"
    inputs: dict[str, Any] = Field(default_factory=dict)
    from_: dict[str, Any] = Field(default_factory=dict, alias="from")
    to: dict[str, Any] = Field(default_factory=dict)
    changed: list[str] = Field(default_factory=list)
    current_validation: RuntimeSwitchValidationSummary
    target_validation: RuntimeSwitchValidationSummary
    product_mode_switch: dict[str, Any] | None = None
    blockers: list[str] = Field(default_factory=list)
    links: dict[str, str] = Field(default_factory=dict)
    error: str | None = None


ProductModeProfile = Literal[
    "teleop",
    "teleop_avoid",
    "map",
    "tracking",
    "nav",
    "inspection",
    "tare_explore",
]


class RuntimeSwitchRequest(GatewayResponseModel):
    current_profile: str | None = None
    target_profile: ProductModeProfile
    current_endpoint: str | None = None
    target_endpoint: str | None = None
    endpoint: str | None = "thunder_field"
    map_name: str | None = None
    relocalize: bool = True
    initial_pose: list[float] | None = None
    strategy: Literal["auto", "hot", "warm", "cold"] = "auto"
    execute: bool = False
    allow_restart: bool = False
    client_id: str = "app"
    request_id: str | None = None

    @field_validator("initial_pose")
    @classmethod
    def _validate_initial_pose(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        if len(value) != 3:
            raise ValueError("initial_pose must be [x, y, yaw]")
        return [float(item) for item in value]


class RuntimeSwitchResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.runtime_switch.v1"] = "lingtu.runtime_switch.v1"
    ok: bool
    ts: float
    accepted: bool = False
    status: str = "planned"
    read_only: bool = True
    dry_run: bool = True
    motion: bool = False
    lifecycle: str = "cold_restart"
    strategy: str = "auto"
    current_profile: str | None = None
    target_profile: str
    map_name: str | None = None
    relocalize: bool = True
    plan: dict[str, Any] = Field(default_factory=dict)
    product_mode_switch: dict[str, Any] | None = None
    effects: list[str] = Field(default_factory=list)
    command: list[str] = Field(default_factory=list)
    command_id: str | None = None
    pid: int | None = None
    log_path: str | None = None
    blockers: list[str] = Field(default_factory=list)
    links: dict[str, str] = Field(default_factory=dict)
    error: str | None = None


class ReadinessLocalizationFrameSummary(GatewayResponseModel):
    runtime_contract: str | None = None
    odometry_frame_id: str | None = "unknown"
    registered_cloud_frame_id: str | None = None
    map_cloud_frame_id: str | None = None
    odometry_expected_frame_ids: list[str] = Field(default_factory=list)
    registered_cloud_expected_frame_ids: list[str] = Field(default_factory=list)
    map_cloud_expected_frame_ids: list[str] = Field(default_factory=list)
    observed_topic_frame_ids: dict[str, str] = Field(default_factory=dict)
    missing_required_topic_frame_ids: list[str] = Field(default_factory=list)
    ok: bool | None = None
    mismatches: list[NavigationFrameMismatch] = Field(default_factory=list)


class ReadinessLocalizationRuntime(GatewayResponseModel):
    state: str | None = None
    ready: bool | None = None
    pose_fresh: bool | None = None
    pose_freshness: str | None = None
    algorithm_healthy: bool | None = None
    runtime_contract: str | None = None
    frames: ReadinessLocalizationFrameSummary = Field(default_factory=ReadinessLocalizationFrameSummary)
    topic_allowed_frame_ids: dict[str, list[str]] = Field(default_factory=dict)
    topic_default_frame_ids: dict[str, str] = Field(default_factory=dict)
    required_topic_frame_ids: list[str] = Field(default_factory=list)
    runtime_data_flow_topics: list[str] = Field(default_factory=list)
    runtime_data_flow_stage_algorithm_interfaces: dict[str, list[str]] = Field(default_factory=dict)
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None


class ReadinessNavigationRuntime(GatewayResponseModel):
    state: str | None = None
    can_accept_goal: bool | None = None
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    active_cmd_source: str | None = None
    error: str | None = None


class ReadinessRuntimeBoundary(GatewayResponseModel):
    ok: bool | None = None
    declared: bool | None = None
    profile: str | None = None
    endpoint: str | None = None
    data_source: str | None = None
    runtime_contract: str | None = None
    command_sink: str | None = None
    expected_command_sink: str | None = None
    frames: RuntimeFrameSummary = Field(default_factory=RuntimeFrameSummary)
    frame_links: dict[str, RuntimeFrameLinkSummary] = Field(default_factory=dict)
    topic_allowed_frame_ids: dict[str, list[str]] = Field(default_factory=dict)
    topic_default_frame_ids: dict[str, str] = Field(default_factory=dict)
    required_topic_frame_ids: list[str] = Field(default_factory=list)
    runtime_data_flow_topics: list[str] = Field(default_factory=list)
    resolved_runtime_data_flow: list[RuntimeDataFlowStageSummary] = Field(default_factory=list)
    runtime_data_flow_stage_algorithm_interfaces: dict[str, list[str]] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)


class ReadinessRuntimeModeSummary(GatewayResponseModel):
    data_ready: bool | None = None
    motion_ready: bool | None = None
    non_motion_safe: bool | None = None
    active_cmd_source: str | None = None
    mission_state: str | None = None
    data_blockers: list[str] = Field(default_factory=list)


class ReadinessRuntimeSummary(GatewayResponseModel):
    localization: ReadinessLocalizationRuntime | None = None
    navigation: ReadinessNavigationRuntime | None = None
    boundary: ReadinessRuntimeBoundary | None = None
    safety: dict[str, Any] = Field(default_factory=dict)
    calibration: dict[str, Any] = Field(default_factory=dict)
    summary: ReadinessRuntimeModeSummary | None = None


class ReadinessResponse(GatewayResponseModel):
    schema_version: int
    status: str
    ready: bool
    data_ready: bool
    motion_ready: bool
    non_motion_safe: bool
    modules: dict[str, ReadinessModuleStatus]
    module_count: int
    failed_modules: list[str]
    reasons: list[str]
    advisories: list[str] = Field(default_factory=list)
    runtime: ReadinessRuntimeSummary = Field(default_factory=ReadinessRuntimeSummary)
    ts: float


class LivenessResponse(GatewayResponseModel):
    status: str
    ts: float
    details_url: str = "/api/v1/health?details=true"
    gateway: dict[str, Any] = Field(default_factory=dict)
    sensors: dict[str, Any] = Field(default_factory=dict)


class DevicesResponse(GatewayResponseModel):
    devices: list[Any] = Field(default_factory=list)
    manager: str
    spec_count: int = 0
    opened_count: int = 0
    error: str | None = None


class HealthResponse(GatewayResponseModel):
    status: str
    modules_ok: int = 0
    modules_fail: int = 0
    gateway: dict[str, Any] = Field(default_factory=dict)
    teleop: TeleopSummary
    sensors: dict[str, Any] = Field(default_factory=dict)
    slam_hz: float = 0.0
    map_points: int = 0
    has_odom: bool = False
    modules: dict[str, str] = Field(default_factory=dict)
    brainstem: dict[str, Any] = Field(default_factory=dict)


__all__ = (
    "DevicesResponse",
    "HealthResponse",
    "LivenessResponse",
    "ProductModeProfile",
    "ReadinessLocalizationFrameSummary",
    "ReadinessLocalizationRuntime",
    "ReadinessModuleStatus",
    "ReadinessNavigationRuntime",
    "ReadinessResponse",
    "ReadinessRuntimeBoundary",
    "ReadinessRuntimeModeSummary",
    "ReadinessRuntimeSummary",
    "RuntimeDataflowSubscribeRequest",
    "RuntimeDataflowSubscribeResponse",
    "RuntimeSwitchPlanRequest",
    "RuntimeSwitchPlanResponse",
    "RuntimeSwitchRequest",
    "RuntimeSwitchResponse",
    "RuntimeSwitchValidationSummary",
)
