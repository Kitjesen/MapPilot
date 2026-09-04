"""FastAPI request/response schemas for app/web Gateway contracts."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from lingtu.products import ProductName
from runtime.runtime_interface import body_frame_id, map_frame_id

GATEWAY_MAP_FRAME_ID = map_frame_id()
GATEWAY_BODY_FRAME_ID = body_frame_id()
MapFrameId = Literal["map"]


class GatewayResponseModel(BaseModel):
    """Base model that documents known fields without stripping future additions."""

    model_config = ConfigDict(extra="allow")


class CommandReceipt(GatewayResponseModel):
    name: str
    task_id: str | None = None
    request_id: str | None = None
    native_request_id: str | None = None
    client_id: str
    accepted: bool
    replay: bool
    ts: float


class GatewayCommandErrorDetail(GatewayResponseModel):
    reason_code: str
    reason: str | None = None
    source: str | None = None
    path: str | None = None
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    safety: dict[str, Any] | None = None
    preview: dict[str, Any] | None = None
    lease: dict[str, Any] | None = None
    state: str | None = None
    has_odometry: bool | None = None
    session_mode: str | None = None
    localization: dict[str, Any] | None = None
    error: str | None = None


class GatewayErrorResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: Literal[False] = False
    error: str
    message: str | None = None
    detail: dict[str, Any] | GatewayCommandErrorDetail | None = None
    command: CommandReceipt | None = None


class RoutecheckLatestResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    artifacts_root: str
    count: int = 0
    artifact_dir: str | None = None
    summary_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    non_motion: bool | None = None
    simulation_only: bool | None = None
    real_robot_motion: bool | None = None
    cmd_vel_sent_to_hardware: bool | None = None
    gateway_used: bool | None = None
    driver_used: bool | None = None
    published: dict[str, Any] | None = None
    latest: dict[str, Any] | None = None
    reason: str | None = None
    ts: float


class RealRuntimeEvidenceLatestResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    artifacts_root: str
    count: int = 0
    artifact_dir: str | None = None
    report_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    max_age_s: float
    runtime_contract: str | None = None
    runtime_evidence_ok: bool = False
    simulation_only: bool | None = None
    real_robot_motion: bool | None = None
    cmd_vel_sent_to_hardware: bool | None = None
    blockers: list[str] = Field(default_factory=list)
    reason: str | None = None
    ts: float


class RuntimeContractResponse(GatewayResponseModel):
    schema_version: int = 1
    source: str
    manifest: RuntimeContractManifest
    ts: float


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
    task_id: str | None = Field(default=None, max_length=128)
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
    task_id: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class DirectedExplorationTargetRequest(BaseModel):
    """Explicit native TARE direction intent; never a navigation goal."""

    model_config = ConfigDict(extra="forbid")

    x: float
    y: float
    ttl_s: float = Field(default=30.0, gt=0.0, le=120.0)
    reason: str = Field(default="operator_directed_explore", min_length=1, max_length=256)
    request_id: str | None = Field(default=None, max_length=128)

    @field_validator("x", "y", "ttl_s")
    @classmethod
    def finite(cls, value: float) -> float:
        """Reject non-finite operator coordinates and TTLs."""
        import math

        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value

    @field_validator("reason")
    @classmethod
    def normalized_reason(cls, value: str) -> str:
        """Keep the native audit reason non-empty after whitespace cleanup."""
        reason = value.strip()
        if not reason:
            raise ValueError("must not be blank")
        return reason


class DirectedExplorationClearRequest(BaseModel):
    """Explicit native TARE direction-intent clear request."""

    model_config = ConfigDict(extra="forbid")

    reason: str = Field(default="operator_clear_directed_explore", min_length=1, max_length=256)
    request_id: str | None = Field(default=None, max_length=128)

    @field_validator("reason")
    @classmethod
    def normalized_reason(cls, value: str) -> str:
        """Keep the native audit reason non-empty after whitespace cleanup."""
        reason = value.strip()
        if not reason:
            raise ValueError("must not be blank")
        return reason


class ExplorationStartRequest(BaseModel):
    """Idempotent request to start one finite Explore execution."""

    model_config = ConfigDict(extra="forbid")

    request_id: str | None = Field(default=None, max_length=128)


class ExplorationRunCommandRequest(BaseModel):
    """One idempotent pause, resume, or finish request for an Explore run."""

    model_config = ConfigDict(extra="forbid")

    request_id: str | None = Field(default=None, max_length=128)
    reason: str | None = Field(default=None, min_length=1, max_length=256)


class PlanPreviewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    x: float
    y: float
    z: float = 0.0
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID

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


class InstructionRequest(BaseModel):
    text: str = Field(min_length=1, max_length=1024)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class SafetyEstopRequest(BaseModel):
    """Emergency-stop activation request."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str | None = Field(default=None, max_length=128)


class VisualServoRequest(BaseModel):
    mode: str = Field(default="find", max_length=16)
    target: str | None = Field(default=None, max_length=256)
    target_id: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @model_validator(mode="after")
    def validate_command(self) -> VisualServoRequest:
        self.mode = self.mode.strip().lower()
        if self.mode not in {"find", "follow", "stop"}:
            raise ValueError("mode must be find|follow|stop")
        if self.target is not None:
            self.target = self.target.strip()
        if self.target_id is not None:
            self.target_id = self.target_id.strip()
        if self.target_id and self.mode != "follow":
            raise ValueError("target_id is only valid for follow")
        if self.mode == "find" and not self.target:
            raise ValueError("target is required for find")
        if self.mode == "follow" and not (self.target or self.target_id):
            raise ValueError("target or target_id is required for follow")
        return self


class StopRequest(BaseModel):
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class CancelRequest(BaseModel):
    reason: str = Field(default="client_cancel", max_length=256)
    task_id: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class NavigationTaskPauseRequest(BaseModel):
    """Request a stop-confirmed pause for one stable navigation task."""

    reason: str = Field(default="operator_pause", min_length=1, max_length=256)
    task_id: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class NavigationTaskResumeRequest(BaseModel):
    """Request continuation of the same paused navigation task."""

    reason: str = Field(default="operator_resume", min_length=1, max_length=256)
    task_id: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class ModeRequest(BaseModel):
    mode: str
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("mode")
    @classmethod
    def valid_mode(cls, v: str) -> str:
        if v not in ("manual", "autonomous", "estop"):
            raise ValueError(f"mode must be manual|autonomous|estop, got {v!r}")
        return v


class LeaseRequest(BaseModel):
    action: str
    client_id: str = Field(default="unknown", max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    ttl: float = Field(default=30.0, gt=0, le=3600)

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        if v not in ("acquire", "release", "renew"):
            raise ValueError(f"action must be acquire|release|renew, got {v!r}")
        return v


class MapRenameRequest(BaseModel):
    old_name: str | None = Field(default=None, max_length=128)
    new_name: str | None = Field(default=None, max_length=128)


class MapSaveRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(
        default=None,
        max_length=128,
        pattern=r"^[A-Za-z0-9_][A-Za-z0-9_-]*$",
    )


class TemporalSemanticRequest(BaseModel):
    embedding: list[float] | None = None
    since: str | None = Field(default=None, max_length=64)
    top_k: int = Field(default=10, ge=1, le=1000)
    label: str | None = Field(default=None, max_length=128)


class LocalizationInitialPose(BaseModel):
    """Caller-provided planar seed in the active map frame."""

    model_config = ConfigDict(extra="forbid")

    x: float
    y: float
    yaw: float

    @field_validator("x", "y", "yaw")
    @classmethod
    def finite(cls, value: float) -> float:
        import math

        if not math.isfinite(value):
            raise ValueError("must be finite")
        return value


class LocalizationRelocalizationRequest(BaseModel):
    """Request seeded or global localization on the Product-selected map."""

    model_config = ConfigDict(extra="forbid")

    map_name: str = Field(min_length=1, max_length=128)
    mode: Literal["seeded", "global"]
    initial_pose: LocalizationInitialPose | None = None
    request_id: str | None = Field(
        default=None,
        max_length=128,
        pattern=r"^[A-Za-z0-9_][A-Za-z0-9_-]*$",
    )

    @model_validator(mode="after")
    def validate_mode_payload(self) -> LocalizationRelocalizationRequest:
        if self.mode == "seeded" and self.initial_pose is None:
            raise ValueError("initial_pose is required when mode=seeded")
        if self.mode == "global" and self.initial_pose is not None:
            raise ValueError("initial_pose is not allowed when mode=global")
        return self


class LocalizationMapTrackingRequest(BaseModel):
    """Request continuous correction against the Product-selected map."""

    model_config = ConfigDict(extra="forbid")

    map_name: str = Field(min_length=1, max_length=128)
    request_id: str | None = Field(
        default=None,
        max_length=128,
        pattern=r"^[A-Za-z0-9_][A-Za-z0-9_-]*$",
    )


class RecordingStartRequest(BaseModel):
    duration: int = Field(default=600, ge=1, le=86400)
    prefix: str = Field(
        default="web",
        min_length=1,
        max_length=40,
        pattern=r"^[A-Za-z0-9_-]+$",
    )
    # These are the only operator-configurable capture choices. Topic lists,
    # DDS transport settings, and device paths remain native-owned.
    capture_profile: Literal["sensors", "evidence"] = "sensors"
    task_id: str | None = None
    camera: bool = False
    minimum_free_gib: int = Field(default=5, ge=1, le=100)

    @model_validator(mode="after")
    def bind_evidence_to_inspection_task(self) -> RecordingStartRequest:
        if self.task_id is not None:
            self.task_id = self.task_id.strip()
            if not self.task_id:
                raise ValueError("task_id must not be blank")
            if "\x00" in self.task_id or len(self.task_id.encode("utf-8")) > 256:
                raise ValueError("task_id must be at most 256 UTF-8 bytes and contain no NUL")
        if self.capture_profile == "evidence" and self.task_id is None:
            raise ValueError("task_id is required for evidence recording")
        if self.capture_profile == "sensors" and self.task_id is not None:
            raise ValueError("task_id is only valid for evidence recording")
        return self


class ServerInfo(GatewayResponseModel):
    api_version: str
    time: float


class EndpointSpec(GatewayResponseModel):
    method: str
    path: str
    operation_id: str | None = None
    request_schema: str | None = None
    response_schema: str | None = None
    response_content_types: list[str] = Field(default_factory=list)
    status_codes: list[str] = Field(default_factory=list)


class SSEEventEnvelope(GatewayResponseModel):
    schema_version: int = 1
    event_id: int | None = None
    type: str
    ts: float
    data: Any = None


class TeleopSummary(GatewayResponseModel):
    active: bool
    clients: int


class ReadinessModuleStatus(GatewayResponseModel):
    ok: bool
    detail: dict[str, Any] | None = None
    error: str | None = None


class NavigationFrameMismatch(GatewayResponseModel):
    source: str
    expected_frame: str
    received_frame: str


class RuntimeDataFlowStageSummary(GatewayResponseModel):
    name: str
    inputs: list[str] = Field(default_factory=list)
    outputs: list[str] = Field(default_factory=list)
    owner: str
    frame_role: str
    map_dependency: str


class RuntimeFrameSummary(GatewayResponseModel):
    map: str = "map"
    odom: str = "odom"
    body: str = "body"
    model_base: str = "base_link"
    lidar: str = "lidar_link"
    real_lidar: str = "livox_frame"
    camera: str = "camera_link"
    simulator_world: str = "world"
    axis_convention: str = "x_forward_y_left_z_up"
    body_aliases: list[str] = Field(default_factory=lambda: ["base_link"])
    lidar_aliases: list[str] = Field(default_factory=lambda: ["livox_frame"])


class RuntimeFrameLinkSummary(GatewayResponseModel):
    parent: str
    child: str
    required: bool = True


class RuntimeTransform3D(GatewayResponseModel):
    parent: str
    child: str
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class RuntimeMessageFormatSummary(GatewayResponseModel):
    name: str
    frame_role: str
    required_fields: list[str] = Field(default_factory=list)
    note: str = ""


class RuntimeArtifactFormatSummary(GatewayResponseModel):
    name: str
    path: str
    artifact_type: str
    frame_role: str
    required_fields: list[str] = Field(default_factory=list)
    required_metadata: list[str] = Field(default_factory=list)
    note: str = ""


class RuntimeDataSourceSummary(GatewayResponseModel):
    name: str
    provider: str
    owns: list[str] = Field(default_factory=list)
    normalized_outputs: list[str] = Field(default_factory=list)
    command_sink: str
    source_outputs: list[str] = Field(default_factory=list)
    algorithm_entry_outputs: list[str] = Field(default_factory=list)
    algorithm_context_outputs: list[str] = Field(default_factory=list)
    lidar_extrinsic_profile: str | None = None
    slam_source: str = "not_declared"
    localization_source: str = "not_declared"
    mapping_source: str = "not_declared"


class RuntimeAlgorithmInterfaceSummary(GatewayResponseModel):
    name: str
    inputs: list[str] = Field(default_factory=list)
    outputs: list[str] = Field(default_factory=list)
    owner: str
    map_dependency: str


class RuntimeAdapterAliasSummary(GatewayResponseModel):
    source: str
    target: str
    msg_format: str
    scope: str = "adapter_only"
    note: str = ""


class RuntimeProductDataSourceBinding(GatewayResponseModel):
    product: str
    data_source: str
    mode: str
    note: str = ""


class RuntimeContractManifest(GatewayResponseModel):
    schema_version: str
    frames: RuntimeFrameSummary
    topics: dict[str, str] = Field(default_factory=dict)
    core_required_topics: list[str] = Field(default_factory=list)
    frame_links: dict[str, RuntimeFrameLinkSummary] = Field(default_factory=dict)
    lidar_extrinsics: dict[str, RuntimeTransform3D] = Field(default_factory=dict)
    runtime_data_flow: list[RuntimeDataFlowStageSummary] = Field(default_factory=list)
    resolved_runtime_data_flow: dict[str, list[RuntimeDataFlowStageSummary]] = Field(default_factory=dict)
    message_formats: dict[str, RuntimeMessageFormatSummary] = Field(default_factory=dict)
    topic_formats: dict[str, list[str]] = Field(default_factory=dict)
    artifact_formats: dict[str, RuntimeArtifactFormatSummary] = Field(default_factory=dict)
    topic_allowed_frame_ids: dict[str, list[str]] = Field(default_factory=dict)
    topic_default_frame_ids: dict[str, str] = Field(default_factory=dict)
    real_runtime_topic_allowed_frame_ids: dict[str, list[str]] = Field(default_factory=dict)
    real_runtime_topic_default_frame_ids: dict[str, str] = Field(default_factory=dict)
    real_runtime_required_topic_frame_ids: list[str] = Field(default_factory=list)
    real_runtime_required_endpoint_input_topics: list[str] = Field(default_factory=list)
    runtime_data_flow_topics: dict[str, list[str]] = Field(default_factory=dict)
    runtime_data_flow_stage_algorithm_interfaces: dict[str, list[str]] = Field(default_factory=dict)
    data_sources: dict[str, RuntimeDataSourceSummary] = Field(default_factory=dict)
    adapter_aliases: dict[str, list[RuntimeAdapterAliasSummary]] = Field(default_factory=dict)
    adapter_relays: dict[str, list[RuntimeAdapterAliasSummary]] = Field(default_factory=dict)
    algorithm_interfaces: dict[str, RuntimeAlgorithmInterfaceSummary] = Field(default_factory=dict)
    product_data_sources: dict[str, RuntimeProductDataSourceBinding] = Field(default_factory=dict)


class RuntimeDataflowPortSummary(GatewayResponseModel):
    module: str
    port: str
    direction: Literal["in", "out"]
    type: str | None = None
    msg_count: int = 0
    rate_hz: float = 0.0
    stale_ms: Any = None
    connected: bool | None = None
    callbacks: int | None = None


class RuntimeDataflowObservability(GatewayResponseModel):
    observable: bool
    observable_via: list[str] = Field(default_factory=list)
    module_port_candidates: list[RuntimeDataflowPortSummary] = Field(default_factory=list)
    gateway_channels: list[dict[str, Any]] = Field(default_factory=list)
    live_module_samples: bool = False
    has_fresh_module_sample: bool = False
    fresh_stale_ms_limit: float | None = None
    endpoint_topic_required: bool = False


class RuntimeDataflowCommunication(GatewayResponseModel):
    allowed: bool
    interfaces: list[dict[str, Any]] = Field(default_factory=list)
    arbitrary_publish_supported: bool = False
    policy: str


class RuntimeDataflowTokenEvidence(GatewayResponseModel):
    token: str
    kind: str
    observable: bool = False
    live: bool = False
    reason: str
    module_ports: list[RuntimeDataflowPortSummary] = Field(default_factory=list)
    gateway_channels: list[dict[str, Any]] = Field(default_factory=list)


class RuntimeDataflowStageEvidence(GatewayResponseModel):
    name: str
    owner: str | None = None
    frame_role: str | None = None
    map_dependency: str | None = None
    inputs: list[str] = Field(default_factory=list)
    outputs: list[str] = Field(default_factory=list)
    input_evidence: list[RuntimeDataflowTokenEvidence] = Field(default_factory=list)
    output_evidence: list[RuntimeDataflowTokenEvidence] = Field(default_factory=list)
    observable: bool = False
    live: bool = False
    status: str
    missing_inputs: list[str] = Field(default_factory=list)
    missing_outputs: list[str] = Field(default_factory=list)
    not_live_inputs: list[str] = Field(default_factory=list)
    not_live_outputs: list[str] = Field(default_factory=list)


class RuntimeDataflowTopicSummary(GatewayResponseModel):
    topic: str
    message_formats: list[str] = Field(default_factory=list)
    default_frame_id: str | None = None
    allowed_frame_ids: list[str] = Field(default_factory=list)
    required_for_real_runtime_frame_evidence: bool = False
    data_flow_stages: list[dict[str, Any]] = Field(default_factory=list)
    observability: RuntimeDataflowObservability
    communication: RuntimeDataflowCommunication
    inspection: dict[str, Any] = Field(default_factory=dict)


class RuntimeDataflowResponse(GatewayResponseModel):
    schema_version: int = 1
    ts: float
    authoritative: bool = False
    available: bool = False
    error: str | None = None
    run_plan: dict[str, Any] | None = None
    runtime_contract: str | None = None
    runtime_boundary: dict[str, Any] = Field(default_factory=dict)
    transport_layers: dict[str, Any] = Field(default_factory=dict)
    motion_path: dict[str, Any] = Field(default_factory=dict)
    endpoint_topic_required: bool = False
    module_ports: dict[str, Any] = Field(default_factory=dict)
    topics: list[RuntimeDataflowTopicSummary] = Field(default_factory=list)
    stage_evidence: list[RuntimeDataflowStageEvidence] = Field(default_factory=list)
    control_boundary: dict[str, Any] = Field(default_factory=dict)
    links: dict[str, str] = Field(default_factory=dict)


class RuntimeDataflowTopicDetailResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    ts: float
    selector: str
    topic: RuntimeDataflowTopicSummary | None = None
    runtime_contract: str | None = None
    runtime_boundary: dict[str, Any] = Field(default_factory=dict)
    inspection: dict[str, Any] = Field(default_factory=dict)
    control_boundary: dict[str, Any] = Field(default_factory=dict)
    available_topics: list[str] = Field(default_factory=list)
    links: dict[str, str] = Field(default_factory=dict)
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
    env: Literal["real", "sim"] | None = None
    product: ProductName | None = None
    state: Literal["active", "standby"] | None = None
    product_session_id: str | None = None
    simulation_only: bool | None = None
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


class ReadinessProductContract(GatewayResponseModel):
    product: ProductName | None = None
    product_session_id: str | None = None
    command_output_mode: str | None = None
    hardware_control_boundary: str | None = None
    processes: list[str] = Field(default_factory=list)
    required_topics: list[str] = Field(default_factory=list)
    required_capabilities: list[str] = Field(default_factory=list)
    native_readiness_required: bool = False


class ReadinessResponse(GatewayResponseModel):
    schema_version: int
    status: str
    ready: bool
    startup_state: str | None = None
    critical_modules: list[str] = Field(default_factory=list)
    critical_failed_modules: list[str] = Field(default_factory=list)
    data_ready: bool
    motion_ready: bool
    non_motion_safe: bool
    modules: dict[str, ReadinessModuleStatus]
    module_count: int
    failed_modules: list[str]
    reasons: list[str]
    advisories: list[str] = Field(default_factory=list)
    product_contract: ReadinessProductContract = Field(default_factory=ReadinessProductContract)
    runtime: ReadinessRuntimeSummary = Field(default_factory=ReadinessRuntimeSummary)
    ts: float


class LivenessResponse(GatewayResponseModel):
    status: str
    ts: float
    details_url: str = "/api/v1/health?details=true"
    gateway: dict[str, Any] = Field(default_factory=dict)
    sensors: dict[str, Any] = Field(default_factory=dict)


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


class ConstructedGoalTarget(GatewayResponseModel):
    schema_version: int = 1
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    frame_id: str = GATEWAY_MAP_FRAME_ID
    source: str = "coordinate"
    target_type: str = "coordinate"
    label: str | None = None
    location_name: str | None = None
    acceptance_radius_m: float | None = None
    max_speed_mps: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    ts: float | None = None


class ControlCommandResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    task_id: str | None = None
    native_request_id: str | None = None
    goal: list[float] | None = None
    yaw: float | None = None
    frame_id: str | None = None
    instruction: str | None = None
    mode: str | None = None
    reason: str | None = None
    target: ConstructedGoalTarget | None = None


class SafetyEstopResponse(GatewayResponseModel):
    active: bool
    enabled: bool
    timestamp: float
    accepted: bool = True
    replay: bool = False
    request_id: str | None = None
    client_id: str = "unknown"
    control_boundary: str | None = None
    message: str = ""
    command: CommandReceipt | None = None


class AuthLoginRequest(BaseModel):
    key: str = Field(default="", max_length=4096)


class AuthLoginResponse(GatewayResponseModel):
    ok: bool
    message: str


class AuthCheckResponse(GatewayResponseModel):
    auth_required: bool


class LeaseResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    holder: str | None = None
    active: bool | None = None
    expires_in: float | None = None


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
    map_content_epoch: int | None = Field(default=None, ge=0, strict=True)
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
    evidence_stage: str = "unavailable"
    final_output_confirmed: bool = False
    driver_delivery_accepted: bool = False
    output_sequence: int = 0
    ts: float | None = None
    operator_motion: dict[str, Any] | None = None


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
    path: list[PathPoint] = Field(default_factory=list)
    count: int = 0
    distance_m: float | None = None
    plan_ms: float | None = None
    planner: str | None = None
    source: str = "native_nav"
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


class InspectionAcceptanceRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mode: Literal["non_motion", "simulation", "field"] = "simulation"
    points: list[str] = Field(default_factory=list)
    tag: str | None = Field(default=None, max_length=128)
    require_octomap: bool = False
    require_occupancy: bool = False
    expected_data_source: str | None = None
    expected_source_profile: str | None = None
    expected_frame_id: str | None = None


class ProductFieldCheckRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mode: Literal["non_motion", "simulation", "field"] = "simulation"
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
    navigation: dict[str, Any] = Field(default_factory=dict)
    evidence: dict[str, Any] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
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
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None


class InspectionAcceptanceResponse(GatewayResponseModel):
    schema_version: str
    ok: bool
    summary: str
    mode: str = "field"
    field_ready: bool = False
    field_summary: str = "UNKNOWN"
    target_count: int = 0
    pass_count: int = 0
    fail_count: int = 0
    locations_count: int | None = None
    targets: list[InspectionAcceptanceTargetResult] = Field(default_factory=list)
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    evidence: dict[str, Any] = Field(default_factory=dict)
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
    map_content_epoch: int | None = Field(default=None, ge=0, strict=True)
    revision: int = Field(ge=1, le=_UINT64_MAX)
    points: list[InspectionRoutePointRequest] = Field(min_length=1)
    loop_count: int = Field(default=1, ge=1)
    failure_policy: InspectionFailurePolicy = "stop"
    max_retries: int = Field(default=0, ge=0)


class InspectionTaskStartRequest(BaseModel):
    route_id: str = Field(min_length=1, max_length=128)
    map_id: str | None = Field(default=None, max_length=128)
    revision: int = Field(default=0, ge=0, le=_UINT64_MAX)
    request_id: str | None = Field(default=None, max_length=128)


class InspectionTaskControlRequest(BaseModel):
    reason: str = Field(default="operator", max_length=128)
    request_id: str | None = Field(default=None, max_length=128)


class InspectionRouteRecord(GatewayResponseModel):
    id: str
    name: str | None = None
    map_id: str | None = None
    map_content_epoch: int | None = Field(default=None, ge=0, strict=True)
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
    action: Literal["delete"]
    route_id: str | None = None
    map_id: str | None = None
    ts: float = Field(default_factory=time.time)


class InspectionTaskCommandResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.task.v1"] = "lingtu.inspection.task.v1"
    ok: bool = True
    accepted: bool = True
    action: Literal["start", "pause", "resume", "cancel"]
    task_id: str
    request_id: str
    route_id: str | None = None
    map_id: str | None = None
    revision: int | None = None
    lifecycle: Literal["submission_accepted"] = "submission_accepted"
    terminal: bool = False
    ts: float = Field(default_factory=time.time)


class InspectionTaskStatusResponse(GatewayResponseModel):
    """Read-only projection of one native inspection task timeline."""

    schema_version: Literal["lingtu.inspection.task.v1"] = "lingtu.inspection.task.v1"
    found: bool
    task_id: str
    current_state: Literal[
        "PLANNING",
        "EXECUTING",
        "PAUSED",
        "RECOVERING",
        "SUCCESS",
        "FAILED",
        "CANCELLED",
    ] | None
    state_available: bool
    phase: Literal[
        "VALIDATING",
        "PLANNING",
        "NAVIGATING",
        "DWELLING",
        "PAUSED",
        "RECOVERING",
        "SUCCEEDED",
        "FAILED",
        "CANCELLED",
        "SETTLING",
        "ACTION_PENDING",
        "PAUSING",
        "CANCELLING",
    ] | None = None
    transition: Literal["PAUSE_REQUESTED", "CANCEL_REQUESTED"] | None = None
    state_source: str
    execution_confirmed: bool
    terminal: bool
    terminal_source: str = ""
    reason: str = ""
    progress: dict[str, Any] = Field(default_factory=dict)
    available_actions: list[str] = Field(default_factory=list)
    can_pause: bool = False
    can_resume: bool = False
    can_cancel: bool = False
    identity: dict[str, Any] = Field(default_factory=dict)
    last_submission: dict[str, Any] | None = None
    latest_event: dict[str, Any] | None = None
    timeline: list[dict[str, Any]] = Field(default_factory=list)
    recording: dict[str, Any] | None = None
    delivery: dict[str, Any] = Field(default_factory=dict)
    updated_at: float


class InspectionTaskListResponse(GatewayResponseModel):
    """Bounded discovery response for Gateway's non-authoritative task projection."""

    schema_version: Literal["lingtu.inspection.task.v1"] = "lingtu.inspection.task.v1"
    retention: Literal[
        "process_local_gateway_projection",
        "durable_gateway_projection",
    ]
    count: int = Field(ge=0)
    tasks: list[InspectionTaskStatusResponse] = Field(default_factory=list)
    ts: float


class InspectionTaskReportPoint(GatewayResponseModel):
    """One route visit and its business-evidence outcome."""

    loop_index: int = Field(ge=0)
    point_index: int = Field(ge=0)
    point_id: str
    action: str = ""
    status: Literal[
        "PENDING",
        "IN_PROGRESS",
        "COMPLETED",
        "MISSING_EVIDENCE",
        "INVALID_EVIDENCE",
        "UNAVAILABLE_EVIDENCE",
        "UNKNOWN",
    ]
    evidence_status: Literal[
        "NOT_REQUIRED",
        "PENDING",
        "VERIFIED",
        "MISSING",
        "INVALID",
        "UNAVAILABLE",
        "UNKNOWN",
    ]
    evidence_id: str = ""
    reason: str = ""


class InspectionTaskReportResponse(GatewayResponseModel):
    """Read-only product outcome for one task-addressed inspection."""

    schema_version: Literal["lingtu.inspection.report.v1"]
    task_id: str
    report_status: Literal[
        "IN_PROGRESS",
        "COMPLETE",
        "PARTIAL",
        "FAILED",
        "CANCELLED",
        "UNKNOWN",
    ]
    acceptance: Literal[
        "PENDING",
        "ACCEPTABLE",
        "REVIEW_REQUIRED",
        "NOT_ACCEPTABLE",
        "UNKNOWN",
    ]
    terminal: bool
    execution: dict[str, Any]
    identity: dict[str, Any]
    coverage: dict[str, int]
    points: list[InspectionTaskReportPoint] = Field(default_factory=list)
    issues: list[dict[str, Any]] = Field(default_factory=list)


class InspectionStatusResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.inspection.v1"] = "lingtu.inspection.v1"
    ok: bool = True
    status: dict[str, Any] = Field(default_factory=dict)
    ts: float = Field(default_factory=time.time)


class NavigationRuntimeBoundary(GatewayResponseModel):
    ok: bool = True
    declared: bool = False
    env: Literal["real", "sim"] | None = None
    product: ProductName | None = None
    state: Literal["active", "standby"] | None = None
    product_session_id: str | None = None
    data_source: str | None = None
    runtime_contract: str | None = None
    simulation_only: bool | None = None
    command_sink: str | None = None
    expected_command_sink: str | None = None
    slam_source: str | None = None
    localization_source: str | None = None
    mapping_source: str | None = None
    frames: RuntimeFrameSummary = Field(default_factory=RuntimeFrameSummary)
    frame_links: dict[str, RuntimeFrameLinkSummary] = Field(default_factory=dict)
    topic_allowed_frame_ids: dict[str, list[str]] = Field(default_factory=dict)
    topic_default_frame_ids: dict[str, str] = Field(default_factory=dict)
    required_topic_frame_ids: list[str] = Field(default_factory=list)
    runtime_data_flow_topics: list[str] = Field(default_factory=list)
    resolved_runtime_data_flow: list[RuntimeDataFlowStageSummary] = Field(default_factory=list)
    runtime_data_flow_stage_algorithm_interfaces: dict[str, list[str]] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)


class LocalizationFrameSummary(GatewayResponseModel):
    runtime_contract: str | None = None
    odometry_frame_id: str = "unknown"
    registered_cloud_frame_id: str | None = None
    map_cloud_frame_id: str | None = None
    odometry_expected_frame_ids: list[str] = Field(default_factory=list)
    registered_cloud_expected_frame_ids: list[str] = Field(default_factory=list)
    map_cloud_expected_frame_ids: list[str] = Field(default_factory=list)
    observed_topic_frame_ids: dict[str, str] = Field(default_factory=dict)
    missing_required_topic_frame_ids: list[str] = Field(default_factory=list)
    ok: bool = True
    mismatches: list[NavigationFrameMismatch] = Field(default_factory=list)


class LocalizationStatusResponse(GatewayResponseModel):
    schema_version: int
    state: str
    ready: bool
    has_odometry: bool
    odometry: Any = None
    session_mode: str | None = None
    active_map: str | None = None
    icp_quality: float = 0.0
    reported_state: Any = None
    confidence: float | None = None
    algorithm_healthy: bool = False
    backend: str | None = None
    health_source: str | None = None
    pose_fresh: bool | None = None
    pose_freshness: str = "unknown"
    stale_odometry: bool = False
    odom_age_ms: float | None = None
    cloud_age_ms: float | None = None
    degeneracy: str | None = None
    icp_fitness: float | None = None
    degeneracy_detected: bool | None = None
    effective_ratio: float | None = None
    condition_number: float | None = None
    min_eigenvalue: float | None = None
    max_eigenvalue: float | None = None
    degenerate_dof_count: int | None = None
    pos_cov_trace: float | None = None
    ieskf_iter_num: int | None = None
    ieskf_converged: bool | None = None
    map_cloud_fresh: bool | None = None
    status_target_hz: float | None = None
    imu_input_hz: float | None = None
    lidar_input_hz: float | None = None
    slam_tick_hz: float | None = None
    processed_scan_hz: float | None = None
    runtime_instance_id: str | None = None
    observation_sequence: int | None = None
    registered_points: int | None = None
    map_points: int | None = None
    imu_buffer: int | None = None
    lidar_buffer: int | None = None
    imu_batch: int | None = None
    dropped_lidar_frames: int | None = None
    dropped_imu_frames: int | None = None
    scan_start_s: float | None = None
    scan_end_s: float | None = None
    last_imu_s: float | None = None
    sync_wait_count: int | None = None
    imu_rollback_count: int | None = None
    lidar_rollback_count: int | None = None
    map_loaded: bool | None = None
    map_tracking: dict[str, Any] = Field(default_factory=dict)
    map_frame_jump: bool | None = None
    map_frame_jump_sequence: int | None = None
    scene_mode: str | None = None
    gnss_fusion_health: dict[str, Any] = Field(default_factory=dict)
    map_odom_tf: dict[str, Any] | None = None
    has_map_odom_tf: bool = False
    map_state: str | None = None
    map_save_supported: bool | None = None
    map_save_source: str | None = None
    relocalization_supported: bool = True
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    relocalization_state: str | None = None
    recovery_signal: str | None = None
    recovery_action: str | None = None
    localizer_health: str | None = None
    localizer_health_raw: str | None = None
    localizer_health_source: str | None = None
    localizer_health_topic_age_ms: float | None = None
    localizer_health_fitness: float | None = None
    localizer_health_iter: int | None = None
    localizer_health_cov_trace: float | None = None
    ts: float | None = None
    diag_received_ts: float | None = None
    diag_age_ms: float | None = None
    runtime: NavigationRuntimeBoundary
    frames: LocalizationFrameSummary
    can_relocalize: bool = False
    reasons: list[str] = Field(default_factory=list)
    raw: dict[str, Any] = Field(default_factory=dict)


class NavigationPathSummary(GatewayResponseModel):
    points: int = 0
    endpoint: str


class NavigationControlActiveSource(GatewayResponseModel):
    name: str
    label: str
    category: str
    owner: str
    priority: int | None = None
    active: bool | None = None
    age_ms: int | None = None


class NavigationControlSummary(GatewayResponseModel):
    mode: str
    lease: dict[str, Any] = Field(default_factory=dict)
    authority_source: str = "native_endpoint"
    authority_available: bool = False
    native_endpoint_available: bool = False
    active_cmd_source: str
    command_owner: str
    source_category: str
    manual_override: bool
    autonomy_requested: bool
    preempting_autonomy: bool
    operator_takeover_latched: bool = False
    resume_required: bool = False
    estop_latched: bool = False
    active_source: NavigationControlActiveSource
    sources: dict[str, Any] = Field(default_factory=dict)
    native_endpoint_control: dict[str, Any] = Field(default_factory=dict)


class NavigationLocalizationSummary(GatewayResponseModel):
    state: str | None = None
    ready: bool | None = None
    degraded: bool = False
    algorithm_healthy: bool | None = None
    pose_fresh: bool | None = None
    pose_freshness: str | None = None
    degeneracy: str | None = None
    speed_scale: float | None = None
    reasons: list[str] = Field(default_factory=list)


class NavigationReadinessSummary(GatewayResponseModel):
    navigation_ready: bool
    can_accept_goal: bool
    can_execute_autonomy: bool
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    localization_ready: bool
    control_owner: str
    session_mode: str | None = None


class NavigationProgressSummary(GatewayResponseModel):
    wp_index: int = 0
    wp_total: int = 0
    fraction: float = 0.0
    path_points: int = 0
    replan_count: int = 0
    active: bool = False
    terminal: bool = False


class NavigationDiagnosticsSummary(GatewayResponseModel):
    reason_codes: list[str] = Field(default_factory=list)
    failure_reason: str = ""
    localization_reasons: list[str] = Field(default_factory=list)
    frame_mismatches: list[NavigationFrameMismatch] = Field(default_factory=list)
    safety: dict[str, Any] | None = None


class NavigationMissionSummary(GatewayResponseModel):
    state: str
    raw: dict[str, Any] = Field(default_factory=dict)


class NavigationTargetSummary(GatewayResponseModel):
    goal: PathPoint | None = None
    current_waypoint: PathPoint | None = None
    distance_to_goal_m: float | None = None
    active_waypoint_distance_m: float | None = None
    remaining_waypoints: int | None = None


class NavigationSpeedPolicy(GatewayResponseModel):
    scale: float | None = None
    mode: Literal["normal", "cautious", "restricted", "hold", "unknown"] = "unknown"
    reason: str | None = None
    source: str = "native_navigation_state"
    applied: bool | None = None


class NavigationMotionSummary(GatewayResponseModel):
    current_speed_mps: float | None = None
    speed_scale: float | None = None
    speed_policy: NavigationSpeedPolicy
    active_cmd_source: str
    command_owner: str


class NavigationFeedbackSummary(GatewayResponseModel):
    next_action: str
    primary: str
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    reason_codes: list[str] = Field(default_factory=list)


class NavigationFrameSummary(GatewayResponseModel):
    planning_frame_id: str = GATEWAY_MAP_FRAME_ID
    odom_frame_id: str = "unknown"
    costmap_frame_id: str = "unknown"
    goal_frame_id: str | None = None
    ok: bool = True
    mismatches: list[NavigationFrameMismatch] = Field(default_factory=list)


class NavigationOperatorTaskState(GatewayResponseModel):
    state: Literal[
        "IDLE",
        "PLANNING",
        "EXECUTING",
        "RECOVERING",
        "PAUSED",
        "SUCCESS",
        "FAILED",
        "CANCELLED",
        "UNKNOWN",
    ]
    task_id: str
    request_id: str
    terminal: bool
    progress: float | None = None
    reason: str


class NavigationOperatorGoalAdmission(GatewayResponseModel):
    state: Literal["ACCEPTING", "BLOCKED", "UNKNOWN"]
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)


class NavigationOperatorControlState(GatewayResponseModel):
    authority: Literal["AUTONOMY", "OPERATOR", "NONE", "UNKNOWN"]
    resume_required: bool
    reason: str


class NavigationOperatorMotionState(GatewayResponseModel):
    permission: Literal["CLEAR", "HELD", "ESTOPPED", "UNKNOWN"]
    observation: Literal["MOVING", "QUIET", "UNKNOWN"]
    stop_confirmation: Literal[
        "NOT_REQUESTED",
        "PENDING",
        "CONFIRMED",
        "FAILED",
        "UNKNOWN",
    ]
    linear_speed_mps: float | None = None
    angular_speed_radps: float | None = None
    reason: str


class NavigationOperatorSummary(GatewayResponseModel):
    severity: Literal["OK", "INFO", "WARNING", "CRITICAL"]
    code: Literal[
        "STOP_CONFIRMATION_FAILED",
        "STOP_CONFIRMATION_PENDING",
        "ESTOPPED",
        "STATUS_SOURCE_UNKNOWN",
        "MOTION_HELD",
        "GOAL_ADMISSION_BLOCKED",
        "TASK_FAILED",
        "TASK_RECOVERING",
        "TASK_PLANNING",
        "TASK_PAUSED",
        "NAVIGATION_ADVISORY",
        "TASK_EXECUTING",
        "TASK_SUCCEEDED",
        "TASK_CANCELLED",
        "READY_FOR_GOAL",
    ]
    next_action: Literal[
        "inspect_stop_failure",
        "wait_for_stop_confirmation",
        "clear_estop",
        "check_status_sources",
        "resolve_motion_hold",
        "resolve_goal_blockers",
        "inspect_task_failure",
        "monitor_recovery",
        "wait_for_plan",
        "resume_or_cancel",
        "review_advisories",
        "monitor_progress",
        "choose_goal",
    ]


class NavigationOperatorState(GatewayResponseModel):
    schema_version: Literal[1] = 1
    task: NavigationOperatorTaskState
    goal_admission: NavigationOperatorGoalAdmission
    control: NavigationOperatorControlState
    motion: NavigationOperatorMotionState
    summary: NavigationOperatorSummary


class NavigationStatusResponse(GatewayResponseModel):
    schema_version: int
    state: str
    has_odometry: bool
    can_accept_goal: bool
    navigation_ready: bool
    wp_index: int = 0
    wp_total: int = 0
    replan_count: int = 0
    speed_scale: float | None = None
    failure_reason: str = ""
    reason_codes: list[str] = Field(default_factory=list)
    readiness: NavigationReadinessSummary
    progress: NavigationProgressSummary
    path: NavigationPathSummary
    runtime: NavigationRuntimeBoundary
    frames: NavigationFrameSummary
    control: NavigationControlSummary
    localization: NavigationLocalizationSummary
    target: NavigationTargetSummary
    motion: NavigationMotionSummary
    feedback: NavigationFeedbackSummary
    diagnostics: NavigationDiagnosticsSummary
    mission: NavigationMissionSummary
    goal_status: dict[str, Any] | None = None
    operator_state: NavigationOperatorState
    ts: float


class NavigationGoalStatusQueryResponse(GatewayResponseModel):
    schema_version: int = 1
    found: bool
    request_id: str
    status: dict[str, Any] | None = None
    reason: str = ""
    ts: float = Field(default_factory=time.time)


class NavigationTaskStatusQueryResponse(GatewayResponseModel):
    schema_version: int = 1
    found: bool
    task_id: str
    request_id: str = ""
    status: dict[str, Any] | None = None
    source: Literal["", "live_gateway_cache", "durable_task_ledger", "durable_native_goal_status"] = ""
    evidence_status: str = ""
    reason: str = ""
    ts: float = Field(default_factory=time.time)


class SessionResponse(GatewayResponseModel):
    mode: str
    env: Literal["real", "sim"] = "real"
    product: ProductName | None = None
    product_session_id: str | None = None
    slam_profile: str = "stopped"
    localization_backend: str | None = None
    health_source: str | None = None
    active_map: str | None = None
    saved_active_map: str | None = None
    map_has_pcd: bool = False
    map_has_octomap: bool = False
    can_activate: bool = False
    since: float | None = None
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
    exploration_blockers: list[str] = Field(default_factory=list)
    safety_clear: bool = True
    safety: dict[str, Any] | None = None
    explorer_backend: Literal["none", "tare"] = "none"
    explorer_available: bool = False
    explorer_unavailable_reason: str | None = None
    explorer_required_product: str | None = None


class MapInfo(GatewayResponseModel):
    model_config = ConfigDict(extra="ignore")

    name: str
    has_pcd: bool = False
    has_occupancy: bool = False
    has_octomap: bool = False
    can_activate: bool
    state: str | None = None
    is_active: bool = False
    size_mb: float | None = None
    patch_count: int = 0


class MapListResponse(GatewayResponseModel):
    model_config = ConfigDict(extra="ignore")

    schema_version: int = 1
    maps: list[MapInfo] = Field(default_factory=list)
    count: int = 0
    active: str = ""
    ts: float = Field(default_factory=time.time)


class MapSaveOperationStatus(GatewayResponseModel):
    """Customer-visible state of one durable map-save operation."""

    model_config = ConfigDict(extra="ignore")

    operation_id: str | None = None
    request_id: str | None = None
    map_id: str | None = None
    name: str | None = None
    state: str | None = None
    phase: str | None = None
    progress: float | None = None
    reason: str | None = None
    reason_code: str | None = None
    message: str | None = None
    created_at: int | float | str | None = None
    updated_at: int | float | str | None = None
    completed_at: int | float | str | None = None
    started_at: int | float | str | None = None
    finished_at: int | float | str | None = None
    created_at_ns: int | None = None
    updated_at_ns: int | None = None
    completed_at_ns: int | None = None
    started_at_ns: int | None = None
    finished_at_ns: int | None = None
    ts: int | float | str | None = None
    cancel_requested: bool | None = None
    recovered: bool | None = None
    replayed: bool | None = None
    attempt: int | None = None


class MapSaveOperationResponse(MapSaveOperationStatus):
    """Narrow external response for map-save admission and operation queries."""

    model_config = ConfigDict(extra="ignore")

    schema_version: int = 1
    ok: bool
    success: bool | None = None
    accepted: bool | None = None
    status: str | None = None
    operation: MapSaveOperationStatus | None = None
    operations: list[MapSaveOperationStatus] | None = None
    count: int | None = None
    result: dict[str, Any] | None = None
    can_activate: bool | None = None
    occupancy_ok: bool | None = None
    octomap_ok: bool | None = None
    metadata_ok: bool | None = None
    semantic_ok: bool | None = None
    point_count: int | None = None
    slam_profile: str | None = None
    map_save_source: str | None = None


class MapLifecycleResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool | None = None
    message: str | None = None
    status: Any = None
    reason_code: str | None = None
    name: str | None = None
    active: str | None = None
    old_name: str | None = None
    new_name: str | None = None
    octomap_ok: bool | None = None
    octomap_message: str | None = None
    occupancy_ok: bool | None = None
    occupancy_message: str | None = None
    can_activate: bool | None = None
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
    maps: list[Any] | None = None
    live_cloud_reset: bool | None = None
    requested_map: str | None = None
    switch: str | None = None
    operator_command: str | None = None
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
    content_epoch: int | None = Field(default=None, gt=0, strict=True)
    points: list[float] | list[tuple[float, float, float]] = Field(default_factory=list)
    bounds: dict[str, list[float]] | None = None
    ts: float = Field(default_factory=time.time)


class TemporalMemoryResponse(GatewayResponseModel):
    observations: list[Any] = Field(default_factory=list)
    count: int = 0


class DirectedExplorationIntent(GatewayResponseModel):
    """The native TARE direction preference currently accepted by Gateway."""

    active: bool
    x: float | None = None
    y: float | None = None
    ttl_s: float | None = None
    product_session_id: str
    frame_id: MapFrameId = GATEWAY_MAP_FRAME_ID
    reason: str
    request_id: str | None = None


class DirectedExplorationResponse(GatewayResponseModel):
    """Accepted explicit native TARE direction-intent operation."""

    schema_version: int = 1
    ok: Literal[True] = True
    accepted: bool
    status: Literal["accepted", "cleared"]
    intent: DirectedExplorationIntent
    native: dict[str, Any] = Field(default_factory=dict)


class ExplorationCommandResponse(GatewayResponseModel):
    schema_version: str | int | None = None
    ok: bool | None = None
    accepted: bool | None = None
    request_id: str | None = None
    exploration_run_id: str | None = None
    replay: bool = False
    admission: str | None = None
    state: str | None = None
    reason: str | None = None
    terminal: bool = False
    motion_stop: dict[str, Any] = Field(default_factory=dict)
    native: dict[str, Any] | None = None
    status: Any = None


class ExplorationRunResponse(GatewayResponseModel):
    """Admission receipt or durable projection for one Explore execution."""

    model_config = ConfigDict(extra="ignore")

    schema_version: str = "lingtu.explore.run.v1"
    ok: bool = True
    found: bool = True
    accepted: bool | None = None
    request_id: str | None = None
    exploration_run_id: str
    replay: bool = False
    admission: str | None = None
    state: str
    state_source: str | None = None
    reason: str
    terminal: bool = False
    can_resume: bool = False
    motion_stop: dict[str, Any] = Field(default_factory=dict)
    identity: dict[str, Any] | None = None
    native: dict[str, Any] | None = None
    status: Any = None


class ExplorationRunListResponse(GatewayResponseModel):
    """Bounded recent Explore executions and projection health."""

    schema_version: Literal["lingtu.explore.run.list.v1"] = (
        "lingtu.explore.run.list.v1"
    )
    runs: list[ExplorationRunResponse] = Field(default_factory=list)
    health: dict[str, Any] = Field(default_factory=dict)


class ExplorationStatusResponse(GatewayResponseModel):
    available: bool
    backend: Literal["none", "tare"] = "none"
    exploring: bool = False
    frontier_count: int = 0
    can_start: bool = False
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    navigation: dict[str, Any] = Field(default_factory=dict)
    run_projection: dict[str, Any] = Field(default_factory=dict)
    reason: str | None = None
    required_product: str | None = None
    supported_products: list[str] | None = None
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
    manual_systemctl_required: bool = False


class ServiceStatusResponse(GatewayResponseModel):
    schema_version: int = 1
    services: dict[str, str] = Field(default_factory=dict)
    service_details: dict[str, dict[str, Any]] = Field(default_factory=dict)
    readiness: dict[str, Any] = Field(default_factory=dict)
    field_readiness: dict[str, Any] = Field(default_factory=dict)
    service_groups: dict[str, list[str]] = Field(default_factory=dict)
    service_metadata: dict[str, dict[str, Any]] = Field(default_factory=dict)
    product_runtime: str = "native_dds"


class LocalizationOperationResponse(GatewayResponseModel):
    """Stable result envelope for localization domain commands."""

    schema_version: int = 1
    ok: bool
    success: bool
    map_name: str | None = None
    mode: Literal["seeded", "global", "tracking"] | None = None
    request_id: str | None = None
    message: str | None = None
    quality: float | None = None
    details: dict[str, Any] | None = None
    ts: float = Field(default_factory=time.time)


class RecordingOperationResponse(GatewayResponseModel):
    status: str | None = None
    state: str | None = None
    backend: str | None = None
    session_id: str | None = None
    path: str | None = None
    pid: int | None = None
    duration: int | None = None
    prefix: str | None = None
    capture_profile: Literal["sensors", "evidence"] | None = None
    task_id: str | None = None
    camera: bool | None = None
    minimum_free_gib: int | None = None
    error: str | None = None
    detail: Any = None


class RecordingStatusResponse(GatewayResponseModel):
    available: bool = False
    healthy: bool = False
    backend: str = "native_mcap"
    state: str = "idle"
    session_id: str | None = None
    recording: bool
    path: str | None = None
    duration_s: float = 0.0
    size_bytes: int = 0
    size_truncated: bool = False
    pid: int | None = None
    exit_code: int | None = None
    disk_free: int = 0
    disk_total: int = 0
    error: str | None = None


class Go2RTCStatusResponse(GatewayResponseModel):
    available: bool
    reason: str | None = None
    status: int | None = None
    streams: list[str] = Field(default_factory=list)


class ClientLinks(GatewayResponseModel):
    bootstrap: str | None = None
    capabilities: str | None = None
    traffic: str | None = None
    state: str | None = None
    scene_graph: str | None = None
    locations: str | None = None
    location_detail: str | None = None
    path: str | None = None
    localization_status: str | None = None
    navigation_status: str | None = None
    navigation_goal_status: str | None = None
    navigation_task_status: str | None = None
    runtime_dataflow: str | None = None
    runtime_dataflow_topic: str | None = None
    runtime_dataflow_subscribe: str | None = None
    readiness: str | None = None
    metrics: str | None = None
    auth_login: str | None = None
    auth_check: str | None = None
    events: str | None = None
    teleop_ws: str | None = None
    camera_ws: str | None = None
    cloud_ws: str | None = None
    scan_ws: str | None = None
    camera_snapshot: str | None = None
    webrtc_whep: str | None = None
    go2rtc_status: str | None = None
    health: str | None = None
    session: str | None = None
    navigation_goal_candidate: str | None = None
    navigation_plan: str | None = None
    inspection_acceptance: str | None = None
    inspection_routes: str | None = None
    inspection_route_detail: str | None = None
    inspection_tasks: str | None = None
    inspection_task_status: str | None = None
    inspection_task_report: str | None = None
    inspection_task_pause: str | None = None
    inspection_task_resume: str | None = None
    inspection_task_cancel: str | None = None
    inspection_status: str | None = None
    navigation_cancel: str | None = None
    navigation_task_cancel: str | None = None
    navigation_task_pause: str | None = None
    navigation_task_resume: str | None = None
    navigation_resume: str | None = None
    goal: str | None = None
    navigate_click: str | None = None
    stop: str | None = None
    estop_reset: str | None = None
    instruction: str | None = None
    visual_servo: str | None = None
    mode: str | None = None
    lease: str | None = None
    maps: str | None = None
    map_delete: str | None = None
    map_build_occupancy: str | None = None
    map_rename: str | None = None
    map_save: str | None = None
    map_operations: str | None = None
    map_operation_status: str | None = None
    map_operation_cancel: str | None = None
    map_operation_retry: str | None = None
    map_import_pcd: str | None = None
    map_crop: str | None = None
    map_mark_zone: str | None = None
    map_build_octomap: str | None = None
    map_validate_plan: str | None = None
    map_cloud_reset: str | None = None
    map_points: str | None = None
    saved_map_points: str | None = None
    explore_status: str | None = None
    explore_start: str | None = None
    explore_stop: str | None = None
    explore_directed: str | None = None
    explore_directed_clear: str | None = None
    service_status: str | None = None
    slam_status: str | None = None
    localization_relocalize: str | None = None
    localization_map_tracking: str | None = None
    recording_start: str | None = None
    recording_stop: str | None = None
    recording_status: str | None = None
    memory_temporal: str | None = None
    memory_temporal_semantic: str | None = None
    diagnostic_pack: str | None = None
    field_check: str | None = None
    routecheck_latest: str | None = None
    real_runtime_evidence_latest: str | None = None
    runtime_contract: str | None = None


class CameraPortStatus(GatewayResponseModel):
    frames: int = 0
    fps: float = 0.0
    stale_ms: float | None = None


class CameraInfoStatus(GatewayResponseModel):
    frames: int = 0
    active_topic: str | None = None
    preferred_topic: str | None = None
    topics: list[str] = Field(default_factory=list)


class CameraJpegStatus(GatewayResponseModel):
    cached: bool = False
    seq: int = 0
    bytes: int = 0


class CameraMediaStatus(GatewayResponseModel):
    schema_version: int
    available: bool
    status: Literal["streaming", "idle", "stale", "error", "not_loaded"]
    reason: str | None = None
    backend: str | None = None
    fps: float = 0.0
    frames: int = 0
    color: CameraPortStatus
    depth: CameraPortStatus
    camera_info: CameraInfoStatus
    reconnect_count: int = 0
    service_recovery_allowed: bool = False
    service_recovery_suppressed: bool = False
    jpeg: CameraJpegStatus
    ts: float
    error: str | None = None


class WHEPMediaStatus(GatewayResponseModel):
    supported: bool
    endpoint: str
    go2rtc_status: str


class AppMediaLinks(GatewayResponseModel):
    events: str
    teleop_ws: str
    camera_ws: str
    cloud_ws: str
    scan_ws: str | None = None
    camera_snapshot: str
    webrtc_whep: str | None = None
    go2rtc_status: str | None = None
    camera: CameraMediaStatus
    whep: WHEPMediaStatus


class RealtimeEventsCapability(GatewayResponseModel):
    path: str
    transport: Literal["sse"]
    initial_snapshot: bool
    heartbeat_s: float
    schema_version: int | None = None
    event_schema: str | None = None
    event_id_field: str | None = None
    timestamp_field: str | None = None
    heartbeat_type: str | None = None
    snapshot_type: str | None = None
    event_types: list[str] = Field(default_factory=list)
    diagnostic_event_types: list[str] = Field(default_factory=list)
    named_events: bool | None = None
    browser_handler: str | None = None
    retry_ms: int | None = None
    replay_supported: bool | None = None
    last_event_id_header: str | None = None
    drop_policy: str | None = None
    large_event_policy: dict[str, Any] = Field(default_factory=dict)


class RealtimeTeleopCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    control_messages: list[str] = Field(default_factory=list)
    binary_camera_frames: bool


class RealtimeCameraCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    binary_camera_frames: bool
    explicit_subscription: bool


class RealtimeCloudCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    binary_point_cloud_frames: bool
    semantic: str | None = None
    drop_policy: str | None = None


class AppRealtimeCapabilities(GatewayResponseModel):
    events: RealtimeEventsCapability
    teleop: RealtimeTeleopCapability
    camera: RealtimeCameraCapability
    cloud: RealtimeCloudCapability
    scan: RealtimeCloudCapability | None = None
    scene_layers: dict[str, Any] = Field(default_factory=dict)


class ValidationGateCapability(GatewayResponseModel):
    schema_version: str
    scope: str
    acceptance_step: int | None = None
    required_when: str | None = None
    command: str | None = None
    artifact: str
    expected_runtime_contract: str | None = None


class TrafficSSEStats(GatewayResponseModel):
    clients: int = 0
    queue_maxsize: int | None = None
    queue_depths: list[int] = Field(default_factory=list)
    max_depth_seen: int = 0
    latest_event_id: int = 0
    published_events: int = 0
    dropped_events: int = 0
    suppressed_events: dict[str, int] = Field(default_factory=dict)
    raster_min_interval_s: float | None = None
    drop_policy: str | None = None


class TrafficCloudStats(GatewayResponseModel):
    clients: int = 0
    queue_maxsize: int | None = None
    queue_depths: list[int] = Field(default_factory=list)
    max_depth_seen: int = 0
    published_frames: int = 0
    dropped_frames: int = 0
    drop_policy: str | None = None
    latest_seq: int = 0


class AppTrafficResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    status: Literal["ok", "degraded"]
    sse: TrafficSSEStats
    cloud: TrafficCloudStats
    scan: TrafficCloudStats | None = None
    recommended_client_rates_hz: dict[str, float] = Field(default_factory=dict)
    client_policy: dict[str, Any]
    warnings: list[str] = Field(default_factory=list)
    links: ClientLinks


class StateResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    lease: dict[str, Any]
    teleop: TeleopSummary
    session: dict[str, Any]
    localization: LocalizationStatusResponse
    navigation: NavigationStatusResponse
    visual_servo: dict[str, Any] | None = None
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    media: AppMediaLinks
    links: ClientLinks


class AppBootstrapResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    robot: dict[str, Any]
    session: dict[str, Any]
    mission: dict[str, Any]
    safety: dict[str, Any]
    localization: LocalizationStatusResponse
    navigation: NavigationStatusResponse
    control: dict[str, Any]
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    media: AppMediaLinks
    traffic: dict[str, Any]
    capabilities: dict[str, bool]
    runtime_products: dict[str, Any] = Field(default_factory=dict)
    capabilities_endpoint: str
    links: ClientLinks


class AppCapabilitiesResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    auth: dict[str, Any]
    features: dict[str, bool]
    runtime_products: dict[str, Any] = Field(default_factory=dict)
    endpoints: dict[str, dict[str, EndpointSpec]]
    probes: dict[str, EndpointSpec]
    validation_gates: dict[str, ValidationGateCapability]
    realtime: AppRealtimeCapabilities
    client_policy: dict[str, Any]
    links: ClientLinks
