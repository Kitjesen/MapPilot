"""FastAPI request/response schemas for app/web Gateway contracts."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from runtime.runtime_interface import body_frame_id, map_frame_id

GATEWAY_MAP_FRAME_ID = map_frame_id()
GATEWAY_BODY_FRAME_ID = body_frame_id()
MapFrameId = Literal["map"]


class GatewayResponseModel(BaseModel):
    """Base model that documents known fields without stripping future additions."""

    model_config = ConfigDict(extra="allow")


class CommandReceipt(GatewayResponseModel):
    name: str
    request_id: str | None = None
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
    validation_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    max_age_s: float
    runtime_contract: str | None = None
    runtime_evidence_ok: bool = False
    simulation_only: bool | None = None
    real_robot_motion: bool | None = None
    cmd_vel_sent_to_hardware: bool | None = None
    checked_real_motion_evidence: dict[str, Any] = Field(default_factory=dict)
    checked_hardware_boundary_evidence: dict[str, Any] = Field(default_factory=dict)
    checked_live_topic_freshness: dict[str, Any] = Field(default_factory=dict)
    checked_runtime_data_flow_evidence: dict[str, Any] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    reason: str | None = None
    ts: float


class AlgorithmBenchmarkLatestResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.algorithm_benchmark_latest.v1"] = "lingtu.algorithm_benchmark_latest.v1"
    ok: bool
    read_only: bool = True
    ros2_topic_required: bool = False
    publishes: list[str] = Field(default_factory=list)
    artifacts_root: str
    count: int = 0
    summary_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    max_age_s: float
    preset: str = "dimos_benchmark"
    source: str = "server_sim_closure"
    active_product_profile: str = "inspection_mvp"
    strict_benchmark_profile: str = "dimos_benchmark"
    summary_schema_version: str | None = None
    claim_allowed: bool = False
    missing_or_failed: list[str] = Field(default_factory=list)
    required_gate_sequence: list[str] = Field(default_factory=list)
    validation_flow: list[dict[str, Any]] = Field(default_factory=list)
    claim_boundary: dict[str, Any] = Field(default_factory=dict)
    product_profiles: dict[str, dict[str, Any]] = Field(default_factory=dict)
    dimos_gap: dict[str, Any] = Field(default_factory=dict)
    blocking_categories: dict[str, list[str]] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    reason: str | None = None
    latest: dict[str, Any] | None = None
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


class CmdVelRequest(BaseModel):
    vx: float
    vy: float = 0.0
    wz: float
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("vx", "vy", "wz")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class InstructionRequest(BaseModel):
    text: str = Field(min_length=1, max_length=1024)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class VoiceTurnRequest(BaseModel):
    """AskMe voice-runtime bridge request."""

    model_config = ConfigDict(extra="forbid")

    text: str = Field(min_length=1, max_length=1024)
    operator_id: str = Field(default="", max_length=128)
    session_id: str = Field(default="", max_length=128)
    channel: str = Field(default="voice", max_length=32)
    robot_id: str | None = Field(default=None, max_length=128)
    site_id: str | None = Field(default=None, max_length=128)
    submit: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str | None = Field(default=None, max_length=128)

    @field_validator("text")
    @classmethod
    def normalized_text(cls, value: str) -> str:
        text = value.strip()
        if not text:
            raise ValueError("text must not be blank")
        return text

    @field_validator(
        "operator_id",
        "session_id",
        "channel",
        "robot_id",
        "site_id",
        "request_id",
        "client_id",
    )
    @classmethod
    def normalized_identifier(cls, value: str | None) -> str | None:
        if value is None:
            return None
        return value.strip()


class SafetyEstopRequest(BaseModel):
    """AskMe-compatible emergency-stop activation request."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str | None = Field(default=None, max_length=128)


class VisualServoRequest(BaseModel):
    mode: str = Field(default="find", max_length=16)
    target: str | None = Field(default=None, max_length=256)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @model_validator(mode="after")
    def validate_command(self) -> VisualServoRequest:
        self.mode = self.mode.strip().lower()
        if self.mode not in {"find", "follow", "stop"}:
            raise ValueError("mode must be find|follow|stop")
        if self.target is not None:
            self.target = self.target.strip()
        if self.mode != "stop" and not self.target:
            raise ValueError("target is required for find/follow")
        return self


class StopRequest(BaseModel):
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class CancelRequest(BaseModel):
    reason: str = Field(default="client_cancel", max_length=256)
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


class SessionStartRequest(BaseModel):
    mode: str | None = Field(default=None, max_length=32)
    map_name: str | None = Field(default=None, max_length=128)
    map: str | None = Field(default=None, max_length=128)
    profile: str | None = Field(default=None, max_length=64)
    product_profile: str | None = Field(default=None, max_length=64)
    product_session: str | None = Field(default=None, max_length=64)
    slam_profile: str | None = Field(default=None, max_length=64)
    slam_backend: str | None = Field(default=None, max_length=64)


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


class TemporalSemanticRequest(BaseModel):
    embedding: list[float] | None = None
    since: str | None = Field(default=None, max_length=64)
    top_k: int = Field(default=10, ge=1, le=1000)
    label: str | None = Field(default=None, max_length=128)


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
    ros_type: str
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


class RuntimeProfileDataSourceBinding(GatewayResponseModel):
    profile: str
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
    topic_ros_types: dict[str, list[str]] = Field(default_factory=dict)
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
    profile_data_sources: dict[str, RuntimeProfileDataSourceBinding] = Field(default_factory=dict)


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
    ros2_topic_required: bool = False


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
    runtime_contract: str | None = None
    runtime_boundary: dict[str, Any] = Field(default_factory=dict)
    transport_layers: dict[str, Any] = Field(default_factory=dict)
    endpoint_topic_required: bool = False
    ros2_topic_required: bool = False
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


class ControlCommandResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    goal: list[float] | None = None
    yaw: float | None = None
    frame_id: str | None = None
    instruction: str | None = None
    mode: str | None = None
    reason: str | None = None
    target: ConstructedGoalTarget | None = None


class VoiceTurnResult(GatewayResponseModel):
    action_type: Literal["runtime"] = "runtime"
    spoken_reply: str = Field(min_length=1)
    text: str
    status: str
    submitted: bool
    accepted: bool
    replay: bool = False
    request_id: str | None = None
    client_id: str
    operator_id: str = ""
    session_id: str = ""
    channel: str = "voice"
    robot_id: str | None = None
    site_id: str | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    command: CommandReceipt | None = None
    reason: str | None = None


class VoiceTurnResponse(GatewayResponseModel):
    handled: Literal[True] = True
    turn: VoiceTurnResult


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


class NavigationRuntimeBoundary(GatewayResponseModel):
    ok: bool = True
    declared: bool = False
    profile: str | None = None
    endpoint: str | None = None
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
    active_cmd_source: str
    command_owner: str
    source_category: str
    manual_override: bool
    autonomy_requested: bool
    preempting_autonomy: bool
    mux_available: bool
    active_source: NavigationControlActiveSource
    sources: dict[str, Any] = Field(default_factory=dict)
    cmd_vel_mux: dict[str, Any] = Field(default_factory=dict)


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
    cmd_vel_mux_available: bool = False
    frame_mismatches: list[NavigationFrameMismatch] = Field(default_factory=list)
    safety: dict[str, Any] | None = None
    plan_safety_policy: str | None = None
    last_plan_report: dict[str, Any] = Field(default_factory=dict)


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
    source: str = "mission_status"
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


class NavigationStatusResponse(GatewayResponseModel):
    schema_version: int
    state: str
    has_odometry: bool
    can_accept_goal: bool
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
    ts: float


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


class TemporalMemoryResponse(GatewayResponseModel):
    observations: list[Any] = Field(default_factory=list)
    count: int = 0


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
    runtime_dataflow: str | None = None
    runtime_dataflow_topic: str | None = None
    runtime_dataflow_subscribe: str | None = None
    runtime_switch_plan: str | None = None
    runtime_switch: str | None = None
    algorithm_benchmark_latest: str | None = None
    devices: str | None = None
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
    session_start: str | None = None
    session_end: str | None = None
    navigation_goal_candidate: str | None = None
    navigation_plan: str | None = None
    inspection_acceptance: str | None = None
    inspection_routes: str | None = None
    inspection_route_detail: str | None = None
    inspection_route_start: str | None = None
    inspection_status: str | None = None
    inspection_pause: str | None = None
    inspection_resume: str | None = None
    inspection_cancel: str | None = None
    navigation_cancel: str | None = None
    navigation_resume: str | None = None
    goal: str | None = None
    navigate_click: str | None = None
    stop: str | None = None
    instruction: str | None = None
    visual_servo: str | None = None
    mode: str | None = None
    lease: str | None = None
    maps: str | None = None
    map_lifecycle: str | None = None
    map_activate: str | None = None
    map_rename: str | None = None
    map_save: str | None = None
    map_import_pcd: str | None = None
    map_crop: str | None = None
    map_mark_zone: str | None = None
    map_build_octomap: str | None = None
    map_validate_plan: str | None = None
    map_restore_predufo: str | None = None
    map_cloud_reset: str | None = None
    map_points: str | None = None
    saved_map_points: str | None = None
    explore_status: str | None = None
    explore_start: str | None = None
    explore_stop: str | None = None
    service_status: str | None = None
    slam_status: str | None = None
    slam_switch: str | None = None
    slam_restart: str | None = None
    slam_auto_relocalize: str | None = None
    slam_relocalize: str | None = None
    slam_track_against_map: str | None = None
    bag_start: str | None = None
    bag_stop: str | None = None
    bag_status: str | None = None
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
    teleop_stream_clients: int = 0
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
    legacy_event_types: list[str] = Field(default_factory=list)
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
    legacy_camera_query: str | None = None


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
    requires_prior_gates: list[str] = Field(default_factory=list)
    conditional_prior_gates: list[str] = Field(default_factory=list)
    proves: list[str] = Field(default_factory=list)
    operator_summary_sections: list[str] = Field(default_factory=list)
    command: str | None = None
    script: str | None = None
    collector_command: str | None = None
    gate_command: str | None = None
    artifact: str
    expected_runtime_contract: str | None = None
    requires_ros: bool
    requires_real_robot_runtime: bool
    requires_active_robot_run: bool
    collector_publishes_control_topics: bool
    control_topics_published: list[str] = Field(default_factory=list)
    validates: list[str] = Field(default_factory=list)
    checks: list[str] = Field(default_factory=list)
    coverage: dict[str, list[str]] = Field(default_factory=dict)


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
    slope_grid_inline: bool = False
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
    odometry: Any = None
    safety: Any = None
    mission: Any = None
    eval: Any = None
    dialogue: Any = None
    mode: str
    lease: dict[str, Any]
    teleop: TeleopSummary
    session: dict[str, Any]
    localization: LocalizationStatusResponse
    navigation: NavigationStatusResponse
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
    capabilities_endpoint: str
    links: ClientLinks


class AppCapabilitiesResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    auth: dict[str, Any]
    features: dict[str, bool]
    endpoints: dict[str, dict[str, EndpointSpec]]
    probes: dict[str, EndpointSpec]
    validation_gates: dict[str, ValidationGateCapability]
    realtime: AppRealtimeCapabilities
    client_policy: dict[str, Any]
    links: ClientLinks


class DriverSwapRequest(BaseModel):
    """POST /api/v1/driver/swap request body."""

    driver: str = Field(min_length=1, max_length=64)
    config: dict[str, Any] = Field(default_factory=dict)


class DriverSwapResponse(GatewayResponseModel):
    """POST /api/v1/driver/swap response."""

    schema_version: int = 1
    success: bool
    message: str = ""
    swap_time_ms: float = 0.0
    driver: str | None = None
    detail: dict[str, Any] | None = None
    error: str | None = None
