"""Internal Gateway runtime contracts request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


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


__all__ = (
    "RuntimeAdapterAliasSummary",
    "RuntimeAlgorithmInterfaceSummary",
    "RuntimeArtifactFormatSummary",
    "RuntimeContractManifest",
    "RuntimeDataFlowStageSummary",
    "RuntimeDataSourceSummary",
    "RuntimeDataflowCommunication",
    "RuntimeDataflowObservability",
    "RuntimeDataflowPortSummary",
    "RuntimeDataflowResponse",
    "RuntimeDataflowStageEvidence",
    "RuntimeDataflowTokenEvidence",
    "RuntimeDataflowTopicDetailResponse",
    "RuntimeDataflowTopicSummary",
    "RuntimeFrameLinkSummary",
    "RuntimeFrameSummary",
    "RuntimeMessageFormatSummary",
    "RuntimeProfileDataSourceBinding",
    "RuntimeTransform3D",
)
