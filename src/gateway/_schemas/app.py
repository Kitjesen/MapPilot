"""Internal Gateway app request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field

from gateway._schemas.common import (
    EndpointSpec,
    GatewayResponseModel,
    ServerInfo,
    TeleopSummary,
)
from gateway._schemas.navigation_status import (
    LocalizationStatusResponse,
    NavigationStatusResponse,
)


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


__all__ = (
    "AppBootstrapResponse",
    "AppCapabilitiesResponse",
    "AppMediaLinks",
    "AppRealtimeCapabilities",
    "AppTrafficResponse",
    "CameraInfoStatus",
    "CameraJpegStatus",
    "CameraMediaStatus",
    "CameraPortStatus",
    "ClientLinks",
    "RealtimeCameraCapability",
    "RealtimeCloudCapability",
    "RealtimeEventsCapability",
    "RealtimeTeleopCapability",
    "StateResponse",
    "TrafficCloudStats",
    "TrafficSSEStats",
    "ValidationGateCapability",
    "WHEPMediaStatus",
)
