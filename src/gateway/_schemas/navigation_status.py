"""Internal Gateway navigation status request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field

from gateway._schemas.common import (
    GATEWAY_MAP_FRAME_ID,
    GatewayResponseModel,
    NavigationFrameMismatch,
)
from gateway._schemas.navigation import (
    PathPoint,
)
from gateway._schemas.runtime_contracts import (
    RuntimeDataFlowStageSummary,
    RuntimeFrameLinkSummary,
    RuntimeFrameSummary,
)


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


__all__ = (
    "LocalizationFrameSummary",
    "LocalizationStatusResponse",
    "NavigationControlActiveSource",
    "NavigationControlSummary",
    "NavigationDiagnosticsSummary",
    "NavigationFeedbackSummary",
    "NavigationFrameSummary",
    "NavigationLocalizationSummary",
    "NavigationMissionSummary",
    "NavigationMotionSummary",
    "NavigationPathSummary",
    "NavigationProgressSummary",
    "NavigationReadinessSummary",
    "NavigationRuntimeBoundary",
    "NavigationSpeedPolicy",
    "NavigationStatusResponse",
    "NavigationTargetSummary",
)
