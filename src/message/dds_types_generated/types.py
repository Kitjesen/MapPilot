r"""Auto-generated DDS dataclass definitions.

Source IDL: src/message/idl/lingtu_slam.idl
Generated:  2026-08-02T05:11:02Z

DO NOT EDIT BY HAND -- regenerate with::

    python scripts/codegen/idl_to_python.py src/message/idl/lingtu_slam.idl --output <output_dir>/
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(kw_only=True)
class Time:
    r"""IDL struct: lingtu::dds::Time
    """
    sec: int
    nanosec: int


@dataclass(kw_only=True)
class Header:
    r"""IDL struct: lingtu::dds::Header
    """
    stamp: Time
    frame_id: str


@dataclass(kw_only=True)
class Vector3:
    r"""IDL struct: lingtu::dds::Vector3
    """
    x: float
    y: float
    z: float


@dataclass(kw_only=True)
class Quaternion:
    r"""IDL struct: lingtu::dds::Quaternion
    """
    x: float
    y: float
    z: float
    w: float


@dataclass(kw_only=True)
class Imu:
    r"""IDL struct: lingtu::dds::Imu
    """
    header: Header
    orientation: Quaternion
    orientation_covariance: list[float] = field(default_factory=list)
    angular_velocity: Vector3
    angular_velocity_covariance: list[float] = field(default_factory=list)
    linear_acceleration: Vector3
    linear_acceleration_covariance: list[float] = field(default_factory=list)


@dataclass(kw_only=True)
class LivoxPoint:
    r"""IDL struct: lingtu::dds::LivoxPoint
    """
    offset_time: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


@dataclass(kw_only=True)
class LivoxFrame:
    r"""IDL struct: lingtu::dds::LivoxFrame
    """
    header: Header
    timebase: int
    point_num: int
    lidar_id: int
    rsvd: list[int] = field(default_factory=list)
    points: list[LivoxPoint] = field(default_factory=list)


@dataclass(kw_only=True)
class Point:
    r"""IDL struct: lingtu::dds::Point
    """
    x: float
    y: float
    z: float


@dataclass(kw_only=True)
class Pose:
    r"""IDL struct: lingtu::dds::Pose
    """
    position: Point
    orientation: Quaternion


@dataclass(kw_only=True)
class PoseStamped:
    r"""IDL struct: lingtu::dds::PoseStamped
    """
    header: Header
    pose: Pose


@dataclass(kw_only=True)
class PoseWithCovariance:
    r"""IDL struct: lingtu::dds::PoseWithCovariance
    """
    pose: Pose
    covariance: list[float] = field(default_factory=list)


@dataclass(kw_only=True)
class Twist:
    r"""IDL struct: lingtu::dds::Twist
    """
    linear: Vector3
    angular: Vector3


@dataclass(kw_only=True)
class TwistStamped:
    r"""IDL struct: lingtu::dds::TwistStamped
    """
    header: Header
    twist: Twist


@dataclass(kw_only=True)
class FinalVelocityCommand:
    r"""IDL struct: lingtu::dds::FinalVelocityCommand
    """
    host_boot_id: str
    producer_boot_id: str
    output_seq: int
    source_boottime_ns: int
    source_wall_ns: int
    twist: Twist


@dataclass(kw_only=True)
class Transform:
    r"""IDL struct: lingtu::dds::Transform
    """
    translation: Vector3
    rotation: Quaternion


@dataclass(kw_only=True)
class TransformStamped:
    r"""IDL struct: lingtu::dds::TransformStamped
    """
    header: Header
    child_frame_id: str
    transform: Transform


@dataclass(kw_only=True)
class TFMessage:
    r"""IDL struct: lingtu::dds::TFMessage
    """
    transforms: list[TransformStamped] = field(default_factory=list)


@dataclass(kw_only=True)
class TwistWithCovariance:
    r"""IDL struct: lingtu::dds::TwistWithCovariance
    """
    twist: Twist
    covariance: list[float] = field(default_factory=list)


@dataclass(kw_only=True)
class Odometry:
    r"""IDL struct: lingtu::dds::Odometry
    """
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


@dataclass(kw_only=True)
class PointField:
    r"""IDL struct: lingtu::dds::PointField
    """
    name: str
    offset: int
    datatype: int
    count: int


@dataclass(kw_only=True)
class PointCloud2:
    r"""IDL struct: lingtu::dds::PointCloud2
    """
    header: Header
    height: int
    width: int
    fields: list[PointField] = field(default_factory=list)
    is_bigendian: bool
    point_step: int
    row_step: int
    data: list[int] = field(default_factory=list)
    is_dense: bool


@dataclass(kw_only=True)
class MapObservation:
    r"""IDL struct: lingtu::dds::MapObservation
    """
    header: Header
    observation_sequence: int
    reset_epoch: int
    sensor_frame: str
    map_sensor: Transform
    sensor_origin: Vector3
    scan: PointCloud2
    pose_confidence: float
    localization_quality: float
    pose_state: str
    pose_reason: str


@dataclass(kw_only=True)
class MapArtifactIdentity:
    r"""IDL struct: lingtu::MapArtifactIdentity
    """
    type: str
    uri: str
    sha256: str


@dataclass(kw_only=True)
class MapIdentity:
    r"""IDL struct: lingtu::MapIdentity
    """
    present: bool
    map_id: str
    version_id: str
    frame_id: str
    map_dir: str
    artifacts: list[MapArtifactIdentity] = field(default_factory=list)


@dataclass(kw_only=True)
class MapActivationRequest:
    r"""IDL struct: lingtu::MapActivationRequest
    """
    request_id: str
    operation: MapActivationOperation
    target: MapIdentity
    previous: MapIdentity
    caller: str
    reason: str


@dataclass(kw_only=True)
class MapActivationAck:
    r"""IDL struct: lingtu::MapActivationAck
    """
    request_id: str
    operation: MapActivationOperation
    accepted: bool
    message: str
    changed: bool
    target: MapIdentity
    previous: MapIdentity
    active: MapIdentity
    producer_boot_id: str


@dataclass(kw_only=True)
class Image:
    r"""IDL struct: lingtu::Image
    """
    header: Header
    height: int
    width: int
    encoding: str
    is_bigendian: bool
    step: int
    data: list[int] = field(default_factory=list)


@dataclass(kw_only=True)
class CameraInfo:
    r"""IDL struct: lingtu::CameraInfo
    """
    header: Header
    height: int
    width: int
    depth_scale: float
    distortion_model: str
    d: list[float] = field(default_factory=list)
    k: list[float] = field(default_factory=list)
    r: list[float] = field(default_factory=list)
    p: list[float] = field(default_factory=list)


@dataclass(kw_only=True)
class GnssFix:
    r"""IDL struct: lingtu::GnssFix
    """
    header: Header
    latitude: float
    longitude: float
    altitude: float
    fix_type: int
    position_covariance: list[float] = field(default_factory=list)
    num_sat: int
    num_sat_used: int
    hdop: float
    rtcm_age_s: float


@dataclass(kw_only=True)
class GnssStatus:
    r"""IDL struct: lingtu::GnssStatus
    """
    header: Header
    device: str
    fix_type: int
    link_ok: bool
    rtk: bool
    num_sat: int
    num_sat_used: int
    hdop: float
    rtcm_age_s: float
    error: str


@dataclass(kw_only=True)
class DriverControlState:
    r"""IDL struct: lingtu::DriverControlState
    """
    header: Header
    connected: bool
    ready: bool
    motors_enabled: bool
    critical_fault: bool
    lease_valid: bool
    lease_remaining_ms: int
    accepted_sequence: int
    accepted_producer_boot_id: str
    accepted_output_sequence: int
    last_command_accepted: bool
    fsm: str
    owner: str
    owner_id: str
    reason: str


@dataclass(kw_only=True)
class Float32:
    r"""IDL struct: lingtu::Float32
    """
    data: float


@dataclass(kw_only=True)
class Bool:
    r"""IDL struct: lingtu::Bool
    """
    data: bool


@dataclass(kw_only=True)
class Text:
    r"""IDL struct: lingtu::Text
    """
    data: str


@dataclass(kw_only=True)
class NavigationCommandRequest:
    r"""IDL struct: lingtu::NavigationCommandRequest
    """
    header: Header
    client_id: str
    task_id: str
    request_id: str
    kind: int
    goal: Pose
    reason: str


@dataclass(kw_only=True)
class NavigationCommandAck:
    r"""IDL struct: lingtu::NavigationCommandAck
    """
    header: Header
    task_id: str
    request_id: str
    kind: int
    accepted: bool
    duplicate: bool
    reason: str


@dataclass(kw_only=True)
class OperatorMotionControl:
    r"""IDL struct: lingtu::OperatorMotionControl
    """
    header: Header
    source_id: str
    source_epoch: int
    source_sequence: int
    request_id: str
    action: int
    lease_ttl_ms: int
    reason: str


@dataclass(kw_only=True)
class OperatorMotionSample:
    r"""IDL struct: lingtu::OperatorMotionSample
    """
    header: Header
    source_id: str
    source_epoch: int
    source_sequence: int
    request_id: str
    deadman: bool
    velocity: Twist
    freshness_budget_ms: int
    source_stamp_ns: int


@dataclass(kw_only=True)
class OperatorMotionAck:
    r"""IDL struct: lingtu::OperatorMotionAck
    """
    header: Header
    source_id: str
    source_epoch: int
    source_sequence: int
    request_id: str
    action: int
    accepted: bool
    reason: str
    accepted_sequence: int
    final_output_sequence: int


@dataclass(kw_only=True)
class OperatorMotionStatus:
    r"""IDL struct: lingtu::OperatorMotionStatus
    """
    header: Header
    active_source_id: str
    active_source_epoch: int
    has_active_authority: bool
    holding: bool
    has_active_sample: bool
    last_sample_sequence: int
    admitted_sequence: int
    final_output_sequence: int
    authority_reason: str
    input_gate_reason: str
    teleop_output: Twist
    final_cmd_vel: Twist


@dataclass(kw_only=True)
class NavigationGoalStatus:
    r"""IDL struct: lingtu::NavigationGoalStatus
    """
    header: Header
    boot_id: str
    event_sequence: int
    task_id: str
    request_id: str
    state: int
    goal_epoch: int
    reason: str


@dataclass(kw_only=True)
class NavigationState:
    r"""IDL struct: lingtu::NavigationState
    """
    header: Header
    boot_id: str
    state_sequence: int
    control_mode: int
    lifecycle_state: int
    active_task_id: str
    active_request_id: str
    goal_epoch: int
    map_id: str
    map_version: int
    map_hash: str
    planning_state: int
    execution_state: int
    recovery_state: int
    progress: float
    authority: str
    hold_reason: str
    failure_code: str


@dataclass(kw_only=True)
class ExplorationCommandRequest:
    r"""IDL struct: lingtu::ExplorationCommandRequest
    """
    header: Header
    request_id: str
    exploration_run_id: str
    kind: int
    session_id: str
    has_directed_target: bool
    directed_target_x: float
    directed_target_y: float
    directed_target_ttl_s: float
    reason: str


@dataclass(kw_only=True)
class ExplorationCommandAck:
    r"""IDL struct: lingtu::ExplorationCommandAck
    """
    header: Header
    request_id: str
    exploration_run_id: str
    kind: int
    accepted: bool
    duplicate: bool
    reason: str
    session_id: str
    intent_revision: int


@dataclass(kw_only=True)
class ExplorationRunEvent:
    r"""IDL struct: lingtu::ExplorationRunEvent
    """
    timestamp_s: float
    frame_id: str
    boot_id: str
    event_sequence: int
    kind: int
    exploration_run_id: str
    start_request_id: str
    command_request_id: str
    product_session_id: str
    state: int
    route: str
    map_id: str
    map_version: int
    artifact_hash: str
    reason: str
    motion_stop_confirmed: bool
    motion_stop_reason: str


@dataclass(kw_only=True)
class ExplorationSegmentRequest:
    r"""IDL struct: lingtu::ExplorationSegmentRequest
    """
    header: Header
    request_id: str
    kind: int
    session_id: str
    reset_epoch: int
    minimum_generation: int
    target: Pose
    reason: str


@dataclass(kw_only=True)
class ExplorationSegmentAck:
    r"""IDL struct: lingtu::ExplorationSegmentAck
    """
    header: Header
    request_id: str
    kind: int
    accepted: bool
    session_id: str
    reset_epoch: int
    generation: int
    live: bool
    reason: str


@dataclass(kw_only=True)
class ExplorationSegmentStatus:
    r"""IDL struct: lingtu::ExplorationSegmentStatus
    """
    header: Header
    request_id: str
    state: int
    session_id: str
    reset_epoch: int
    generation: int
    live: bool
    reason: str


@dataclass(kw_only=True)
class InspectionTaskRequest:
    r"""IDL struct: lingtu::InspectionTaskRequest
    """
    header: Header
    task_id: str
    request_id: str
    kind: int
    route_id: str
    route_revision: int
    reason: str


@dataclass(kw_only=True)
class InspectionTaskAck:
    r"""IDL struct: lingtu::InspectionTaskAck
    """
    header: Header
    task_id: str
    request_id: str
    kind: int
    accepted: bool
    reason: str
    run_id: str


@dataclass(kw_only=True)
class InspectionEvidenceRequest:
    r"""IDL struct: lingtu::InspectionEvidenceRequest
    """
    header: Header
    request_id: str
    run_id: str
    route_id: str
    revision: int
    map_id: str
    map_version: int
    point_index: int
    point_id: str
    action: str
    deadline_s: float


@dataclass(kw_only=True)
class InspectionEvidenceResult:
    r"""IDL struct: lingtu::InspectionEvidenceResult
    """
    header: Header
    request_id: str
    evidence_id: str
    persisted: bool
    reason: str
    analysis_verdict: str


@dataclass(kw_only=True)
class InspectionStatus:
    r"""IDL struct: lingtu::InspectionStatus
    """
    header: Header
    run_id: str
    route_id: str
    route_revision: int
    state: int
    point_index: int
    point_count: int
    loop_index: int
    retry_count: int
    point_id: str
    action: str
    action_request_id: str
    evidence_id: str
    phase_started_at: float
    stable_since: float
    deadline: float
    reason: str


@dataclass(kw_only=True)
class InspectionTaskEvent:
    r"""IDL struct: lingtu::InspectionTaskEvent
    """
    header: Header
    boot_id: str
    event_sequence: int
    kind: int
    task_id: str
    request_id: str
    command_request_id: str
    state: int
    map_id: str
    map_version: int
    route_id: str
    route_revision: int
    point_index: int
    point_count: int
    loop_index: int
    retry_count: int
    point_id: str
    action: str
    action_request_id: str
    evidence_id: str
    reason: str


@dataclass(kw_only=True)
class RelocalizationRequest:
    r"""IDL struct: lingtu::RelocalizationRequest
    """
    request_id: str
    action: str
    engine: str
    map_path: str
    has_initial_pose: bool
    initial_pose: Pose
    timeout_s: float


@dataclass(kw_only=True)
class RelocalizationResponse:
    r"""IDL struct: lingtu::RelocalizationResponse
    """
    request_id: str
    action: str
    engine: str
    success: bool
    message: str
    quality: float
    map_loaded: bool
    has_map_body: bool
    map_body: Pose
    has_map_odom: bool
    map_odom: Pose
    state: str
    refine_backend: str
    refine_iterations: int
    refine_inliers: int
    refine_input_points: int
    refine_evaluated_points: int
    refine_support_ratio: float
    refine_overlap_inlier_ratio: float
    refine_converged: bool
    refine_pos_cov_trace: float
    track_against_map_supported: bool
    track_against_map_enabled: bool
    track_against_map_failures: int


@dataclass(kw_only=True)
class Path:
    r"""IDL struct: lingtu::Path
    """
    header: Header
    poses: list[PoseStamped] = field(default_factory=list)


@dataclass(kw_only=True)
class MapMetaData:
    r"""IDL struct: lingtu::MapMetaData
    """
    map_load_time: Time
    resolution: float
    width: int
    height: int
    origin: Pose


@dataclass(kw_only=True)
class ExplorationExecutionGrid:
    r"""IDL struct: lingtu::ExplorationExecutionGrid
    """
    header: Header
    info: MapMetaData
    occupancy: list[int] = field(default_factory=list)
    terrain_cost: list[int] = field(default_factory=list)
    session_id: str
    reset_epoch: int
    generation: int
    live: bool
    terrain_risk_stamp: Time
    terrain_risk_ready: bool


@dataclass(kw_only=True)
class OccupancyGrid:
    r"""IDL struct: lingtu::OccupancyGrid
    """
    header: Header
    info: MapMetaData
    data: list[int] = field(default_factory=list)


@dataclass(kw_only=True)
class MapCloudLayer:
    r"""IDL struct: lingtu::MapCloudLayer
    """
    header: Header
    layer: str
    reset_epoch: int
    observation_sequence: int
    generation: int
    live: bool
    cloud: PointCloud2


@dataclass(kw_only=True)
class MapGrid:
    r"""IDL struct: lingtu::MapGrid
    """
    header: Header
    layer: str
    info: MapMetaData
    data: list[float] = field(default_factory=list)
    reset_epoch: int
    observation_sequence: int
    generation: int
    live: bool


@dataclass(kw_only=True)
class MapRuntimeState:
    r"""IDL struct: lingtu::MapRuntimeState
    """
    header: Header
    producer_boot_id: str
    active_map: MapIdentity
    running: bool
    live: bool
    reset_epoch: int
    observation_sequence: int
    generation: int
    accepted_observations: int
    processed_observations: int
    replaced_observations: int
    stale_observations: int
    invalid_observations: int
    epoch_resets: int
    queue_depth: int
    live_points: int
    voxel_points: int
    voxel_cells: int
    voxel_snapshot_omitted_cells: int
    voxel_capacity_rejections: int
    accumulated_cells: int
    accumulated_snapshot_cells: int
    accumulated_capacity_rejections: int
    capacity_limited: bool
    pose_quality: float
    pose_state: str
    pose_reason: str
    dds_received_samples: int
    dds_decoded_samples: int
    dds_rejected_samples: int
    dds_write_attempts: int
    dds_write_failures: int
    dds_serialization_rejections: int
    dds_scene_oversize_rejections: int
    dds_unhealthy_writers: int
    required_publications_ready: bool
    current_generation_published: bool
    state_published_generation: int
    realtime_clouds_published_generation: int
    map_layers_published_generation: int
    scene_published_generation: int
    engine_error: str
    input_error: str
    output_error: str


@dataclass(kw_only=True)
class MapScene:
    r"""IDL struct: lingtu::MapScene
    """
    header: Header
    producer_boot_id: str
    reset_epoch: int
    observation_sequence: int
    generation: int
    live: bool
    map_sensor: Pose
    live_cloud: MapCloudLayer
    voxel_cloud: MapCloudLayer
    accumulated_cloud: MapCloudLayer
    occupancy: MapGrid
    elevation: MapGrid
    esdf: MapGrid


@dataclass(kw_only=True)
class ExplorationGrid:
    r"""IDL struct: lingtu::ExplorationGrid
    """
    header: Header
    info: MapMetaData
    data: list[int] = field(default_factory=list)
    session_id: str
    map_id: str
    map_version: int
    artifact_hash: str
    reset_epoch: int
    generation: int
    live: bool
