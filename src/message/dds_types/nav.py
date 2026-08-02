"""Native LingTu DDS navigation/map types used by Python adapters."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header, Time, Vector3
from .geometry import Pose, PoseStamped, PoseWithCovariance, Transform, Twist, TwistWithCovariance
from .pointcloud import PointCloud2


@dataclass
class Odometry(IdlStruct, typename="lingtu::dds::Odometry"):
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


@dataclass
class Path(IdlStruct, typename="lingtu::dds::Path"):
    header: Header
    poses: types.sequence[PoseStamped]


@dataclass
class MapMetaData(IdlStruct, typename="lingtu::dds::MapMetaData"):
    map_load_time: Time
    resolution: types.float32
    width: types.uint32
    height: types.uint32
    origin: Pose


@dataclass
class OccupancyGrid(IdlStruct, typename="lingtu::dds::OccupancyGrid"):
    header: Header
    info: MapMetaData
    data: types.sequence[types.int8]


@dataclass
class MapObservation(IdlStruct, typename="lingtu::dds::MapObservation"):
    header: Header
    observation_sequence: types.uint64
    reset_epoch: types.uint64
    sensor_frame: str
    map_sensor: Transform
    sensor_origin: Vector3
    scan: PointCloud2
    pose_confidence: types.float32
    localization_quality: types.float32
    pose_state: str
    pose_reason: str


@dataclass
class MapArtifactIdentity(IdlStruct, typename="lingtu::dds::MapArtifactIdentity"):
    type: str
    uri: str
    sha256: str


@dataclass
class MapIdentity(IdlStruct, typename="lingtu::dds::MapIdentity"):
    present: types.boolean
    map_id: str
    version_id: str
    frame_id: str
    map_dir: str
    artifacts: types.sequence[MapArtifactIdentity]


@dataclass
class MapActivationRequest(IdlStruct, typename="lingtu::dds::MapActivationRequest"):
    request_id: str
    operation: types.int32
    target: MapIdentity
    previous: MapIdentity
    caller: str
    reason: str


@dataclass
class MapActivationAck(IdlStruct, typename="lingtu::dds::MapActivationAck"):
    request_id: str
    operation: types.int32
    accepted: types.boolean
    message: str
    changed: types.boolean
    target: MapIdentity
    previous: MapIdentity
    active: MapIdentity
    producer_boot_id: str


@dataclass
class MapCloudLayer(IdlStruct, typename="lingtu::dds::MapCloudLayer"):
    header: Header
    layer: str
    cloud: PointCloud2
    reset_epoch: types.uint64
    observation_sequence: types.uint64
    generation: types.uint64
    live: types.boolean


@dataclass
class MapGrid(IdlStruct, typename="lingtu::dds::MapGrid"):
    header: Header
    layer: str
    info: MapMetaData
    data: types.sequence[types.float32]
    reset_epoch: types.uint64
    observation_sequence: types.uint64
    generation: types.uint64
    live: types.boolean


@dataclass
class MapRuntimeState(IdlStruct, typename="lingtu::dds::MapRuntimeState"):
    header: Header
    producer_boot_id: str
    active_map: MapIdentity
    running: types.boolean
    live: types.boolean
    reset_epoch: types.uint64
    observation_sequence: types.uint64
    generation: types.uint64
    accepted_observations: types.uint64
    processed_observations: types.uint64
    replaced_observations: types.uint64
    stale_observations: types.uint64
    invalid_observations: types.uint64
    epoch_resets: types.uint64
    queue_depth: types.uint32
    live_points: types.uint64
    voxel_points: types.uint64
    voxel_cells: types.uint64
    voxel_snapshot_omitted_cells: types.uint64
    voxel_capacity_rejections: types.uint64
    accumulated_cells: types.uint64
    accumulated_snapshot_cells: types.uint64
    accumulated_capacity_rejections: types.uint64
    capacity_limited: types.boolean
    pose_quality: types.float32
    pose_state: str
    pose_reason: str
    dds_received_samples: types.uint64
    dds_decoded_samples: types.uint64
    dds_rejected_samples: types.uint64
    dds_write_attempts: types.uint64
    dds_write_failures: types.uint64
    dds_serialization_rejections: types.uint64
    dds_scene_oversize_rejections: types.uint64
    dds_unhealthy_writers: types.uint32
    required_publications_ready: types.boolean
    current_generation_published: types.boolean
    state_published_generation: types.uint64
    realtime_clouds_published_generation: types.uint64
    map_layers_published_generation: types.uint64
    scene_published_generation: types.uint64
    engine_error: str
    input_error: str
    output_error: str


@dataclass
class MapScene(IdlStruct, typename="lingtu::dds::MapScene"):
    header: Header
    producer_boot_id: str
    reset_epoch: types.uint64
    observation_sequence: types.uint64
    generation: types.uint64
    live: types.boolean
    map_sensor: Pose
    live_cloud: MapCloudLayer
    voxel_cloud: MapCloudLayer
    accumulated_cloud: MapCloudLayer
    occupancy: MapGrid
    elevation: MapGrid
    esdf: MapGrid


@dataclass
class ExplorationGrid(IdlStruct, typename="lingtu::dds::ExplorationGrid"):
    header: Header
    info: MapMetaData
    data: types.sequence[types.int8]
    session_id: str
    map_id: str
    map_version: int
    artifact_hash: str
    reset_epoch: types.uint64
    generation: types.uint64
    live: types.boolean

@dataclass
class ExplorationExecutionGrid(IdlStruct, typename="lingtu::dds::ExplorationExecutionGrid"):
    header: Header
    info: MapMetaData
    occupancy: types.sequence[types.int8]
    terrain_cost: types.sequence[types.int8]
    session_id: str
    reset_epoch: types.uint64
    generation: types.uint64
    live: types.boolean
    terrain_risk_stamp: Time
    terrain_risk_ready: types.boolean




@dataclass
class NavigationCommandRequest(
    IdlStruct,
    typename="lingtu::dds::NavigationCommandRequest",
):
    header: Header
    client_id: str
    task_id: str
    request_id: str
    kind: types.int32
    goal: Pose
    reason: str


@dataclass
class NavigationCommandAck(
    IdlStruct,
    typename="lingtu::dds::NavigationCommandAck",
):
    header: Header
    task_id: str
    request_id: str
    kind: types.int32
    accepted: types.boolean
    reason: str


@dataclass
class OperatorMotionControl(IdlStruct, typename="lingtu::dds::OperatorMotionControl"):
    header: Header
    source_id: str
    source_epoch: types.uint64
    source_sequence: types.uint64
    request_id: str
    action: types.int32
    lease_ttl_ms: types.uint32
    reason: str


@dataclass
class OperatorMotionSample(IdlStruct, typename="lingtu::dds::OperatorMotionSample"):
    header: Header
    source_id: str
    source_epoch: types.uint64
    source_sequence: types.uint64
    request_id: str
    deadman: types.boolean
    velocity: Twist
    freshness_budget_ms: types.uint32
    source_stamp_ns: types.uint64


@dataclass
class OperatorMotionAck(IdlStruct, typename="lingtu::dds::OperatorMotionAck"):
    header: Header
    source_id: str
    source_epoch: types.uint64
    source_sequence: types.uint64
    request_id: str
    action: types.int32
    accepted: types.boolean
    reason: str
    accepted_sequence: types.uint64
    final_output_sequence: types.uint64


@dataclass
class OperatorMotionStatus(IdlStruct, typename="lingtu::dds::OperatorMotionStatus"):
    header: Header
    active_source_id: str
    active_source_epoch: types.uint64
    has_active_authority: types.boolean
    holding: types.boolean
    has_active_sample: types.boolean
    last_sample_sequence: types.uint64
    admitted_sequence: types.uint64
    final_output_sequence: types.uint64
    authority_reason: str
    input_gate_reason: str
    teleop_output: Twist
    final_cmd_vel: Twist


@dataclass
class NavigationGoalStatus(
    IdlStruct,
    typename="lingtu::dds::NavigationGoalStatus",
):
    header: Header
    boot_id: str
    event_sequence: types.uint64
    task_id: str
    request_id: str
    state: types.int32
    goal_epoch: types.uint64
    reason: str


@dataclass
class NavigationState(IdlStruct, typename="lingtu::dds::NavigationState"):
    header: Header
    boot_id: str
    state_sequence: types.uint64
    control_mode: types.int32
    lifecycle_state: types.int32
    active_task_id: str
    active_request_id: str
    goal_epoch: types.uint64
    map_id: str
    map_version: types.int64
    map_hash: str
    planning_state: types.int32
    execution_state: types.int32
    recovery_state: types.int32
    progress: types.float32
    authority: str
    hold_reason: str
    failure_code: str


@dataclass
class ExplorationCommandRequest(
    IdlStruct,
    typename="lingtu::dds::ExplorationCommandRequest",
):
    header: Header
    request_id: str
    exploration_run_id: str
    kind: types.int32
    session_id: str
    has_directed_target: types.boolean
    directed_target_x: types.float64
    directed_target_y: types.float64
    directed_target_ttl_s: types.float64
    reason: str


@dataclass
class ExplorationSegmentRequest(IdlStruct, typename="lingtu::dds::ExplorationSegmentRequest"):
    header: Header
    request_id: str
    kind: types.int32
    session_id: str
    reset_epoch: types.uint64
    minimum_generation: types.uint64
    target: Pose
    reason: str


@dataclass
class ExplorationSegmentAck(IdlStruct, typename="lingtu::dds::ExplorationSegmentAck"):
    header: Header
    request_id: str
    kind: types.int32
    accepted: types.boolean
    session_id: str
    reset_epoch: types.uint64
    generation: types.uint64
    live: types.boolean
    reason: str


@dataclass
class ExplorationSegmentStatus(IdlStruct, typename="lingtu::dds::ExplorationSegmentStatus"):
    header: Header
    request_id: str
    state: types.int32
    session_id: str
    reset_epoch: types.uint64
    generation: types.uint64
    live: types.boolean
    reason: str


@dataclass
class ExplorationCommandAck(
    IdlStruct,
    typename="lingtu::dds::ExplorationCommandAck",
):
    header: Header
    request_id: str
    exploration_run_id: str
    kind: types.int32
    accepted: types.boolean
    duplicate: types.boolean
    reason: str
    session_id: str
    intent_revision: types.uint64


@dataclass
class ExplorationRunEvent(IdlStruct, typename="lingtu::dds::ExplorationRunEvent"):
    """Immutable ordered fact for one finite native exploration execution."""

    timestamp_s: types.float64
    frame_id: str
    boot_id: str
    event_sequence: types.uint64
    kind: types.int32
    exploration_run_id: str
    start_request_id: str
    command_request_id: str
    product_session_id: str
    state: types.int32
    route: str
    map_id: str
    map_version: types.int64
    artifact_hash: str
    reason: str
    motion_stop_confirmed: types.boolean
    motion_stop_reason: str


@dataclass
class InspectionTaskRequest(
    IdlStruct,
    typename="lingtu::dds::InspectionTaskRequest",
):
    """A task-addressed inspection lifecycle command from the product boundary."""

    header: Header
    task_id: str
    request_id: str
    kind: types.int32
    route_id: str
    route_revision: types.uint64
    reason: str


@dataclass
class InspectionTaskAck(
    IdlStruct,
    typename="lingtu::dds::InspectionTaskAck",
):
    """Native admission result preserving both task and request identity."""

    header: Header
    task_id: str
    request_id: str
    kind: types.int32
    accepted: types.boolean
    reason: str
    run_id: str


@dataclass
class InspectionEvidenceRequest(
    IdlStruct,
    typename="lingtu::dds::InspectionEvidenceRequest",
):
    header: Header
    request_id: str
    run_id: str
    route_id: str
    revision: types.uint64
    map_id: str
    map_version: int
    point_index: types.uint32
    point_id: str
    action: str
    deadline_s: types.float64


@dataclass
class InspectionEvidenceResult(
    IdlStruct,
    typename="lingtu::dds::InspectionEvidenceResult",
):
    header: Header
    request_id: str
    evidence_id: str
    persisted: types.boolean
    reason: str
    analysis_verdict: str


@dataclass
class InspectionStatus(IdlStruct, typename="lingtu::dds::InspectionStatus"):
    header: Header
    run_id: str
    route_id: str
    route_revision: types.uint64
    state: types.int32
    point_index: types.uint32
    point_count: types.uint32
    loop_index: types.uint32
    retry_count: types.uint32
    point_id: str
    action: str
    action_request_id: str
    evidence_id: str
    phase_started_at: types.float64
    stable_since: types.float64
    deadline: types.float64
    reason: str


@dataclass
class InspectionTaskEvent(IdlStruct, typename="lingtu::dds::InspectionTaskEvent"):
    """Immutable native fact for one inspection task lifecycle transition."""

    header: Header
    boot_id: str
    event_sequence: types.uint64
    kind: types.int32
    task_id: str
    request_id: str
    command_request_id: str
    state: types.int32
    map_id: str
    map_version: int
    route_id: str
    route_revision: types.uint64
    point_index: types.uint32
    point_count: types.uint32
    loop_index: types.uint32
    retry_count: types.uint32
    point_id: str
    action: str
    action_request_id: str
    evidence_id: str
    reason: str

DDS_ExplorationExecutionGrid = ExplorationExecutionGrid
DDS_ExplorationSegmentRequest = ExplorationSegmentRequest
DDS_ExplorationSegmentAck = ExplorationSegmentAck
DDS_ExplorationSegmentStatus = ExplorationSegmentStatus

DDS_Odometry = Odometry
DDS_Path = Path
DDS_MapMetaData = MapMetaData
DDS_OccupancyGrid = OccupancyGrid
DDS_MapObservation = MapObservation
DDS_MapCloudLayer = MapCloudLayer
DDS_MapArtifactIdentity = MapArtifactIdentity
DDS_MapIdentity = MapIdentity
DDS_MapActivationRequest = MapActivationRequest
DDS_MapActivationAck = MapActivationAck
DDS_MapGrid = MapGrid
DDS_MapRuntimeState = MapRuntimeState
DDS_MapScene = MapScene
DDS_ExplorationGrid = ExplorationGrid
DDS_NavigationCommandRequest = NavigationCommandRequest
DDS_NavigationCommandAck = NavigationCommandAck
DDS_OperatorMotionControl = OperatorMotionControl
DDS_OperatorMotionSample = OperatorMotionSample
DDS_OperatorMotionAck = OperatorMotionAck
DDS_OperatorMotionStatus = OperatorMotionStatus
DDS_NavigationGoalStatus = NavigationGoalStatus
DDS_NavigationState = NavigationState
DDS_ExplorationCommandRequest = ExplorationCommandRequest
DDS_ExplorationCommandAck = ExplorationCommandAck
DDS_ExplorationRunEvent = ExplorationRunEvent
DDS_InspectionTaskRequest = InspectionTaskRequest
DDS_InspectionTaskAck = InspectionTaskAck
DDS_InspectionEvidenceRequest = InspectionEvidenceRequest
DDS_InspectionEvidenceResult = InspectionEvidenceResult
DDS_InspectionStatus = InspectionStatus
DDS_InspectionTaskEvent = InspectionTaskEvent
