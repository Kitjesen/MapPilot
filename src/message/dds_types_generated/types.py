r"""Auto-generated DDS dataclass definitions.

Source IDL: src\message\idl\lingtu_slam.idl
Generated:  2026-07-20T05:28:09Z

DO NOT EDIT BY HAND -- regenerate with::

    python scripts/codegen/idl_to_python.py src\message\idl\lingtu_slam.idl --output <output_dir>/
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
class Image:
    r"""IDL struct: lingtu::dds::Image
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
    r"""IDL struct: lingtu::dds::CameraInfo
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
    r"""IDL struct: lingtu::dds::GnssFix
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
    r"""IDL struct: lingtu::dds::GnssStatus
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
    r"""IDL struct: lingtu::dds::DriverControlState
    """
    header: Header
    connected: bool
    ready: bool
    motors_enabled: bool
    critical_fault: bool
    lease_valid: bool
    lease_remaining_ms: int
    accepted_sequence: int
    last_command_accepted: bool
    fsm: str
    owner: str
    owner_id: str
    reason: str


@dataclass(kw_only=True)
class Float32:
    r"""IDL struct: lingtu::dds::Float32
    """
    data: float


@dataclass(kw_only=True)
class Bool:
    r"""IDL struct: lingtu::dds::Bool
    """
    data: bool


@dataclass(kw_only=True)
class Text:
    r"""IDL struct: lingtu::dds::Text
    """
    data: str


@dataclass(kw_only=True)
class NavigationCommandRequest:
    r"""IDL struct: lingtu::dds::NavigationCommandRequest
    """
    header: Header
    request_id: str
    kind: int
    goal: Pose
    velocity: Twist
    reason: str


@dataclass(kw_only=True)
class NavigationCommandAck:
    r"""IDL struct: lingtu::dds::NavigationCommandAck
    """
    header: Header
    request_id: str
    kind: int
    accepted: bool
    reason: str


@dataclass(kw_only=True)
class ExplorationCommandRequest:
    r"""IDL struct: lingtu::dds::ExplorationCommandRequest
    """
    header: Header
    request_id: str
    kind: int
    session_id: str
    reason: str


@dataclass(kw_only=True)
class ExplorationCommandAck:
    r"""IDL struct: lingtu::dds::ExplorationCommandAck
    """
    header: Header
    request_id: str
    kind: int
    accepted: bool
    reason: str
    session_id: str


@dataclass(kw_only=True)
class InspectionCommandRequest:
    r"""IDL struct: lingtu::dds::InspectionCommandRequest
    """
    header: Header
    request_id: str
    kind: int
    route_id: str
    route_revision: int
    reason: str


@dataclass(kw_only=True)
class InspectionCommandAck:
    r"""IDL struct: lingtu::dds::InspectionCommandAck
    """
    header: Header
    request_id: str
    kind: int
    accepted: bool
    reason: str
    run_id: str


@dataclass(kw_only=True)
class InspectionEvidenceRequest:
    r"""IDL struct: lingtu::dds::InspectionEvidenceRequest
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
    r"""IDL struct: lingtu::dds::InspectionEvidenceResult
    """
    header: Header
    request_id: str
    evidence_id: str
    persisted: bool
    reason: str
    analysis_verdict: str


@dataclass(kw_only=True)
class InspectionStatus:
    r"""IDL struct: lingtu::dds::InspectionStatus
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
class RelocalizationRequest:
    r"""IDL struct: lingtu::dds::RelocalizationRequest
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
    r"""IDL struct: lingtu::dds::RelocalizationResponse
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
    refine_converged: bool
    refine_pos_cov_trace: float
    track_against_map_supported: bool
    track_against_map_enabled: bool
    track_against_map_failures: int


@dataclass(kw_only=True)
class Path:
    r"""IDL struct: lingtu::dds::Path
    """
    header: Header
    poses: list[PoseStamped] = field(default_factory=list)


@dataclass(kw_only=True)
class MapMetaData:
    r"""IDL struct: lingtu::dds::MapMetaData
    """
    map_load_time: Time
    resolution: float
    width: int
    height: int
    origin: Pose


@dataclass(kw_only=True)
class OccupancyGrid:
    r"""IDL struct: lingtu::dds::OccupancyGrid
    """
    header: Header
    info: MapMetaData
    data: list[int] = field(default_factory=list)


@dataclass(kw_only=True)
class ExplorationGrid:
    r"""IDL struct: lingtu::dds::ExplorationGrid
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
