"""Native LingTu DDS navigation/map types used by Python adapters."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header, Time
from .geometry import Pose, PoseStamped, PoseWithCovariance, Twist, TwistWithCovariance


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
class NavigationCommandRequest(
    IdlStruct,
    typename="lingtu::dds::NavigationCommandRequest",
):
    header: Header
    request_id: str
    kind: types.int32
    goal: Pose
    velocity: Twist
    reason: str


@dataclass
class NavigationCommandAck(
    IdlStruct,
    typename="lingtu::dds::NavigationCommandAck",
):
    header: Header
    request_id: str
    kind: types.int32
    accepted: types.boolean
    reason: str


@dataclass
class InspectionCommandRequest(
    IdlStruct,
    typename="lingtu::dds::InspectionCommandRequest",
):
    header: Header
    request_id: str
    kind: types.int32
    route_id: str
    route_revision: types.uint64
    reason: str


@dataclass
class InspectionCommandAck(
    IdlStruct,
    typename="lingtu::dds::InspectionCommandAck",
):
    header: Header
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


DDS_Odometry = Odometry
DDS_Path = Path
DDS_MapMetaData = MapMetaData
DDS_OccupancyGrid = OccupancyGrid
DDS_NavigationCommandRequest = NavigationCommandRequest
DDS_NavigationCommandAck = NavigationCommandAck
DDS_InspectionCommandRequest = InspectionCommandRequest
DDS_InspectionCommandAck = InspectionCommandAck
DDS_InspectionEvidenceRequest = InspectionEvidenceRequest
DDS_InspectionEvidenceResult = InspectionEvidenceResult
DDS_InspectionStatus = InspectionStatus
