"""Native LingTu DDS navigation/map types used by Python adapters."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header, Time
from .geometry import Pose, PoseStamped, PoseWithCovariance, TwistWithCovariance


@dataclass
class Odometry(IdlStruct, typename="lingtu::dds::Odometry"):
    header: Header
    child_frame_id: str
    pose: PoseWithCovariance
    twist: TwistWithCovariance


@dataclass
class Path(IdlStruct, typename="nav_msgs::msg::dds_::Path_"):
    header: Header
    poses: types.sequence[PoseStamped]


@dataclass
class MapMetaData(IdlStruct, typename="nav_msgs::msg::dds_::MapMetaData_"):
    map_load_time: Time
    resolution: types.float32
    width: types.uint32
    height: types.uint32
    origin: Pose


@dataclass
class OccupancyGrid(IdlStruct, typename="nav_msgs::msg::dds_::OccupancyGrid_"):
    header: Header
    info: MapMetaData
    data: types.sequence[types.int8]


DDS_Odometry = Odometry
DDS_Path = Path
DDS_MapMetaData = MapMetaData
DDS_OccupancyGrid = OccupancyGrid
