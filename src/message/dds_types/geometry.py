"""Native LingTu DDS geometry types."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header, Point, Quaternion, Vector3


@dataclass
class Pose(IdlStruct, typename="lingtu::dds::Pose"):
    position: Point
    orientation: Quaternion


@dataclass
class PoseWithCovariance(IdlStruct, typename="lingtu::dds::PoseWithCovariance"):
    pose: Pose
    covariance: types.array[types.float64, 36]


@dataclass
class Twist(IdlStruct, typename="lingtu::dds::Twist"):
    linear: Vector3
    angular: Vector3


@dataclass
class TwistWithCovariance(IdlStruct, typename="lingtu::dds::TwistWithCovariance"):
    twist: Twist
    covariance: types.array[types.float64, 36]


@dataclass
class PoseStamped(IdlStruct, typename="geometry_msgs::msg::dds_::PoseStamped_"):
    header: Header
    pose: Pose


@dataclass
class TwistStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TwistStamped_"):
    header: Header
    twist: Twist


DDS_Pose = Pose
DDS_PoseWithCovariance = PoseWithCovariance
DDS_Twist = Twist
DDS_TwistWithCovariance = TwistWithCovariance
DDS_PoseStamped = PoseStamped
DDS_TwistStamped = TwistStamped
