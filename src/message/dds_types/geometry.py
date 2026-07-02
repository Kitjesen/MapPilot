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
class PoseStamped(IdlStruct, typename="lingtu::dds::PoseStamped"):
    header: Header
    pose: Pose


@dataclass
class TwistStamped(IdlStruct, typename="lingtu::dds::TwistStamped"):
    header: Header
    twist: Twist


@dataclass
class Transform(IdlStruct, typename="lingtu::dds::Transform"):
    translation: Vector3
    rotation: Quaternion


@dataclass
class TransformStamped(IdlStruct, typename="lingtu::dds::TransformStamped"):
    header: Header
    child_frame_id: str
    transform: Transform


@dataclass
class TFMessage(IdlStruct, typename="lingtu::dds::TFMessage"):
    transforms: types.sequence[TransformStamped]


DDS_Pose = Pose
DDS_PoseWithCovariance = PoseWithCovariance
DDS_Twist = Twist
DDS_TwistWithCovariance = TwistWithCovariance
DDS_PoseStamped = PoseStamped
DDS_TwistStamped = TwistStamped
DDS_Transform = Transform
DDS_TransformStamped = TransformStamped
DDS_TFMessage = TFMessage
