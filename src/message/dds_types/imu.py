"""Native LingTu DDS IMU type."""

from dataclasses import dataclass

from ._base import IdlStruct, types
from .common import Header, Quaternion, Vector3


@dataclass
class Imu(IdlStruct, typename="lingtu::dds::Imu"):
    header: Header
    orientation: Quaternion
    orientation_covariance: types.array[types.float64, 9]
    angular_velocity: Vector3
    angular_velocity_covariance: types.array[types.float64, 9]
    linear_acceleration: Vector3
    linear_acceleration_covariance: types.array[types.float64, 9]


DDS_Imu = Imu


def dds_imu_to_imu(msg):
    from runtime.msgs.geometry import Quaternion as RuntimeQuaternion
    from runtime.msgs.geometry import Vector3 as RuntimeVector3
    from runtime.msgs.sensor import Imu as RuntimeImu

    stamp = msg.header.stamp
    o = msg.orientation
    av = msg.angular_velocity
    la = msg.linear_acceleration
    return RuntimeImu(
        orientation=RuntimeQuaternion(o.x, o.y, o.z, o.w),
        orientation_covariance=list(msg.orientation_covariance),
        angular_velocity=RuntimeVector3(av.x, av.y, av.z),
        angular_velocity_covariance=list(msg.angular_velocity_covariance),
        linear_acceleration=RuntimeVector3(la.x, la.y, la.z),
        linear_acceleration_covariance=list(msg.linear_acceleration_covariance),
        ts=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
        frame_id=msg.header.frame_id,
    )
