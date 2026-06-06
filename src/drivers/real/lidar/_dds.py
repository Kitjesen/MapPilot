"""Livox CustomMsg + ROS2 Imu DDS IDL types for cyclonedds.

Mirrors livox_ros_driver2 and sensor_msgs/Imu wire layouts exactly
so Python can subscribe to /lidar/scan and /imu/data without rclpy.

NOTE: No `from __future__ import annotations` — cyclonedds IdlStruct
requires real type objects at class-definition time.
"""

import logging
from dataclasses import dataclass
from typing import Any

from core.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

try:
    from cyclonedds.idl import IdlStruct, types

    # Reuse core IDL types if available, else define inline.
    try:
        from core.dds import DDS_Header, DDS_Quaternion, DDS_Vector3
    except ImportError:
        @dataclass
        class _DDS_Time(IdlStruct):
            sec: types.int32
            nanosec: types.uint32

        @dataclass
        class DDS_Header(IdlStruct):
            stamp: _DDS_Time
            frame_id: str

        @dataclass
        class DDS_Quaternion(IdlStruct):
            x: types.float64
            y: types.float64
            z: types.float64
            w: types.float64

        @dataclass
        class DDS_Vector3(IdlStruct):
            x: types.float64
            y: types.float64
            z: types.float64

    # ── Livox CustomMsg ─────────────────────────────────────────────────

    @dataclass
    class LivoxPoint(IdlStruct):
        """Single point in a Livox CustomMsg frame."""
        offset_time: types.uint32    # ns offset from timebase
        x: types.float32
        y: types.float32
        z: types.float32
        reflectivity: types.uint8
        tag: types.uint8             # return type flags
        line: types.uint8            # scan line index

    @dataclass
    class LivoxCustomMsg(
        IdlStruct,
        typename="livox_ros_driver2::msg::dds_::CustomMsg_",
    ):
        """livox_ros_driver2/CustomMsg — one full LiDAR scan."""
        header: DDS_Header
        timebase: types.uint64       # absolute ns timestamp of first point
        point_num: types.uint32
        lidar_id: types.uint8
        rsvd: types.array[types.uint8, 3]
        points: types.sequence[LivoxPoint]

    # ── sensor_msgs/Imu ─────────────────────────────────────────────────

    @dataclass
    class DDS_Imu(
        IdlStruct,
        typename="sensor_msgs::msg::dds_::Imu_",
    ):
        """sensor_msgs/Imu — angular velocity + linear acceleration + orientation."""
        header: DDS_Header
        orientation: DDS_Quaternion
        orientation_covariance: types.array[types.float64, 9]
        angular_velocity: DDS_Vector3
        angular_velocity_covariance: types.array[types.float64, 9]
        linear_acceleration: DDS_Vector3
        linear_acceleration_covariance: types.array[types.float64, 9]

    HAS_LIVOX_IDL = True

except ImportError:
    HAS_LIVOX_IDL = False
    LivoxCustomMsg = None  # type: ignore[assignment, misc]
    LivoxPoint = None      # type: ignore[assignment, misc]
    DDS_Imu = None         # type: ignore[assignment, misc]


# ── Conversion helpers ──────────────────────────────────────────────────


def livox_msg_to_numpy(msg) -> Any | None:
    """Convert a LivoxCustomMsg to numpy (N, 4): x, y, z, intensity.

    Uses list-comprehension + single np.array() call — measured ~1.5ms
    for 24k points on aarch64, well within the 100ms frame budget.
    """
    pts = msg.points
    if not pts:
        return None
    arr = np.array(
        [(p.x, p.y, p.z, float(p.reflectivity)) for p in pts],
        dtype=np.float32,
    )
    return arr


def dds_imu_to_imu(msg):
    """Convert a DDS_Imu message to core.msgs.sensor.Imu."""
    from core.msgs.geometry import Quaternion, Vector3
    from core.msgs.sensor import Imu

    o = msg.orientation
    av = msg.angular_velocity
    la = msg.linear_acceleration
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return Imu(
        orientation=Quaternion(o.x, o.y, o.z, o.w),
        angular_velocity=Vector3(av.x, av.y, av.z),
        linear_acceleration=Vector3(la.x, la.y, la.z),
        ts=ts,
        frame_id=msg.header.frame_id,
    )
