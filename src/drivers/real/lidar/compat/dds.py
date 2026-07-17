"""Livox CustomMsg + ROS2 Imu DDS IDL types for cyclonedds.

Mirrors livox_ros_driver2 and sensor_msgs/Imu wire layouts exactly
so Python can subscribe to /lidar/scan and /imu/data without rclpy.

NOTE: No `from __future__ import annotations` �?cyclonedds IdlStruct
requires real type objects at class-definition time.
"""

import logging
from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np

from ..api.frames import POINT_DTYPE, LivoxPointFrame

logger = logging.getLogger(__name__)

try:
    from cyclonedds.idl import IdlStruct, types

    # Reuse core IDL types if available, else define inline.
    try:
        from runtime.adapters.dds.reader import DDS_Header, DDS_Quaternion, DDS_Vector3
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

        offset_time: types.uint32  # ns offset from timebase
        x: types.float32
        y: types.float32
        z: types.float32
        reflectivity: types.uint8
        tag: types.uint8  # return type flags
        line: types.uint8  # scan line index

    @dataclass
    class LivoxCustomMsg(
        IdlStruct,
        typename="livox_ros_driver2::msg::dds_::CustomMsg_",
    ):
        """livox_ros_driver2/CustomMsg �?one full LiDAR scan."""

        header: DDS_Header
        timebase: types.uint64  # absolute ns timestamp of first point
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
        """sensor_msgs/Imu �?angular velocity + linear acceleration + orientation."""

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
    LivoxPoint = None  # type: ignore[assignment, misc]
    DDS_Imu = None  # type: ignore[assignment, misc]


# ── Conversion helpers ──────────────────────────────────────────────────


def _timestamp_ns(msg) -> int:
    timebase = int(getattr(msg, "timebase", 0) or 0)
    if timebase > 0:
        return timebase
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def livox_msg_to_frame(msg) -> LivoxPointFrame | None:
    """Convert a LivoxCustomMsg to a lossless Livox point frame."""

    pts = msg.points
    if not pts:
        return None
    arr = np.empty(len(pts), dtype=POINT_DTYPE)
    arr["x"] = [p.x for p in pts]
    arr["y"] = [p.y for p in pts]
    arr["z"] = [p.z for p in pts]
    arr["intensity"] = [float(p.reflectivity) for p in pts]
    arr["offset_time_ns"] = [p.offset_time for p in pts]
    arr["tag"] = [p.tag for p in pts]
    arr["line"] = [p.line for p in pts]
    arr["flags"] = 0
    return LivoxPointFrame(
        points=arr,
        timestamp_ns=_timestamp_ns(msg),
        sequence=int(getattr(msg, "lidar_id", 0)),
    )


def _make_dds_time(timestamp_ns: int):
    try:
        from runtime.adapters.dds.reader import DDS_Time
    except ImportError:
        DDS_Time = globals().get("_DDS_Time")
    if DDS_Time is None:
        raise ImportError("DDS_Time is unavailable")
    return DDS_Time(
        sec=int(timestamp_ns) // 1_000_000_000,
        nanosec=int(timestamp_ns) % 1_000_000_000,
    )


def livox_frame_to_msg(frame: LivoxPointFrame | Any):
    """Convert a LingTu Livox frame to the Livox DDS CustomMsg."""

    if hasattr(frame, "timebase") and hasattr(frame, "points"):
        return frame
    if not HAS_LIVOX_IDL:
        raise ImportError("cyclonedds IDL types are unavailable")

    points = np.asarray(getattr(frame, "points", frame), dtype=POINT_DTYPE)
    timestamp_ns = int(getattr(frame, "timestamp_ns", 0) or 0)
    dds_points = [
        LivoxPoint(
            offset_time=int(point["offset_time_ns"]),
            x=float(point["x"]),
            y=float(point["y"]),
            z=float(point["z"]),
            reflectivity=max(0, min(255, int(round(float(point["intensity"]))))),
            tag=int(point["tag"]),
            line=int(point["line"]),
        )
        for point in points
    ]
    return LivoxCustomMsg(
        header=DDS_Header(
            stamp=_make_dds_time(timestamp_ns),
            frame_id=str(getattr(frame, "frame_id", "livox_frame")),
        ),
        timebase=timestamp_ns,
        point_num=len(dds_points),
        lidar_id=int(getattr(frame, "sequence", 0) or 0) & 0xFF,
        rsvd=[0, 0, 0],
        points=dds_points,
    )


def livox_msg_to_numpy(msg) -> Any | None:
    """Convert a LivoxCustomMsg to numpy (N, 4): x, y, z, intensity.

    Uses list-comprehension + single np.array() call �?measured ~1.5ms
    for 24k points on aarch64, well within the 100ms frame budget.
    """
    frame = livox_msg_to_frame(msg)
    if frame is None:
        return None
    return frame.to_xyzi()


def dds_imu_to_imu(msg):
    """Convert a DDS_Imu message to runtime.msgs.sensor.Imu."""
    from runtime.msgs.geometry import Quaternion, Vector3
    from runtime.msgs.sensor import Imu

    o = msg.orientation
    av = msg.angular_velocity
    la = msg.linear_acceleration
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return Imu(
        orientation=Quaternion(o.x, o.y, o.z, o.w),
        orientation_covariance=list(msg.orientation_covariance),
        angular_velocity=Vector3(av.x, av.y, av.z),
        angular_velocity_covariance=list(msg.angular_velocity_covariance),
        linear_acceleration=Vector3(la.x, la.y, la.z),
        linear_acceleration_covariance=list(msg.linear_acceleration_covariance),
        ts=ts,
        frame_id=msg.header.frame_id,
    )
