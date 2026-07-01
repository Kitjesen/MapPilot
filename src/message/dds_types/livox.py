"""Native LingTu DDS Livox frame type and lossless conversions."""

from dataclasses import dataclass

from drivers.real.lidar.frames import POINT_DTYPE, LivoxPointFrame
from runtime.msgs.numpy_compat import np

from ._base import IdlStruct, types
from .common import Header, Time


@dataclass
class LivoxPoint(IdlStruct, typename="lingtu::dds::LivoxPoint"):
    offset_time: types.uint32
    x: types.float32
    y: types.float32
    z: types.float32
    reflectivity: types.uint8
    tag: types.uint8
    line: types.uint8


@dataclass
class LivoxFrame(IdlStruct, typename="lingtu::dds::LivoxFrame"):
    header: Header
    timebase: types.uint64
    point_num: types.uint32
    lidar_id: types.uint8
    rsvd: types.array[types.uint8, 3]
    points: types.sequence[LivoxPoint]


LivoxCustomMsg = LivoxFrame


def _make_time(timestamp_ns: int) -> Time:
    return Time(
        sec=int(timestamp_ns) // 1_000_000_000,
        nanosec=int(timestamp_ns) % 1_000_000_000,
    )


def livox_frame_to_msg(frame) -> LivoxFrame:
    if isinstance(frame, LivoxFrame):
        return frame
    if hasattr(frame, "timebase") and hasattr(frame, "points"):
        return frame

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
    return LivoxFrame(
        header=Header(
            stamp=_make_time(timestamp_ns),
            frame_id=str(getattr(frame, "frame_id", "livox_frame")),
        ),
        timebase=timestamp_ns,
        point_num=len(dds_points),
        lidar_id=int(getattr(frame, "sequence", 0) or 0) & 0xFF,
        rsvd=[0, 0, 0],
        points=dds_points,
    )


def livox_msg_to_frame(msg) -> LivoxPointFrame | None:
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
    timebase = int(getattr(msg, "timebase", 0) or 0)
    if timebase <= 0:
        stamp = msg.header.stamp
        timebase = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return LivoxPointFrame(
        points=arr,
        timestamp_ns=timebase,
        sequence=int(getattr(msg, "lidar_id", 0)),
    )


def livox_msg_to_numpy(msg):
    frame = livox_msg_to_frame(msg)
    return None if frame is None else frame.to_xyzi()
