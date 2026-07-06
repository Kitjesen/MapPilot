"""Typed DDS payload conversion helpers for LingTu runtime messages."""

from __future__ import annotations

import json
import time
from collections.abc import Mapping
from typing import Any

from runtime.msgs.geometry import (
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from runtime.msgs.nav import OccupancyGrid, Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.tf import TF_STATIC_TOPIC, TF_TOPIC, tf_message_from_any


def _dds() -> Any:
    from . import dds_types as dds_mod

    return dds_mod


def to_dds_message(topic: str, msg: Any) -> Any:
    if topic in {TF_TOPIC, TF_STATIC_TOPIC}:
        return to_dds_tf_message(msg)
    if topic == TOPICS.lidar_scan:
        return to_dds_livox_custom_msg(msg)
    if topic in {TOPICS.registered_cloud, TOPICS.map_cloud, TOPICS.saved_map_cloud}:
        return to_dds_pointcloud2(msg)
    if topic == TOPICS.traversability:
        return to_dds_occupancy_grid(msg)
    if topic == TOPICS.odometry:
        return to_dds_odometry(msg)
    if topic == TOPICS.imu:
        return to_dds_imu(msg)
    if topic == TOPICS.localization_quality:
        return to_dds_float32(msg)
    if topic == TOPICS.localization_health:
        return to_dds_text(json.dumps(dict(msg), ensure_ascii=True, sort_keys=True))
    if topic == TOPICS.goal_pose:
        return to_dds_pose_stamped(msg)
    if topic in {TOPICS.cancel, TOPICS.semantic_instruction}:
        return to_dds_string(msg)
    raise ValueError(f"DDS endpoint publisher for {topic} is not implemented")


def from_dds_message(topic: str, msg: Any) -> Any:
    if topic in {TF_TOPIC, TF_STATIC_TOPIC}:
        return tf_message_from_any(msg)
    if topic == TOPICS.lidar_scan:
        return from_dds_lidar_scan(msg)
    if topic in {TOPICS.global_path, TOPICS.local_path}:
        return msg
    if topic == TOPICS.nav_way_point:
        return from_dds_pose_stamped(msg, topic_default_frame_id(TOPICS.nav_way_point))
    if topic == TOPICS.cmd_vel:
        return from_dds_twist_stamped(msg)
    if topic == TOPICS.goal_pose:
        return from_dds_pose_stamped(msg, topic_default_frame_id(TOPICS.goal_pose))
    if topic in {TOPICS.cancel, TOPICS.semantic_instruction}:
        return from_dds_string(msg)
    if topic == TOPICS.odometry:
        return from_dds_odometry(msg)
    if topic in {TOPICS.registered_cloud, TOPICS.map_cloud, TOPICS.saved_map_cloud}:
        return from_dds_pointcloud2(msg)
    if topic == TOPICS.traversability:
        return from_dds_occupancy_grid(msg)
    return msg


def to_dds_time(ts: float | int | None) -> Any:
    value = float(ts or time.time())
    sec = int(value)
    nanosec = int(max(0.0, value - sec) * 1_000_000_000)
    return _dds().DDS_Time(sec=sec, nanosec=nanosec)


def to_dds_header(frame_id: str, ts: float | int | None) -> Any:
    return _dds().DDS_Header(stamp=to_dds_time(ts), frame_id=str(frame_id or ""))


def to_dds_vector(value: Any) -> Any:
    return _dds().DDS_Vector3(x=float(value.x), y=float(value.y), z=float(value.z))


def to_dds_quaternion(value: Any) -> Any:
    return _dds().DDS_Quaternion(
        x=float(value.x),
        y=float(value.y),
        z=float(value.z),
        w=float(value.w),
    )


def to_dds_point(value: Any) -> Any:
    return _dds().DDS_Point(x=float(value.x), y=float(value.y), z=float(value.z))


def to_dds_pose(value: Any) -> Any:
    return _dds().DDS_Pose(
        position=to_dds_point(value.position),
        orientation=to_dds_quaternion(value.orientation),
    )


def to_dds_odometry(odom: Odometry) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Odometry(
        header=to_dds_header(odom.frame_id, odom.ts),
        child_frame_id=str(odom.child_frame_id or body_frame_id()),
        pose=dds_mod.DDS_PoseWithCovariance(
            pose=to_dds_pose(odom.pose),
            covariance=[0.0] * 36,
        ),
        twist=dds_mod.DDS_TwistWithCovariance(
            twist=dds_mod.DDS_Twist(
                linear=to_dds_vector(odom.twist.linear),
                angular=to_dds_vector(odom.twist.angular),
            ),
            covariance=[0.0] * 36,
        ),
    )


def to_dds_pointcloud2(cloud: PointCloud2) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_PointCloud2(
        header=to_dds_header(cloud.frame_id, cloud.ts),
        height=int(cloud.height),
        width=int(cloud.width),
        fields=[
            dds_mod.DDS_PointField(
                name=field.name,
                offset=int(field.offset),
                datatype=int(field.datatype),
                count=int(field.count),
            )
            for field in cloud.fields
        ],
        is_bigendian=bool(cloud.is_bigendian),
        point_step=int(cloud.point_step),
        row_step=int(cloud.row_step),
        data=list(cloud.data),
        is_dense=bool(cloud.is_dense),
    )


def to_dds_occupancy_grid(grid: OccupancyGrid | Mapping[str, Any]) -> Any:
    if isinstance(grid, OccupancyGrid):
        payload = grid.to_dict()
    elif isinstance(grid, Mapping):
        payload = dict(grid)
    else:
        raise TypeError(
            f"traversability expects OccupancyGrid or dict, got {type(grid).__name__}"
        )

    values = np.asarray(payload.get("grid"), dtype=np.int16)
    if values.ndim != 2:
        values = np.zeros((0, 0), dtype=np.int16)
    height, width = values.shape
    origin = payload.get("origin")
    if isinstance(origin, Mapping):
        origin_xy = (
            float(origin.get("x", 0.0)),
            float(origin.get("y", 0.0)),
        )
    elif origin is not None:
        origin_xy = (float(origin[0]), float(origin[1]))
    else:
        origin_xy = (
            float(payload.get("origin_x") or 0.0),
            float(payload.get("origin_y") or 0.0),
        )
    ts = float(payload.get("ts") or time.time())
    dds_mod = _dds()
    return dds_mod.DDS_OccupancyGrid(
        header=to_dds_header(str(payload.get("frame_id") or "map"), ts),
        info=dds_mod.DDS_MapMetaData(
            map_load_time=to_dds_time(ts),
            resolution=float(payload.get("resolution") or 0.0),
            width=int(width),
            height=int(height),
            origin=dds_mod.DDS_Pose(
                position=dds_mod.DDS_Point(x=origin_xy[0], y=origin_xy[1], z=0.0),
                orientation=dds_mod.DDS_Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.clip(values, -1, 100).astype(np.int8).reshape(-1).tolist(),
    )


def to_dds_imu(imu: Imu) -> Any:
    return _dds().Imu(
        header=to_dds_header(imu.frame_id, imu.ts),
        orientation=to_dds_quaternion(imu.orientation),
        orientation_covariance=list(imu.orientation_covariance),
        angular_velocity=to_dds_vector(imu.angular_velocity),
        angular_velocity_covariance=list(imu.angular_velocity_covariance),
        linear_acceleration=to_dds_vector(imu.linear_acceleration),
        linear_acceleration_covariance=list(imu.linear_acceleration_covariance),
    )


def to_dds_livox_custom_msg(scan: Any) -> Any:
    if hasattr(scan, "timebase") and hasattr(scan, "points"):
        return scan
    if hasattr(scan, "timestamp_ns") and hasattr(scan, "points"):
        return _dds().livox_frame_to_msg(scan)
    if not isinstance(scan, PointCloud2):
        raise TypeError(
            f"lidar_scan expects LivoxCustomMsg or PointCloud2, got {type(scan).__name__}"
        )

    points = np.asarray(scan.points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(f"lidar_scan points must be (N,3) or (N,4), got {points.shape}")
    stamp_ns = int(float(scan.ts or time.time()) * 1_000_000_000)
    livox_points = []
    for row in points:
        intensity = float(row[3]) if row.shape[0] > 3 else 0.0
        livox_points.append(
            _dds().LivoxPoint(
                offset_time=0,
                x=float(row[0]),
                y=float(row[1]),
                z=float(row[2]),
                reflectivity=max(0, min(255, int(round(intensity)))),
                tag=0,
                line=0,
            )
        )
    return _dds().LivoxFrame(
        header=to_dds_header(scan.frame_id, scan.ts),
        timebase=stamp_ns,
        point_num=len(livox_points),
        lidar_id=0,
        rsvd=[0, 0, 0],
        points=livox_points,
    )


def to_dds_pose_stamped(pose: PoseStamped) -> Any:
    return _dds().DDS_PoseStamped(
        header=to_dds_header(pose.frame_id, pose.ts),
        pose=to_dds_pose(pose.pose),
    )


def to_dds_tf_message(msg: Any) -> Any:
    tf_msg = tf_message_from_any(msg)
    dds_mod = _dds()
    return dds_mod.DDS_TFMessage(
        transforms=[
            dds_mod.DDS_TransformStamped(
                header=to_dds_header(transform.frame_id, transform.ts),
                child_frame_id=str(transform.child_frame_id),
                transform=dds_mod.DDS_Transform(
                    translation=to_dds_vector(transform.translation),
                    rotation=to_dds_quaternion(transform.rotation),
                ),
            )
            for transform in tf_msg.transforms
        ]
    )


def to_dds_string(value: Any) -> Any:
    return _dds().DDS_String(data=str(value or ""))


def to_dds_text(value: Any) -> Any:
    return _dds().Text(data=str(value or ""))


def to_dds_float32(value: Any) -> Any:
    return _dds().DDS_Float32(data=float(value))


def from_dds_time(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0.0)) + float(
        getattr(stamp, "nanosec", 0.0)
    ) * 1e-9


def from_dds_pose_stamped(msg: Any, frame_id: str) -> PoseStamped:
    header = getattr(msg, "header", None)
    pose = getattr(msg, "pose", None)
    if pose is None:
        raise TypeError("DDS PoseStamped missing pose")
    position = getattr(pose, "position")
    orientation = getattr(pose, "orientation", None)
    return PoseStamped(
        pose=Pose(
            position=Vector3(
                float(getattr(position, "x", 0.0)),
                float(getattr(position, "y", 0.0)),
                float(getattr(position, "z", 0.0)),
            ),
            orientation=Quaternion(
                float(getattr(orientation, "x", 0.0)),
                float(getattr(orientation, "y", 0.0)),
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            ),
        ),
        ts=from_dds_time(getattr(header, "stamp", None)),
        frame_id=str(getattr(header, "frame_id", "") or frame_id),
    )


def from_dds_string(msg: Any) -> str:
    return str(getattr(msg, "data", msg) or "")


def from_dds_twist_stamped(msg: Any) -> Twist:
    twist = getattr(msg, "twist", msg)
    return Twist(
        linear=to_vector3(twist.linear),
        angular=to_vector3(twist.angular),
    )


def from_dds_occupancy_grid(msg: Any) -> dict[str, Any]:
    width = int(getattr(msg.info, "width", 0))
    height = int(getattr(msg.info, "height", 0))
    values = np.asarray(list(getattr(msg, "data", []) or []), dtype=np.int16)
    if width > 0 and height > 0 and values.size >= width * height:
        grid = values[: width * height].reshape((height, width)).astype(np.int16)
    else:
        grid = np.zeros((0, 0), dtype=np.int16)
    origin = getattr(msg.info, "origin", None)
    position = getattr(origin, "position", None)
    return {
        "grid": grid.tolist(),
        "resolution": float(getattr(msg.info, "resolution", 0.0)),
        "origin": [
            float(getattr(position, "x", 0.0)),
            float(getattr(position, "y", 0.0)),
        ],
        "frame_id": str(getattr(msg.header, "frame_id", "") or "map"),
        "ts": from_dds_time(getattr(msg.header, "stamp", None)),
        "width": width,
        "height": height,
    }


def to_vector3(value: Any) -> Vector3:
    return Vector3(
        float(getattr(value, "x", 0.0)),
        float(getattr(value, "y", 0.0)),
        float(getattr(value, "z", 0.0)),
    )


def from_dds_odometry(msg: Any) -> Odometry:
    pose = msg.pose.pose
    twist = msg.twist.twist
    position = pose.position
    orientation = pose.orientation
    return Odometry(
        pose=Pose(
            position=Vector3(position.x, position.y, position.z),
            orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        ),
        twist=Twist(
            linear=Vector3(twist.linear.x, twist.linear.y, twist.linear.z),
            angular=Vector3(twist.angular.x, twist.angular.y, twist.angular.z),
        ),
        ts=from_dds_time(getattr(msg.header, "stamp", None)),
        frame_id=str(getattr(msg.header, "frame_id", "") or topic_default_frame_id(TOPICS.odometry)),
        child_frame_id=str(getattr(msg, "child_frame_id", "") or body_frame_id()),
    )


def from_dds_pointcloud2(msg: Any) -> PointCloud2:
    n = int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 0))
    step = int(getattr(msg, "point_step", 0))
    if n <= 0:
        return PointCloud2(points=np.zeros((0, 3), dtype=np.float32), frame_id=frame_id(msg))
    if step < 12:
        raise ValueError(f"PointCloud2 point_step too small: {step}")
    raw = np.frombuffer(bytes(getattr(msg, "data", b"")), dtype=np.uint8).reshape(n, step)
    fields = {
        str(getattr(field, "name", "")): int(getattr(field, "offset", 0))
        for field in getattr(msg, "fields", [])
    }
    x_off = fields.get("x", 0)
    y_off = fields.get("y", 4)
    z_off = fields.get("z", 8)
    cols = [
        raw[:, x_off : x_off + 4].copy().view(np.float32).reshape(n),
        raw[:, y_off : y_off + 4].copy().view(np.float32).reshape(n),
        raw[:, z_off : z_off + 4].copy().view(np.float32).reshape(n),
    ]
    intensity_off = fields.get("intensity")
    if intensity_off is not None and intensity_off + 4 <= step:
        cols.append(raw[:, intensity_off : intensity_off + 4].copy().view(np.float32).reshape(n))
    points = np.stack(cols, axis=1).astype(np.float32, copy=False)
    return PointCloud2(
        points=points,
        ts=from_dds_time(getattr(msg.header, "stamp", None)),
        frame_id=frame_id(msg),
        height=int(getattr(msg, "height", 1) or 1),
        width=int(getattr(msg, "width", n) or n),
        is_bigendian=bool(getattr(msg, "is_bigendian", False)),
        is_dense=bool(getattr(msg, "is_dense", True)),
    )


def from_dds_lidar_scan(msg: Any) -> PointCloud2 | None:
    if hasattr(msg, "points") and hasattr(msg, "timebase"):
        from .dds_types.livox import livox_msg_to_numpy

        points = livox_msg_to_numpy(msg)
        if points is None:
            return None
        return PointCloud2(
            points=points,
            ts=float(getattr(msg, "timebase", 0) or 0) * 1e-9,
            frame_id=frame_id(msg),
        )
    return from_dds_pointcloud2(msg)


def from_dds_imu(msg: Any) -> Imu:
    orientation = msg.orientation
    angular = msg.angular_velocity
    linear = msg.linear_acceleration
    return Imu(
        orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        orientation_covariance=list(getattr(msg, "orientation_covariance", [0.0] * 9)),
        angular_velocity=Vector3(angular.x, angular.y, angular.z),
        angular_velocity_covariance=list(getattr(msg, "angular_velocity_covariance", [0.0] * 9)),
        linear_acceleration=Vector3(linear.x, linear.y, linear.z),
        linear_acceleration_covariance=list(getattr(msg, "linear_acceleration_covariance", [0.0] * 9)),
        ts=from_dds_time(getattr(msg.header, "stamp", None)),
        frame_id=frame_id(msg, default="imu_link"),
    )


def coerce_health_payload(msg: Any) -> Mapping[str, Any] | str:
    data = getattr(msg, "data", msg)
    if isinstance(data, Mapping):
        return data
    text = str(data or "")
    if text.startswith("{"):
        return json.loads(text)
    return text


def frame_id(msg: Any, *, default: str = "") -> str:
    header = getattr(msg, "header", None)
    return str(getattr(header, "frame_id", "") or default)
