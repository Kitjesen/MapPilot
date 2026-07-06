"""MuJoCo raw sensor bridge helpers for ROS-facing simulation adapters.

These functions are intentionally not a gate harness. They convert MuJoCo
state and point samples into canonical ROS-compatible sensor messages so
runtime adapters and validation gates can share the same sensor semantics.
"""

from __future__ import annotations

import math
import struct
from typing import Any

import numpy as np

from runtime.runtime_interface import FRAME_LINKS, TOPICS, topic_default_frame_id


MUJOCO_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_BODY_FRAME_ID = FRAME_LINKS["odom_to_body"].child
MUJOCO_RAW_IMU_FRAME_ID = MUJOCO_BODY_FRAME_ID


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q[:4]]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def world_xyzi_to_sensor_xyzi(engine: Any, pts_xyzi_world: np.ndarray) -> np.ndarray:
    """Convert MuJoCo world-frame XYZI points into the LiDAR sensor frame."""

    pts = np.asarray(pts_xyzi_world, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")

    data = getattr(engine, "_data", None)
    model = getattr(engine, "_model", None)
    sensor_pos = None
    sensor_rmat = None
    if data is not None:
        try:
            import mujoco

            lidar_cfg = getattr(engine, "_lidar_cfg", None)
            site_name = str(getattr(lidar_cfg, "site_name", "") or "lidar_site")
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            if site_id >= 0:
                sensor_pos = np.asarray(data.site_xpos[site_id], dtype=np.float64)
                sensor_rmat = np.asarray(data.site_xmat[site_id], dtype=np.float64).reshape(3, 3)
        except Exception:
            sensor_pos = None
            sensor_rmat = None
    if sensor_pos is None or sensor_rmat is None:
        lidar_id = int(getattr(engine, "_lidar_body_id", 0))
        if data is not None and lidar_id >= 0:
            sensor_pos = np.asarray(data.xpos[lidar_id], dtype=np.float64)
            sensor_rmat = np.asarray(data.xmat[lidar_id], dtype=np.float64).reshape(3, 3)
        else:
            state = engine.get_robot_state()
            sensor_pos = np.asarray(state.position, dtype=np.float64)
            sensor_rmat = quat_xyzw_to_matrix(
                np.asarray(state.orientation, dtype=np.float64)
            )
    xyz_sensor = (pts[:, :3].astype(np.float64) - sensor_pos) @ sensor_rmat
    intensity = (
        pts[:, 3:4].astype(np.float32)
        if pts.shape[1] >= 4
        else np.full((len(pts), 1), 100.0, dtype=np.float32)
    )
    return np.hstack([xyz_sensor.astype(np.float32), intensity]).astype(
        np.float32,
        copy=False,
    )


def sensor_xyzi_to_body_xyzi(pts_xyzi_sensor: np.ndarray, extrinsic: Any) -> np.ndarray:
    """Convert LiDAR-local XYZI points into the canonical body frame."""

    pts = np.asarray(pts_xyzi_sensor, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")

    rotation = quat_xyzw_to_matrix(np.asarray(extrinsic.rotation_xyzw, dtype=np.float64))
    translation = np.asarray(extrinsic.translation, dtype=np.float64)
    xyz_body = pts[:, :3].astype(np.float64) @ rotation.T + translation
    intensity = (
        pts[:, 3:4].astype(np.float32)
        if pts.shape[1] >= 4
        else np.full((len(pts), 1), 100.0, dtype=np.float32)
    )
    return np.hstack([xyz_body.astype(np.float32), intensity]).astype(
        np.float32,
        copy=False,
    )


def make_pointcloud2(
    *,
    points_xyzi: np.ndarray,
    stamp: Any,
    frame_id: str,
    pointcloud_cls: Any,
    pointfield_cls: Any,
    relative_times_s: np.ndarray | None = None,
    rings: np.ndarray | None = None,
) -> Any:
    pts = np.asarray(points_xyzi, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 4:
        raise ValueError(f"expected XYZI point cloud shape (N, 4), got {pts.shape}")
    n_pts = int(len(pts))
    if relative_times_s is None:
        relative_times = np.zeros(n_pts, dtype=np.float32)
    else:
        relative_times = np.asarray(relative_times_s, dtype=np.float32).reshape(-1)
        if relative_times.shape[0] != n_pts:
            raise ValueError(
                "relative_times_s must have one entry per point "
                f"({relative_times.shape[0]} != {n_pts})"
            )
    if rings is None:
        ring_values = (np.arange(n_pts, dtype=np.uint16) % 4).astype(np.uint16)
    else:
        ring_values = np.asarray(rings, dtype=np.uint16).reshape(-1)
        if ring_values.shape[0] != n_pts:
            raise ValueError(
                f"rings must have one entry per point ({ring_values.shape[0]} != {n_pts})"
            )

    msg = pointcloud_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n_pts
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        pointfield_cls(name="x", offset=0, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="y", offset=4, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="z", offset=8, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="intensity", offset=12, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="time", offset=16, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="ring", offset=20, datatype=pointfield_cls.UINT16, count=1),
    ]
    msg.point_step = 24
    msg.row_step = msg.point_step * n_pts
    data = bytearray(msg.row_step)
    for idx in range(n_pts):
        base = idx * msg.point_step
        struct.pack_into(
            "<fffffH",
            data,
            base,
            float(pts[idx, 0]),
            float(pts[idx, 1]),
            float(pts[idx, 2]),
            float(pts[idx, 3]),
            float(relative_times[idx]),
            int(ring_values[idx]),
        )
    msg.data = bytes(data)
    return msg


def make_livox_custom_msg(
    *,
    points_xyzi: np.ndarray,
    stamp: Any,
    frame_id: str,
    custom_msg_cls: Any,
    custom_point_cls: Any,
    relative_times_s: np.ndarray,
    rings: np.ndarray | None = None,
) -> Any:
    pts = np.asarray(points_xyzi, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 4:
        raise ValueError(f"expected XYZI point cloud shape (N, 4), got {pts.shape}")
    n_pts = int(len(pts))
    relative_times = np.asarray(relative_times_s, dtype=np.float64).reshape(-1)
    if relative_times.shape[0] != n_pts:
        raise ValueError(
            "relative_times_s must have one entry per point "
            f"({relative_times.shape[0]} != {n_pts})"
        )
    if rings is None:
        ring_values = (np.arange(n_pts, dtype=np.uint16) % 4).astype(np.uint16)
    else:
        ring_values = np.asarray(rings, dtype=np.uint16).reshape(-1)
        if ring_values.shape[0] != n_pts:
            raise ValueError(
                f"rings must have one entry per point ({ring_values.shape[0]} != {n_pts})"
            )

    msg = custom_msg_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.point_num = n_pts
    if hasattr(msg, "timebase"):
        msg.timebase = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
            getattr(stamp, "nanosec", 0)
        )
    if hasattr(msg, "lidar_id"):
        msg.lidar_id = 0
    if hasattr(msg, "rsvd"):
        msg.rsvd = [0, 0, 0]

    points = []
    for idx in range(n_pts):
        point = custom_point_cls()
        point.offset_time = int(max(0.0, float(relative_times[idx])) * 1_000_000_000.0)
        point.x = float(pts[idx, 0])
        point.y = float(pts[idx, 1])
        point.z = float(pts[idx, 2])
        point.reflectivity = int(np.clip(round(float(pts[idx, 3])), 0, 255))
        point.tag = 0x10
        point.line = int(ring_values[idx] % 4)
        points.append(point)
    msg.points = points
    return msg


def projected_gravity_body(state: Any) -> np.ndarray:
    """Return the body-frame gravity direction as a unit vector."""

    projected = np.asarray(
        getattr(state, "imu_projected_gravity", ()),
        dtype=np.float64,
    ).reshape(-1)
    if projected.shape[0] == 3 and np.isfinite(projected).all():
        norm = float(np.linalg.norm(projected))
        if norm > 1e-9:
            return (projected / norm).astype(np.float64, copy=False)

    gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    rot_body_to_world = quat_xyzw_to_matrix(np.asarray(state.orientation, dtype=np.float64))
    return (rot_body_to_world.T @ gravity_world).astype(np.float64, copy=False)


def sensor_specific_force_body(state: Any) -> np.ndarray:
    """Return MuJoCo IMU accelerometer data as Fast-LIO specific force.

    Dynamic MuJoCo policy mode exposes a proper accelerometer signal with
    gravity magnitude already present. Kinematic mode advances poses with
    ``mj_forward`` and its accelerometer can be near zero, so synthesize the
    missing gravity term from the same state orientation instead of publishing
    a non-physical zero-g IMU.
    """

    accel = np.asarray(
        getattr(state, "imu_linear_acceleration", ()),
        dtype=np.float64,
    ).reshape(-1)
    if accel.shape[0] != 3 or not np.isfinite(accel).all():
        accel = np.zeros(3, dtype=np.float64)
    else:
        accel = accel.astype(np.float64, copy=True)

    if float(np.linalg.norm(accel)) < 0.5 * 9.80665:
        accel = accel - projected_gravity_body(state) * 9.80665
    return accel.astype(np.float64, copy=False)


def specific_force_body(
    state: Any,
    prev_velocity: np.ndarray | None,
    dt: float,
    *,
    mode: str = "finite_difference",
) -> np.ndarray:
    mode = str(mode or "finite_difference").strip().lower()
    if mode not in {"finite_difference", "gravity_only", "sensor"}:
        raise ValueError(f"unsupported imu acceleration mode: {mode}")

    if mode == "sensor":
        return sensor_specific_force_body(state)

    velocity = np.asarray(state.linear_velocity, dtype=np.float64)
    if mode == "gravity_only" or prev_velocity is None or dt <= 0.0:
        world_acc = np.zeros(3, dtype=np.float64)
    else:
        world_acc = (velocity - prev_velocity) / dt
    gravity_world = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    rot_body_to_world = quat_xyzw_to_matrix(np.asarray(state.orientation, dtype=np.float64))
    return rot_body_to_world.T @ (world_acc - gravity_world)


def make_imu_msg(
    *,
    state: Any,
    prev_velocity: np.ndarray | None,
    dt: float,
    stamp: Any,
    imu_cls: Any,
    acc_mode: str = "finite_difference",
) -> Any:
    msg = imu_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = MUJOCO_RAW_IMU_FRAME_ID
    gyro = np.asarray(state.imu_gyro, dtype=np.float64)
    acc = specific_force_body(state, prev_velocity, dt, mode=acc_mode)
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    return msg


def make_sim_odometry_msg(*, state: Any, stamp: Any, odometry_cls: Any) -> Any:
    msg = odometry_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = MUJOCO_ODOM_FRAME_ID
    msg.child_frame_id = MUJOCO_BODY_FRAME_ID
    pos = np.asarray(state.position, dtype=np.float64)
    quat = np.asarray(state.orientation, dtype=np.float64)
    lin = np.asarray(state.linear_velocity, dtype=np.float64)
    ang = np.asarray(state.angular_velocity, dtype=np.float64)
    msg.pose.pose.position.x = float(pos[0])
    msg.pose.pose.position.y = float(pos[1])
    msg.pose.pose.position.z = float(pos[2])
    msg.pose.pose.orientation.x = float(quat[0])
    msg.pose.pose.orientation.y = float(quat[1])
    msg.pose.pose.orientation.z = float(quat[2])
    msg.pose.pose.orientation.w = float(quat[3])
    msg.twist.twist.linear.x = float(lin[0])
    msg.twist.twist.linear.y = float(lin[1])
    msg.twist.twist.linear.z = float(lin[2])
    msg.twist.twist.angular.x = float(ang[0])
    msg.twist.twist.angular.y = float(ang[1])
    msg.twist.twist.angular.z = float(ang[2])
    return msg


def make_transform_msg(
    *,
    stamp: Any,
    transform_cls: Any,
    parent: str,
    child: str,
    translation_xyz: Any = (0.0, 0.0, 0.0),
    rotation_xyzw: Any = (0.0, 0.0, 0.0, 1.0),
) -> Any:
    msg = transform_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    xyz = np.asarray(translation_xyz, dtype=np.float64)
    quat = np.asarray(rotation_xyzw, dtype=np.float64)
    msg.transform.translation.x = float(xyz[0]) if xyz.shape[0] > 0 else 0.0
    msg.transform.translation.y = float(xyz[1]) if xyz.shape[0] > 1 else 0.0
    msg.transform.translation.z = float(xyz[2]) if xyz.shape[0] > 2 else 0.0
    msg.transform.rotation.x = float(quat[0]) if quat.shape[0] > 0 else 0.0
    msg.transform.rotation.y = float(quat[1]) if quat.shape[0] > 1 else 0.0
    msg.transform.rotation.z = float(quat[2]) if quat.shape[0] > 2 else 0.0
    msg.transform.rotation.w = float(quat[3]) if quat.shape[0] > 3 else 1.0
    return msg


def make_odom_body_tf(*, state: Any, stamp: Any, transform_cls: Any) -> Any:
    return make_transform_msg(
        stamp=stamp,
        transform_cls=transform_cls,
        parent=FRAME_LINKS["odom_to_body"].parent,
        child=FRAME_LINKS["odom_to_body"].child,
        translation_xyz=np.asarray(state.position, dtype=np.float64),
        rotation_xyzw=np.asarray(state.orientation, dtype=np.float64),
    )


def yaw_from_quat_xyzw(quat: Any) -> float:
    q = np.asarray(quat, dtype=np.float64)
    if q.shape[0] < 4:
        return 0.0
    x, y, z, w = [float(v) for v in q[:4]]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def angle_delta_rad(a: float, b: float) -> float:
    return float(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))
