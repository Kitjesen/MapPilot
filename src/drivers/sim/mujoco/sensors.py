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
_NAVIGATION_FIXTURE_GROUND_X_MIN_M = -1.0
_NAVIGATION_FIXTURE_GROUND_X_MAX_M = 4.0
NAVIGATION_FIXTURE_GROUND_Y_HALF_M = 1.6
NAVIGATION_FIXTURE_GROUND_RESOLUTION_M = 0.2
_NAVIGATION_FIXTURE_GROUND_INTENSITY = 12.0
_NAVIGATION_FIXTURE_RAW_OVERLAY_MIN_ABOVE_GROUND_M = 0.05


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = [float(v) for v in quat / norm]
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


def world_points_to_body_frame(
    points: Any,
    position_xyz: Any,
    orientation_xyzw: Any,
) -> np.ndarray:
    """Transform world-frame points to the body frame, preserving extra columns."""

    cloud = np.asarray(points, dtype=np.float32).copy()
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        return cloud
    rotation_body_to_world = quat_xyzw_to_matrix(
        np.asarray(orientation_xyzw, dtype=np.float64)
    )
    relative_world = cloud[:, :3].astype(np.float64) - np.asarray(
        position_xyz,
        dtype=np.float64,
    ).reshape(3)
    cloud[:, :3] = (relative_world @ rotation_body_to_world).astype(np.float32)
    return cloud


def world_xyzi_to_sensor_xyzi(
    engine: Any,
    pts_xyzi_world: np.ndarray,
    *,
    data: Any | None = None,
) -> np.ndarray:
    """Convert MuJoCo world-frame XYZI points into the LiDAR sensor frame."""

    pts = np.asarray(pts_xyzi_world, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")

    data = getattr(engine, "_data", None) if data is None else data
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


def world_xyzi_to_body_xyzi(state: Any, pts_xyzi_world: np.ndarray) -> np.ndarray:
    """Convert world-frame XYZI endpoints into the body's scan-time frame."""

    pts = np.asarray(pts_xyzi_world, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")
    position = np.asarray(state.position, dtype=np.float64).reshape(-1)
    orientation = np.asarray(state.orientation, dtype=np.float64).reshape(-1)
    if position.size != 3 or orientation.size != 4:
        raise ValueError("body pose must contain XYZ and XYZW")
    if not bool(np.isfinite(position).all()) or not bool(np.isfinite(orientation).all()):
        raise ValueError("body pose must be finite")
    body_points = world_points_to_body_frame(pts, position, orientation)
    intensity = (
        pts[:, 3:4].astype(np.float32)
        if pts.shape[1] >= 4
        else np.full((len(pts), 1), 100.0, dtype=np.float32)
    )
    return np.hstack([body_points[:, :3], intensity]).astype(
        np.float32,
        copy=False,
    )


def navigation_fixture_ground_body_points(
    robot_z_m: float,
    *,
    orientation_xyzw: Any = (0.0, 0.0, 0.0, 1.0),
    resolution_m: float = NAVIGATION_FIXTURE_GROUND_RESOLUTION_M,
    y_half_m: float = NAVIGATION_FIXTURE_GROUND_Y_HALF_M,
) -> np.ndarray:
    """Build deterministic world-horizontal ground in the scan-time body frame."""

    robot_z = float(robot_z_m)
    resolution = float(resolution_m)
    y_half = float(y_half_m)
    if not math.isfinite(robot_z):
        raise ValueError("robot_z_m must be finite")
    if not math.isfinite(resolution) or resolution <= 0.0 or resolution > 0.2:
        raise ValueError("navigation fixture ground resolution must be in (0, 0.2]")
    if not math.isfinite(y_half) or y_half <= 0.0:
        raise ValueError("navigation fixture ground y half-width must be positive and finite")
    xs = np.arange(
        _NAVIGATION_FIXTURE_GROUND_X_MIN_M,
        _NAVIGATION_FIXTURE_GROUND_X_MAX_M + resolution * 0.5,
        resolution,
        dtype=np.float64,
    )
    ys = np.arange(
        -y_half,
        y_half + resolution * 0.5,
        resolution,
        dtype=np.float64,
    )
    grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")
    count = int(grid_x.size)
    orientation = np.asarray(orientation_xyzw, dtype=np.float64).reshape(-1)
    if orientation.size != 4 or not bool(np.isfinite(orientation).all()):
        raise ValueError("body orientation must contain finite XYZW")
    rotation_body_to_world = quat_xyzw_to_matrix(orientation)
    yaw = yaw_from_quat_xyzw(orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    horizontal_body_grid_world = np.column_stack(
        (
            grid_x.reshape(count) * cos_yaw - grid_y.reshape(count) * sin_yaw,
            grid_x.reshape(count) * sin_yaw + grid_y.reshape(count) * cos_yaw,
            np.full(count, -robot_z, dtype=np.float64),
        )
    )
    xyz_body = horizontal_body_grid_world @ rotation_body_to_world
    return np.column_stack(
        (
            xyz_body,
            np.full(count, _NAVIGATION_FIXTURE_GROUND_INTENSITY, dtype=np.float64),
        )
    ).astype(np.float32, copy=False)


def navigation_fixture_registered_body_points(
    raw_body_points: Any,
    state: Any,
    *,
    max_points: int,
    raw_overlay_enabled: bool = True,
    ground_resolution_m: float = NAVIGATION_FIXTURE_GROUND_RESOLUTION_M,
    ground_y_half_m: float = NAVIGATION_FIXTURE_GROUND_Y_HALF_M,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Compose fixture ground and an optional obstacle-preserving raw overlay."""

    def bounded(points: Any, maximum: int) -> np.ndarray:
        values = np.asarray(points, dtype=np.float32)
        if values.size == 0:
            return np.zeros((0, 4), dtype=np.float32)
        if values.ndim != 2 or values.shape[1] < 3:
            raise ValueError(f"expected point cloud shape (N, >=3), got {values.shape}")
        if values.shape[1] == 3:
            values = np.column_stack(
                (values, np.full((len(values),), 100.0, dtype=np.float32))
            )
        else:
            values = values[:, :4]
        if not bool(np.isfinite(values).all()):
            raise ValueError("navigation fixture raw points must be finite")
        if maximum == 0:
            return np.zeros((0, 4), dtype=np.float32)
        if values.shape[0] > maximum:
            stride = math.ceil(values.shape[0] / maximum)
            values = values[::stride][:maximum]
        return values.astype(np.float32, copy=False)

    if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points <= 0:
        raise ValueError("navigation fixture max_points must be a positive integer")
    if not isinstance(raw_overlay_enabled, bool):
        raise TypeError("navigation fixture raw_overlay_enabled must be bool")

    raw = bounded(raw_body_points, max_points)
    robot_z = float(np.asarray(state.position, dtype=np.float64).reshape(3)[2])
    orientation = np.asarray(
        getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)),
        dtype=np.float64,
    ).reshape(-1)
    if orientation.size != 4 or not bool(np.isfinite(orientation).all()):
        raise ValueError("body orientation must contain finite XYZW")
    rotation_body_to_world = quat_xyzw_to_matrix(orientation)
    ground = navigation_fixture_ground_body_points(
        robot_z,
        orientation_xyzw=orientation,
        resolution_m=ground_resolution_m,
        y_half_m=ground_y_half_m,
    )
    ground_count = int(ground.shape[0])
    if max_points < ground_count:
        raise ValueError(
            "navigation fixture cloud point budget must fit synthetic ground coverage"
        )
    raw_budget = max_points - ground_count
    height_above_ground = (
        robot_z + raw[:, :3].astype(np.float64) @ rotation_body_to_world[2, :]
    )
    obstacle_mask = (
        height_above_ground > _NAVIGATION_FIXTURE_RAW_OVERLAY_MIN_ABOVE_GROUND_M
    )
    if not raw_overlay_enabled or raw.shape[0] == 0:
        raw_overlay = np.zeros((0, 4), dtype=np.float32)
        obstacle_overlay = np.zeros((0, 4), dtype=np.float32)
        context_overlay = np.zeros((0, 4), dtype=np.float32)
    elif raw.shape[0] <= raw_budget:
        raw_overlay = raw
        obstacle_overlay = raw[obstacle_mask]
        context_overlay = raw[~obstacle_mask]
    else:
        raw_obstacles = raw[obstacle_mask]
        raw_context = raw[~obstacle_mask]
        obstacle_budget = min(int(raw_obstacles.shape[0]), raw_budget)
        obstacle_overlay = bounded(raw_obstacles, obstacle_budget)
        context_budget = max(0, raw_budget - int(obstacle_overlay.shape[0]))
        context_overlay = bounded(raw_context, context_budget)
        raw_overlay = (
            np.vstack((obstacle_overlay, context_overlay))
            if obstacle_overlay.shape[0] or context_overlay.shape[0]
            else np.zeros((0, 4), dtype=np.float32)
        )
    combined = (
        np.vstack((ground, raw_overlay)).astype(np.float32, copy=False)
        if raw_overlay.shape[0]
        else ground
    )
    return combined, {
        "enabled": True,
        "synthetic_ground_points": ground_count,
        "raw_body_points": int(raw.shape[0]),
        "raw_overlay_points": int(raw_overlay.shape[0]),
        "raw_obstacle_overlay_points": int(obstacle_overlay.shape[0]),
        "raw_context_overlay_points": int(context_overlay.shape[0]),
        "raw_overlay_enabled": raw_overlay_enabled,
        "published_points": int(combined.shape[0]),
        "max_points": max_points,
        "x_min_m": _NAVIGATION_FIXTURE_GROUND_X_MIN_M,
        "x_max_m": _NAVIGATION_FIXTURE_GROUND_X_MAX_M,
        "y_half_m": float(ground_y_half_m),
        "resolution_m": float(ground_resolution_m),
        "z_m": -robot_z,
    }


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
        ring_values = np.zeros(n_pts, dtype=np.uint16)
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
        ring_values = np.zeros(n_pts, dtype=np.uint16)
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
        point.tag = 0x00
        point.line = int(ring_values[idx])
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
