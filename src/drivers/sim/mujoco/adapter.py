"""MuJoCo adapter for LingTu portable bottom-layer contracts.

MuJoCo is one caller of the portable contract, not the center of the runtime.
This adapter converts a ``SimEngine``-like object into
``PortableSensorFrame``/``PortableCommandFrame`` so the same contract can also be
used by replay, ROS-compat, and hardware adapters.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat, Imu, PointCloud2
from runtime.portable.contracts import PortableCommandFrame, PortableSensorFrame
from runtime.portable.topic_transport import PortableTopicTransport
from runtime.runtime_interface import FRAMES, TOPICS, topic_default_frame_id

MUJOCO_PORTABLE_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
MUJOCO_PORTABLE_BODY_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
MUJOCO_PORTABLE_MAP_CLOUD_FRAME_ID = MUJOCO_PORTABLE_ODOM_FRAME_ID
MUJOCO_PORTABLE_CAMERA_FRAME_ID = FRAMES.camera
MUJOCO_PORTABLE_IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)


@dataclass
class MujocoPortableAdapter:
    """Translate MuJoCo engine state/commands to portable LingTu frames."""

    engine: Any
    source: str = "mujoco"
    camera_name: str = "front_camera"
    include_camera: bool = True

    def poll(self) -> PortableSensorFrame:
        """Read the current engine state without advancing physics."""

        state = self.engine.get_robot_state()
        return self.sensor_frame_from_state(state)

    def step(self, command: PortableCommandFrame | None = None) -> PortableSensorFrame:
        """Advance one engine step and return a portable sensor frame."""

        velocity = self.velocity_command_from_frame(command) if command is not None else None
        state = self.engine.step(velocity)
        return self.sensor_frame_from_state(state)

    def apply(self, command: PortableCommandFrame) -> None:
        """Apply a command to the engine's async command sink."""

        velocity = self.velocity_command_from_frame(command)
        self.engine.set_cmd_vel(velocity)

    def publish(self, transport: PortableTopicTransport, frame: PortableSensorFrame | None = None) -> None:
        """Publish a frame through Local/SHM/LCM/DDS transport."""

        transport.publish_sensor_frame(frame or self.poll())

    def sensor_frame_from_state(self, state: Any) -> PortableSensorFrame:
        ts = time.time()
        odometry = self._odometry_from_state(state, ts)
        imu = self._imu_from_state(state, ts)
        lidar_cloud = None
        map_cloud = None
        try:
            pts = self.engine.get_lidar_points()
        except Exception:
            pts = None
        if pts is not None and len(pts) > 0:
            pts_world = np.asarray(pts, dtype=np.float32)
            pts_body = world_points_to_body_frame(
                pts_world,
                np.asarray(state.position, dtype=float),
                np.asarray(state.orientation, dtype=float),
            )
            lidar_cloud = PointCloud2(
                points=pts_body,
                frame_id=MUJOCO_PORTABLE_BODY_FRAME_ID,
                ts=ts,
            )
            map_cloud = PointCloud2(
                points=pts_world,
                frame_id=MUJOCO_PORTABLE_MAP_CLOUD_FRAME_ID,
                ts=ts,
            )

        camera_image = None
        depth_image = None
        camera_info = None
        if self.include_camera:
            try:
                camera = self.engine.get_camera_data(self.camera_name)
            except Exception:
                camera = None
            if camera is not None:
                camera_image, depth_image, camera_info = self._camera_bundle(camera, ts)

        return PortableSensorFrame(
            odometry=odometry,
            lidar_cloud=lidar_cloud,
            map_cloud=map_cloud,
            imu=imu,
            camera_image=camera_image,
            depth_image=depth_image,
            camera_info=camera_info,
            timestamp_s=ts,
            source=self.source,
        )

    @staticmethod
    def velocity_command_from_frame(command: PortableCommandFrame | None) -> Any:
        from sim.engine.core.engine import VelocityCommand

        if command is None or command.requests_stop or command.cmd_vel is None:
            return VelocityCommand()
        twist = command.cmd_vel
        return VelocityCommand(
            linear_x=float(getattr(twist.linear, "x", 0.0)),
            linear_y=float(getattr(twist.linear, "y", 0.0)),
            angular_z=float(getattr(twist.angular, "z", 0.0)),
        )

    @staticmethod
    def _odometry_from_state(state: Any, ts: float) -> Odometry:
        quat = np.asarray(state.orientation, dtype=float)
        return Odometry(
            pose=Pose(
                position=Vector3(
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.position[2]),
                ),
                orientation=Quaternion(
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                ),
            ),
            twist=Twist(
                linear=Vector3(
                    float(state.linear_velocity[0]),
                    float(state.linear_velocity[1]),
                    float(state.linear_velocity[2]),
                ),
                angular=Vector3(
                    float(state.angular_velocity[0]),
                    float(state.angular_velocity[1]),
                    float(state.angular_velocity[2]),
                ),
            ),
            ts=ts,
            frame_id=MUJOCO_PORTABLE_ODOM_FRAME_ID,
            child_frame_id=MUJOCO_PORTABLE_BODY_FRAME_ID,
        )

    @staticmethod
    def _imu_from_state(state: Any, ts: float) -> Imu:
        quat = np.asarray(state.orientation, dtype=float)
        gyro = np.asarray(getattr(state, "imu_gyro", (0.0, 0.0, 0.0)), dtype=float)
        accel = np.asarray(getattr(state, "imu_linear_acceleration", (0.0, 0.0, 0.0)), dtype=float)
        return Imu(
            orientation=Quaternion(
                float(quat[0]),
                float(quat[1]),
                float(quat[2]),
                float(quat[3]),
            ),
            angular_velocity=Vector3(float(gyro[0]), float(gyro[1]), float(gyro[2])),
            linear_acceleration=Vector3(
                float(accel[0]),
                float(accel[1]),
                float(accel[2]),
            ),
            ts=ts,
            frame_id=MUJOCO_PORTABLE_IMU_FRAME_ID,
        )

    @staticmethod
    def _camera_bundle(camera: Any, ts: float) -> tuple[Image | None, Image | None, CameraIntrinsics | None]:
        rgb = getattr(camera, "rgb", None)
        depth = getattr(camera, "depth", None)
        if rgb is not None:
            height, width = rgb.shape[:2]
        elif depth is not None:
            height, width = depth.shape[:2]
        else:
            return None, None, None

        camera_image = None
        if rgb is not None:
            camera_image = Image(
                data=rgb,
                format=ImageFormat.RGB,
                ts=ts,
                frame_id=MUJOCO_PORTABLE_CAMERA_FRAME_ID,
            )
        depth_image = None
        if depth is not None:
            depth_image = Image(
                data=depth,
                format=ImageFormat.DEPTH_F32,
                ts=ts,
                frame_id=MUJOCO_PORTABLE_CAMERA_FRAME_ID,
            )
        fx, fy, cx, cy = camera.intrinsics
        camera_info = CameraIntrinsics(
            fx=float(fx),
            fy=float(fy),
            cx=float(cx),
            cy=float(cy),
            width=int(width),
            height=int(height),
            depth_scale=1.0,
        )
        return camera_image, depth_image, camera_info


def quat_xyzw_to_rotation_matrix(quat: Any) -> np.ndarray:
    q = np.asarray(quat, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        return np.eye(3, dtype=float)
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def world_points_to_body_frame(points: Any, position_xyz: Any, orientation_xyzw: Any) -> np.ndarray:
    cloud = np.asarray(points, dtype=np.float32).copy()
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        return cloud
    rotation_body_to_world = quat_xyzw_to_rotation_matrix(orientation_xyzw)
    relative_world = cloud[:, :3].astype(float) - np.asarray(position_xyz, dtype=float).reshape(3)
    cloud[:, :3] = (relative_world @ rotation_body_to_world).astype(np.float32)
    return cloud
