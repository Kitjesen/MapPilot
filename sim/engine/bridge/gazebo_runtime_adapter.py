#!/usr/bin/env python3
"""Normalize private Gazebo/GZ ROS topics into LingTu runtime topics.

This adapter is deliberately simulation-only. It does not publish goals or
commands. Its job is to keep raw Gazebo frames and topics out of `/nav/*` by
owning the simulator boundary:

  /lingtu/gazebo/raw/* -> /slam/odometry, /slam/map_cloud, camera topics
  TF: map -> odom, odom -> body

The point-cloud path treats raw Gazebo lidar points as sensor-frame points,
projects them through the configured body->lidar mounting extrinsic, and then projects
the live cloud through the timestamp-matched odom->body pose. This keeps
`/slam/map_cloud` live and odom-fixed, `/slam/registered_cloud` body-relative,
`/nav/terrain_map` as the CMU-style local-planner input, and
`/slam/cumulative_map_cloud` plus `/nav/terrain_map_ext` as simulation-only
accumulated maps for Gazebo deliverable gates.
"""

from __future__ import annotations

import copy
import math
import sys
import time
from collections import deque
from dataclasses import dataclass

from runtime.runtime_interface import FRAMES, lidar_extrinsic, rpy_to_quaternion_xyzw


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class TimedPose:
    stamp_sec: float
    pose: Pose3


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_rotate(point: tuple[float, float, float], quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Rotate a point by a normalized xyzw quaternion."""

    x, y, z = point
    qx, qy, qz, qw = quat
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return point
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # q * p * q^-1, expanded to avoid temporary quaternion objects.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _transform_xyz(
    point: tuple[float, float, float],
    *,
    translation: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    rx, ry, rz = _quat_rotate(point, rotation_xyzw)
    tx, ty, tz = translation
    return rx + tx, ry + ty, rz + tz


def _odom_xyz_to_body(
    point: tuple[float, float, float],
    pose: Pose3,
) -> tuple[float, float, float]:
    translated = (point[0] - pose.x, point[1] - pose.y, point[2] - pose.z)
    return _quat_rotate(translated, (-pose.qx, -pose.qy, -pose.qz, pose.qw))


def main() -> int:
    try:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
        from sensor_msgs_py import point_cloud2
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    except ImportError as exc:
        print(f"Gazebo runtime adapter requires ROS 2 Python packages: {exc}", file=sys.stderr)
        return 2

    from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig

    class GazeboRuntimeAdapter(Node):
        def __init__(self) -> None:
            super().__init__("lingtu_gazebo_runtime_adapter")
            self._cfg = GazeboBridgeConfig()
            default_lidar = lidar_extrinsic("gazebo_proxy")
            raw = self._cfg.raw_ros_topics()
            self.declare_parameter("raw_lidar_frame", self._cfg.frames.lidar_frame)
            self.declare_parameter("body_to_lidar_x", default_lidar.x)
            self.declare_parameter("body_to_lidar_y", default_lidar.y)
            self.declare_parameter("body_to_lidar_z", default_lidar.z)
            self.declare_parameter("body_to_lidar_roll", default_lidar.roll)
            self.declare_parameter("body_to_lidar_pitch", default_lidar.pitch)
            self.declare_parameter("body_to_lidar_yaw", default_lidar.yaw)
            # Backwards-compatible parameter aliases for older gate scripts.
            self.declare_parameter("lidar_to_body_x", default_lidar.x)
            self.declare_parameter("lidar_to_body_y", default_lidar.y)
            self.declare_parameter("lidar_to_body_z", default_lidar.z)
            self.declare_parameter("lidar_to_body_roll", default_lidar.roll)
            self.declare_parameter("lidar_to_body_pitch", default_lidar.pitch)
            self.declare_parameter("lidar_to_body_yaw", default_lidar.yaw)
            self.declare_parameter("prefer_point_cloud_over_scan", True)
            self.declare_parameter("scan_fallback_after_sec", 0.75)
            self.declare_parameter("cloud_min_range", 0.25)
            self.declare_parameter("cloud_max_range", 8.0)
            self.declare_parameter("cloud_min_body_z", 0.05)
            self.declare_parameter("cloud_max_body_z", 1.60)
            self.declare_parameter("cloud_self_filter_forward", 0.65)
            self.declare_parameter("cloud_self_filter_backward", 0.50)
            self.declare_parameter("cloud_self_filter_lateral", 0.44)
            self.declare_parameter("cloud_self_filter_min_z", -0.30)
            self.declare_parameter("cloud_self_filter_max_z", 0.48)
            self.declare_parameter("max_cloud_odom_stamp_lag_sec", 0.25)
            self.declare_parameter("terrain_height_reference_z", 0.0)
            self.declare_parameter("terrain_min_height", 0.05)
            self.declare_parameter("terrain_max_height", 1.60)
            self.declare_parameter("terrain_max_body_range", 4.0)
            self.declare_parameter("terrain_map_ext_max_body_range", 8.0)
            self.declare_parameter("terrain_self_filter_forward", 0.62)
            self.declare_parameter("terrain_self_filter_backward", 0.40)
            self.declare_parameter("terrain_self_filter_lateral", 0.34)
            self.declare_parameter("terrain_self_filter_min_z", -0.30)
            self.declare_parameter("terrain_self_filter_max_z", 0.36)
            self.declare_parameter("terrain_ext_exclude_near_field", True)
            self.declare_parameter("terrain_ext_near_field_forward", 0.60)
            self.declare_parameter("terrain_ext_near_field_lateral", 0.25)
            self.declare_parameter("publish_cumulative_map", True)
            self.declare_parameter("cumulative_voxel_size", 0.08)
            self.declare_parameter("cumulative_max_points", 80000)
            self.declare_parameter("cumulative_min_range", 0.25)
            self.declare_parameter("cumulative_max_range", 6.0)
            self.declare_parameter("cumulative_min_z", 0.08)
            self.declare_parameter("cumulative_max_z", 2.00)
            self._raw_lidar_frame = str(self.get_parameter("raw_lidar_frame").value)
            self._body_to_lidar = (
                self._mount_param("x", default_lidar.x),
                self._mount_param("y", default_lidar.y),
                self._mount_param("z", default_lidar.z),
            )
            self._body_to_lidar_rpy = (
                self._mount_param("roll", default_lidar.roll),
                self._mount_param("pitch", default_lidar.pitch),
                self._mount_param("yaw", default_lidar.yaw),
            )
            self._body_to_lidar_rotation = rpy_to_quaternion_xyzw(*self._body_to_lidar_rpy)
            self._prefer_point_cloud_over_scan = bool(
                self.get_parameter("prefer_point_cloud_over_scan").value
            )
            self._scan_fallback_after_sec = float(
                self.get_parameter("scan_fallback_after_sec").value
            )
            self._cloud_min_range = max(
                0.0,
                float(self.get_parameter("cloud_min_range").value),
            )
            self._cloud_max_range = max(
                self._cloud_min_range,
                float(self.get_parameter("cloud_max_range").value),
            )
            self._cloud_min_body_z = float(self.get_parameter("cloud_min_body_z").value)
            self._cloud_max_body_z = float(self.get_parameter("cloud_max_body_z").value)
            self._cloud_self_filter_forward = max(
                0.0,
                float(self.get_parameter("cloud_self_filter_forward").value),
            )
            self._cloud_self_filter_backward = max(
                0.0,
                float(self.get_parameter("cloud_self_filter_backward").value),
            )
            self._cloud_self_filter_lateral = max(
                0.0,
                float(self.get_parameter("cloud_self_filter_lateral").value),
            )
            self._cloud_self_filter_min_z = float(
                self.get_parameter("cloud_self_filter_min_z").value
            )
            self._cloud_self_filter_max_z = float(
                self.get_parameter("cloud_self_filter_max_z").value
            )
            self._max_cloud_odom_stamp_lag_sec = max(
                0.0,
                float(self.get_parameter("max_cloud_odom_stamp_lag_sec").value),
            )
            self._terrain_height_reference_z = float(
                self.get_parameter("terrain_height_reference_z").value
            )
            self._terrain_min_height = max(
                0.0,
                float(self.get_parameter("terrain_min_height").value),
            )
            self._terrain_max_height = max(
                self._terrain_min_height,
                float(self.get_parameter("terrain_max_height").value),
            )
            self._terrain_max_body_range = max(
                0.1,
                float(self.get_parameter("terrain_max_body_range").value),
            )
            self._terrain_map_ext_max_body_range = max(
                self._terrain_max_body_range,
                float(self.get_parameter("terrain_map_ext_max_body_range").value),
            )
            self._terrain_self_filter_forward = max(
                0.0,
                float(self.get_parameter("terrain_self_filter_forward").value),
            )
            self._terrain_self_filter_backward = max(
                0.0,
                float(self.get_parameter("terrain_self_filter_backward").value),
            )
            self._terrain_self_filter_lateral = max(
                0.0,
                float(self.get_parameter("terrain_self_filter_lateral").value),
            )
            self._terrain_self_filter_min_z = float(
                self.get_parameter("terrain_self_filter_min_z").value
            )
            self._terrain_self_filter_max_z = float(
                self.get_parameter("terrain_self_filter_max_z").value
            )
            self._terrain_ext_exclude_near_field = bool(
                self.get_parameter("terrain_ext_exclude_near_field").value
            )
            self._terrain_ext_near_field_forward = max(
                0.0,
                float(self.get_parameter("terrain_ext_near_field_forward").value),
            )
            self._terrain_ext_near_field_lateral = max(
                0.0,
                float(self.get_parameter("terrain_ext_near_field_lateral").value),
            )
            self._publish_cumulative_map_enabled = bool(
                self.get_parameter("publish_cumulative_map").value
            )
            self._cumulative_voxel_size = max(
                0.01,
                float(self.get_parameter("cumulative_voxel_size").value),
            )
            self._cumulative_max_points = max(
                1,
                int(self.get_parameter("cumulative_max_points").value),
            )
            self._cumulative_min_range = max(
                0.0,
                float(self.get_parameter("cumulative_min_range").value),
            )
            self._cumulative_max_range = max(
                self._cumulative_min_range,
                float(self.get_parameter("cumulative_max_range").value),
            )
            self._cumulative_min_z = float(self.get_parameter("cumulative_min_z").value)
            self._cumulative_max_z = float(self.get_parameter("cumulative_max_z").value)
            self._latest_body_pose = Pose3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            self._pose_history: deque[TimedPose] = deque(maxlen=300)
            self._last_point_cloud_wall_time = 0.0
            self._cumulative_voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}

            self._odom_pub = self.create_publisher(Odometry, self._cfg.lingtu_odometry, 10)
            self._map_cloud_pub = self.create_publisher(PointCloud2, self._cfg.lingtu_map_cloud, 2)
            self._terrain_map_pub = self.create_publisher(PointCloud2, self._cfg.lingtu_terrain_map, 2)
            self._terrain_map_ext_pub = self.create_publisher(
                PointCloud2,
                self._cfg.lingtu_terrain_map_ext,
                2,
            )
            self._cumulative_map_cloud_pub = self.create_publisher(
                PointCloud2,
                self._cfg.lingtu_cumulative_map_cloud,
                2,
            )
            self._registered_cloud_pub = self.create_publisher(
                PointCloud2,
                self._cfg.lingtu_registered_cloud,
                2,
            )
            self._color_pub = self.create_publisher(Image, self._cfg.lingtu_color_image, 2)
            self._depth_pub = self.create_publisher(Image, self._cfg.lingtu_depth_image, 2)
            self._info_pub = self.create_publisher(CameraInfo, self._cfg.lingtu_camera_info, 2)

            self._tf_pub = TransformBroadcaster(self)
            self._static_tf_pub = StaticTransformBroadcaster(self)
            self._publish_static_map_to_odom()

            self.create_subscription(Odometry, raw["odometry"], self._on_odom, 10)
            self.create_subscription(PointCloud2, raw["lidar_points"], self._on_cloud, 2)
            self.create_subscription(LaserScan, raw["lidar_scan"], self._on_scan, 2)
            self.create_subscription(Image, raw["color_image"], self._on_color, 2)
            self.create_subscription(Image, raw["depth_image"], self._on_depth, 2)
            self.create_subscription(CameraInfo, raw["camera_info"], self._on_info, 2)

            self.get_logger().info(
                "Gazebo runtime adapter: raw Gazebo topics -> LingTu /nav topics, "
                "TF map->odom->body"
            )

        def _mount_param(self, suffix: str, default: float) -> float:
            """Read canonical body_to_lidar_* params with legacy alias fallback."""

            canonical = float(self.get_parameter(f"body_to_lidar_{suffix}").value)
            legacy = float(self.get_parameter(f"lidar_to_body_{suffix}").value)
            if canonical == float(default) and legacy != float(default):
                return legacy
            return canonical

        def _publish_static_map_to_odom(self) -> None:
            stamp = self.get_clock().now().to_msg()
            tfs = []

            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = stamp
            map_to_odom.header.frame_id = FRAMES.map
            map_to_odom.child_frame_id = FRAMES.odom
            map_to_odom.transform.rotation.w = 1.0
            tfs.append(map_to_odom)

            body_to_lidar = TransformStamped()
            body_to_lidar.header.stamp = stamp
            body_to_lidar.header.frame_id = FRAMES.body
            body_to_lidar.child_frame_id = self._cfg.frames.lidar_frame
            body_to_lidar.transform.translation.x = self._body_to_lidar[0]
            body_to_lidar.transform.translation.y = self._body_to_lidar[1]
            body_to_lidar.transform.translation.z = self._body_to_lidar[2]
            (
                body_to_lidar.transform.rotation.x,
                body_to_lidar.transform.rotation.y,
                body_to_lidar.transform.rotation.z,
                body_to_lidar.transform.rotation.w,
            ) = self._body_to_lidar_rotation
            tfs.append(body_to_lidar)

            body_to_camera = TransformStamped()
            body_to_camera.header.stamp = stamp
            body_to_camera.header.frame_id = FRAMES.body
            body_to_camera.child_frame_id = self._cfg.frames.camera_frame
            body_to_camera.transform.translation.x = 0.36
            body_to_camera.transform.translation.y = 0.0
            body_to_camera.transform.translation.z = 0.22
            body_to_camera.transform.rotation.w = 1.0
            tfs.append(body_to_camera)

            self._static_tf_pub.sendTransform(tfs)

        def _on_odom(self, msg: Odometry) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = FRAMES.odom
            out.child_frame_id = FRAMES.body
            self._odom_pub.publish(out)
            pose = Pose3(
                out.pose.pose.position.x,
                out.pose.pose.position.y,
                out.pose.pose.position.z,
                out.pose.pose.orientation.x,
                out.pose.pose.orientation.y,
                out.pose.pose.orientation.z,
                out.pose.pose.orientation.w,
            )
            self._latest_body_pose = pose
            self._pose_history.append(TimedPose(_stamp_sec(out.header.stamp), pose))

            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = FRAMES.odom
            tf.child_frame_id = FRAMES.body
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self._tf_pub.sendTransform(tf)

        def _on_cloud(self, msg: PointCloud2) -> None:
            self._last_point_cloud_wall_time = time.monotonic()
            registered_cloud = self._transform_cloud(msg, target_frame=FRAMES.body)
            if self._point_count(registered_cloud) <= 0:
                return
            registered_cloud.header.stamp = self._matched_stamp_msg(
                _stamp_sec(registered_cloud.header.stamp)
            )
            self._registered_cloud_pub.publish(registered_cloud)

            map_cloud = self._body_cloud_to_odom(registered_cloud)
            map_cloud.header.frame_id = FRAMES.odom
            if self._point_count(map_cloud) <= 0:
                return
            self._map_cloud_pub.publish(map_cloud)
            self._terrain_map_pub.publish(
                self._terrain_cloud_from_xyz(
                    map_cloud,
                    max_body_range=self._terrain_max_body_range,
                )
            )
            self._publish_cumulative_map(map_cloud)

        def _on_scan(self, msg: LaserScan) -> None:
            if (
                self._prefer_point_cloud_over_scan
                and self._last_point_cloud_wall_time > 0.0
                and time.monotonic() - self._last_point_cloud_wall_time
                <= self._scan_fallback_after_sec
            ):
                return
            points: list[tuple[float, float, float]] = []
            angle = float(msg.angle_min)
            for distance in msg.ranges:
                r = float(distance)
                if (
                    math.isfinite(r)
                    and float(msg.range_min) <= r <= float(msg.range_max)
                    and self._cloud_min_range <= r <= self._cloud_max_range
                ):
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    body_point = _transform_xyz(
                        (x, y, 0.0),
                        translation=self._body_to_lidar,
                        rotation_xyzw=self._body_to_lidar_rotation,
                    )
                    if self._body_cloud_point_allowed(body_point):
                        points.append(body_point)
                angle += float(msg.angle_increment)

            header = copy.deepcopy(msg.header)
            header.frame_id = FRAMES.body
            registered_cloud = point_cloud2.create_cloud_xyz32(header, points)
            registered_cloud.header.stamp = self._matched_stamp_msg(_stamp_sec(msg.header.stamp))
            self._registered_cloud_pub.publish(registered_cloud)

            map_cloud = self._body_cloud_to_odom(registered_cloud)
            map_cloud.header.frame_id = FRAMES.odom
            self._map_cloud_pub.publish(map_cloud)
            self._terrain_map_pub.publish(
                self._terrain_cloud_from_xyz(
                    map_cloud,
                    max_body_range=self._terrain_max_body_range,
                )
            )
            self._publish_cumulative_map(map_cloud)

        def _transform_cloud(self, msg: PointCloud2, *, target_frame: str) -> PointCloud2:
            fields = {field.name for field in msg.fields}
            if not {"x", "y", "z"} <= fields:
                self.get_logger().warn("raw lidar cloud has no x/y/z fields; publishing empty normalized cloud")
                header = copy.deepcopy(msg.header)
                header.frame_id = target_frame
                return point_cloud2.create_cloud(header, msg.fields, [])
            points = []
            names = [field.name for field in msg.fields]
            in_body_frame = self._raw_lidar_frame in (FRAMES.body, *FRAMES.body_aliases)
            for raw in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
                values = list(raw)
                x_idx, y_idx, z_idx = names.index("x"), names.index("y"), names.index("z")
                body_point = (
                    (float(values[x_idx]), float(values[y_idx]), float(values[z_idx]))
                    if in_body_frame
                    else _transform_xyz(
                        (float(values[x_idx]), float(values[y_idx]), float(values[z_idx])),
                        translation=self._body_to_lidar,
                        rotation_xyzw=self._body_to_lidar_rotation,
                    )
                )
                if not self._body_cloud_point_allowed(body_point):
                    continue
                values[x_idx], values[y_idx], values[z_idx] = body_point
                points.append(values)
            header = copy.deepcopy(msg.header)
            header.frame_id = target_frame
            return point_cloud2.create_cloud(header, msg.fields, points)

        def _body_cloud_point_allowed(self, point: tuple[float, float, float]) -> bool:
            x, y, z = point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return False
            if z < self._cloud_min_body_z or z > self._cloud_max_body_z:
                return False
            if self._is_body_self_filter_point((x, y, z)):
                return False
            distance = math.sqrt(x * x + y * y + z * z)
            return self._cloud_min_range <= distance <= self._cloud_max_range

        def _body_cloud_to_odom(self, msg: PointCloud2) -> PointCloud2:
            timed_pose = self._timed_pose_for_stamp(_stamp_sec(msg.header.stamp))
            pose = timed_pose.pose
            names = [field.name for field in msg.fields]
            if not {"x", "y", "z"} <= set(names):
                return self._copy_cloud_with_frame(msg, FRAMES.odom)
            points = []
            for raw in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
                values = list(raw)
                x_idx, y_idx, z_idx = names.index("x"), names.index("y"), names.index("z")
                values[x_idx], values[y_idx], values[z_idx] = _transform_xyz(
                    (float(values[x_idx]), float(values[y_idx]), float(values[z_idx])),
                    translation=(pose.x, pose.y, pose.z),
                    rotation_xyzw=(pose.qx, pose.qy, pose.qz, pose.qw),
                )
                points.append(values)
            header = copy.deepcopy(msg.header)
            header.frame_id = FRAMES.odom
            header.stamp = self._stamp_msg_from_sec(timed_pose.stamp_sec)
            return point_cloud2.create_cloud(header, msg.fields, points)

        def _pose_for_stamp(self, stamp_sec: float) -> Pose3:
            return self._timed_pose_for_stamp(stamp_sec).pose

        def _timed_pose_for_stamp(self, stamp_sec: float) -> TimedPose:
            if not self._pose_history or stamp_sec <= 0.0:
                return TimedPose(stamp_sec, self._latest_body_pose)
            latest = self._pose_history[-1]
            if abs(latest.stamp_sec - stamp_sec) > self._max_cloud_odom_stamp_lag_sec:
                return latest
            return min(
                self._pose_history,
                key=lambda item: abs(item.stamp_sec - stamp_sec),
            )

        def _matched_stamp_msg(self, stamp_sec: float):
            return self._stamp_msg_from_sec(self._timed_pose_for_stamp(stamp_sec).stamp_sec)

        @staticmethod
        def _stamp_msg_from_sec(stamp_sec: float):
            sec = int(math.floor(max(0.0, float(stamp_sec))))
            nanosec = int(round((max(0.0, float(stamp_sec)) - sec) * 1_000_000_000))
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
            from builtin_interfaces.msg import Time

            stamp = Time()
            stamp.sec = sec
            stamp.nanosec = nanosec
            return stamp

        def _copy_cloud_with_frame(self, msg: PointCloud2, frame_id: str) -> PointCloud2:
            out = copy.deepcopy(msg)
            out.header.frame_id = frame_id
            return out

        def _terrain_cloud_from_xyz(
            self,
            msg: PointCloud2,
            *,
            max_body_range: float,
            exclude_near_field: bool = False,
        ) -> PointCloud2:
            names = {field.name for field in msg.fields}
            header = copy.deepcopy(msg.header)
            header.frame_id = FRAMES.odom
            if not {"x", "y", "z"} <= names:
                return self._create_xyzi_cloud(header, [])
            points = []
            pose = self._pose_for_stamp(_stamp_sec(msg.header.stamp))
            max_body_range_sq = max_body_range * max_body_range
            for x, y, z in point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                px, py, pz = float(x), float(y), float(z)
                if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                    continue
                height = max(0.0, pz - self._terrain_height_reference_z)
                if height < self._terrain_min_height or height > self._terrain_max_height:
                    continue
                bx, by, bz = _odom_xyz_to_body((px, py, pz), pose)
                if bx * bx + by * by + bz * bz > max_body_range_sq:
                    continue
                if self._is_self_filter_point((bx, by, bz)):
                    continue
                if exclude_near_field and self._is_terrain_ext_near_field_point((bx, by, bz)):
                    continue
                points.append((px, py, pz, height))
            return self._create_xyzi_cloud(header, points)

        def _is_self_filter_point(self, point: tuple[float, float, float]) -> bool:
            bx, by, bz = point
            return (
                -self._terrain_self_filter_backward <= bx <= self._terrain_self_filter_forward
                and abs(by) <= self._terrain_self_filter_lateral
                and self._terrain_self_filter_min_z <= bz <= self._terrain_self_filter_max_z
            )

        def _is_body_self_filter_point(self, point: tuple[float, float, float]) -> bool:
            bx, by, bz = point
            return (
                -self._cloud_self_filter_backward <= bx <= self._cloud_self_filter_forward
                and abs(by) <= self._cloud_self_filter_lateral
                and self._cloud_self_filter_min_z <= bz <= self._cloud_self_filter_max_z
            )

        def _is_terrain_ext_near_field_point(self, point: tuple[float, float, float]) -> bool:
            bx, by, _ = point
            return (
                0.0 < bx < self._terrain_ext_near_field_forward
                and abs(by) < self._terrain_ext_near_field_lateral
            )

        @staticmethod
        def _create_xyzi_cloud(header, points: list[tuple[float, float, float, float]]) -> PointCloud2:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            return point_cloud2.create_cloud(header, fields, points)

        def _publish_cumulative_map(self, map_cloud: PointCloud2) -> None:
            if not self._publish_cumulative_map_enabled:
                return
            names = {field.name for field in map_cloud.fields}
            if not {"x", "y", "z"} <= names:
                return
            min_range = self._cumulative_min_range
            max_range = self._cumulative_max_range
            voxel_size = self._cumulative_voxel_size
            pose = self._pose_for_stamp(_stamp_sec(map_cloud.header.stamp))
            for x, y, z in point_cloud2.read_points(
                map_cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                px, py, pz = float(x), float(y), float(z)
                if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                    continue
                if pz < self._cumulative_min_z or pz > self._cumulative_max_z:
                    continue
                dx = px - pose.x
                dy = py - pose.y
                dz = pz - pose.z
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                if distance < min_range or distance > max_range:
                    continue
                key = (
                    math.floor(px / voxel_size),
                    math.floor(py / voxel_size),
                    math.floor(pz / voxel_size),
                )
                self._cumulative_voxels[key] = (px, py, pz)
            while len(self._cumulative_voxels) > self._cumulative_max_points:
                self._cumulative_voxels.pop(next(iter(self._cumulative_voxels)))
            if not self._cumulative_voxels:
                return
            header = copy.deepcopy(map_cloud.header)
            header.frame_id = FRAMES.odom
            points = list(self._cumulative_voxels.values())
            out = point_cloud2.create_cloud_xyz32(
                header,
                points,
            )
            self._cumulative_map_cloud_pub.publish(out)
            self._terrain_map_ext_pub.publish(
                self._terrain_cloud_from_xyz(
                    out,
                    max_body_range=self._terrain_map_ext_max_body_range,
                    exclude_near_field=self._terrain_ext_exclude_near_field,
                )
            )

        @staticmethod
        def _point_count(msg: PointCloud2) -> int:
            return int(msg.width) * int(msg.height)

        def _on_color(self, msg: Image) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = FRAMES.camera
            self._color_pub.publish(out)

        def _on_depth(self, msg: Image) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = FRAMES.camera
            self._depth_pub.publish(out)

        def _on_info(self, msg: CameraInfo) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = FRAMES.camera
            self._info_pub.publish(out)

    rclpy.init(args=None)
    node = GazeboRuntimeAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
