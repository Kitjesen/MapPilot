"""Fast-LIO2 to LingTu navigation topic bridge.

This module owns the reusable ROS topic normalization boundary:

``/Odometry`` + ``/cloud_registered`` + ``/cloud_map``
    -> ``/slam/odometry`` + ``/slam/registered_cloud`` + ``/slam/map_cloud``

Simulation gates may observe the state exposed here, but they should not own
the runtime bridge logic.
"""

from __future__ import annotations

import copy
import math
from typing import Any, Callable

from runtime.runtime_interface import TOPICS, topic_default_frame_id


FASTLIO_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
FASTLIO_REGISTERED_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# Fast-LIO live mapping has no saved-map relocalization transform yet, so its
# world cloud stays in odom until a localizer owns map->odom.
FASTLIO_LIVE_MAP_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.odometry)


class FastLio2NavBridgeRuntime:
    """Normalize Fast-LIO2 live ROS topics into LingTu canonical nav topics."""

    def __init__(
        self,
        *,
        node: Any,
        slam_bridge: Any,
        odometry_cls: Any,
        pointcloud2_cls: Any,
        pointcloud_xy_stats: Callable[[Any], dict[str, Any]] | None = None,
        odom_xyz: Callable[[Any], list[float] | None] | None = None,
        odom_yaw: Callable[[Any], float | None] | None = None,
        on_nav_map_cloud: Callable[[Any], None] | None = None,
        module_driver: Any | None = None,
        use_fastlio_for_nav: bool = True,
        publish_tare_context_topics: bool = False,
    ) -> None:
        self.node = node
        self.slam_bridge = slam_bridge
        self.odometry_cls = odometry_cls
        self.pointcloud2_cls = pointcloud2_cls
        self.pointcloud_xy_stats = pointcloud_xy_stats
        self.odom_xyz = odom_xyz
        self.odom_yaw = odom_yaw
        self.on_nav_map_cloud = on_nav_map_cloud
        self.module_driver = module_driver
        self.use_fastlio_for_nav = bool(use_fastlio_for_nav)
        self.publish_tare_context_topics = bool(publish_tare_context_topics)

        self.odom_out: list[Any] = []
        self.registered_cloud_out: list[Any] = []
        self.map_cloud_out: list[Any] = []
        self.nav_odom_out: list[Any] = []
        self.nav_registered_cloud_out: list[Any] = []
        self.nav_map_cloud_out: list[Any] = []
        self.nav_map_cloud_area_samples: list[dict[str, Any]] = []

        self.first_odom_xyz: list[float] | None = None
        self.last_odom_xyz: list[float] | None = None
        self._prev_odom_xyz: list[float] | None = None
        self.odom_path_length_m = 0.0
        self.first_odom_yaw: float | None = None
        self.last_odom_yaw: float | None = None
        self.first_odom_stamp_s: float | None = None
        self.last_odom_stamp_s: float | None = None

        self.nav_odom_pub = node.create_publisher(
            odometry_cls, TOPICS.odometry, 100
        )
        self.nav_registered_cloud_pub = node.create_publisher(
            pointcloud2_cls, TOPICS.registered_cloud, 10
        )
        self.nav_map_cloud_pub = node.create_publisher(
            pointcloud2_cls, TOPICS.map_cloud, 10
        )
        self.nav_state_estimation_at_scan_pub = node.create_publisher(
            odometry_cls, TOPICS.state_estimation_at_scan, 10
        )
        self.nav_terrain_map_pub = node.create_publisher(
            pointcloud2_cls, TOPICS.terrain_map, 10
        )
        self.nav_terrain_map_ext_pub = node.create_publisher(
            pointcloud2_cls, TOPICS.terrain_map_ext, 10
        )

        node.create_subscription(odometry_cls, "/Odometry", self.on_odom, 100)
        node.create_subscription(
            pointcloud2_cls, "/cloud_registered", self.on_registered_cloud, 10
        )
        node.create_subscription(
            pointcloud2_cls, "/cloud_map", self.on_map_cloud, 10
        )

    @staticmethod
    def nav_frame_msg(msg: Any, frame_id: str) -> Any:
        cloned = copy.deepcopy(msg)
        header = getattr(cloned, "header", None)
        if header is None or not hasattr(header, "frame_id"):
            raise ValueError("Fast-LIO nav bridge message missing header.frame_id")
        header.frame_id = frame_id
        return cloned

    @staticmethod
    def stamp_s(msg: Any) -> float | None:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        try:
            return float(getattr(stamp, "sec")) + float(getattr(stamp, "nanosec")) * 1e-9
        except Exception:
            return None

    def publish_nav_odom(self, msg: Any) -> None:
        self.nav_odom_pub.publish(msg)
        if self.publish_tare_context_topics:
            self.nav_state_estimation_at_scan_pub.publish(msg)
        self.nav_odom_out.append(msg)
        driver = self.module_driver
        publisher = getattr(driver, "publish_odometry_from_ros", None)
        if callable(publisher):
            publisher(msg)

    def publish_nav_registered_cloud(self, msg: Any) -> None:
        self.nav_registered_cloud_pub.publish(msg)
        self.nav_registered_cloud_out.append(msg)
        driver = self.module_driver
        publisher = getattr(driver, "publish_registered_cloud_from_ros", None)
        if callable(publisher):
            publisher(msg)

    def publish_nav_map_cloud(self, msg: Any) -> None:
        self.nav_map_cloud_pub.publish(msg)
        if self.publish_tare_context_topics:
            self.nav_terrain_map_pub.publish(msg)
            self.nav_terrain_map_ext_pub.publish(msg)
        self.nav_map_cloud_out.append(msg)
        driver = self.module_driver
        publisher = getattr(driver, "publish_map_cloud_from_ros", None)
        if callable(publisher):
            publisher(msg)
        if self.pointcloud_xy_stats is not None:
            self.nav_map_cloud_area_samples.append(self.pointcloud_xy_stats(msg))
        if self.on_nav_map_cloud is not None:
            self.on_nav_map_cloud(msg)

    def on_odom(self, msg: Any) -> None:
        self.odom_out.append(msg)
        nav_msg = self.nav_frame_msg(msg, FASTLIO_ODOM_FRAME_ID)
        if self.use_fastlio_for_nav:
            self.publish_nav_odom(nav_msg)

        stamp_s = self.stamp_s(msg)
        if stamp_s is not None:
            if self.first_odom_stamp_s is None:
                self.first_odom_stamp_s = stamp_s
            self.last_odom_stamp_s = stamp_s

        xyz = self.odom_xyz(msg) if self.odom_xyz is not None else None
        if xyz is not None:
            if self.first_odom_xyz is None:
                self.first_odom_xyz = xyz
            if self._prev_odom_xyz is not None:
                self.odom_path_length_m += math.dist(self._prev_odom_xyz, xyz)
            self._prev_odom_xyz = xyz
            self.last_odom_xyz = xyz

        yaw = self.odom_yaw(msg) if self.odom_yaw is not None else None
        if yaw is not None:
            if self.first_odom_yaw is None:
                self.first_odom_yaw = yaw
            self.last_odom_yaw = yaw

        self.slam_bridge._on_rclpy_odom(nav_msg)
        worker = getattr(self.slam_bridge, "_odom_worker_thread", None)
        if worker is not None:
            worker.join(timeout=0.1)

    def on_registered_cloud(self, msg: Any) -> None:
        self.registered_cloud_out.append(msg)
        nav_msg = self.nav_frame_msg(msg, FASTLIO_REGISTERED_CLOUD_FRAME_ID)
        if self.use_fastlio_for_nav:
            self.publish_nav_registered_cloud(nav_msg)

    def on_map_cloud(self, msg: Any) -> None:
        self.map_cloud_out.append(msg)
        nav_msg = self.nav_frame_msg(msg, FASTLIO_LIVE_MAP_CLOUD_FRAME_ID)
        if self.use_fastlio_for_nav:
            self.publish_nav_map_cloud(nav_msg)
        self.slam_bridge._process_rclpy_cloud(nav_msg)

    @property
    def moved_m(self) -> float | None:
        if self.first_odom_xyz is None or self.last_odom_xyz is None:
            return None
        return math.dist(self.first_odom_xyz, self.last_odom_xyz)
