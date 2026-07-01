"""Typed DDS topic registry for LingTu runtime streams."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import Any

from runtime.runtime_interface import TOPICS


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    type_name: str
    import_path: str
    idl_type: str
    cpp_type: str
    ros_compatible: bool = True

    def dds_type(self) -> Any:
        module_name, attr = self.import_path.rsplit(".", 1)
        return getattr(import_module(module_name), attr, None)


TOPIC_SPECS: dict[str, TopicSpec] = {
    # Native hardware wire. ROS2/Livox-compatible DDS is adapter-only and must
    # translate into this schema before entering the SLAM hot path.
    TOPICS.lidar_scan: TopicSpec(
        TOPICS.lidar_scan,
        "LivoxFrame",
        "message.dds_types.livox.LivoxFrame",
        "lingtu.dds.LivoxFrame",
        "lingtu::dds::LivoxFrame",
        ros_compatible=False,
    ),
    TOPICS.imu: TopicSpec(
        TOPICS.imu,
        "Imu",
        "message.dds_types.imu.Imu",
        "lingtu.dds.Imu",
        "lingtu::dds::Imu",
        ros_compatible=False,
    ),
    TOPICS.odometry: TopicSpec(
        TOPICS.odometry,
        "Odometry",
        "message.dds_types.nav.Odometry",
        "lingtu.dds.Odometry",
        "lingtu::dds::Odometry",
        ros_compatible=False,
    ),
    TOPICS.state_estimation_at_scan: TopicSpec(
        TOPICS.state_estimation_at_scan,
        "Odometry",
        "message.dds_types.nav.Odometry",
        "lingtu.dds.Odometry",
        "lingtu::dds::Odometry",
        ros_compatible=False,
    ),
    TOPICS.registered_cloud: TopicSpec(
        TOPICS.registered_cloud,
        "PointCloud2",
        "message.dds_types.pointcloud.PointCloud2",
        "lingtu.dds.PointCloud2",
        "lingtu::dds::PointCloud2",
        ros_compatible=False,
    ),
    TOPICS.map_cloud: TopicSpec(
        TOPICS.map_cloud,
        "PointCloud2",
        "message.dds_types.pointcloud.PointCloud2",
        "lingtu.dds.PointCloud2",
        "lingtu::dds::PointCloud2",
        ros_compatible=False,
    ),
    TOPICS.cumulative_map_cloud: TopicSpec(
        TOPICS.cumulative_map_cloud,
        "PointCloud2",
        "message.dds_types.pointcloud.PointCloud2",
        "lingtu.dds.PointCloud2",
        "lingtu::dds::PointCloud2",
        ros_compatible=False,
    ),
    TOPICS.saved_map_cloud: TopicSpec(
        TOPICS.saved_map_cloud,
        "PointCloud2",
        "message.dds_types.pointcloud.PointCloud2",
        "lingtu.dds.PointCloud2",
        "lingtu::dds::PointCloud2",
        ros_compatible=False,
    ),
    TOPICS.localization_quality: TopicSpec(
        TOPICS.localization_quality,
        "Float32",
        "message.dds_types.scalar.Float32",
        "lingtu.dds.Float32",
        "lingtu::dds::Float32",
        ros_compatible=False,
    ),
    TOPICS.localization_health: TopicSpec(
        TOPICS.localization_health,
        "Text",
        "message.dds_types.scalar.Text",
        "lingtu.dds.Text",
        "lingtu::dds::Text",
        ros_compatible=False,
    ),
    TOPICS.global_path: TopicSpec(
        TOPICS.global_path,
        "Path",
        "message.dds_types.nav.Path",
        "lingtu.dds.Path",
        "lingtu::dds::Path",
        ros_compatible=False,
    ),
    TOPICS.local_path: TopicSpec(
        TOPICS.local_path,
        "Path",
        "message.dds_types.nav.Path",
        "lingtu.dds.Path",
        "lingtu::dds::Path",
        ros_compatible=False,
    ),
    TOPICS.nav_way_point: TopicSpec(
        TOPICS.nav_way_point,
        "PoseStamped",
        "message.dds_types.geometry.PoseStamped",
        "lingtu.dds.PoseStamped",
        "lingtu::dds::PoseStamped",
        ros_compatible=False,
    ),
    TOPICS.goal_pose: TopicSpec(
        TOPICS.goal_pose,
        "PoseStamped",
        "message.dds_types.geometry.PoseStamped",
        "lingtu.dds.PoseStamped",
        "lingtu::dds::PoseStamped",
        ros_compatible=False,
    ),
    TOPICS.cancel: TopicSpec(
        TOPICS.cancel,
        "String",
        "message.dds_types.scalar.String",
        "lingtu.dds.String",
        "lingtu::dds::String",
        ros_compatible=False,
    ),
    TOPICS.semantic_instruction: TopicSpec(
        TOPICS.semantic_instruction,
        "String",
        "message.dds_types.scalar.String",
        "lingtu.dds.String",
        "lingtu::dds::String",
        ros_compatible=False,
    ),
    TOPICS.cmd_vel: TopicSpec(
        TOPICS.cmd_vel,
        "TwistStamped",
        "message.dds_types.geometry.TwistStamped",
        "lingtu.dds.TwistStamped",
        "lingtu::dds::TwistStamped",
        ros_compatible=False,
    ),
    TOPICS.exploration_grid: TopicSpec(
        TOPICS.exploration_grid,
        "OccupancyGrid",
        "message.dds_types.nav.OccupancyGrid",
        "lingtu.dds.OccupancyGrid",
        "lingtu::dds::OccupancyGrid",
        ros_compatible=False,
    ),
}


def topic_spec(topic: str) -> TopicSpec | None:
    return TOPIC_SPECS.get(str(topic))


def dds_type_for_topic(topic: str) -> Any | None:
    spec = topic_spec(topic)
    return spec.dds_type() if spec else None


def to_dds_message(topic: str, msg: Any) -> Any:
    """Convert a runtime message to the registered DDS payload for a topic."""

    topic = str(topic)
    if topic == TOPICS.raw_lidar_points:
        from message.dds_types.livox import livox_frame_to_msg

        return livox_frame_to_msg(msg)
    from runtime.adapters.dds.endpoint_service import _to_dds_message

    return _to_dds_message(topic, msg)


def from_dds_message(topic: str, msg: Any) -> Any:
    """Convert a registered DDS payload back to a runtime message."""

    topic = str(topic)
    if topic == TOPICS.raw_lidar_points:
        from message.dds_types.livox import livox_msg_to_frame

        return livox_msg_to_frame(msg)
    if topic == TOPICS.raw_imu:
        from message.dds_types.imu import dds_imu_to_imu

        return dds_imu_to_imu(msg)
    from runtime.adapters.dds.endpoint_service import _from_dds_message

    return _from_dds_message(topic, msg)


def dds_topic_name(topic: str, *, typed: bool) -> str:
    name = str(topic)
    if typed and name.startswith("/"):
        return "rt" + name
    return name
