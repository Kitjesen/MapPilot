"""Explicit legacy topic aliases used only by compatibility adapters."""

from dataclasses import dataclass

from message.topics import TOPICS


@dataclass(frozen=True)
class AdapterTopicAlias:
    """Legacy/native endpoint topic mapped into the LingTu runtime contract."""

    source: str
    target: str
    msg_format: str
    scope: str = "adapter_only"
    note: str = ""


ADAPTER_TOPIC_ALIASES = {
    "livox_driver": (
        AdapterTopicAlias(
            source="/livox/lidar",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Livox driver output normalized for LingTu SLAM input.",
        ),
        AdapterTopicAlias(
            source="/livox/imu",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Livox IMU output normalized for LingTu SLAM input.",
        ),
    ),
    "fastlio2": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
            note="Fast-LIO2 current scan in body frame.",
        ),
        AdapterTopicAlias(
            source="/cloud_map",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="Fast-LIO2 local/world map cloud.",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
            note="Fast-LIO2 odometry normalized to odom->body.",
        ),
        AdapterTopicAlias(
            source="/imu/data",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Canonical IMU input for Fast-LIO2 launch/service paths.",
        ),
        AdapterTopicAlias(
            source="/lidar/scan",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Canonical LiDAR input for Fast-LIO2 launch/service paths.",
        ),
    ),
    "tare": (
        AdapterTopicAlias(
            source="/registered_scan",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="TARE/CMU registered scan is a world/map cloud, not body scan.",
        ),
        AdapterTopicAlias(
            source="/terrain_map",
            target=TOPICS.terrain_map,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/terrain_map_ext",
            target=TOPICS.terrain_map_ext,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/state_estimation",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/state_estimation_at_scan",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/way_point",
            target=TOPICS.exploration_way_point,
            msg_format="geometry_msgs/msg/PointStamped",
        ),
    ),
    "terrain_analysis": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/map_clearing", target=TOPICS.map_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
    ),
    "terrain_analysis_ext": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/cloud_clearing", target=TOPICS.cloud_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
    ),
    "local_planner": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
        AdapterTopicAlias(
            source="/way_point",
            target=TOPICS.nav_way_point,
            msg_format="geometry_msgs/msg/PointStamped",
        ),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(
            source="/navigation_boundary",
            target=TOPICS.navigation_boundary,
            msg_format="geometry_msgs/msg/PolygonStamped",
        ),
        AdapterTopicAlias(
            source="/added_obstacles",
            target=TOPICS.added_obstacles,
            msg_format="sensor_msgs/msg/PointCloud2",
        ),
        AdapterTopicAlias(source="/check_obstacle", target=TOPICS.check_obstacle, msg_format="std_msgs/msg/Bool"),
    ),
    "path_follower": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(source="/cmd_vel", target=TOPICS.cmd_vel, msg_format="cmd_vel"),
        AdapterTopicAlias(source="/planner_status", target=TOPICS.planner_status, msg_format="std_msgs/msg/String"),
    ),
}


ADAPTER_RELAY_ALIASES = {}


def adapter_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return adapter-only legacy/native aliases for one endpoint surface."""

    try:
        return ADAPTER_TOPIC_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_TOPIC_ALIASES))
        raise ValueError(f"unknown adapter alias surface {surface!r}; available: {available}") from exc


def adapter_remappings(surface: str) -> dict[str, str]:
    """Return source->target remappings for a native launch/service surface."""

    return {alias.source: alias.target for alias in adapter_aliases(surface)}


def adapter_source_for_target(surface: str, target: str) -> str:
    """Return the native source topic or service remapped to one target."""

    for alias in adapter_aliases(surface):
        if alias.target == target:
            return alias.source
    available = ", ".join(alias.target for alias in adapter_aliases(surface))
    raise ValueError(f"surface {surface!r} has no adapter alias targeting {target!r}; available targets: {available}")


def adapter_relay_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return bidirectional relay aliases for simulation bridge surfaces."""

    try:
        return ADAPTER_RELAY_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_RELAY_ALIASES))
        raise ValueError(f"unknown adapter relay surface {surface!r}; available: {available}") from exc
