"""Manifest for ROS 2 compatibility boundaries.

Thunder product code should depend on Module ports and ``runtime.msgs`` contracts,
not on ROS packages directly. This manifest names the transitional files that
may still import ROS while bridge modules and endpoint adapters are migrated.
"""

from __future__ import annotations

from dataclasses import dataclass

ROS_COMPAT_STATUS = "legacy_optional"
ROS_COMPAT_POLICY = (
    "ROS 2 adapters are explicit compatibility boundaries only. Product and "
    "portable runtime paths must communicate through Module ports and runtime.msgs "
    "contracts without importing ROS packages."
)

ROS_IMPORT_ROOTS: frozenset[str] = frozenset(
    {
        "builtin_interfaces",
        "geometry_msgs",
        "livox_ros_driver2",
        "nav_msgs",
        "pcl_msgs",
        "rclpy",
        "rosbag2_py",
        "rosgraph_msgs",
        "rosidl_runtime_py",
        "sensor_msgs",
        "std_msgs",
        "tf2_msgs",
        "tf2_ros",
        "visualization_msgs",
    }
)

ROS_SCAN_EXCLUDED_PREFIXES: tuple[str, ...] = (
    "drivers/real/camera/OrbbecSDK_ROS2/",
    "nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/",
)


@dataclass(frozen=True)
class RosCompatBoundary:
    """A source subtree that is allowed to import ROS during migration."""

    prefix: str
    category: str
    rationale: str

    def matches(self, source_relpath: str) -> bool:
        return source_relpath == self.prefix or source_relpath.startswith(self.prefix)


ROS_COMPAT_IMPORT_BOUNDARIES: tuple[RosCompatBoundary, ...] = (
    RosCompatBoundary(
        "drivers/adapters/ros2/livox_driver.py",
        "livox_driver_ros2_adapter",
        "Legacy official Livox ROS2 driver process adapter.",
    ),
    RosCompatBoundary(
        "localization/adapters/ros2/relocalization_service.py",
        "relocalization_ros2_adapter",
        "Legacy relocalization service-call adapter kept outside localization facade.",
    ),
    RosCompatBoundary(
        "localization/adapters/ros2/slam_bridge.py",
        "slam_ros2_bridge",
        "External localization service bridge into Module ports.",
    ),
    RosCompatBoundary(
        "nav/adapters/ros2/nav/map_out.py",
        "nav_ros2_bridge",
        "Occupancy grid ROS bridge; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "nav/adapters/ros2/nav/nav_in.py",
        "nav_ros2_bridge",
        "Navigation input ROS adapter; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "nav/adapters/ros2/nav/nav_out.py",
        "nav_ros2_bridge",
        "Navigation output ROS adapter; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "nav/adapters/ros2/tare_bridge.py",
        "tare_ros2_bridge",
        "Bridge between TARE ROS outputs and LingTu Module ports.",
    ),
    RosCompatBoundary(
        "perception/adapters/ros2/bag_reader.py",
        "offline_rosbag_reader",
        "Offline bag ingestion is a tooling compatibility path.",
    ),
    RosCompatBoundary(
        "perception/adapters/ros2/perception_publishers.py",
        "semantic_ros2_publisher_adapter",
        "Legacy perception visualization publishers.",
    ),
    RosCompatBoundary(
        "runtime/adapters/ros2/context.py",
        "ros2_runtime_context",
        "Shared rclpy context used by transitional bridge modules.",
    ),
    RosCompatBoundary(
        "runtime/adapters/ros2/map_save.py",
        "map_save_ros2_adapter",
        "Legacy PGO SaveMaps service adapter kept outside navigation logic.",
    ),
    RosCompatBoundary(
        "runtime/adapters/ros2/rerun_overlay.py",
        "rerun_visualization_ros2_adapter",
        "Optional Rerun visualization overlay for ROS message streams.",
    ),
)


def ros_compat_boundary_for(source_relpath: str) -> RosCompatBoundary | None:
    """Return the explicit ROS boundary for a source path, if one exists."""

    for boundary in ROS_COMPAT_IMPORT_BOUNDARIES:
        if boundary.matches(source_relpath):
            return boundary
    return None
