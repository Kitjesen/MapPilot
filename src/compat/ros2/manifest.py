"""Manifest for ROS 2 compatibility boundaries.

Thunder product code should depend on Module ports and ``core.msgs`` contracts,
not on ROS packages directly. This manifest names the transitional files that
may still import ROS while bridge modules and endpoint adapters are migrated.
"""

from __future__ import annotations

from dataclasses import dataclass

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
    "global_planning/pct_planner/launch/",
    "global_planning/pct_planner/planner/lib/3rdparty/",
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
        "compat/ros2/bag_reader.py",
        "offline_rosbag_reader",
        "Offline bag ingestion is a tooling compatibility path.",
    ),
    RosCompatBoundary(
        "compat/ros2/camera_bridge.py",
        "hardware_camera_bridge",
        "Temporary camera topic bridge for the real Thunder driver.",
    ),
    RosCompatBoundary(
        "compat/ros2/camera_snapshot.py",
        "camera_snapshot_ros2_adapter",
        "Legacy one-shot compressed camera snapshot adapter for Gateway fallback.",
    ),
    RosCompatBoundary(
        "compat/ros2/context.py",
        "ros2_runtime_context",
        "Shared rclpy context used by transitional bridge modules.",
    ),
    RosCompatBoundary(
        "compat/ros2/map_save.py",
        "map_save_ros2_adapter",
        "Legacy PGO SaveMaps service adapter kept outside navigation logic.",
    ),
    RosCompatBoundary(
        "compat/ros2/mujoco_ros2_bridge.py",
        "simulation_endpoint_adapter",
        "Legacy MuJoCo ROS 2 bridge script kept outside the product runtime path.",
    ),
    RosCompatBoundary(
        "compat/ros2/mujoco_viz_bridge.py",
        "simulation_endpoint_adapter",
        "Legacy MuJoCo ROS visualization bridge script.",
    ),
    RosCompatBoundary(
        "compat/ros2/nav/grid_bridge.py",
        "nav_ros2_bridge",
        "Occupancy grid ROS bridge; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "compat/ros2/nav/path_bridge.py",
        "nav_ros2_bridge",
        "Path ROS bridge; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "compat/ros2/nav/waypoint_bridge.py",
        "nav_ros2_bridge",
        "Waypoint ROS bridge; should stay outside product logic.",
    ),
    RosCompatBoundary(
        "compat/ros2/nova_nav_bridge.py",
        "simulation_endpoint_adapter",
        "Legacy NOVA dog ROS simulation bridge script.",
    ),
    RosCompatBoundary(
        "compat/ros2/perception_publishers.py",
        "semantic_ros2_publisher_adapter",
        "Legacy perception visualization publishers.",
    ),
    RosCompatBoundary(
        "compat/ros2/relocalization_service.py",
        "relocalization_ros2_adapter",
        "Legacy relocalization service-call adapter kept outside SLAM facade.",
    ),
    RosCompatBoundary(
        "compat/ros2/rerun_bridge.py",
        "visualization_bridge",
        "Optional visualization bridge for ROS message streams.",
    ),
    RosCompatBoundary(
        "compat/ros2/sim_driver.py",
        "simulation_endpoint_adapter",
        "ROS 2 simulation endpoint adapter for Gazebo/MuJoCo topic bridges.",
    ),
    RosCompatBoundary(
        "compat/ros2/slam_bridge.py",
        "slam_ros2_bridge",
        "External SLAM service bridge into Module ports.",
    ),
    RosCompatBoundary(
        "compat/ros2/tare_bridge.py",
        "tare_ros2_bridge",
        "Bridge between TARE ROS outputs and LingTu Module ports.",
    ),
    RosCompatBoundary(
        "global_planning/pct_planner/",
        "legacy_ros_algorithm",
        "Vendored PCT ROS scripts kept outside the product runtime path.",
    ),
)


def ros_compat_boundary_for(source_relpath: str) -> RosCompatBoundary | None:
    """Return the explicit ROS boundary for a source path, if one exists."""

    for boundary in ROS_COMPAT_IMPORT_BOUNDARIES:
        if boundary.matches(source_relpath):
            return boundary
    return None
