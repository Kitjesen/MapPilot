"""Optional ROS2 compatibility plugin catalog."""

from __future__ import annotations

from collections.abc import Mapping

ROS2_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "map_save_adapter": (
        "runtime.adapters.ros2.map_save",
    ),
    "map_ros2": (
        "nav.adapters.ros2.nav.map_out",
    ),
    "navigation_ros2": (
        "nav.adapters.ros2.nav.nav_in",
        "nav.adapters.ros2.nav.nav_out",
    ),
    "slam_ros2": (
        "localization.adapters.ros2.slam_bridge",
    ),
    "exploration_ros2": (
        "nav.adapters.ros2.tare_bridge",
    ),
    "visualization_ros2": (
        "gateway.visualization.rerun_bridge",
    ),
}
