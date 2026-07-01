"""Compatibility import for the ROS 2 waypoint bridge.

The implementation lives in :mod:`compat.ros2.nav.waypoint_bridge` so the
navigation package does not own ROS runtime code.
"""

from compat.ros2.nav.waypoint_bridge import ROS2WaypointBridgeModule

__all__ = ["ROS2WaypointBridgeModule"]
