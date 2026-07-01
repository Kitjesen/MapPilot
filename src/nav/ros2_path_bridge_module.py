"""Compatibility import for the ROS 2 path bridge.

The implementation lives in :mod:`compat.ros2.nav.path_bridge` so the
navigation package does not own ROS runtime code.
"""

from compat.ros2.nav.path_bridge import ROS2PathBridgeModule

__all__ = ["ROS2PathBridgeModule"]
