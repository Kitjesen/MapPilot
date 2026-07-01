"""Compatibility import for the ROS 2 occupancy-grid bridge.

The implementation lives in :mod:`compat.ros2.nav.grid_bridge` so the
navigation package does not own ROS runtime code.
"""

from compat.ros2.nav.grid_bridge import ROS2GridBridgeModule

__all__ = ["ROS2GridBridgeModule"]
