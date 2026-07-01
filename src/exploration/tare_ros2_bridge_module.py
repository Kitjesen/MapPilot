"""Compatibility import for the ROS-backed TARE bridge.

The implementation lives in :mod:`compat.ros2.tare_bridge` so the exploration
package does not own ROS runtime code.
"""

from compat.ros2.tare_bridge import TAREROS2BridgeModule

__all__ = ["TAREROS2BridgeModule"]
