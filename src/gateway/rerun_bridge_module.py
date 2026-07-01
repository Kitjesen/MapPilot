"""Compatibility import for the ROS-backed Rerun visualization bridge.

The implementation lives in :mod:`compat.ros2.rerun_bridge` so the gateway
package does not own ROS runtime code.
"""

from compat.ros2.rerun_bridge import RerunBridgeModule

__all__ = ["RerunBridgeModule"]
