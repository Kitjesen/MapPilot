"""Compatibility import for the ROS-backed SLAM bridge.

The implementation lives in :mod:`compat.ros2.slam_bridge` so the ``slam``
package does not own ROS runtime code.
"""

from compat.ros2.slam_bridge import *  # noqa: F401,F403
