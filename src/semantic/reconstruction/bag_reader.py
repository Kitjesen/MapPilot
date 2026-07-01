"""Compatibility import for offline ROS bag reconstruction readers.

The implementation lives in :mod:`compat.ros2.bag_reader` so the semantic
reconstruction package does not own ROS runtime code.
"""

from compat.ros2.bag_reader import read_lidar_bag, read_rgb_d_bag

__all__ = ["read_rgb_d_bag", "read_lidar_bag"]
