"""Compatibility import for legacy ROS perception publishers.

The implementation lives in :mod:`compat.ros2.perception_publishers` so the
semantic perception package does not own ROS runtime code.
"""

from compat.ros2.perception_publishers import PerceptionPublishersMixin

__all__ = ["PerceptionPublishersMixin"]
