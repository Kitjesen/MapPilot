"""Compatibility import for the ROS-backed simulation driver.

The implementation lives in :mod:`compat.ros2.sim_driver` so the simulation
driver package does not own ROS runtime code.
"""

from compat.ros2.sim_driver import (
    ROS2_SIM_CAMERA_FRAME_ID,
    ROS2_SIM_CMD_VEL_FRAME_ID,
    ROS2_SIM_GOAL_FRAME_ID,
    ROS2_SIM_LIVE_MAP_CLOUD_FRAME_ID,
    ROS2_SIM_ODOM_FRAME_ID,
    ROS2_SIM_REGISTERED_CLOUD_FRAME_ID,
    ROS2SimDriverModule,
    _parse_xyz_offsets,
)

__all__ = [
    "ROS2SimDriverModule",
    "ROS2_SIM_CAMERA_FRAME_ID",
    "ROS2_SIM_CMD_VEL_FRAME_ID",
    "ROS2_SIM_GOAL_FRAME_ID",
    "ROS2_SIM_LIVE_MAP_CLOUD_FRAME_ID",
    "ROS2_SIM_ODOM_FRAME_ID",
    "ROS2_SIM_REGISTERED_CLOUD_FRAME_ID",
    "_parse_xyz_offsets",
]
