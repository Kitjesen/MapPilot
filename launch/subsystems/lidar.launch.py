"""
子系统: Livox MID360 LiDAR 驱动

输出 (标准接口契约):
  /nav/lidar_scan — LiDAR 点云 (CustomMsg)
  /nav/imu        — IMU 数据

对应 systemd 服务: nav-lidar.service
"""

import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 加载集中化机器人参数 (config/robot_config.yaml)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    config_path = os.path.join(livox_share, "config", "MID360_config.json")

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 1},      # 1=customized pointcloud format
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": robot_cfg('lidar', 'publish_freq')},
            {"output_data_type": 0},
            {"frame_id": robot_cfg('lidar', 'frame_id')},
            {"user_config_path": config_path},
            {"cmdline_input_bd_code": ""},
        ],
        remappings=[
            # 标准接口契约
            ("livox/lidar", "/nav/lidar_scan"),
            ("livox/imu",   "/nav/imu"),
        ],
    )

    return LaunchDescription([livox_driver])
