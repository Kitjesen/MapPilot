"""
算法 Profile: SLAM Stub (无硬件测试用)

发布虚拟里程计到标准接口, 用于在没有 LiDAR 的环境下测试 gRPC Gateway 等上层模块。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 使用 ROS2 内置的 static_transform_publisher 提供 odom → body TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="stub_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "body"],
    )

    return LaunchDescription([static_tf])
