"""
子系统: gRPC Gateway (远程监控 & 控制入口)

包含:
  - grpc_gateway (remote_monitoring 包)

对应 systemd 服务: nav-grpc.service
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    grpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("remote_monitoring"),
                "launch",
                "grpc_gateway.launch.py",
            )
        )
    )

    return LaunchDescription([grpc_launch])
