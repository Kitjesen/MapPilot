"""
导航系统统一启动 — 建图模式

组合子系统:
  1. lidar     — LiDAR 驱动 (传感器标准接口)
  2. slam      — SLAM (可选 profile: fastlio2, stub)
  3. autonomy  — 地形分析 + 局部规划 (手柄遥控 + 避障)
  4. driver    — 底盘驱动 (通用模式)
  5. grpc      — gRPC Gateway (远程监控)

用法:
  ros2 launch navigation_bringup.launch.py
  ros2 launch navigation_bringup.launch.py maxSpeed:=1.0 autonomyMode:=false
  ros2 launch navigation_bringup.launch.py slam_profile:=stub  # 无硬件测试

注意:
  - PGO (回环优化) 按需手动启动: ros2 run pgo pgo_node
  - 建图模式使用 driver_node.py (通用底盘驱动)
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 加载集中化机器人参数 (config/robot_config.yaml)
sys.path.insert(0, os.path.dirname(__file__))
from _robot_config import robot_cfg_str


def generate_launch_description():
    subsystems_dir = os.path.join(os.path.dirname(__file__), "subsystems")

    # ---- CycloneDDS 配置 (如未手动设置则使用项目默认) ----
    cyclonedds_xml = os.path.join(
        os.path.dirname(__file__), "..", "config", "cyclonedds.xml")
    if os.path.isfile(cyclonedds_xml) and "CYCLONEDDS_URI" not in os.environ:
        os.environ["CYCLONEDDS_URI"] = "file://" + os.path.normpath(cyclonedds_xml)

    # ---- 参数声明 (传递给子系统, 默认值来自 robot_config.yaml) ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed", default_value=robot_cfg_str('speed', 'max_speed'),
        description="最大速度 (m/s)"
    )
    autonomy_mode_arg = DeclareLaunchArgument(
        "autonomyMode", default_value="false", description="自主模式 (false=手柄控制)"
    )
    slam_profile_arg = DeclareLaunchArgument(
        "slam_profile", default_value="fastlio2",
        description="SLAM 算法 profile (fastlio2, stub)",
    )

    # ---- 1. LiDAR 驱动 ----
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "lidar.launch.py")
        )
    )

    # ---- 2. SLAM (profile 可选) ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "slam.launch.py")
        ),
        launch_arguments={
            "slam_profile": LaunchConfiguration("slam_profile"),
        }.items(),
    )

    # ---- 3. 地形分析 + 局部规划 ----
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "autonomy.launch.py")
        ),
        launch_arguments={
            "maxSpeed": LaunchConfiguration("maxSpeed"),
            "autonomyMode": LaunchConfiguration("autonomyMode"),
        }.items(),
    )

    # ---- 4. 底盘驱动 (通用模式) ----
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "driver.launch.py")
        ),
        launch_arguments={"driver_mode": "generic"}.items(),
    )

    # ---- 5. gRPC Gateway ----
    grpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "grpc.launch.py")
        )
    )

    return LaunchDescription(
        [
            max_speed_arg,
            autonomy_mode_arg,
            slam_profile_arg,
            lidar,
            slam,
            autonomy,
            driver,
            grpc,
        ]
    )
