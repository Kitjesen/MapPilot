"""
导航系统统一启动 — 运行模式 (定位 + 规划 + 自主导航)

组合子系统:
  1. lidar     — LiDAR 驱动 (传感器标准接口)
  2. slam      — SLAM (可选 profile: fastlio2, stub)
  3. planning  — 定位 + 全局规划 (可选 profile: pct/stub + icp)
  4. autonomy  — 地形分析 + 局部规划 + 路径跟踪 (自主模式)
  5. driver    — 四足机器人 gRPC 桥接驱动
  6. grpc      — gRPC Gateway (远程监控)

用法:
  ros2 launch navigation_run.launch.py
  ros2 launch navigation_run.launch.py maxSpeed:=1.0
  ros2 launch navigation_run.launch.py slam_profile:=stub planner_profile:=stub  # 无硬件
  ros2 launch navigation_run.launch.py dog_host:=192.168.4.100 dog_port:=13145
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

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

    # ---- 参数声明 (默认值来自 robot_config.yaml) ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed", default_value=robot_cfg_str('speed', 'max_speed'),
        description="最大速度 (m/s)"
    )
    autonomy_speed_arg = DeclareLaunchArgument(
        "autonomySpeed", default_value=robot_cfg_str('speed', 'autonomy_speed'),
        description="自主导航速度 (m/s)"
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=EnvironmentVariable("NAV_MAP_PATH", default_value=""),
        description="地图路径 (不含扩展名)",
    )
    dog_host_arg = DeclareLaunchArgument(
        "dog_host", default_value=robot_cfg_str('driver', 'dog_host'),
        description="Han Dog CMS gRPC 地址"
    )
    dog_port_arg = DeclareLaunchArgument(
        "dog_port", default_value=robot_cfg_str('driver', 'dog_port'),
        description="Han Dog CMS gRPC 端口"
    )

    # ---- 算法 Profile 参数 ----
    slam_profile_arg = DeclareLaunchArgument(
        "slam_profile", default_value="fastlio2",
        description="SLAM 算法 profile (fastlio2, stub)",
    )
    planner_profile_arg = DeclareLaunchArgument(
        "planner_profile", default_value="pct",
        description="全局规划器 profile (pct, stub)",
    )
    localizer_profile_arg = DeclareLaunchArgument(
        "localizer_profile", default_value="icp",
        description="定位器 profile (icp)",
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

    # ---- 3. 定位 + 全局规划 (profile 可选) ----
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "planning.launch.py")
        ),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "planner_profile": LaunchConfiguration("planner_profile"),
            "localizer_profile": LaunchConfiguration("localizer_profile"),
        }.items(),
    )

    # ---- 4. 地形分析 + 局部规划 (自主模式) ----
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "autonomy.launch.py")
        ),
        launch_arguments={
            "maxSpeed": LaunchConfiguration("maxSpeed"),
            "autonomyMode": "true",
            "autonomySpeed": LaunchConfiguration("autonomySpeed"),
        }.items(),
    )

    # ---- 5. 四足驱动 (dog 模式) ----
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "driver.launch.py")
        ),
        launch_arguments={
            "driver_mode": "dog",
            "dog_host": LaunchConfiguration("dog_host"),
            "dog_port": LaunchConfiguration("dog_port"),
        }.items(),
    )

    # ---- 6. gRPC Gateway ----
    grpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "grpc.launch.py")
        )
    )

    return LaunchDescription(
        [
            max_speed_arg,
            autonomy_speed_arg,
            map_path_arg,
            dog_host_arg,
            dog_port_arg,
            slam_profile_arg,
            planner_profile_arg,
            localizer_profile_arg,
            lidar,
            slam,
            planning,
            autonomy,
            driver,
            grpc,
        ]
    )
