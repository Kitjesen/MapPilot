"""
导航系统统一启动 — 探索模式 (Exploration Mode)

第三种运行模式: 无需预建地图，机器人在完全未知环境中边建图边自主导航。

与现有模式的区别:
  建图模式:  手柄遥控，边走边建图，无自主导航
  导航模式:  需要预建地图，ICP 定位 + PCT 全局规划
  探索模式:  无需预建地图，SLAM 实时位姿 + SCG 拓扑规划 + Frontier 探索

启动内容:
  1. lidar    — LiDAR 驱动
  2. slam     — SLAM 实时建图 (无需预建地图)
  3. autonomy — 地形分析 + 局部规划 (autonomyMode: true)
  4. driver   — 底盘驱动 (generic 或 dog)
  5. semantic — 语义感知 + 规划 (强制启用, 使用探索模式配置)
  6. grpc     — gRPC Gateway (可选)

不启动:
  - planning.launch.py (PCT Planner 需要预建地图)
  - localizer (SLAM 本身提供实时位姿)

用法:
  # 基础探索 (纯环境覆盖探索)
  ros2 launch navigation_explore.launch.py

  # 带语义目标的探索
  ros2 launch navigation_explore.launch.py target:="找到餐桌"

  # 四足机器人模式
  ros2 launch navigation_explore.launch.py driver_mode:=dog dog_host:=<狗板WiFi IP>

  # 仿真/无硬件测试
  ros2 launch navigation_explore.launch.py slam_profile:=stub
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from _robot_config import robot_cfg_str


def generate_launch_description():
    subsystems_dir = os.path.join(os.path.dirname(__file__), "subsystems")
    config_dir = os.path.join(os.path.dirname(__file__), "..", "config")

    # ---- CycloneDDS 配置 ----
    cyclonedds_xml = os.path.join(
        os.path.dirname(__file__), "..", "config", "cyclonedds.xml"
    )
    if os.path.isfile(cyclonedds_xml) and "CYCLONEDDS_URI" not in os.environ:
        os.environ["CYCLONEDDS_URI"] = "file://" + os.path.normpath(cyclonedds_xml)

    # ---- 参数声明 ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed",
        default_value=robot_cfg_str("speed", "max_speed"),
        description="最大速度 (m/s)",
    )
    slam_profile_arg = DeclareLaunchArgument(
        "slam_profile",
        default_value="fastlio2",
        description="SLAM 算法 profile (fastlio2, stub)",
    )
    driver_mode_arg = DeclareLaunchArgument(
        "driver_mode",
        default_value="generic",
        description="底盘驱动模式 (generic=通用遥控, dog=四足 gRPC 桥接)",
    )
    dog_host_arg = DeclareLaunchArgument(
        "dog_host",
        default_value=robot_cfg_str("dog_bridge", "host"),
        description="四足机器人 CMS IP 地址 (driver_mode=dog 时有效)",
    )
    dog_port_arg = DeclareLaunchArgument(
        "dog_port",
        default_value=robot_cfg_str("dog_bridge", "port"),
        description="四足机器人 CMS 端口 (driver_mode=dog 时有效)",
    )
    enable_grpc_arg = DeclareLaunchArgument(
        "enable_grpc",
        default_value="false",
        description="是否启动 gRPC Gateway (远程监控)",
    )

    # ---- 1. LiDAR 驱动 ----
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "lidar.launch.py")
        )
    )

    # ---- 2. SLAM 实时建图 (无需预建地图, 和建图模式相同) ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "slam.launch.py")
        ),
        launch_arguments={
            "slam_profile": LaunchConfiguration("slam_profile"),
        }.items(),
    )

    # ---- 3. 地形分析 + 局部规划 (自主模式) ----
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "autonomy.launch.py")
        ),
        launch_arguments={
            "maxSpeed": LaunchConfiguration("maxSpeed"),
            "autonomyMode": "true",  # 探索模式强制自主
        }.items(),
    )

    # ---- 4. 底盘驱动 ----
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "driver.launch.py")
        ),
        launch_arguments={
            "driver_mode": LaunchConfiguration("driver_mode"),
            "dog_host": LaunchConfiguration("dog_host"),
            "dog_port": LaunchConfiguration("dog_port"),
        }.items(),
    )

    # ---- 5. 语义子系统 (探索模式专用配置) ----
    # 使用 semantic_exploration.yaml 替代默认 semantic_planner.yaml
    # 该配置启用了: SCG auto_expand, GCM, frontier 探索策略, mock LLM
    semantic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "semantic.launch.py")
        ),
        launch_arguments={
            "perception_config": os.path.join(
                config_dir, "semantic_perception.yaml"
            ),
            "planner_config": os.path.join(
                config_dir, "semantic_exploration.yaml"
            ),
        }.items(),
    )

    # ---- 6. gRPC Gateway (可选) ----
    from launch.actions import GroupAction
    from launch.conditions import IfCondition

    grpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(subsystems_dir, "grpc.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_grpc")),
    )

    return LaunchDescription(
        [
            # 参数
            max_speed_arg,
            slam_profile_arg,
            driver_mode_arg,
            dog_host_arg,
            dog_port_arg,
            enable_grpc_arg,
            # 子系统 (按启动顺序)
            lidar,
            slam,
            autonomy,
            driver,
            semantic,
            grpc,
        ]
    )
