"""
导航系统统一启动 — 建图模式

启动内容:
  1. Livox LiDAR 驱动
  2. Fast-LIO2 (实时里程计)
  3. PGO (回环检测与优化)
  4. sensor_scan_generation (坐标转换)
  5. terrain_analysis + ext (地形分析, 建图时避障用)
  6. local_planner (局部规划, 手柄遥控 + 避障)
  7. robot_driver (底盘驱动)
  8. gRPC gateway (远程监控)

用法:
  ros2 launch navigation_bringup.launch.py
  ros2 launch navigation_bringup.launch.py maxSpeed:=1.0 autonomyMode:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 参数声明 ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed", default_value="0.875", description="最大速度 (m/s)"
    )
    autonomy_mode_arg = DeclareLaunchArgument(
        "autonomyMode", default_value="false", description="自主模式 (false=手柄控制)"
    )
    grpc_port_arg = DeclareLaunchArgument(
        "grpc_port", default_value="50051", description="gRPC 端口"
    )

    # ---- 配置文件路径 ----
    lio_config = os.path.join(
        get_package_share_directory("fastlio2"), "config", "lio.yaml"
    )
    pgo_config = os.path.join(
        get_package_share_directory("pgo"), "config", "pgo.yaml"
    )
    grpc_config = os.path.join(
        get_package_share_directory("remote_monitoring"), "config", "grpc_gateway.yaml"
    )

    # ---- 1. Livox LiDAR 驱动 ----
    livox_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            )
        )
    )

    # ---- 2. Fast-LIO2 ----
    lio_node = Node(
        package="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config}],
    )

    # ---- 3. PGO (回环优化) ----
    pgo_node = Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config}],
    )

    # ---- 4. Sensor scan generation ----
    sensor_scan_node = Node(
        package="sensor_scan_generation",
        executable="sensorScanGeneration",
        name="sensorScanGeneration",
        output="screen",
    )

    # ---- 5. Terrain analysis ----
    terrain_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis"),
                "launch",
                "terrain_analysis.launch",
            )
        )
    )

    terrain_ext_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis_ext"),
                "launch",
                "terrain_analysis_ext.launch",
            )
        )
    )

    # ---- 6. Local planner (手柄遥控 + 避障) ----
    local_planner_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("local_planner"),
                "launch",
                "local_planner.launch",
            )
        ),
        launch_arguments={
            "maxSpeed": LaunchConfiguration("maxSpeed"),
            "autonomyMode": LaunchConfiguration("autonomyMode"),
            "realRobot": "true",
        }.items(),
    )

    # ---- 7. Robot driver (底盘控制) ----
    robot_driver_node = Node(
        package="robot_driver",
        executable="driver_node.py",
        name="robot_driver",
        output="screen",
        parameters=[
            {"cmd_vel_timeout_ms": 200.0},
            {"control_rate": 50.0},
        ],
    )

    # ---- 8. gRPC gateway (远程监控) ----
    grpc_gateway_node = Node(
        package="remote_monitoring",
        executable="grpc_gateway",
        name="grpc_gateway",
        output="screen",
        parameters=[grpc_config],
    )

    return LaunchDescription(
        [
            # 参数
            max_speed_arg,
            autonomy_mode_arg,
            grpc_port_arg,
            # 节点 (按依赖顺序)
            livox_launch,
            lio_node,
            pgo_node,
            sensor_scan_node,
            terrain_launch,
            terrain_ext_launch,
            local_planner_launch,
            robot_driver_node,
            grpc_gateway_node,
        ]
    )
