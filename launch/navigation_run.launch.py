"""
导航系统统一启动 — 运行模式 (定位 + 规划 + 自主导航)

启动内容:
  1. Livox LiDAR 驱动
  2. Fast-LIO2 (实时里程计)
  3. Localizer (重定位模块, 加载预建地图)
  4. sensor_scan_generation (坐标转换)
  5. terrain_analysis + ext (地形分析)
  6. local_planner (局部规划 + 避障)
  7. pct_path_adapter (全局路径 → 航点适配)
  8. robot_driver (底盘驱动)
  9. gRPC gateway (远程监控)

注意: PCT 全局规划器 (Python) 需要单独启动:
  ros2 launch pct_planner planner_only_launch.py map_path:=Common

用法:
  ros2 launch navigation_run.launch.py
  ros2 launch navigation_run.launch.py maxSpeed:=1.0

启动后需要手动重定位:
  ros2 service call /relocalize interface/srv/Relocalize \\
    "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 参数声明 ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed", default_value="0.875", description="最大速度 (m/s)"
    )
    autonomy_speed_arg = DeclareLaunchArgument(
        "autonomySpeed", default_value="0.875", description="自主导航速度 (m/s)"
    )
    grpc_port_arg = DeclareLaunchArgument(
        "grpc_port", default_value="50051", description="gRPC 端口"
    )

    # ---- 配置文件路径 ----
    lio_config = os.path.join(
        get_package_share_directory("fastlio2"), "config", "lio.yaml"
    )
    localizer_config = os.path.join(
        get_package_share_directory("localizer"), "config", "localizer.yaml"
    )
    grpc_config = os.path.join(
        get_package_share_directory("remote_monitoring"), "config", "grpc_gateway.yaml"
    )
    pct_adapter_config = os.path.join(
        get_package_share_directory("pct_adapters"), "config", "pct_path_adapter.yaml"
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

    # ---- 3. Localizer (重定位模块) ----
    localizer_node = Node(
        package="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{"config_path": localizer_config}],
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

    # ---- 6. Local planner (自主导航模式) ----
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
            "autonomyMode": "true",
            "autonomySpeed": LaunchConfiguration("autonomySpeed"),
            "realRobot": "true",
        }.items(),
    )

    # ---- 7. PCT path adapter (全局→局部航点适配) ----
    pct_adapter_node = Node(
        package="pct_adapters",
        executable="pct_path_adapter",
        name="pct_path_adapter",
        output="screen",
        parameters=[pct_adapter_config],
    )

    # ---- 8. Robot driver (底盘控制) ----
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

    # ---- 9. gRPC gateway (远程监控) ----
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
            autonomy_speed_arg,
            grpc_port_arg,
            # 节点 (按依赖顺序)
            livox_launch,
            lio_node,
            localizer_node,
            sensor_scan_node,
            terrain_launch,
            terrain_ext_launch,
            local_planner_launch,
            pct_adapter_node,
            robot_driver_node,
            grpc_gateway_node,
        ]
    )
