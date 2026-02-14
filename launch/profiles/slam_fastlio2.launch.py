"""
算法 Profile: Fast-LIO2 SLAM

将 Fast-LIO2 的原生话题 remap 到标准接口契约 (/nav/*):

Fast-LIO2 内部使用的话题 (绝对名, 见 lio_node.cpp):
  publishers:  /cloud_registered, /cloud_map, /Odometry, /path
  subscribers: /imu/data, /lidar/scan (来自 lio.yaml 配置)
  service:     save_map (相对名 → /fastlio2/save_map)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "lio_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("fastlio2"), "config", "lio.yaml"]
        ),
        description="Fast-LIO2 配置文件路径",
    )

    lio_node = Node(
        package="fastlio2",
        executable="lio_node",
        name="lio_node",
        namespace="fastlio2",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("lio_config")}],
        remappings=[
            # ── 输出: 绝对话题 → 标准接口 ──
            ("/cloud_registered",  "/nav/registered_cloud"),
            ("/cloud_map",         "/nav/map_cloud"),
            ("/Odometry",          "/nav/odometry"),
            ("/path",              "/lio_path"),
            # ── 输入: 来自 lio.yaml 配置的话题名 → 标准接口 ──
            ("/imu/data",          "/nav/imu"),
            ("/lidar/scan",        "/nav/lidar_scan"),
            # ── 服务: 相对名, 通过 namespace 变为 /fastlio2/save_map → 标准接口 ──
            ("save_map",           "/nav/save_map"),
        ],
    )

    return LaunchDescription([config_arg, lio_node])
