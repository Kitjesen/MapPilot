"""
子系统: 机器人底盘驱动

模式:
  driver_mode=generic  — driver_node.py  (通用底盘, 建图模式)
  driver_mode=dog      — han_dog_bridge.py (四足机器人 gRPC 桥接, 运行模式)

建图模式由 navigation_bringup.launch.py 调用 (mode=generic)
运行模式由 navigation_run.launch.py 调用 (mode=dog)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import sys

# 加载集中化机器人参数 (config/robot_config.yaml)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg, robot_cfg_str


def generate_launch_description():
    # ---- 参数 (默认值来自 robot_config.yaml) ----
    mode_arg = DeclareLaunchArgument(
        "driver_mode",
        default_value="generic",
        description="驱动模式: generic (通用底盘) / dog (四足 gRPC 桥接)",
    )
    dog_host_arg = DeclareLaunchArgument(
        "dog_host", default_value=robot_cfg_str('driver', 'dog_host'),
        description="Han Dog CMS gRPC 地址"
    )
    dog_port_arg = DeclareLaunchArgument(
        "dog_port", default_value=robot_cfg_str('driver', 'dog_port'),
        description="Han Dog CMS gRPC 端口"
    )

    # ---- 通用底盘驱动 ----
    driver_node = Node(
        package="robot_driver",
        executable="driver_node.py",
        name="robot_driver",
        output="screen",
        parameters=[
            {"cmd_vel_timeout_ms": robot_cfg('safety', 'cmd_vel_timeout_ms')},
            {"control_rate": robot_cfg('driver', 'control_rate')},
        ],
        remappings=[
            ("/cmd_vel", "/nav/cmd_vel"),
        ],
        condition=LaunchConfigurationEquals("driver_mode", "generic"),
    )

    # ---- 四足 gRPC 桥接驱动 ----
    han_dog_bridge_node = Node(
        package="robot_driver",
        executable="han_dog_bridge.py",
        name="han_dog_bridge",
        output="screen",
        parameters=[
            {"dog_host": LaunchConfiguration("dog_host")},
            {"dog_port": LaunchConfiguration("dog_port")},
            {"max_linear_speed": robot_cfg('speed', 'max_linear')},
            {"max_angular_speed": robot_cfg('speed', 'max_angular')},
            {"cmd_vel_timeout_ms": robot_cfg('safety', 'cmd_vel_timeout_ms')},
            {"control_rate": robot_cfg('driver', 'control_rate')},
            {"auto_enable": robot_cfg('driver', 'auto_enable')},
            {"auto_standup": robot_cfg('driver', 'auto_standup')},
            {"reconnect_interval": robot_cfg('driver', 'reconnect_interval')},
        ],
        remappings=[
            ("/cmd_vel", "/nav/cmd_vel"),
            ("/stop",    "/nav/stop"),
            ("/Odometry", "/nav/odometry"),
        ],
        condition=LaunchConfigurationEquals("driver_mode", "dog"),
    )

    return LaunchDescription(
        [
            mode_arg,
            dog_host_arg,
            dog_port_arg,
            driver_node,
            han_dog_bridge_node,
        ]
    )
