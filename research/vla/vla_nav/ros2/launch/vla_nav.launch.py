"""
VLA Navigation Launch File.

Launches the VLA navigation node with configuration from YAML.
Optionally remaps topics for different robot configurations.

Usage:
  ros2 launch vla_nav vla_nav.launch.py
  ros2 launch vla_nav vla_nav.launch.py model_quantized_path:=/path/to/quantized
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vla_nav")

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(pkg_share, "config", "vla_nav.yaml"),
        description="Path to VLA nav configuration YAML",
    )

    quantized_path_arg = DeclareLaunchArgument(
        "model_quantized_path",
        default_value="",
        description="Path to quantized model (empty = load full model)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    # VLA Nav node
    vla_nav_node = Node(
        package="vla_nav",
        executable="vla_nav_node",
        name="vla_nav_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"model.quantized_path": LaunchConfiguration("model_quantized_path")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            # Remap topics if needed for specific robot configurations
            # ("camera/color/image_raw", "/unitree/camera/front/color"),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        quantized_path_arg,
        use_sim_time_arg,
        vla_nav_node,
    ])
