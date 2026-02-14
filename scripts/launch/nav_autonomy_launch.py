"""
兼容薄壳 — 实际定义已迁移到 launch/subsystems/autonomy.launch.py

保留此文件是为了兼容可能引用此路径的外部工具。
新代码应直接使用:
  ros2 launch <workspace>/launch/subsystems/autonomy.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    subsystem = os.path.join(
        os.path.dirname(__file__), "..", "..", "launch", "subsystems", "autonomy.launch.py"
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(subsystem))]
    )
