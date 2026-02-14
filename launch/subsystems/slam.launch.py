"""
子系统: SLAM (里程计 + 建图)

通过 slam_profile 参数选择具体算法实现:
  fastlio2 (默认) — Fast-LIO2
  stub             — 无硬件测试桩

输出 (标准接口契约):
  /nav/odometry          — 里程计
  /nav/registered_cloud  — 机体坐标系点云
  /nav/map_cloud         — 世界坐标系点云
  /nav/save_map          — 保存地图服务

对应 systemd 服务: nav-slam.service
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_profile(context, *args, **kwargs):
    """根据 slam_profile 参数动态加载对应的 profile launch 文件"""
    profile = LaunchConfiguration("slam_profile").perform(context)
    profiles_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "profiles")
    profile_file = os.path.join(profiles_dir, f"slam_{profile}.launch.py")

    if not os.path.isfile(profile_file):
        raise RuntimeError(
            f"SLAM profile '{profile}' not found. "
            f"Expected file: {profile_file}\n"
            f"Available profiles: {[f for f in os.listdir(profiles_dir) if f.startswith('slam_')]}"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(profile_file)
        )
    ]


def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        "slam_profile",
        default_value="fastlio2",
        description="SLAM 算法 profile (fastlio2, stub, ...)",
    )

    return LaunchDescription([
        profile_arg,
        OpaqueFunction(function=_resolve_profile),
    ])
