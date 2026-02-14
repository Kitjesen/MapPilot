"""
子系统: 定位 + 全局规划

通过 profile 参数选择具体算法实现:
  planner_profile:   pct (默认) / stub
  localizer_profile: icp (默认)

输出 (标准接口契约):
  /nav/global_path          — 全局路径
  /nav/planner_status       — 规划器状态
  /nav/waypoint             — 适配后的局部航点
  /nav/localization_quality — 定位质量
  /nav/relocalize           — 重定位服务

参数:
  map_path  - 地图路径 (不含扩展名), 优先读取环境变量 NAV_MAP_PATH
  x, y, z, yaw - 初始位姿

对应 systemd 服务: nav-planning.service
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def _resolve_profiles(context, *args, **kwargs):
    """根据 profile 参数动态加载对应的 launch 文件"""
    profiles_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "profiles")

    planner_profile = LaunchConfiguration("planner_profile").perform(context)
    localizer_profile = LaunchConfiguration("localizer_profile").perform(context)

    planner_file = os.path.join(profiles_dir, f"planner_{planner_profile}.launch.py")
    localizer_file = os.path.join(profiles_dir, f"localizer_{localizer_profile}.launch.py")

    actions = []

    # Localizer
    if os.path.isfile(localizer_file):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localizer_file),
                launch_arguments={
                    "map_path": LaunchConfiguration("map_path").perform(context),
                    "x": LaunchConfiguration("x").perform(context),
                    "y": LaunchConfiguration("y").perform(context),
                    "z": LaunchConfiguration("z").perform(context),
                    "yaw": LaunchConfiguration("yaw").perform(context),
                }.items(),
            )
        )
    else:
        raise RuntimeError(
            f"Localizer profile '{localizer_profile}' not found: {localizer_file}"
        )

    # Planner
    if os.path.isfile(planner_file):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planner_file),
                launch_arguments={
                    "map_path": LaunchConfiguration("map_path").perform(context),
                }.items(),
            )
        )
    else:
        raise RuntimeError(
            f"Planner profile '{planner_profile}' not found: {planner_file}"
        )

    return actions


def generate_launch_description():
    # ---- 参数声明 ----
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=EnvironmentVariable("NAV_MAP_PATH", default_value=""),
        description="地图路径 (不含扩展名), 可通过环境变量 NAV_MAP_PATH 设置",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    planner_arg = DeclareLaunchArgument(
        "planner_profile",
        default_value="pct",
        description="全局规划器 profile (pct, stub, ...)",
    )
    localizer_arg = DeclareLaunchArgument(
        "localizer_profile",
        default_value="icp",
        description="定位器 profile (icp, ...)",
    )

    return LaunchDescription([
        map_path_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        planner_arg,
        localizer_arg,
        OpaqueFunction(function=_resolve_profiles),
    ])
