"""
算法 Profile: PCT 全局规划器 + 路径适配器

将 PCT 原生话题 remap 到标准接口契约 (/nav/*):
  /pct_path          → /nav/global_path
  /pct_planner/status → /nav/planner_status
  /planner_waypoint  → /nav/waypoint
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---- 参数 ----
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=EnvironmentVariable("NAV_MAP_PATH", default_value=""),
        description="地图路径 (不含扩展名)",
    )
    map_path = LaunchConfiguration("map_path")

    pct_adapter_config_path = PathJoinSubstitution(
        [FindPackageShare("pct_adapters"), "config", "pct_path_adapter.yaml"]
    )

    # ---- PCT 全局规划器 ----
    pct_share = get_package_share_directory("pct_planner")
    planner_dir = os.path.join(pct_share, "planner")
    scripts_dir = os.path.join(planner_dir, "scripts")
    lib_dir = os.path.join(planner_dir, "lib")
    python_path = (
        f"{planner_dir}:{scripts_dir}:{lib_dir}:"
        f"{os.environ.get('PYTHONPATH', '')}"
    )

    pct_planner = Node(
        package="pct_planner",
        executable="global_planner.py",
        name="pct_global_planner",
        output="screen",
        parameters=[
            {
                "map_file": map_path,
                "tomogram_resolution": 0.2,
                "tomogram_slice_dh": 0.5,
                "tomogram_ground_h": 0.0,
            }
        ],
        additional_env={"PYTHONPATH": python_path},
        remappings=[
            # 输入: 标准接口
            ("/goal_pose",    "/nav/goal_pose"),
            # 输出: remap 到标准接口
            ("/pct_path",            "/nav/global_path"),
            ("/pct_planner/status",  "/nav/planner_status"),
        ],
    )

    # ---- PCT 路径适配器 (C++ 节点) ----
    pct_adapter = Node(
        package="pct_adapters",
        executable="pct_path_adapter",
        name="pct_path_adapter",
        output="screen",
        parameters=[pct_adapter_config_path],
        remappings=[
            # 输入: 从标准接口读取
            ("/pct_path",   "/nav/global_path"),
            ("/Odometry",   "/nav/odometry"),
            # 输出: remap 到标准接口
            ("/planner_waypoint", "/nav/waypoint"),
        ],
    )

    return LaunchDescription([map_path_arg, pct_planner, pct_adapter])
