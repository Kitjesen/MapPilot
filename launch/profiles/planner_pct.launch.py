"""
算法 Profile: PCT 全局规划器 + 路径适配器

将 PCT 原生话题 remap 到标准接口契约 (/nav/*):
  /pct_path          → /nav/global_path
  /pct_planner/status → /nav/planner_status
  /planner_waypoint  → /nav/way_point
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
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg


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

    # 从 robot_config.yaml 读取 PCT 规划器参数
    _pct = lambda k, d=None: robot_cfg('pct_planner', k, d)

    pct_planner = Node(
        package="pct_planner",
        executable="global_planner.py",
        name="pct_global_planner",
        output="screen",
        parameters=[
            {
                "map_file": map_path,
                "default_goal_height":    _pct('default_goal_height', 0.0),
                "tomogram_resolution":    _pct('tomogram_resolution', 0.2),
                "tomogram_slice_dh":      _pct('tomogram_slice_dh', 0.5),
                "tomogram_ground_h":      _pct('tomogram_ground_h', 0.0),
                "publish_map_pointcloud": _pct('publish_map_pointcloud', True),
                "publish_tomogram":       _pct('publish_tomogram', True),
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
            ("/Odometry",   "/slam/odometry"),
            # 输出: remap 到标准接口
            ("/planner_waypoint", "/nav/way_point"),   # 修正：与 autonomy.launch.py 订阅名一致
        ],
    )

    # ---- Mission Arc (任务生命周期状态机) ----
    mission_arc = Node(
        package="pct_adapters",
        executable="mission_arc.py",
        name="mission_arc",
        output="screen",
        parameters=[
            {
                "max_replan_count": 3,
                "recovery_timeout_sec": 8.0,
                "planning_timeout_sec": 30.0,
                "mission_timeout_sec": 600.0,
                "publish_hz": 2.0,
            }
        ],
    )

    return LaunchDescription([map_path_arg, pct_planner, pct_adapter, mission_arc])
