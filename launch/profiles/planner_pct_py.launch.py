"""
算法 Profile: Python A* PCT 规划器 + 路径适配器

适用场景:
  - x86_64 开发机 / CI (ARM64 .so 不可用)
  - 快速验证: 无需编译 C++ 规划器核心

与 planner_pct.launch.py 的区别:
  - 使用 pct_planner_astar.py 替代 global_planner.py (零 .so 依赖)
  - 规划算法: 2D A* on pre-built .pickle tomogram
  - pct_path_adapter (C++) 不变

话题映射 (与标准接口一致):
  输入:  /nav/goal_pose  → pct_planner_astar 直接订阅
  输出:  /nav/global_path ← pct_planner_astar 直接发布
         /nav/way_point   ← pct_path_adapter

Current Module-first entrypoint:
  python lingtu.py nav --planner astar --tomogram /path/to/building2_9.pickle

Legacy ROS-native smoke usage:
  launch this profile directly only from an isolated ROS 2 launch test.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg


def generate_launch_description():
    # ── 参数 ──────────────────────────────────────────────────────────────────
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=EnvironmentVariable('NAV_MAP_PATH', default_value=''),
        description='tomogram .pickle 文件路径',
    )
    map_path = LaunchConfiguration('map_path')

    pct_adapter_config = PathJoinSubstitution(
        [FindPackageShare('pct_adapters'), 'config', 'pct_path_adapter.yaml']
    )

    # ── Python A* 规划器 ───────────────────────────────────────────────────────
    pct_share   = get_package_share_directory('pct_planner')
    scripts_dir = os.path.join(pct_share, 'planner', 'scripts')

    pct_planner_py = Node(
        package='pct_planner',
        executable='pct_planner_astar.py',
        name='pct_planner_astar',
        output='screen',
        parameters=[
            {
                'tomogram_file':  map_path,
                'obstacle_thr':   robot_cfg('pct_planner', 'obstacle_thr',  49.9),
                'republish_hz':   robot_cfg('pct_planner', 'republish_hz',  1.0),
                'smooth_min_dist': robot_cfg('pct_planner', 'smooth_min_dist', 3),
            }
        ],
        additional_env={'PYTHONPATH': scripts_dir},
    )

    # ── PCT 路径适配器 (C++ 节点，不变) ────────────────────────────────────────
    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter',
        name='pct_path_adapter',
        output='screen',
        parameters=[pct_adapter_config],
        remappings=[
            # 输入: 从标准接口读取全局路径
            ('/pct_path',  '/nav/global_path'),
            ('/Odometry',  '/slam/odometry'),
            # 输出: 标准航点接口
            ('/planner_waypoint', '/nav/way_point'),
        ],
    )

    return LaunchDescription([map_path_arg, pct_planner_py, pct_adapter])
