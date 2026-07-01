"""
算法 Profile: FAR Planner (可视图全局规划器)

CMU 自主导航栈的 FAR Planner，基于动态可视图的全局路径规划。
适合狭长环境（隧道/矿洞/走廊），支持实时动态障碍更新。

来源: https://github.com/jizhang-cmu/autonomy_stack_go2
论文: FAR Planner: Fast, Attemptable Route Planner (IROS 2022)

话题映射到标准接口 (/nav/*):
  /odom_world          ← /slam/odometry
  /terrain_cloud       ← /nav/terrain_map_ext
  /scan_cloud          ← /nav/terrain_map
  /terrain_local_cloud ← /slam/registered_cloud
  /goal_point          ← /nav/goal_point        (PointStamped 目标输入)
  /way_point           → /nav/way_point          (规划输出航点)
  /navigation_boundary → /nav/navigation_boundary
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg


def generate_launch_description():
    # ---- 参数 ----
    vgh_path_arg = DeclareLaunchArgument(
        "vgh_path",
        default_value="",
        description="预计算可视图文件路径 (.vgh)，留空则从零开始构建",
    )

    far_share = get_package_share_directory("far_planner")
    default_config = os.path.join(far_share, "config", "default.yaml")

    # 从 robot_config.yaml 读取 FAR 规划器参数覆盖
    _far = lambda k, d=None: robot_cfg('far_planner', k, d)

    # ---- FAR Planner 节点 ----
    far_planner = Node(
        package="far_planner",
        executable="far_planner",
        name="far_planner",
        output="screen",
        parameters=[
            default_config,
            {
                "far_planner.ros__parameters.sensor_range":
                    _far('sensor_range', 10.0),
                "far_planner.ros__parameters.robot_dim":
                    _far('robot_dim', 0.6),
                "far_planner.ros__parameters.vehicle_height":
                    _far('vehicle_height', 0.5),
                "far_planner.ros__parameters.main_run_freq":
                    _far('main_run_freq', 5.0),
                "far_planner.ros__parameters.is_static_env":
                    _far('is_static_env', False),
            }
        ],
        remappings=[
            # ── 输入: 标准接口 → FAR 内部话题 ──
            ("/odom_world",          "/slam/odometry"),
            ("/terrain_cloud",       "/nav/terrain_map_ext"),
            ("/scan_cloud",          "/nav/terrain_map"),
            ("/terrain_local_cloud", "/slam/registered_cloud"),
            ("/goal_point",          "/nav/goal_point"),
            # ── 输出: FAR 内部话题 → 标准接口 ──
            ("/way_point",              "/nav/way_point"),
            ("/navigation_boundary",    "/nav/navigation_boundary"),
            ("/far_reach_goal_status",  "/nav/far_reach_goal"),
        ],
    )

    # ---- Graph Decoder (可视图可视化/保存/加载) ----
    graph_decoder = Node(
        package="graph_decoder",
        executable="graph_decoder",
        name="graph_decoder",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("graph_decoder"),
                "config", "default.yaml"
            )
        ],
    )

    # ---- 目标转换节点: PoseStamped → PointStamped ----
    # FAR Planner 接收 PointStamped 目标，而我们的系统用 PoseStamped
    # 用一个轻量 Python 节点做转换
    goal_adapter = Node(
        package="far_planner",
        executable="goal_pose_to_point.py",
        name="far_goal_adapter",
        output="screen",
        remappings=[
            ("/goal_pose_in",  "/nav/goal_pose"),
            ("/goal_point_out", "/nav/goal_point"),
        ],
    )

    return LaunchDescription([
        vgh_path_arg,
        far_planner,
        graph_decoder,
        goal_adapter,
    ])
