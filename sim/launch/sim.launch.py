"""LingTu MuJoCo full-stack simulation launch.

This launch file is a legacy ROS launch entry for native MuJoCo/PCT smoke
tests.  It now follows the canonical LingTu runtime contract:

- /slam/registered_cloud is the current body-frame scan.
- /slam/map_cloud is the odom/map-frame accumulated world cloud.
- Native terrain and local planner nodes consume /slam/map_cloud through their
  legacy /cloud_map input.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from runtime.runtime_interface import FRAMES, TOPICS


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="open_field",
        description="MuJoCo world from registry: factory | open_field | building | spiral | empty",
    )
    rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")

    world = LaunchConfiguration("world")
    sim_dir = REPO_ROOT / "sim"

    mujoco_sim = ExecuteProcess(
        cmd=[
            "python3",
            str(sim_dir / "engine" / "cli.py"),
            "--world",
            world,
            "--headless",
        ],
        name="mujoco_sim",
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "--frame-id",
            FRAMES.map,
            "--child-frame-id",
            FRAMES.odom,
        ],
    )

    terrain_analysis = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                name="terrain_analysis",
                remappings=[
                    ("/cloud_map", TOPICS.map_cloud),
                    ("/terrain_map", TOPICS.terrain_map),
                ],
                parameters=[
                    {
                        "sensor_range": 30.0,
                        "vehicle_x": 0.4,
                        "vehicle_y": 0.3,
                        "vehicle_z": 0.3,
                    }
                ],
                output="screen",
            )
        ],
    )

    local_planner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="local_planner",
                executable="localPlannerNodeExe",
                name="local_planner",
                remappings=[
                    ("/cloud_map", TOPICS.map_cloud),
                    ("/terrain_map", TOPICS.terrain_map),
                    ("/terrain_map_ext", TOPICS.terrain_map_ext),
                    ("/way_point", TOPICS.nav_way_point),
                    ("/cmd_vel", TOPICS.cmd_vel),
                ],
                parameters=[
                    {
                        "pathFollower": False,
                        "slopeWeight": 3.0,
                    }
                ],
                output="screen",
            )
        ],
    )

    path_follower = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="local_planner",
                executable="pathFollowerNodeExe",
                name="path_follower",
                remappings=[
                    ("/way_point", TOPICS.nav_way_point),
                    ("/cmd_vel", TOPICS.cmd_vel),
                ],
                parameters=[
                    {
                        "stuck_timeout": 10.0,
                        "stuck_dist_thre": 0.15,
                    }
                ],
                output="screen",
            )
        ],
    )

    pct_adapter = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="pct_adapters",
                executable="pct_path_adapter",
                name="pct_path_adapter",
                remappings=[
                    (TOPICS.global_path, TOPICS.global_path),
                    (TOPICS.nav_way_point, TOPICS.nav_way_point),
                ],
                output="screen",
            )
        ],
    )

    global_planner = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(sim_dir / "scripts" / "run_global_planner.py"),
                ],
                name="global_planner",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            rviz_arg,
            mujoco_sim,
            static_tf,
            terrain_analysis,
            local_planner,
            path_follower,
            pct_adapter,
            global_planner,
        ]
    )
