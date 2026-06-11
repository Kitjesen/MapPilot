"""LingTu Gazebo/GZ simulation entrypoint.

This launch file is a ROS-native scaffold. It keeps Gazebo optional: if
ros_gz_sim or ros_gz_bridge is not installed, launch generation still explains
the missing dependency instead of crashing while imported by tests or tooling.
"""

from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_WORLD = os.path.join(
    _REPO_ROOT,
    "sim",
    "worlds",
    "gazebo",
    "lingtu_gazebo_demo_room.sdf",
)
_DEFAULT_ROBOT_MODEL = os.path.join(
    _REPO_ROOT,
    "sim",
    "assets",
    "sdf",
    "thunder_gazebo_proxy.sdf",
)


def _adapter_env(repo_root: str) -> dict[str, str]:
    src_root = os.path.join(repo_root, "src")
    existing = os.environ.get("PYTHONPATH", "")
    paths = [repo_root, src_root]
    if existing:
        paths.append(existing)
    return {"PYTHONPATH": os.pathsep.join(paths)}


def _optional_gazebo_actions(context, *_args, **_kwargs):
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception as exc:
        return [LogInfo(msg=f"ament_index_python unavailable: {exc}")]

    world = LaunchConfiguration("world").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    use_bridge = LaunchConfiguration("use_bridge").perform(context).lower() == "true"
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    spawn_robot = LaunchConfiguration("spawn_robot").perform(context).lower() == "true"
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    repo_root = _REPO_ROOT
    for path in (repo_root, os.path.join(repo_root, "src")):
        if path not in sys.path:
            sys.path.insert(0, path)
    gz_args = f"-r -s {world}" if headless else f"-r {world}"

    try:
        get_package_share_directory("ros_gz_sim")
    except Exception as exc:
        return [
            LogInfo(
                msg=(
                    "ros_gz_sim is not installed; install Gazebo/GZ ROS 2 "
                    f"packages before running LingTu Gazebo simulation: {exc}"
                )
            )
        ]

    actions = [
        LogInfo(
            msg=(
                "Starting LingTu Gazebo simulation with frame contract: "
                "world -> map -> odom -> body, model base_link aliased to body"
            )
        ),
        ExecuteProcess(
            cmd=["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py", f"gz_args:={gz_args}"],
            name="lingtu_gazebo_world",
            output="screen",
        ),
    ]

    if spawn_robot:
        if not os.path.exists(robot_model):
            actions.append(LogInfo(msg=f"Gazebo robot_model not found: {robot_model}"))
        else:
            actions.append(
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2",
                                "run",
                                "ros_gz_sim",
                                "create",
                                "-name",
                                robot_name,
                                "-file",
                                robot_model,
                                "-x",
                                spawn_x,
                                "-y",
                                spawn_y,
                                "-z",
                                spawn_z,
                                "-Y",
                                spawn_yaw,
                            ],
                            name="lingtu_gazebo_spawn_robot",
                            output="screen",
                        )
                    ],
                )
            )

    if use_bridge:
        try:
            get_package_share_directory("ros_gz_bridge")
            from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig

            cfg = GazeboBridgeConfig(
                world_name=os.path.splitext(os.path.basename(world))[0] or "lingtu_world",
                robot_name=robot_name,
            )
            adapter_env = _adapter_env(repo_root)
            bridge_actions = [
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "ros_gz_bridge",
                        "parameter_bridge",
                        *cfg.ros_gz_bridge_specs(),
                        *cfg.ros_remap_args(),
                    ],
                    name="lingtu_gazebo_parameter_bridge",
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "python3",
                        "-m",
                        "sim.engine.bridge.gazebo_runtime_adapter",
                    ],
                    cwd=repo_root,
                    additional_env=adapter_env,
                    name="lingtu_gazebo_runtime_adapter",
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "python3",
                        "-m",
                        "sim.engine.bridge.gazebo_cmd_vel_adapter",
                    ],
                    cwd=repo_root,
                    additional_env=adapter_env,
                    name="lingtu_gazebo_cmd_vel_adapter",
                    output="screen",
                ),
            ]
            actions.append(
                TimerAction(
                    period=0.0,
                    actions=bridge_actions,
                )
            )
        except Exception as exc:
            actions.append(LogInfo(msg=f"ros_gz_bridge unavailable: {exc}"))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=_DEFAULT_WORLD,
                description="Gazebo/GZ world SDF path or resource name.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="thunder",
                description="Gazebo model name used to derive bridge topics.",
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value=_DEFAULT_ROBOT_MODEL,
                description="URDF or SDF model file passed to ros_gz_sim create.",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description="Spawn the robot model into the Gazebo world.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "use_bridge",
                default_value="true",
                description="Start ros_gz_bridge parameter_bridge for LingTu topics.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo server without GUI for CI and SSH servers.",
            ),
            OpaqueFunction(function=_optional_gazebo_actions),
        ]
    )
