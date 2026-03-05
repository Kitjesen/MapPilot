"""
MapPilot Simulation Full-Stack Launch

启动内容：
  1. MuJoCo simulation node（物理 + LiDAR + bridge）
  2. terrain_analysis（点云 → terrain_map）
  3. local_planner
  4. pathFollower
  5. pct_path_adapter
  6. global_planner.py（PCT，加载 spiral0.3_2.pickle）

用法：
  ros2 launch sim/launch/sim.launch.py
  ros2 launch sim/launch/sim.launch.py world:=open_field
  ros2 launch sim/launch/sim.launch.py use_rviz:=true
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── 参数 ─────────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument('world', default_value='open_field',
                                      description='MuJoCo world: open_field | spiral_terrain')
    rviz_arg  = DeclareLaunchArgument('use_rviz', default_value='false')

    world   = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

    sim_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # ── 1. MuJoCo 仿真节点 ───────────────────────────────────────────────────
    mujoco_sim = ExecuteProcess(
        cmd=['python3', os.path.join(sim_dir, 'scripts', 'run_sim.py'),
             '--world', world],
        name='mujoco_sim',
        output='screen',
    )

    # ── 2. Static TF: map → odom（仿真里相同）───────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
    )

    # ── 3. terrain_analysis（延迟 3s 等 MuJoCo 发布点云）────────────────────
    # terrain_analysis 从 install 目录加载（已 colcon build）
    terrain_analysis = TimerAction(
        period=3.0,
        actions=[Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrain_analysis',
            remappings=[
                ('/cloud_map',    '/livox/lidar'),     # LiDAR 点云 → terrain
                ('/terrain_map',  '/nav/terrain_map'),
            ],
            parameters=[{
                'sensor_range': 30.0,
                'vehicle_x':    0.4,
                'vehicle_y':    0.3,
                'vehicle_z':    0.3,
            }],
            output='screen',
        )]
    )

    # ── 4. local_planner（延迟 4s）──────────────────────────────────────────
    local_planner = TimerAction(
        period=4.0,
        actions=[Node(
            package='local_planner',
            executable='localPlannerNodeExe',
            name='local_planner',
            remappings=[
                ('/cloud_map',   '/livox/lidar'),
                ('/terrain_map', '/nav/terrain_map'),
                ('/way_point',   '/nav/way_point'),
                ('/cmd_vel',     '/nav/cmd_vel'),
            ],
            parameters=[{
                'pathFollower': False,   # local_planner 不做跟踪，只出路径
                'slopeWeight':  3.0,
            }],
            output='screen',
        )]
    )

    # ── 5. pathFollower（延迟 4s）────────────────────────────────────────────
    path_follower = TimerAction(
        period=4.0,
        actions=[Node(
            package='local_planner',
            executable='pathFollowerNodeExe',
            name='path_follower',
            remappings=[
                ('/way_point', '/nav/way_point'),
                ('/cmd_vel',   '/nav/cmd_vel'),
            ],
            parameters=[{
                'stuck_timeout':    10.0,
                'stuck_dist_thre':  0.15,
            }],
            output='screen',
        )]
    )

    # ── 6. pct_path_adapter（延迟 5s）────────────────────────────────────────
    pct_adapter = TimerAction(
        period=5.0,
        actions=[Node(
            package='pct_adapters',
            executable='pct_path_adapter',
            name='pct_path_adapter',
            remappings=[
                ('/nav/global_path', '/nav/global_path'),
                ('/nav/way_point',   '/nav/way_point'),
            ],
            output='screen',
        )]
    )

    # ── 7. global_planner.py（PCT, 延迟 6s）──────────────────────────────────
    global_planner = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['python3',
                 os.path.join(sim_dir, '..', 'sim', 'scripts', 'run_global_planner.py')],
            name='global_planner',
            output='screen',
        )]
    )

    # ── 8. RViz2（可选）──────────────────────────────────────────────────────
    # rviz = Node(package='rviz2', executable='rviz2', ...)  # TODO

    return LaunchDescription([
        world_arg, rviz_arg,
        mujoco_sim,
        static_tf,
        terrain_analysis,
        local_planner,
        path_follower,
        pct_adapter,
        global_planner,
    ])
