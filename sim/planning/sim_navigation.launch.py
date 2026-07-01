r"""
Building2_9 full-stack navigation simulation launch file for ROS 2 SITL.

No physical robot, LiDAR, or SLAM service is required. Node topology:

  sim_robot_node.py      -> /slam/odometry, /slam/map_cloud, /nav/terrain_map(+ext), /nav/stop
  global_planner.py      -> /nav/global_path      (ele_planner.so C++ planner)
  pct_path_adapter (C++) -> /nav/way_point         (waypoint sequence + /nav/adapter_status)
  localPlanner (C++)     -> /nav/local_path        (local obstacle-aware path)
  pathFollower (C++)     -> /nav/cmd_vel           (velocity command + /nav/planner_status)
  sim_robot_node.py      -> /nav/cmd_vel           (integrates motion and updates pose)

Usage:
  # Default Building2_9 corridor case.
  ros2 launch sim/planning/sim_navigation.launch.py

  # Specify map and goal.
  ros2 launch sim/planning/sim_navigation.launch.py \
    map_path:=/home/sunrise/data/SLAM/navigation/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/pcd/building2_9.pickle \
    goal_x:=5.0 goal_y:=-8.0

  # Alternative diagonal corridor case.
  ros2 launch sim/planning/sim_navigation.launch.py goal_y:=-8.0

Before launch, stop other navigation nodes:
  pkill -f 'pct_planner_astar|pct_path_adapter|localPlanner|pathFollower|sim_robot'
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# Default building2_9.pickle path. Prefer installed assets, then fall back to source tree.
def _default_pickle():
    try:
        share = get_package_share_directory('pct_planner')
        p = os.path.join(share, 'rsc', 'pcd', 'building2_9.pickle')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    # Fallback: derive the source-tree path from this launch file.
    planning_dir = os.path.dirname(__file__)        # sim/planning/
    src_root  = os.path.join(planning_dir, '..', '..', 'src')
    return os.path.normpath(os.path.join(
        src_root, 'nav', 'services', 'plan', 'global_planner',
        'backends', 'pct', 'vendor', 'pct_planner',
        'rsc', 'pcd', 'building2_9.pickle',
    ))


def generate_launch_description():
    pickle_default = _default_pickle()

    # Launch arguments.
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=pickle_default,
        description='Tomogram pickle path, for example building2_9 or factory_v4.',
    )
    # Default start and goal are on a passable Building2_9 corridor.
    # Keep goal_z aligned with tomogram_ground_h so planning uses the intended slice.
    # Keep the start inside valid free space so the global planner can snap safely.
    goal_x_arg = DeclareLaunchArgument('goal_x',   default_value='5.0',
                                        description='Goal X in meters.')
    goal_y_arg = DeclareLaunchArgument('goal_y',   default_value='7.3',
                                        description='Goal Y in meters.')
    goal_z_arg = DeclareLaunchArgument('goal_z',   default_value='1.0',
                                        description='Goal Z in meters; should match tomogram_ground_h for this map.')
    start_x_arg = DeclareLaunchArgument('start_x', default_value='-5.5',
                                         description='Start X in meters.')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='7.3',
                                         description='Start Y in meters.')
    # Map world-coordinate bounds. Override these for factory-scale scenes.
    map_x_min_arg = DeclareLaunchArgument('map_x_min', default_value='-7.5',  description='Minimum map X in meters.')
    map_x_max_arg = DeclareLaunchArgument('map_x_max', default_value='10.5',  description='Maximum map X in meters.')
    map_y_min_arg = DeclareLaunchArgument('map_y_min', default_value='-9.5',  description='Minimum map Y in meters.')
    map_y_max_arg = DeclareLaunchArgument('map_y_max', default_value='9.0',   description='Maximum map Y in meters.')
    # tomogram_ground_h: snap robot z below this value to the lowest valid slice.
    # Building2_9 usually uses 1.0; factory maps may use 1.5.
    tomo_gh_arg = DeclareLaunchArgument('tomogram_ground_h', default_value='1.0',
                                         description='tomogram_ground_h used for start-height snapping.')
    # Optional building PCD visualization path and scene name.
    pcd_path_arg   = DeclareLaunchArgument('pcd_path',   default_value='', description='Building PCD path for visualization.')
    scene_name_arg = DeclareLaunchArgument('scene_name', default_value='Building2_9', description='Scene name.')
    use_sim_robot_arg = DeclareLaunchArgument(
        'use_sim_robot',
        default_value='true',
        description='Start sim_robot_node. Set false when Gazebo supplies odometry/clouds.',
    )
    use_terrain_passthrough_arg = DeclareLaunchArgument(
        'use_terrain_passthrough',
        default_value='false',
        description='Copy /slam/map_cloud to terrain topics for Gazebo navigation gates.',
    )
    flatten_global_path_z_arg = DeclareLaunchArgument(
        'flatten_global_path_z',
        default_value='false',
        description='Force /nav/global_path z=0 for Gazebo 2D room validation.',
    )
    use_gazebo_line_planner_arg = DeclareLaunchArgument(
        'use_gazebo_line_planner',
        default_value='false',
        description='Use Gazebo odometry and goal_pose to publish a room-frame global path.',
    )
    gazebo_line_require_grid_arg = DeclareLaunchArgument(
        'gazebo_line_require_grid',
        default_value='false',
        description='Require /nav/exploration_grid before the Gazebo path publisher emits a path.',
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Start foxglove_bridge when the optional package is installed.',
    )

    map_path   = LaunchConfiguration('map_path')
    goal_x     = LaunchConfiguration('goal_x')
    goal_y     = LaunchConfiguration('goal_y')
    goal_z     = LaunchConfiguration('goal_z')
    start_x    = LaunchConfiguration('start_x')
    start_y    = LaunchConfiguration('start_y')
    map_x_min  = LaunchConfiguration('map_x_min')
    map_x_max  = LaunchConfiguration('map_x_max')
    map_y_min  = LaunchConfiguration('map_y_min')
    map_y_max  = LaunchConfiguration('map_y_max')
    pcd_path       = LaunchConfiguration('pcd_path')
    scene_name     = LaunchConfiguration('scene_name')
    tomogram_ground_h = LaunchConfiguration('tomogram_ground_h')
    use_sim_robot = LaunchConfiguration('use_sim_robot')
    use_terrain_passthrough = LaunchConfiguration('use_terrain_passthrough')
    flatten_global_path_z = LaunchConfiguration('flatten_global_path_z')
    use_gazebo_line_planner = LaunchConfiguration('use_gazebo_line_planner')
    gazebo_line_require_grid = LaunchConfiguration('gazebo_line_require_grid')
    use_foxglove = LaunchConfiguration('use_foxglove')

    # global_planner.py: real ele_planner.so C++ planner, same planner family as the RViz demo.
    repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
    # Keep Gazebo gates runnable from a source checkout even when pct_planner
    # has not been installed into the active ROS workspace.
    try:
        pct_share = get_package_share_directory('pct_planner')
    except Exception:
        pct_share = os.path.join(
            repo_root, 'src', 'nav', 'services', 'plan', 'global_planner',
            'backends', 'pct', 'vendor', 'pct_planner'
        )
    source_global_planner_script = os.path.join(
        repo_root,
        'src',
        'nav',
        'services',
        'plan',
        'global_planner',
        'backends',
        'pct',
        'vendor',
        'pct_planner',
        'planner',
        'scripts',
        'global_planner.py',
    )
    legacy_global_planner_script = os.path.join(
        repo_root,
        'src',
        'nav',
        'planning',
        'vendor',
        'pct_planner',
        'planner',
        'scripts',
        'legacy',
        'global_planner.py',
    )
    installed_global_planner_script = os.path.join(
        pct_share,
        'planner',
        'scripts',
        'global_planner.py',
    )
    global_planner_script = (
        source_global_planner_script
        if os.path.exists(source_global_planner_script)
        else legacy_global_planner_script
        if os.path.exists(legacy_global_planner_script)
        else installed_global_planner_script
    )

    # Prefer the historical numpy-1.x venv when present, but keep the Gazebo
    # launch runnable on clean ROS hosts that do not provide /tmp/venv_np1.
    _venv_python = '/tmp/venv_np1/bin/python3'
    _planner_python = _venv_python if os.path.exists(_venv_python) else sys.executable
    # Preserve the existing PYTHONPATH, including rclpy, so ROS 2 packages remain importable.
    _existing_pypath = os.environ.get('PYTHONPATH', '')

    global_planner_proc = ExecuteProcess(
        cmd=[
            _planner_python,
            global_planner_script,
            '--ros-args',
            # map_file points to the current tomogram pickle.
            '-p', ['map_file:=', map_path],
            # Snap start_h to a valid slice; Building2_9 usually uses 1.0, factory maps 1.5.
            '-p', ['tomogram_ground_h:=', tomogram_ground_h],
            '-p', ['flatten_path_z:=', flatten_global_path_z],
            # goal_pose remap: sim_robot_node publishes /nav/goal_pose.
            '-r', '/goal_pose:=/nav/goal_pose',
            # pct_path -> /nav/global_path: pct_adapter subscribes to /pct_path after remap.
            '-r', '/pct_path:=/nav/global_path',
        ],
        output='screen',
        condition=UnlessCondition(use_gazebo_line_planner),
        additional_env={'PYTHONPATH': _existing_pypath},
    )

    gazebo_line_planner_script = os.path.join(
        os.path.dirname(__file__), 'gazebo_line_global_planner.py'
    )
    gazebo_line_planner_proc = ExecuteProcess(
        cmd=[
            'python3',
            gazebo_line_planner_script,
            '--ros-args',
            '-p', ['require_occupancy_grid:=', gazebo_line_require_grid],
        ],
        output='screen',
        condition=IfCondition(use_gazebo_line_planner),
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    # pct_path_adapter: C++ waypoint adapter.
    # YAML is not installed into share here, so keep launch parameters inline.
    pct_adapter_node = Node(
        package='pct_adapters',
        executable='pct_path_adapter',
        name='pct_path_adapter',
        output='screen',
        parameters=[{
            # Waypoint downsampling interval in meters.
            'waypoint_distance':  1.5,
            # Arrival radius in meters; prevents skipping waypoints too aggressively.
            'arrival_threshold':  0.8,
            # Stuck-detection timeout in seconds; relaxed for simulation.
            'stuck_timeout_sec':  60.0,
            # Lookahead distance in meters.
            'lookahead_dist':     2.0,
        }],
        remappings=[
            ('/pct_path',         '/nav/global_path'),
            ('/Odometry',         '/slam/odometry'),
            ('/planner_waypoint', '/nav/way_point'),
        ],
    )

    # localPlanner: C++ local planner.
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder':            path_folder,
            # Robot geometry for the quadruped footprint.
            'vehicleLength':         0.65,
            'vehicleWidth':          0.30,
            'sensorOffsetX':         0.0,
            'sensorOffsetY':         0.0,
            'twoWayDrive':           False,
            # Terrain parameters.
            'laserVoxelSize':        0.05,
            'terrainVoxelSize':      0.2,
            'useTerrainAnalysis':    True,
            'checkObstacle':         True,
            'checkRotObstacle':      False,
            'adjacentRange':         3.5,
            'obstacleHeightThre':    0.2,
            'groundHeightThre':      0.1,
            'costHeightThre1':       0.15,
            'costHeightThre2':       0.1,
            'useCost':               False,
            'slopeWeight':           0.0,
            # Path scoring.
            'slowPathNumThre':       5,
            'slowGroupNumThre':      1,
            'pointPerPathThre':      2,
            'minRelZ':              -0.5,
            'maxRelZ':               0.25,
            'dirWeight':             0.02,
            'dirThre':               90.0,
            'dirToVehicle':          False,
            'pathScale':             1.0,
            'minPathScale':          0.75,
            'pathScaleStep':         0.25,
            # Velocity limits.
            'maxSpeed':              1.0,
            'pathScaleBySpeed':      True,
            'minPathRange':          1.0,
            'pathRangeStep':         0.5,
            'pathRangeBySpeed':      True,
            'pathCropByGoal':        True,
            # Autonomy mode.
            'autonomyMode':          True,
            'autonomySpeed':         0.8,
            'joyToSpeedDelay':       2.0,
            'joyToCheckObstacleDelay': 5.0,
            # Teleop/joystick input is absent in this simulation; keep defaults disabled.
            'joy_axis_fwd':          4,
            'joy_axis_left':         3,
            'joy_axis_autonomy':     2,
            'joy_axis_obstacle':     5,
            # Goal handling.
            'freezeAng':             90.0,
            'freezeTime':            2.0,
            'omniDirGoalThre':       1.0,
            'goalClearRange':        0.5,
            'goalBehindRange':       0.8,
            'goalX':                 ParameterValue(goal_x, value_type=float),
            'goalY':                 ParameterValue(goal_y, value_type=float),
        }],
        remappings=[
            ('/Odometry',    '/slam/odometry'),
            ('/cloud_map',   '/slam/map_cloud'),
            ('/terrain_map', '/nav/terrain_map'),
            ('/terrain_map_ext', '/nav/terrain_map_ext'),
            ('/way_point',   '/nav/way_point'),
            ('/speed',       '/nav/speed'),
            ('/path',        '/nav/local_path'),
            ('/stop',        '/nav/stop'),
            ('/slow_down',   '/nav/slow_down'),
            ('/navigation_boundary', '/nav/navigation_boundary'),
            ('/added_obstacles',     '/nav/added_obstacles'),
            ('/check_obstacle',      '/nav/check_obstacle'),
        ],
    )

    # pathFollower: C++ Pure Pursuit path follower.
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX':    0.0,
            'sensorOffsetY':    0.0,
            'pubSkipNum':       1,
            # Disable reverse driving to avoid forward/backward oscillation in corridor navigation.
            'twoWayDrive':      False,
            # Pure Pursuit parameters.
            'lookAheadDis':     0.5,
            'baseLookAheadDis': 0.3,
            'lookAheadRatio':   0.5,
            'minLookAheadDis':  0.2,
            'maxLookAheadDis':  2.0,
            # Control gains.
            'yawRateGain':      3.5,
            'stopYawRateGain':  3.5,
            'maxYawRate':       45.0,
            'maxSpeed':         1.0,
            'maxAccel':         0.5,
            # Switching and stop thresholds.
            'switchTimeThre':   1.0,
            # Relax direction-error threshold to reduce unnecessary direction switching.
            'dirDiffThre':      0.5,
            'omniDirGoalThre':  1.0,
            'omniDirDiffThre':  1.5,
            'stopDisThre':      0.2,
            'slowDwnDisThre':   1.0,
            # Slope and tilt protection; disabled in this flat simulation.
            'useInclRateToSlow': False,
            'inclRateThre':      120.0,
            'slowRate1':         0.25,
            'slowRate2':         0.5,
            'slowRate3':         0.75,
            'slowTime1':         2.0,
            'slowTime2':         2.0,
            'useInclToStop':     False,
            'inclThre':          45.0,
            'stopTime':          5.0,
            'noRotAtStop':       False,
            'noRotAtGoal':       True,
            # Autonomy mode.
            'autonomyMode':      True,
            'autonomySpeed':     0.8,
            'joyToSpeedDelay':   2.0,
            'joy_axis_fwd':      4,
            'joy_axis_left':     3,
            'joy_axis_yaw':      0,
            'joy_axis_autonomy': 2,
            'joy_axis_obstacle': 5,
        }],
        remappings=[
            ('/Odometry',  '/slam/odometry'),
            ('/path',      '/nav/local_path'),
            ('/speed',     '/nav/speed'),
            ('/stop',      '/nav/stop'),
            ('/slow_down', '/nav/slow_down'),
            ('/cmd_vel',   '/nav/cmd_vel'),
        ],
    )

    # sim_robot_node.py: standalone Python simulation robot node.
    # Use ExecuteProcess because this script is not launched as an installed colcon package.
    sim_script = os.path.join(os.path.dirname(__file__), 'sim_robot_node.py')

    sim_robot = ExecuteProcess(
        cmd=['python3', sim_script],
        output='screen',
        condition=IfCondition(use_sim_robot),
        # Pass launch arguments through environment variables; LaunchConfiguration supports substitution.
        additional_env={
            'SIM_GOAL_X':       goal_x,
            'SIM_GOAL_Y':       goal_y,
            'SIM_GOAL_Z':       goal_z,
            'SIM_START_X':      start_x,
            'SIM_START_Y':      start_y,
            'SIM_START_Z':      '0.0',  # Physical robot height; global_planner snaps to tomogram_ground_h.
            'SIM_MAP_X_MIN':    map_x_min,
            'SIM_MAP_X_MAX':    map_x_max,
            'SIM_MAP_Y_MIN':    map_y_min,
            'SIM_MAP_Y_MAX':    map_y_max,
            'SIM_PCD_PATH':     pcd_path,
            'SIM_SCENE_NAME':   scene_name,
            # Longer warmup ensures map->body TF exists before global_planner receives the goal.
            # Without TF, global_planner may plan from the fallback origin and fail.
            'SIM_WARMUP_S':     '10',
            'PYTHONUNBUFFERED': '1',
        },
    )

    # robot_state_publisher: URDF TF and wheel rotation.
    _urdf_path = os.path.join(os.path.dirname(__file__), 'simple_car.urdf')
    try:
        with open(_urdf_path) as _f:
            _robot_description = _f.read()
    except FileNotFoundError:
        _robot_description = ''

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': _robot_description}],
        condition=IfCondition(use_sim_robot),
    )

    terrain_passthrough_script = os.path.join(
        os.path.dirname(__file__), 'terrain_passthrough_node.py'
    )
    terrain_passthrough = ExecuteProcess(
        cmd=['python3', terrain_passthrough_script],
        output='screen',
        condition=IfCondition(use_terrain_passthrough),
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    # foxglove_bridge: optional web visualization at ws://robot_ip:8765.
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(use_foxglove),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'send_buffer_limit': 10000000,
        }],
    )

    # RViz2: local visualization on the display server.
    # Enable with USE_RVIZ=1; requires DISPLAY or WAYLAND_DISPLAY to be configured.
    _use_rviz = os.environ.get('USE_RVIZ', '0') == '1'
    _rviz_cfg = os.path.join(os.path.dirname(__file__), 'sim_nav.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', _rviz_cfg] if os.path.exists(_rviz_cfg) else [],
        parameters=[{'use_sim_time': True}],
    ) if _use_rviz else None

    # Clear transient-local path latch for the legacy latched PCT planner only.
    # The Gazebo line planner republishes live paths; a late empty latch can
    # reset pct_path_adapter after the real path is already accepted.
    clear_latch = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '--qos-durability', 'transient_local',
            '/nav/global_path', 'nav_msgs/msg/Path', '{}',
        ],
        output='screen',
        name='clear_path_latch',
        condition=UnlessCondition(use_gazebo_line_planner),
    )

    nodes = [
        clear_latch,            # Clear stale path latch first.
        sim_robot,              # Provides odometry, terrain, and /robot_description.
        terrain_passthrough,    # Gazebo-only: map_cloud -> terrain_map topics
        robot_state_pub_node,   # URDF TF from base_link to wheels.
        # Delay global planner startup so sim_robot_node can publish TF first.
        TimerAction(period=2.0, actions=[global_planner_proc]),
        gazebo_line_planner_proc,  # Gazebo gate: odom + goal_pose -> global_path
        pct_adapter_node,       # Waypoint adapter.
        local_planner_node,     # Local planner.
        path_follower_node,     # Velocity controller.
        foxglove_node,          # Optional web visualization fallback.
    ]
    if rviz_node is not None:
        nodes.append(rviz_node)

    return LaunchDescription([
        map_path_arg, goal_x_arg, goal_y_arg, goal_z_arg, start_x_arg, start_y_arg,
        map_x_min_arg, map_x_max_arg, map_y_min_arg, map_y_max_arg,
        tomo_gh_arg, pcd_path_arg, scene_name_arg,
        use_sim_robot_arg, use_terrain_passthrough_arg, flatten_global_path_z_arg,
        use_gazebo_line_planner_arg, gazebo_line_require_grid_arg,
        use_foxglove_arg,
        *nodes,
    ])
