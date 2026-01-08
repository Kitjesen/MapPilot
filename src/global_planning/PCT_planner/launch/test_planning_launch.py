from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Test launch file for checking data flow between:
    PCT Planner -> Adapter -> Local Planner -> Path Follower
    WITHOUT sensors or hardware.
    """

    # 1. Arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='spiral0.3_2',
        description='Path to map file (filename only, searched in rsc/tomogram)'
    )
    map_path = LaunchConfiguration('map_path')

    # 2. Static TF (Simulate Robot at Origin)
    # map -> body (Robot Frame)
    # COMMENTED OUT: Use manual command to set start position
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'body']
    # )

    # 3. PCT Global Planner
    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"

    # Force LD_LIBRARY_PATH to include ROS libs AND ensure HOME is set
    ld_lib_path = f"/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH', '')}"
    home_dir = os.environ.get('HOME', '/home/sunrise')

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[{
            'map_file': map_path,
            'map_frame': 'map',
            'robot_frame': 'body', # Must match static TF
        }],
        env={
            'PYTHONPATH': python_path,
            'LD_LIBRARY_PATH': ld_lib_path,
            'HOME': home_dir
        }
    )

    # 4. PCT Adapter
    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter.py',
        name='pct_path_adapter',
        output='screen',
        parameters=[{
            'waypoint_distance': 0.5,
            'arrival_threshold': 0.5,
            'lookahead_dist': 1.0
        }]
    )

    # 5. Local Planner (Test Mode: No Obstacles, No Terrain)
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': path_folder,
            'vehicleLength': 0.6,
            'vehicleWidth': 0.6,
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': False,  # DISABLED for test
            'checkObstacle': False,       # DISABLED for test
            'checkRotObstacle': False,
            'adjacentRange': 3.5,
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'costHeightThre1': 0.15,
            'costHeightThre2': 0.1,
            'useCost': False,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': 0.5,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
        }],
        remappings=[
            ('/terrain_map', '/terrain_map_dummy') # Connect to nothing
        ]
    )

    # 6. Path Follower
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'realRobot': False,
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'pubSkipNum': 1,
            'twoWayDrive': True,
            'lookAheadDis': 0.5,
            'maxSpeed': 0.5,
            'maxAccel': 1.0, 
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
        }]
    )

    return LaunchDescription([
        map_path_arg,
        # static_tf_node, # Disabled
        pct_planner,
        pct_adapter,
        local_planner_node,
        path_follower_node
    ])

