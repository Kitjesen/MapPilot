from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Unified launch file for 3D Navigation PCT Planner System
    INCLUDES:
    - FASTLIO2 (Localization + SLAM)
    - PCT Global Planner (Global Path)
    - PCT Adapter (Global -> Local Bridge)
    - Sensor Scan Generation (Dynamic Obstacle Clearing)
    - Terrain Analysis (Basic + Extended)
    - Local Planner (Trajectory Planning)
    - Path Follower (Path Tracking -> cmd_vel)
    - Robot Driver (Hardware Interface)
    """

    # --- FASTLIO2 Localization + Localizer ---
    # Detailed configuration with remappings to standard topics
    
    # Launch Arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2',
        description='Path to map file (without extension). E.g. /home/user/map. Layout: map.pcd and map.pickle'
    )
    
    map_path = LaunchConfiguration('map_path')
    
    # Initial Pose Arguments
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Initial Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw orientation')
    
    initial_x = LaunchConfiguration('x')
    initial_y = LaunchConfiguration('y')
    initial_z = LaunchConfiguration('z')
    initial_yaw = LaunchConfiguration('yaw')

    # Config Paths
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])
    
    localizer_config_path = PathJoinSubstitution([
        FindPackageShare("localizer"), "config", "localizer.yaml"
    ])
    
    # FASTLIO2 Node
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path}],
        remappings=[
            # Remap outputs to standard topic names (backward compatibility)
            ("body_cloud", "/cloud_registered"),  # Standard name for point cloud
            ("lio_odom", "/Odometry"),            # Standard name for odometry
            ("lio_path", "/lio_path"),            # Keep path as is
            ("world_cloud", "/world_cloud"),      # Keep world cloud as is
        ]
    )
    
    # Localizer Node
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path},
            {"static_map_path": [map_path, ".pcd"]}, # Auto-load map.pcd
            {"initial_x": initial_x},
            {"initial_y": initial_y},
            {"initial_z": initial_z},
            {"initial_yaw": initial_yaw}
        ],
        remappings=[
            # Input remappings - subscribe to standard names
            ("/fastlio2/body_cloud", "/cloud_registered"),
            ("/fastlio2/lio_odom", "/Odometry"),
        ]
    )
    
    # --- Livox Driver (MID360) ---
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        )
    )

    # --- PCT Global Planner ---
    # Generates the 3D path based on the Tomogram map
    
    # Calculate PYTHONPATH for PCT Planner dependencies
    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')
    
    # Construct PYTHONPATH
    # We need to include 'planner' dir (for config/libs) and 'scripts' (for utils)
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[{
            'map_file': map_path, # PCT planner handles extensions or base name
        }],
        env={'PYTHONPATH': python_path}
    )
    
    # --- PCT Adapter ---
    # Slices the global path into waypoints for the local planner
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

    # --- Perception Stack ---
    
    # [REMOVED] Sensor Scan Generation - Legacy node with no subscribers
    # If needed for debugging, run manually: ros2 run sensor_scan_generation sensorScanGeneration

    # Terrain Analysis (Basic): Analyzes traversability
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[{
            'vehicleHeight': 0.5, 
            'terrainVoxelSize': 0.2, 
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'checkCollision': True,
        }],
        remappings=[
        #    ('/state_estimation', '/Odometry'),
        #    ('/registered_scan', '/cloud_registered')
        ]
    )

    # Terrain Analysis (Extended): Advanced terrain mapping (Optional but good)
    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[{
            'scanVoxelSize': 0.05,
            'decayTime': 2.0,
            'noDecayDis': 4.0,
            'clearingDis': 8.0,
            'useSorting': True,
            'quantileZ': 0.25,
            'vehicleHeight': 0.5,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'lowerBoundZ': -1.5,
            'upperBoundZ': 1.0,
            'disRatioZ': 0.2,
            'checkTerrainConn': True,
            'terrainUnderVehicle': -0.2,
            'terrainConnThre': 0.5,
            'ceilingFilteringThre': 2.0,
            'localTerrainMapRadius': 4.0,
        }],
        remappings=[
        #    ('/state_estimation', '/Odometry'),     # Configured in C++
        #    ('/registered_scan', '/cloud_registered'), # Configured in C++
            ('/terrain_map', '/terrain_map')
        ]
    )

    # --- Local Planner Stack ---
    
    # Find pathFolder for local planner
    # Assuming share directory structure
    # If using 'symlink-install', it might point to source, but better to use share
    # We will pass the parameter dynamically if needed, or rely on default
    # Here we assume the default package share path
    
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    # Local Planner: Decision making & Obstacle Avoidance
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': path_folder,
            'vehicleLength': 0.6,
            'vehicleWidth': 0.6,
            'sensorOffsetX': 0.3, # Match robot config
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True, 
            'checkObstacle': True,
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
        #    ('/state_estimation', '/Odometry'),
        #    ('/registered_scan', '/cloud_registered'),
            ('/terrain_map', '/terrain_map')
        ]
    )

    # Path Follower: Generates Twist commands
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'realRobot': False, # Disable Serial, use Topic
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
        }],
        remappings=[
        #    ('/state_estimation', '/Odometry'),
            # Publishes /cmd_vel
        ]
    )
    
    # --- Hardware Driver ---
    
    # Generic Robot Driver: Topic -> Motors
    # robot_driver = Node(
    #    package='robot_driver',
    #    executable='driver_node.py',
    #    name='robot_driver',
    #    output='screen',
    #    parameters=[{
    #        'control_rate': 50.0,
    #        'robot_port': '/dev/ttyUSB0', 
    #        'baudrate': 115200
    #    }],
    #    remappings=[
    #        ('/wheel_odom', '/wheel_odom'),
    #        ('/cmd_vel', '/cmd_vel')
    #    ]
    # )

    # --- Joystick (Optional for manual control) ---
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device ID (0 -> /dev/input/js0, 1 -> /dev/input/js1)'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # --- Static Transforms ---
    # NOTE: In system_launch, 'localizer_node' publishes the map->lidar transform dynamically.
    # We DO NOT need a static transform here, otherwise it will conflict (TF Jitter).
    
    # --- Visualization ---
    # Optional: Visualization Tools
    # visualization_node = Node(
    #     package='visualization_tools',
    #     executable='visualizationTools',
    #     name='visualizationTools',
    #     output='screen',
    #     remappings=[
    #         ('/state_estimation', '/Odometry'),
    #         ('/registered_scan', '/cloud_registered')
    #     ]
    # )

    return LaunchDescription([
        map_path_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        livox_launch,
        lio_node,
        localizer_node,
        pct_planner,
        pct_adapter,
        # sensor_scan_node,  # REMOVED - no subscribers
        terrain_analysis_node,
        terrain_analysis_ext_node,
        local_planner_node,
        path_follower_node,
        # robot_driver,
        joy_node,
        # visualization_node
    ])
