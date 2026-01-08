from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#command to save the map
#    ros2 service call /pgo/save_map interface/srv/SaveMaps "{file_path: '/home/sunrise/my_map.pcd'}"


def generate_launch_description():
    """
    Launch file for Mapping Phase only.
    Use this to generate a map before running navigation.
    """

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

    # Config Paths
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])

    pgo_config_path = PathJoinSubstitution([
        FindPackageShare("pgo"), "config", "pgo.yaml"
    ])
    
    # PGO Node (Pose Graph Optimization Backend)
    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config_path}]
    )
    
    # FASTLIO2 Node (Mapping Mode)
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path}],
        remappings=[
            ("body_cloud", "/cloud_registered"),
            ("lio_odom", "/Odometry"),
        ]
    )

    # Rviz for visualization (Optional, add your own rviz config if needed)
    # rviz_node = Node( ...)

    # --- Static Transforms ---
    tf_map_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_lidar_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar']
    )

    return LaunchDescription([
        livox_launch,
        tf_map_lidar,
        lio_node,
        pgo_node
    ])
