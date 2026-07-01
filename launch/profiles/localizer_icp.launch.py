"""ICP localizer launch profile.

Remaps native localizer topics/services into the LingTu /nav/* contract.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=EnvironmentVariable(
            "NAV_MAP_PATH", default_value="/home/sunrise/data/nova/maps/active/map"
        ),
        description="Map path without file extension",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    map_path = LaunchConfiguration("map_path")
    static_map_path_pcd = PythonExpression(['"', map_path, '" + ".pcd"'])

    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path},
            {"static_map_path": static_map_path_pcd},
            {"initial_x": LaunchConfiguration("x")},
            {"initial_y": LaunchConfiguration("y")},
            {"initial_z": LaunchConfiguration("z")},
            {"initial_yaw": LaunchConfiguration("yaw")},
        ],
        remappings=[
            # Inputs from Fast-LIO2 and the standard LingTu topic contract.
            ("/cloud_registered", "/slam/registered_cloud"),
            ("/Odometry", "/slam/odometry"),
            # The localizer map_cloud is the static saved map. Live SLAM clouds
            # stay on /slam/map_cloud for mapping/traversability consumers.
            ("map_cloud", "/slam/saved_map_cloud"),
            ("/localization_quality", "/slam/localization_quality"),
            # Services exposed through the standard LingTu topic contract.
            ("relocalize", "/slam/relocalize"),
            ("relocalize_check", "/slam/relocalize_check"),
            ("global_relocalize", "/slam/global_relocalize"),
            ("global_relocalize_status", "/slam/global_relocalize_status"),
        ],
    )

    return LaunchDescription([
        map_path_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        localizer_node,
    ])
