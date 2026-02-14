"""
算法 Profile: ICP 定位器 (Localizer)

将 localizer 原生话题 remap 到标准接口契约 (/nav/*):
  /localization_quality → /nav/localization_quality
  relocalize           → /nav/relocalize
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 参数 ----
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=EnvironmentVariable("NAV_MAP_PATH", default_value=""),
        description="地图路径 (不含扩展名)",
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

    # ---- Localizer 节点 ----
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
            # 输入: 从标准接口读取
            ("/cloud_registered", "/nav/registered_cloud"),
            ("/Odometry",         "/nav/odometry"),
            # 输出: remap 到标准接口
            ("/localization_quality", "/nav/localization_quality"),
            # 服务: remap 到标准名
            ("relocalize",         "/nav/relocalize"),
        ],
    )

    return LaunchDescription([
        map_path_arg, x_arg, y_arg, z_arg, yaw_arg,
        localizer_node,
    ])
