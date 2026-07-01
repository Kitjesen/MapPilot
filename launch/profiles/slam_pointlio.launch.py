"""
算法 Profile: Point-LIO SLAM

基于 SMBU-PolarBear-Robotics-Team/point_lio (ROS2 Humble, iVox)
将 Point-LIO 的原生话题 remap 到标准接口契约 (/nav/*):

Point-LIO 内部使用的话题 (相对名, 通过 namespace 解析):
  publishers:  cloud_registered, cloud_registered_body, aft_mapped_to_init, path
  subscribers: livox/lidar (CustomMsg), livox/imu (来自 yaml 配置)
  TF:          camera_init → aft_mapped (可通过 tf_send_en 关闭)

与 Fast-LIO2 的差异:
  - 里程计话题名: aft_mapped_to_init (而非 Odometry)
  - 话题为相对名 (而非 Fast-LIO2 的绝对名)
  - Frame ID: 已 patch 为 odom/body (源码级修改)
  - 参数通过 ROS2 params 加载 (而非 yaml-cpp config_path)
  - save_map 服务已适配 (与 Fast-LIO2 接口一致)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "pointlio_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pointlio"), "config", "mid360.yaml"]
        ),
        description="Point-LIO 配置文件路径",
    )

    pointlio_node = Node(
        package="pointlio",
        executable="pointlio_node",
        name="pointlio_node",
        namespace="pointlio",
        output="screen",
        parameters=[LaunchConfiguration("pointlio_config")],
        remappings=[
            # ── 输出: 相对话题名 → 标准接口 ──
            # 机体坐标系点云 (body frame)
            ("cloud_registered_body", "/slam/registered_cloud"),
            # 世界坐标系点云 (world frame)
            ("cloud_registered",      "/slam/map_cloud"),
            # 里程计
            ("aft_mapped_to_init",    "/slam/odometry"),
            # 轨迹
            ("path",                  "/lio_path"),
            # ── 输入: yaml 中配置的话题名 → 标准接口 ──
            ("livox/imu",             "/imu/raw"),
            ("livox/lidar",           "/lidar/raw_frame"),
            # ── 服务: 相对名, 通过 namespace 变为 /pointlio/save_map → 标准接口 ──
            ("save_map",              "/slam/save_map"),
        ],
    )

    return LaunchDescription([config_arg, pointlio_node])
