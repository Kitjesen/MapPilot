"""
子系统: 地形分析 + 局部规划 + 路径跟踪

包含:
  - terrain_analysis      (基础地形分析)
  - terrain_analysis_ext  (扩展: 连通性 + 2.5D 高度图)
  - local_planner         (局部规划)
  - pathFollower          (Pure Pursuit 路径跟踪)

话题接口契约:
  输入: /nav/odometry, /nav/map_cloud, /nav/way_point, /nav/speed
  输出: /nav/terrain_map, /nav/terrain_map_ext, /nav/local_path,
        /nav/cmd_vel, /nav/stop, /nav/slow_down

注意: terrain_analysis 和 local_planner 源码内部订阅 /cloud_registered,
      通过 remap 对接到 /nav/map_cloud (世界坐标系点云)。

对应 systemd 服务: nav-autonomy.service
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

# 加载集中化机器人参数 (config/robot_config.yaml)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from _robot_config import robot_cfg, robot_cfg_str

# 从 robot_config.yaml 预加载参数
_geom = lambda k: robot_cfg('geometry', k)
_safety = lambda k: robot_cfg('safety', k)
_ctrl = lambda k: robot_cfg('control', k)


def generate_launch_description():
    # ---- 参数声明 (默认值来自 robot_config.yaml) ----
    max_speed_arg = DeclareLaunchArgument(
        "maxSpeed", default_value=robot_cfg_str('speed', 'max_speed'),
        description="最大速度 (m/s)"
    )
    autonomy_mode_arg = DeclareLaunchArgument(
        "autonomyMode", default_value="true", description="自主模式"
    )
    autonomy_speed_arg = DeclareLaunchArgument(
        "autonomySpeed", default_value=robot_cfg_str('speed', 'autonomy_speed'),
        description="自主导航速度 (m/s)"
    )

    local_planner_share = get_package_share_directory("local_planner")
    path_folder = os.path.join(local_planner_share, "paths")

    # ---- 地形分析 (基础) ----
    terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrainAnalysis",
        output="screen",
        parameters=[
            {
                "vehicleHeight": _geom('vehicle_height'),
                "terrainVoxelSize": 0.2,
                "obstacleHeightThre": _safety('obstacle_height_thre'),
                "groundHeightThre": _safety('ground_height_thre'),
                "checkCollision": True,
            }
        ],
        remappings=[
            # 源码内 /Odometry → 标准接口
            ("/Odometry",          "/nav/odometry"),
            # 源码内 /cloud_registered → 标准接口 (世界坐标系)
            ("/cloud_registered",  "/nav/map_cloud"),
            # 输出: 标准接口
            ("/terrain_map",       "/nav/terrain_map"),
        ],
    )

    # ---- 地形分析 (扩展) ----
    terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrainAnalysisExt",
        output="screen",
        parameters=[
            {
                "scanVoxelSize": 0.05,
                "decayTime": 2.0,
                "noDecayDis": 4.0,
                "clearingDis": 8.0,
                "useSorting": True,
                "quantileZ": 0.25,
                "vehicleHeight": _geom('vehicle_height'),
                "voxelPointUpdateThre": 100,
                "voxelTimeUpdateThre": 2.0,
                "lowerBoundZ": -1.5,
                "upperBoundZ": 1.0,
                "disRatioZ": 0.2,
                "checkTerrainConn": True,
                "terrainUnderVehicle": -0.2,
                "terrainConnThre": 0.5,
                "ceilingFilteringThre": 2.0,
                "localTerrainMapRadius": 4.0,
            }
        ],
        remappings=[
            ("/Odometry",          "/nav/odometry"),
            ("/cloud_registered",  "/nav/map_cloud"),
            ("/terrain_map",       "/nav/terrain_map"),
            ("/terrain_map_ext",   "/nav/terrain_map_ext"),
        ],
    )

    # ---- 局部规划器 ----
    local_planner_node = Node(
        package="local_planner",
        executable="localPlanner",
        name="localPlanner",
        output="screen",
        parameters=[
            {
                "pathFolder": path_folder,
                "vehicleLength": _geom('vehicle_length'),
                "vehicleWidth": _geom('vehicle_width'),
                "sensorOffsetX": _geom('sensor_offset_x'),
                "sensorOffsetY": _geom('sensor_offset_y'),
                "twoWayDrive": True,
                "laserVoxelSize": 0.05,
                "terrainVoxelSize": 0.2,
                "useTerrainAnalysis": True,
                "checkObstacle": True,
                "checkRotObstacle": False,
                "adjacentRange": 3.5,
                "obstacleHeightThre": _safety('obstacle_height_thre'),
                "groundHeightThre": _safety('ground_height_thre'),
                "costHeightThre1": 0.15,
                "costHeightThre2": 0.1,
                "useCost": False,
                "slowPathNumThre": 5,
                "slowGroupNumThre": 1,
                "pointPerPathThre": 2,
                "minRelZ": -0.5,
                "maxRelZ": 0.25,
                "dirWeight": 0.02,
                "dirThre": 90.0,
                "dirToVehicle": False,
                "pathScale": 1.0,
                "minPathScale": 0.75,
                "pathScaleStep": 0.25,
                "maxSpeed": LaunchConfiguration("maxSpeed"),
                "pathScaleBySpeed": True,
                "minPathRange": 1.0,
                "pathRangeStep": 0.5,
                "pathRangeBySpeed": True,
                "pathCropByGoal": True,
                "autonomyMode": LaunchConfiguration("autonomyMode"),
                "autonomySpeed": LaunchConfiguration("autonomySpeed"),
                "joyToSpeedDelay": 2.0,
                "joyToCheckObstacleDelay": 5.0,
                "freezeAng": 90.0,
                "freezeTime": 2.0,
                "omniDirGoalThre": 1.0,
                "goalClearRange": 0.5,
                "goalBehindRange": 0.8,
                "goalX": 0.0,
                "goalY": 0.0,
            }
        ],
        remappings=[
            ("/Odometry",          "/nav/odometry"),
            ("/cloud_registered",  "/nav/map_cloud"),
            ("/terrain_map",       "/nav/terrain_map"),
            ("/terrain_map_ext",   "/nav/terrain_map_ext"),
            ("/way_point",         "/nav/way_point"),
            ("/speed",             "/nav/speed"),
            ("/path",              "/nav/local_path"),
            ("/stop",              "/nav/stop"),
            ("/slow_down",         "/nav/slow_down"),
        ],
    )

    # ---- 路径跟踪器 (Pure Pursuit) ----
    path_follower_node = Node(
        package="local_planner",
        executable="pathFollower",
        name="pathFollower",
        output="screen",
        parameters=[
            {
                "sensorOffsetX": 0.0,  # pathFollower 的 sensor offset 通常为 0
                "sensorOffsetY": 0.0,
                "pubSkipNum": 1,
                "twoWayDrive": True,
                "lookAheadDis": 0.5,
                "baseLookAheadDis": 0.3,
                "lookAheadRatio": 0.5,
                "minLookAheadDis": 0.2,
                "maxLookAheadDis": 2.0,
                "yawRateGain": _ctrl('yaw_rate_gain'),
                "stopYawRateGain": _ctrl('stop_yaw_rate_gain'),
                "maxYawRate": _ctrl('max_yaw_rate'),
                "maxSpeed": LaunchConfiguration("maxSpeed"),
                "maxAccel": _ctrl('max_accel'),
                "switchTimeThre": 1.0,
                "dirDiffThre": 0.1,
                "omniDirGoalThre": 1.0,
                "omniDirDiffThre": 1.5,
                "stopDisThre": 0.2,
                "slowDwnDisThre": 1.0,
                "useInclRateToSlow": False,
                "inclRateThre": 120.0,
                "slowRate1": 0.25,
                "slowRate2": 0.5,
                "slowRate3": 0.75,
                "slowTime1": 2.0,
                "slowTime2": 2.0,
                "useInclToStop": False,
                "inclThre": 45.0,
                "stopTime": 5.0,
                "noRotAtStop": False,
                "noRotAtGoal": True,
                "autonomyMode": LaunchConfiguration("autonomyMode"),
                "autonomySpeed": LaunchConfiguration("autonomySpeed"),
                "joyToSpeedDelay": 2.0,
            }
        ],
        remappings=[
            ("/Odometry",   "/nav/odometry"),
            ("/path",       "/nav/local_path"),
            ("/speed",      "/nav/speed"),
            ("/stop",       "/nav/stop"),
            ("/slow_down",  "/nav/slow_down"),
            ("/cmd_vel",    "/nav/cmd_vel"),
        ],
    )

    return LaunchDescription(
        [
            max_speed_arg,
            autonomy_mode_arg,
            autonomy_speed_arg,
            terrain_analysis_node,
            terrain_analysis_ext_node,
            local_planner_node,
            path_follower_node,
        ]
    )
