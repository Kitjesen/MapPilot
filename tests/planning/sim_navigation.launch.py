"""
Building2_9 全栈导航仿真启动文件 (ROS2 SITL)

无需硬件 / LiDAR / SLAM. 节点拓扑:

  sim_robot_node.py      → /nav/odometry, /nav/map_cloud, /nav/terrain_map(+ext), /nav/stop
  pct_planner_astar.py   → /nav/global_path      (building2_9.pickle A* 规划)
  pct_path_adapter (C++) → /nav/way_point         (航点序列 + /nav/adapter_status)
  localPlanner     (C++) → /nav/local_path        (局部避障路径)
  pathFollower     (C++) → /nav/cmd_vel           (速度指令 + /nav/planner_status)
  sim_robot_node.py      ← /nav/cmd_vel           (积分运动学 → 更新位姿)

用法:
  # 基本用法 (Corridor_E: (-5.5,7.3) → (5.0,7.3))
  ros2 launch tests/planning/sim_navigation.launch.py

  # 指定地图和目标
  ros2 launch tests/planning/sim_navigation.launch.py \\
    map_path:=/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/building2_9.pickle \\
    goal_x:=5.0 goal_y:=-8.0

  # Corridor_SE 场景 (长对角线)
  ros2 launch tests/planning/sim_navigation.launch.py goal_y:=-8.0

注意: 启动前请停止其他导航节点:
  pkill -f 'pct_planner_astar\\|pct_path_adapter\\|localPlanner\\|pathFollower\\|sim_robot'
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# ── building2_9.pickle 默认路径 (安装目录优先, 回退到源码目录) ────────────────
def _default_pickle():
    try:
        share = get_package_share_directory('pct_planner')
        p = os.path.join(share, 'rsc', 'pcd', 'building2_9.pickle')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    # 回退: 从 launch 文件位置推算源码路径
    tests_dir = os.path.dirname(__file__)           # tests/planning/
    src_root  = os.path.join(tests_dir, '..', '..', 'src')
    return os.path.normpath(os.path.join(
        src_root, 'global_planning', 'PCT_planner',
        'rsc', 'pcd', 'building2_9.pickle',
    ))


def generate_launch_description():
    pickle_default = _default_pickle()

    # ── 启动参数声明 ──────────────────────────────────────────────────────────
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=pickle_default,
        description='番茄图 pickle 路径 (building2_9 或 factory_v4)',
    )
    # 起终点: 起点(-5.5,7.3) 恰在楼梯间 (Z=0.5→3.0m 全段可通行, trav=21.4)
    # 终点(2.0,-3.0) 在上层走廊 Z=2.0m (trav=0.0)
    # 3D A* 路径: 地面层出发 → 经楼梯间上至 Z=1.0m 走廊层 → 到达目标 Z=2.0m
    # Z=1.0m 走廊层: 4123格连通, 起终点均可达
    goal_x_arg = DeclareLaunchArgument('goal_x',   default_value='2.0',
                                        description='目标 X (m)')
    goal_y_arg = DeclareLaunchArgument('goal_y',   default_value='-3.0',
                                        description='目标 Y (m)')
    goal_z_arg = DeclareLaunchArgument('goal_z',   default_value='2.0',
                                        description='目标 Z (m)  (2.0=上层楼面, 3D楼梯间规划)')
    start_x_arg = DeclareLaunchArgument('start_x', default_value='-5.5',
                                         description='起点 X (m)  (楼梯间入口)')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='7.3',
                                         description='起点 Y (m)  (楼梯间入口)')
    # 地图世界坐标边界 (building2_9 默认值; 工厂场景需覆盖)
    map_x_min_arg = DeclareLaunchArgument('map_x_min', default_value='-7.5',  description='地图 X 最小值 (m)')
    map_x_max_arg = DeclareLaunchArgument('map_x_max', default_value='10.5',  description='地图 X 最大值 (m)')
    map_y_min_arg = DeclareLaunchArgument('map_y_min', default_value='-9.5',  description='地图 Y 最小值 (m)')
    map_y_max_arg = DeclareLaunchArgument('map_y_max', default_value='9.0',   description='地图 Y 最大值 (m)')
    # 可选: 建筑 PCD 可视化 + 场景名称
    pcd_path_arg   = DeclareLaunchArgument('pcd_path',   default_value='', description='建筑 PCD 路径 (可视化用)')
    scene_name_arg = DeclareLaunchArgument('scene_name', default_value='Building2_9', description='场景名称')

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
    pcd_path   = LaunchConfiguration('pcd_path')
    scene_name = LaunchConfiguration('scene_name')

    # ── pct_planner_astar.py (Python A* 全局规划器) ───────────────────────────
    pct_share   = get_package_share_directory('pct_planner')
    scripts_dir = os.path.join(pct_share, 'planner', 'scripts')

    # 必须把 scripts_dir 追加到现有 PYTHONPATH 前面 (不能替换, 否则 rclpy 不可见)
    _existing_pypath = os.environ.get('PYTHONPATH', '')
    _full_pypath = f"{scripts_dir}:{_existing_pypath}" if _existing_pypath else scripts_dir

    pct_planner_node = Node(
        package='pct_planner',
        executable='pct_planner_astar.py',
        name='pct_planner_astar',
        output='screen',
        parameters=[{
            'tomogram_file':   map_path,
            'obstacle_thr':    49.9,  # 正确值: 50.0格是墙壁边界, 不可通行; 3D A*经楼梯间绕过
            # 降低重发频率, 防止 pct_adapter 每秒重置航点索引 (10s 内机器人能走完一个航点段)
            'republish_hz':    0.02,  # 50s 间隔, 不干扰 25s 导航循环 (防止重发引起路径闪烁)
            # 增大航点间距 → 每段 ~1.6m, 全程 ~6-7 航点 (减少快速索引跳跃)
            'smooth_min_dist': 8,
        }],
        additional_env={'PYTHONPATH': _full_pypath},
    )

    # ── pct_path_adapter (C++ 航点适配器) ────────────────────────────────────
    # 注: yaml 未安装到 share 目录, 直接内联参数
    pct_adapter_node = Node(
        package='pct_adapters',
        executable='pct_path_adapter',
        name='pct_path_adapter',
        output='screen',
        parameters=[{
            # 降采样间距 (m): 与 smooth_min_dist=8×0.2=1.6m 对应
            'waypoint_distance':  1.5,
            # 到达半径 (m): 0.8m 防止立即跳过还未到达的航点
            'arrival_threshold':  0.8,
            # 卡顿检测超时 (s): 仿真中适当放宽
            'stuck_timeout_sec':  60.0,
            # 前视距离 (m)
            'lookahead_dist':     2.0,
        }],
        remappings=[
            ('/pct_path',         '/nav/global_path'),
            ('/Odometry',         '/nav/odometry'),
            ('/planner_waypoint', '/nav/way_point'),
        ],
    )

    # ── localPlanner (C++ 局部规划器) ─────────────────────────────────────────
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder':            path_folder,
            # 机器人几何 (四足标准值)
            'vehicleLength':         0.65,
            'vehicleWidth':          0.30,
            'sensorOffsetX':         0.0,
            'sensorOffsetY':         0.0,
            'twoWayDrive':           False,
            # 地形参数
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
            # 路径评分
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
            # 速度
            'maxSpeed':              1.0,
            'pathScaleBySpeed':      True,
            'minPathRange':          1.0,
            'pathRangeStep':         0.5,
            'pathRangeBySpeed':      True,
            'pathCropByGoal':        True,
            # 自主模式
            'autonomyMode':          True,
            'autonomySpeed':         0.8,
            'joyToSpeedDelay':       2.0,
            'joyToCheckObstacleDelay': 5.0,
            # 手柄轴 (仿真中无手柄, 保留默认值)
            'joy_axis_fwd':          4,
            'joy_axis_left':         3,
            'joy_axis_autonomy':     2,
            'joy_axis_obstacle':     5,
            # 目标处理
            'freezeAng':             90.0,
            'freezeTime':            2.0,
            'omniDirGoalThre':       1.0,
            'goalClearRange':        0.5,
            'goalBehindRange':       0.8,
            'goalX':                 0.0,
            'goalY':                 0.0,
        }],
        remappings=[
            ('/Odometry',    '/nav/odometry'),
            ('/cloud_map',   '/nav/map_cloud'),
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

    # ── pathFollower (C++ Pure Pursuit 路径跟踪器) ────────────────────────────
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX':    0.0,
            'sensorOffsetY':    0.0,
            'pubSkipNum':       1,
            # 禁用双向行驶, 防止前后振荡 (建筑走廊向前导航不需要倒车)
            'twoWayDrive':      False,
            # Pure Pursuit 参数
            'lookAheadDis':     0.5,
            'baseLookAheadDis': 0.3,
            'lookAheadRatio':   0.5,
            'minLookAheadDis':  0.2,
            'maxLookAheadDis':  2.0,
            # 控制增益
            'yawRateGain':      3.5,
            'stopYawRateGain':  3.5,
            'maxYawRate':       45.0,
            'maxSpeed':         1.0,
            'maxAccel':         0.5,
            # 切换 / 停止
            'switchTimeThre':   1.0,
            # 方向差阈值放宽 (0.5 rad ≈ 28.6°): 减少不必要的行驶方向切换
            'dirDiffThre':      0.5,
            'omniDirGoalThre':  1.0,
            'omniDirDiffThre':  1.5,
            'stopDisThre':      0.2,
            'slowDwnDisThre':   1.0,
            # 坡度/倾斜保护 (仿真中关闭)
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
            # 自主模式
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
            ('/Odometry',  '/nav/odometry'),
            ('/path',      '/nav/local_path'),
            ('/speed',     '/nav/speed'),
            ('/stop',      '/nav/stop'),
            ('/slow_down', '/nav/slow_down'),
            ('/cmd_vel',   '/nav/cmd_vel'),
        ],
    )

    # ── sim_robot_node.py (Python 仿真机器人节点) ─────────────────────────────
    # 使用 ExecuteProcess 启动独立 Python 脚本 (非 colcon 包内)
    sim_script = os.path.join(os.path.dirname(__file__), 'sim_robot_node.py')

    sim_robot = ExecuteProcess(
        cmd=['python3', sim_script],
        output='screen',
        # 通过环境变量传入 launch 参数 (LaunchConfiguration 支持 Substitution)
        additional_env={
            'SIM_GOAL_X':       goal_x,
            'SIM_GOAL_Y':       goal_y,
            'SIM_GOAL_Z':       goal_z,
            'SIM_START_X':      start_x,
            'SIM_START_Y':      start_y,
            'SIM_MAP_X_MIN':    map_x_min,
            'SIM_MAP_X_MAX':    map_x_max,
            'SIM_MAP_Y_MIN':    map_y_min,
            'SIM_MAP_Y_MAX':    map_y_max,
            'SIM_PCD_PATH':     pcd_path,
            'SIM_SCENE_NAME':   scene_name,
            'PYTHONUNBUFFERED': '1',
        },
    )

    # ── robot_state_publisher (URDF TF + 车轮旋转) ───────────────────────────
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
    )

    # ── foxglove_bridge (备用 Web 可视化: ws://robot_ip:8765) ────────────────
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'send_buffer_limit': 10000000,
        }],
    )

    # ── RViz2 (显示屏本地可视化) ───────────────────────────────────────────────
    # 通过环境变量 USE_RVIZ=1 启用; 需要 DISPLAY 或 WAYLAND_DISPLAY 已设置
    _use_rviz = os.environ.get('USE_RVIZ', '0') == '1'
    _rviz_cfg = os.path.join(os.path.dirname(__file__), 'sim_nav.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', _rviz_cfg] if os.path.exists(_rviz_cfg) else [],
    ) if _use_rviz else None

    nodes = [
        sim_robot,              # 最先: 提供 odometry + terrain + /robot_description
        robot_state_pub_node,   # URDF TF (base_link → wheels)
        pct_planner_node,       # 全局规划
        pct_adapter_node,       # 航点适配
        local_planner_node,     # 局部规划
        path_follower_node,     # 速度控制
        foxglove_node,          # Web 备用可视化
    ]
    if rviz_node is not None:
        nodes.append(rviz_node)

    return LaunchDescription([
        map_path_arg, goal_x_arg, goal_y_arg, goal_z_arg, start_x_arg, start_y_arg,
        map_x_min_arg, map_x_max_arg, map_y_min_arg, map_y_max_arg,
        pcd_path_arg, scene_name_arg,
        *nodes,
    ])
