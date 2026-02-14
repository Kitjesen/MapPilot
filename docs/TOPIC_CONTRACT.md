# Topic Interface Contract

## 概述

3d_NAV 使用**标准话题接口契约**解耦具体算法实现与上层系统。所有模块间通信通过 `/nav/*` 前缀的标准话题进行，具体算法通过 **launch profile** 将自身原生话题 remap 到标准名称。

这意味着：
- **换算法 = 换 profile**，不需要改 C++ 代码或 YAML 配置
- 上层模块（gRPC Gateway、StatusAggregator、HealthMonitor 等）只知道 `/nav/*` 话题
- 零破坏性变更：老的 launch 文件和 profile 可以共存

## 标准话题列表

完整定义见 [`config/topic_contract.yaml`](../config/topic_contract.yaml)。

### 传感器层

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/nav/lidar_scan` | `livox_ros_driver2/CustomMsg` | LiDAR 点云 |
| `/nav/imu` | `sensor_msgs/Imu` | IMU 数据 |

### SLAM 层

| 话题/服务 | 类型 | 说明 |
|-----------|------|------|
| `/nav/odometry` | `nav_msgs/Odometry` | 里程计 |
| `/nav/registered_cloud` | `sensor_msgs/PointCloud2` | 机体坐标系点云 |
| `/nav/map_cloud` | `sensor_msgs/PointCloud2` | 世界坐标系点云 |
| `/nav/save_map` | `interface/srv/SaveMaps` | 保存地图服务 |

### 定位层

| 话题/服务 | 类型 | 说明 |
|-----------|------|------|
| `/nav/localization_quality` | `std_msgs/Float32` | ICP 匹配质量分数 |
| `/nav/relocalize` | `interface/srv/Relocalize` | 重定位服务 |

### 全局规划层

| 话题 | 类型 | 说明 |
|------|------|------|
| `/nav/global_path` | `nav_msgs/Path` | 全局路径 |
| `/nav/planner_status` | `std_msgs/String` | 规划器状态 |
| `/nav/waypoint` | `geometry_msgs/PointStamped` | 适配后的局部航点 |

### 自主导航层

| 话题 | 类型 | 说明 |
|------|------|------|
| `/nav/terrain_map` | `sensor_msgs/PointCloud2` | 基础地形图 |
| `/nav/terrain_map_ext` | `sensor_msgs/PointCloud2` | 扩展地形图 |
| `/nav/local_path` | `nav_msgs/Path` | 局部路径 |
| `/nav/cmd_vel` | `geometry_msgs/TwistStamped` | 速度指令 |
| `/nav/stop` | `std_msgs/Int8` | 急停 |
| `/nav/slow_down` | `std_msgs/Int8` | 减速 |
| `/nav/way_point` | `geometry_msgs/PointStamped` | 目标航点 |
| `/nav/speed` | `std_msgs/Float32` | 速度倍率 |

### 高层控制

| 话题 | 类型 | 说明 |
|------|------|------|
| `/nav/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标 |

### TF 坐标系

| 帧 | 说明 |
|----|------|
| `map` | 全局地图坐标系 |
| `odom` | 里程计坐标系 |
| `body` | 机体坐标系 |

TF 链: `map` -> `odom` (定位器发布) -> `body` (SLAM 发布)

## Algorithm Profiles

Profile 是一个小的 launch 文件，负责启动具体算法节点并 remap 其原生话题到标准接口。

### 目录结构

```
launch/profiles/
  slam_fastlio2.launch.py    # Fast-LIO2 SLAM
  slam_stub.launch.py        # 测试桩 (无硬件)
  planner_pct.launch.py      # PCT 全局规划器
  planner_stub.launch.py     # 测试桩
  localizer_icp.launch.py    # ICP 定位器
```

### 使用方式

```bash
# 默认 (Fast-LIO2 + PCT)
ros2 launch navigation_run.launch.py

# 切换 SLAM 算法
ros2 launch navigation_run.launch.py slam_profile:=stub

# 切换规划器
ros2 launch navigation_run.launch.py planner_profile:=stub

# 全部使用测试桩 (无硬件开发)
ros2 launch navigation_run.launch.py \
  slam_profile:=stub \
  planner_profile:=stub
```

## 如何添加新算法

### 示例: 添加 LIO-SAM 作为 SLAM 后端

1. **创建 profile 文件**: `launch/profiles/slam_liosam.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    liosam_node = Node(
        package="lio_sam",
        executable="lio_sam_node",
        name="lio_sam_node",
        output="screen",
        remappings=[
            # LIO-SAM 原生话题 → 标准接口
            ("/lio_sam/mapping/odometry", "/nav/odometry"),
            ("/lio_sam/mapping/cloud_registered", "/nav/registered_cloud"),
            ("/lio_sam/mapping/map_global", "/nav/map_cloud"),
            # 输入
            ("/imu_raw", "/nav/imu"),
            ("/points_raw", "/nav/lidar_scan"),
        ],
    )
    return LaunchDescription([liosam_node])
```

2. **使用**:

```bash
ros2 launch navigation_run.launch.py slam_profile:=liosam
```

无需修改任何其他文件。上层 gRPC Gateway、HealthMonitor 等自动通过 `/nav/odometry`、`/nav/registered_cloud` 接收数据。

### 示例: 添加 A* 全局规划器

1. **创建 profile**: `launch/profiles/planner_astar.launch.py`
2. 确保节点发布 `/nav/global_path` (nav_msgs/Path) 和 `/nav/planner_status` (std_msgs/String)
3. 如果有路径适配器，确保发布 `/nav/waypoint` (geometry_msgs/PointStamped)
4. 使用: `ros2 launch navigation_run.launch.py planner_profile:=astar`

## 后向兼容

所有话题名通过 ROS2 参数配置（`grpc_gateway.yaml`），C++ 代码中的默认值已更新为 `/nav/*`。如果需要兼容旧版话题名，可以在 YAML 中覆盖：

```yaml
grpc_gateway:
  ros__parameters:
    odom_topic: "/Odometry"           # 覆盖回旧名
    save_map_service: "/fastlio2/save_map"  # 覆盖回旧名
```
