# SLAM Interface Services

本包定义了用于 SLAM 系统交互的自定义 ROS 2 服务接口，包括地图保存、重定位、轨迹记录等功能。

## 服务列表

| 服务名称 | 接口类型 | 提供者 | 功能 |
|---------|---------|--------|------|
| `/save_map` | `SaveMaps.srv` | FastLIO2, PGO | 保存全局地图 |
| `/relocalize` | `Relocalize.srv` | Localizer | 加载地图并重定位 |
| `/relocalize_check` | `IsValid.srv` | Localizer | 检查重定位状态 |
| `/pgo/save_maps` | `SaveMaps.srv` | PGO | 保存优化后的地图 |
| `/pgo/save_poses` | `SavePoses.srv` | PGO | 保存关键帧轨迹 |

---

## 1. SaveMaps.srv - 保存地图

### 接口定义
```
# Request
string file_path      # 保存目录的绝对路径
bool save_patches     # 是否保存分块地图（用于大地图场景）

# Response
bool success          # 是否成功
string message        # 状态消息
```

### 使用示例

#### 保存 FastLIO2 地图（单文件）
```bash
ros2 service call /save_map interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/office_map', save_patches: false}"
```

#### 保存 PGO 优化地图（分块）
```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/warehouse', save_patches: true}"
```

### 输出文件结构
```
/home/user/maps/office_map/
├── map.pcd              # 全局点云地图
├── poses.txt            # 关键帧位姿列表 (如果 save_patches=true)
└── patches/             # 分块地图 (如果 save_patches=true)
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

---

## 2. Relocalize.srv - 重定位

### 接口定义
```
# Request
string pcd_path       # 地图文件路径 (*.pcd)
float32 x             # 初始位置估计 X (米)
float32 y             # 初始位置估计 Y (米)
float32 z             # 初始位置估计 Z (米)
float32 yaw           # 初始偏航角 (弧度)
float32 pitch         # 初始俯仰角 (弧度)
float32 roll          # 初始滚转角 (弧度)

# Response
bool success
string message
```

### 使用场景
- 机器人启动时，在已知地图中定位
- 绑架问题（kidnapped robot problem）恢复
- 切换到新的工作区域

### 使用示例

#### 简单定位（仅指定位置）
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 0.0, y: 0.0, z: 0.0, \
    yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

#### 精确定位（已知初始姿态）
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/warehouse/map.pcd', \
    x: 10.5, y: -3.2, z: 0.0, \
    yaw: 1.57, pitch: 0.0, roll: 0.0}"
```

### 参数说明
- **`pcd_path`**: 必须是 `.pcd` 格式的点云地图
- **位置估计**: 允许 ±5 米误差，定位算法会自动优化
- **姿态估计**: 允许 ±30° 误差（`yaw` 最重要，`pitch/roll` 通常为 0）

---

## 3. IsValid.srv - 检查定位状态

### 接口定义
```
# Request
int32 code            # 查询代码 (通常为 0)

# Response
bool valid            # 定位是否有效
```

### 使用示例

#### 检查重定位是否完成
```bash
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
```

#### 返回示例
```yaml
valid: true
```

### 典型工作流
```bash
# Step 1: 发起重定位请求
ros2 service call /relocalize interface/srv/Relocalize ...

# Step 2: 等待 2-5 秒

# Step 3: 检查是否成功
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"

# Step 4: 如果 valid=true，可以开始导航
```

---

## 4. SavePoses.srv - 保存轨迹

### 接口定义
```
# Request
string file_path      # 保存路径 (*.txt)

# Response
bool success
string message
```

### 使用示例

```bash
ros2 service call /pgo/save_poses interface/srv/SavePoses \
  "{file_path: '/home/user/trajectories/run_001.txt'}"
```

### 输出格式
```
# file_name x y z qw qx qy qz
0.pcd 0.000 0.000 0.000 1.000 0.000 0.000 0.000
1.pcd 0.523 0.102 0.000 0.998 0.000 0.000 0.065
2.pcd 1.045 0.205 0.000 0.995 0.000 0.000 0.098
...
```

---

## 5. RefineMap.srv - 优化地图

### 接口定义
```
# Request
string maps_path      # 地图目录路径

# Response
bool success
string message
```

### 使用示例

```bash
ros2 service call /refine_map interface/srv/RefineMap \
  "{maps_path: '/home/user/maps/office_map'}"
```

---

## 常见工作流程

### 📍 场景1: 建图（Mapping）

```bash
# 1. 启动系统（建图模式）
ros2 launch PCT_planner system_launch.py

# 2. 控制机器人移动（手动或自主）
# 使用手柄或发送导航目标

# 3. 建图完成后保存
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/new_map', save_patches: true}"

# 4. 保存轨迹（可选，用于分析）
ros2 service call /pgo/save_poses interface/srv/SavePoses \
  "{file_path: '/home/user/maps/new_map/trajectory.txt'}"
```

---

### 📍 场景2: 定位与导航（Localization & Navigation）

```bash
# 1. 启动系统
ros2 launch PCT_planner system_launch.py

# 2. 加载已有地图并重定位
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 3. 等待 2-3 秒，检查定位状态
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"

# 4. 如果 valid=true，发送导航目标
ros2 topic pub /way_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 5.0, z: 0.0}}"
```

---

### 📍 场景3: 绑架恢复（Kidnapped Robot Recovery）

```bash
# 1. 机器人被搬运到未知位置，TF 树断裂

# 2. 手动估计大致位置（通过 RViz 或已知信息）
# 假设机器人在地图的 (5, -2) 位置，朝东（yaw ≈ 0）

# 3. 重新定位
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 5.0, y: -2.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 4. 验证
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
```

---

## 调试技巧

### 查看服务状态
```bash
# 列出所有正在运行的服务
ros2 service list

# 查看服务接口类型
ros2 service type /save_map

# 查看服务详细信息
ros2 service info /relocalize
```

### 监控定位质量
```bash
# 查看定位误差（如果 Localizer 发布该话题）
ros2 topic echo /localization_error

# 可视化 TF 树
ros2 run rqt_tf_tree rqt_tf_tree
```

---

