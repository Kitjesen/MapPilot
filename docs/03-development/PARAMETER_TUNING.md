# Parameter Tuning Guide

## Quick Reference

### 🎯 Start Here: Default Configurations

For most applications, use these presets:

| Scenario | Config File | Description |
|----------|-------------|-------------|
| **Indoor Navigation** | `config/indoor.yaml` | Slow speed, tight turns |
| **Outdoor Open Space** | `config/outdoor.yaml` | Fast speed, wide turns |
| **Rough Terrain** | `config/offroad.yaml` | Conservative, terrain-aware |

---

## Layer-by-Layer Tuning

### 1️⃣ Path Follower Parameters

**File**: `src/base_autonomy/local_planner/launch/pathFollower.launch.xml`

#### Speed & Acceleration

```yaml
maxSpeed: 1.0          # Maximum velocity (m/s)
  # 💡 Tip: Start at 0.5 m/s for testing
  # 🎯 Indoor: 0.6-1.0 | Outdoor: 1.5-3.0
  
maxAccel: 1.0          # Maximum acceleration (m/s²)
  # 💡 Tip: Higher = more responsive, but jerky
  # 🎯 Smooth: 0.5 | Aggressive: 2.0
```

#### Lookahead Distance (Adaptive)

```yaml
baseLookAheadDis: 0.3  # Base lookahead at 0 m/s
lookAheadRatio: 0.5    # Additional distance per m/s
minLookAheadDis: 0.2   # Minimum lookahead
maxLookAheadDis: 2.0   # Maximum lookahead

# Formula: L = base + ratio * speed
# Example: At 2 m/s → L = 0.3 + 0.5*2 = 1.3m
```

**Tuning Strategy**:
- **Too Small**: Oscillation, overshooting
- **Too Large**: Cuts corners, slow response
- **Recommended**: Start with defaults, adjust `lookAheadRatio` by ±0.1

#### Turning Control

```yaml
yawRateGain: 7.5       # Proportional gain for turning
  # 💡 Higher = faster rotation response
  # 🎯 Sluggish: Increase to 10-15
  # 🎯 Oscillating: Decrease to 5-7
  
maxYawRate: 45.0       # Maximum turn rate (deg/s)
  # 💡 Safety limit, not control gain
  # 🎯 Slow robots: 30 | Agile: 60-90
```

---

### 2️⃣ Local Planner Parameters

**File**: `src/base_autonomy/local_planner/launch/localPlanner.launch.xml`

#### Obstacle Detection

```yaml
laserVoxelSize: 0.1    # Point cloud downsampling (m)
  # ⚖️ Tradeoff: Smaller = more detail, higher CPU
  # 🎯 Fast CPU: 0.05 | Slow CPU: 0.15
  # ⚠️ Affects collision detection resolution

obstacleHeightThre: 0.2  # Minimum obstacle height (m)
  # 💡 Points taller than this = obstacle
  # 🎯 Flat ground: 0.15 | Rough: 0.25
```

#### Planning Horizon

```yaml
minPathRange: 2.5      # Minimum planning distance (m)
  # ⚠️ MUST be ≥ maxLookAheadDis (Path Follower)
  # 🎯 Hallways: 2.0 | Open: 3.5
  
adjacentRange: 3.5     # Obstacle consideration radius (m)
  # 💡 Larger = safer, but slower
  # 🎯 Cluttered: 2.5 | Sparse: 5.0
```

#### Path Selection

```yaml
dirWeight: 0.02        # Direction alignment weight
  # 💡 Higher = prefers straight paths
  # 🎯 Goal-seeking: 0.05 | Exploration: 0.01
  
useCost: true          # Enable terrain cost
  # 💡 Requires terrain_analysis running
  # 🎯 Flat: false | Slopes: true
```

---

### 3️⃣ Terrain Analysis Parameters

**File**: `src/base_autonomy/terrain_analysis/launch/terrainAnalysis.launch.xml`

#### Voxel Configuration

```yaml
scanVoxelSize: 0.1     # Scan downsampling (m)
  # ⚖️ Smaller = finer terrain map, more memory
  # 🎯 Default: 0.1 | High-res: 0.05

terrainVoxelSize: 1.0  # Terrain grid resolution (m)
  # 💡 Affects rolling map size
  # 🎯 Indoor: 0.5 | Outdoor: 2.0
```

#### Temporal Filtering

```yaml
decayTime: 10.0        # Point cloud lifetime (s)
  # 💡 How long to remember obstacles
  # 🎯 Static: 30 | Crowded: 5
  
noDecayDis: 2.0        # No decay within this radius (m)
  # ⚠️ Points near robot never decay
  # 🎯 Small robots: 1.0 | Large: 3.0
```

#### Ground Detection

```yaml
groundHeightThre: 0.1  # Ground classification (m)
  # 💡 Points below this = ground
  # 🎯 Smooth: 0.08 | Bumpy: 0.15
  
disRatioZ: 0.1         # Distance-based Z tolerance
  # 💡 Allows more Z variance far away
  # Formula: threshold = base + ratio * distance
```

---

### 4️⃣ Global Planner Parameters

**File**: `src/global_planning/PCT_planner/config/params.yaml`

#### A* Search

```yaml
w_traversability: 1.0  # Traversability cost weight
w_smoothness: 0.2      # Path smoothness weight
w_length: 0.1          # Path length weight

# 💡 Higher weight = more influence
# 🎯 Off-road: Increase traversability
# 🎯 Racing: Increase smoothness
```

#### Trajectory Optimization

```yaml
trajectory_resolution: 0.1  # Waypoint spacing (m)
max_iterations: 100         # Optimization iterations
convergence_threshold: 0.01 # Stop criteria

# ⚖️ More iterations = smoother, slower
```

---

## Common Tuning Scenarios

### 🐌 Robot is Too Slow

**Symptoms**: Robot crawls even with high joystick input

**Solutions**:
1. ✅ Increase `maxSpeed` (Path Follower)
2. ✅ Increase `autonomySpeed` if in autonomous mode
3. ✅ Check `/slow_down` topic isn't constantly triggering
4. ✅ Reduce `slowPathNumThre` (Local Planner)

---

### 📐 Robot Cuts Corners

**Symptoms**: Doesn't follow path accurately

**Solutions**:
1. ✅ Increase `baseLookAheadDis` (Path Follower)
2. ✅ Decrease `lookAheadRatio`
3. ✅ Increase `yawRateGain` for sharper turns
4. ✅ Reduce `maxSpeed` in tight spaces

---

### 🌀 Robot Oscillates

**Symptoms**: Snakes left-right around path

**Solutions**:
1. ✅ Decrease `yawRateGain` (Path Follower)
2. ✅ Increase `baseLookAheadDis`
3. ✅ Check odometry quality (`ros2 topic hz /Odometry`)
4. ✅ Smooth terrain map (`scanVoxelSize` → 0.15)

---

### 🚫 Too Many "Slow Down" Warnings

**Symptoms**: Robot constantly braking

**Solutions**:
1. ✅ Increase `slowPathNumThre` (Local Planner)
2. ✅ Increase `laserVoxelSize` (less dense obstacles)
3. ✅ Adjust `obstacleHeightThre` if ground is bumpy
4. ✅ Check LiDAR isn't seeing own robot parts

---

### 🗺️ Global Planner Fails Often

**Symptoms**: "No path found" errors

**Solutions**:
1. ✅ Check tomogram map covers goal area
2. ✅ Reduce `w_traversability` (less picky)
3. ✅ Increase `inflation_radius` for narrower gaps
4. ✅ Rebuild tomogram if environment changed

---

## Runtime Parameter Changes

### Using ROS 2 CLI

```bash
# View current parameters
ros2 param list /pathFollower

# Change a parameter
ros2 param set /pathFollower maxSpeed 1.5

# Dump all parameters to file
ros2 param dump /pathFollower > my_config.yaml
```

### Permanent Changes

Edit launch files, then rebuild:
```bash
colcon build --packages-select local_planner
source install/setup.bash
```

---

## Performance vs. Safety Tradeoffs

| Parameter | ← Conservative | Aggressive → |
|-----------|---------------|--------------|
| `maxSpeed` | 0.5 m/s | 2.0 m/s |
| `maxAccel` | 0.5 m/s² | 2.0 m/s² |
| `laserVoxelSize` | 0.05 m | 0.2 m |
| `adjacentRange` | 5.0 m | 2.0 m |
| `decayTime` | 30 s | 3 s |
| `obstacleHeightThre` | 0.1 m | 0.3 m |

**Recommendation**: Start conservative, tune aggressive once confident.

---

## Debugging Tools

### Visualize in RViz2

```bash
rviz2 -d src/base_autonomy/local_planner/rviz/navigation.rviz
```

**Key Topics**:
- `/free_paths`: See which paths are collision-free (green)
- `/terrain_map`: Check terrain analysis quality
- `/path`: Current selected path

### Monitor Metrics

```bash
# Path Follower performance
ros2 topic echo /cmd_vel

# Local Planner decisions
ros2 topic echo /slow_down

# Global Planner status
ros2 topic echo /planning_status
```

---

## 相关文档

- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 故障排查
- [BUILD_GUIDE.md](BUILD_GUIDE.md) — 编译指南
- [ARCHITECTURE.md](ARCHITECTURE.md) — 系统架构

---

*Last Updated: February 2026*

---

## slopeWeight — 坡度代价调参

### 参数位置
`config/robot_config.yaml` → `local_planner.dir_weight` 同级（实际参数名为 `slopeWeight`）

在 `launch/subsystems/autonomy.launch.py` 中通过 `robot_cfg` 读取并传给 localPlanner 节点。

### 推荐值

| 值 | 效果 |
|---|---|
| `0` (默认) | 关闭坡度代价，路径选择纯方向最优 |
| `3` | 轻度坡度惩罚，轻微避开陡坡 |
| `6` | 重度坡度惩罚，强烈偏好平坦路径 |
| `>10` | 过于保守，可能无法找到可行路径 |

### 与 dir_weight 配合
- `dir_weight` 控制方向对齐代价（默认 `0.02`），影响全局路径跟随度
- `slopeWeight` 独立叠加坡度代价项
- 两者乘积效应: 室内平坦场景保持 `slopeWeight=0`；户外复杂地形建议 `slopeWeight=3~6`

### 示例
```bash
# 临时覆盖（不改 yaml）:
ros2 param set /local_planner slopeWeight 4.0
```

---

## pathFollower 渐进卡死检测

### 参数位置
`pathFollower.launch.xml` — `stuck_timeout` 和 `stuck_dist_thre` 参数。

### 工作原理

pathFollower 周期性检查机器人在 `stuck_timeout` 秒内的位移。如果位移小于 `stuck_dist_thre`：
1. **半超时 (50%)**: 发布 `WARN_STUCK` 到 `/nav/planner_status`（早期预警）
2. **全超时 (100%)**: 发布 `STUCK` 到 `/nav/planner_status`（确认卡死）
3. **反向运动加速**: 如果命令前进但实际后退，剩余超时压缩到最多 3s
4. **恢复确认**: 连续 3 帧速度 >0.05m/s 才清除卡死状态

### 推荐值

| 参数 | 默认 | 室内 | 室外 | 说明 |
|---|---|---|---|---|
| `stuck_timeout` | 10.0 | 8.0 | 15.0 | 卡死判定时间窗口 (s) |
| `stuck_dist_thre` | 0.15 | 0.10 | 0.20 | 最小位移阈值 (m) |

### 示例
```bash
ros2 param set /pathFollower stuck_timeout 8.0
ros2 param set /pathFollower stuck_dist_thre 0.10
```

---

## pct_path_adapter 航点跟踪保护

### 参数位置
`pct_path_adapter` 节点 ROS2 参数。

### 参数说明

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_index_jump` | 3 | 单次回调允许的最大航点索引跳跃，超过时 WARN 日志（可能 TF 抖动） |
| `max_first_waypoint_dist` | 10.0 | 路径首航点与机器人的最大距离 (m)，超过则拒绝路径（坐标系不匹配） |
| `stuck_timeout_sec` | 10.0 | 航点卡死超时 (s)，触发重规划 |
| `max_replan_count` | 2 | 最大重规划次数，超过后发布 `stuck_final` |
| `replan_cooldown_sec` | 5.0 | 两次重规划之间的冷却时间 (s) |

### 调参建议
- `max_index_jump=3` 适合大多数场景；如果机器人速度快且航点间距小，可增大到 5
- `max_first_waypoint_dist=10.0` 若建图区域大，可增大到 20.0
- 如果频繁出现 "Waypoint index jumped" 警告，检查 TF 链是否稳定
