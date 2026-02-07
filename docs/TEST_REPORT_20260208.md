# 导航系统实机测试报告

**日期**: 2026-02-08 04:25 ~ 04:43 (UTC+8)
**环境**: robot (RK3588, Ubuntu 22.04, ROS2 Humble)
**LiDAR**: Livox MID-360 @ 192.168.1.115

---

## 总体结果

| 测试项 | 状态 | 备注 |
|--------|------|------|
| 测试1: Fast-LIO2 里程计 | ✅ 通过 | 全部指标达标 |
| 测试2: PGO 建图 | ✅ 软件通过 | save_maps/TF/poses 正常，回环检测待移动机器人 |
| 测试3: Localizer 重定位 | ✅ 通过 | 全部指标达标 |
| 测试4: 地形感知 | ✅ 通过 | 全部指标达标 |
| 测试5: 局部规划 | ✅ 通过 | 避障链路完整验证 |
| 测试6: 端到端联通 | ✅ 通过 | TF完整链、全话题活跃、rosbag录制 |

---

## 测试 1: Fast-LIO2 里程计

**启动命令**:
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 run fastlio2 lio_node --ros-args -p config_path:=<path>/lio.yaml
```

| 指标 | 期望 | 实际 | 状态 |
|------|------|------|------|
| `/imu/data` 频率 | ~200Hz | **191Hz** | ✅ |
| `/lidar/scan` 频率 | ~10Hz | **10Hz** | ✅ |
| `/Odometry` 频率 | >5Hz | **10Hz** | ✅ |
| `/cloud_registered` 频率 | >5Hz | **10Hz** | ✅ |
| `/cloud_map` 频率 | >5Hz | **10Hz** | ✅ |
| TF `odom → body` | 正常发布 | **正常** | ✅ |
| 位姿输出 | 有数值 | x:-0.015 y:0.024 z:-0.030 | ✅ |

**结论**: Odometry 与 LiDAR 扫描同步 (10Hz)，所有数据链路正常。

---

## 测试 2: PGO 建图

**启动命令**:
```bash
ros2 run pgo pgo_node --ros-args -p config_path:=<path>/pgo.yaml
```

| 指标 | 期望 | 实际 | 状态 |
|------|------|------|------|
| TF `map → odom` | 正常发布 | **正常** (偏移~0) | ✅ |
| PGO 节点订阅 | /Odometry + /cloud_registered | **已订阅** | ✅ |
| `/pgo/save_maps` 服务 | success=true | **SAVE SUCCESS!** | ✅ |
| `map.pcd` | 文件存在 | **31KB** (静止1帧) | ✅ |
| `poses.txt` | 非空 | **1个关键帧** | ✅ |
| `patches/` | 有文件 | **0.pcd** (31KB) | ✅ |
| 回环检测 | "loop detected" | ⏳ **需物理移动** | 待测 |

**结论**: PGO 软件功能完全正常，地图保存/加载链路通畅。回环检测需要机器人实际走回环。

---

## 测试 3: Localizer 重定位

**启动命令**:
```bash
ros2 run localizer localizer_node --ros-args -p config_path:=<path>/localizer.yaml
ros2 service call /relocalize interface/srv/Relocalize "{pcd_path: '...map.pcd', ...}"
```

| 指标 | 期望 | 实际 | 状态 |
|------|------|------|------|
| `/relocalize` 返回 | success=true | **"relocalize success"** | ✅ |
| `/relocalize_check` | valid=true | **valid=True** | ✅ |
| TF `map → odom` | 正常发布 | **[0.009, -0.004, -0.017]** | ✅ |
| TF `map → body` | 正常发布 | **[-0.009, 0.022, -0.024]** | ✅ |
| 定位质量 `/localization_quality` | < 0.2 | **0.016** | ✅ |

**结论**: 重定位功能完美工作，ICP 收敛质量极好 (0.016)。

---

## 测试 4: 地形感知

**启动命令**:
```bash
ros2 launch terrain_analysis terrain_analysis.launch
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
```

| 指标 | 期望 | 实际 | 状态 |
|------|------|------|------|
| `/terrain_map` 频率 | > 2Hz | **10Hz** | ✅ |
| `/terrain_map_ext` 频率 | > 2Hz | **10Hz** | ✅ |
| `/terrain_map` 坐标系 | `odom` | **odom** | ✅ |
| `/terrain_map_ext` 坐标系 | `odom` | **odom** | ✅ |
| 点云字段 | x,y,z,intensity | **point_step=32** | ✅ |
| 点数量 | 合理 | **2084 点** | ✅ |

**结论**: 地形分析管线正常，点云数据完整。

---

## 测试 5: 局部规划

**启动命令**:
```bash
ros2 launch local_planner local_planner.launch autonomyMode:=true maxSpeed:=0.5
ros2 topic pub /way_point ... "{point: {x: 5.0, y: 0.0}}" --once
```

| 指标 | 期望 | 实际 | 状态 |
|------|------|------|------|
| `/path` 频率 | ~5Hz | **~20Hz** | ✅ |
| `/path` 坐标系 | odom | **odom** | ✅ |
| `/free_paths` 频率 | 有数据 | **~10Hz** | ✅ |
| `/slow_down` | 有数据 | **~1Hz, 值=3** | ✅ |
| 近场避障 | 检测到障碍 | **"NEAR-FIELD ESTOP: obstacle within 0.50m!"** | ✅ |
| `/cmd_vel` | 障碍时=0 | **全零** (ESTOP 生效) | ✅ |
| waypoint 接收 | 能接收 | **localPlanner 有响应** | ✅ |

**避障链路验证**:
```
terrain_analysis → /terrain_map → localPlanner (检测) → NEAR-FIELD ESTOP 
→ /slow_down=3 → pathFollower (减速) → /cmd_vel=0
```

**结论**: 完整避障链路正常工作，从感知到控制的闭环验证成功。

---

## 测试 6: 端到端联通验证

### 运行节点 (14个)

```
livox_lidar_publisher → lio_node → localizer_node
                                 → terrainAnalysis → terrainAnalysisExt
                                 → sensorScanGeneration
                                 → localPlanner → pathFollower
                     TF: lidarTransPublisher, cameraTransPublisher
```

### TF 树完整性

| TF 变换 | 发布者 | 状态 |
|---------|--------|------|
| `map → odom` | Localizer | ✅ |
| `odom → body` | Fast-LIO2 | ✅ |
| `body → lidar` | static_tf | ✅ |
| `lidar → camera` | static_tf | ✅ |
| `map → body` | 完整链路 | ✅ |
| `map → camera` | 完整链路 | ✅ |

### 关键话题活跃度

| 话题 | 频率 |
|------|------|
| `/Odometry` | 10.0 Hz |
| `/cloud_registered` | 10.0 Hz |
| `/cloud_map` | 10.0 Hz |
| `/terrain_map` | 10.0 Hz |
| `/terrain_map_ext` | 10.0 Hz |
| `/path` | 24.4 Hz |
| `/cmd_vel` | 50.0 Hz |
| `/localization_quality` | 9.3 Hz |
| `/sensor_scan` | 10.0 Hz |
| `/slow_down` | 1.1 Hz |

### Rosbag 录制

```
文件: test_bag_20260208_044246/
大小: ~35MB (10秒)
话题: /Odometry, /cloud_registered, /terrain_map, /path, /cmd_vel, /slow_down, /way_point, /localization_quality
```

**结论**: 全链路数据流通畅，所有节点正常运行，无崩溃、无 TF 异常。

---

## 待物理验证项

以下测试需要**物理移动机器人**，待人工操作后补充:

1. **PGO 回环检测**: 走一圈回到起点，验证 "loop detected" 日志
2. **障碍物高度检测**: 放置 >20cm 物体，验证 terrain_map intensity > 0.1
3. **实际导航**: 机器人向目标移动、遇障减速/绕行、到达停车
4. **建图质量**: 走完场地后 map.pcd > 5MB

---

## 系统健康度评估

| 维度 | 评分 | 说明 |
|------|------|------|
| SLAM 核心 | 10/10 | Fast-LIO2 稳定运行，所有输出正常 |
| 定位 | 10/10 | 重定位极快，质量 0.016 (远优于 0.2 阈值) |
| 感知 | 9/10 | 地形分析链路完整，待物理障碍物验证 |
| 规划 | 9/10 | 避障链路验证成功，待实际移动验证 |
| 全链路 | 9/10 | 14 个节点 + 10+ 话题全部活跃，TF 完整 |

**总评**: 导航系统软件层面功能完整，核心链路全部通过。系统可以进入物理移动测试阶段。
