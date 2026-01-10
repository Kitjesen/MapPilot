# 🚀 SLAM 导航系统快速启动指南

## 📋 目录

1. [系统概述](#系统概述)
2. [快速启动脚本](#快速启动脚本)
3. [完整工作流程](#完整工作流程)
4. [故障排查](#故障排查)

---

## 系统概述

本工作空间包含完整的 **3D SLAM + 全局规划** 系统，基于 ROS 2 Humble：

- **建图系统**: FAST-LIO2 + PGO (位姿图优化)
- **规划系统**: PCT Planner (Point Cloud Tomography)
- **传感器支持**: Orbbec Gemini 330 相机、Livox 激光雷达

---

## 快速启动脚本

### 1️⃣ 建图 (Mapping)

```bash
./mapping.sh
```

**功能:**
- 启动传感器 (相机/激光雷达)
- 启动 FAST-LIO2 进行实时 SLAM
- 启动 PGO 进行后端优化
- 启动 RViz2 可视化建图效果

**操作流程:**
1. 选择传感器类型 (Orbbec/Livox/跳过)
2. 等待系统启动 (约 5-10 秒)
3. 在 RViz2 中观察建图效果
4. 移动机器人/传感器扫描环境
5. 完成后运行 `./save_map.sh` 保存地图

---

### 2️⃣ 保存地图 (Save Map)

```bash
./save_map.sh
```

**功能:**
- 保存 PGO 优化后的点云地图 (.pcd)
- 生成 PCT Tomogram 3D 地图 (.pickle)
- 自动组织文件到对应目录

**保存选项:**
1. **仅保存 PGO 点云** - 快速保存原始点云
2. **保存 + 生成 PCT 地图** ⭐ 推荐 - 完整地图生成
3. **仅生成 PCT Tomogram** - 从已有 PCD 生成

**地图文件位置:**
- 原始点云: `maps/map_YYYYMMDD_HHMMSS.pcd`
- PCT 地图: `src/global_planning/PCT_planner/rsc/tomogram/*.pickle`
- PCD 备份: `src/global_planning/PCT_planner/rsc/PCD/*.pcd`

---

### 3️⃣ 规划/导航 (Planning)

```bash
./planning.sh
```

**功能:**
- 加载已保存的地图
- 启动定位系统 (假定位/FAST-LIO2)
- 启动 PCT 全局规划器
- 启动 RViz2 进行交互式规划

**定位模式:**
1. **假定位 (Fake Localization)** ⭐ 测试推荐
   - 手动设置机器人位置
   - 在 RViz 使用 "2D Pose Estimate" 工具
   
2. **真定位 (FAST-LIO2 Localization)**
   - 需要激光雷达/相机
   - 自动定位在已有地图中

3. **跳过定位**
   - 使用外部定位系统

**规划操作:**
1. 确保机器人位置正确 (绿色箭头)
2. 在 RViz 中使用 **"Publish Point"** 工具点击目标点
3. 规划器自动计算 3D 路径
4. 查看路径可视化 (蓝色线条)

---

## 完整工作流程

### 📍 首次使用流程

```bash
# 1. 编译工作空间
colcon build --symlink-install
source install/setup.bash

# 2. 建图
./mapping.sh
# 移动机器人扫描环境...

# 3. 保存地图
./save_map.sh
# 选择: 2) 保存 + 生成 PCT 地图

# 4. 规划测试
./planning.sh
# 定位: 1) 假定位
# 传感器: 3) 跳过
# 在 RViz 中点击设置目标点
```

---

### 🔁 日常使用流程

```bash
# 已有地图，直接启动规划
./planning.sh

# 在 RViz 中:
# 1. 使用 "2D Pose Estimate" 设置起点 (假定位模式)
# 2. 使用 "Publish Point" 点击目标点
# 3. 观察生成的 3D 路径
```

---

## 故障排查

### ❌ 问题: 找不到地图文件

**现象:**
```
警告: 未找到地图文件！
```

**解决:**
```bash
# 检查地图是否存在
ls -lh src/global_planning/PCT_planner/rsc/tomogram/

# 如果没有，运行建图流程
./mapping.sh
# ... 建图后 ...
./save_map.sh
```

---

### ❌ 问题: RViz 无法显示点云

**现象:** RViz 中看不到地图点云

**解决:**
1. 检查话题是否发布:
   ```bash
   ros2 topic list | grep tomogram
   ros2 topic echo /pct_planner/tomogram --once
   ```

2. 检查 QoS 设置:
   - 在 RViz 中，PointCloud2 显示的 QoS 设置为:
     - Reliability: Best Effort
     - Durability: **Transient Local** ⚠️ 重要

3. 重启 RViz:
   ```bash
   killall rviz2
   rviz2 -d src/global_planning/PCT_planner/rsc/rviz/pct_ros.rviz
   ```

---

### ❌ 问题: 无法规划路径

**现象:** 点击目标点后没有路径生成

**排查步骤:**
1. **检查定位是否正常**:
   ```bash
   ros2 topic echo /tf --once
   # 应该看到 map -> body 的变换
   ```

2. **检查目标点是否有效**:
   - 目标点必须在地图范围内
   - 目标点必须在可通行区域 (非障碍物)

3. **查看规划器日志**:
   - 切换到 "PCT Global Planner" 终端
   - 查看是否有错误信息

4. **手动测试规划器**:
   ```bash
   python3 src/global_planning/PCT_planner/planner/scripts/check_map.py
   ```

---

### ❌ 问题: 传感器无法启动

**Orbbec 相机问题:**
```bash
# 检查 USB 权限
sudo bash src/utils/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh

# 重新插拔相机

# 手动启动测试
ros2 launch orbbec_camera gemini_330_series.launch.py
```

**Livox 激光雷达问题:**
```bash
# 检查网络连接
ping 192.168.1.1

# 检查设备 IP 配置
# 编辑: src/drivers/livox_ros_driver2/config/MID360_config.json
```

---

### ❌ 问题: 编译错误

```bash
# 清理重新编译
rm -rf build/ install/ log/
colcon build --symlink-install

# 如果是 PCT_planner C++ 库问题
cd src/global_planning/PCT_planner/planner
./build.sh
cd ../../../../
colcon build --packages-select pct_planner pct_adapters --symlink-install
```

---

## 高级功能

### 🔧 手动启动各模块

如果自动脚本有问题，可以手动启动：

```bash
# Terminal 1: 相机
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py

# Terminal 2: FAST-LIO2
source install/setup.bash
ros2 launch fastlio2 lio_launch.py

# Terminal 3: PGO
source install/setup.bash
ros2 launch pgo pgo_launch.py

# Terminal 4: PCT Planner
source install/setup.bash
python3 src/global_planning/PCT_planner/planner/scripts/global_planner.py

# Terminal 5: RViz
source install/setup.bash
rviz2 -d src/global_planning/PCT_planner/rsc/rviz/pct_ros.rviz
```

---

### 📊 查看系统状态

```bash
# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /Odometry
ros2 topic hz /pct_planner/tomogram

# 查看 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 查看节点信息
ros2 node list
ros2 node info /global_planner
```

---

### 🎨 可视化 Tomogram

```bash
# 可视化已保存的地图
python3 src/global_planning/PCT_planner/tomography/scripts/visualize_tomogram.py --scene map_20240115_143000

# 查看地图统计信息
python3 src/global_planning/PCT_planner/planner/scripts/check_map.py
```

---

## 📞 需要帮助?

- 查看详细文档: `docs/`
- 查看包 README: `src/*/README.md`
- GitHub Issues: [项目链接]

---

**祝你使用愉快！** 🎉
