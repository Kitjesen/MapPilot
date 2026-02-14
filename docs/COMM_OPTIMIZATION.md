# 通信优化指南 — ROS 2 + CycloneDDS

> 目标: 不改业务代码，通过配置优化让系统更稳更快更可控

---

## 1. 启用 CycloneDDS (Step 1)

### 1.1 安装

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 1.2 环境变量

```bash
# 添加到 ~/.bashrc 或 entrypoint.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/3d_NAV/config/cyclonedds.xml
```

### 1.3 验证

```bash
ros2 doctor --report | grep -i middleware
# 期望输出: middleware name: rmw_cyclonedds_cpp
```

### 1.4 Docker 集成

在 `docker-compose.yml` 中:

```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONEDDS_URI=file:///nav_ws/config/cyclonedds.xml
```

---

## 2. QoS 配置落地 (Step 2)

### 2.1 场景化 QoS 参数表

基于 `config/qos_profiles.yaml` 定义，每条链路的推荐配置:

```
链路类型          Reliability    History     Depth  Deadline  Lifespan
─────────────────────────────────────────────────────────────────────
LiDAR 点云        BEST_EFFORT    KEEP_LAST   2      —         200ms
Odometry/IMU      BEST_EFFORT    KEEP_LAST   5      20ms      50ms
cmd_vel/stop      RELIABLE       KEEP_LAST   1      50ms      —
全局路径           RELIABLE       T_LOCAL     1      —         —
系统状态           RELIABLE       T_LOCAL     1      —         —
围栏/目标          RELIABLE       T_LOCAL     1      —         —
图像流             BEST_EFFORT    KEEP_LAST   1      —         100ms
TF                (ROS 2 默认)   —           —      —         —
```

### 2.2 在代码中应用

现有代码中，大多数话题使用 `rclcpp::SensorDataQoS()` 或
默认 `rclcpp::QoS(10)`。建议按以下方式调整:

```cpp
// 高频大数据: 点云
auto lidar_qos = rclcpp::SensorDataQoS()  // BEST_EFFORT + VOLATILE
    .keep_last(2)
    .lifespan(std::chrono::milliseconds(200));
sub_terrain_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/nav/terrain_map", lidar_qos, callback);

// 控制指令: 必达
auto cmd_qos = rclcpp::QoS(1)
    .reliable()
    .deadline(std::chrono::milliseconds(50));
pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/nav/cmd_vel", cmd_qos);

// 全局路径: TRANSIENT_LOCAL
auto path_qos = rclcpp::QoS(1)
    .reliable()
    .transient_local();
sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/nav/global_path", path_qos, callback);
```

### 2.3 QoS 兼容性矩阵

发布者和订阅者的 QoS 必须兼容，否则连接静默失败:

```
Publisher    ↓  Subscriber →   RELIABLE    BEST_EFFORT
RELIABLE                       ✅           ✅
BEST_EFFORT                    ❌           ✅

Publisher    ↓  Subscriber →   T_LOCAL     VOLATILE
T_LOCAL                        ✅           ✅
VOLATILE                       ❌           ✅
```

**常见坑**: 如果 rviz2 订阅不到某个 topic，多半是 QoS 不匹配。
用 `ros2 topic info -v /topic_name` 查看两端 QoS 配置。

---

## 3. 链路级 Profiling (Step 3)

### 3.1 端到端延迟测量

```bash
# 方法 1: ros2 topic delay (需要 header.stamp)
ros2 topic delay /nav/odometry

# 方法 2: ros2 topic hz (频率)
ros2 topic hz /nav/terrain_map

# 方法 3: ros2 topic bw (带宽)
ros2 topic bw /nav/slam/cloud_registered
```

### 3.2 关键链路延迟分解

```
LiDAR → SLAM → SafetyGate → cmd_vel

   ┌──────┐    ┌──────┐    ┌───────────┐    ┌─────────┐
   │Livox │───>│SLAM  │───>│SafetyGate │───>│Driver   │
   │10Hz  │ T1 │LIO   │ T2 │避障检测    │ T3 │cmd_vel │
   └──────┘    └──────┘    └───────────┘    └─────────┘
      T1: LiDAR → SLAM 传输 (DDS + PCL 转换)
      T2: SLAM → SafetyGate 传输 (DDS)
      T3: SafetyGate → Driver 传输 (DDS)

  测量: 对比 header.stamp 和 ros2::now()
  目标: T1+T2+T3 < 100ms (一个 LiDAR 周期)
```

### 3.3 CycloneDDS 内部诊断

```bash
# 启用详细追踪 (临时调试)
export CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing>
  <Verbosity>fine</Verbosity>
  <OutputFile>/tmp/cyclonedds.log</OutputFile>
</Tracing></Domain></CycloneDDS>'

# 查看发现时间、重传次数、丢包率
grep -E "matched|retransmit|drop" /tmp/cyclonedds.log
```

### 3.4 性能基线 (期望值)

| 指标 | 期望 | 报警阈值 |
|------|------|---------|
| LiDAR 发布频率 | 10 Hz | < 8 Hz |
| Odometry 发布频率 | 100 Hz | < 80 Hz |
| cmd_vel 端到端延迟 | < 20ms | > 50ms |
| SLAM → terrain_map 延迟 | < 50ms | > 100ms |
| 节点发现收敛时间 | < 3s | > 10s |
| PointCloud2 单帧传输时间 | < 5ms (loopback) | > 20ms |
| CPU (CycloneDDS 线程) | < 5% | > 15% |

---

## 4. 局部旁路方案 (Step 4)

仅在 Step 3 证明某条链路不达标时才做。

### 4.1 共享内存零拷贝 (iceoryx)

适用场景: 同主机进程间传输大数据 (PointCloud2)

```bash
# 安装 iceoryx
sudo apt install ros-humble-rmw-cyclonedds-cpp
# CycloneDDS 已内置 iceoryx 支持

# 启用 (在 cyclonedds.xml 中取消注释 SharedMemory 段)
```

```xml
<SharedMemory>
  <Enable>true</Enable>
  <LogLevel>warning</LogLevel>
</SharedMemory>
```

收益: PointCloud2 传输从 **~5ms** 降到 **~0.1ms** (零拷贝)

### 4.2 RT 线程隔离 (cmd_vel 闭环)

如果 SafetyGate → Driver 的 cmd_vel 延迟 > 50ms:

```cpp
// 在 safety_gate.cpp 中设置 RT 优先级
#include <sched.h>
struct sched_param sp;
sp.sched_priority = 80;  // RT 优先级
sched_setscheduler(0, SCHED_FIFO, &sp);

// 或通过 launch 参数
ros2 run remote_monitoring grpc_gateway --ros-args \
    --remap __node:=grpc_gateway \
    -p use_realtime:=true
```

### 4.3 专用通道 (极端场景)

如果以上都不够，仅对 cmd_vel 做非 DDS 通道:

```
SafetyGate ──[共享内存/POSIX shm]──> Driver
         ↕ (状态同步仍走 ROS 2)
```

这只需改 SafetyGate 的 `pub_cmd_vel_` 和 Driver 的 `sub_cmd_vel_` (2 处)，
其余系统保持 ROS 2 不变。

---

## 5. 网络场景配置

### 5.1 单机 (开发/测试)

```xml
<!-- cyclonedds.xml: 单机优化 -->
<General>
  <Interfaces>
    <NetworkInterface name="lo" />  <!-- 仅 loopback -->
  </Interfaces>
  <AllowMulticast>false</AllowMulticast>  <!-- 单机不需要组播 -->
</General>
```

### 5.2 同一子网 (机器人 + 开发机)

```xml
<!-- cyclonedds.xml: 默认配置即可 -->
<General>
  <Interfaces>
    <NetworkInterface autodetermine="true" />
  </Interfaces>
  <AllowMulticast>default</AllowMulticast>
</General>
```

### 5.3 跨子网 (远程运维)

```xml
<!-- cyclonedds.xml: 需要显式 peer -->
<Discovery>
  <Peers>
    <Peer address="192.168.1.100" />  <!-- 机器人 IP -->
  </Peers>
</Discovery>
<General>
  <AllowMulticast>false</AllowMulticast>
</General>
```

### 5.4 DDS Domain 隔离 (多机器人)

```bash
# 每台机器人使用不同 Domain ID
export ROS_DOMAIN_ID=42    # 机器人 A
export ROS_DOMAIN_ID=43    # 机器人 B
```

---

## 6. 排错手册

| 问题 | 原因 | 解决 |
|------|------|------|
| 节点间看不到话题 | RMW 不一致 | 确认所有节点 `RMW_IMPLEMENTATION` 一致 |
| rviz2 订阅不到 | QoS 不匹配 | `ros2 topic info -v /topic` 对比 QoS |
| 发现超时 | 组播被防火墙拦截 | 检查 `ufw` / iptables 放行 UDP 7400-7500 |
| 点云延迟大 | 默认缓冲太小 | `cyclonedds.xml` 增大 `SocketReceiveBufferSize` |
| 多网卡发现混乱 | DDS 绑错网卡 | `cyclonedds.xml` 指定 `NetworkInterface name` |
| Docker 容器间不通 | 需要 host 网络 | `network_mode: host` 或配置 DDS peer |

---

## 7. 总结

```
投入产出比:

配置优化 (Step 1-2)     零代码改动    高收益 (稳定性 + 可观测性)
链路 Profiling (Step 3)  测量脚本     找到真实瓶颈
共享内存 (Step 4a)       1 行 XML     点云零拷贝
RT 隔离 (Step 4b)        10 行代码    控制回路确定性
全量解耦到裸 DDS          ~3500 行改   不推荐
```

---

*最后更新: 2026-02-13*
