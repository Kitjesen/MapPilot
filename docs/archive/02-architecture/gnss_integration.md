# GNSS 融合集成设计 (v2.3.0)

**状�?*: Phase 1+2 实现�?| **目标硬件**: WitMotion WTRTK-980 (UM980 单天�?RTK)

## 1. 背景

LingTu 原有 SLAM �?(Fast-LIO2 + ICP Localizer) �?*室内和中距离 (<500m) 场景**表现良好，但面对户外长距离任务时存在两个问题�?

1. **位姿漂移**：Fast-LIO2 是相对定位，长时间累积漂移无界�?
2. **回环依赖**：ICP Localizer 需要预建地图才能全局定位�?

真实客户场景 (巡检、马拉松、长距离配�? 要求机器人在**公里级开阔户�?*保持绝对定位精度。解决方案：引入 **GNSS 作为全局约束**�?

## 2. 设计目标

| 目标 | 度量 |
|------|------|
| 公里级户外定�?| RMS 误差 < 10cm (RTK Fix) |
| �?GNSS 信号丢失鲁棒 | 信号恢复�?< 3s 重收�?|
| 不破坏现�?Fast-LIO2 | 作为可选增强，未启用时行为完全一�?|
| 多传感器可插�?| 通过 `runtime.registry` 切换后端 |
| 开发成本可�?| 复用 ironoa/um982_ros2_driver，不自写串口 |

**非目�?*:
- 紧耦合 GNSS (伪距 + 多普�? �?放到 Phase 3 �?[GLIO](https://github.com/XikunLiu-huskit/GLIO)
- 双天线航�?�?WTRTK-980 是单天线，航向由 IMU + LiDAR 解算
- �?RTK 的米级精度场�?�?默认配置要求 RTK，单点模式只�?fallback

## 3. 硬件

### WTRTK-980 规格

| 参数 | �?|
|------|---|
| 芯片 | Unicore UM980 (NebulasIV) |
| 通道 | 1408 (BDS + GPS + GLONASS + Galileo) |
| 更新�?| 50 Hz RTK |
| RTK 精度 | 水平 0.8 cm + 1 ppm |
| 接口 | USB-C + XH2.54 6pin UART |
| 协议 | NMEA 0183 + Unicore 扩展 (PVTSLN / BESTNAV) |
| 差分输入 | RTCM 3.x |

### 安装与接�?

- 安装位置：机器人顶部，避开 LiDAR 射线遮挡
- UART: 460800 baud 8N1
- 或直�?USB-C 枚举�?`/dev/ttyACM0`
- dialout 组权限必�?

## 4. 系统架构

```
硬件�?
  WTRTK-980 ──UART/USB──> S100P (/dev/ttyUSB0)

系统服务�?(systemd, 独立�?LingTu):
  gnss.service
    └── ironoa/um982_ros2_driver
          publishes:
            /gps/fix   sensor_msgs/NavSatFix   (50 Hz)
            /gps/odom  nav_msgs/Odometry (UTM) (50 Hz)

LingTu 运行�?(DDS 订阅, 不依�?rclpy):
  GnssModule (src/localization/gnss_module.py)
    ├── cyclonedds subscribe /gps/fix
    ├── Quality filter (sat count, HDOP, fix_type)
    ├── LLA �?ENU conversion (against map origin)
    └── publishes:
          gnss_fix    Out[GnssFix]     (raw, diagnostic)
          gnss_status Out[GnssStatus]  (link health, 2 Hz)
          gnss_odom   Out[GnssOdom]    (ENU, filtered)

融合�?(Phase 2 - TODO):
  fastlio2_gnss backend (src/localization/backends/fastlio2_gnss.py)
    ├── Fast-LIO2 原生 (LiDAR + IMU factor)
    ├── GTSAM PGO 后端
    └── + gnss_odom �?GPSFactor with fix_type-dependent covariance
```

**关键决策**�?

1. **驱动独立 systemd**：不�?LingTu 进程内处理串口，隔离硬件故障；驱�?crash 不会拖垮导航栈�?
2. **DDS 解�?*：LingTu 通过 cyclonedds 订阅，与 rclpy 无关；保�?Python 侧无 rclpy 依赖�?
3. **原点锁定**：首次运行自动记�?GNSS 原点，写�?`config/robot_config.yaml`，保证后续会话地图一致�?

## 5. 数据�?

### GnssFix (raw)

所有来自硬件的 fix，不过滤，供诊断�?Dashboard 显示�?

### GnssOdom (filtered, ENU)

仅当 fix 通过质量门槛后发布，可直接作�?PGO 全局约束。协方差已根�?`fix_type` 缩放�?

| Fix Type | weight | 结果协方�?|
|----------|--------|----------|
| RTK_FIXED | 1.0 | cov × 1.0 (~0.01 m²) |
| RTK_FLOAT | 0.3 | cov × 3.3 (~0.1 m²) |
| DGPS | 0.1 | cov × 10 (~1 m²) |
| SINGLE | 0.05 | cov × 20 (~9 m² 如通过 HDOP 门槛) |
| NO_FIX | 0 | 不发�?|

### GnssStatus (health, 2 Hz)

�?Dashboard GnssCard �?health watchdog 消费�?
- `fix_type`, `num_sat_used / num_sat`, `hdop`
- `age_s`: 最�?fix 年龄 (> 2s 判定 link down)
- `rtcm_age_s`: RTCM 差分老化 (未接收则 99.9)
- `link_ok`: 综合健康

## 6. 坐标�?

| 坐标�?| 描述 | 原点 |
|--------|------|------|
| WGS84 (LLA) | 地球椭球经纬�?| �?|
| ECEF | 地心地固笛卡�?| 地心 |
| **ENU (map)** | **东北天，LingTu 地图坐标�?* | `gnss.origin` (首次 fix 锁定) |
| body | 机器人本体系 | 机体中心，x �?y �?z �?|

ENU 换算采用本地平面线性化 (<10km 半径误差 < 1cm)。超大距离需 UTM 或球面坐标�?

## 7. 配置

```yaml
# config/robot_config.yaml
gnss:
  enabled: false                 # 全局开�?
  model: "WTRTK-980"
  device: "/dev/ttyUSB0"
  baud: 460800
  antenna_offset:                # 天线相对 body 的偏�?(m)
    x: 0.00
    y: 0.00
    z: 0.45
  topic_fix: "/gps/fix"
  origin:
    lat: null                    # 首次运行自动初始�?
    lon: null
    alt: null
    auto_init: true
  quality:
    min_sat_used: 8
    max_hdop: 2.5
    max_age_s: 2.0
    require_fix_type: 1          # 1=Single, 4=RTK_FIXED
    allow_float: true
  fusion:
    backend: "fastlio2_gnss"     # fastlio2_gnss | glio
    factor_weight_fix: 1.0
    factor_weight_float: 0.3
    factor_weight_single: 0.05
```

## 8. 失败模式与降�?

| 场景 | 行为 |
|------|------|
| GNSS 硬件未接�?| `enabled=false`，不影响现有 Fast-LIO2 |
| GNSS 驱动 crash | `gnss_status.link_ok=false`，融合后端忽�?GNSS 输入 |
| RTK 降级�?Float | 协方差放�?3×，位姿继续更新但不收敛严�?|
| 完全�?fix | 融合后端退化为�?LiDAR-IMU (现有行为) |
| 信号恢复 | 自动切回 RTK 权重�? 3s 收敛 |

## 9. 测试策略

### 单元测试 (已实�? 40 passing)

- `core/tests/test_gnss.py`
  - GnssFix 编码/解码 roundtrip
  - GnssFixType 枚举属�?
  - 坐标�?LLA↔ECEF↔ENU 转换精度 (1cm 以内)
  - 质量过滤�?fix_type 权重
  - MapOrigin auto-init
  - WTRTK-980 simulator end-to-end
  - Loss-of-signal 场景

### 仿真测试 (已实�?

- `sim/sensors/wtrtk980_sim.py` 注入合成 fix
- 可配置噪�?+ loss windows 模拟隧道/树荫

### 真机验证 (待做, Phase 3 �?

- [ ] S100P 户外开阔静置测�?(RTK Fix 稳定)
- [ ] 走路径闭�?(看是否漂�?
- [ ] 1 km 长距离走�?
- [ ] 城市峡谷降级测试

## 10. 实施路线

**Phase 1 �?已完�?*:
- GnssFix / GnssStatus / GnssOdom 消息类型
- GnssModule 订阅 + 过滤 + ENU 转换
- WTRTK-980 仿真�?
- 40 个单元测�?
- systemd 服务单元 + 安装脚本
- robot_config.yaml 扩展

**Phase 2 进行�?*:
- Dashboard GnssCard 组件
- fastlio2_gnss 融合后端
- MCP tool: `get_gnss_status`

**Phase 3 规划**:
- GLIO 紧耦合后端
- RTCM NTRIP client (千寻 CORS)
- 多传感器一致性验�?

## 11. 参�?

- [WTRTK-980 产品页](https://www.wit-motion.com/RTK/68.html)
- [UM980 命令参考手册](https://en.uniruntime.com/uploads/file/um980-user-manual-en-r1-1.pdf)
- [ironoa/um982_ros2_driver](https://github.com/ironoa/um982_ros2_driver)
- [zhh2005757/FAST-LIO-Multi-Sensor-Fusion](https://github.com/zhh2005757/FAST-LIO-Multi-Sensor-Fusion)
- [GLIO: Tightly-Coupled GNSS/LiDAR/IMU (HKU)](https://github.com/XikunLiu-huskit/GLIO)
- [GTSAM GPSFactor](https://gtsam.org/doxygen/a03552.html)
