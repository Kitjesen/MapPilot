# MATRiX 功能迁移到 LingTu

更新：2026-10-13。目标是复用已有 Windows 仿真体系，补齐有用的功能，
不创建 MATRiX 专属的第二个 Runtime。这里区分原始实现的静态证据、
LingTu 现有实现、本轮交付和仍需实际运行证明的内容。

## 功能对应与取舍

| 功能 | MATRiX 已确认的机制 | LingTu 对应位置 | 迁移决定与剩余验收 |
| --- | --- | --- | --- |
| MuJoCo 与 UE 联动 | Worker 推进物理并产生姿态快照；UE 更新组件，禁用机器人自身 UE 物理/重力 | `sim/runtime/physics/`、`sim/runtime/visual/RobotSimUE/` | 已有单一物理权威与 follower，不重写；后续测同会话的同步延迟、Reset 和正常终止。 |
| 几何、坐标和模型绑定 | 米到厘米、Y 反转、body/geom/component 的变换补偿 | `sim/catalog/visual_projection.py`、UE `LingTuSimVisual` | 已有统一坐标合同；逐 link 验证引入的机器人，不能以外观正确代替碰撞验证。 |
| 多频率传感器 | 仿真时钟驱动捕获、纳秒时间、代际标记 | `sim/runtime/sensors/runtime.py` | 已有多频率与显式丢帧策略；保持现有 SensorPlan 驱动，不写死官方示例频率。 |
| RGB/Depth 异步读取 | 捕获、渲染命令、GPU readback、编码/发布分开 | UE `LingTuSimSensors`、`sim/adapters/shm/` | 已有异步管线；实际 Hz、延迟、丢帧及 Reset 后旧帧拒绝需同一真实运行证据。 |
| 传感器误差 | 已确认 UE 里程计组件有偏置/高斯误差分支；另有无估计器的状态提取路径 | `sim/runtime/sensors/noise.py`、`mid360.py` | 本轮独立补齐可控 Mid360 距离噪声。不是复刻未证实的 MATRiX LiDAR 噪声算法；真值不加噪。 |
| Mid360 发布 | 26-byte packed ROS2 PointCloud2 CDR，含 x/y/z/intensity/tag/line/timestamp | 现有 typed DDS `LivoxFrame`、`sim/runtime/sensors/dds_adapter.py` | LingTu 保留自己的 native DDS 和 LTU1 进程合同；MATRiX 格式只作为可选外部 Adapter 输入，不能只换 Topic。 |
| 场景碰撞与地形 | Tagged box/cylinder wrapper；动态地形 patch 与物理 Worker 调用路径 | `sim/catalog/`、PhysicsSceneComposer、WorldPackage/ScenarioPackage | 已有静态高度场/代理和行人切片；动态更新、接触、raycast 和 UE 投影一致性待实测。 |
| 动态对象与展示 | 原始运行包具有相关场景内容；完整动态碰撞、发送背压和 streaming 资格未闭环 | `sim/runtime/scenario/`、UE 展示层 | 在现有 Entity/authority/generation 合同内扩展；不把未确认的 MATRiX 行为当作已完成迁移。 |

原始证据在 `third_party/matrix/analysis/v1.1.2/`。具体包括 Worker `Run`
`0xddd8a90`、姿态更新 `0xdd5e850`、PointCloud2 serializer `0xda52350`、
相机捕获 `0x11a595b0`、GPU readback 处理 `0x11a57590`、GPU LiDAR 完成帧处理
`0x11a23680` 和动态地形更新 `0xdd7e140`。这些是静态实现证据，不是性能测量。
点云差异细节见 `third_party/matrix/analysis/v1.1.2/LINGTU-CONTRACT-MAP.md`。

## 消息格式决策

LingTu 的主消息合同由自己的 IDL 和运行时样本决定：

```text
MuJoCo / UE Sensor Runtime
    -> Mid360FrameSample
    -> native typed DDS lingtu.dds.LivoxFrame
```

进程之间的 LTU1 是一个内部有界记录格式。它使用 28 字节记录头和每点
24 字节的 `x/y/z/intensity/offset_time_ns/tag/line/flags` 布局，和当前 C++
`lingtu::drivers::lidar::Point` 一致。`decode_mid360_frame_record()` 已加入
Python 侧，用于录制/回放及跨进程合同测试；session、frame、model/reset
generation 仍由外部 `SensorSampleStamp` 提供，不从字节流猜测。

MATRiX 的 26 字节点格式不进入这个主链路。以后若要接收它的数据，单独建立
一个 MATRiX 输入 Adapter，必须先完成时间参考、frame、line、tag、intensity
和 reflectivity 的语义转换，再构造 `Mid360FrameSample`。转换失败时拒绝进入
导航；主 DDS IDL 不随外部格式变化。

## 本轮：显式 Mid360 距离噪声

生产链路：

```text
SensorRig.configuration.range_noise
    -> CatalogResolver 编译 sensor.plan.json
    -> SensorRuntime 读取不可变 RangeNoiseConfig
    -> Mid360EndpointFactory 创建已有采样器
    -> MuJoCo raycast / 原始碰撞命中
    -> 测量层距离误差
    -> 原有 LivoxFrame typed DDS 编码
```

`noise.py` 只定义配置；`Mid360RaycastExtractor` 拥有采样与显式开关。
没有新增后台进程、噪声管理器、协议类型或第三方依赖。

行为合同：

- 默认关闭。关闭或零强度时，点坐标和原有 native wire bytes 不变。
- 开启后沿原射线方向扰动距离，使用零均值 Gaussian draw；结果饱和在传感器
  最小/最大量程。饱和后的分布不再是无界 Gaussian，不声称是真实 Livox 标定模型。
- 每帧根据显式 seed、session、sensor ID、model/reset generation 和 sequence
  初始化 RNG；同一身份与同一输入可复算，丢帧和开关历史不改变后续帧随机序列。
- 不修改物理命中、碰撞代理、IMU 或 truth odometry；不制造新的命中点。
- 保留点数、方向、offset_time_ns、reflectivity、tag、line 和帧 stamp。
- 样本携带有效配置，SessionSensorRuntime 和 strict sensor summary 披露配置。
  摘要要求强度和 seed 与编译计划一致，开关允许通过明确的 owner-thread API 改变。
- 配置缺字段、拼写错误、非布尔开关、非有限/负强度或非法 seed 会被拒绝。
- 当前只支持 Mid360 距离噪声。IMU 漂移、图像噪声、丢点、GPS 等不是本轮交付。

### 配置和使用

在一个新的/实验用 SensorRig 的 Mid360 声明中加入：

```yaml
configuration:
  raycast_frame: lidar1_link_site
  range_noise:
    enabled: true
    stddev_m: 0.02
    seed: 17
```

`stddev_m` 的单位是米，这个示例表示名义 2 cm 标准差，不是标定结果。
不要修改已经冻结并用于证据验收的 Rig/Bundle。独立示例已放在：

- `sim/packages/sensor_rigs/doso/thunder_v4/mid360_noise/sensor-rig.package.yaml`
- `sim/sessions/examples/thunderv4_mid360_noise/session.yaml`

该示例只有物理与 Mid360，没有控制器、UE 或完整导航传感器组。它不会启动机器狗运动。
在仓库根目录生成 Bundle（只编译配置，不启动仿真）：

```powershell
& .venv/Scripts/python.exe -m sim.catalog resolve `
  sim/sessions/examples/thunderv4_mid360_noise/session.yaml `
  --output-dir thunderv4-mid360-noise
```

默认写入 `build/simstudio/thunderv4-mid360-noise/`；相对输出路径以 CLI 的
`artifact_root` 为基准，不是仓库根目录。

运行时调用已绑定的 `Mid360RaycastExtractor.set_noise_enabled(False/True)`。
必须由 Sensor Runtime 所属线程在采样之间调用；已经生成的帧不受后续开关影响。
目前没有新增 HUD 按钮或远程切换命令，不宣称 UI 已集成。

### 验证范围

`tests/sim/test_mid360_range_noise.py` 覆盖编译器到采样器的配置链路、默认行为、
开关、种子/代际隔离、跳帧复现、方向/量程/点属性、证据披露及非法配置。
这些是本地合同测试；未运行 UE、原始 UeSim、真实 DDS/SLAM 或运动任务。
开启噪声的满点数吞吐量和完整 recorder/replay/UI 仍需后续验收。

## 接下来按纵向切片推进

1. **点云互操作**：先固定已知样本与明确的 frame_start/时间单位，再解包 26-byte
   点记录并转换到 LivoxFrame。CPU semantic intensity/tag 不直接冒充反射率/Livox tag。
   缺少时间含义时拒绝自动进入导航；不改变 typed DDS 主合同。
2. **一个机器人/道具的资产闭环**：通过现有 importer 生成标准包，验证许可、单位、
   Frame、视觉材质与 MuJoCo 碰撞代理。XGB 已能在 Windows MuJoCo SDK 编译，但
   尚未证明控制、UE 导入或运动，不能把编译成功当作可用机器人。
3. **同会话联动验收**：机器人/动态对象状态、物理接触、LiDAR 命中和 UE 投影一致；
   再测 RGB/Depth 实际频率、延迟、Reset 与正常停止。
4. **交付体验**：在已有会话与资产管理入口接 UI、打包与 Pixel Streaming。
   不另起产品生命周期，不让展示层成为第二个物理权威。

功能实现、离线测试、UE 实际运行和真实机器人验收分别报告，不能互相替代。
