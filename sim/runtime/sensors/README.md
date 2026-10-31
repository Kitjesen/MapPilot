# 传感器接入与巡检验证状态

更新：2026-10-23。**当前不能宣称全部传感器已在同一 Factory Product 联合验收。**
最新选择器增量：Thunder `navigation/headless_navigation@1.1.0` 已显式声明九类条目，
新增投影／软件云台、GNSS 模拟定位启动配置；四个新增流默认关闭，原五流不变。
World 没有 `measurements` 时，热像／语义／声学显示缺失原因且不能启用，不伪造数据。
详细边界与验证见[传感器选择](../../../docs/design/sensor-selection.md)。
最新增量：UE 原生“传感器”页已通过实际 HUD＋HTTP＋原生控制 DDS＋MuJoCo 的
有界诊断。30 项原生界面测试、97 项启动／工厂绑定测试通过；实际控件联测得到
9 条匹配回执、23 份原始 NPZ 并自然关闭。详见本节末尾证据入口；不是工厂巡检验收。
热成像、声学、语义三种新增模态已实现统一原生远控和原始归档，真实 HTTP 路由 → native
DDS → MuJoCo 受控热像 fixture 已通过；RGB／Depth／LiDAR／IMU 尚未全部统一远控。
下表中的频率是配置值，不是本轮工厂实跑测量值。当前 Factory Environment Collection
尚未成为同源碰撞／导航世界，因此没有在新工厂中宣称联合传感器验收通过。

## 当前实际配置

源配置：[`Thunder navigation 1.1.0 rig`](../../packages/sensor_rigs/doso/thunder_v4/navigation/v1.1.0/sensor-rig.package.yaml)。

| 类型 | 当前接入 | 生产者／接口 | UE rig 配置 | 本轮验证边界 |
| --- | --- | --- | --- | --- |
| RGB | 已有采集和传输实现 | Unreal Camera → camera_shm | 640×480，30 Hz | 合同与证据结构测试通过；本轮未启动 UE 相机／新工厂实跑 |
| 深度 | 已有米制深度实现 | Unreal Camera → camera_shm，wire 16UC1，请求编码可为 32FC1 | 640×480，30 Hz | 编码、相机证据和绑定测试；不等于已参与巡检结果判定 |
| MID-360 LiDAR | 已有扫描图案、MuJoCo raycast、LivoxFrame | Physics → typed DDS | 10 Hz | 调度、提取、绑定、噪声与证据合同测试；本轮无新工厂扫描 |
| IMU | 已有姿态／角速度／加速度提取 | MuJoCo sensor → typed DDS | 200 Hz | 身份、时间、坐标与提取合同测试；非硬件精度验收 |
| 真值里程计 | 已有；额外辅助流，不属于 GPS | MuJoCo truth → typed DDS | 100 Hz | 精确时间／generation 绑定；不能冒充 GNSS 或 SLAM 定位 |
| GPS／GNSS | 新增可选实验包、rig、编译输出和端点；默认导航未启用 | MuJoCo 位姿 → GNSS 测量 → native DDS Fix/Status | 独立实验 rig 10 Hz | 已用现有 Thunder 实际物理快照和 DDS 订阅器验证 FIX／NO_FIX／reset；不是正式 Product／工厂巡检验收 |
| 热成像 | 已有物理逐像素测量、可选包、原生远控与原始归档 | MuJoCo nearest-geom → float32 Kelvin + valid mask → ThermalImage | 尚未绑定 UE 默认 rig | 受控模型及真实 HTTP→DDS→MuJoCo 控制 fixture 通过；非 Factory Product/UE UI 验收 |
| 声学 | 已有声源／麦克风测量、可选包、共享原生远控与原始归档 | 直接传播／延迟／遮挡 → Pa waveform → AcousticBlock | 尚未绑定 UE 默认 rig | 模型与波形合同、实际浏览器 Pa 显示和设置查询通过；不是工厂同跑或持续吞吐证明 |
| 语义／实例 | 已有物理标签图、可选包、共享原生远控与原始归档 | 物理 geom 标签 → SemanticImage | 尚未绑定 UE 默认 rig | 采样／投影、浏览器像素 ID、关闭／重启检查通过；不是识别能力或 Factory/UE 验收 |

代码入口：

- [`factory.py`](factory.py)：真值里程计、IMU、MID-360、可选 GNSS 的唯一端点匹配与绑定。
- [`runtime.py`](runtime.py)、[`session.py`](session.py)：按模拟时钟调度、运行与证据。
- [`extractors.py`](extractors.py)：严格从物理快照提取，不回写动力学真值。
- [`UE CameraCaptureSubsystem`](../visual/RobotSimUE/Plugins/LingTuSim/Source/LingTuSimSensors/Private/LingTuSimCameraCaptureSubsystem.cpp)：RGB／Depth 渲染与 SHM。

## 2026-10-21 后续：GNSS 实验测量链

已新增 [`gnss.py`](gnss.py)：显式 WGS84 原点／地图朝向、天线安装偏置、同代物理快照
到 `GnssFix` 的测量模型，以及 `SensorRuntime` 的可选计划读取／端点输出／证据记录。
owner thread 可通过 `SessionSensorRuntime.set_noise_enabled(sensor_id, bool)` 控制噪声，
通过 `set_gnss_fix_available(sensor_id, bool)` 控制明确的场景失锁。未绑定或不支持的
传感器会报错，不假装所有传感器都有开关。参数只影响后续测量；默认无定位，不编造
卫星数或 RTK，不改物理真值。

本轮已经补齐 Catalog SensorPackage／rig／schema／编译输出、Python 原生端点以及
`lingtu_gnss_publisher`。专用示例是
[`thunderv4_gnss/session.yaml`](../../sessions/examples/thunderv4_gnss/session.yaml)，
使用 **synthetic WGS84 零原点**、默认 `NO_FIX`、headless、无驾驶控制器，不是实际工厂
的地理标定。它不会替换默认导航 rig；无 GNSS 的原有计划保持不变。

可选 `PlayableRuntimeConfig.gnss_publisher`／`--gnss-publisher` 只在显式提供原生
程序路径时注册 GNSS 端点。它不是正式 `config/system` ProductControl feeder 的
启用开关；正式 Product 的可选接线已在下节补齐。**Gateway／UI、定位融合与新工厂
巡检联合实跑仍未完成**。

实际测试已运行当前 Thunder v1.0.3 MJCF，经 MuJoCo 进程真实快照、SensorRuntime、
原生 DDS 发布器和独立 DDS 订阅器，收到 3 个 fix 与 3 个 status：FIX → NO_FIX → reset
后 FIX，时间戳为 0／100000000／0 ns。物理只推进 0.1 秒，没有驾驶或 UE；不能当成
10 Hz 持续吞吐或机器人运动验收。证据目录：
`build/inspection-tests/20261021/gnss-native/gnss-native-sb70hasf/`，其中
`observed.json` 是实际 DDS 接收值，`physics-and-sensors.json` 保留来源和有效配置，
`result.json` 记录退出码。所有本测试拥有的进程自然关闭。

状态证据明确标 `EXPERIMENTAL`；流 ACTIVE 只表示端点输出，不等于 `NO_FIX` 可以导航。
模型不模拟卫星、遮挡、多径、RTK 或硬件标定。GNSS 的模拟时钟不能当成现实 UTC。
对照 MATRiX 的发现、可复算原始证据和剩余接线见
[`SENSORS_20261021.md`](../../../research/matrix/SENSORS_20261021.md)。

## 正式 Product feeder 的 GNSS 可选接线

新增 [`headless_gnss rig`](../../packages/sensor_rigs/doso/thunder_v4/headless_gnss/sensor-rig.package.yaml)
和 [`Product session`](../../sessions/products/doso/thunder_v4/gnss.yaml)。后者复用旧
`factory_workshop@2.0.0`，**不是新 Fab 工厂**；保留原五条流及频率，追加 10 Hz GNSS。
原点仍明确是 synthetic zero，缺省 `NO_FIX`／噪声关闭。

正式路径复用 ProductControl 已管理的 `imu_publisher` 进程及认证 LTU1 连接：

```text
RunPlan GNSS declaration → formal feeder live MuJoCo body + simulation time
  → canonical GnssExtractor → LTU1 type 8 / LTGN payload
  → owned imu_publisher → rt/gnss/fix + rt/gnss/status
```

不增加后台子进程、第二套控制器或 Product 启动器。第一条合法 GNSS 到达前，不创建
GNSS DDS writer。认证端检查 Product 会话；GNSS 不经过 IMU 的 epoch 时间换算或
replay restamp。晚到的采样只使用当前物理状态与当前模拟时间，记录错过的采样期限，
不伪造过去的位姿。正式 feeder 同进程时间倒退会报错；新 Product 会话重新初始化。

只编译检查、不启动程序：

```python
from lingtu.assembly.compile_plan import compile_run_plan

plan = compile_run_plan(
    "inspection", "sim", robot="doso/thunder_v4",
    env_config={
        "backend": "mujoco",
        "session": "sim/sessions/products/doso/thunder_v4/gnss.yaml",
    },
)
```

实际运行仍经 `python -m lingtu.control`，使用已有 `--sim-session` 参数选择该 YAML，
并满足 inspection 的真实已保存地图、原生构建和启动前提。本轮没有启动整套 Product。
不提供虚构地图名来绕过前提。默认 session 不变，选 GNSS 示例不等于启用 GNSS 融合。

[`FormalGnss`](../../scripts/mujoco/formal_gnss.py) 提供 owner-thread
`set_noise_enabled(bool)`／`set_fix_available(bool)`，配置通过冻结的 SensorPlan 输入；
尚未增加远程或 UI 开关。`mujoco_feeder.status.json` 的可选 `gnss` 字段披露采样次数、
错过期限、源时间、有效设置与 `EXPERIMENTAL`。这些是采样／排队信息，**不是 DDS
订阅器接收 ACK**，也不代表有可用导航定位。

Python 编码 → 实际原生 DDS 互操作证据：
`build/inspection-tests/20261021/formal-gnss/formal-gnss-wire-h5s80m5k/`。
`received.json` 包含 5 fix + 6 status；输入为受控位姿，不是整场仿真。首次发现期间
允许 best-effort 数据丢失，不宣称首包必达。两个测试进程正常 exit 0。

复审发现并修正了 post-step 位姿滞后一物理步的问题：MuJoCo `mj_step` 积分后，
原 `xpos/xquat` 缓存可能未更新。新增 `MuJoCoEngine.get_body_pose()` 在原数据锁内调用
`mj_kinematics`，仅在 GNSS 到期采样时读取刷新后的位姿，不修改 qpos/qvel 或推进时间。
真实 MuJoCo 3.10、Euler 5 ms、平移＋旋转＋非零天线偏移回归先失败再通过；原失败报告
`formal-gnss-poststep-red.xml` 与通过报告 `formal-gnss-poststep-green.xml` 均保留。
依据 [MuJoCo API](https://mujoco.readthedocs.io/en/3.10.0/APIreference/APIfunctions.html#mj-kinematics)。

另一次互操作测试的失败 `formal-gnss-native-final.xml`、目录 `formal-gnss-wire-9yixti_1`
保留：两路连续 DDS 流收到的数量可能暂时不同，原 fixture 错误要求恰好各 5 条。
observer 默认仍为 exact；本连续流测试显式选 minimum，保留全部记录并逐条核对唯一、
递增时间及发送内容。新通过报告为 `formal-gnss-native-streaming-green.xml`，没有放宽
生产协议、伪造补包或把不相等计数写成相等。

## 两条相机链路不能混淆

`config/system/envs/sim.yaml` 的正式 ProductControl MuJoCo 路径使用 native
camera publisher；`sim/sessions/products/doso/thunder_v4/nav.yaml` 当前绑定另一份
`factory_workshop@2.0.0` 与
[`headless_navigation rig`](../../packages/sensor_rigs/doso/thunder_v4/headless_navigation/sensor-rig.package.yaml)：
其 RGB/Depth 由 MuJoCo 渲染，640×480，**1 Hz**，Depth 为 16UC1。

上述 1 Hz 与 UE navigation rig 的 30 Hz 是不同执行路径。不能将 headless Product
就绪、已有 UE 相机代码或本轮合同测试，拼成“新 Fab 工厂中 UE 相机已与正式原生巡检
同一次运行联通”的证明。该联合链路仍需在选定新世界、同一任务中验收。
不通过把 UE 30 Hz 静默改成 1 Hz 来过性能门。

## 三种新增模态的原生远控与原始归档

当前统一范围为 thermal／acoustic／semantic。可选实验包和
[`headless_multimodal rig`](../../packages/sensor_rigs/doso/thunder_v4/headless_multimodal/README.md)
保留原五路并追加三路；不自动启用默认 Product，也不补造 World 属性。
完整 rig／SensorPlan 严格 schema 已通过，原两处 xfail 已移除。Mid360 overlay 只允许
布尔值且只属于 Mid360；MuJoCo depth 保留原有 `16UC1` plan，UE 仍需合法请求编码和
`0.001` depth scale，未向已有 plan 添加默认字段。

实际控制路径为 Gateway HTTP 路由 → Host `sensor.controls` → native DDS → 既有
MuJoCo 采样 owner → 匹配的结果 ACK。代码入口：

- [`SensorControlRequest/Result`](../../../src/runtime/contracts/sensor_control.py)：请求、来源和 generation 身份。
- [`commands.py`](commands.py)：在既有采样边界应用操作，不增加进程或运动控制 owner。
- [`control_bridge.py`](../../../src/drivers/sim/measurement/control_bridge.py)：有归属的 ctypes/native DDS 端点。
- [`control_module.py`](../../../src/drivers/sim/measurement/control_module.py)：匹配实际 source ACK，发布有效设置。
- [`HTTP route`](../../../src/gateway/routes/sensor_control.py)：`POST /api/v1/sensors/{sensor_id}/control`。

请求体必需 `request_id`、`expected_product_session_id`、`operation`，以及对应的 `params`：

| operation | params | 语义 |
| --- | --- | --- |
| `query` | `{}` | 读取 owner 实际状态 |
| `enabled` | `enabled: bool` | 控制后续采样／发布，不移动机器人 |
| `rate` | `rate_hz: number` | 在 producer 和调度允许范围内改频率 |
| `noise` | `enabled, unit, stddev, seed` | 显式单位和噪声配置；语义标签不可启用噪声 |
| `noise_enabled` | `enabled: bool` | 保留参数，仅切换支持的噪声开关 |
| `recording` | `enabled: bool` | 控制后续原始样本归档，必须有 recording owner |
| `gimbal` | `pan_deg, tilt_deg` | 仅支持的相机投影，麦克风不支持 |

成功响应来自 source 的 `SensorControlResult`，包含匹配的请求／Product／source session／
sensor／model/reset generation、`applied_sim_time_ns`、`applied`、`reason` 和有效 `settings`。
不能从 HTTP 请求已发送、publisher 排队或 bridge setup 推断已生效。最多等待 2 秒；
超时返回 504，表示结果未知，不表示 source 一定未应用。source 拒绝或身份／请求冲突为
409，未知传感器为 404，无控制模块为 503，非法参数为 422。控制 ACK 不是测量数据订阅
ACK，也不是巡检采集成功或原生任务推进许可。旧 RGB／Depth／LiDAR／IMU 尚未全部
接入这条统一远控链，不能因它们仍在 rig 中就宣称支持全部操作。

[`MeasurementArchive`](../recording/measurement_archive.py) 已实现有容量上限、显式归属的
raw NPZ 归档，不新建 worker。正式 feeder 可选测量链在当前 run 的 `measurements/`
使用它；只在录制开启后写样本，完整 raw 文件写入后才发布 `index.jsonl` 条目，关闭状态
写入 `recording.json`。文件保存原数组／mask、单位、校准、来源身份及源时间，不是伪彩图
或播放音效；容量耗尽明确报错。录制开关 ACK 不等于已有样本完成持久化，须核对索引和
归档摘要。

实测证据：
[`sensor-control-s07ezbq8/result.json`](../../../build/inspection-tests/20261022/sensor-control-s07ezbq8/result.json)
及其 [`recording/`](../../../build/inspection-tests/20261022/sensor-control-s07ezbq8/recording/)。
测试通过真实 FastAPI HTTP 路由（TestClient）、真实 native DDS bridge 和实际 `mj_step`
推进的 MuJoCo fixture，对 `robot.thermal` 验证七种操作的真实 ACK。记录中
`passed=true`、`factory_product=false`、`source_errors=[]`、`source_thread_closed=true`；
归档 `state=closed`，1 个 2530-byte NPZ，含 2×2 float32 Kelvin 与 bool valid mask。
这不是网络部署吞吐测试、64×48 三模态 rig 联合实跑或正式 Factory Product 巡检。
选定 Fab 世界的同源碰撞／设备／声源／地图绑定、UE 实时数据／控制 UI 和真实 Product
原生到点／停稳／采集／结果回执同跑验收仍未完成；完整功能未验收。

## 受限 preview 与 Web 工作台状态

[`GET /api/v1/measurements/{sensor_id}/preview`](../../../src/gateway/routes/measurements.py)
已实现，投影代码为
[`measurement_preview.py`](../../../src/gateway/services/measurement_preview.py)。
它返回原 `measurement` 身份／时间／状态与 `display`，并标 `display_only=true`：

- thermal 保留 K 值，semantic 保留 class/instance IDs；按原像素最近邻选取，最多
  192×144，返回原始行列索引和 valid mask，无效或非有限值为 JSON `null`，不插值补值。
- acoustic 为所有通道各返回最多 256 bins 的 min/max Pa 包络与有效比例，并给出
  sample rate、源样本总数及 bin 起止 sample；由样本区间得到相对采样时间，不冒充
  全量波形或重新生成采样时钟。无有效数据的 bin，其 min/max 为 `null`。
- preview 不是原始测量替代物；完整 raw NPZ 下载与归档保持不变。未知、缺样本或
  不新鲜数据分别保留 404／503／409 语义，不用假数据占位。

当前源码已有
[`MeasurementInspector`](../../../web/src/components/MeasurementInspector.tsx)，接入实际
[`InspectionWorkbench`](../../../web/src/components/InspectionWorkbench.tsx)，包含 preview、
raw 下载和 acknowledged controls；这不等于 UE UI 或浏览器端到端验收。
最终 preview API 25 项通过；控制／数据／巡检接线联合回归 391 项通过，相关 12 个源码
文件定向 mypy 通过。Web 测量与既有工作台共 47 项测试、ESLint 和正式构建通过；保留
既有 Three.js 大 chunk 警告。计数是不同范围的检查，不相加成完整产品验收。
ACK/list revision reducer 已关闭陈旧本地 ACK 永久覆盖新状态的问题；preview 接收年龄
额外计入单调时钟测得的请求耗时，跨越 TTL 的 200 响应不能重新显示为新鲜帧。

实际浏览器证据位于 `build/inspection-tests/20261022/measurement-browser-02/` 和 `-03/`：
显示 300–400 K 热图、8 kHz／80 样本声压、约 ±0.04998 Pa 的测试声源结果，以及中心
像素 class=7、instance=42 的语义真值。热像录制通过真实 native DDS ACK 开启，归档
187 帧、477435 bytes，最终 closed。语义关闭后清空预览，重新启用后新帧恢复。
浏览器查出了单样本包络零长度 SVG 线不可见的问题，已用圆端点修复并截图复核；没有
放大或改写声压数据。`measurement-browser-01` 是先前达到 420 秒上限而关闭、未完成
浏览器操作的尝试；`-02` 也到达时限，不包含声学录制成功回执，不能补写成功。

这些是**隔离 MuJoCo 测量夹具的浏览器验证**：控制走 native DDS，测量数据在进程内
解码 LTU1；不是完整 Host 数据 DDS 部署，更不是工厂 Product 或 UE 内嵌界面验收。
夹具与临时 Vite 服务均已关闭。实际工厂绑定、正式分辨率持续吞吐和 UE UI 仍未完成。

preview/API/Web 全范围独立复审已 **APPROVE / CLEAR**；这不包括未完成的 UE／Factory
Product 验收。截图路径和全部残余项记录在
`build/inspection-tests/20261022/multimodal-delivery-increment.json`。

## 2026-10-22 声学长信号热路径

浏览器夹具暴露了实际计算成本：每个短块重建 AcousticSource 时复制／校验整段波形，
插值又为每通道生成完整 `arange/pad`。现用显式验证、不可变 bytes 支撑的 `AcousticClip`
在场景内复用快照，只采当前块相邻样本。普通数组输入仍完整验证并拥有独立副本；
没有根据数组只读标记跳过验证。源位置和 stamp 仍逐块更新。

同一 420 秒 × 8 kHz 波形、双通道、80 samples/block、20 次 `AcousticScene.sample`
（不含初始化）的 agent 实测：总耗时从 1.0142576 s 到 0.0073931 s，平均 50.71288 ms
到 0.369655 ms。原初优化版 0.006479 s 不是最后所有权兼容修复后的复测值。
这只证明局部热路径提速，不证明整套 Product 的 RTF 或浏览器帧率。
声学 204 项、与 extractor/configuration/session/HTTP 等相关集成共 333 项通过，
后者报告 `build/inspection-tests/20261022/acoustic-performance-integration.xml`；范围重叠，
不相加。两源码文件定向 mypy 和四文件 Ruff 通过。有限支撑、起始前静音、最后样本
渐降至零、分数索引、传播延迟、噪声／seed、stamp 与输入所有权合同保持不变。
此优化已完成独立复审 **APPROVE / CLEAR**，不扩展为整机实时率结论。

## 噪声控制现状

- 当前 SensorPlan 的显式噪声配置包括 MID-360、可选 GNSS 和三种新增测量；MID-360 使用
  `RangeNoiseConfig(enabled, stddev_m, seed)`。缺省关闭。
- `Mid360RaycastExtractor.set_noise_enabled(bool)` 可在 owner thread 切换；参数和 seed
  保留，生成的样本／证据披露实际配置，不改物理命中与碰撞真值。
- 这是实验性、限幅高斯距离误差，不是校准后的完整 Livox 噪声模型。
- [`mid360_noise rig`](../../packages/sensor_rigs/doso/thunder_v4/mid360_noise/sensor-rig.package.yaml)
  是单雷达实验配置，**不是可替换完整导航 rig 的“总噪声开关”**。
- GNSS 实验模型有自己的显式噪声开关与 seed，并记录有效配置；尚未接入默认 Product。
- 热成像／声学／语义的新端点已实现 `configure_noise(MeasurementNoise)`，通过现有
  `SessionSensorRuntime` 统一启停、频率、噪声、状态和录制。操作与采集用实例锁串行化，
  允许既有主线程 prepare → 后台线程 advance，不另起线程／进程 owner。三模态已有上节
  的 acknowledged native 远控；语义标签噪声必须关闭，不能把支持配置接口等同于可加噪。
- RGB、Depth、IMU 尚未全部接到这些可调参数。兼容引擎的 IMU／雷达
  配置还存在其他噪声参数，正式 headless feeder 也传入 `add_noise=True`；不能将其与
  此处 SensorPlan 缺省关闭混为同一套语义。本轮未改这些默认值。
- 待统一的是每种传感器明确的开关、参数、seed、记录和回放语义，不是一枚显示按钮。

## 巡检实际需要什么

第一版三点巡检优先使用 RGB、对应时间的真实位姿和原生到点／停稳状态。
LiDAR、IMU 与定位支撑导航；Depth 的存在不代表已经保存为巡检测量证据。
`InspectionEvidenceModule` 当前存储的主证据是 RGB、位姿、时间和可选检测／设备观察，
`capture:overview` 不自动得出“设备正常”。新增热／声学原始 artifact 与显式阈值评估，
正式 `InspectionEvidenceModule` 已接上热／声学／标签输入、传感器选择、身份与时间校验，
并参与既有采集请求／结果回执。受控生产者到持久化测试通过；尚未在新 Fab 工厂完成原生
自主到点同一次运行验收，不能把合同测试当成该场景已贯通。

采集必须拒绝过期照片、过期位姿或超出配置的 RGB/odom 时间偏差。低频相机使用图像
源时间附近的真实位姿，不拿最新位置替换拍照时的位置。原生任务完成、证据完整和
人工接受是三个不同结论。

### 多模态交付目标（进行中，不再以点温度代替热成像）

完整计划：[multimodal-sensors.md](../../../docs/design/multimodal-sensors.md)。

- [x] 整幅 Kelvin 图、相机标定／安装姿态、近表面遮挡、未知／无效像素。
- [x] 灰体辐射 apparent/corrected 区分、显式温度／发射率、K 域噪声／seed。
- [x] Pa 波形、源到麦克风距离／传播延迟、标定、遮挡失效、采样时钟、噪声。
- [x] 物理语义／实例真值、云台／鱼眼／全景投影；不是算法识别结果。
- [x] 现有 Session 控制、NPZ 原始录制／回放；巡检 store 原子保存测量＋RGB＋pose。
- [x] Gateway 按 sensor_id 下载原始 measurement，双传感器不误选首项。
- [x] 独立 ThermalImage／AcousticBlock／SemanticImage IDL 与 native DDS 互通。
- [x] Catalog／世界测量属性／RunPlan 可选声明、正式 feeder 采样和 Host typed measurement 接线。
- [x] 巡检测量输入／选择／原始持久化与判定代码；Gateway 实时状态及原始下载接口。
- [x] 三种新增模态的可选实验包和完整八流 rig，严格 schema 无 xfail。
- [x] 三模态原生远控与 raw archive 实现；thermal 的真实 HTTP→DDS→MuJoCo 受控 fixture。
- [ ] RGB／Depth／LiDAR／IMU 的全部统一远控操作。
- [ ] 选定工厂设备／声源绑定、正式 Factory Product 实际激活验收。
- [ ] 正式 Product 原生到点／停稳／测量采集／结果回执同跑；UE 实时数据与控制 UI 验收。
- [ ] 选定 Fab 生产线的同源碰撞／设备／地图绑定，以及同一次运行的联合验收。

热模型明确是宽带灰体实验仿真，不是校准 LWIR 或热传导求解器；声学是冻结块几何的
直接传播，不声称反射／绕射／Doppler。未知结果保持不可判定，不补正常温度或静音。
RGB 是佐证，不是温度。深度 `32FC1` 仍是米，声学不能伪装成 Image。

实际证据：`build/inspection-tests/20261021/multimodal-physical-01/`：320 K／0.0704 Pa
正常、390 K／0.2347 Pa 超阈值、遮挡两路无效；6 份测量＋同场景 RGB＋位姿持久化。
这是 **controlled physical data，不是 Fab Product**。585 项组合回归通过，独立增量复审
APPROVE/CLEAR；全导入类型检查仍有 15 项既存范围外错误，未宣称全仓类型检查通过。

原生链：`build/inspection-tests/20261022/measurement-wire-kgg8_qvv/` 为 Python → LTU1
→ 实际 native DDS 三种消息的受控数组互通，3 条实际接收，发布器／观察器均 exit 0。
`measurement-native-tests-green.xml` 为 4/4 原生测试；首次失败报告保留。初始 IDL
`sequence` 保留字已改为 `sample_sequence`；测试中重复 sensor_id 也已纠正，不放松生产
身份校验。协议：[measurement-wire.md](../../../docs/design/measurement-wire.md)。

2026-10-22 接线增量：Catalog/schema 110 passed；正式 feeder／巡检／Gateway／装配
215 passed；编译后 inspection Host 重建及 API 合同 23 passed。报告分别为
`measurement-catalog-schemas.xml`、`measurement-production-integration.xml`、
`measurement-compiled-host-gateway-green.xml`，位于 `build/inspection-tests/20261022/`。
计数有重叠，不求和。原生 DDS 实际互通与本轮离线装配验证分开记录。

实时读取：`GET /api/v1/measurements`、`/{sensor_id}`、`/{sensor_id}/raw`、
`/{sensor_id}/preview`（后三者接在前一
路径下）。返回源身份、单位、校准、有效比例／时间状态；raw 为含原数组和 mask 的 NPZ。
这些 GET 不是 UE 可视化或控制入口；远控使用上节独立的 POST。部分发送失败现在关闭共享连接，迟到帧使用与 RGB 一致
的观测时刻。正式声学要求采样网格对齐物理步长，不兼容配置在建引擎前拒绝；无隐式重采样。

## 2026-10-21 测试记录

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 巡检服务、事件、报告／引导、证据模块／存储／桥、UE 任务绑定 | 178 passed | `build/inspection-tests/20261021/inspection-contracts-green.xml` |
| SensorRuntime、绑定、端点、跨流证据、MID-360／噪声、IMU、UE 相机证据合同 | 121 passed | `build/inspection-tests/20261021/sensor-contracts.xml` |
| 当前源码原生 inspection C++ 测试目标 | 4/4 passed | `build/inspection-tests/20261021/native-inspection-with-three-point.xml` |
| GNSS 新增坐标／安装／噪声／失锁／身份／端点／证据合同 | 29 passed | `tests/sim/test_gnss_sensor_runtime.py` |
| 加入 GNSS 后的上述传感器组合回归 | 150 passed | `build/inspection-tests/20261021/sensors-with-gnss.xml` |
| 本轮 Catalog／端点／session／可选 playable 路由联合回归 | 217 passed，30 skipped，1 deselected | `build/inspection-tests/20261021/gnss-combined.xml`；30 项 schema 测试因该 venv 缺少 jsonschema 跳过，原生实测单独执行 |
| GNSS Catalog／schema 补跑 | 68 passed，0 skipped | `build/inspection-tests/20261021/gnss-catalog-schemas.xml`；使用本机已有 Miniconda Python 3.13/jsonschema，覆盖上述 30 项跳过；与联合回归有重叠，不叠加计数 |
| 当前 Thunder 真实 MuJoCo → GNSS → native DDS | 1 passed | `build/inspection-tests/20261021/gnss-native-thunder.xml`；3 fix + 3 status，非 UE／驾驶测试 |
| 原生 GNSS 协议／DDS 进程测试与 IMU／truth-odom 回归 | 2/2 + 4/4 passed | `build/windows-native-dds-adapter` 的 CTest；新增 observer 匹配握手后 helper 1/1 重跑通过 |
| 正式 feeder／控制／编码／认证客户端／证据回归 | 201 passed，0 skipped | `build/inspection-tests/20261021/formal-gnss-regression-final.xml`；含真实 MuJoCo post-step 测量测试、编译后的 inspection 配置到 feeder 合同 |
| Product GNSS opt-in 配置与 Catalog／编译器相关回归 | 77 passed | `test_formal_gnss_plan.py` + Catalog + 两组 compiler 测试；包含 Windows/Linux 进程配置等价，非 Linux 运行证明 |
| 正式 GNSS 原生协议及 publisher 回归 | 8/8 passed | 新原生 GNSS LTU1／认证端点与原协议测试；helper 调整后相关进程 3/3 再通过，见 `formal-gnss-native-ctest.xml` |
| Python 正式 GNSS 编码到实际 DDS | 1 passed | `formal-gnss-native-streaming-green.xml`；5 fix／6 status，各记录均核对 |

独立代码复审 APPROVE／CLEAR，未发现本切片具体阻断项；范围不包含正式 Product 激活。
主线 Mypy（含所依赖的 evidence/contracts/extractors/runtime）8 个文件通过；独立复审的
另一轮 Mypy 2.1.0 出现内部错误，不能把该轮计为类型检查通过。

15:12:33 +08:00 进程复核未见 UE／UBT／UAT／ShaderCompileWorker／dotnet／cl／link、
本次 MuJoCo、GNSS publisher／reader 或 IMU／truth-odom publisher。
仍观察到 MSBuild.exe PID 8852（15:02:17 创建，父 PID 61960 已不在，`nodeReuse:true`）；
未确认该复用节点的独占归属，未操作它，因此不宣称“整台电脑所有构建进程全清”。

正式 feeder 增量最终源码复审 APPROVE／CLEAR；上述 post-step 阻断已修复。
相关 Python Ruff 通过，新增／修改 helper 三文件 Mypy 通过。对整个 `formal_feeder.py`
另跑类型检查仍有 5 项诊断，位于本次未改动的 driver poll、policy profile、camera 与
registered-cloud 代码段；没有将整文件类型检查标绿。既有 cacheprovider 警告仍在。
18:28:46 +08:00 最终进程名扫描为空，包括之前的 MSBuild 复用节点；本轮没有关闭
非本任务进程。扫描范围与时刻保留于 `formal-gnss/postflight.json`。

首轮巡检 Python 检查为 177 passed / 1 failed：上一轮运行入口新增制作插件排除参数后，
UE inspection binding 的精确 argv 预期尚未同步。已补齐预期并重跑通过，没有放松任务绑定。
首轮失败 XML 保留为 `inspection-contracts.xml`。

新原生测试 `lingtu_inspection_three_point_capture_test` 使用真实 C++ Executor，
导航到达、停稳采样和证据回执是受控输入，坐标为明确的 synthetic-contract-map：

- 三点各有停稳、采集握手和持久化成功回执后才推进。
- 同一执行器三次干净重跑，九个 action request 身份不重复。
- 上一点／上一轮迟到结果不能推进当前点；第二点失败不能访问第三点。

这些测试不是新工厂中的物理行驶、实时 DDS 频率、模型识别准确率或实机巡检证明。
构建的 D9025 `/DNDEBUG` 被 `/UNDEBUG` 覆盖提示来自既有 Release 测试策略，确保 assert 生效；
Python 仍有禁用 cacheprovider 时的既有 `cache_dir` 配置警告。

## UE 测量页与工厂设备切片：2026-10-22

运行时菜单新增“传感器”。热像可查看 K／°C、像素坐标和无效掩码；语义页明确显示
class／instance 真值，不冒充算法识别；声学页显示 Pa 包络、采样率、块内时间及有效比例。
启停、查询、频率、噪声、种子、录制与适用云台参数使用同一个真实源端回执接口。
参数输入是草稿，回执前不表示应用；尚无首帧也可查询已声明设备。

查看测量时释放手动驾驶输入并保持采样；原生自主任务仍由其原有管理端负责。
退出查看后恢复原暂停模式和焦点，不擅自进入驾驶。过期、禁用、隐藏、换代数据清空。
原始下载由浏览器保存最新 NPZ，不保证恰为当前预览帧，也不把打开浏览器说成已保存。

证据目录 `build/inspection-tests/20261022/ue-measurements-live-02/`：
实际 UE 的 `thermal.png`、`acoustic.png`、`semantic.png`、`disabled.png` 以及 Automation 报告。
同级 `ue-measurements-live-02-source/recording/` 归档 23 份 NPZ、56,902 bytes，closed、无错误。
本测试实际触发 Slate 查询按钮／录制与启停复选框，9 条 ACK 全部匹配；还注入了慢列表
响应及首帧未收到的边界。测量数据路径在此 fixture 内为 LTU1 本地解码，控制为 native DDS。
它不证明完整 Product、原生测量 DDS、运动、持续频率或全部数值参数控件验收。

工厂切片 `factory-device-measurements-01/` 另保存了真实 Fab 候选几何上的 64×48
热／语义图及双通道 Pa 块。设备热属性、声源和 rig 标定是显式人工仿真声明；实际墙体
遮挡保留无效结果。尚未发布该 WorldPackage 或冻结真实机器人巡检路线。
完整勾选计划与限制见 [`multimodal-sensors.md`](../../../docs/design/multimodal-sensors.md)。
