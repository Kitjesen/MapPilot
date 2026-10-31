# GLIM 独立验证与原生接入评估

日期：2026-10-29；2026-10-30 更新。状态：外部证据核查、接入设计和隔离原生输出
适配实验完成；按用户最新方向，下一步收窄为现有 LTLIO 内的匹配机制实验。
完整 GLIM 原生后端尚未实现，不再作为当前下一任务。

## 当前决定

GLIM 已有作者之外的定量验证，其中包含 MID-360、CPU GICP 和独立动捕真值，
足以支持 LingTu 继续做原生接入实验。结果也包含退化和失败，不能据此宣布它全面
优于 LTLIO，或直接切换现场默认实现。

10-30 用户进一步明确：保留已有系统，主要研究 GLIM 核心机制能否结合当前算法。
因此当前采用 **LTLIO 保持主实现，先隔离验证激光匹配观测** 的方向，不继续把完整
GLIM 后端接入当作默认下一动作。原生输出适配和整体精度对照仍作为已完成证据保留，
不能当成某一个机制已提升 LTLIO 的证明。

现有 LTLIO 已有概率平面和物理不确定性处理；GenZ 混合匹配候选默认关闭，不能随本次
实验启用。暂不引入固定窗口优化或第二套全局地图优化。本文不改变原生 SLAM、mapd、
RobotConfig、RunPlan 或 ProductControl 的所有权，也不新增正式运行依赖。

新增的[匹配兼容性实验](../../build/glim-matching-20261030/README.md)已通过：复用官方
GLIM 正则化、原生 LTLIO 方差函数和两份保存扫描，共 12,672 点。GLIM 默认将局部
几何特征值固定为 `[0.001,1,1]`；这类形状权重不能直接替换 LTLIO 的物理测量/平面
方差。实验只确认迁移语义，不是轨迹精度改善证据。

下一实验固定 **相同预测状态、相同更新前局部地图、相同 LTLIO 去畸变/采样云**，
仅比较现有概率平面观测与定义清楚的匹配候选。候选必须先明确残差、Jacobian 和
不确定性/信息尺度；同一批回波不叠加两份独立约束。局部结果支持后再跑完整录制，
同时报告位置、高度、最大/末端误差、覆盖率和成本，不以邻域拟合误差替代实际精度。

## 别人实际验证了什么

### 作者论文：算法与整套系统有定量实验

Koide 等，*GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated
Scan Matching Factors*，Robotics and Autonomous Systems 179，104750，2024。
[论文](https://arxiv.org/html/2407.10344v1)、[DOI](https://doi.org/10.1016/j.robot.2024.104750)。

Newer College 九序列平均平移 ATE：GLIM 里程计 0.120 m、加入全局优化 0.088 m，
FAST-LIO2 0.242 m。该实验主要使用 Ouster，性能路径使用 GPU；论文报告的里程计
29.3 ms/frame 来自 i7-8700K 和 1660 Ti。它证明作者配置有效，不是我们的 MID-360
CPU 组件、NX 实时性或现场导航验证。

### 独立学术评测：CPU GICP + MID-360 有正面结果，也有落后项

Ribeiro 等，*Indoor Benchmark of 3D LiDAR SLAM at iilab – Industry and Innovation
Laboratory*，IEEE Access 13:212421–212442，2025，
[DOI](https://doi.org/10.1109/ACCESS.2025.3643753)。
[数据与方法](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/)、
[MID-360 原始结果表](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/benchmark/results/3d_lidar_sensors/livox_mid-360/)。

轮式室内平台，使用独立动捕真值；下表为位置 ATE RMSE，单位 m。

| 算法 | Nav A Diff | Nav A Omni | Loop | Slippage | Ramp |
| --- | ---: | ---: | ---: | ---: | ---: |
| GLIM | 0.017 | 0.016 | 0.104 | 0.025 | 0.057 |
| DLIO | 0.027 | 0.021 | 0.017 | 0.030 | 0.012 |

GLIM 在普通运动与 Slippage 有优势，但 Loop、Ramp 落后。这支持有效性，不能支持
普遍领先。评测版本与我们当前固定提交并非已证明相同。

为区分算法能力表和实际开启的功能，核对评测仓库固定提交
`6f813a277d3eaa881a729b01b5454a8c74545f07`：

- [MID-360 配置](https://github.com/jorgedfr/3d_lidar_slam_benchmark_at_iilab/blob/6f813a277d3eaa881a729b01b5454a8c74545f07/slam_benchmark_ros2_conf/launch/slam/glim/config_iilabs3d/livox_mid_360/config.json#L8-L10) 选择 CPU 模块。
- [CPU 配准配置](https://github.com/jorgedfr/3d_lidar_slam_benchmark_at_iilab/blob/6f813a277d3eaa881a729b01b5454a8c74545f07/slam_benchmark_ros2_conf/launch/slam/glim/config_iilabs3d/config_odometry_cpu.json#L39-L51) 选择 GICP。
- [全局优化配置](https://github.com/jorgedfr/3d_lidar_slam_benchmark_at_iilab/blob/6f813a277d3eaa881a729b01b5454a8c74545f07/slam_benchmark_ros2_conf/launch/slam/glim/config_iilabs3d/config_global_mapping_cpu.json#L26-L35) 关闭优化，不能将该组结果当成完整回环收益。

### 独立困难场景反例

Będkowski 等，*The benchmark of LiDAR odometry algorithms utilised for a low-cost
mobile mapping system*，ISPRS Archives，2025。
[文章与复现入口](https://isprs-archives.copernicus.org/articles/XLVIII-1-W6-2025/25/2025/index.html)、
[PDF Table 4](https://isprs-archives.copernicus.org/articles/XLVIII-1-W6-2025/25/2025/isprs-archives-XLVIII-1-W6-2025-25-2025.pdf)。

Bunker DVI 测试中 GLIM ATE 为 35.905 m，作者将该次运行判为失败；FASTER-LIO
为 0.363 m。论文没有充分交代 GLIM 固定提交和 CPU/GPU 配置，所以它证明存在
困难场景失败，不能预测我们当前组件会出现相同结果。

补充的[独立社区 MID-360 实验](https://github.com/harshareddy1611/livox-mid360-slam-evaluation/tree/792f18cecae77a7aaec71586b91c7e72f700565b)
公开了脚本和结果；CPU TIERS 三段平均 APE 为 GLIM 0.069 m、FAST-LIO2 0.067 m。
它不属于同行评审证据，也不支持只选其中一段宣称全面领先。

## 我们自己的证据和当前组件

[隔离对照入口](../../build/mapping-advance-20261029/README.md) 保存命令、配置、轨迹、
评分器、覆盖率和图。在线位置 RMSE：Slippage 为 LTLIO 0.020610 m / GLIM 0.018068 m；
Corridor 前 241 s 为 0.239426 m / 0.214719 m。Slippage 的高度误差和 Corridor 的
最大/末端误差仍有落后项；完整 Corridor 尚未完成，不能把前缀高度分数当成回归修复。

固定版本为 GLIM `1a197c999e4865f5cab7c9cd25474128413ebbca`、gtsam_points
`8e94654558dfbe7ddb0341de300daaf17e25ee02`。当前组件裁剪了 mapping、serialization
及 GPU 等设施，并有构建层 API 兼容修补；数值源码未修改。它不是上游完整标准包
原样安装，精确差异见[构建记录](../../build/mapping-advance-20261029/build/README.md)。

CPU 路线先对当前帧做 GICP，再形成位姿约束，与 IMU 及 bias 约束进入固定窗口。
它没有把所有原始匹配因子放进窗口联合优化；GPU 论文收益不能直接记在该 CPU
组件上。参见[固定官方 CPU 源码](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/odometry/odometry_estimation_cpu.cpp#L93-L167)。

## 最小接入位置与实际差异

| 现有位置 | 接入时承担什么 |
| --- | --- |
| `src/localization/slam/cpp/slam.hpp:200,312` | 复用 `SlamOutputs` 和 `ISlamBackend`；不新建 provider 框架。 |
| `src/localization/slam/cpp/cyclone_runtime.cpp:127,361` | 当前只创建 LTLIO；后续完整接入仍走同一个 DDS 输出和地图观测转换。 |
| `src/localization/slam/cpp/ltlio.cpp:457,471,2033` | 现有 IMU/body/LiDAR 转换和扫描输出语义的对照。 |
| `src/maps/cpp/mapd/save.cpp:129` | 继续由 mapd 发起并保存完整地图事务，不另起一个地图所有者。 |
| `src/localization/slam/cpp/CMakeLists.txt:21` | 正式 DDS 构建当前要求 LTLIO；正式双后端支持需要有意识地修改该约束。 |
| `src/lingtu/assembly/compile_plan.py:632` | 正式配置通过 RunPlan；不在研究脚本里改现场进程选择。 |

这次源码核对发现两个真实适配差异，不应靠额外防御包装处理：

1. **坐标系。** GLIM `EstimationFrame::frame` 已经去畸变并变换到 IMU 坐标；
   LTLIO 点云辅助函数接收的是 LiDAR 坐标。直接套用会重复施加 LiDAR→IMU 外参。
   适配时使用既有标定把 IMU 点变为 body 点，并计算真实 LiDAR 射线原点。
2. **时间与快照。** GLIM 估计参考扫描起点，LTLIO 当前参考扫描末端。初始离线
   实验保留真实起点时间，位姿和去畸变点云必须配对；不得只把时间标签改成末端。
   GLIM 历史 frame 会被后续平滑更新，在线输出应在本次插入后复制数值快照。

`ISlamBackend` 本身没有强制扫描末端；现有 DDS 可携带真实扫描起点，Host 状态适配
也优先使用显式扫描估计时间。后续若选择转换到末端，需要真正传播状态并转换点云
参考时刻，不能靠重标时间实现。

现有 `odom_prior_bypass_ltlio` 将 prior 当扫描末端状态，并刚性变换原始点云，未做
GLIM 去畸变；它可以检查外部位姿连线，不能作为这次完整接入的捷径。

## 原完整组件接入计划与已完成证据

以下清单保留之前的接入方案。按用户最新方向，未完成的完整后端和生命周期接入
暂不推进；接下来执行匹配机制实验。

- [x] 外部作者实验、独立正例和反例核查。
- [x] 本地隔离 CPU 里程计组件及两组真实数据对照，保留其已声明的范围。
- [x] 现有原生接口、配置选择、地图保存和坐标/时间差异源码核对。
- [x] **隔离的原生输出适配实验。** 10-30 复用同一官方 CPU 组件与 LTU 读取器，
  将立即复制的在线 pose 和同帧 IMU 去畸变点云转换为真实扫描起点的 `SlamOutputs`，
  复用现有 map frame 语义。非零外参/快照单测通过；Slippage 与 Corridor 各 150 云
  实际回放，各 120 帧的转换、时间、原点和同次位姿检查通过。不生成 TRACKING，
  不是完整后端或地图生命周期验收。[程序、命令与全部结果](../../build/glim-native-adapter-20261030/README.md)。
- [ ] 使用配对在线输出生成地图并验证保存、重载、射线原点；补完整 Corridor 和
  高度/楼梯数据对照，同时看高度、最大/末端误差与失败覆盖，不能只看平均 ATE。
- [ ] 上述结果支持采用后，再评估正式依赖和选定 RobotConfig 平台的 p95/p99 时延、
  队列和内存，以及 `ISlamBackend` 的 save/load/relocalize/reset 和 Product 启停。

这一阶段分别运行 GLIM 和 LTLIO。两者共享原始 IMU，不能把 GLIM 估计再当成一份
独立 IMU 来源的位置观测喂回 LTLIO，获得虚假的额外信息。暂不同时移植初始化、
GICP、滑窗和回环，避免失去归因依据。

## 依赖和许可

GLIM、gtsam_points 核心均为 MIT，复制或分发实质代码应保留版权和许可。
[GLIM LICENSE](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/LICENSE)、
[gtsam_points LICENSE](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/LICENSE)。
GTSAM、vendored nanoflann 等依赖有各自许可，不能统一写成 MIT。

现有批准只覆盖隔离研究目录。标准 GLIM 构建还涉及 GTSAM serialization、
gtsam_points/GTSAM_UNSTABLE 等版本要求；不能把“只拿平滑器”当作无需依赖和维护的
小改动。此次没有新增正式依赖、修改现场默认值或部署。

## 10-30 完整组件实验保留的判断

采用完整 CPU 里程计组件无需先穷举内部消融；拆某个内部算法进入 LTLIO 才需要隔离
证明该项收益。当前先保留目标 10000 点、2 线程、8 轮 GICP、5 秒历史窗口。
目标平台真实节奏和资源配额测量应尽早进行，避免完整产品接入后才发现算力不足。
若瓶颈在预处理/匹配，先做 10000→5000 点的单变量试验，并复测完整轨迹与失败覆盖；
暂不同时调噪声、窗口和迭代数。该参数试验本轮没有执行。

原始 GLIM 两次独立运行的前 120 个在线位姿出现最大约 5.21 mm 的平移差。
上游并行采样存在调度顺序依赖；不能用跨独立运行逐位相等来验收适配，也不能从单个
时刻差异推断 ATE 波动大小。适配已改用同次同帧的独立数值基准，保持原精度要求。
正式选择前应测完整序列的重复运行范围，尤其 Slippage 目前单次 RMSE 优势只有约
2.54 mm。[问题证据与验收修正](../../build/glim-native-adapter-20261030/README.md#验证方法与已发现的问题)。

## 本轮验证范围

10-29 为外部一手资料核查、已有结果复用和接入设计；10-30 新增隔离适配单测和两段
150 云回放。没有重新执行第三方 benchmark、完整精度评估或现场运行。代码质量
规则已记录于仓库根 `AGENTS.md` 的 `Code Simplicity`；规则本身不构成代码库已完成
清理的证明。正式依赖、DDS 运行、地图保存/重载和重定位仍未由 GLIM 接管。

同日随后新增匹配兼容性实验，CTest 1/1 与两份保存扫描检查通过。没有重新回放
传感器数据、修改 LTLIO 数值实现或生成新的 ATE；当前下一动作以上方收窄后的决定为准。
