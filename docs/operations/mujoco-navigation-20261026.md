# 2026-10-26：原生导航 MuJoCo 组件物理回归

同日后续选型已更新：本地 Go2 real、Thunder real/sim 正式默认改为平台组合碰撞模型，
配置水平余量和 endpoint 当前机身姿态接线已修正。以下旧实验的模型、参数和失败保留；
其中“当前”指各次实验时点，不代表最新配置。最新配置与本轮验证见
[维护入口](../design/navigation-body-model-and-motion-posture.md#当前决定与实际状态)。

本文中的“URDF 分部模型/部件模型”现统一称**组合碰撞模型**；`BODY2` 仅为内部文件标记。
[命名与行业来源](../design/navigation-body-model-and-motion-posture.md#名称行业来源与实现边界)
不改变下述历史模型标签、参数或结果，不能把命名更新算作模型接入或测试通过。

**状态：当前源码的 22 m 多转弯和关节部件短程到达；横穿障碍失败，同参数旧版也失败。
动态避障缺口尚未修复。完整 Product/native DDS 链另行记录，不能据本页判为通过。**

本页记录新运行的实际物理结果，以及保留的启动失败和旧版对照。没有修改生产源码、
产品配置、原场景、actor 运动、CCD 参数或接触分类来求通过，没有连接或驱动实机。
[错误记录](error_record.md) 保留了此前编译验证与本次物理结果的先后关系。

**后续配置审查已确认：下述原始 OBB 运行错误地读取 `nav/real` 几何；parts 与 OBB
还使用了不同控制参数。因此这些历史结果不能当作纯模型优劣对照。测试入口现已修正，
新同配置短程物理复跑见文末，旧产物不覆盖。**

## 运行范围与可复查产物

Python 通过 Thunder V4 的 ONNX 策略推进 MuJoCo 动力学，native C++ 接收实测位姿、
速度和机体状态，再执行全局路线、Executor、SCAN、Follower 及该基准的最终几何/制动
检查。运行使用 `--external`，没有用积分更新替代物理，也没有播放旧轨迹充当新运行。

- 物理模型：[Thunder V4 MJCF](../../sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml)，
  16 个关节；策略为
  [policy_4998.onnx](../../sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_4998.onnx)。
- Python 入口：[body_physics.py](../../tools/bench/body_physics.py)；native 入口：
  [benchmark_body_execution.cpp](../../tests/nav/cpp/benchmark_body_execution.cpp)。
- Windows MSVC x64 Release 在本轮独立 `component/native-build` 中重建了 native 目标；
  [配置日志](../../build/nav-mujoco-20261026/component/configure.log)、
  [构建日志](../../build/nav-mujoco-20261026/component/build.log) 和
  [实际源树状态](../../build/nav-mujoco-20261026/component/source-status.txt) 已保留。
  工作树有未提交修改，HEAD 提交号本身不能代表本轮二进制。
- MuJoCo 3.10.0、ONNX Runtime 1.23.2、NumPy 1.26.4；保持 `nativeccd` 和 `1e-6` 容差。
  实际物理步长为 0.005 s，harness 控制反馈步长 0.02 s，使用 `--pace`。
- 导航读取人工确定的完整三维地图；横穿 actor 的真实位置、速度按固定 fixture 注入。
  引擎虽然初始化 MuJoCo LiDAR，本基准不以 LiDAR 感知结果驱动导航。
  这不是完整 Product、Go2 当前速度验收、NX 或现场证据。
- 总产物：[本轮 HTML](../../build/nav-mujoco-20261026/component/index.html)、
  [results.json](../../build/nav-mujoco-20261026/component/results.json)、
  [三项当前源码轨迹与命令/实测速度图](../../build/nav-mujoco-20261026/component/report/component-physics-overview.png)。
  图中明确标出了横穿用例零命令和真实接触时刻。

## 三项当前源码结果

以下“执行时间”是初始化站稳之后的物理仿真时间，包含末端观察；墙钟和规划耗时不与之混用。
接触统计是每个已检查物理子步上的**不允许接触**，不是说机器人没有合法足底接触。

| 用例 | 路线来源与物理场景 | 结果 | 执行时间 | 实测 XY 路程 | 终点 XY 误差 | 不允许接触子步 |
|---|---|---|---:|---:|---:|---:|
| 配置 OBB 的 22 m 多转弯 | 既有 22 m 分段 pose route，`--use-route`；原解析平面 | 到达 | 50.00 s | 21.2646 m | 17.211 mm | 0 / 10,000 |
| 实测部件的 1.2 m＋90° | 原平面；native 全局搜索，14 点；38 个关节碰撞部件 | 到达 | 9.78 s | 1.24443 m | 0.609 mm | 0 / 1,956 |
| 配置 OBB 的 8 m 横穿障碍 | 原 crossing 场景和 actor；native 全局搜索，81 点 | **接触失败，未到达** | 6.895 s 首次接触；记录至 6.90 s | 3.26835 m | 未到达，不作为精度通过项 | 2 / 1,380 |

三项运行的原始命令和指标分别为：

- 22 m：[实际 Python 命令](../../build/nav-mujoco-20261026/component/current-multi-turn-plane/harness-invocation.json)、
  [native 命令和物理设置](../../build/nav-mujoco-20261026/component/current-multi-turn-plane/invocation.json)、
  [物理指标](../../build/nav-mujoco-20261026/component/current-multi-turn-plane/physical-metrics.json)。
- 部件短程：[实际 Python 命令](../../build/nav-mujoco-20261026/component/current-parts-short-plane-scan-env/harness-invocation.json)、
  [native 命令和物理设置](../../build/nav-mujoco-20261026/component/current-parts-short-plane-scan-env/invocation.json)、
  [物理指标](../../build/nav-mujoco-20261026/component/current-parts-short-plane-scan-env/physical-metrics.json)。
- 横穿：[实际 Python 命令](../../build/nav-mujoco-20261026/component/current-crossing/harness-invocation.json)、
  [native 命令和物理设置](../../build/nav-mujoco-20261026/component/current-crossing/invocation.json)、
  [物理指标](../../build/nav-mujoco-20261026/component/current-crossing/physical-metrics.json)。

配置 OBB 的尺寸为 **1.16 × 0.76 × 0.60 m**，中心 `(0,0,0.05)`，margin 为零；
不能把它写成 Go2 几何。最终解析控制值为最大速度 1.41421356237 m/s、加速度
4 m/s²、nominal dt 0.01 s、反应时间 0.35 s、线/角减速度各 4；实际最大发送 XY
速度为 0.75 m/s。以
[resolved-control.json](../../build/nav-mujoco-20261026/component/current-multi-turn-plane/native/resolved-control.json)
和 [navigation-body.json](../../build/nav-mujoco-20261026/component/current-multi-turn-plane/native/navigation-body.json)
为准，不能只读基准在 `--obb` 覆盖之前的 0.5 m/s 初值。

部件模式使用基准自身的 0.5 m/s 最大速度、2 m/s² 加速度、0.02 s nominal dt、
0.1 s 反应时间、0.5 m/s² 线减速度和 1 rad/s² 角减速度，不能与 OBB 控制参数混作同参对照。
38 个部件按当前关节、roll/pitch 和加载足状态形成实际快照；这证明该组件入口接收并使用
快照，不证明正式 navd 已完成驱动关节订阅和 FK 接线。

## 到达之后的实际停车，以及仍存在的转角停顿

到达两项均继续观察了末端 0.8 s；该窗口中各 41 个命令全部为零。实际速度如下：

| 用例 | 末次实际速度 XYZ / m/s | 末次实际 XY 速度 | 末 0.8 s 最大实际 XY 速度 | 末 0.8 s 最大实测 yaw 差分速率 |
|---|---|---:|---:|---:|
| 22 m OBB | `(0.001474, 0.000117, -0.000198)` | 0.001478 m/s | 0.001697 m/s | 0.001411 rad/s |
| 部件短程 | `(0.000034, 0.000255, -0.001971)` | 0.000257 m/s | 0.001064 m/s | 0.000416 rad/s |

这是发零命令后**实测接近静止**的证据，不是数学零速度。统计来自新物理记录，未以命令值
代替实际速度；两项均没有“控制帧完全无加载足支撑”的记录。

22 m 路线仍有停顿：第二个转角约 `(8,6)` 处，`aligning_goal_yaw` 共 170 个
20 ms 样本，即 **3.40 s**。其平移命令为零，主体从约 90° 转向 0°；样本约覆盖执行
32.98–36.42 s，中间有短暂其他状态。第一角的圆滑过渡不能外推为整条路线全程连续转弯，
本轮没有解决第二角停顿。该阶段与终点 `settling_at_goal` 分开计时。

## 横穿障碍：已停车仍发生真实接触，缺口未修复

当前版本约在执行 5.92 s 开始近场零命令，6.10 s 进入最终几何阻挡；此后命令持续为零。
执行 6.895 s，`RR_wheel` 与 `fixture_actor_geom` 发生侧向接触：约 **1.997 mm**
穿入，三个接触点法向力约 **146.9–151.2 N**，法线约 `(0,1,0)`。
接触前实际 XY 速度约 0.000727 m/s，actor 仍按既有运动继续横穿，随后碰到已停下的机器人。

这不是地面/heightfield 法线导致的误分类；实际接触几何、力、qpos/qvel 和控制量在
[首次坏子步](../../build/nav-mujoco-20261026/component/current-crossing/first-bad-substep.json)，
停车与接触过程在 [时间线](../../build/nav-mujoco-20261026/component/crossing-timeline.json)。
**结果是动态场景失败，不是“正确停车所以通过”。**

为避免把不同速度配置的历史成功当成当前版本回归，额外用留存的 `tangent-v4.exe`
和同一当前 harness/配置复跑了 crossing。它也在执行 **6.50 s** 发生相同后右轮/actor
侧撞，1 / 1,300 个检查子步出现不允许接触。
[旧版新运行摘要](../../build/nav-mujoco-20261026/component/archived-v4-crossing/summary.json)
及 [首次接触](../../build/nav-mujoco-20261026/component/archived-v4-crossing/first-bad-substep.json) 均保留。

双方的 resolved controls、导航 OBB、navigation config 逐值相同，生成地图字节、世界
XML 和 15 项 harness 物理输入也相同，见
[匹配证据](../../build/nav-mujoco-20261026/component/crossing-matched-inputs.json)。
两版各一次均失败，支持“此配置下仍有动态能力缺口”；不支持声称本轮引入、已修复，
也不能据 6.895 与 6.50 s 推断成功率或性能变化。本轮没有为通过而改变 actor、地图、
速度、CCD 或碰撞分类，未尝试生产修复。

## 全局折线距离与局部样条跟踪误差

两类指标由既有
[body_tracking_metrics.py](../../tools/bench/body_tracking_metrics.py) 分别计算：

| 当前源码用例 | 全程到有限全局 XY 折线的距离 RMSE / 最大值 | 活跃样条跟踪阶段的位置误差 RMSE / 最大值 |
|---|---|---|
| 22 m 多转弯 | 98.918 / 579.877 mm | 6.588 / 22.963 mm |
| 部件短程 | 2.393 / 6.204 mm | 8.626 / 13.086 mm |
| 横穿障碍，未到达 | 206.069 / 463.767 mm | 13.366 / 23.184 mm |

左列是到**有限全局折线**的最近几何距离，包含所有状态、转弯切角和绕障偏移；右列只统计
`spline_control_ready`，对照 native 当前执行时间上的样条参考位置。左列不能改称样条
跟踪误差，右列的小误差也不能证明横穿避障安全。
[指标和定义](../../build/nav-mujoco-20261026/component/tracking-metrics.json) 已保留。

历史 heading04 的 22 m 控制参数是最大速度 0.6、加速度 2、线减速度 0.5、角减速度 3，
与本轮不同。历史 47 s 只作参考；没有追加旧版 22 m 同参运行，也没有宣称本轮速度提升。

## 保留的首次 parts 启动失败

最初 `--navigation-body parts` 未给 native parser 解析环境。当前 parser 默认 CMU，
在进入执行前要求 path library，原生报错后 harness 留下 `EOFError`。
[失败摘要](../../build/nav-mujoco-20261026/component/current-parts-short-plane/summary.json)
和 [native 原因](../../build/nav-mujoco-20261026/component/current-parts-short-plane/native.log)
保留，不能计为物理通过或机体碰撞失败。

复跑只给该子进程设置 `LINGTU_NAV_LOCAL_PLANNER_BACKEND=scan`，与基准实际使用 SCAN
一致，见 [单独环境记录](../../build/nav-mujoco-20261026/component/current-parts-short-plane-scan-env/harness-env.json)。
没有伪造 path library，没有修改生产默认、速度或物理参数；复跑产物使用新目录，未覆盖失败。

## 本地视频

视频是对本轮新物理 `states.jsonl` 中实际机体与关节状态的可视化回放；回放不推进新物理，
不把视频渲染次数算成新增试验。画面和原始物理结论以对应源记录为准。

- 22 m：[本地视频页面](../../build/nav-mujoco-20261026/component/current-multi-turn-plane-replay/index.html)、
  [MP4](../../build/nav-mujoco-20261026/component/current-multi-turn-plane-replay/measured-route-replay.mp4)、
  [回放来源记录](../../build/nav-mujoco-20261026/component/current-multi-turn-plane-replay/replay-record.json)。
- 横穿失败：[本地视频页面](../../build/nav-mujoco-20261026/component/current-crossing-replay/index.html)、
  [MP4](../../build/nav-mujoco-20261026/component/current-crossing-replay/measured-route-replay.mp4)、
  [回放来源记录](../../build/nav-mujoco-20261026/component/current-crossing-replay/replay-record.json)。

以上是工作区本地产物链接，不是已发布远程视频地址。

## 原生 DDS 与 MuJoCo 独立运行

**未通过导航到达验收。** 独立目录重新编译 Windows Release navd/control，接现有原生
DDS sensor publisher、driver bridge 和 mapd。定位为 MuJoCo 真值，地图为同源 synthetic hits；
没有经过 ProductControl 事务，也不验证 LTLIO。

- 首次 `short-flat-run` 使用 0.1 m 地图，但生成器仍只生成三层已知自由空间，实际机器人
  中心高度落在 unknown。最早为 `start_pose_blocked`，BodyQuery 尚未进入。
  这是小 fixture 构造错误；地图 artifact gate 只证明格式/激活元数据，不能证明可导航。
- 同源修正恢复原测试的 0.2 m 分辨率，并将起终点放在已观测地面内部。原生查询确认
  中心支撑通过；随后 guide/heading 返回 `guide_body_or_turn_rejected` / `empty_path`。
  同 OT、同 OBB 查询证明：地面占据体素 Z=[0,0.2]，目标 body.z=0.45、below=0.25，
  机体底部恰好为 0.2，按既有接触规则返回 `occupied_body`。实际起点 z=0.453126
  为 free。没有再改变目标、缩小机体、关闭支撑或降低验收标准求通过。
- 修正运行输入就绪比例 1.0，但没有形成非零导航运动命令，实际约 0.005 m 是站立漂移。
  两轮均取得最终零命令、物理应用后的 STOPPED ACK、控制权清除与 owned-process cleanup。
  零命令物理应用证据不等于整条导航已通过。
- 运行中修复一项验收工具接线：`_component_native_environment` 原先仅保留碰撞前缀，
  会丢掉编译输出的机身长、宽、余量。现在额外透传这三个明确键，实际编译配置的
  最窄回归 3/3 通过；没有放行其他控制参数或修改生产 RobotConfig。

证据：[首轮报告](../../build/nav-mujoco-20261026/product/short-flat-run/report.json)、
[修正报告](../../build/nav-mujoco-20261026/product/short-flat-corrected-run/report.json)、
[首轮地面证明](../../build/nav-mujoco-20261026/product/map-support-proof.log)、
[修正地面证明](../../build/nav-mujoco-20261026/product/map-support-corrected-proof.log)、
[目标与机体接触查询](../../build/nav-mujoco-20261026/product/map-body-corrected-proof.log)、
[工具回归](../../build/nav-mujoco-20261026/product/obb-env-green.log)。

后续应先统一同源地图的体素表面、自由空间高度与 body-origin 目标定义，再单独验收该链。
本轮组件的两项到达不替代此结果；当前不能宣布完整 Product 或 Go2 实机已通过。

## 模型来源与配置混用的修正

2026-10-26 后续审查确认，“有 URDF”与“导航已使用实时关节几何”曾被混为一谈。
当前实际状态如下：

| 使用位置 | 实际模型与来源 | 尚不能据此宣称的能力 |
| --- | --- | --- |
| Thunder MuJoCo 物理接触 | URDF 转换的 MJCF 碰撞原语，包含机体、腿、轮 | 不会自动把关节模型传给 navd |
| 正式真实 Thunder 导航 | `robot.yaml` 未设置 `navigation_body_file`，native 构建单 OBB，1.16×0.76×0.60 m | 不代表真实 Thunder 的逐关节模型已接通或校准 |
| 正式 sim 导航 | package 已解析的 below=0.05、above=0.35，单 OBB 为1.16×0.76×0.40 m | 不代表导航与物理接触都使用单盒 |
| `body_physics --navigation-body parts` | MuJoCo 实测关节与碰撞原语导出 38 个分部快照 | 仅证明组件分部入口，不证明正式产品实时 FK 接线 |

确认并修正了两个配置接线错误：Python 的 OBB 分支独自解析 `nav/real`，忽略已有
sim package 的 torso-origin 下界；native 又以 `if(obb)` 独自切换 Product control，
导致 parts 保留另一套手写速度、加速度、制动和时钟。原生 DDS 独立组件的 manifest
below=0.25 也不能当作正式 sim package 的生效配置。

现在两种物理 benchmark 模式均只调用一次现有 `compile_run_plan(nav, sim, ...scan)`，
使用同一份 MJCF、policy 和 native 环境。native 的全局、局部、Executor、制动与 dt
统一从该配置建立，所选 Body 一贯传至 request、route、Executor 和最终检查。
支撑保持 package 的 0.435±0.10 m；只将 local bitmap resolution 匹配实际输入地图，
同时记录配置、输入和生效分辨率。模型选项只选择 OBB 或实测分部及其对应状态更新。

组件显式模型选择会清理 ambient `LINGTU_NAV_BODY_FILE`，避免终端残留的模型路径
替换本次几何。其他既有 native tuning override 继续支持，并完整记录相对 plan 的
差异。本次两项物理复跑共同保留 `LINGTU_NAV_SCAN_HEADING_ERROR_RAD=0.4`，不能写成
没有 override。真实 RobotConfig、正式产品默认和部署均未改变。

验证与新物理复跑保存在独立的 `build/nav-model-consistency-20261026/`：

- 独立 MSVC Release 构建通过；同一初始状态的一帧 native 红绿对照确认：旧 binary
  两模式控制不同，新 binary 的 `resolved-control.json` 逐值相同，模型为 38 parts / 1 OBB。
  [原生红绿证据](../../build/nav-model-consistency-20261026/verification.json)。
- Python 模块回归 21 项通过；补充 ambient BODY_FILE 与 support-patch override 的
  2 项回归通过，修改文件 Ruff 通过。覆盖实际传给 `Popen` 的环境与保存的证据，
  不只是检查配置辅助函数。
- 实际 ONNX＋MuJoCo 各运行一次同一 1.2 m＋90° 请求，使用同一地图、RunPlan、
  初始请求与 override；实际物理步长均为 0.005 s。

| 新同配置运行 | 执行时间，含末端观察 | XY / yaw 终点误差 | 不允许接触子步 |
| --- | ---: | ---: | ---: |
| 38 个实测部件 | 15.02 s | 6.672 mm / 1.003° | 0 / 3,004 |
| 单 OBB | 13.26 s | 16.930 mm / 1.681° | 0 / 2,652 |

末端 0.8 s 命令均为零，实际最大 XY 速度分别为 0.001277、0.003736 m/s。
[新物理记录与共同参数](../../build/nav-model-consistency-20261026/physics-verification.json)、
[parts 原始摘要](../../build/nav-model-consistency-20261026/physics-parts/summary.json)、
[OBB 原始摘要](../../build/nav-model-consistency-20261026/physics-obb/summary.json) 可复查。
这两次只验证修正后的短程接线与执行；不支持模型总体精度、性能或动态避障优劣结论。
几何余量也未被伪装成相同：部件 Body 保留默认各向同性 0.02 m，OBB 水平每侧
0.08 m 已计入其尺寸，Body.margin 为零。当前只有控制配置统一；这不是等余量的
两种几何表达精度评测。正式分部模型需要再明确水平余量与允许足底接触的语义。
未重跑长路线、动态横穿或完整 Product，也未部署到 Go2/Thunder 实机。
