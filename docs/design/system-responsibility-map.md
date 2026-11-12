# 系统职责分工与待整理问题

核查：2026-11-01。本文按当前工作区的代码和配置梳理职责，供排查与后续整理使用。
产品选择以 `config/system/` 和实际 RunPlan 为准；运行结果以实际状态和测试记录为准。
设计目标不等于功能已实现，本表不代表任何现场设备已经部署或验收。

## 2026-11-08：删除本地终点转向扩展（当前）

普通导航按 XYZ 位置完成，不再要求终点 yaw。Follower 只跟踪 SCAN 的局部样条，沿用
其轨迹时间与位置误差结束规则；Executor 检查任务位置和高度，navd 的停止确认事务
继续负责停车完成后提交任务结果。中途显式姿态转向仍服务于后续路径，不增加终点转向。

已删除 FollowerGoal、目标朝向接管、goalTurn、最低终点转速、转向停止滞回、终点收尾
状态，以及整组 goal_follower 配置和运行时参数接线。普通参考冻结仍经过已有加速度限制，
真实碰撞/急停仍立即输出零。0.8 s 仅用于普通路径朝向前视。

SDK/DDS Pose 的 yaw 保留为输入位姿信息，普通导航不承诺执行终点朝向；执行 Route
已没有 finalYaw 字段。姿态路线末尾仅为朝向存在的转向段同样删除。
本轮证据位于 `build/scan-upstream-goal-20261108/`；此前保留终点转向的结果仅是历史对照。

2026-11-09 的跨会话同场景对照位于 `artifacts/nav/turn-compare-20261109/README.md`。
上面的删除试次沿用了冻结旧参数，不能据此概括当前正式 nav Product。当前程序配完整正式
参数及新版 mapd 已通过同一双箱 S 弯；只换地图服务或只换运动参数的试次仍失败。
这说明箱角续接的稳定性尚未解决，不能归结为单个参数或删除终点转向的必然结果。

## 2026-11-07：navd 接口收敛（历史实现）

**以下终点朝向与收尾描述已被上面的当前实现替代，不代表现行行为。**


本节记录本轮实际改动；下方其他日期的记录保留为历史快照。详细验证保存在
`build/nav-converge-20261107/`，源码修改、组件通过与整机运动验收分别记录。

**继续使用已有 SDK、Gateway 和 navd，不新增服务进程。**

```text
外部 SDK / Web / gRPC
  → Gateway 共享导航操作（格式、地图身份、控制权）
  → nav.goals（请求记录和原生事实关联）
  → nav.commands（Host 装配、复用原生客户端）
  → 原生 Client / DDS → navd 导航生命周期入口
                          → 全局路线 → SCAN → Follower → 最终控制 → 驱动
```

### 本轮已实现

| 目标 | 当前实现 | 保留的边界 |
| --- | --- | --- |
| 一套目标提交逻辑 | 普通目标和地图点击都调用 `gateway/navigation/actions.py::go`；删除 `_publish_goal` 的重复复制与不可达 True 分支 | 来源、地图身份与输入含义仍分别解析 |
| 一份原生回执含义 | 原生 Python 客户端校验命令/任务/请求匹配；GoalService 消费类型化回执并记录。删除无回执也算成功的退路 | 通信未确认、原生拒绝、历史写入失败仍是不同结果；保留时钟重试身份 |
| 减少 Host 中转 | `Commands` 复用一个客户端视图；目标参数直接传递，删除回执转发 lambda 和重复转换 | Host 注册、共享 DDS session、C ABI 与直接构造客户端的资源所有权保留 |
| SDK 与实际接线完整 | 同步/异步 SDK、兼容入口、CLI 支持 XYZ、可选 yaw、位置容差和限速；领域接口增加按 task_id 暂停、恢复、取消及限速 | SDK 等待超时不隐式取消或重新发目标；ACK 不等于执行完成 |
| 暂停/恢复可追溯 | GoalService 复用取消的请求记录/派发流程，账本识别 pause/resume；相同请求重放不重复派发 | PAUSED/EXECUTING 仍只能由原生执行事实推进 |
| navd 单目标入口 | 现有 NavigationRuntimeController 接收暂停、恢复、取消，拥有等待停稳的恢复请求；巡检通过同一入口结束导航子任务 | 巡检父任务提交、异步全局规划、停止确认与结果投递仍各有实现，不新增 TaskManager |
| Follower 末端控制 | Follower 计算终点角速度、确认停稳后的实际位姿；Executor 协调轨迹与局部碰撞检查 | 位置修正仍使用 SCAN，并保持同一指定朝向；进入终点时减速，范围外的位置修正不反复触发减速；验收范围不放宽 |

普通 HTTP/SDK/CLI 目标未指定 yaw 时是 position-only；显式 `0.0` 仍要求零朝向。
这是有意的默认行为调整。原来依赖隐式零朝向的调用方应明确传 `yaw=0.0`。
已有 gRPC Pose 的数值 yaw 继续按指定朝向解释，本轮没有修改其 protobuf wire。

### 到达与参数归属（2026-11-08 更新）

终点控制参数由 `config/system/settings/follower.yaml` 的 `parameters.goal_follower`
维护，通过已有参数表、RunPlan 环境和原生配置解析到 Follower。C++ 默认值从同一 YAML
生成。已删除 Executor 中独立的转向增益/上下限默认值。

跟踪与到点收尾现在都通过 `Follower::follow(plan, state)` 完成，目标及容差随
`FollowerState` 传入。公开 `finish()`、Follower 的 0.2 秒计时器及对应配置已删除；
Executor 消费 Follower 的命令和完成结果，不再逐拍选择 follow/finish。

Follower 完成要求当前位置、高度、指定朝向和实测平移/垂直/角速度合格，且本拍转向命令
已降为零；不再额外等待“已应用零命令”或累计连续停稳时间。实际命令反馈仍用于加速度
限制，navd 的任务停止确认仍保留，Follower 返回完成不直接越过最终控制提交任务成功。

最终样条的现有朝向前视到达终点、剩余曲线路程进入相应速度前视范围，或机器人进入目标范围时，Follower 保留指定朝向；
中间局部轨迹的终点不会提前接管。活动末段样条先按已有结束距离收敛，不在刚触及到达
范围时截断；已在目标范围内停下则不必等待旧轨迹时间。在同一次收尾更新中减速并控制朝向，
进入收尾后，只要位置仍合格就保持该阶段，速度噪声不能恢复跟踪。范围外的位置修正继续使用 SCAN 轨迹及同一
朝向，不重新朝向短回程切线。默认朝向接受范围为 10°，显式任务容差优先。
为处理仿真中已观察到的停止后朝向偏移，正在进行的转向进入容差内侧一半后才结束：默认
约 5° 是主动转向的停止边界，不是任务接受阈值；已经静止且在 10° 内时不额外转向。
角速度继续受已应用命令的加速度限制、按剩余角度计算的停车速度和平台角速度上限约束。
接近目标与停住收尾共用同一个转向控制及转向状态，不再在两处使用不同的最低转速和停止边界。
路程前视沿三维曲线累计，保留剩余弯道；短曲线被重定时拉长时，不必等待被朝向对齐冻结的参考时钟。
普通朝向对齐只冻结参考进度，平移按原加速度减速；外部安全停止仍直接输出零。

原版 SCAN 固定版本 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` 的控制器按样条和位置误差
结束，不提供完整的显式终点 yaw / 停止后位姿确认契约。上述末端处理是灵途的本地扩展。
最新证据位于 `build/turn-continuity-20261108/report.md`：原倍率下四个同场景试次均正式
到达、零接触，并通过原运动门槛；到达后的记录直到约 45 秒观察结束仍满足 15 cm / 10°。
首次进入最后 30 cm 后，S 形最大朝向误差由上一轮 36.27° 降为 4.04°，偏置箱体由 35.54°
降为 4.85°。Follower 81 项、姿态路线 66 项及目标衔接 5 项通过；未重跑全仓库测试。
额外一次原倍率 S 弯复测仍正式到达、零接触且朝向合格，但停后位置最大漂至 15.80 cm、
最终 15.32 cm。因此五次原倍率运行的原运动门槛为 5/5，额外的停后位姿观察为 4/5；
转向衔接改善不等于停止后的定位保持已经全部通过。
暂停参考后的跟踪继续复用最终控制已确认的命令，避免内部速度历史清零造成突变。
`build/follower-approach-20261108/` 的 3/4、`build/follower-unified-20261108/` 的 2/4 运动门槛
结果及 8 项隔离基线失败保留为历史证据。本轮早期的旧 DDS 客户端、队列重叠和启动超时
试次也保留在新报告中，不作为转向对照；不能据针对性回归宣称整套导航或实机验收。
指定末端朝向及本轮主动转向停止余量均是本地集成，不是 SCAN 官方默认行为。

**以下为 2026-11-07 历史实验，不代表当前收尾实现。** 当时 CTS v39 对 0.2 rad/s 纯转向响应过弱，0.3 rad/s 的到达结果仍不稳定。修复阶段切换并使用 0.4 rad/s 后，连续三次普通绕箱测试通过既有验收；
边界漂移造成的流程切换另有回归测试。额外停后观察 32.8 秒，朝向保持在 5° 内，但位置缓慢漂移约 3 mm，最终超出 15 cm 范围约 0.86 mm，不能据此宣称长期定点保持合格。
完整对照、失败试次和验证边界见 `build/nav-converge-20261107/report.md`。

随后复核其中的跟随视频，发现短距离位置修正曾把机头从约 −21° 转到 −152°，再转回目标
0°；到达通过不能代表转向质量合格。现已在首次进入目标范围时接入指定朝向，Follower
先按现有加速度限制减速，再进行末端转向或请求 SCAN 修正位置，避免为短回退重新朝向路径
切线。新一轮证据与仓库去重记录位于 `build/wrapper-cleanup-20261107/`。
用户随后选择日常点选导航默认朝向容差为 10°，已写入 `follower.yaml`。显式路线给出的
朝向容差仍优先；前述 5° 实验保留为对照，不能作为 10° 配置的结果。

### 保留的实际职责

Gateway 的 CommandJournal 是按 client/command/request 关联的内存请求重放；
NavigationTaskLedger 是持久的导航请求和原生事实记录。两者的键、生命周期与覆盖命令不同，
没有简单删除其中一个。任务是否受理、是否完成仍以 navd 的实际结果为准。

ProductControl、RobotConfig、地图所有权、SCAN 求解器和最终运动仲裁未在本轮另建实现。
安全停止、急停、取消和暂停继续保留不同含义；关闭进程的停车事务也继续保留。

## 先看三条主线

```text
启动：Product + env + 机器人配置 → ProductControl 解析一次 RunPlan
      → real 或 sim 的进程管理 → native 服务 + Python Host

运动：目标 → 全局路线 → 局部轨迹 → 跟踪速度 → 最终仲裁
      → lingtu-driver → 平台运控

地图：传感器 → SLAM 位姿/点云 → mapd 实时地图 → 局部碰撞查询
                                    ↓ 新鲜空闲证据可纠正保存图的旧占据
      mapd 保存事务 → 保存 OctoMap → 准备全局表面和路线索引
```

## 产品、运行环境和硬件

| 问题 | 职责和代码入口 | 不负责 |
| --- | --- | --- |
| 产品有哪些能力 | `config/system/products/*.yaml` | 不启动进程、不决定硬件标定 |
| real/sim 具体用什么实现 | `config/system/envs/*.yaml` | 不执行产品切换 |
| 机器人几何、传感器和标定 | `config/robots/<vendor>/<model>/robot.yaml` | 不决定当前 Product |
| 解析生成 RunPlan | [`assembly/compile_plan.py`](../../src/lingtu/assembly/compile_plan.py) | 不启动原生进程；运行快照的发布和提交由 ProductControl 事务负责 |
| 发布运行快照与产品启停事务 | [`control.py`](../../src/lingtu/control.py) ProductControl | 负责提交、启动、切换和停止，不实现导航或 SLAM 算法 |
| real/sim 进程与各自回滚 | [`lingtu/real/`](../../src/lingtu/real/) / [`lingtu/sim/`](../../src/lingtu/sim/) | 不重新选择 Product；两者不强行合并回滚 |
| 实机服务目录与诊断 | [`runtime/service_catalogs/real.py`](../../src/runtime/service_catalogs/real.py) | 通用 Product unit 来源是 `real.yaml`；运行时以已解析 RunPlan 为准。目录提供安装和就绪元数据，不选择机器人型号或启动 Product |
| Python 进程内模块与连接 | [`runtime/host.py`](../../src/runtime/host.py) 和 [`assembly/host/`](../../src/lingtu/assembly/host/) | 不拥有原生进程生命周期 |
| Web/API 传输和状态展示 | [`gateway/`](../../src/gateway/) 与 `web/` | 不实现地图、规划或硬件运动 |
| 硬件速度转发和设备协议 | [`drivers/real/motion/`](../../src/drivers/real/motion/) | 不规划路线；只转发获准命令 |
| 仿真机器人物理与传感器 | [`sim/`](../../sim/) | 仿真结果不能当作实机反馈 |

Host 仍在使用：它承载 Gateway、Agent 和所选 Python Modules。Host 内装配和
ProductControl 的进程事务处于不同层，不应仅因两者都叫“启动”就合并。

## 定位、地图和地形

| 职责 | 当前实现及产物 | 边界 |
| --- | --- | --- |
| 激光和 IMU 采集 | [`drivers/`](../../src/drivers/) → 实测数据 | 设备 I/O，不负责地图或路径 |
| 连续位姿估计与去畸变 | [`localization/ltlio/`](../../src/localization/ltlio/) | 输出里程计和配准点云，不负责产品切换 |
| SLAM 服务与重定位结果提交 | [`localization/slam/cpp/`](../../src/localization/slam/cpp/) | 接收传感器和重定位结果，发布状态/位姿与地图快照 |
| 在线历史图优化 | [`localization/online_mapping/`](../../src/localization/online_mapping/) | 在线调度和后端共同处理关键帧图及校正快照，不取代高频 LTLIO |
| 保存时图优化 | [`localization/map_optimization/`](../../src/localization/map_optimization/) | 处理保存作业；点云和位姿文件读取与在线回放共享 |
| 地图实时占据、自由与未知 | [`maps/cpp/mapd/`](../../src/maps/cpp/mapd/) | mapd 是现场实时地图状态所有者，不发运动命令 |
| 地图保存事务与地图包 | [`mapd/save.cpp`](../../src/maps/cpp/mapd/save.cpp)、MapStore | mapd 协调 SLAM 快照和保存；ProductControl 负责激活地图与产品切换 |
| 保存时点云清理 | [`maps/prune/`](../../src/maps/prune/) | 用历史扫描的可见性反证和局部表面证据分流原始点；不负责实时占据或运动许可 |
| 全局多层表面与推断支撑 | [`maps/layers/surface.cpp`](../../src/maps/cpp/layers/surface.cpp) | 从准备的保存地图建立索引；推断面不写回原始占据，也不是足端承重证明 |
| 局部实测地形和机体接触查询 | [`collision/terrain.hpp`](../../src/nav/cpp/collision/terrain.hpp)、SCAN Grid | 用于实际碰撞/接触解释；分类用拟合不能直接当运动放行 |
| 地形风险代价 | [`endpoint/traversability/`](../../src/nav/cpp/endpoint/traversability/) | 独立生产风险信息；当前 nav+SCAN 不消费其代价作为通行许可 |
| 展示点云或占据 | `web/src/components/scene3d/` | 显示大小/合并不改变导航地图；实际重建是单独后端操作 |
| 语义目标、对象和记忆 | [`perception/`](../../src/perception/)、[`decision/`](../../src/decision/)、[`memory/`](../../src/memory/)；地图绑定注释由 mapd 保存 | 分别处理观测、任务意图、语义记忆；不复制权威占据或最终运动仲裁 |

prune 的表面保留、SurfaceIndex 的候选支撑、局部 Terrain 的碰撞解释和
traversability 风险都是“地面”相关功能，但输出含义不同，不能互相冒充。
未接入产品的 Patchwork++ 地面分割和增量清理实验已移到
[`research/maps/`](../../research/maps/README.md)，不属于当前实时建图或保存链。

**2026-11-01 决定：SCAN 暂不使用独立二维 traversability 风险评分。** 当前 `nav`
Product 已关闭该评分且不启动独立 traversability 进程；SCAN 的配置、请求和恢复路径
也不消费该二维评分。保留三维占据、地面接触解释及已接入的支撑判断；这不表示
固定站姿、坡面或楼梯的真实步态已通过验证。`tracking`、`explore` 等其他产品仍有
各自的 traversability 接线，不能因 SCAN 暂不用就删除它们。

## 导航与停车

| 职责 | 当前实现 | 输出及边界 |
| --- | --- | --- |
| 目标接收、取消、全局重规划生命周期 | [`endpoint/nav/app/goal/`](../../src/nav/cpp/endpoint/nav/app/goal/) | 管目标身份和全局规划请求，不负责跟踪 |
| 全局路线 | [`planning/global/octoplanner/`](../../src/nav/cpp/planning/global/octoplanner/) | 从保存 OctoMap 生成 XYZ 引导，使用三维圆柱通道和支撑检查 |
| 路线推进和局部目标/恢复协调 | [`navigation/executor.cpp`](../../src/nav/cpp/navigation/executor.cpp) | 组织局部规划、跟踪与恢复；终点和恢复旋转的速度由跟踪器生成，不构造 SCAN 网格，也不控制硬件 |
| 局部轨迹 | [`planning/local/scan/`](../../src/nav/cpp/planning/local/scan/)；CMU 为另一后端 | 结合路线、实时几何与预测生成轨迹，或报告 pending/blocked |
| 跟踪 | [`tracking/follower.cpp`](../../src/nav/cpp/tracking/follower.cpp) | 从局部轨迹和状态产生期望速度，不拥有最终控制权 |
| 已有阻塞恢复 | [`navigation/recovery.cpp`](../../src/nav/cpp/navigation/recovery.cpp)、`recovery_search.cpp` | 根据阻塞/停滞选择候选，当前不是完整的静止主动避让 |
| 输入状态、动态观测和预测 | [`endpoint/nav/input/`](../../src/nav/cpp/endpoint/nav/input/) | 组织坐标、新鲜度、障碍与预测，不直接选择产品 |
| 最终控制权与命令仲裁 | [`endpoint/nav/control/final.cpp`](../../src/nav/cpp/endpoint/nav/control/final.cpp) | `FinalControl` 从输入状态取得当拍数据，负责速度整形、限速、SCAN 实测/候选制动及内部停车状态；`main.cpp` 负责构造和接线 |
| 已检查命令的硬件适配和停车确认 | `lingtu-driver` | 处理命令新鲜度、限幅、实际驱动与停止确认 |

规划器说“轨迹被挡住”、跟踪器冻结参考、最终仲裁停止命令、驱动确认实际停车是
不同职责，不因它们都可能产生零速度就删减。最终仲裁中的实测速度制动与下发速度
制动也有不同输入，不能仅按名称判为重复。

普通目标的全局重规划由 `GoalPlanController` 保留任务目标并提交全局搜索；局部轨迹
重规划由 SCAN 负责。重试协调器负责等待和衔接；在途替换规划的状态观察与发起重规划
时的请求接收现已分开：后者交由 `GoalPlanController::replanActive()`，不再重复调用
在途替换检查。输入暂不可用时沿现有运行状态流程等待恢复。

终点转向仍是独立的阶段，Executor 提供目标朝向；跟踪器计算角速度，局部规划器
检查转身扫掠。满足 SCAN 避障配置时，最终仲裁按运行参数检查实测运动和下发速度的制动；
局部转向不再用固定 `0.1 / 0.5 / 1.0` 另做一次制动判断。部分阶段衔接和遥控动作使用
局部规划器的几何查询。自主 SCAN 平移恢复使用正式局部规划和样条跟踪；旋转恢复仍
使用原有转身扫掠检查与角速度生成。

恢复旋转期望速度由跟踪器生成，Executor 负责发起恢复、协调临时目标和报告状态。
CMU 平移保留原有恢复路径和跟踪参数；SCAN 平移使用其正式样条跟踪器。物理验收
结果单独记录。

### 局部规划与受阻恢复

2026-11-03 的自主 SCAN 平移恢复使用下列职责划分。

```text
Recovery    保留原任务，选择一个短距离临时目标，按实测位置判断到达或重试
Executor    将临时 guide、reference 和身份一起交给已有局部规划接口
SCAN Task   用正式 FSM 求解、更新碰撞、重规划并返回样条
Follower    跟踪这条样条，返回候选速度
FinalControl 检查和仲裁最终命令
```

Recovery 从原目标方向附近选择有限的侧向、后向候选位置。候选只是规划输入，只有
SCAN 返回可执行轨迹，执行输出才标记 `recovery_verified`。SCAN 平移分支不再调用
独立的恢复格搜索，也不再另行验证一条几何路径后直接跟踪。机身朝向固定为此次恢复
开始时的实测 yaw，临时参考及其 generation 跨 tick 保留，避免异步求解被每帧重启。

临时目标保留原 `Route` 与任务身份。测得抵达后，下一次局部请求恢复原路线；SCAN
明确无路时换候选，持续没有位置进展时使用既有恢复超时与次数上限。取消、目标替换、
坐标系变化和人工接管沿原执行清理流程撤销临时请求。候选完成不等于用户任务完成。

恢复旋转保留原有三维转身扫掠检查和角速度生成。CMU 保留原路径搜索与几何路径
跟踪，它们与 SCAN 样条接口的差别在局部规划能力边界表达。辅助遥控中已有的边界
脱离分支仍单独使用几何检查，不能统称为 SCAN 求解的恢复轨迹。

实现位于 [`executor.cpp`](../../src/nav/cpp/navigation/executor.cpp)、
[`recovery.cpp`](../../src/nav/cpp/navigation/recovery.cpp) 与
[`recovery_search.cpp`](../../src/nav/cpp/navigation/recovery_search.cpp)。
定向测试覆盖正式异步 SCAN 的临时轨迹复用、实测抵达后恢复原路线、待求解时替换
目标和取消；编译、测试结果和物理动态障碍结果分别记录，软件接线不能代表实机通过。

**2026-11-02：正式 navd 暂停楼梯自动接入。** 路线不再读取楼梯标注并附加楼梯段，
也不再发送楼梯模式切换或等待其恢复。地图和标注保留，试验原型仍供离线研究；普通三维
规划不等于楼梯识别、上下楼运动模型或实际通行已经验证。

最终制动现在优先使用当前路线的机身，没有路线机身时使用配置机身，和局部路线使用的
机身一致；局部另有可选的实测关节几何输入，现场路径当前没有接入该输入。运行参数不是
实测制动能力。此集成不是上游 SCAN 默认行为，参照固定版本
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。本地回归以同一动态障碍检验：旧固定制动
否决、配置制动允许；修改后局部保留扫掠判断，最终控制在同一条件允许，并在配置制动
距离不足时限制命令。这是代码级证据，不是物理制动测试。

2026-11-02 的全局边界对照：修改前 `start_evidence_smoke` 中同一张合成保存图的
空闲目标，加入实时占据查询后规划失败；修改后全局请求不再接收该查询，同一静态目标
可规划，静态墙和终点连接障碍测试仍通过。局部输入测试另确认实时占据仍在碰撞图中；
SCAN 的障碍重规划、停车与清图恢复组件测试通过。该对照不是动态物体的物理联调。
在原 `903room_v6_corridor` 保存图上，五个固定目标各查询一次均生成路线；单次查询
约 91–363 ms，另有约 27.4 s 的地图读取和准备。结果保存在
`build/global-static-routing-20261102/results.jsonl`，不是同条件性能基准或实机验收。

## 已确认需要整理的地方

| 分类 | 当前事实 | 影响与下一步 |
| --- | --- | --- |
| 功能缺口：实时地面反证 | 实时占据和自由空间由局部使用。准备阶段的 `SurfaceIndex` 及其推断支撑不随每帧直接增删 | 确定地面被新观测否定时由 nav 如何处理支撑；mapd 继续拥有观测事实。不能把单个自由格等同整块悬崖 |
| 功能缺口：静止后有人靠近 | 现有制动检查覆盖反应和停下过程；普通恢复按 blocked/stalled 触发 | 尚无“当前位置未来能否等待”完整行为；可选主动避让要另行设计、仿真验收并保留最终仲裁 |
| 产品模型边界 | 全局是圆柱通道；局部使用所选 Body。Go2 为 17 部件，Thunder 为 8 部件。早期 Thunder 38 部件地面误拒为历史实验 | 不能称所有层已用同一逐部件模型；先修已复现的接地与低障碍区分，再谈完整步态覆盖 |
| 地形接口生效范围易误读 | 产品关闭 traversability cost，SCAN 解析/请求也不使用它 | 文档应表述为独立能力；不能说打开一个开关就获得低物可跨的 SCAN 行为 |
| 保存来源命名不清 | 状态仍使用 `native_slam_dds_control`，但保存提交由 mapd 协调；SLAM 负责提供快照 | 区分“能产快照”和“当前 Product 可提交保存”；该字段可能为外部接口，改名须先核查消费者和契约，不把名称残留当作已证实双写 |
| 在线与离线建图 | 在线调度与 backend 均在 `online_mapping/`；离线优化使用 `map_optimization/` | 两者共用文件读取，不是两套并行的在线估计器 |
| 目标状态估计 | Gateway 的就绪状态用于展示；普通目标是否接收由 navd 回执决定 | 缓存状态不再挡住普通目标；控制租约和保存位置地图身份仍在 Gateway 边界核对 |
| 动态障碍与全局重规划 | 初次规划、预览和重规划只接收保存地图几何。SCAN 与最终控制处理实时占据 | 长期封堵触发带替代通道偏好的全局规划尚未接通，不能认为已经实现自动跨通道绕行 |
| 控制权表述与入口 | HTTP 客户端租约、WebSocket 连接占用和原生运动租约分别存在；WebSocket 不查询 HTTP 租约 | 明确网页客户端占用与原生控制权各自的作用，再统一对外行为，不能直接删除原生租约 |
| 局部与最终机身上下文 | 最终制动与局部路线统一使用当前路线机身；无路线时使用配置机身 | 可选实测关节几何尚未接入现场控制；不能声称已逐拍使用实时关节姿态 |
| 显示缺口 | Web 主要显示原始占据，实际推断支撑面尚未作为同一权威图层接入 | 后续应显示原生使用的结果，不能由浏览器另算一套地面 |

文档中的旧全局 Body 和 endpoint 姿态描述已按当前工作区修正；地图保存 API 的
说明使用“请求地图保存”，不再把 SLAM 描述成永久地图事务所有者。

用户已确认第一版允许多个页面同时查看，同一时刻只允许一个页面控制机器人。HTTP 与
WebSocket 如何共享该占用、如何与原生控制租约衔接仍需实现和验证，不能将当前行为
描述为已经满足此要求。

## 明确的未完成验证

该分类依据源码和现有记录，不证明新的 Product、仿真或实机通过。下一步按
[地图交付计划](map-delivery-20261031.md) 与
[机体碰撞模型](navigation-body-model-and-motion-posture.md) 中已有证据推进；
读取目标机实际 RunPlan 后才能判断现场选用了哪些模型和功能。
