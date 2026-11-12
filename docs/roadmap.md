# Roadmap

**Status:** Active product roadmap
**Audience:** Product, runtime, release, simulation, and field maintainers
**Runs on:** Planning across local, simulation, release, and supported real-robot targets

This page is the single work queue for the current delivery cycle, unfinished
qualification, and delivery gaps. Only the current task card and its follow-up
queue decide what runs next; domain priorities below do not create parallel
"current tasks".

Product purpose, user workflows, and deliverable limits belong in
[Product](product.md). Shipped behavior belongs in the owning code and current
documentation; architecture and evidence levels belong in
[Architecture](architecture.md) and [Testing](testing.md).

## Current delivery cycle

2026-11-12 客户 Agent 交付子项记录在 [客户交付与响应清单](design/customer-agent-delivery.md)：
Python 环境准备、普通建图／导航的基础对话、模型配置、响应测量及待验收功能。
客户步骤见 [部署与首次使用](customer-deployment.md)。这些本地改进不替代下述原生／实机资格。

2026-10-31 地图构建、恢复与导航验证按
[地图交付计划](design/map-delivery-20261031.md)推进，优先处理 10 cm 路网与补面尺度，
再对照真实地图、优化保存性能并验证恢复续建。计划记录了当前测试失败与实机资格边界。

2026-10-26 决定：**组合碰撞模型作为已适配平台的正式规划默认**。本地 Go2 real、
Thunder real/sim 已绑定平台模型；余量统一取平台配置、只扩水平面一次。2026-11-01
已删除 endpoint 的实时 roll/pitch 机体快照；当前碰撞使用固定名义姿态与查询 yaw，
尚无在线关节 FK。当前全局使用简化圆柱通道，所选组合模型用于局部与执行碰撞检查。
Go2 附件实测和 NX 部署分别记录，不能混称全部验收。
代码、模型来源、验证与后续范围见
[导航碰撞模型与运动姿态](design/navigation-body-model-and-motion-posture.md)。
本轮正式 Thunder 名义模型 DDS＋MuJoCo 短程仍起步阻塞：全局成功、局部零命令，
本地模型接入不等于已通过执行验收。先关闭该固定失败，再安排新包实机运动；
证据见 [错误记录](operations/error_record.md#2026-10-26正式名义组合模型-dds-物理起步仍阻塞)。

此前同日命名记录：“URDF 分部碰撞模型”统一称**组合碰撞模型（Compound Collision Model）**。
这是已有的多形状组合表示；灵途实现的是本地几何与导航接线，不以内部 `BODY2` 格式名
宣称新算法。上游依据、术语映射和当前缺口统一见
[导航碰撞模型与运动姿态](design/navigation-body-model-and-motion-posture.md#名称行业来源与实现边界)。
该命名步骤当时只更新名称；随后模型接入以上述最新决定和对应验证记录为准。

2026-10-23 历史推进：在已通过的单OBB＋SCAN＋Thunder MuJoCo直线闭环上，
按森哥要求组队完成首次/连续任务响应、宽敞转角连续运动和复杂平地场景验证。
代码分工与固定输入记录在 `build/navigation-team-20261023/PLAN.md`；
原直线证据保存在 `build/navigation-closeout-20261023/`。正式Product整链路、
坡地楼梯与实机验收仍单列未完成，不以组件测试代替。此项导航验收目标尚未完成，
下方历史任务缺口保留。

更新：2026-10-22。当前任务和优先级以本节为准；下方日期记录与研究计划提供背景，
不再各自维护另一项“现在正在做”。

**产品主线：建图 → 保存/重载 → 定位 → 自主导航 → 停止。**
森哥于 2026-10-19 要求按勾选计划推进 SC-02/03/04 场景适配。
2026-10-21 当前优先级调整为**公开原始数据集建图 → 保存射线地图 → 原生三维路径规划**；
两组完整地图已生成，三条长路线在原地图上仍未找到路径；独立短路线已修正并通过几何、
扫掠和运动约束检查。长路线已查明局部保存观测缺失及擦地射线误清除；原证据 Corridor
重建恢复 226 个检查区地面体素，但原两条路线仍失败；更密 GEODE 地图已构建并复测，
同一区域占据表面 150→356，原长路线仍失败、短路线仍通过。
下方当前任务卡维护下一步；对话现场准备、玻璃人工约束和 SLAM 退化修复保留未完成。
首批**真实退化方向诊断和状态输出（D1/D4）**已完成本地检查；真实 AIST/BikeTunnel2 回放暴露持续漂移和失败前漏提示，因此 D1 重新保持未完成，D4 保留完整链路时延验收。2026-10-20 森哥调整本轮目标为**团队协作完成对话控制的首轮修复，并准备实机验证**；D1 场景覆盖与 D2 退化恢复保留未完成，后续按
[实施计划书](design/slam-scene-adaptation.md#执行清单) 继续推进。轮式平台、目标现场录制数据、
相机与标定暂未提供；本地实现与现场验收分别记录。此前语义任务的未完成项保留在下方。

此前先完成环境与代码基础，再接入真实机器人。视频的固定版本环境、
标准构建与运行库安装、Product 生命周期和断流恢复已通过本地验证。
**相机 → 共同观测 → Follow 的本地验收准备已完成**：tracking 视频变体、同步/异步 SDK、
真实 SHM 到 Follow API 的组件集成，以及真实尺寸合成输入的编码并行负载均有结果。
此前推进**自然语言任务的仿真与实机静止验证**：技能契约与共同发现、
Agent 实际模型配置、普通 HTTP/MCP 指令进入 Agent 的受理链，以及 Agent 运行查询和
定向取消已经接入代码。原生语义观察视点的客户端桥和“目标不在视野时主动搜索”链路
已有本地契约和原生桥接证据，完整导航仿真仍未通过。2026-10-18 已重新连接 NX
现场 NX，通过正式 ProductControl 启动已安装 `.93` 的 `map/camera`，
真实 RGB-D、SLAM 和连续预览正常；实采三组 RGB-D 已用生产检测/投影函数在开发机回放，
每组检测到一人。当前源码未部署，板端在线语义检测、人物身份与运动资格仍待验证。
“跟随我”和“帮我找一瓶水”是后续功能验收目标；后者继续以 VL-Nav 为主要方法参考，
真实模型、导航执行和视觉确认分别留证。TaskPlan、任务经历和现场资格的缺口仍保留，
机器人未接入时顺序推进可本地完成的部分。连续运动诊断和地图页面结果保留；
903 原始地图任务仍待原图，不使用派生图替代输入。
本轮完成不等于实机导航验收完成，也不自动批准部署、运动或发布。

### 推进规则

1. **一次一个主要验收目标。** 分工可以并行查证、构建和审阅，但服务于同一目标。
   新问题记入队列；用户调整优先级时，更新本节并保留旧任务的未完成状态。
2. **开工前写清输入与完成条件。** 固定地图、起终点、参数、代码/实际二进制版本和运行平台。
   脏工作区的 HEAD 不能单独代表被测源码；保留本轮相关改动或已有构建快照。
3. **用证据推进。** 复现 → 最小修改 → 同条件复测 → 对照结果 → 关闭任务。
   失败记录保留；需要调整假设、输入或目标时说明理由，不能挪动目标点或放宽安全条件换通过。
4. **完成状态只针对声明的层级。** 已写代码、离线算法通过、仿真通过、实机静止通过、
   实机运动通过分别记录。每项任务使用“待执行 / 执行中 / 待条件 / 待验收 / 已验收”。
   “已验收”必须附结果文件及其证明范围；没有跑过就写“未测试”，不用估计完成百分比。
5. **每轮留下可续接的结论。** 固定汇报：本轮完成和证据、仍失败或缺少的条件、下一项具体动作、
   是否需要森哥提供输入或现场配合。已有授权范围内的本地工作连续推进，不反复申请同一许可。
   遇到阻塞，记清缺什么，并继续同目标下不依赖它的工作。
6. **先审核，再发布。** 按森哥 2026-10-14 的要求，待发布材料先整理在本地文件夹，
   列清范围，明确确认后才上传。测试通过不等于获得发布确认。现场动作遵循现有
   ProductControl、就绪和运动验收约定。

当前工作由 Codex 推进、验证和记录；产品取舍与发布确认由森哥决定。
每次续接先读取本节，结束前更新状态。这里记录推进方式，不另建任务服务、数据库或定时任务。

### 当前任务卡：数据集三维建图到原生全局规划

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 两组完整原始录制已用原生 LTLIO 回放、保存并完成 saved-ray OctoMap。2026-10-22 去掉强制全朝向外接圆后，同一 GEODE 1,301 帧地图上的原长请求找到 367 点路线，独立 route/sweep/motion 全通过；原 303 帧地图尚未按新配置重测，Corridor 两条长请求仍保留失败。新通路宽度的工业园 MuJoCo 物理组件执行通过；GEODE 物理执行、完整定位/导航 Product 和实机仍未验收 |
| 固定输入 | SuperLoc Corridor02 全部 8,859 云/178,499 IMU、GEODE Stairs Alpha 全部 3,456 云/34,556 IMU；使用各自标定及保留扫描末端的载体。原始 GT PCD 不替代生成地图；固定请求/机器人几何见 [manifest](../build/dataset-mapping-20261021/manifest.json) |
| 已完成 | 原生 replay 增加可选 `--save-map`，复用现有保存链；Corridor 保存 10,644,022 点/495 关键帧，GEODE 保存 6,243,593 点/303 关键帧，manifest 均完整无丢弃。地图按真实保存射线构建，没有填假地面/未知自由空间；提供真实三维查看、三视图和实际短路径图，Arial 10 pt |
| 证据 | [原始实验](../build/dataset-mapping-20261021/README.md)和[修正与真实断口证据](../build/dataset-planning-fix-20261021/README.md)。细 Z 搜索和米制边约束通过原生 5 项、Python 5 项及定向 lint；各数据集分别展示三视图和当前路线。完整地图无闭环图优化、此次未走 Product prune；条件位置 RMSE Corridor 1.546 m、GEODE 0.769 m，不作为无条件精度或现场指标 |
| 明确失败 | Corridor 140→180 s、180→300 s 和 GEODE 15→45 s 在原图仍 open 集耗尽。Corridor 首段存在支撑断口：局部实测地面被擦过体素上方的射线误清除；GEODE 局部 687 个 PCD 表面体素中 537 个无保存扫描命中，清单完整不能替代观测覆盖。原短路线坡度失败已修正。旧 WSL GEODE 749 姿态后 LOST 的平台差异未隔离；更密保存已完成 1,301 帧，但旧构建曾内存不足，失败保留 |
| 射线修正结果 | builder 0.3.2 的原生 ray/store 2 项通过；完整 Corridor 同证据重建成功，原检查区 226 个 free→occupied，代表体素概率 0.279→0.566；原两条路线仍 empty_path。起点完整连通分量 3,754 与 A* 一致，目标不在其中，前向边界因地面支撑失败而非机体碰撞。实际源点/射线图、叶节点查询和失败日志保留 |
| 更密保存对照 | GEODE 1,301 帧、16,783,728 点的地图构建成功；687 格检查区 occupied 150→356，原长路线仍 empty_path，短路线 route/sweep/motion 均通过。位姿逐值不变，但帧选择与原 303 帧只重合 131 帧，并非严格超集，不能把每格变化单归因于密度；未改现场默认 |
| 本轮保存修复 | 10-21 的两次密集保存失败保留于[原记录](../build/map-repair-20261021/README.md)。10-22 已改为流式 PCD、只生成最终紧凑输出云、直接初始化 ICP target；异步 worker 只暂存，owner 确认源未重置后才替换旧图。当前源码 Linux 四目标正常构建及保存/语义/ICP 三项回归通过；固定 495 帧/10,644,022 点与 2,816 帧/37,196,794 点 capture 均完整保存并逐点验证，保存阶段 24.822/89.216 s，进程峰值 RSS 0.847/2.581 GB，定位 target 均可用。[新保存记录](../build/map-save-20261022/README.md)。这是固定 capture IO 证明，不是完整传感回放或现场内存；**生产采样默认仍为 1 s**，原密度实验存在 IMU deskew 混杂变量，未据此改估计器或推广 0 s |
| 下一动作 | 10-22 的全局 XYZ 通路/局部机身分工及物理组件结果保留；10-23 已按用户选择接入单 OBB 的 OctoPlanner3D + SCAN 姿态主链路，当前组件与长路线证据见下方最新状态，不沿用旧版本的物理资格。继续处理仍失败的 Corridor 地图和同请求复测：495 帧闭环校正在同一早期测量基准下消除 A/B 邻域 12/9 条晚期穿面射线，A新增真实命中、B仍缺，尚需重建整图。静止覆盖、完整传感流程保存与 GEODE 物理场景仍待验证；不补假地面或自由空间、不缩小实际机体、不放宽坡度 |
| 后续资格 | 隔离跨平台结果，再推进同场景原生仿真执行/重载/定位闭环；本轮无 Product 启动、地图激活或机器人运动，不能勾掉现场资格。玻璃三维人工约束仍按下表待执行 |

完成状态：

- [x] 两组公开原始 LiDAR/IMU 完整回放并原生保存地图。
- [x] 保存关键帧/射线证据完整，使用真实射线构建原生占据地图。
- [x] 同图固定起终点执行原生规划，失败和实际短路径均保留。
- [x] 三维查看器、独立三视图、Arial 10 pt PNG/PDF 和过程记录。
- [x] 修正 ICP 地图下采样溢出和额外副本，通过窄回归与同一真实大图加载。
- [x] 完成候选密集保存评估与失败留存；生产默认保持 1 s，未推广未通过的候选。
- [x] 分块保存和直接定位 target 初始化；固定 1,064 万/3,719 万点 capture 保存、全点一致性和异步 reset 保留旧图回归。
- [ ] 静止覆盖、完整传感流程保存、完整重访校正地图与固定路线复测。
- [x] 同一 GEODE 加密地图的原长请求，按新全局通路宽度找到路线并通过独立三维核验。
- [ ] Corridor 两条失败路线的支撑/自由空间断点修复复测；原 303 帧 GEODE 地图的新配置回归。
- [x] 解决返回路径与连续运动坡度诊断不一致；原 GEODE 短路线 route/sweep/motion 均通过。
- [ ] GEODE Windows/WSL 行为差异隔离、正式 Linux/目标板资格。
- [x] 当前原生导航的工业园静态长路线和行人横穿 MuJoCo 物理组件验收（真值定位、合成射线地图）。
- [ ] 原数据集场景执行、完整 Product 定位闭环及实机保存重载、导航和停止验收。

2026-10-21 导航组件补充：已冻结 GEODE/旧 Garage 地图、原始请求和实际源码，
使用正式 Go2 RunPlan 与旧诊断配置分别调用共享全局核心、SCAN 和 Follower/Executor，
结果见[固定地图组件实验](../build/nav-dataset-execution-20261021/README.md)。
GEODE 标准碰撞输入可生成样条并完成理想 XY 反馈；Garage 正式参数返回35点但局部仍被阻挡。
Garage同图原叶查询确认：一个0.10 m占据叶在全局中心圆柱外、在SCAN后圆柱内，
说明当前起点的全局/局部机体外形合同确有不一致；不以删格或放宽局部尺寸处理。
旧诊断 Garage 生成轨迹并移动，但从抬高0.10 m的吸附起点开始、20 s未达到高度条件。
全部测试固定Z，无物理/传感反馈，不能勾掉上面的原生仿真或现场资格。
旧 GEODE 起点碰撞已定位为实验额外膨胀造成，修正的是fixture，没有修改生产规划算法。
2026-10-22 初轮修正正式 SCAN 组合的包络与起点接入，见
[同图复测记录](../build/nav-envelope-20261022/README.md)。全局 XYZ 无朝向搜索使用
至少 `local cylinder radius + offset` 的保守圆，上下净空按局部碰撞网格向上取整；
真实起点及其到首航路点的连接都接受碰撞、支撑和运动约束检查，反馈从原请求起点开始。
Go2 正式参数仅全局半径 0.155→0.44 m、上净空 0.24→0.25 m 改变，局部机体不缩小。
同一 Garage 原请求现在明确 `start_snap_exhausted`、不进入局部执行，不再返回无法起步的35点路线；
这是消除错误放行，不是 Garage 导航通过。GEODE 原请求仍返回26点/2.76891 m，
原起点接入及25段 route/sweep/motion 验证通过，SCAN ready；理想 XY 反馈5.64 s达到组件目标条件，
碰撞计数0，取消输出为零。Z固定、终点高度差0.10021 m，不是三维物理或实机到达证明。
Python配置54项与原生定向5项均通过。保守圆可能拒绝特定朝向可穿过的窄通道；
后续应基于实际失败证据判断是否扩展带朝向的全局搜索，不缩小局部包络来提高返回率。
导航侧下一步是同源物理场景的高度跟踪、定位与停止验收；地图侧保存、完整Garage、
多层楼梯和长路线任务保持原来的未完成状态。

**2026-10-23 当前碰撞方向与接入状态：** 森哥确定主组合 OctoPlanner3D + SCAN
采用一个随 yaw 旋转的三维 OBB；已有共同 `collision::Body` 从全局带朝向请求贯通
Route/Executor、SCAN、Recovery 和原生最终指令仲裁，不在线加载 URDF 或执行关节 FK。
工厂复用 RobotConfig 的长宽、水平硬余量及上下净空；Go2 当前源码在2026-10-25按用户要求
将每侧水平余量改为0.05 m，盒体为0.86 × 0.41 m，机体系 Z ∈ [-0.10, +0.24] m。
水平余量只应用一次，不新增垂直余量、依赖或调参开关。该配置修改未部署；此前现场与
仿真记录保留当时0.10 m余量和0.96 × 0.51 m盒体的历史含义。
局部消费已有 raw measured occupied/known-free 三维证据；动态预测继续参与，预测圆柱
保守包成 AABB，与同一 OBB 按时间和扫掠检查。完整 roll/pitch、腿部步态范围与实机包络
仍需独立测量和验证，不能由固定 yaw 盒体推出。

这是主组合的统一几何合同，不覆盖所有兼容后端：FAR + SCAN 保留 FAR 的 XYZ 全局候选，
SCAN 局部/恢复/最终仲裁使用 OBB；CMU 保留既有 XYZ/空 Body 兼容路径。
楼梯保留模式切换、梯段朝向、限速与取消，并由同一局部 OBB 验证转身/运动。
当前 Executor 116项、SCAN Body 12项、SCAN planner 101项、姿态路线14项及 Recovery sequence 16项本地回归通过，
不等于楼梯物理或现场资格。Executor 显式 Body 覆盖已通过外部时钟推进后的实际运动检查。
新 OBB 的30 m直线组件运行53.08 s到达；双墙绕行重复56.20 / 56.45 s到达，
OBB占据命中、无效查询及最终几何制动触发均为零，取消下一拍为零。
收尾合并并行轨迹衔接修改后的最新组件运行，直线55.15 s、双墙59.18 s到达，
上述命中/无效查询/制动计数仍为零；速度波动和较慢用时如实保留，不宣布平顺性全面改善。
首次全朝向搜索超过300 s的问题，已用三维候选加同一OBB整段核验和朝向回退处理；
最新双墙全局冷请求2.380 s，原总展开预算不变。该结果仅证明固定合成场景的理想平面组件执行，
既有实测地图连通性失败、NX时延及物理执行仍未据此关闭。
证据保存在 `build/obb-unification-20261023/`；几何合同、后端边界和历史 URDF 实验入口见
[机体碰撞设计](design/navigation-body-model-and-motion-posture.md)。旧物理或双圆柱结果不移作新 OBB 的验收证据。

**10-23 后续轨迹修复：** 已定位并修正稀疏参考引起的直线提前摆动、移动重规划的
停到停时长假设、单独截断轴速引起的方向偏差，以及规划与执行轴速能力未接通。
原 OBB、0.6 m/s、2 m/s²、0.35 m/s 侧向能力及制动条件保持不变。
同初始状态、同全局路线、同原 plane 双墙世界的真实 Thunder MuJoCo 配对：修前45s未到达，
修后19.00s到达，0最终几何拦停、0非法接触/3962物理子步，终点误差9.93mm。
到达后0.80s实测最大XY速度0.00206m/s；局部线到全局折线最大偏离仍有21.1cm。
证据与中间失败保存在 `build/obb-tracking-20261023/`。这是策略驱动的原生组件物理结果；
旧 runner 在拦停后的 applied-command 协议缺口仍限制修前后段误差归因，修后未进入该路径。
不关闭完整 Product、Go2 实机、NX 实时或其他地形资格。

以下保留 2026-10-22 的实验与当时接入状态；其中 URDF 正式方向、未接线和
全局 XYZ/双圆柱默认的描述已由上方 10-23 决定更新，不是当前待办。

2026-10-22 朝向/实际模型对照：已用安装版 Go2 URDF 的34个碰撞几何完成离线三维姿态查询，
并对固定高度 XY+yaw 搜索输出[对比图、原始数据与动画](../build/body-envelope-20261022/index.html)。
0.60 m门洞从外接圆拒绝变为朝向双圆柱/URDF姿态路径可达；0.45 m仅标称 URDF + 实验余量可达，
髋关节外展探针则再次碰墙。Garage原体素与base仍交叠约13 mm，未删除原图证据。
这不是正式规划器或步态验证；生产 yaw 接入、安装附件、运动包络与共享模型查询仍待完成，
顺序和边界见[机体碰撞设计](design/navigation-body-model-and-motion-posture.md)。
当时森哥确定实验方向为URDF分部模型+朝向；首个原生交付为模型导入、共同C++姿态/扫掠查询与固定场景探针，
随后贯通XYZ+yaw搜索、路线转换和局部执行。接入入口已核实，实施计划已更新。

同日原生对照补充：共享C++全局核心已接受显式固定姿态分部模型，并使用XYZ+yaw搜索。
[四场demo及原始路线](../build/native-body-routes-20261022/index.html)中，历史0.44 m圆柱绕路
4.211270 m，新分支2.800000 m；初始90°穿门与L形转角分别返回2.441421/2.024264 m，
三个路线均通过返回后分部扫掠复查；0.28 m通道拒绝实际起始姿态。
已完成34个primitive保守OBB导出、共同SAT、三维地图扫掠和带朝向索引；
正式RobotConfig选择、端点到SCAN的yaw消费和动态姿态仍未完成。此项是显式请求的
核心实验，没有切换下述正式参考通路/局部分工或覆盖其配置。单次穿门规划12.7 s，
还不满足实时要求；当前固定关节及缺相机支架的模型不具备现场资格。

2026-10-22 执行补充：新构建的原生 `navd` 已在 MuJoCo 真实物理反馈中完成两项
原始 manifest 验证，见[本轮修复与证据](../build/navigation-repair-20261022/README.md)。
行人横穿用例实走 5.71873 m，遇阻停车后继续原目标，最终 XY 误差 0.177995 m，
3,942 个物理检查步无实体接触；静态工业园原起终点相距 59.94164 m，实走 67.05093 m，
最终 XY 误差 0.301289 m、原生 reached，33,131 个物理检查步无实体接触。
两项都取得终态零命令在物理桥施加的精确 ACK，并清理全部拥有进程；这不等于独立测得
机体完全停稳。测试使用原有 ONNX 策略、声明的合成射线保存地图和仿真真值定位，
未放宽阈值或挪动目标；不是原失败数据集、LTLIO、完整 ProductControl 或实机验收。
新增 SCAN 同目标动态等待/重试回归及关联 17 项通过，尚未加入通用时空等待优化器。

GEODE 原长请求的完整连通分量诊断也已完成：67,454 格、队列耗尽，目标不在其中，
最近可达点距目标仍 13.8496 m。该点向前九个邻居都存在支撑失败，其中八个同时有
机身碰撞；另一个机身净空有效，但已观测顶层表面不在当前位置要求的支撑高度内。
因此不能统一归因于“地面没保存”或“搜索预算不足”。[原叶节点与边界记录](../build/navigation-repair-20261022/geode-frontier/README.md)
保留实际占据/自由/未知证据；原三条长路线和楼梯执行资格继续保持未完成。
同一断口随后用当前源码检查 98 个实际可达前驱与 6 个候选高度、共 588 条连接；
四个抬高候选虽有支撑却均有机身碰撞，未找到被 26 邻域遗漏的合法连接。
此处不能靠增加搜索邻居消除拒绝；独立合成台阶的邻接表达缺口仍需另行处理。
进一步对照正式局部双圆柱发现，上述碰撞属于保守外接圆：使用半径 0.25 m、
前后偏移 ±0.19 m、上下净空 0.10/0.25 m，在原地图同高 +X 的 20 cm 连接上，
实际朝向包络连续扫掠和严格支撑均通过，未知机体体素为 0。
[朝向包络对照](../build/navigation-repair-20261022/geode-endpoint-probe/oriented-envelope.json)
确认存在外接圆误拒绝的具体例子。随后用户要求避免全局朝向搜索和执行链扩建；
重新审查确认已有局部和最终命令链负责实际机身及转身扫掠，因此撤回该扩大方案。
当时改为全局检查实际通路宽度，局部保留完整双圆柱；Go2 全局半径 0.44→0.25 m，
Thunder 0.65→0.40 m，显式更大值仍保留，上下净空、支撑和坡度不变。
这是全局参考通路与局部运动验证的分工，不宣称参考通路包含每个转角的可执行朝向。
本轮[简化修复记录](../build/nav-corridor-20261022/README.md)包含 Python 53 项、
全局 6 组、局部 2 个新增和 6 个相关用例、端点配置与真实 Grid 纯旋转集成 2 项通过。
同一 GEODE 加密地图的原长请求已返回 367 点，终点误差 0.055576 m，独立核验通过；
只调整全局半径 0.44→0.25 m 和上净空 0.24→0.25 m，原始起终点及其他条件保留。
离线规划进程 115.361 s（含地图加载），不是实机规划时延验收。原 303 帧图未重测，
Corridor 两条长请求和 GEODE 物理运动仍未通过本轮验收。
新的 Thunder 0.40 m 通路宽度 MuJoCo 工业园用例实走 67.44638 m、目标 XY 误差
0.303272 m、32,616 步无实体碰撞，短暂近场停车后恢复到达；转向与最终零命令 ACK
也保留。原 manifest 显式半径为 0.95 m，本次派生用例仅改此字段为 0.40 m，
未改目标、阈值或实际局部机体，不把它写成相同半径下的性能 A/B。

森哥进一步要求每次结果同时展示其他算法，具体规则见[算法结果对照](testing.md#algorithm-result-comparisons)。
已核对的官方 Elevator-LIO/GenZ-LIO 与当前 LingTu 的共同样本结果、历史903规划对照及瓶颈
集中在[对照报告](../build/algorithm-comparison-20261021/README.md)。GEODE当前概率平面
0.292340/0.313724 m（IMU/LiDAR假设）仍高于官方GenZ的0.247597/0.277696 m；
Slippage上的排序不同，不能以单场景替换整套系统。内部GenZ机制适配与官方完整算法分开标识。
当前Corridor完整官方外部定位基线、GEODE/Garage同地图同几何的独立外部规划基线仍未测，
这些缺口必须出现在结果表；历史不同默认参数的返回率不能作为当前算法优劣结论。

论文材料按 [统一材料入口](research/paper-materials.md) 收集到仓库外的日期快照。
程序、结果与失败记录一起保留；收集完成不改变以上验收状态。

### 保留未完成：对话跟随与搜寻的首轮控制及实机准备

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 本轮对话控制的本地收尾已验收，独立代码/架构审阅通过；10-20 开机后曾连接 Go2/NX，最后成功状态为 `.93` stopped；10-20 20:37 及 10-21 原地址 SSH/HTTP 均不可达，当前在线状态未确认。当前修复未部署，本轮静止采集待正式启动，监督运动未验收 |
| 产品与团队 | 固定现有 `nav/semantic`；产品经理、控制架构、前后端实现、现场验收和独立审阅分工，主代理集成。各角色与首演范围见 [团队决议](research/plan.md#团队首轮决议与实机顺序2026-10-20) |
| 本轮改动 | Agent owner 全句识别查询/保持/取消并保持 run；空闲和终态查询如实显示；HTTP/MCP 的 control_only 约束阻止盲态/本地仍活动时启动新任务；停止未确认不能新建，真实受理的新 run 不因旧轮询隐藏；成功任务取消不改写结果；Web 保留输入、草稿和 HTTP 409 具体拒绝原因；未绑定我/他拒绝选人；修复缺省标定校验与旧测试夹具 |
| 完成证据 | [本轮验收](../build/dialogue-control-20261020/README.md)：最终 owner/HTTP/MCP/Agent 组件/标定合并 158 项及 59 subtests、网页 46 项通过，类型/lint/API 生成一致性及网页生产构建通过；首轮 Follow/语义/Agent 组合 121 项保留。记录跨层问题发现和修复，候选源码另留选定文件快照 |
| 测试入口 | [10-21 启动说明](operations/dialogue-test-20261021.md) 区分 Windows 本地回归、旧 `.93 map/camera` 静止检查与新版本 `nav/semantic`。当前工作树定向回归 68 项、网页控制 46 项通过；它们与既有测试重叠，不累计为新场景验收。结果保存在 `build/dialogue-start-20261021/`。纠正旧 doctor 示例不支持的 `--robot` 参数及视频 Rust/Cargo 构建说明 |
| 方案讨论 | 10-21 森哥确认“一个执行中运动任务，明确目标直接切换，有歧义先停止旧运动再询问”。[已确认交互](research/plan.md#已确认的任务切换交互2026-10-21待实现) 由同一后端 Agent owner 管理交接，网页请求函数与 Python SDK 封装同一 HTTP API；等待澄清不自动恢复旧运动，自动切换尚未实现 |
| 实机缺口 | 森哥已明确授权代理执行正式 `map/camera` 启动，但实际 `sudo -n ... switch` 返回 `a password is required`，部署密钥的 root SSH 也被拒绝；需终端交互提权。控制 HTTP 不接收 variant，无法确认守护进程固定相机变体。相机使用宇树 EDU 架子，实测外参仍未验证。旧 DINO/人物 YOLO TensorRT 引擎有既往数值失败，fixed-conv 人物 ONNX 待 NX 新对照。NX 缺精确 GTSAM 4.3 与锁定 ARM 视频 runtime，本地 x86 产物不能替代；此前完整 calibration 39 项及 18 subtests 通过仅证明配置校验，未改物理外参 |
| 下一动作 | [开机后现场记录](../build/dialogue-field-20261020/README.md)：恢复目标连接和交互提权后正式启动 `.93 map/camera`，另存静止 RGB-D/SLAM/authority/停止证据；7,990 文件源码候选已冻结且两份正式 real RunPlan 纯解析通过，fixed-conv 人物诊断候选及同输入 CPU 参考已备齐、待上传确认与 NX 对照。`lt_pgo` 漏包已修且打包自测通过。ARM 构建与新版本沿正式候选/安装流程验收，其他身份算法/多轮功能保留缺口 |
| 仍未交付 | 语音、Follow pause/resume、原地观察对话任务、多轮目标纠正/后继任务、长期身份算法补齐及整机资格；不因本轮控制回归通过勾掉 |

### 地图编辑与人工障碍约束（2026-10-21）

本节维护地图编辑工作的完成状态与后续顺序；上方对话/现场任务和下方 SLAM 的
未完成资格保持原状。当前界面验收使用保留的 903 点云采样和封闭浏览器夹具，
不能据此勾掉原生完整地图、仿真或实机运动验收。实现与截图见
[本地编辑器记录](../build/ui-map-editor-20261020/verification.md)。

已完成的本地功能：

- [x] 紧凑地图入口按名称自适应，沿用已确认的 A 版浅色保存窗口。
- [x] 三维范围删除/保留、原图对照、撤销/重做；不改原始地图，不生成虚构地面。
- [x] 选区预览点数/比例、整段高度提醒；已删除点不重复计入。
- [x] 保存与重建分阶段反馈，保留目标名称和服务端点数；重建失败可查看已保存点云、
  仅重试重建；查看不会激活新地图。
- [x] 草稿只在本浏览器保存名称、源地图身份和编辑历史；同名/同版本/同坐标系才允许恢复，
  保留撤销后的重做记录。保存前先保留待确认请求，重入后不能改名重复另存；
  保存成功或明确放弃普通草稿后清理。旧版本或旧坐标系的几何不能用于当前地图。
- [x] 查清现有人工体素编辑的玻璃支持边界，并同步地图模块与 API 说明。
  此项是源码查证，不是玻璃避障验收。

未完成项按以下顺序推进，不新增独立地图服务或任务引擎：

| 顺序 | 状态 | 要做什么 | 关闭条件 |
| --- | --- | --- | --- |
| 1 | 待执行 | 在现有 mapd/MapStore 中保存独立的三维人工障碍。首版玻璃用墙段两端、厚度、上下 Z 表示；附材料/名称，绑定地图身份，原始 PCD/rays 保持传感证据含义 | 增删改、保存重载、从源重建后几何一致；删除约束恢复底层传感证据，不写成 free；地图身份失配时不套用 |
| 2 | 待执行 | 同一份约束接入已声明版本的全局几何、实时局部碰撞和最终运动检查。通过现有 ProductControl/地图加载约定确认生效，不假设运行中的 nav 自动重读文件 | 固定输入上全局路径不能穿玻璃；局部候选和最终检查也拒绝穿越；传感器无返回不能清掉人工障碍，且不能误阻挡不同 Z 的有效通道 |
| 3 | 待执行 | 前两项有原生证据后，在原地图工具内接“标记玻璃”：俯视标墙段，填厚度和高度，三维核对，保存并显示实际生效状态。玻璃门首版按关闭障碍处理 | UI 显示与原生存储一致，重建/重载不丢；无可靠门状态时不自动放行；不把着色当成碰撞支持完成 |
| 4 | 待执行 | 合并 MapView 重复顶部入口；把新地图查看接到现有路线预览，复用同一地图版本 | 不遮挡画布或重复导航控件；查看不激活；当前 validate_plan 仅支持活动地图，非活动副本不能悄悄用原图代测，需明确实现并验证只读离线路径 |
| 5 | 待条件 / 待验收 | 用同一份 903 原图和固定起终点回归；随后完成原生仿真及有监督现场重载、定位、导航、停止 | 保留完整输入/失败/对照；不以采样浏览器或离线几何结果代替真实执行。原图和现场条件不足时继续本地工作，资格保持未完成 |

玻璃处理准则：人工确认的障碍与真实点云分开保存；没有激光/深度返回不等于自由空间。
墙段位置需来自对齐的结构参照、标定图像深度或现场量测，不能凭空补传感点。
视觉模型可辅助提名玻璃区域，不能单独授权通过。门的开关感知与自动通行留作后续，
本轮不引入自动门状态系统。

现有 `mark_zone` 只支持球/等边立方体，修改保存 OctoMap；`clear` 实为写 free。
直接重读修改后的树能保留效果，但从源重建不回放 `voxel_edits.jsonl`，运行 nav
保留私有旧快照，局部 SCAN 与最终制动仍读取传感器实时碰撞图。因此现有接口
不能算玻璃防撞完整支持，必须完成上表 1–3 才能展示为产品功能。

### 保留未完成：SLAM 场景适配与 D2 退化恢复

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | D1 本地检查通过但真实隧道失败前漏提示，整项未勾；D4 原生/Host/UI 本地检查通过，时延与轮速提示待验收。D2 噪声/方向/扩大邻域消融仍有跨场景回归，未改变默认；平面拟合已修正忽略近邻配置的问题，默认轨迹字节一致。L1 多地点竞争与 L2 独立 SE(3) 确认已接入，本地连续周期/近距离别名及正常密集回访通过；真实车库与 NX 长时吞吐待验收 |
| 输入 | 当前 LTLIO 源码、实际有效点面对应、单墙/走廊/房间等可控原生几何输入；不把 IMU 先验混入 LiDAR 可观测性判断 |
| 完成条件 | 保持已完成的 D1/D4 本地结果；D2 的均值、增益与协方差采用一致观测模型，并同时通过横向位置修正恢复负例、无跳变三维运动、强方向纠偏和运输回归；当前诊断版本不声称已抑制退化或达到现场漂移指标 |
| 当前动作 | AIST、BikeTunnel2 和完整 Corridor02 原方法基准已跑；按 RELEAD/X-ICP/VoxelMap 机制继续处理平面不确定性、方向更新与地图反馈。已修全共线点生成伪平面的数值错误，真实 Slippage/隧道轨迹与基线相同；硬性抬高平面跨度候选使精度回归，未采用。保留 6 cm 修正负例和全部失败结果。L2 修复单次错匹配立即入图，L1/L3 及玻璃有限面/射线闭环仍待完成；不改速度门槛冒充修复 |
| 公开数据准备 | [本地数据索引](../datasets/README.md)：AIST Flat-wall 8 段 + GT、TRaIL-Odom BikeTunnel2 的 LiDAR/IMU/位置 GT/静态外参、SuperLoc Corridor02 的原始 ZIP/GT/地图/标定全部已下载。后续转换和离线回放记录已进入各数据集 README 与实施计划；回放失败、GT 参考点和现场资格分别记录，不将下载或条件轨迹指标记作场景验收 |
| 待条件 | 双轮式底盘/编码器接口、走廊车库玻璃真值录制、相机及同步外参；森哥确认目前均未提供 |
| 实施记录 | [SLAM 场景适配实施计划书](design/slam-scene-adaptation.md)；旧语义/仿真失败记录及后续工作保持原状态 |

### 保留未完成：自然语言任务到语义执行的本地收口

2026-10-20 对齐的对话控制目标与用户功能矩阵见 [任务与架构计划](research/plan.md)：
跟随/搜寻的算法改进服务于目标确认、执行、停止和结果反馈；语音、多轮纠正、领域暂停/继续、
原地跟踪及巡检对话接线分别记录缺口，不因解析规则或动作枚举存在而视为已交付。

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 本地契约已验收；普通 HTTP/MCP Agent 受理、查询/取消、执行证据门控、原生 semantic views 桥、无候选搜索和正式 `nav/semantic` 装配均有定向证据。已安装 `.93` 的实机静止 RGB-D/SLAM/预览已恢复；完整原生仿真、当前语义检测的板端部署、正式保存地图 endpoint 和实机运动尚未验收 |
| 本轮目标 | 让“跟随我”“帮我找一瓶水”等普通语言任务获得一个可查询、可定向取消的 Agent run；模型不能只靠文字宣称物理任务完成；目标不在当前视野时，由正式地图提出观察视点，再经原生三维预览后执行搜索 |
| 范围 | 复用现有 Agent、SemanticPlanner、GoalService、共同观测和原生导航；增加普通入口适配、运行查询/取消、执行证据门控、原生语义视点查询桥与搜索状态；不新增任务引擎，不让 Python 投影地图授权运动 |
| 完成条件 | 普通入口返回 `run_id`；旧 run 的取消不能影响新 run；执行任务没有领域终态不能成功；搜索候选必须通过原生三维预览；到达观察点后必须用更新鲜观测重新判断目标；迟到结果、相机重启和地图切换不能推进旧任务 |
| 本轮改动 | Agent 与 LLMModule 使用同一已解析 model、URL、密钥环境变量名和单次请求超时；AgentLoop 增加单次模型总时限；HTTP/MCP 普通入口 Agent-first；Semantic 的通用 `send_instruction` 不再作为独立 `@skill` owner；新增 Agent run 查询/取消和原生语义视点桥接代码 |
| 本地证据 | 总记录见 [本轮验收](../build/semantic-acceptance-20261017/README.md)：语义/观测/Agent 相关 157 项、搜索状态 7 项、Follow 95 项、HTTP ASGI 4 项、架构 16 项、最终 Agent/导航/生命周期/集成 103 项、正式变体与 stack 5 项通过；native adapter 37 项通过，WSL `lingtu_nav_client` 构建成功，真实 CycloneDDS request/result、无关 request_id 隔离和 120 ms 无响应超时通过，既有 semantic views 算法退出码 0。Follow 首轮 95 项曾有 1 项等待正感知超时，隔离和整组复跑均通过，原记录保留 |
| 真实模型边界 | `kimi-k3` 用生产 Agent 工具格式做无运动检查：单步“找水”在 17.4 s 选择 `detect_object`，单步“跟随我”在 18.2 s 选择 `follow_person`；两次继续找水均在首次模型响应前 30 s 超时，handler 未执行。因此只证明工具选择接口可达，不证明真实 provider 全链成功 |
| 2026-10-18 现场与仿真 | [本轮记录](../build/semantic-field-20261018/README.md)：NX 原 Wi-Fi IP 已验证；正式 `map/camera` 静止采集 RGB/深度约 30 FPS，连续预览可用，SLAM `TRACKING`，authority `none`。三组实采 RGB-D 在 PC 用生产函数均检测到一人、输出相机光学坐标，未验证人物身份或机体坐标。NX Host 仍缺模型运行依赖；未部署当前源码或发送运动目标。GTSAM 4.3 Windows SDK 和当前 `ltlio` SLAM 构建已完成，`route` 已修正；no-motion 已通过启动/传感器就绪，但固定目标 `goal_snap_exhausted`、周期定位 stale。另查明采集使用后雷达 site、SLAM 配前雷达外参；无视频时真值缺采样被误报成漂移。停止 ACK/零命令/清权/零残留通过，完整导航仍未验收。语义 task-flow 仅为固定响应/fixture grounding，计划仍 pending |
| 同条件修正复测 | 已显式统一仿真前雷达 mount、mapd offset 并补静止真值，6 项定向测试通过。18:31 同场景复测仍失败：真实机体 XY 变化约 3.2 mm，但 IMU 实际仅约 10 Hz、4370 个调度 tick 被丢弃，SLAM 只接收 197 个初始化样本（要求 200），没有注册点云，未进入规划；停止与进程清理仍通过。缺失 SLAM 测量另报 unavailable 的修正有 3 项定向测试，不再误称为漂移。原失败和新失败分别保留，不能宣称外参修正已经通过定位/规划验收 |
| 下一动作 | 优先修正仿真 IMU 调度/发布实际频率并恢复 SLAM 初始化，再复测固定场景与目标；恢复后继续验证 saved-ray 地图自由空间/支撑和周期定位 freshness。收口目标板模型环境和正式 `nav/semantic` 前，保持相机外参未验证的 preflight 阻断；这些条件完成后再安排监督实机运动，不扩成 TaskPlan 持久引擎 |

### 已有结果：相机到 Follow 的产品验收准备

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 已验收（本地装配、SDK、SHM 组件和合成编码并行负载）；完整 tracking Product、真实模型、浏览器与感知并行、实机运动仍待验收 |
| 本轮目标 | 在现有 tracking Product 中明确声明预览视频，验证同一产品运行中的画面、共同观测、人物身份和 Follow 开始/查询/取消；不把 map/nav 的预览资格当成 tracking 已接通 |
| 已确认输入 | 固定版本视频运行时、现有 SHM 夹具、Follow 生命周期与共同观测实现、tracking Product；真实相机和标定仍待设备 |
| 改动范围 | 现有 Product 声明与装配、Follow 的 Web/SDK/MCP 适配和必要状态呈现；不新增任务引擎、媒体管理进程或另一套 Follow 状态机 |
| 完成条件 | 产品计划明确包含所选视频；视频失败只影响预览，必需感知失败不能放行跟随；同一任务引用能查询/定向取消；旧帧、相机重启、地图/产品切换、旧取消和迟到终态不影响新任务；保留多视角及连续三次不同有效观测核验 |
| 验证范围 | 先补本轮装配和任务交互的定向回归，再用目标相机分辨率的连续输入，同时运行视频和感知/共同观测消费者，通过实际产品调用入口保存本地端到端记录；记录首帧/恢复耗时、帧龄 p50/p95、码率和 CPU/内存。替身、真实尺寸合成输入、连续真实回放分别标记，运动端点替身不计作运动或仿真验收 |
| 本轮改动 | tracking 增加与 map/nav 相同的 standard/camera 变体，sim 保留 standard；同步/异步 SDK 增加开始、双 ID 查询和定向取消；新增 640×480 SHM→共同观测→Follow→Gateway 集成；原生限频从处理结束改为 SHM 读取开始，避免将校验/复制/提交耗时额外叠加 |
| 结果 | 装配相关 26 项、MuJoCo 进程声明 1 项、SDK 81 项/21 subtests、新旧 Follow 观测集成 6 项、原生 GStreamer 2/2 通过；相同 20 秒输入解码帧数 254→373，双时钟确认轮 382（约 12.7→19.1 FPS）；确认轮无/有视频单调时钟处理 p95 为 92.73/96.61 ms。观测到的 wall-clock 秒级跳变及首帧长耗时均保留，不作为实机实时资格 |
| 结果入口 | [本轮验收与过程](../build/tracking-video-check/README.md)；[负载工具](../tools/bench/follow_camera.py)；[SHM 到 Follow API 集成](../tests/integration/semantic/test_follow_camera_api.py)；API 的 Follow 章节已同步 SDK 用法 |
| 下一动作 | 本地完成自然语言 Agent 入口和语义搜索的组合回归；机器人接入后测匹配架构的运行时、真实 RGB-D、相机外参、浏览器/感知并行负载和完整产品启停，再进行监督下的基础导航与跟随 |

### 已有结果：视频环境与产品启停基础

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 已验收（本地）；真实目标板、完整机器人 Product、硬件编码与公网尚未验收 |
| 已完成 | GStreamer 1.28.8 / gst-plugins-rs 0.15.4 固定环境；map/camera、nav/camera 显式 video；RunPlan v11；视频降级、原生新帧状态、断流恢复、标准构建和带架构检查的运行库安装/打包 |
| 实际服务证据 | 安装并迁移后的程序与运行库启动临时 systemd 服务；601 代来源就绪、断流超过 6 秒、同进程恢复 602 代来源，NRestarts=0；停止后端口、临时单元与 SHM 清理 |
| 结果入口 | [验收报告](../build/camera_video_foundation_tests/RESULTS.md)；[实现与研究记录](research/video.md#正式启停接入2026-10-17本地基础已实现)；[目标配置模板](../config/video.env.example) |

### 已有结果：语义代码职责整理

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 已完成（文档与代码核对）；运行时职责调整尚未实施 |
| 本轮目标 | 说清普通指令、Agent、TaskPlan、找物/Follow、技能发现、模型调用与记忆各自负责什么，区分现有实现和计划 |
| 已核实的问题 | `TaskPlan` 与 AgentLoop 仍是并行机制；语义规划器持有多类状态；MCP/Agent 的 `find_object` 含义仍不同；任务经历尚未统一。技能重复 owner、Agent 模型配置和普通语言受理入口已在后续代码中收拢 |
| 本轮产出 | [Decision 代码导航](../src/decision/README.md)；[任务设计第 8 节](research/tasks.md) 的职责、差异与整理顺序；架构索引已连接这两处 |
| 验证范围 | 代码静态核对、修改文档的源路径和差异检查；没有新增运行测试、真实模型、仿真或实机证据 |
| 下一动作 | 当前按 [任务设计第 8 节](research/tasks.md#8-现有代码分工与整理顺序) 完成找物搜索与原生反馈验证；之后再连接最小任务经历。仍不为此新增另一套 Mission 或通用工作流框架 |

### 已有结果：语义找物与共同观测

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 已验收（本地契约/组件集成）：196 项定向测试通过，修改文件定向 lint 通过；本轮未运行真实检测/VLM、仿真运动或实机运动 |
| 修复的缺口 | Follow 已使用共同观测，而找物规划器仍独立接收图像、位姿和场景；本轮已补上相机重启即时失效事件到找物规划器的连接 |
| 本轮改动 | 找物改用现有 `SemanticObservation`，移除四个旧输入；校验 Product、顺序及地图身份；源重启使旧查询和视觉确认失效，取消原生接近任务；临时缺帧不延长确认预算 |
| 验收输入 | 受控 RGB-D、检测框和原生端点替身；实际同步/投影/场景构建、中文目标解析、观察位姿和任务代码；不以此声称模型识别能力 |
| 完成条件 | “帮我找一瓶水”生成水瓶旁的原生可行性查询；旧帧不覆盖新图，相机重启和地图切换不能使旧目标/视觉回复继续生效；Follow、多视角和现有任务回归保持通过 |
| 结果存放 | `artifacts/semantic-observation-20261016/`；用例在 `tests/decision/test_semantic_observation.py` 和 `tests/integration/semantic/test_object_observation.py` |
| 后续功能 | 普通语言现已优先进入现有 Agent，并通过所属语义任务等待领域反馈；目标不在视野时的原生语义观察视点桥和搜索链正在收口。下一步完成全链回归，再做连续 RGB-D 真实检测/视觉核验。Agent 摘要/记忆迁移、板端模型和仿真/实机运动仍需验证 |

### 已有结果：连续运动诊断

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 已验收（本地修复）：分离全段 sweep 与量化坡度诊断，修正机身姿态模式 A* 仅检端点、漏检相邻边的问题；未验证 SCAN 或实机 |
| 冻结输入 | `artifacts/903-planning-20261014/official/` 已转换 OctoMap、三条 `request.json` 与 `path.xyz`；保留原地图、端点和参数，先复测保存路线，再同请求重规划 |
| 已知失败 | Building 横穿第 16 段、Building 楼梯第 7 段、Plaza 第 2 段；`route_valid=true`、`valid=false` |
| 查明原因 | 上述首个失败均为体素边坡度诊断；旧验证器早退，掩盖了 Building 横穿后续 6 段 `occupied_body`、2 段 `unknown_space`，以及楼梯的 3 段 `occupied_body`。端点可通行不代表中间扫掠可通行 |
| 修复及复测 | 连续机身姿态模式在写入 A* 候选前检查边 sweep；顶点/26 邻接、地图、端点与所有参数保持原约定。同请求重新规划后两条 Building 的 328/140 段、Plaza 原路 99 段全部 sweep 通过；原始旧路径仍如实检出 8/3 段碰撞；三个负例继续拒绝。原生回归 4/4、Python 5/5、定向 lint 通过 |
| 保留边界 | 格边坡度诊断仍失败 134/67/2 次，`motion_diagnostic_valid=false`、聚合 `valid=false`，门限未放宽。图路线与连续几何通过不能替代地形坡度/SCAN 轨迹和 raw-to-snapped 接入验证；旧支撑层模式行为未变 |
| 路线图复核 | 已补源 PCD 和等比例展示：Building 跨楼 89.78 m，经 Z=13.9 m 高处连接后返回；上楼 38.37 m；Plaza 20.36 m，Z 仅 0.4–0.6 m。原图坐标自动缩放会夸大起伏，已标记为仅作失败对照；复核与修正版见 `artifacts/route-audit-20261016/README.md`。算法和既有结果未改；基准摘要新增几何验收范围/执行未验收标记，完整复跑 6/6、Python 5/5、lint 通过，仍未验收执行能力 |
| 结果展示 | 根 README 已在本地加入三组、六张等比例路线图和 PCD→OctoMap→A*→路径诊断流程图，文件集中于 [planning](assets/planning/README.md)；补充 [903 实采点云、导航操作台及历史测试记录](assets/demo/README.md)，首屏明确原生 C++ / ROS-free。已移除指定的首页长说明和 Contributing，详细验证边界保留在结果文档；完整预览和改前对照在 `build/readme-planning-review-20261016/`，等待确认后发布 |
| 完成条件 | 拒绝原因精确到段与判据；修正有证据支持的代码缺陷，固定路线同条件复测；墙、未知空间、缺支撑与运动限制拒绝用例保持有效。图路径与可执行运动分别汇报，不通过降低门限或移动端点换通过 |
| 结果存放 | `artifacts/segment-motion-20261016/`：修复前后、实际命令、源码与结果；本地 Linux/WSL，不代表实机 |
| 下一动作 | 将通过全段几何检查的路线接到本地 SCAN 轨迹/地形检查，验证可执行运动与起点吸附接入；903 原始 OctoMap 仍待设备可连接后读取，不替换原图 |

### 已有结果：正式地图到产品页面

| 项目 | 当前约定 |
| --- | --- |
| 当前状态 | 待验收：接入、统一与显示设置已实现；本地入口浏览器回归 8/8、显示交互 7/7、相关前端测试 31/31、Gateway 路由与制品流检查、类型和生产构建通过；实机未测试 |
| 当前子项 | 已验收（本地）：顶部摘要区分“仅可查看 / 导航数据齐全 / 已加载到现场”，展开显示点云、规划地图与服务阻塞原因；切图收起旧提示，窄屏浮层保持在视口内。18/18 状态与导航回归、类型、定向 lint、生产构建通过；桌面/390px 浏览器确认缺规划图原因，查看与切图新增写请求为 0。未验证原生 Linux FD 或实机 |
| 唯一产品页面 | React `MapView` / `PointCloudViewer`；旧 `/map/viewer?map=...` 同源跳转到 `/?tab=map&map=...` |
| 数据所有权 | `mapd` 保存制品与身份 → Gateway 原生 PCD 流 → UI。`view.json` 只保留给离线导出检查；工具的 service 模式进入同一产品页 |
| 本轮行为 | 核对地图名、正整数内容版本和坐标系；切图取消旧请求；同名重存、会话变化与手工刷新重新读取；旧列表晚到不覆盖新列表 |
| 显示规范 | 复用产品字体与控件；可选配色、低/高颜色、点大小、不透明度、背景与网格，浏览器记住设置；最高 Z 只裁切显示，隐藏点不参与拾取；明确原始点数与采样点数。地图库预览与现场活动地图叠加分别记录 |
| 验证 | 前端单元测试、Gateway 路由/制品流测试、类型/构建，以及完整产品页面的切图、内容更新、模拟会话变化、错误身份、列表失败、窄屏和入口汇合 |
| 证据边界 | 本地 Windows Gateway 正式路由与实际 Web 构建，MapClient 为测试替身；真实 903 PCD 和合成小地图明确区分。该验证不是原生 Linux FD 全链路、仿真或实机运行 |
| 结果存放 | 服务接入 `build/map-service-20261015/README.md`；显示设置 `build/map-style-20261015/README.md`；地图状态 `build/map-status-20261016/README.md`（源码快照、截图与日志）。材料仅保留本地；此前截图在审核包 `build/readme-review-20261014/`，未加入发布清单 |
| 下一动作 | 停止扩展界面配置；待机器狗可连接后读取 903 原始 `octomap.ot` 与配套元数据，进入下方四路线回归。之后验证保存重载、定位与现场无运动路线预览；当前原图缺口仍未解除 |
| 完成条件 | 产品查看不需手工准备 `view.json`；入口汇合、身份核对与过期响应失效检查通过；旧 Gateway HTML 不再独立维护；查看地图不产生激活或运动请求 |

### 待条件：903 地图到全局路径

| 项目 | 当前约定 |
| --- | --- |
| 输入 | `903room_v6_corridor`；原始 `saved_rays` OctoMap；`build/go2-map-save-v6-20260930/plans.json` 中的四组固定起终点和配置 |
| 当前状态 | 待条件：原始地图回归缺输入；输入盘点已完成，当前代码的四路线重跑尚未开始 |
| 已有输入 | 本地 v6 目录中的 `map.pcd`、`metadata.json`、`poses.txt`、`plans.json` 已确认存在 |
| 已确认缺口 | 相关本地目录及归档未找到原始 `octomap.ot`；v6 目录也缺 `scan_origin.txt` 与原始扫描包。`occupied.bin` 只有占据信息，派生实验图不能替代完整原始证据。森哥确认机器狗未开机 |
| 当前执行程序 | 2026-10-14 当前工作区已在 Ubuntu/WSL 的 `build/official-global-planner/` 完成 headless、验证器、转换器和编辑器构建；源码快照、工作区差异与日志保存于 `artifacts/903-planning-20261014/` |
| 本轮本地结果 | 原生小回归 3/3、官方固定场景 6/6 符合预期；三条官方正例 `route_valid=true`，但连续诊断均 `segment_motion_failed`。这不是 903 原图或实机运动验收 |
| 下一动作 | 机器狗可连接后只读取回原始 v6 `octomap.ot` 和配套元数据，运行已固定的四路线请求；同步保留连续运动失败的具体段及拒绝理由。原始扫描取回后再追溯清理/射线缺口 |
| 已知基线 | 2026-09-30 的 `.88` 离线记录：短走廊、房间到走廊成功；拐角、下侧长走廊 60 秒超时。历史结果不代表当前代码 |
| 必须保持 | 原图只读；推断空闲空间的实验图不能替代原始射线地图；三维支撑、机身包络、未知空间和连续扫掠规则保留 |
| 本轮交付 | 四路线的当前结果表、路径/拒绝原因、耗时、连续几何验证，以及障碍/缺失支撑用例的拒绝结果；记录源码、二进制、输入和平台 |
| 完成条件 | 当前基线可复现；四条固定正常路线在冻结配置的时间预算内得到路径，并通过连续几何检查；应拒绝的场景继续拒绝。历史成功路线不得退化；仍有超时、碰撞或未知空间失败时，保留未通过状态，不称导航验收完成 |
| 结果存放 | 新运行沿用 `artifacts/` 下的独立记录，保留请求、日志和结果；详细指标留在结果包，本节只更新状态与入口 |

本轮清单：

- [x] 已完成已知 v6 归档目录的输入盘点，原图缺口与派生图边界已记录。
- [x] 已查找相关本地归档并构建当前执行程序；3 项原生回归、6 项官方固定场景结果已留存，连续运动诊断失败独立记录。
- [ ] 原始射线地图、固定请求、执行程序和配置已就绪。
- [ ] 当前代码的四路线基线及必要拒绝用例完成。
- [ ] 根据复现结果修正具体问题，并完成同条件对照。
- [ ] 更新结论、证据入口与下一轮任务；未通过项明确保留。

### 后续队列

| 顺序 | 任务 | 进入与完成依据 |
| --- | --- | --- |
| 当前，2026-10-19 森哥安排 | SC-02/03/04 走廊、车库与玻璃建图适配 | [勾选计划](design/slam-scene-adaptation.md#执行清单)；D1 本地几何检查通过，但真实隧道失败前漏提示；D4 时延待验收，D2 候选未通过。真实漂移、原生重复柱误回环及玻璃射线反例已记录，未计为现场验收 |
| 1，本地准备已完成 | 相机到 Follow 的产品验收准备 | 上方保留装配、SHM、SDK 和并行负载结果；完整产品与实机部分进入第 4/6 项，不重复媒体基础验证 |
| 2，保留未完成 | 收口真实语言任务到语义执行 | 技能 owner、模型配置、Agent-first 受理和运行查询/取消已接代码；继续验证 Agent 子任务引用、原生语义观察搜索和领域反馈。保存实际模型响应，覆盖“跟随我”“帮我找一瓶水”和顺序/关系指令，规则或 mock 不计作模型验收 |
| 3，本地运动验证 | 全局路径接 SCAN 与原生仿真 | 固定地图、端点和参数；验收连续几何、起点接入、地形与 SCAN 可执行轨迹，再做现有原生仿真的到达/取消；不把路线图、回放或运动替身记为仿真成功 |
| 4，机器人接入后先静止 | 板端相机、硬编与观测资格 | 核对实际架构/BSP，准备匹配运行时与明确编码器；确认 RGB-D 配准、时间含义、camera→body 标定和采集时位姿；测真实浏览器解码、帧龄、码率、CPU/GPU/内存和感知并行负载，测量后固定该目标板配置 |
| 5，具备输入和现场条件后 | 保存重载、定位与基础导航 | 使用明确版本的地图经 ProductControl 完成保存/重载与就绪；先静止路径预览，再监督下验收短直线、转弯、房间到走廊、取消、接管和定位失效停止，保留连续视频及原生终态 |
| 6，基础运动通过后 | 真实人物跟随 | 同一段连续录像和固定现场动作对比：目标离开/回来、两人交叉、遮挡、深度失效和取消；保留多视角与三次有效观测核验，统计错接、拒认、重获耗时和三维位置跳变。先静止感知，再监督下低速跟随，不确定身份时等待/停止；不承诺未测场景零误认 |
| 7，前述任务与运动证据就绪后 | 真实语义找水 | 按 VL-Nav 方法对应执行真实模型理解→当前观测/记忆候选→原生观察位姿/导航→到达后视觉复核；目标不在视野时验证搜索与反馈修正，记录未找到、受阻及取消。第一次演示不包含抓取/递送 |
| 8，单机闭环稳定后 | 公网图传、多机器人接入与巡检扩展 | 先测直连和实际网络条件；只在所测网络需要时增加 TURN/中继，记录时延/码率/丢包与连接恢复。随后扩展现有巡检和任务记忆，按真实需要增加能力，不前置建设通用调度平台 |
| 待原图恢复，纳入基础导航验收 | 903 地图到全局路径 | 取回原始 v6 octomap.ot 后恢复上方四路线固定验收；地面证据和残点只按实际失败原因调整；已有构建和失败记录保留 |
| 待森哥审核 | README、UI、仿真与真实输入演示发布 | 本地审核包 `build/readme-review-20261014/`；此前发布已撤回。明确确认内容后才重新上传 |

设备条件到位时，优先进入只读接入与静止验收，不要求等待本地语义扩展全部完成。
各项运动仍以该目标平台的观测、定位和原生运动资格为前置，模型调用成功不能替代这些条件。

各阶段输出统一包含：实际命令/配置、输入来源、结果表、连续视频或日志、失败原因和
下一动作。性能记录使用帧龄和端到端耗时分位数，身份记录使用错接/拒认/重获结果，
三维位置记录保留轨迹与跳变；不以平均 FPS 或通过的组件数量代替整条功能链验收。

已具备的支撑证据包括真实 v6 点云、历史规划请求、只读地面亮带分析和本地查看器字体/交互检查。
查看器改进不计作导航验收进度；地面补全的旧候选实验仍未通过连续路径检查。
各证据的详细来源仍留在现有记录中，避免另起一份重复台账。

2026-10-14 已将保存地图查看器源码整理到
[`tools/visualization/`](../tools/visualization/README.md)，数据与历史记录仍保留原处。
903 的旧页面入口可继续使用；同一真实数据的 5 组交互、4 组字体/布局检查通过，
另有 4 组入口/小样本检查通过，详细记录在 `build/code-tidy-20261014/`。
该整理只涉及查看工具与说明，当前原始 `octomap.ot` 的输入缺口仍未解除。

## Delivery gaps by domain

These are unfinished qualification and engineering items for the first
delivery described in [Product](product.md). Their priorities are retained;
execution order remains the [current delivery cycle](#current-delivery-cycle).
Each claimed robot and platform needs its own evidence. Optional Products and
research do not become prerequisites for the shared map/navigation delivery.

| Priority | Work | Completion gate |
| --- | --- | --- |
| P0 | Shared mapping-to-navigation delivery | Durable save completes with map identity/artifacts; restart/reload uses no previous in-memory map or pose; fresh localization, 3-D goal execution, terminal zero, cancellation, localization loss and failed-startup recovery pass on the named robot/platform |
| P0 | Native MuJoCo qualification for `map` and `nav` | The reference workflow passes lifecycle, scenario, terminal-zero, cleanup, rollback, and repeatability on each claimed platform; preserve regression coverage for other supported Products |
| P0 | Canonical native release | `install/.../{bin,lib,etc,share}` passes package, install, activation, and rollback without production references to `build/` |
| P0 | Stable mapping/no-map `teleop_avoid` | Native sensor-to-command flow survives representative free-space, obstacle, stop, cleanup, and repeatability scenarios |
| P0 | Physical command safety on each claimed target | Fresh provenance, no-motion, fault injection, bounded supervised motion, terminal zero, and driver ACK pass on the named target |
| P1 | Dynamic-obstacle clearing and resource bounds | Labelled replay and long MID-360 runs meet residual, thin-obstacle, reset, CPU, memory, and DDS-volume limits |
| P1 | Motion and collision-aware path smoothing | Ramp, reversal, rotation, emergency zero, and safety ordering pass with an explicit path-smoother failure policy |
| P1 | Typed native map control/query | Persistent field operations no longer require a competing Python map-management path |
| P1 | Online global mapping backend | Resolve rejected-keyframe coverage, verify corrected save/reload, measure paced ARM performance, then complete supervised mapping/navigation; retain the dated [September mapping handoff](operations/localization-opt-handoff-20260917.md) as context |
| P2 | Route, following, and docking Products | Each has an explicit Product, typed lifecycle, safety boundary, simulation evidence, and physical evidence where applicable |
| P2 | Cross-language schema stability | DDS and shared-memory payloads carry explicit version, frame, timestamp, bounds, and compatibility tests |

Build and package the selected Linux native release before target no-motion and
supervised-motion qualification. A clean checkout must reproduce the package
from documented dependencies and inputs; production artifacts must not rely on
`build/`. ProductControl owns the fresh session and lifecycle checks. Directory
cleanup follows the [development rules](development.md#product-focused-simplification).

### Reproducible delivery: next concrete steps

The 2026-10-15 delivery work separates the robot and Web build/package entrypoints,
uses one selected Python throughout installation and rollback, aligns CI and
containers with `uv.lock`, and selects the camera build from the configured
capture driver. These are delivery-tool changes; named-target qualification
remains open. Commands and migration order live in
[Deployment](../scripts/deploy/README.md), not in another lifecycle layer.

- [ ] Prepare one reproducible Linux native dependency environment for each
  claimed platform. The GTSAM 4.3.0 source preparation, SDK packaging and CI
  entrypoints are implemented. WSL Ubuntu 24.04 x86_64 passed the native
  SLAM/optimizer regressions and installed CLI/library-loading checks on
  2026-10-15; see the [dependency guide](../scripts/build/README.md#gtsam-cpu-dependency-package)
  and [SAM qualification](../src/localization/online_mapping/README.md). Actual ARM
  SDK/image builds, Windows SDK builds and the selected robot SDK remain open.
  A locked Python dependency plan does not close them.
- [ ] Build a complete real release, extract it outside the source checkout,
  and verify Python imports, native library loading, CLI startup and API-only
  Gateway readiness without attaching devices or issuing motion. Use the
  extracted artifact for subsequent checks. Package self-tests use fixture
  binaries and cannot close this item.
- [ ] Exercise the independently packaged Web with the extracted Gateway:
  static assets/guide, API, SSE, cloud/camera WebSocket and the chosen camera
  transport. Retain the matched Web/robot versions and check upgrade/revert.
- [ ] Complete fixed-input regression on the original 903 map and recorded
  inputs. Preserve missing-stream/drop counts and time coverage; input replay
  must replace only the selected sensor sources and must address recorded
  timestamps before claiming a new live-time algorithm run. Current MCAP
  publication alone does not prove fault reproduction.
- [ ] Install that same candidate on each claimed robot, then complete cold
  start, save/reload/localization, no-motion preview, supervised navigation,
  cancellation/manual takeover and upgrade recovery. Record repeats and failed
  attempts as well as successes; promote the candidate only at this evidence level.

The original 903 input retrieval remains the active navigation task above.
No deployment, publication or physical motion is implied by these local changes.

### Manual takeover: remaining field acceptance

Native control regressions and the Windows `navd` build are local evidence.
The keyboard and standard USB/Bluetooth gamepad flow is documented in
[Operations](operations.md#manual-takeover-during-navigation); neither establishes
physical device compatibility or supervised robot motion.

- [ ] On each claimed target, verify deliberate input pauses the task only
  after stop confirmation, manual motion does not compete with autonomous
  output, and releasing input holds without losing the target.
- [ ] Verify explicit TaskResume reconnects the retained 3-D route from the
  measured pose after manual movement, including backward/lateral repositioning;
  preserve task identity, goal, speed, tolerances, and route heights.
- [ ] Verify an unused Claim closes without inserting zero into autonomy;
  actual takeover, input expiry/disconnect, cancellation and E-stop retain their
  declared stopping behavior. Source permission closure is not physical rest.
- [ ] Go2 factory handheld integration remains open. Before enabling its
  handoff, verify on the target firmware that disabling factory joystick motion
  leaves usable DDS input, preserves the intended emergency/posture-button
  behavior, and handles radio loss without reusing stale stick intent. It is
  not yet routed through LingTu's operator protocol; local SDK definitions do
  not close this item.

## Batch deployment — pending (2026-10-10)

These items are planned, not implemented or accepted by the deployment review.
Reuse the existing per-robot release installer, rollback and ProductControl.
Keep the initial batch delivery small enough to verify on named targets.

- [ ] P0: Separate model defaults from per-unit identity, sensor bindings,
  network configuration and measured calibration. Resolve one effective
  RobotConfig; preserve per-unit data across software upgrades. The current
  Go2 profile includes one unit's ID, camera serial and raised-mount evidence.
- [ ] P0: Make the management page reachable after installation and reboot,
  with an explicit standby/mode state. Preserve ProductControl as the only
  Product lifecycle owner and require an explicit task to start motion.
- [ ] P0: Assign a stable management address to each robot, using a static
  address or DHCP reservation. Record its robot ID and access endpoint. Keep
  vendor-internal sensor/control networks separate from management addressing;
  isolated internal networks may reuse their existing addresses.
- [ ] P0: Verify two robots concurrently on the intended network. Establish
  DDS/interface isolation under each vendor's driver contract and demonstrate
  that telemetry and commands belong to the selected robot.
- [ ] P1: Add a target inventory and batch result collection around the existing
  installer. Record requested/actual release, success, failure and skipped
  targets; support retrying failed targets without rebuilding on every robot.
- [ ] P1: Record staged rollout and per-unit rollback outcomes. Start with one
  target and expand only after its checks pass; retain manual intervention
  information when the existing rollback cannot finish.

The P0 single-robot workflow is tracked in the current delivery cycle and the
domain table above. Batch deployment depends on that workflow passing new
mapping, durable save/prune, saved-ray OctoMap build, restart/reload,
localization, supervised navigation and terminal stop on Go2/NX. Repeat it on
each other claimed robot/platform rather than copying Go2 evidence.

Existing offline production-pipeline checks and field stationary checks are
documented in [navigation preflight](operations/navigation-preflight-20260927.md),
[Go2 deployment](operations/go2-deploy-20260929.md) and
[the navigation incident](operations/go2-navigation-incident-20260929.md).
They do not close the full field workflow: the recorded incident start still
failed global planning and a separate local collision failure remained under
investigation. The October UI fixture checks add frontend evidence only.

## Map-bound places

The [memory guide](../src/memory/README.md) describes two current stores:
`TaggedLocationStore` for `/locations` and the mapd-backed `PlaceCatalog` for
`/places`. Basic locations still depend on an optional Host Module absent from
standard `map/nav`.

Consolidate map-bound persistence under mapd while retaining Python name/alias
interpretation. Acceptance must cover Web create/list, semantic resolution,
and checked navigation using the same identity, with semantic planning off for
basic location operations. Cover same-name places on two maps, restart/reload,
map changes, and I/O failures. Preserve stable IDs, migrate existing records
before removing their owner, and keep unbound/stale records non-executable.
Consolidate repeated map queries and adapters before adding caches or stores.

## Semantic navigation integration

Semantic planning is selected by `inspection`, not standard `nav`; `tracking`
excludes `SemanticPlannerModule`. Existing local task/recovery/grounding tests
do not establish physical target search or verification.

| Remaining work | Owning reference |
| --- | --- |
| Visibility-aware viewpoints, frontier input, native/field acceptance; model qualification is tracked below | [Navigation review](../research/semantic/navigation_review.md) |
| Detector/ReID assembly, time-aligned target coordinates, shared observations, offline speech | [Edge integration](../research/semantic/edge_follow_patrol_integration.md) |

Inspection route/actions and findings have one qualification list in the next
section. These optional capabilities use the existing camera, lifecycle, and
motion owners; they are not prerequisites for the first shared delivery.

## Inspection reliability qualification

Qualification uses indicator on/off/color and cabinet door state, finite rounds
with an interval, an optional final standby destination and manual charging.
Automatic docking, battery-triggered return and unattended 24-hour service
remain unqualified. Native route/restart semantics are documented in the
[inspection owner](../src/nav/inspection/README.md); findings and review are
documented in the [observation package](../src/perception/inspection/README.md).
The [inspection motion plan](../research/semantic/inspection_motion_plan.md)
provides the route, recovery and observation-viewpoint scenarios.

Remaining delivery work:

- [ ] Run repeated native MuJoCo inspection through the complete supported
  ProductControl/HTTP/recording path, with a fixed three-point route, separate
  per-round evidence, native recovery, observation viewpoints, pause/resume,
  restart continuation and terminal stop. The Windows
  DDS recorder now has local start/stop/MCAP evidence; that component result
  still needs the complete inspection Product run. Do not bypass recording and
  call that complete Product acceptance.
- [ ] Qualify indicator and door findings against labelled photographs of the
  actual equipment, including unreadable/occluded views. Measure incorrect
  normal findings separately from missing observations; preserve review history.
- [ ] Repeat the declared route on each claimed physical robot, including real
  obstacles, changed localization and a return to the standby destination.
  Preserve task progress and confirm actual stopped output before charging.
- [ ] Add battery-triggered return and automatic charging only after a supported
  platform supplies real battery/docking/charge-state interfaces. Arrival at a
  point is not evidence of charging. Scheduled unattended work depends on that
  capability and its field qualification.

## Following and semantic-task qualification

The [Spot comparison](research/spot.md) and [task interface design](research/tasks.md)
provide references, not proof of LingTu capability. Local Follow ownership/API
tests cover stable task/session references, scoped cancellation, state queries,
typed native feedback and visual-waiting versus navigation-failure reporting.
The complete `tracking` Product simulation and field qualification remain open;
motion confirmation stays with native navigation.

| Queued work | Acceptance evidence still needed |
| --- | --- |
| Recorded-image/model acceptance through existing perception/decision interfaces | Local CLIP-consumer and six-image HTTP regressions pass; real models must select the intended person, reject different people and return UNKNOWN when evidence is insufficient. Stub service evidence proves transport, not recognition. |
| Complete the Follow task lifecycle and align Web/SDK/MCP call/results | Correlate language intent, native execution and scoped stop/cancel results through the supported `tracking` Product; qualify simulation and then physical target following. Measure targeted transport changes only after this lifecycle works. |
| Visual-target motion inputs | Verify YOLO/DINO output, D435i calibration, time-aligned target coordinates and rejection of occluding depth surfaces before admitting motion. |

Schedule these through the [current delivery cycle](#current-delivery-cycle).
The [person-follow record](research/person-follow.md) separates CPU/model-export,
Host-contract, recorded-camera and physical-motion evidence; a failed DINO
engine does not block non-semantic map/navigation qualification. Shared
map/restart/goal/cancel work is tracked in the domain table, inspection in its
qualification list, and battery/docking in its hardware-dependent item above.
Multi-robot scheduling and manipulation without supported hardware remain deferred.
No repository-wide transport migration, generic mission engine or new task
persistence service is planned by this qualification work.

## Terrain research threshold

Do not add PCA ground segmentation merely because another project uses it.

Only clean-room implement the necessary portion if representative real MID-360
data shows that the current terrain core repeatedly misclassifies slopes,
stairs, or mixed cells and the failure changes motion safety.

The acceptance set must compare the current terrain owner and the candidate on
the same labelled data, including thin obstacles, reset behavior, CPU, memory,
and latency on each claimed target platform. Simulation alone cannot trigger adoption.

## Release and license gates

FAST_LIO, ikd-Tree, and IKFoM provenance and license obligations require an
explicit release decision. Replacing one container does not establish a
closed-source distribution boundary.

Before commercial distribution, choose one complete path:

- authorization covering the relevant copyright chain;
- a reviewed GPL-compliant distribution boundary; or
- a clean-room replacement of the affected SLAM core.

This is an engineering release gate, not legal advice.

## Sources for closing work

[Architecture](architecture.md) owns module and lifecycle boundaries;
[Testing](testing.md) owns local, simulation and named-target evidence levels;
[Operations](operations.md) owns deployment and field procedures. Close an item
only at its stated evidence gate and update the owning contract. Local or
simulation success does not qualify physical motion or authorize publication.


### 2026-10-30 全局规划边界调整

用户要求撤除全局组合机身/独立 yaw 搜索/转身扫掠准入。当前全局返回 XYZ 候选，
保留已有三维障碍、地面与高度约束；局部实时碰撞仍归 SCAN，终点朝向归 Executor。
此前 XYZ+yaw 接入和分部全局扫掠条目保留为历史实验，不再描述生产默认。
MLS 三维表面连通图、补面与跨层验证仍是后续工作，未切换生产算法。
证据：[2026-10-30 简化与表面调查](../build/xyz-global-20261030/README.md)。
