# 导航错误记录

更新：2026-10-30，北京时间。这里记录已确认错误、发现/留存时间、原因、验证与部署边界。
它是开发和现场复盘账，不是自动追加的运行时日志。源码决定行为；研究假设不能写成现场根因。
完整对比见 [官方 SCAN 与灵途扩展审查](../research/scan-upstream-comparison-20261026.md)。

本页旧称“URDF 分部/部件碰撞模型”统一按**组合碰撞模型**理解；`BODY2` 保留为内部格式名。
[命名、行业来源与能力边界](../design/navigation-body-model-and-motion-posture.md#名称行业来源与实现边界)
于 2026-10-26 记录；改名不代表以下缺陷已修复。

## 原生补面与独立搜索网格（2026-10-31）

- 原生加入 20 cm 分层闭运算，保留原始占据树；XY 搜索改为 10 cm，细图仍为 5 cm。全局保持普通 XYZ A*，不恢复 yaw 搜索。
- 同五请求：首次 4/5，末轮 4/5，重复 3/5。远端可生成 49.01 m 路线，但重复仍超时，不能记为稳定修复。
- 关闭补面的单因素对照也为 4/5；主要提速来自查询索引与搜索节点数量，不是“缺补洞”这一个原因。
- 最长跨度起点支撑通过、圆柱内命中 10 个原始占据体素。未缩小产品几何或删占据来得到成功数字。
- 已通过 Linux x86_64 navd 构建、5 个原生/3 个端点测试目标、66 项 Python 配置测试。未部署 NX，未做本轮 MuJoCo/实机运动。
- 完整输入、尝试、重复和剩余项：[原生补面记录](../../build/surface-native-20261030/README.md)。

## 原图规划的额外拒绝与重复搜索（2026-10-30）

- 后续长距离对照：9–30.5 m 起终点跨度五请求，当前灵途 1/5，DimOS MLS 在完整原图占据点上 5/5，
  但其表面处理/全局简化形状与灵途不同。固定 5 cm、0.60 m 净空、0.25 m 墙距，只关闭表面修补就
  从 5/5 降为 0/5；20 cm 修补半径已恢复 5/5。最长生成 52.56 m 候选，当前切线机身复核仍有分歧。
  原版 OctoPlanner3D 在共同范围副本上 0/5；完整图预处理超过观察期限，不能记为完整图搜索失败。
  [三方原始结果与边界](../../build/long-range-20261030/README.md)。没有因这次对照直接切换生产默认。

- 对照固定 OctoPlanner3D `9a9cc431ea905a5878975cc6fbbce6c9618b31a4`：原版没有整身下沿扩张、
  独立支撑预搜索或中心自由格否决；保留本项目仍需要的三维机身、支撑、步高与扫掠。
- 已修复：引导圆柱误包含足底深度；Go2 正式模型残留小腿/足端（33→17 部件）；
  独立支撑预搜索；支撑高度单侧体素补偿；邻域有效但中心自由射线提前否决；
  容差内引导终点被覆盖回碰撞中的原始目标。精确切向加入已有候选，未新增准入开关。
- 区域空占据缓存保留原碰撞结果。真实地图 3,888 个位姿/扫掠判定逐行一致；
  直接每拍遍历 OctoMap 的慢版本未采用。房间短路线并非找到路线后继续耗时：
  第 6,208 次展开才找到第一条，随即返回。
- 完整 v6 原图、8 个固定请求，修改前 0/8；最后复测 2/8。短走廊约 25 ms，
  房间短路线约 5.15 s；均逐段扫掠通过。拐角有一次约 8.74 s 成功但复跑超时，不能报稳定通过。
  现场近点/房间穿行仍未接出路线，其余三条长路线仍超时；没有修改地图或增加等待时限。
- 8 个原生套件（含 128 个独立穷举图）与 11 个导出测试通过。仅 Linux x86_64 原生验证，
  未部署 NX、未做本轮 MuJoCo 或实机运动。Thunder 本轮仍保留其 38 部件资产，未把 Go2 修改冒充全平台完成。
- [固定输入、全部失败与成功、源码差异和结果](../../build/field-map-20261030/planner-fix/README.md)。

## 保存地图预览过稀疏、毫米级目标高差导致引导回退（2026-10-29）

- 后续审查：相对起始 yaw 的 16 档会漏掉三维网格的真实行进方向，窄 L 通道有连续
  无碰撞路线仍返回 `empty_path`。保留原方向并补入八个网格方向后，同条件返回有效路线。
  同时删除不可达引导后缀的无效扫描（固定用例 guide 边查询 136→11），以及未知空气的
  无效实时空闲证明调用；保留真实占据、转身扫掠和地面支撑。128 组独立穷举对照一致。
  备用搜索仍较重，未部署或做本轮物理验证；详见下方完整记录的“朝向与扫掠的后续审查”。
- 网页原来只取 3 万点，和建图显示密度不一致。本地改为完整 PCD 加载，删除解析/渲染抽样上限，
  默认显示全部高度。36 项前端测试、构建和 ESLint 通过；浏览器实显旧 v6 地图 3,836,924 点。
- 部署源中直接路线要求 Z 差小于 `1e-9 m`，与现场约 1.571 mm 高差不符；本地已有修正。
  本轮又修复引导末端追加真实目标形成纯竖直小段的问题，L 形通道回归先失败后通过，
  3 个相关原生测试目标通过；没有放宽障碍、支撑、坡度或扫掠条件。
- NX 已关机，未部署。现场 7 s 超时已复现，但新图 preclean/removed/OctoMap 对照尚未取得，
  不能宣称该场景已解决或确认误删范围。[完整证据和上游边界](map-preview-planning-20261029.md)。

## 全局起点机身准入取消，局部恢复复核（2026-10-28）

- 2026-10-29 现场：NX 上 `.97` 通过 ProductControl 加载 `903room_v6_corridor`，雷达及 IMU 输入正常；
  用户明确纠正网页机器人位置和朝向与现场不一致。导航任务 IDLE、运动 HELD，原因
  `localization_not_tracking`。地图定位反复将约 6.5–6.75° 的候选修正按连续跟踪 2° 门槛拒绝；
  此证据不证明候选绝对正确。用户暂时无法从网页辨认实际位姿，不得用旧姿态、候选姿态或猜测种子发起运动。
- 离线源码核查发现，简单放开全局重新定位的连续修正门槛还不能保证 nav 清除旧路线：定位新的对齐推进
  source epoch，但导航 TF 跳变检测的角度阈值为 0.25 rad，约 6.6° 未必触发。需要先贯通明确恢复语义与
  坐标身份变化，验证旧路线失效；当前没有修改现场阈值或部署定位候选。

- 2026-10-29：原生 MotionLayer 回归复现 8 cm 体素、10 Hz、0.3667 m/s 移动物体因
  逐帧体素质心跳变而无法达到连续移动确认。使用已有 EMA 速度判断最小运动速度，
  最大原始速度、方向、垂直速度和自由证据条件保持；原生测试与静态、噪声、短遮挡测试通过。
  [红测](../../build/nav-execution-simplify-20261028/prediction-subvoxel-red.log)、
  [回归](../../build/nav-execution-simplify-20261028/prediction-ema-motion-layer.log)。
  该稠密面片 fixture 不含真实遮挡与时序抖动，不能单独证明现场根因。
- 修复后同场景动态箱体：带 DDS 录制的运行避停后续走但未到达，零实体接触，
  存在输入过期；无录制首次报告到达且无箱体接触，但一个轮地接触步和输入过期使验收失败；
  无录制复测未续走到达，另有一个轮地接触步。不能据此认定恢复稳定或地面接触是误报。
  [带录制](../../build/nav-execution-simplify-20261028/crossing-ema-1029/report.json)、
  [无录制首次](../../build/nav-execution-simplify-20261028/crossing-ema-unrecorded-1029/report.json)、
  [无录制复测](../../build/nav-execution-simplify-20261028/crossing-ema-repeat-1029/report.json)。
  同参数静态箱体报告到达、零场景实体接触且验收通过
  （[报告](../../build/nav-execution-simplify-20261028/box-ema-1029/report.json)）。
- 本轮同时保存 [完整地图观测 MCAP](../../build/nav-execution-simplify-20261028/crossing-ema-observations.mcap)：
  `/slam/map_observation` 271 条、`/slam/odometry` 1452 条，录制器报告 dropped=0；
  录到不代表 mapd 消费了全部序列。对这 271 条发布流用生产 RollingOccupancyGrid 离线积分，
  旧关键格在第 61 帧命中、第 64 帧清除，第 65 帧再次命中、第 76 帧清除，之后保持空闲；
  新运行碰撞快照也显示该格为空。这不证明旧运行的 mapd 消费时序或根因，也不能用该格解释
  新运行的另一处碰撞。[回放说明](../../build/nav-execution-simplify-20261028/map-replay-probe/README.md)。
  未部署 Go2。

- 2026-10-29 定位到此前 26.785 s 失败快照的一处扫掠误判：机身实际朝向至计划朝向纯水平转动约 4.25°，
  101 个细分姿态与实测命中边界均不相交；局部扫掠约 4.32 mm 的各向采样余量扩展到 Z，
  吃掉约 1.86 mm 的高度间隙并判碰。构造的纯水平转动和制动回归修改前失败；
  候选将水平余量与垂直位移分开后该回归及 33 项机身测试通过。
- 候选同场景 MuJoCo 未通过：记录 279 个涉及移动箱体的接触物理步、未到达；首次接触在
  6.155 s，箱体碰到前轮，此前多个采样的命令已为零。上次同输入一次复测无场景实体接触，
  单次对比不能证明候选是接触原因，但不足以证明候选安全或解决问题，因此候选已撤回。
  撤回后重新构建 navd 并通过 32 项机身测试；候选不属于生产代码。
  [候选物理报告](../../build/nav-execution-simplify-20261028/crossing-xy-sweep-1029/report.json)、
  [回归失败](../../build/nav-execution-simplify-20261028/sweep-red.log)、
  [候选测试](../../build/nav-execution-simplify-20261028/sweep-green.log)、
  [恢复测试](../../build/nav-execution-simplify-20261028/sweep-restored-test.log)。
- 同日进一步对齐命令与物理时序：候选在 5.8 s 已下发零命令，但位姿差分速度约 0.685 m/s、
  里程计窗口约 0.719 m/s；6.1 s 仍约 0.10 m/s，6.155 s 首次与箱体前轮接触。
  零命令后机身仍向前约 6.6 cm，不能描述为已停稳后箱体单方面撞到静止机器人。
  候选轮次约 6.1 s 才在展示采样出现预测体积，此前快速接近阶段记录数为 0；同期有箱体附近雷达点，
  不能推断雷达完全没有看到箱体。仅凭两次运行不能确定跟踪为何晚确认，也不能将候选改动认定为接触原因。
  [按启动对齐的时序证据](../../build/nav-execution-simplify-20261028/collision-timeline-20261029.json)。
- 原无接触轮次约 26.8 s，恢复请求 0.8 rad/s 被最终制动检查压到约 0.022 rad/s；
  同次 28 s 为 0.015625 rad/s，并伴随受阻重试，因此发生微动而非稳定前进。
  上述扫掠误判是已证实的某次碰撞判定，不足以独立解释全部时间片。
- 旧箱体位置的关键残留体素在箱体离开后，展示子集 153 帧中有 54 帧、共 116 条
  重建射线穿过，26.7 s 仍有穿越样本，而 26.785 s 快照该格仍占据。
  这些是抽样展示点及 qpos 重建的传感器原点，采样时刻与 mapd 处理帧可能不同，不能证明
  mapd 一定收到并按该射线执行 miss；需完整原始输入和单格概率/射线追踪。
  [残留格证据](../../build/nav-execution-simplify-20261028/ghost-cell-ray-evidence-1029.json)。
- 旧箱体轨迹区域确有历史命中，但尚未证明导致本次每一次阻塞的唯一原因或具体清除缺陷；
  仍需完整核对射线覆盖、动态障碍预测与停让位置，不能以删除碰撞或清空地图替代验证。

- 随后用最新 Windows `navd` 重跑原动态箱体 MuJoCo 场景（隔离 DDS domain 85），
  雷达记录 240 个箱体观测采样、60 个近距离遭遇样本，运动箱体位移约 2.165 m；
  5682 个物理观察步记录零场景实体接触。实际轨迹长 1.868 m、净位移 0.490 m，
  未报告到达，验收仍报告未恢复向目标推进
  （[报告](../../build/nav-execution-simplify-20261028/crossing-current-1028-rerun/report.json)）。
  本次是仿真真值定位和 22 部件隔离模型，不能外推到 Go2 实机。
- 一份失败快照（仿真时刻 26.785 s）记录 SCAN 候选起始碰撞
  `collision_at_trajectory_start`，对应机器人位置约 (1.956, -0.248, 0.436)；
  末段交替出现恢复转动与 `scan_actual_motion_blocked`。
  这只能定位失败环节，尚不能证明该次占据来自移动箱体残影或错误射线清除。
  mapd 最终状态记录 processed_observations=226，generation=226；更新持续不等于
  箱体原位置已被自由射线覆盖。复测未录制视频。

- 原全局有向搜索在起点机身查询为占据时立即失败。现在保留实际起点，允许第一段
  同朝向平移连接相邻空闲姿态，交由实时 SCAN 决定能否执行。原本空闲的起点仍检查
  第一段扫掠；后续姿态、转身、支撑和路径碰撞不变，不清空地图障碍。若所有相邻姿态
  仍占据，搜索仍可能无路；没有加入任意长度的“逃离碰撞”豁免。
- 新固定回归在修改前复现立即失败（[日志](../../build/start-admission-20261028/red.log)）；
  修改后全局 heading/body-query 两组通过，包含起点历史占据可连接、相邻姿态全堵仍无路、
  中途碰撞和转身检查（[结果](../../build/start-admission-20261028/global-tests.log)）。
  SCAN 机身 32 项通过（[结果](../../build/start-admission-20261028/scan-body.log)）；
  局部图清除障碍后无需重新点目标的恢复等 4 项通过
  （[结果](../../build/start-admission-20261028/scan-recovery.log)）。
- 当前源码 Windows Release navd 与 SCAN 测试构建通过
  （[构建](../../build/start-admission-20261028/integration-build.log)）。未修改的上游文件仍报告
  C4819、C4702、C4701 警告，不能称为无警告构建。
- 官方固定版本 SCAN 也融合滚动占据与 log-odds，不是每帧完全替换地图。此前动态箱体
  物理失败仍保留旧位置命中边界；不能把“扫描在更新”当作“原障碍已被自由射线清除”。
  本轮组件恢复测试不覆盖射线清除和物理运动，未重跑动态 MuJoCo，未部署 Go2。

## 时间与状态规则

- 分开记录代码已存在、故障观察、修复验证、安装和实际运动验收的时间。
- 历史测试 XML 没有时区时保留原值，不用文件修改时间推断首次编码时间。
- 本地通过不写成已部署；进程就绪不写成已行走；规划到达不写成物理停稳。
- 后续错误沿用本页追加事实、源码和证据，不另建一个重复日志服务。

## 2026-10-28：输入、脱困和终点的重复等待

**状态：本地实现已精简，定向回归通过；尚未部署 Go2。物理对照仍单独记录。**

- 输入门原先在数据恢复后还等待多个来源的 generation 同步推进。箱体旧版本物理
  记录的 8.8 s，碰撞图年龄已回到约 0.053 s，其他必要输入有效，却仍是
  `recovering / fresh_frames=0 / required_frames=1`。现在直接判断当前必要输入，
  不再维持独立恢复计数。当前明确 LOST/catastrophic 与真正缺失/过期仍保持运动。
- Recovery 完成旋转或未找到候选后，Executor 还要求地图 stamp/epoch/sequence
  变化才允许继续。红例在相同有效几何下停在 `recovery_observation_wait`；删除该
  屏障后可继续搜索，真实占据、剩余平移及转身扫掠仍由原查询验证。34 项直接回归通过。
- 调度持续超时曾清执行路线、建立人工接管锁。现在仅挂起当前输出并保留路线，
  调度恢复后解除自身保持，不修改真正的接管、暂停或急停状态。输入/调度/配置/
  投影器四个原生测试目标与 11 项调度接线合同通过。
- 位置目标的 `finalYaw=nullopt` 被 `activatePoseStage()` 替换成路径末段 yaw。
  红例在目标位置偏差仅 2 cm 时仍输出 -0.6 rad/s 并拒绝到达。现在最后阶段保留
  请求的可选 yaw；显式终点朝向仍需满足，路径朝向继续参与移动和碰撞检查。
  63 项 PoseRoute 回归通过，包括位置目标和显式朝向的对照、转身碰撞和楼梯出口恢复。
- 首次全局搜索普通无路原先立即 Failed。现在复用 pending 保存同一目标和身份，
  0.5 s 退避后使用当前位置重试；保持 Planning，直到实际激活路径。等待中的驱动
  瞬断不再取消该目标。真实超时、取消、换图、异常、外部目标与巡检重试的三个
  原生整合测试目标通过；不将纯预览加入自动运动重试。
- `/go x y` 预览与提交现在显式使用同一个有效机身高度；Web 保留具体规划失败
  原因，避免将支撑缺失、数据等待和服务退出都翻译为“条件未满足”。
- 证据目录：[本轮回归与物理对照](../../build/nav-execution-simplify-20261028/)。

## 2026-10-28：仿真运行异常清空已采集证据

**状态：测试驱动报告已修，41 项发布生命周期及 18 项相关运行回归通过。**

- `box-before` 实际已加载 v39、接收全局路线并移动约 1.757 m，随后发生
  `queue_oldest_age:1.375s>0.5s`。这是发送线程实际墙钟年龄，不是模拟时间换算。
- 原 runner 将运行异常抛到 main 后，用空计数器和空运动指标生成报告，造成
  “模型未加载”“没有运动”等派生缺证。现在保留原始异常并执行清理，报告使用
  已累计的数据；未采到的指标仍缺失，失败仍是失败。没有放宽发布或碰撞阈值。
- 原始记录在本轮证据目录 `box-before/motion/`；它不是完成到达的有效基线，
  不得把中途退出的时长与完整任务直接比较。

## 2026-10-28：并行修改期间整包混入不一致结构布局

**状态：已修复混编产物；冻结源码后完整重编 navd，并通过静态箱体物理试验。**

- 组件测试通过后，首次新包 `after-navd.exe` 在两个独立 DDS 域均发生
  `0xc0000005`，故障偏移相同为 `0x9ac2`；Windows Application Error 事件和
  crash dump 独立确认。驱动等待 ready 超时是导航先崩溃的后果。
- dump 与链接 map 表明：`GoalPlanController::snapshot()` 按旧字段偏移写入，
  `GoalPlanSnapshot` 析构已按新增 planning_origin 后的偏移读取 string，导致
  错误释放。编译与源码编辑有交叠，不能将通过的组件程序等同于整包已就绪。
- 保留失败 exe、dump、LTCG 缓存与 map；冻结源码后重编。单文件重编时链接器
  自动回落完整 LTCG，因此不能声称该试验单独排除了增量链接缓存的影响。
- 不为构建产物混编新增运动检查。后续物理验收必须使用通过整包启动的实际 exe，
  失败两轮不能用于轨迹效果对比。

## 2026-10-28：动态箱体测试发现配准点云遗漏真实雷达原点

**状态：仿真输入接线已修；静态回归通过，动态停让后的续行仍未通过。**

- 原瞬时点云在 LiDAR site 位置生成，但 `_write_native_registered_cloud()` 没有
  转交现有 sensor_origin_world，输出旧记录 type 4；原生 fixture 因而使用机身
  原点。当前 Thunder 雷达安装偏移约 X=0.403 m、Z=0.058 m，地图清除射线与
  实际测量射线不一致。NAV 展示层自己的 sampling_origin 正确不能证明 mapd 输入正确。
- 在同一采样状态捕获 site 原点，沿既有 type 7 传递。真实调用 run() 的回归先失败
  后通过；85 项 publisher/record 测试、6 项相应运行测试和当前原生发布器的 type 7
  隔离输入验证通过。没有增加协议、生产开关、衰减阈值或修改 SCAN 算法。
- 两版在修正后的输入下均通过静态绕箱、到达、零实体接触和停车确认。证据为
  `box-before-origin` 与 `box-after-origin`，不能与旧输入混作精确前后比较。
- 修正输入后的 `crossing-origin`：真实雷达看见运动箱体，记录到停让；没有实体接触，
  但任务窗口内未恢复推进至目标。旧输入下的 `crossing-after` 则出现右后轮接触侧墙，
  该失败保留，不被新回归覆盖。
- 早期失败快照有旧箱体命中残留。当前 mapd 占据衰减配置为 0；250 ms 是调用周期，
  不是占据失效时间。是否有新的实际穿越射线尚需逐格核对，不能直接断言地图清除
  算法错误，更不能用全局 TTL 删除静态实测障碍来通过测试。
- 本轮动态场景尚未验收；不恢复逐脚接地证明，不因动态试验失败删掉真实接触判定。

## 2026-10-28：中间路段精确到点导致反复纠偏

**状态：本地执行衔接已修，MuJoCo 已越过两个原卡点；完整楼梯任务未通过。**

- 1 cm XY 与实测机身 2 cm Z 门槛属于当前未提交的 pose-stage 适配，Git HEAD
  没有这些常量和阶段函数，无法给出准确引入提交或编码日期。10-27 的本地审查已有
  1 cm 问题记录，但该记录不是代码提交时间。
- 干净上游 SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` 使用配置的
  `finish_dist` 判断样条末端，不要求每个全局中间转角达到上述固定 XY/Z 精度。
- 实际残差约 2 cm 时，旧执行器持续小幅纠偏；第二轮又记录到 XY 1.55 cm、
  Z 2.091 cm，固定高度预筛先于三维扫掠将其拒绝。
- 修正：复用现有支撑高度容差，从实测位姿验证转身与接回路线的机身扫掠；
  进入已验证的中间停点后先停稳再推进。它只证明障碍扫掠通过，不能替代地面支撑判断。
- 60 项 PoseRoute 回归通过。同场景同目标物理执行通过两个中间点、累计约 7.13 m、
  未触发脱困，随后在楼板边缘前倾停止，不得写成上楼成功。
- 证据：[前后对照与失败范围](../../build/stair-multifloor-20261028/README.md)、
  [修改后轨迹](../../build/stair-multifloor-20261028/spin-final-trace.jsonl)。

## 2026-10-28：地面边界外推与噪声侧边回波被当成支撑

**状态：单侧邻域外推与实际 XY 相位已修；带噪真实保存地图尚未通过边缘通行验收。**

- 任意一个邻格就可证明支撑的循环，仓库最早可追溯到 `c66068722`，提交记录日期
  2026-07-01。该提交是目录整理，不能断言它就是算法首次添加日期。7-17、9-16
  增加高度带、首表面等判断后仍保留此邻域语义。这不是 SCAN 局部优化器添加的规则。
- 同目标旧预览包含 9 个低层越界点，约 1.037 m 中心路线没有物理楼板。
  实际前倾位置与重生成路线相近；不能主要归因于局部跟踪切角。
- 修正：缺测中心只能在邻域支撑包围实际查询 XY 时补洞，不能从单侧、共线邻点
  或凹角外推；直接已观测列保持原语义。固定 L 形路由、稀疏孔和 20 cm 台阶回归通过。
- 实图复查没有全部通过。两个疑点各来自一条原始保存 PCD 命中；同方向 MuJoCo
  射线真实命中楼板侧边，径向测距误差分别约 -7.169 cm、-5.915 cm。
  它们属于已启用的 2～3 cm 标准差噪声模型，不是重复外参，也不能删除噪声来宣布修复。
- 现有支撑面参数 `radius=1/min_samples=3` 的隔离对照仍返回相同越界路线；
  未作为新生产默认值。后续应修正孤立/侧边占据升级为可站表面的证据语义，
  同时补齐局部连接的地面边界约束，不重新加入逐脚准入或未知空气包络证明。
- 证据：[同请求旧预览](../../build/stair-multifloor-20261028/spin-final-repeat-preview.json)、
  [修正后的实图预览](../../build/stair-multifloor-20261028/support-fixed-preview.json)、
  [原始叶节点](../../build/stair-multifloor-20261028/support-gap-probe/raw-leaves-offset.csv)。

## 2026-10-28：驱动短暂超时保留任务却清空执行路线

**状态：本地接线已修，相关停车与任务协调回归通过；未部署 Go2。**

- 清理调用所在行最早可追溯到 `f670464d8`，提交记录日期 2026-08-02；
  它与后来“保留活动任务”规则首次发生冲突的日期尚未确定，不能仅用 blame 推断。
- 协调器在掉权时保留任务，主循环却继续调用 `clearEndpointMotion()`，清掉
  Executor 路线，驱动恢复后持续 `no_global_path`。
- 改用既有 `holdEndpointMotion()`，立即输出零并挂起，保留路线；驱动未就绪时
  输入门继续阻止运动。没有提高命令超时上限。主循环接线、停车动作和任务协调测试通过。

## 2026-10-28：Thunder v39 精转响应仍未验收

**状态：隔离物理测试确认局限，未推广速度补偿候选。**

- v39 model61128、相同 MuJoCo 场景、5° 目标容差重跑：首次进入范围约在
  开始转向后 25.40 s，45 s 转向窗口内未达到稳定判定；随后零指令 10 s 的最终
  残差约 6.239°，记录 438 次零/非零切换，没有非地面接触。
- 该测试不等同于产品默认朝向容差，也不能把它说成所有导航均失败；它明确说明
  当前模型与小角速度控制组合尚未通过 5° 精转，不应单凭规划单元测试宣布转向完成。
- 证据：[同条件重跑](../../build/stair-multifloor-20261028/yaw-origin-rerun.json)。

## 2026-10-28：实测机身几何的能力表述校准

- 当前 `InputProjector::updateBodyGeometry()` 克隆名义组合模型，再将定位姿态的
  roll/pitch 烘入各部件；它不读取电机关节角，也没有实时正向运动学。
- 因此 `body_geometry` 应理解为名义关节几何与实测机身姿态，不能描述成实时关节重建。
  支持的 Go2/Thunder SCAN 路径在有效定位后都会构造它，不能将缺关节快照推断成
  `body_geometry` 必为空，也不为这个错误推断新增生产回退。
- 本轮最新平地目标实测出现其他目标与人工接管输入，固定目标被覆盖，故结果标记无效，
  不能宣称自动导航通过。轨迹与裁决保存在
  [混合输入记录](../../build/stair-multifloor-20261028/support-fixed-flat-trace.jsonl)、
  [无效裁决](../../build/stair-multifloor-20261028/support-fixed-flat-verdict.json)。

## 2026-10-22 已存在，10-25 现场复现：未知空气导致整机拒绝

**状态：当前本地已修；局部修复已安装到 `.97`，随后断网前没有完成实走验收。**

- 最早可确认留存：`2026-10-22T22:38:41.342`，来自
  [测试 XML](../../build/body-execution-20261022/test_scan_body.xml)。该字符串未注明时区，
  不是精确编码时刻。10-22 的 [集成记录](../../build/body-execution-20261022/README.md)
  已将 unknown 拒绝写成显式 Body 的合同。
- [冻结代码](../../build/body-execution-20261022/source/src/nav/cpp/planning/local/scan/grid.cpp)
  在机体相交格上使用 `return occupied ? 1 : -1`；
  [冻结测试](../../build/body-execution-20261022/source/tests/nav/cpp/test_scan_body.cpp)
  还要求无 known-free 的空气返回 `-1`。这是错误合同被测试固化，不是单纯缺少测试。
- 10-25 现场：原生收到 112 次遥控输入但输出均为停车；快照为
  `collision_at_trajectory_start`、state=-1。实时 OBB 内没有实测占据，存在自遮挡等未观测格。
- 根因：显式机体/URDF 执行适配把未知空间政策与形状替换一起改变。用户选择 OBB
  不等于要求整机体逐体素 known-free；官方 SCAN 没有这个要求。
- 修正：当前 [Grid](../../src/nav/cpp/planning/local/scan/grid.cpp) 仅对 measured occupied
  做机体相交，三态位图不改写；全局 BodyQuery 当前也仅对 occupied 做实体碰撞。
  支撑、范围和数据有效性单独处理。
- 证据：[10-25 现场和安装记录](go2-release-20261024.md)、
  [原始快照与 208 项定向回归记录](../../build/nav-field-20261025/README.md)。
  同一快照的起点、前 10 cm、连续段、制动查询从 -1 变为 0，SCAN 第 3 tick 得到 Ready。
- 引入提交/作者：现有 Git 历史没有相应引入提交，相关代码仍是未提交工作；不能据此
  编造精确作者或时刻。此前“换 OBB 时引入”的概括不够准确，应改为上述可核实阶段。

## 2026-10-25—26：扫掠端点额外半步余量造成假阻塞

**状态：本地修复并通过组件对照；未重新发布到 NX。**

- 现象：配置 OBB 与墙仍有 0.5 mm 间隙，沿离墙方向移动，却在起点被额外采样余量挡住。
  同一局部用例的制动允许比例只有 `0.046875`。
- 根因：全局边段、局部段、动态相对段和制动弧对起终点也增加半段运动上界 padding，
  把采样覆盖余量延伸到实际出发区间之外。
- 修正：真实端点按配置 margin 检查，分段中点加半段运动上界覆盖连续运动；制动取真实弧中点。
  没有缩小机体、删除障碍或降低速度。
- 对照：离障和保留间隙的到达通过，制动比例恢复 1.0；真实端点、中段障碍和转身碰撞仍拒绝。
- 证据：[对照与范围](../../build/nav-field-20261025/replan-departure.md)、
  [局部修改前](../../build/nav-field-20261025/departure-red.log)、
  [局部 24 项通过](../../build/nav-field-20261025/departure-green.log)、
  [全局修改前](../../build/nav-field-20261025/body-query-edge-red.log)、
  [全局修改后](../../build/nav-field-20261025/body-query-edge-green.log)。

## 2026-10-25—26：等效重规划无条件换曲线、失败后反复优化

**状态：本地修复并通过组件对照；未重新发布到 NX。**

- 现象：同一目标、候选成功但没有足够收益，仍提出 `executing=1 → proposed=2`。
  候选失败而旧尾段有效时，FSM 持续 REPLAN；上层原本可以保留 active，不能概括为每次都停车。
- 根因：有失败/待交接期间保留，但缺少成功候选与有效旧尾段之间的选择。
- 修正：在现有 FSM 交接处重查当前连接和完整旧尾段；同覆盖、同末端速度下，新候选
  剩余时间改善不超过约 10% 时保留旧曲线。保留原 controls、导数、ID 和时钟。
  新参考、碰撞强制重规划、冻结和既有降速重置不被偏好覆盖。
- 对照：等效候选变成 `offered=0, executing=1, proposed=1`；失败候选可继续有效旧尾段。
  新参考、新障碍、迟收与暂停撤交接仍有覆盖。
- 证据：[有效红期](../../build/nav-field-20261025/retention-effective-red.log)、
  [SCAN 116 项](../../build/nav-field-20261025/test_scan_planner.retention-green.log)、
  [异步任务 23 项](../../build/nav-field-20261025/test_local_plan_task.retention-green.log)。
  两个原异步测试现显式提交新参考，保证真的生成 offer，未放松迟收/暂停断言。

## 2026-10-26：旋转转弯包络的 AABB 空角被要求有地面

**状态：2026-10-26 本地已修，27 项机体回归通过；尚未部署或实走验收。**

这是当时将 AABB 全列改为实际投影筛选的历史修复。后续已确认该整身前置本身不合理，
现已整段删除；最新状态见本页“删除局部整身逐列地面准入”，本节不再定义当前准入规则。

- 范围：带 `supportHeightM` 的连续转弯 blend 预检和持续局部规划；普通无此元数据的
  键盘辅助遥控不走该支撑分支。不是 10-25 未知空气故障的同一个原因。
- 根因：[Grid 支撑分支](../../src/nav/cpp/planning/local/scan/grid.cpp) 枚举世界 AABB
  全部 XY 列，没有区分列是否与候选旋转包络实际投影相交。
- 隔离反例：0.6×0.6 m 方包络、yaw=45°，实际投影相交的 60 列均有实测地面及其上方 free，
  旧实现仍拒绝。初版 probe 的 12 列只统计中心位于外框内的额外列；正式回归完整枚举
  相交体素，外框共 100 列，其中 40 列不与真实投影相交。此前将 12 写成总额外列数不准确。
- 修正：逐部件准备既有三维碰撞几何，以完整高度的列筛选其真实投影，保留实测地面、
  free-above、高度与全路径检查；分离部件之间的空隙也不再补成支撑区域。
  投影筛选排除只有零面积接触的列，实测障碍的接触判定与机器人 margin 不变。
- 证据：[隔离代码](../../build/nav-audit-20261026/support-footprint-probe.cpp)、
  [预期通过但实际失败的日志，EXIT_CODE=1](../../build/nav-audit-20261026/support-footprint-probe.log)。
  [正式修改前](../../build/nav-audit-20261026/support-footprint-red-final.log)、
  [27 项修改后](../../build/nav-audit-20261026/support-footprint-green.log)。
  正例、真实内部缺口、余量扩大、分离部件、零面积角点及原有支撑/障碍回归均有覆盖；
  没有实机或完整规划耗时结论。

## 2026-10-26：非对齐地图原点下点查询与 DDA 查错体素

**状态：本地已修，118 项 SCAN 与 56 项 core 回归通过；尚未部署。**

- 范围：公开 in-process LocalCollisionMapView 接受有限、跨度有效的非对齐原点。
  当前 mapd 的实际导出原点按分辨率对齐，不能把此项直接认定为现场地图问题。
- 根因：view 和 body 查询按相对原点解释 bitmap，无 Body 的 Grid point/DDA 却先取
  全局 floor，再减 round(origin/resolution)，合法原点的小数部分被丢掉。
  例如 origin=0.03、resolution=0.1、point=0.105，会把 cell0 读成 cell1。
- 修改前：[两项正式回归均失败](../../build/nav-audit-20261026/origin-red.log)，
  包括正/负原点、XYZ、双向线段、空闲邻道，以及 planning→grid 变换。
  测试手工构造 bitmap，不沿用原测试 fixture 中同样的全局索引错误。
- 中间修法也保留：[先减原点再除分辨率的完整回归](../../build/nav-audit-20261026/origin-first-relative-formula.log)
  暴露 7 项失败。恰好在体素面上的坐标受到浮点舍入影响，例如
  `(0.4-(-1))/0.1` 在 binary64 中略低于 14；角点的同时 crossing 也受影响。
- 第二个候选 `floor(point/resolution-origin/resolution)` 虽通过已有整套回归，仍被独立
  发布编号反例否定：resolution=0.05、origin=-10、point=-12*resolution 时，mapd 发布
  cell187 occupied，该候选却读 cell188 free。
  [真实 Grid 修改前探针](../../build/nav-audit-20261026/origin-aligned-face-red.log)
  返回 view_occupied=0、grid_state=0、EXIT_CODE=1；不能通过同步改 fixture 掩盖发布层差异。
- 最终计算保留整数基准与小数偏移：base=round(origin/resolution)、phase=origin/resolution-base，
  q=point/resolution-phase、cell=floor(q)-base；只有几个 ULP 内的对齐误差归零 phase。
  对齐窗口保留 mapd 原运算顺序，非对齐窗口保留完整小数偏移，DDA 在 q 域计算 crossing。
  view、Grid point/DDA 和测试 bitmap 共用小型 GridAxis 几何工具，旧测试期望不放宽。
- 修改后：[非对齐原点两项通过](../../build/nav-audit-20261026/origin-green.log)、
  [体素面/core 56 项通过](../../build/nav-audit-20261026/test_local_planner_core.integration.log)、
  [保留角点与完整 SCAN 118 项通过](../../build/nav-audit-20261026/test_scan_planner.integration.log)。

## 2026-10-26：部件位姿几何重复计算

**状态：本地已优化并接入全局与局部；不是已证实的现场卡顿根因。**

- 根因：同一部件、同一位姿和 margin 下，每个 occupied cell 的 SAT 查询都重新变换
  中心、计算 sin/cos 和世界轴。支撑投影筛选也可复用同一几何。
- 修正：PreparedPart 是一次位姿内的普通栈值；每个 part/pose 重新准备，无跨位姿缓存、
  无模型管理器。复用原 SAT/sphere 和接触内核，尺寸、margin、unknown 与占据策略不变。
- 对照：四种形状、多旋转/yaw/margin 下 136,080 个 cell 的旧/新结果一致，另有接触边界
  和 support exemption 对照；现有全局障碍、支撑和扫掠回归通过。
- 本机 WSL x86_64 Release 内核对比：每批 1,114,112 查询，7 批交替顺序，包含 prepare
  成本；中位耗时 90.6143 ms→19.5815 ms，命中数均 3,504。只说明此固定几何内核的
  计算减少，不能外推整体规划、NX 耗时或物理响应提速。
  [基准源码](../../build/nav-field-20261025/prepared-part-bench.cpp)、
  [基准结果](../../build/nav-field-20261025/prepared-part-bench.log)、
  [全局回归](../../build/nav-field-20261025/prepared-global-ctest.log)、
  [支撑集成 28 项](../../build/nav-field-20261025/prepared-global-support-run.log)。

## 2026-10-26：模型能力和部署口径尚未闭合

**状态：接线缺口/交付风险，不冒充已发生的碰撞事故。**

以下为该次审查时状态；同日后续已完成平台组合模型选择及当前 roll/pitch 的
observedBody 接线，见本页末两项。在线关节 FK、安装实测和部署仍未闭合。

- 审查时本地 Go2 选单 OBB、每侧 5 cm；最后安装 `.97` 的记录仍是此前 10 cm 余量。
- BODY2 导入/加载存在；固定站姿资产缺 D435i/支架的完整安装覆盖，不能宣称完整实机几何。
- Go2 驱动已发布关节 q/dq/力矩，Thunder DOSO adapter 尚未实现同类关节输出；navd
  尚未订阅后做 FK，正式最终刹车 Grid 也未传 observedBody。
  只设置模型路径不能获得随步态变化的碰撞几何。
- 原始 manual 模式跳过 SCAN 几何分支；自主导航和辅助遥控有检查，不能说所有接管模式均有避障。
- 细节与推进边界：[机体模型与运动姿态](../design/navigation-body-model-and-motion-posture.md)、
  [本次对比](../research/scan-upstream-comparison-20261026.md)。

## 已有运行时证据在哪里

- systemd journal：服务 stderr/运行日志，现场日期以 journal 的墙钟时间为准。
- nav 状态 JSON：状态、原因、计数和最新失败摘要。
- `<status_file>.scan-failure.json`：按失败 episode 记录不可变输入、位图、Body、候选和原因；
  后台写入、同一路径替换，**不是完整错误历史**。其 clock 明确区分 steady/external，
  不能把 steady 秒数直接当公历发生时间。
- 上述失败证据应与运行版本、地图和现场记录一起保存；本页是其人可读复盘索引。

## 2026-10-26 本轮集成验证

本机 WSL x86_64、Release，受影响目标重新编译后：

| 目标 | 结果 |
|---|---|
| SCAN backend/Grid/FSM | 118/118 |
| Body 障碍、支撑与扫掠 | 27/27 |
| LocalPlanTask 异步交接 | 23/23 |
| LocalPlanner core/map view/recovery | 56/56 |
| 全局/局部支撑集成 | 28/28 |
| 几何原语与全局 BodyQuery smoke | 独立 CMake/CTest 2/2 |
| Body contact、遥控安全 | 均通过 |

[局部构建](../../build/nav-audit-20261026/local-green-build.log)、
[异步交接](../../build/nav-audit-20261026/test_local_plan_task.integration.log)、
[遥控安全](../../build/nav-audit-20261026/teleop.integration.log)、
[全局 CTest](../../build/nav-field-20261025/prepared-global-ctest.log)。
上述结果属于此前的定向编译与原生组件测试阶段；该阶段没有运行 NX ARM64 构建、MuJoCo
或实机行走，没有改产品配置、服务或地图，没有重新部署。后续本轮实际 MuJoCo 结果见下节，
不能用此前通过项外推全部导航场景、正式实时 URDF FK 或物理制动验收。

## 2026-10-26：新 MuJoCo 组件运行与横穿障碍未修复缺口

**状态：已实际运行新物理；22 m 和关节部件短程到达，横穿障碍当前版与同参旧版均发生真实接触。
动态缺口未修复，不宣称由本轮引入或已解决；尚无实机或 NX 新部署结论。**

- 运行范围：当前源码在独立 Windows MSVC Release 目录重建，Thunder V4 ONNX 策略推进
  实际 MuJoCo 动力学，native 接收实测状态；导航使用人工完整三维地图及固定 actor 真值。
  不是轨迹积分、旧状态播放、完整 Product 或 LiDAR 感知验收。CCD、接触分类、原场景和速度
  保持本次解析设置，没有为通过修改物理或生产配置。
- 当前三项：22 m supplied pose route 在 50.00 s 执行时间到达，实测 XY 里程 21.2646 m、
  终点误差 17.211 mm，0 / 10,000 个检查子步不允许接触；38 个实测关节部件的 1.2 m＋90°
  在 9.78 s 到达，误差 0.609 mm，0 / 1,956 个子步不允许接触。后者只证明基准快照入口，
  不代表正式 navd 已接通驱动关节 FK。
- 停车证据：22 m 与部件短程最后 0.8 s 命令均为零，但还核查了实际速度；末次 XY
  分别为 0.001478 与 0.000257 m/s，该窗口最大为 0.001697 与 0.001064 m/s。
  记录为接近静止，不写成数学零速度。22 m 第二角约 `(8,6)` 仍有 `aligning_goal_yaw`
  累计 3.40 s 平移零命令停顿，不能称为全程连续无停顿。
- 当前横穿失败：约 5.92 s 开始近场零命令，6.10 s 最终几何阻挡，6.895 s 后右轮
  `RR_wheel` 与 `fixture_actor_geom` 真实侧撞，2 / 1,380 个子步发生不允许接触；
  首次记录约 1.997 mm 穿入、146.9–151.2 N 法向力。碰撞前机器人 XY 速度约
  0.000727 m/s，actor 仍在移动，不能因输出已为零就判定动态场景通过。
- 同参旧版：留存的 `tangent-v4.exe` 用当前 harness、相同解析控制/OBB/配置、地图、
  世界 XML 和 actor 输入复跑，也在 6.50 s 侧撞，1 / 1,300 个子步不允许接触。
  此配对只确认两版在当前配置下均有缺口，不能给出引入版本、成功率或性能变化结论。
- 指标口径：22 m 全程到有限全局折线距离 RMSE 为 98.918 mm，活跃局部样条跟踪 RMSE
  为 6.588 mm；转弯切角、绕障偏离全局折线与样条跟踪误差分开记录。历史 47 s 控制参数
  与本轮不同，没有旧版 22 m 同参提升结论。
- 首次 parts 还保留了启动失败：parser 默认 CMU，因 path library 缺失在执行前退出；
  新目录复跑仅给子进程设置 `LINGTU_NAV_LOCAL_PLANNER_BACKEND=scan` 后到达，
  没有伪造路径库或修改生产默认。
- 证据：[完整运维记录与视频](mujoco-navigation-20261026.md)、
  [本轮 HTML 汇总](../../build/nav-mujoco-20261026/component/index.html)、
  [物理与速度图](../../build/nav-mujoco-20261026/component/report/component-physics-overview.png)、
  [两版匹配输入](../../build/nav-mujoco-20261026/component/crossing-matched-inputs.json)、
  [停车/接触时间线](../../build/nav-mujoco-20261026/component/crossing-timeline.json)、
  [当前首次坏子步](../../build/nav-mujoco-20261026/component/current-crossing/first-bad-substep.json)。
- 原生 DDS 独立运行未通过到达：首轮 fixture 已知自由高度不足；修正分辨率后中心支撑
  通过，但目标 z=0.45 与 below=0.25 使 OBB 底部接触 Z=0.2 的占据地面表面，实际
  `occupied_body` 查询确认。只有约 0.005 m 站立漂移，无非零导航命令；两轮退出零命令、
  STOPPED ACK 和 owned-process cleanup 通过。没有再缩模型或改阈值求通过。
  [完整独立运行记录](mujoco-navigation-20261026.md#原生-dds-与-mujoco-独立运行)。

## 2026-10-26：MuJoCo 验收工具丢失 OBB 编译参数

**状态：工具已修，最窄实际编译环境回归 3/3 通过；不代表原生 DDS 导航到达通过。**

- `_component_native_environment` 只保留 `LINGTU_NAV_COLLISION_*`，遗漏已编译的
  vehicle length/width 和 obstacle margin；因此 fixture 声明不一定等于 navd 实际几何。
- 修正仅额外保留三个既有键，无新协议、控制包装层、速度覆盖或 RobotConfig 修改。
  使用 RunPlan 的路径仍采用其原有完整 environment。
- [修改前](../../build/nav-mujoco-20261026/product/obb-env-red.log)、
  [修改后 3 项](../../build/nav-mujoco-20261026/product/obb-env-green.log)、
  [对应测试](../../tests/sim/test_mujoco_native_navigation_acceptance.py)。

## 2026-10-26：物理 benchmark 的模型选择混入 real 配置和另一套控制

**状态：组件入口本地修复，原生红绿对照与两项新短程 MuJoCo 通过；未切换正式产品模型或部署。**

- 旧 `body_physics` 只有 OBB 分支解析 `nav/real`，将 Thunder 的 below=0.25 带入
  仿真，未消费已有 sim package 的 below=0.05。原生 benchmark 又仅在 OBB 分支
  切换 global/local、follower、制动和内部 dt，38 部件分支使用旧手写值。
  这是实际配置来源错误；旧两模式结果不构成纯模型 A/B。
- 两模式现只解析一次同一 sim RunPlan，物理 MJCF、policy 与 native 配置同源。
  模型开关只决定几何与测量更新；选中 Body 贯穿全局、局部、Executor 和最终检查。
  保留 package 支撑高度，不再按初始化姿态手写覆盖。
- 另清理了 ambient `BODY_FILE` 对显式 benchmark 模型选择的污染；既有 native tuning
  override 继续支持并留存有效差异，避免保存 plan 时漏记实际参数来源。
- [原生红绿证据](../../build/nav-model-consistency-20261026/verification.json)：旧两模式
  控制不同，新两模式控制逐值相同；38 parts 与单 OBB 区别明确。Python 21 项回归通过，
  后续有效 override 回归 2 项通过，Ruff 通过。
- [新物理证据](../../build/nav-model-consistency-20261026/physics-verification.json)：
  同一 1.2 m＋90° 请求，parts 15.02 s、OBB 13.26 s 均到达，分别 0/3004、0/2652
  个不允许接触子步；两者共同显式保留 heading-error=0.4 rad。
- 正式 Thunder 的实时关节/FK 分部模型仍未接通；URDF-derived MJCF 物理接触、导航
  OBB 与组件部件模型的界线见 [配置审查](mujoco-navigation-20261026.md#模型来源与配置混用的修正)。
  不覆盖旧物理结果，不宣称动态横穿、完整 Product 或 Go2 实机已经通过。

## 2026-10-26：组合模型加载未应用配置水平余量

**状态：后续已本地修复，47 项几何、支撑、局部和配置窄回归通过；尚未部署。**

- 受支持的 `geometry.navigation_body_file` 非空时，
  [buildNavigationBody](../../src/nav/cpp/endpoint/nav/app/config/build.cpp) 直接返回 BODY2
  加载结果；文件未保存 margin，[Body](../../src/nav/cpp/collision/body.hpp) 使用默认 0.02 m。
  同一个 builder 的 OBB 分支才会将配置水平余量计入尺寸。
- [无运动复现](../../build/nav-model-consistency-20261026/audit-margin/result.json)：
  配置从 0.05 改到 0.12 m，OBB 的水平尺寸改变；同一个 BODY2 文件的实际 margin
  始终是 0.02 m。碰撞仍有检查，失效的是配置的额外间隙，不是全部碰撞检测。
- 正式 builder 现置 `margin_m=0`，把配置写入独立 `horizontal_margin_m`；全局、局部、
  动态和扫掠查询共同使用 world XY 余量，Z 不扩大。足底支撑证据不被扩大成额外接地要求，
  已确认地面的接触免除覆盖余量，侧障碍仍检查。单 OBB 的尺寸内 padding 不重复增加。
- [本地 47 项窄回归](../../build/nav-compound-model-20261026/native-margin-test.log)。
  复现时两个 real RobotConfig 尚未选择 BODY2；后续 Go2/Thunder 切换模型是独立的最新决定，
  不能倒推此前实机 OBB 的配置余量失效。

## 2026-10-26：正式模型接入尚缺同路径验证

**状态：后续已接入本地 formal real/sim 配置、平台资产与当前机身姿态；未部署。**

- 正式 sim package/schema/resolver/physics plan 已支持包内 `body_file`，沿原 compiler
  传入 `LINGTU_NAV_BODY_FILE`；不读取 real RobotConfig。Thunder real/sim 资产同次导出、
  字节一致，Go2 与 Thunder 使用各自来源和名义姿态。
- endpoint 现从完整机身变换构造固定名义关节＋当前 roll/pitch 的 Body，传入
  `ExecutionObservation.body_geometry` 和最终 `observedBody`；不宣称在线关节 FK。
- Go2 改用 raised MID-360 URDF，补旧配置声明的保守上部覆盖盒；尚缺 D435i/支架实测 CAD，
  该覆盖仅保存既有声明范围，不能证明实物已经完全包住。模型 metadata 保留未验收状态。
- 通用方法、内部格式、平台模型与证据边界统一记录在
  [导航碰撞模型与运动姿态](../design/navigation-body-model-and-motion-posture.md)。

## 2026-10-26：倾斜 TF 更新后规划 yaw 与碰撞姿态不一致

**状态：本地已修，固定失败回归转绿，未部署。**

- `projectTf()` 以前用 yaw 相加更新 `map_body`，完整机身变换却使用四元数组合；
  `map→odom` 有 roll/pitch 时，两种 yaw 可不一致。当前姿态 Body 去除的是完整变换的
  yaw，后续查询却再施加另一个 yaw，因此仅一次 TF 更新也能改变几何朝向。
- 现从同一个完整组合变换提取规划 Pose，再生成当前姿态几何；不更改模型尺寸或容差。
- [修改前失败](../../build/nav-compound-model-20261026/tf-regression-red.log)、
  [修改后姿态与快照回归](../../build/nav-compound-model-20261026/endpoint-snapshot-test.log)。

## 2026-10-26：正式名义组合模型 DDS 物理起步仍阻塞

**状态：实际 Thunder MuJoCo/DDS 短程未通过，当前默认切换尚未获运动验收；未部署。**

- 新 navd 使用正式 Thunder sim BODY2 与原生控制参数，真实 policy/MuJoCo 物理、
  native DDS、最终检查和 driver bridge 参与；环境使用隔离 domain 226。
  这是组件接线验证，没有执行 ProductControl 生命周期事务，也不是 Go2 实机。
- 全局路线已生成，但局部持续零命令，实际路径仅约 4.15 mm、净位移约 3.63 mm，
  不能记为行走成功。5294 个物理观察步未见外部非法接触，停车 ACK 与自有进程清理通过，
  这些单项也不能替代到达。
- 同份失败的完整 measured/free/inflated 位图、姿态和来源时间保存在
  [捕获目录](../../build/nav-compound-model-20261026/formal-physics/collision-capture/map.json)；
  对照 [运行报告](../../build/nav-compound-model-20261026/formal-physics/run-physics/motion/report.json)。
  [同位图原生重放](../../build/nav-compound-model-20261026/contact-diagnostic/README.md)
  已确认两个独立阻塞：
  1. 关闭额外 `supportHeight` 前置，Grid 仍返回占据；4 个足部均与 z-index=41 的
     地面体素相交，非足部没有实测占据命中。FR 足部当前模型底部为 0.0358517 m，
     命中格 Z=[0.0000000305,0.0500000313] m，上方为空闲，却没有有效接地 surface。
  2. 保留原支撑检查时，另有缺少占据/上方空闲证据的支撑列，例如 base_link_2 的
     [96,97]。修一处地面接触并不能据此宣布另一项也通过。
- 位图仅说明整格被占据，没有格内原始命中高度，不能把 voxel 命中直接解释为
  真实脚碰地；同一格也可能包含真实凸起。局部需要同源、覆盖当前占据证据的亚体素
  几何与正确支撑依据。不能把最后一帧点云当成累计占据格的完整历史来免除障碍。
- 保留失败，不降低穿地要求、不缩脚、不改实时地面输入来凑通过。此前组件实测关节
  快照的成功不能冒充本轮正式名义模型通过。

## 2026-10-26：删除局部整身逐列地面准入

**状态：本地删除已落实，ScanBodyTest＋LocalPlanTask 完整 49/49 通过，navd 重新编译成功，同位图重放已完成；未部署。**

- 官方固定提交 `348e8a5` 的 `getInflateOccupancy()` 只检查前后检测中心的膨胀
  占据，不要求每个机体部件投影下逐列存在地面及紧邻 free。这段整身地面前置是
  LingTu 的扩展。此前只剔除 AABB 空角仍保留了不合理准入，不能视作完整修复。
- 当前已从 local Grid 整段删除该循环，以及只服务于它的私有支撑高度状态与校验；
  不新增替代条件、开关或管理层。`supportHeightM` 和容差仍留在 route/global 链，
  既有全局三维支撑、高差和高度语义未删除。
- 实测占据与部件碰撞、明确 `support_required` 接触部件、地图边界、扫掠、动态预测
  和最终制动检查保留。没有缩小模型或扩大足部接触容差。
- [删除前 3 项定向检查](../../build/nav-support-gate-removal-20261026/red-test.log)：
  optional-foot 机体与异步 LocalPlanTask 两个正常场景失败，明确 required 接触脚的
  缺支撑反例通过；[删除后完整 49 项](../../build/nav-support-gate-removal-20261026/green-test.log)
  全部通过。该结果证明额外准入已移除且上述明确接触支撑检查仍有效，不代表足部
  体素量化或正式物理起步已修好。
- 局部圆角或优化轨迹偏离已检查的全局路径时，其新覆盖区域的支撑仍未验收；
  全局支撑检查存在，不能自动证明新圆角也有支撑。
- Windows MSVC Release 的完整 navd 已重新编译成功，见
  [构建日志](../../build/nav-support-gate-removal-20261026/navd-build.log)。
- [同位图重放](../../build/nav-support-gate-removal-20261026/same-bitmap-summary.json)
  确认新 Grid 的 34 个非接触部件查询均无碰撞；四个足部仍命中原来的地面占据格，
  完整模型仍返回占据。保留原始位图、姿态和 0.15 s 捕获时间差，没有重跑物理或改动输入。
- 本次验证证据放在 `build/nav-support-gate-removal-20261026/`。上节原始失败保留：
  去掉整身前置不能消除四个足部与 0～5 cm 地面占据格相交的独立问题，正式名义
  模型的 DDS/MuJoCo 起步与到达验收仍未通过。

## 2026-10-26：继续移除四处不合理阻塞与失效条件

**状态：本地四处修复完成，前述三处的 32 项检查及巡检重规划的三个测试程序通过，Windows navd 重建成功；未部署。**

- **人工直控被过期碰撞图误挡。** `manualModeMayBypassInputGate()` 原先只按
  `local_collision_` 前缀匹配，遗漏实际的 `collision_stale`，导致缺图可以人工脱离，
  图刚过期反而不能。现复用既有 `localCollisionInputHold()`，在入站、发布前复查
  和遥控处理共用处修一次。辅助避障模式仍拒绝旧图，驱动、急停、控制权和按键时效未绕过。
- **恢复等待错用旁路点云。** SCAN 实际搜索碰撞 Grid 时，旧判断仅看点云代际，
  可以在地图未更新时重复消耗尝试；也可能在图已更新时被旧点云挡住。现按当前后端
  实际使用的证据判断推进，使用现有地图 epoch/generation/sequence 及源时间。
  原点云 fallback、CMU traversability 规则保留，没有增加尝试次数、超时或开关。
- **已无效的支撑高度数值仍使轨迹失效。** 删除整身地面 gate 后，这两个具体数值
  不再参与局部碰撞或规划计算，但 Task 和 backend 仍因数值变化清曲线、重建 FSM
  或重发参考。现仅保留字段存在性变化导致的初始化切换，原值继续保存在请求和快照中。
- **巡检临时受阻提前结束航点。** 原先同一个 typed `local_recovery_exhausted`，普通
  目标会保留目的地重规划，巡检来源却先进入终止分支。现巡检目标也复用现有重规划、
  无路等待与后续重试；保留航点身份和到点容差，不再调用设备检查失败策略。
  明确取消、地图身份变化、外部目标被巡检接管以及实际设备动作失败的处理仍保留，
  未改巡检的公开状态机或 Stop/Retry/Skip 业务策略。
- 修改前的独立失败证据：
  [人工输入两项](../../build/nav-guard-pruning-20261026/manual-red-test.log)、
  [恢复等待](../../build/nav-guard-pruning-20261026/recovery-red-test.log)、
  [元数据改值清轨迹](../../build/nav-guard-pruning-20261026/metadata-red-test.log)、
  [巡检提前终止](../../build/nav-guard-pruning-20261026/inspection-red-test.log)。
- [修改后 32 项](../../build/nav-guard-pruning-20261026/green-test.log) 包括完整
  LocalPlanTask、六项恢复/观测关联回归及两个输入/遥控测试程序；
  [构建日志](../../build/nav-guard-pruning-20261026/green-build.log) 保留完整 navd 结果。
- [巡检重规划三程序](../../build/nav-guard-pruning-20261026/inspection-green-test.log)
  验证临时局部失败后首次重规划无路、等待后再次成功，原航点身份和位置/朝向容差保留；
  原有取消、暂停/恢复、地图变化及显式失败回归也在这些程序内。
  [巡检改动后的 navd 构建](../../build/nav-guard-pruning-20261026/inspection-green-build.log)
  验证主循环回调的实际接线可编译。
  这些是本地合同与组件证据，不证明实机卡顿或正式模型起步已经全部修复。

### 本次发现但未混入上述修复的边界

- 公开 `--check-obstacle=false` / `LINGTU_NAV_CHECK_OBSTACLE` 与显式 Body 同时使用时，
  Grid 仍强制检查 Body，Recovery search 的开关却只来自 params.checkObstacle。
  这是既有检查语义不一致，需单独明确该组合合同；当前正式 checkObstacle=true 路径
  不受它影响，本轮没有把它当作放行理由或已复现现场根因。后续修复见下节。
- 足部与 0～5 cm 地面体素相交及局部新圆角支撑覆盖仍是原有未验收项；本轮未重跑
  完整 MuJoCo、未部署 NX 或进行实机运动。

## 2026-10-27：继续核查重复拒绝与替换副作用

### 显式 Body 与关闭点采样的恢复语义已统一

此前问题在当前工作树中仍然存在，并非已经在其他模块修好：Grid 构造时显式 Body
会保持碰撞查询，Recovery search 却用第二个 `checkObstacles` 条件跳过该 Grid。
本轮删除搜索中四处重复条件，起点、连续平移和旋转都查询传入的同一 Grid；
Executor 的观测等待也按 SCAN 实际使用的 Body 碰撞图推进。没有 Body 的关闭检查
采样和 CMU 原有路径不变，未新增开关，也未修改模型或接触容差。

修改前 [三项回归全部失败](../../build/nav-guard-followup-20261026/body-switch-red-test.log)：
实际占据的机体可被恢复接受，途中平移及旋转碰撞可被漏过，同一份旧碰撞图也未等待刷新。

### 新目标被拒绝不再取消旧目标的重规划

`NavigationRuntimeController::submitGoal()` 原先先通知 `kNewGoal`，再判断新目标准入。
当 A 已进入重规划等待、B 因 `driver_heartbeat_stale` 被拒绝时，A 的 trigger 和重试
却已经被取消。修改前 [真实控制器回归失败](../../build/nav-guard-followup-20261026/submission-red-test.log)。
现保留入口终态屏障，只调用一次现有 submit，准入失败立即返回；仅已接受的外部目标
才取代旧重试。旧 deferred B 的终态仍用 B 的票据确认，不将已接受的新 C 误报为拒绝，
且确认完成前不能激活 C。未删除驱动准入，也未增加校验包装层。

### 足部地面量化仍未修好，不能写成其他模块已完成

全局规划已有从同版本 PCD 构建 `Terrain` 的原始点细化，入口为
`endpoint/nav/input/active/octomap.cpp::loadDeclaredTerrain()`。
但实时局部 `endpoint/nav/dds/codec.cpp::LocalCollisionMap::view()` 仍只装配位图等字段，
未设置默认空的 `LocalCollisionMapView::terrain`。因此 Grid 已有的原始命中解释能力
尚未贯通这条正式局部接线。

最新同位图重放仍是四足占据、34 个非接触部件无碰撞、`formal_motion_passed=false`。
FR 足底约 0.0358517 m，而命中格的上表面约 0.0500000 m，相差约 14.15 mm，不能满足
现有 5 mm 接触穿入容差。这是接触证据与体素量化的缺口，不能据此称为真实脚部穿地，
也没有通过缩模型、增容差或删除真实障碍查询来放行。

本轮未删除备用点云的处理：它仍参与恢复候选的障碍间距评分，并非纯无用输入。
点云容量策略和恢复候选排序均保持原状。本轮没有 NX 部署或新的 MuJoCo/实机运动。

### 本轮验证结果

- [恢复与观测回归 27/27 通过](../../build/nav-guard-followup-20261026/body-switch-green-test.log)，
  包含全部 RecoverySequence、RecoveryRotation，以及七项 Executor 相关检查。
  Body+false 的三项失败用例已转绿，无 Body 关闭检查的采样反例保留。
- [导航运行控制器测试程序通过](../../build/nav-guard-followup-20261026/submission-green-test.log)，
  验证被拒绝的新目标不会破坏旧目标重试，以及旧 deferred 目标确认前新路径不能激活。
- [完整 Windows navd 构建成功](../../build/nav-guard-followup-20261026/body-switch-green-build.log)，
  其中包含主循环回执条件修改；本轮触及文件的 diff whitespace 检查通过。
  上述结果不证明 Thunder 正式足部模型已经起步、到达或无接触。

### 接地规则来源及未闭合原因复核

本地保留的官方 SCAN `grid_map.h::getInflateOccupancy()` 查询前后两个圆心对应的
膨胀占据值，不包含组合模型每个脚底的逐列接地证明。这套 `support_contact` /
`observedSupportSurface()` 是 LingTu 组合碰撞适配新增的逻辑；不能把它的阈值或
完整观测要求说成官方 SCAN 的必要条件。当前 support.hpp 仍是未跟踪文件，Git
没有可用于精确归属其首次添加时间的提交历史。

失败链是：加入完整足部几何后，地面占据格与足部相交；适配层通过接地例外处理它，
却把体素上表面当接触高度、要求整块接触区域连续观测，并使用固定 5 mm / 30 mm
接触阈值。正式局部快照又未携带原始高度细化证据，因而不能区分量化膨胀与真实接触。
“必须完整证明地面，否则正常站立也不放行”并不是已经验证的产品要求。

本次更正了把体素上表面称为 observed surface 的源码注释；没有改变运动行为。
此前删除整身门槛后的组件测试证明的是指定规则变化，不是正式 Thunder 可以起步。
正式站立仍失败时应继续按失败处理，不能用通过的组件数量替代端到端结论。

## 2026-10-27：删除新增逐脚接地证明

**状态：清理已落盘并重建 Windows navd；未部署。逐脚证明已删除，原 Thunder 快照仍有四足占据格几何碰撞。**

- 删除 `collision/support.hpp`，移除 `observedSupportSurface()`、
  `mappedTerrainSupport()`、逐脚 occupied/free 列证明及其 5 mm / 30 mm 接地判定。
  全局 BodyQuery 与局部 Grid 均不再因某只脚缺少地面观测或未落地而否决。
- 删除 `Part::support_required`、诊断输出、Go2/Thunder 模型报告中的该字段、导出器
  和物理工具的承重准入。BODY2 与实验 P 流中原有最后一位作为格式占位读取，当前导出为零；
  该位置不再能启用任何导航判断，既有模型的几何尺寸、姿态和余量未改。
- 删除“逐脚证明可以替代全局普通支撑”的分支及 `require_center_support` 开关。
  全局三维路线继续使用其原有地面、支撑、台阶和坡度查询。
- 足部保留真实几何碰撞与底部接触：仅允许已标注接触部件触碰完全位于原始几何最低点
  以下的材料。墙、立面、上层楼板和实际进入足部几何的占据格仍检查；没有移除足部部件。
  原始点地形细化保留在 `collision/terrain.hpp`，不再承接逐脚承重证明。
- 清理两个直接遗留：Body 查询不再要求提供已不读取的 known-free 位图；无动态预测时，
  接触部件沿用普通机体的空间扫掠采样，不再为了逐脚证明重复密集时间采样。

验证目录：`build/nav-foot-proof-removal-20261027/`。

- [修改前回归](../../build/nav-foot-proof-removal-20261027/red-test.log) 复现：缺少脚边
  空闲/地面观测时，全局与局部同时拒绝没有真实障碍的机体。
- [最终 CTest](../../build/nav-foot-proof-removal-20261027/verification.log)：**132/134 通过**。
  两项未通过是 `PoseRoute.FailedBlendedSplineRestoresTheOriginalCheckedStages` 和
  `PoseRoute.MidBlendFailureKeepsItsCurrentReferenceInsteadOfAnOldTurnCentre`。
  共享工作树的转角实现同期另有变化，本次未把这两项记为通过，也未修改转角算法。
- [模型导出 11 项](../../build/nav-foot-proof-removal-20261027/export-tests.log) 和
  [物理工具 21 项](../../build/nav-foot-proof-removal-20261027/physics-tool-tests.log) 全部通过。
  所选 Go2/Thunder 模型重新导出后几何文件与原配置一致；这些工具测试不是完整物理导航验收。
- [最终构建](../../build/nav-foot-proof-removal-20261027/cleanup-final-build.log) 包含
  原生 navd、全局 heading/body 回归与组合模型执行工具，编译成功。
- [同位图重放](../../build/nav-foot-proof-removal-20261027/same-bitmap.json)：38 部件中
  34 个无碰撞，四足仍返回 occupied，整机 occupancy=1。当前拒绝来自实测占据格与足部
  几何相交，不再经过已删除的逐脚证明；不能称为已解决原快照的起步问题。

本轮没有 ARM64/NX 部署、完整 MuJoCo 导航或实机运动。

### 同格地面与足部位置核对（2026-10-27）

**状态：已核对模型与观测；正式实时局部地图的误判尚未修复。**

- 原 Thunder 仿真记录中的物理地面为 Z=0；仿真捕获的占据格覆盖约 0～5 cm。
  碰撞查询使用的固定名义模型四个足底约为 3.59～3.68 cm。
- 重放同一次 `run-collision-diagnostic` 保存的关节位置，经 MuJoCo 正向运动学计算，
  四个实际足部碰撞几何的最低点约为 -0.07 mm，接近仿真地面；分别比名义模型低约
  3.59、3.63、3.65、3.69 cm。所选关节样本比 odom 快照早 50 ms，地图快照比 odom
  早 150 ms；这是邻近时刻的比较，并非严格同步或真实机器人足部测量。
  计算结果见 [actual-foot-pose.json](../../build/nav-foot-proof-removal-20261027/actual-foot-pose.json)。
- 新增原生回归 `BodySupportMapTest.OriginalHitsDistinguishFloorAndObstacleInSameVoxel`：
  在占据位图完全相同的条件下，给同一格提供位于 0.2 cm 的合成地面点与位于 4.5 cm
  的合成凸起点。已有 `Terrain` 原始点细化同时让全局和局部查询区分两种情况；
  [测试 1/1 通过](../../build/nav-foot-proof-removal-20261027/same-voxel-test.log)。
  该用例使用合成球形足部，不是 Thunder 实际轨迹重放。
- `mapd` 实时碰撞层当前只发送体素位图，导航解码后的局部碰撞图没有对应的原始命中点。
  因此此回归不证明正式链路已接入细化；不能据此忽略足部所在占据格，也不能用单帧
  点云或旧保存地图代替同一时序的完整实时证据。

本次没有修改正式碰撞判断，没有重新进行完整 MuJoCo 导航或实机测试。

### 实时命中信息接入及 Thunder 平地回放（2026-10-27）

**状态：候选实现已编译并通过局部回归，原噪声场景仍未通过；不可视为实机就绪。**

- 滚动占据地图按体素维护实际回波的坐标上下界，随同一代占据位图经
  `MapCollisionLayer` 交给导航端，复用现有三维 `Terrain` 查询。上下界是对实际
  回波的保守包围，不是每一条原始点；未携带命中信息的格仍按原占据碰撞处理。
  体素被观测清除、滚出窗口或地图重置时，相应命中记录也清除。
- 新命中即使尚未达到历史占据概率，也进入组合机身的实测占据位图。
  同格地面与凸起的 DDS 解码到 SCAN 查询回归通过；没有重新引入逐脚接地证明。
- 协议新增 `hit_bounds`，并将分辨率改为双精度以保持体素边界一致。mapd 与 navd
  要使用同一版本的 DDS 消息定义；旧录制及其他读者的兼容性尚未验证。
- Thunder 模型仍由仓库中的 `v4_nominal_stand` MJCF 关键帧导出，实机与仿真资产
  重新导出一致。仿真关节记录对应的落稳姿态与该关键帧不同，不把单次仿真姿态
  写入实机标定；该固定模型也不代表在线关节运动学。
- [原捕获位图重放](../../build/nav-live-hit-20261027/original-snapshot-replay.json)
  没有原始命中数据，仍有四个足部占据。其原始点无法从位图中反算出来。
- 使用相同场景、噪声配置和 MuJoCo 原生 DDS 链重新运行两次：机器人的非零运动
  命令均为 0，位移约 3.63 mm，距离目标约 1.196 m；仍未通过导航验收。
  报告见 [第一次](../../build/nav-live-hit-20261027/mujoco/motion/report.json) 和
  [第二次](../../build/nav-live-hit-20261027/mujoco-capture2/motion/report.json)。
  其中 [同帧地图与观测](../../build/nav-live-hit-20261027/captured2/observation.json)
  的观测序号均为 120；机身姿态是邻近时刻采样，相差约 0.32 s。
- 仿真使用 MID-360 近距离 0.03 m、远距离 0.02 m 的径向噪声标准差；
  在平地附近捕获到约 5～6 cm 高的原始回波。历史极值在占据格保持占据期间累积，
  让地面拟合失败，随后足部和小腿仍被判碰撞。它不能仅通过删足部或忽略这些格解决：
  单次含噪平地回波与相近高度的真实凸起可能无法从同一条测量区分。
  [逐部件回放](../../build/nav-live-hit-20261027/captured2-replay.log)保留具体证据。
- 当前应继续评估基于多次真实观测的地面/障碍区分，并用带噪地面、同格小凸起、
  侧墙和新出现障碍做漏检、误停与延迟对照；不能以关闭传感器噪声或放弃障碍
  检查作为原场景通过的证据。

未进行 ARM64/NX 构建、部署或真实机器人运动。

### 平地复测与轨迹重定时补充（2026-10-27）

**状态：确定性体素误判和重定时失败已修正；原噪声 MuJoCo 仍未通过，不可部署为已验证的行走修复。**

- 局部碰撞仅在该体素中保存的全部命中高度低于相关部件的真实底部时，排除体素厚度造成的碰撞；没有命中数据或有命中进入部件高度时仍沿用碰撞检查。地面拟合覆盖实际查询的水平余量，不再要求足部中心那一列必须有点。这不是逐脚承重证明。
- [无噪声捕获的同快照修复前](../../build/nav-live-hit-20261027/no-noise-before-replay.json) 四足均被判占据，[修复后](../../build/nav-live-hit-20261027/no-noise-ground-replay.json) 四足与整机均无占据；这是显式关闭传感器噪声的几何对照，不是原噪声验证。缺失真实命中数据的旧快照不能反推这些数据。
- 带噪捕获中，[累计命中](../../build/nav-live-hit-20261027/noisy-ground-replay.json) 仍有四足判占据。把同一帧回波单独重建为诊断位图与命中，仍有三足判占据（[逐部件日志](../../build/nav-live-hit-20261027/single-frame-hit-box-replay.log)）；部分回波的包围盒确实与加余量的足部几何相交。因此不能仅靠缩短历史保留或排除所有不相交的包围盒解决。该单帧替换仅用于定位，不作为正式地图或安全方案。
- 对该单帧逐点核查，造成前左足判碰的 4.71 cm 回波实际位于加余量后的足部几何内，不只是命中范围盒误差（[记录](../../build/nav-live-hit-20261027/single-frame-exact-hits.log)）。因此逐点替代范围本身也不能完全解决。
- 仅离线试用仓库已有 Patchwork++ 地面分割：原帧中该回波仍被标为非地面；在原帧额外加入 169 个 4.5 cm 高合成凸起顶面点时，57 个被标地面、112 个被标非地面（[原帧](../../build/nav-live-hit-20261027/ground-probe-plain.txt)、[叠加凸起](../../build/nav-live-hit-20261027/ground-probe-bump.txt)）。这是人工叠点实验，不是真实遮挡的传感器场景，不能据此判断整个凸起是否漏检；现有分割不能直接作为本次导航放行依据。
- 另用 MuJoCo 平地与方形凸起几何、捕获帧的射线方向和同档径向噪声生成标签对照；四个噪声种子下，4.5 cm 凸起的 56 个几何命中中有 14～25 个获得地面标签；纯平地 2865 个命中中有 486～496 个未获得地面标签（[记录](../../build/nav-live-hit-20261027/ground-probe/physical-results.txt)）。未获地面标签不等于算法已断言是障碍；单点标签也不证明整块障碍会漏检。这是简化静态几何和单帧试验，不是完整导航或现场传感器验证。
- 原仿真记录中机器人稳定后的足部位置与固定的 `v4_nominal_stand` 不同。名义组合模型仍与配置和导出一致；没有把单次仿真关节姿态写入实机标定，也没有宣称在线关节运动学已接入。
- 另一个失败是：接近静止但带非零实测速度时，局部轨迹反复重新拟合后仍略微超出速度上限而放弃。现优先使用保留端点速度、加速度且通过动力学和碰撞检查的直接重定时结果；若不能通过，仍走原拟合流程。[回归](../../build/nav-live-hit-20261027/retime-regression.log) 11 项通过。
- 同 Thunder 地图和目标的无噪声 MuJoCo 对照，修复前约 3.63 mm 位移、没有非零运动指令；修复后实际位移约 1.195 m，终点平面距离约 0.066 m，但没有完成目标状态验收，另有反向指令和局部地图数据过期（[报告](../../build/nav-live-hit-20261027/mujoco-no-noise-retime/motion/report.json)）。
- 原噪声、原目标重新运行后，仍无非零运动指令，位移约 3.63 mm，目标距离约 1.196 m，未到达（[最终版本报告](../../build/nav-live-hit-20261027/mujoco-final-verified/motion/report.json)）。本测试只覆盖原生 DDS 与 MuJoCo 物理组件，不覆盖 ProductControl、实机传感器、LTLIO 或动态障碍验收。
- 失败快照新增同步命中范围，避免只凭位图重放错误碰撞；[快照测试](../../build/nav-live-hit-20261027/snapshot-regression.log) 2 项通过，最终 MuJoCo 失败快照记录了 10066 个命中范围。[定向导航测试](../../build/nav-live-hit-20261027/final-targeted-tests.log) 42 项、地形测试 13 项、支撑地图测试 27 项和地图服务测试 3 项通过。此候选有额外数据量和实时开销，旧 DDS 读者及 NX 性能尚未验证。

下一步应在有标注地面和不同高度真实凸起的含噪序列上，验证能否利用传感器噪声、空间与时间观测区分地面和障碍，并测量漏检、误停和处理延迟。不能简单把低于某个固定高度的回波全部忽略。

### 含噪地面、障碍与楼梯雷达验证（2026-10-27）

**状态：修正一处体素几何误判及仿真建图输入；原含噪平地、组合模型楼梯规划仍未通过。**

- 实时局部碰撞查询现在使用每个占据体素内的完整命中包围范围与部件几何相交；若范围全部在部件外，不再把整个体素当成碰撞。保留部件的水平余量，真实命中或缺少命中范围的体素仍按碰撞处理；没有增加逐脚接地证明。这个更精细的几何修正尚不能区分落在部件内部的地面噪声与真实障碍。
- 在纯平地、4.5 cm 高方形凸起、4.5 cm 高窄凸起及侧墙的 MuJoCo 静态射线试验中，四个噪声样本的平地仍全部判占据。仅为诊断，使用 MuJoCo 几何标签移除地面回波后，四组方形凸起、窄凸起和侧墙的真实回波均仍判碰撞（[记录](../../build/nav-ground-noise-20261027/object-only.json)）；生产代码不会获得这些标签。射线方向取自旧仿真快照，包括测试夹具的支持射线；射线重建的起点使用的是近似机身中心，不是经过前置雷达标定的实际传感器位置。因此这是简化几何压力试验，不代表实际雷达视场、现场雷达数据或全物理避障测试。
- 用四帧、三维邻域和已知地面高度做离线原型：扩大到 10 组独立噪声序列后，阈值 40 下平地仍有 2/10 误判。新出现窄凸起的首帧有 4/10 没有碰撞输出；余下 6/10 只有整机碰撞输出，不能直接说都检出了凸起。按仿真真值拆分过滤后的回波重查，仅 2/10 中单独的凸起点足以触发碰撞；5/10 的地面点独自也触发碰撞，其中 1 组同时有凸起碰撞（[原结果](../../build/nav-ground-noise-20261027/temporal3d-more.json)、[拆分归因](../../build/nav-ground-noise-20261027/temporal3d-attribution.json)）。此前把 6/10 都称为检出是错误的。序列复用固定射线方向，原型依赖真值高度，**没有接入生产**；首帧结果不代表障碍持续出现后的最终检出率。
- 原平地、低凸起、侧墙分别进行了当前修改前后的 Thunder MuJoCo 物理对照。三组前后均为非零运动命令 0、位移约 3.63 mm、离目标约 1.196 m，均未到达；未发生场景障碍接触，不能把静止当作避障通过。新结果见 [平地](../../build/nav-ground-noise-20261027/physics/flat/motion/report.json)、[凸起](../../build/nav-ground-noise-20261027/physics/candidate/bump/motion/report.json)、[侧墙](../../build/nav-ground-noise-20261027/physics/candidate/wall/motion/report.json)。该链路使用真实 MuJoCo 物理和雷达，但定位是仿真真值夹具，未覆盖 LTLIO、ProductControl 或实机。
- 仿真建图现在将每帧 MuJoCo 雷达的实际发射位置与命中一并存成现有的射线束格式；缺帧、采集超时、遗留扫描和人工合成支撑点不会被误标为完整真实雷达记录。Thunder 楼梯采集显式使用已配置的前置倾斜雷达，非另行修改实机外参。
- 楼梯的 90 个虚拟观测位姿收集了 840871 条裁剪后雷达射线，地图标记为 `saved_rays`、`navigation_ready=true`。简化圆柱模型的上下方向各得到 53 点、约 5.412 m 的路径；在相同地图上用正式 38 部件组合模型、Thunder 平地测试配置进行原生三维规划时，上下方向均在起点报告 `FR_calf_17` 占据，没有路径（[上行](../../build/nav-ground-noise-20261027/stair-native-probe/front_up.json)、[下行](../../build/nav-ground-noise-20261027/stair-native-probe/front_down.json)）。采集是虚拟位姿巡检，不是物理上下楼；两个模型的结果不可混作组合模型通过。
- 对这两个起点的首个拒绝格逐点检查，上行格包含 139 个保存点、下行格包含 321 个保存点，均没有点落入相应小腿包络（[上行明细](../../build/nav-ground-noise-20261027/stair-native-probe/front_up_debug.log)、[下行明细](../../build/nav-ground-noise-20261027/stair-native-probe/front_down_debug.log)）。当前全局仍使用占据立方体与地面拟合；不能直接把任意 `map.pcd` 的现存点视为该体素全部射线命中，因为地图清理及保存射线构图允许采样点与命中不完全重合。需先建立完整命中证据，再检验真实障碍与上下楼路径。
- 针对碰撞和射线束的本轮测试：`BodyTerrain` 14 项、`ScanBodyTest` 28 项、Python 射线采集 6 项通过。未在 ARM64/NX、真实机器人或完整的物理上下楼场景验证，当前状态不能作为部署行走的依据。

### SCAN 双圆柱同命中几何对照（2026-10-27）

- 固定官方提交 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` 的示例参数：体素 5 cm、双圆柱半径 25 cm、前后偏移 18 cm、Z 方向上下各膨胀 10 cm；官方在运动轨迹高度的前后中心查询膨胀体素，不逐个检查固定姿态足部。
- 同一批单帧 MuJoCo 噪声命中、同一 Thunder 位姿，将所有命中直接视为占据，用官方双圆柱膨胀／查询公式独立对照：4/4 组平地在双圆柱下为空闲，而当前组合模型 4/4 判碰撞。将障碍回波单独留下时，4/4 低凸起、窄凸起和侧墙在组合模型下判碰撞；同参数双圆柱都为空闲。[明细](../../build/nav-ground-noise-20261027/same-hit-footprint-comparison.json)。仅以每类第 0 个样本，用本仓库无 Body 的原生 Grid 查询与公式互相核对，4/4 一致（[结果](../../build/nav-ground-noise-20261027/legacy-query-native-crosscheck.json)）。
- 雷达点来自带标注的简化静态几何，射线方向复用旧仿真快照；公式对照没有运行官方 ROS、官方概率融合或完整规划器，也不是 MuJoCo 运动或实机试验。低凸起是否可由真实步态跨越、侧墙是否实际接触均未验证，不以包络碰撞冒充物理接触。

### 去除小腿和轮子碰撞的隔离对照（2026-10-27）

- 只在实验文件中移除 Thunder 组合模型的 16 个小腿／轮子原语，保留 22 个机身、髋和大腿原语；未修改物理 MJCF，也未切换正式 RobotConfig。相同快照的 4/4 带噪平地从占据变为空闲，但单独的 4.5 cm 凸起和窄凸起均不再触发包络碰撞，侧墙只有 2/4 组仍触发（[数据](../../build/nav-ground-noise-20261027/no-lower-leg-snapshot.json)）。
- 该删减模型在带噪 MuJoCo 平地移动约 1.188 m、报告到达，反向指令比例仍未通过；轮子路径以外的 4.5 cm 凸起场景移动约 1.200 m、报告到达，仍有反向指令比例和输入门验收项失败。相同雷达楼梯地图中，删减模型上下方向均生成 53 点路径；这是规划结果，非物理上下楼。
- 将 4.5 cm 凸起置于前轮路线上，物理仿真记录了前轮接触；轮子与低凸起接触可以是正常通行，不单凭接触断言发生事故。将障碍改为 16 cm 高硬凸起后，前轮首次接触面法向约为水平方向、穿入约 3.17 mm，累计 1645 个仿真步记录到场景接触，本次没有到达目标（[报告](../../build/nav-ground-noise-20261027/physics/no-lower/wheel_obstacle/motion/report.json)）。这说明直接删下肢的方案无法满足同时保留该真实障碍检查的要求。
- 另试去掉逐部件下肢，再加一个离地的整体下部包络；简化静态侧墙样本 4/4 检出，但 16 cm 凸起的物理测试仍有前轮水平接触（[报告](../../build/nav-ground-noise-20261027/physics/proxy/wheel_obstacle/motion/report.json)），楼梯全局规划又被起点的地面噪声占据格阻塞（[记录](../../build/nav-ground-noise-20261027/stair-native-probe/proxy_up.json)）。两个候选均未进入正式配置。

### 只用雷达回波的演示对照（2026-10-27）

- 功能测试夹具原来向实时点云添加了 442 个合成地面点。新增的 `local_scan_lidar.json` 可关闭这些点，保留真实 MuJoCo MID-360 射线回波；负 X 起点的参数传递也已修正。该预设若自行生成地图仍沿用原有合成地图，使用同场景保存射线地图才能避免这一点。
- 同一平地的传感器位姿来自 Thunder 的配置，生成 66 个虚拟观测位姿、683963 条有效射线的 [保存地图](../../build/nav-ground-noise-20261027/corridor-lidar-front/same_source_map/metadata.json)，没有合成地面点；虚拟位姿采集不是机器人行走建图，也不是 LTLIO。
- 同样使用该射线地图和无合成地面的实时雷达，正式 38 部件模型位移约 0.0036 m，未起步；隔离的 22 部件候选移动约 1.182 m，离目标约 0.084 m，任务没有报告到达。两组均无场景接触记录（[原模型](../../build/nav-ground-noise-20261027/physics/full-body/flat_actual_lidar_survey/motion/report.json)、[候选模型](../../build/nav-ground-noise-20261027/physics/no-lower/flat_actual_lidar_survey/motion/report.json)）。
- 在同一射线地图基础上，向场景添加 60 cm 高箱体，22 部件候选实际绕行约 4.363 m，净位移约 3.595 m，任务报告到达，未记录场景碰撞；反向指令比例与输入新鲜度仍未通过该夹具的验收（[报告](../../build/nav-ground-noise-20261027/physics/no-lower/long_visible_box_actual_lidar_survey/motion/report.json)）。箱体是添加到已保存地图之后的静态障碍，不是动态运动物体。
- 把一条已观测点放入前轮位置、高出地面 14 cm，原模型和带整体下部包络的候选均判占据；只删下肢的 22 部件候选判空闲（[查询结果](../../build/nav-ground-noise-20261027/observed-low-obstacle/)）。这是直接构造已观测点的碰撞查询，不是对传感器盲区的推测。因此仅删下肢会漏掉部分已经看见的低障碍，尚不能同时满足保留障碍避让的要求。
- 使用同场景射线地图和无合成地面实时雷达的 16 cm 障碍物理试验中，前轮接触硬障碍的侧面，约 2.90 mm 穿入，报告了 73 个接触仿真步；任务随后仍报告到达（[报告](../../build/nav-ground-noise-20261027/physics/no-lower/long_wheel_obstacle_actual_lidar_survey/motion/report.json)）。箱体未写进事先保存的地图。另一次同几何试验中，距离较远时该障碍在 Thunder 传感器配置的视场之外；使用当次物理位姿重放 4000 和 15000 条射线均没有命中，靠近后才进入视场（[视场检查](../../build/nav-ground-noise-20261027/far-front-sensor-fov.json)）。这是仿真配置，不证明实机标定或 Go2 视场。
- 在 15 cm 踏步的 `stair_easy` 场景，使用同场景雷达射线地图、实时雷达和 4998 普通模型，上楼移动约 1.820 m，停在首阶前、未到达；下楼净位移约 5.149 m、距目标约 0.004 m，但未报告到达，且记录到轮子对阶面侧向接触（[上楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_up_actual_lidar_4998/motion/report.json)、[下楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_down_actual_lidar_4998/motion/report.json)）。不等于上下楼验收通过；本组没有切换楼梯模型。直接从起点加载实验 1119 模型时，上下方向的实际站姿相对所在层地面的高度分别约 0.389 m 和 0.382 m，规划都报告起点受阻，没有产生路线（[上楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_up_actual_lidar/motion/report.json)、[下楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_down_actual_lidar/motion/report.json)）。
- 这些试验均为 Thunder MuJoCo 物理 + 真值定位的组件诊断，并非 Go2 实机或完整 Product 链路。22 部件模型仍仅供隔离测试，正式 Go2 / Thunder 配置未切换；下一步应在不引入逐脚接地证明的前提下覆盖可观测低障碍，再处理楼梯路线高度与实际站姿的衔接。

### Thunder CTS MoE v39 模型与楼梯复测（2026-10-27）

- 上一组直接使用 1119 是旧模型试验，不是用户提供的最新模型。现在 Thunder 楼梯仿真绑定用户提供的 `model_61128.pt` / `student_61129.pt` 导出的静态 ONNX；普通行走仍使用 4998。原 ONNX 的计算图和权重未改，只按原始部署契约添加运行时所需元数据。[适配对照](../../build/cts-v39-model61128-20261027/parity.json)的 16 组网络输出完全一致，40 个连续状态的观测、目标和电机力矩也对齐；这不证明真实步态能力。
- 在模型包自己的 MuJoCo 环境中，0.5 m/s 指令、2 秒静止后运动 5 秒，机器人前进约 2.15 m，已踏上 15 cm 台阶，但未到楼梯出口（[报告](../../build/cts-v39-model61128-20261027/reference-forward-15.json)）。同一 v39、同一楼梯演示场景、同一指令，旧地面摩擦 `1.0 0.5 0.5` 运动 8 秒仅前进 0.18 m；调整为 `1.0 0.03 0.0002` 后为 3.34 m（[对比视频](../../build/cts-v39-model61128-20261027/v39_friction_comparison.mp4)、[数据](../../build/cts-v39-model61128-20261027/friction-ablation.json)）。旧场景的高转动摩擦是主要原因，场景现已修正；新数值来自模型包的仿真假设，并非现场地面标定。
- 修正后的十二级楼梯演示在 MuJoCo 真值朝向反馈下，真实切换 4998 → v39 → 4998：上楼和下楼均到达对应出口并停住，期间无非足部场景接触（[上楼](../../build/cts-v39-model61128-20261027/stair-corrected-handoff/cts_moe_up/result.json)、[下楼](../../build/cts-v39-model61128-20261027/stair-corrected-handoff/cts_moe_down/result.json)）。这是运动模型及接口切换测试，未使用导航、SLAM，也未验证实机模式切换。
- 真值定位、4000 条带噪 MuJoCo MID-360 射线、同场景虚拟射线地图、22 部件隔离候选和模型包摩擦条件下，将 v39 从起点直接加载到原生导航链：15 cm 台阶上楼净位移约 4.42 m、最终水平误差 0.76 m，进入 `local_recovery_exhausted`；下楼净位移约 2.00 m、最终水平误差 3.16 m，未到达，并有输入过期（[上楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_up_v39_training_lidar/motion/report.json)、[下楼](../../build/nav-ground-noise-20261027/physics/no-lower/stair_easy_down_v39_training_lidar/motion/report.json)）。两次均记录轮子与台阶立面接触；是否属于可接受的楼梯动作仍需结合力与步态判断，不能将其直接当作无碰撞通过。下楼约 25.5 s，局部目标与机身水平位置接近，但目标 Z 比机身高约 0.23 m，输出主要为侧移，随后没有继续下降；高度不一致的最初成因仍需定位。这两次没有自动楼梯模型切换，不能代表 Product 完整流程通过。
- 相同 v39、22 部件候选、无合成地面点的实时射线与同场景保存射线地图，还分别运行了平地、4.5 cm 凸起、侧墙、60 cm 箱体和 16 cm 低障碍。五次均未通过完整任务验收：平地净位移 0.87 m、终点误差 0.33 m；凸起净位移 1.10 m、误差 0.30 m并有轮子接触；侧墙净位移 1.62 m、误差 0.47 m；60 cm 箱体净位移 3.52 m、误差 0.28 m，未记录场景接触；16 cm 低障碍有 41 个轮子侧面接触仿真步，首次穿入约 2.98 mm。记录分别为 [平地](../../build/nav-ground-noise-20261027/physics/no-lower/flat_v39_lidar/motion/report.json)、[凸起](../../build/nav-ground-noise-20261027/physics/no-lower/bump_v39_lidar/motion/report.json)、[侧墙](../../build/nav-ground-noise-20261027/physics/no-lower/wall_v39_lidar/motion/report.json)、[箱体](../../build/nav-ground-noise-20261027/physics/no-lower/visible_box_v39_lidar/motion/report.json)、[低障碍](../../build/nav-ground-noise-20261027/physics/no-lower/wheel_obstacle_v39_lidar/motion/report.json)。平地与侧墙报告中的首个接触是轮子与地面，不能仅凭报告将其视为障碍碰撞；低凸起的轮子接触也不能单凭接触认定事故。物理模型仍保留所有腿和轮子碰撞。
- 平地轨迹曾同时接近目标：约第 9 秒，水平误差 1.19 cm、高度误差 1.19 cm，但仍保持 `spline_control_ready`，随后离开并进入恢复；采样日志没有出现水平与高度误差同时不超过 1 cm 的时刻。`Executor` 中间姿态阶段使用 1 cm 位置容差，可能妨碍向最后转向阶段切换；现有记录未包含每拍姿态阶段编号，尚不能据此认定这就是唯一原因。下一步应重放该状态并核对阶段切换与当前位置转身扫掠，不能单纯放宽容差后宣称解决。
- 全局地面支撑高度此前按体素中心计算，忽略机身实际落在格内的偏移，造成某些合法起点被错判；现按实际机身高度计算，并考虑占据格的上半部分。起点、坡面、实际支撑缺失和障碍相关的 5 个原生回归通过。仍未验证物理楼梯全部导航通过。

正式 38 部件碰撞模型没有切到删小腿与轮子的 22 部件候选：该候选对已观测的 14 cm 凸起判空闲，但仅凭高度或碰撞查询结果，不能判断它应被跨越还是绕开；完整场景的通过能力与不可跨越物体处理仍需验证。没有部署到 Thunder 或 Go2，也没有实机运动。

### 16 cm 凸起接触结果澄清（2026-10-27）

上面“低障碍避让未通过”的表述容易让人误以为机器人被障碍卡住，现更正：此场景的物体是右侧轮迹上的 **14 × 14 × 16 cm 小方块**，不是横跨通道的台阶。记录中右前轮在仿真时间约 5.79 s 与方块侧面接触，约 6.0 s 右前轮已抬起；约 7.3 s 四轮均已通过方块所在的前后范围。机器人随后继续前行，到过目标附近；任务最终没有完成，末帧仍距目标约 1.03 m。接触本身不能单独判为越障失败，也不能据此认定任务已成功。

[双视角视频](../../build/cts-v39-model61128-20261027/obstacle-replay/thunder-v39-16cm-obstacle-replay.mp4)按原记录约 10 Hz 的 MuJoCo 关节和机身状态重建，以 0.5 倍速播放；橙色仅为突出原场景中的方块。首次接触的 5.79 s 来自物理事件记录，最近的重放画面约为 5.80 s，不能据此判断每个接触瞬间的力或部件。用户提供的 23 cm 通行能力尚未在此场景及实机验证，本次仅证明该仿真记录中的这块方块已被通过。对于可以跨越的地形，不应仅凭有接触或处于下肢区域就判为必须绕开的障碍；真实不可跨越的障碍仍须分别判断。

### v39 物理箱体避障与正式模型起步复测（2026-10-27）

- 使用上述同源射线地图及无合成地面点的实时 MuJoCo 雷达，在原路径上添加有碰撞几何的静态箱体，尺寸 40 × 50 × 60 cm；箱体不在事先保存的地图中。22 部件隔离候选绕过箱体，仿真逐步接触记录中没有场景接触，但目标附近反复调整，最终未报告到达，末帧水平误差约 0.28 m（[报告](../../build/nav-ground-noise-20261027/physics/no-lower/visible_box_v39_lidar/motion/report.json)）。[完整状态回放](../../build/cts-v39-model61128-20261027/box-avoidance-replay/thunder-v39-box-full-replay.mp4)由保存的机身和关节状态重建，蓝线和目标标记是可视化辅助；不是实机录像。
- 同场景改用正式 38 部件规划碰撞模型，全球规划返回 `start_body_occupied`，没有生成可执行路径（[报告](../../build/nav-ground-noise-20261027/physics/full-body/visible_box_v39_lidar/motion/report.json)、[日志](../../build/nav-ground-noise-20261027/physics/full-body/visible_box_v39_lidar/motion/navigation.log)）。这次尚未确定触发占据的具体部件，不能归因于箱体或认定正式模型已通过避障。
- 到点失败的轨迹复核：本轮状态报告的水平到点范围为 0.15 m，高度容差为 0.35 m。MuJoCo 真值从约 9.8 至 12.6 s 连续处于水平范围内，10.0 s 水平误差约 0.011 m；12.3 s 状态进入 `execution_stalled` 和恢复平移，随后离开目标。不能以末帧误差 0.28 m 解释此前为什么没有到达。记录没有保存执行器的姿态阶段编号，尚未确认是哪条阶段条件阻止了到点；接近期间仍在移动，也不能据此证明已满足静稳条件。
- 同次正式模型试验中，记录的速度指令始终为零，v39 仿真机身约 24 秒内后退 0.53 m；原因尚未隔离，也不能外推到实机。以上均使用仿真真值定位，未验证 LTLIO 或完整产品链；物理腿和轮子碰撞始终保留。静态箱体结果不证明移动障碍避让通过。

### 最终到点阶段衔接与箱体复测（2026-10-27）

- 中间姿态阶段使用 1 cm 容差。最后平移接同位置终点转向时，已进入任务到点范围仍可能被前一阶段约束住。回归先复现该情况，再改为直接交接到终点转向：仍检查任务高度、朝向、转身碰撞和停稳；若仍在平移则先停，再转。普通中间拐点和楼梯阶段不变。箱体旧记录没有姿态阶段编号，不能证明该逻辑是旧失败的唯一原因。
- 同一个实体箱体和 v39 雷达场景使用当前源码重建基线，已绕过箱体并报告到达：终点水平误差 0.006 m，零场景接触，验收通过（[基线](../../build/arrival-box-20261027/baseline/motion/report.json)）。修改后两次都报告到达，水平误差分别约 0.056 m、0.054 m；第一轮记录一个地面与轮子接触步，穿入约 6.9e-8 m，因此验收仍失败；第二轮零场景接触、验收通过（[首次](../../build/arrival-box-20261027/after/motion/report.json)、[复测](../../build/arrival-box-20261027/after-repeat/motion/report.json)）。这不能据单次基线认定到点性能提升。
- 三次均为 22 部件隔离规划候选、实际 MuJoCo 物理机身和腿轮、模拟雷达和真值定位。正式 38 部件模型起点受阻问题不在本次修复范围内。姿态路线 48 项、到点相关执行器 4 项回归通过，不代表实机或完整产品验收。

### MuJoCo 雷达自身遮挡（2026-10-28）

- 旧射线查询排除整台机器人，可能错误返回机身后方的环境点。现按机器人光学几何判断首次遮挡，丢弃被遮挡的环境回波，也不发布自身命中；精确视觉网格优先于粗物理碰撞盒，没有视觉网格的部件使用碰撞几何。雷达本体与其支架需要正确拆分资产，视觉网格与实物仍需对照。
- Thunder 箱体记录中同一姿态、同一 4000 条射线、无噪声对照：未考虑自身遮挡时 3030 个环境回波，启用遮挡后 2249 个，移除了 781 个机身遮挡后的回波（[数据](../../build/arrival-box-20261027/self-occlusion-snapshot.json)）。固定几何测试覆盖无遮挡、自身遮挡、近于最小测距距离的遮挡、部件运动和传感器旋转。
- 沿用同一个箱体和旧的保存地图，启用遮挡后的真值定位组件仿真绕箱并报告到达，终点水平误差约 0.062 m，1998 个物理步中无场景碰撞记录，雷达约 10.02 Hz，累计 72685 个候选环境回波被自身遮挡（[报告](../../build/arrival-box-20261027/self-occlusion-final/motion/report.json)）。该保存地图是在旧遮挡模型下生成；不能将本次当作新模型建图、LTLIO、完整产品或实机验收。CPU MuJoCo-LiDAR 已验证，其他后端未做完整运行验证。
- 使用新遮挡模型在 66 个固定虚拟位姿采样并保存地图，436666 个有效端点，未插入合成支撑点（[地图报告](../../build/arrival-box-20261027/occlusion-map-final/report.json)）；这不是机器人行走建图。以新地图和同一箱体运行真值定位组件导航，报告到达，终点水平误差约 0.065 m、2109 个物理步未记录场景碰撞、雷达约 10.06 Hz（[导航报告](../../build/arrival-box-20261027/occlusion-map-navigation/motion/report.json)）。不等于 LTLIO、真实现场或长期连续运行已验证。

### 三层地图与正式机身楼梯试验（2026-10-28）

- 同一 MuJoCo 世界用 MID360 模拟射线及自身遮挡在 222 个外部放置的位姿采样，得到 0、1.8、3.6 m 三层地图，未加入合成支撑点。原生 OctoPlanner3D 的简化包络上下行分别生成约 30.34 m、30.49 m 路线（[地图与报告](../../build/stair-multifloor-20261028/README.md)）。这不是机器人实际爬楼或 LTLIO 建图。
- 同源的两平台、15 cm 台阶场景中，正式 38 部件最初被 `FR_calf_17` 与地面粗体素的相交挡住；该格的 79 个命中点均在部件外。全局碰撞现可在完整、未人工编辑的保存扫描上使用真实命中细化；人工编辑地图保留体素判断，可能更保守。原地面误判解除后，`FR_foot_19` 仍与 141 个带噪地面回波相交。预加载命中索引后原生规划约 5.4 ms 返回 `start_body_occupied`，没有生成路线或运动（[复跑报告](../../build/stair-interactive-20261028/full-up-warm/motion/report.json)）。不能说正式模型已经能上下楼。
- 22 部件隔离候选在同一楼梯场景生成了路线并上到首阶附近，之后实际高度落后于全局参考，局部规划反复报告起点碰撞并最终恢复失败，目标仍有 XY 2.57 m、Z 0.84 m 误差（[报告](../../build/stair-interactive-20261028/candidate-up/motion/report.json)）。足轮接触台阶本身不能判定登阶失败。窄凸起 4/10 来自另一项时序滤波实验，不是 22 部件模型的结论。
- v39 零命令仍输出轮速，20 秒零命令时曾后退约 0.38 m；启用已有的零命令轮速归零后，相同物理场景的前进恢复时间均为 5.02 s，停止后 5 秒净后退由约 8.85 cm 降至约 0.27 cm（[对照](../../build/stair-interactive-20261028/idle_resume_postfix.json)）。这是仿真策略适配的证据，不证明实机或楼梯中途停稳。
- 三层 ProductControl 仿真在零任务状态运行数分钟后，Windows 状态快照的原子替换失败导致传感器进程退出。现保留旧快照、继续发布并在下个周期重试，Windows 实际文件占用回归确认快照恢复及雷达／里程数据继续；过期时间未延长。具体占用者尚未确认，完整三层长期稳定运行尚未验收。
- 三维表面选点以前把点云表面 Z 直接当机身目标 Z；现使用当前 Product 会话的机器人配置支撑高度转换一次，同层选点不变。墙面仍可能被点选，最终可行性仍需原生三维规划检查。三层隔离候选使用不含小腿／足轮的 22 部件规划模型、完整 MuJoCo 物理机身及固定 v39：一次网页 0.6 m 平地目标报告到达，实际移动约 0.22 m，末帧距目标约 0.39 m，页面到点范围 0.45 m（[记录](../../build/stair-multifloor-20261028/README.md)）。未记录物理接触，也未完成上下楼或步态自动切换验证。
- 同一三层地图的一层至三层正式链路预览在 7 秒超时；采样后的 XYZ 简化先导使用配置半径 0.390512 m 时能找到候选，但组合机身姿态和扫掠尚需验证。旧先导强制取约 0.581721 m 外接圆，在 1.2 m 楼梯中无候选，随后姿态搜索产生约 1.4 亿次体素查询。候选路径存在不等于真实机身可通过。
- 修改先导半径并减少朝向动态规划的重复查询后，相同 22 部件候选请求的原生离线三层对照由 7 秒及 30 秒超时，变为两次 5.49／5.85 秒生成 291 点路线，并独立逐姿态、逐边检查通过。Product 链实测二层／三层预览分别 3.52／5.37 秒通过；二层目标的实际 MuJoCo 任务在 74.1 秒观测中仍在一层附近并反复恢复，随后人工取消，原生速度命令归零。未记录完整物理接触，不能宣称上楼通过（[对照](../../build/stair-multifloor-20261028/guide-probe/README.md)、[执行记录](../../build/stair-multifloor-20261028/README.md)）。
- 该执行失败首先发生在起始原地转向：机身偏离原起点 1.09 cm 后，1 cm 的阶段容差触发短回位样条和反复恢复。现仅对首个平移前的转向复用已有位置与支撑高度容差，在实测位置检查旋转扫掠；中间转角的条件保留。55 项姿态路线回归及同场 MuJoCo 复跑已验证机器人能离开起点，但以 0.4 m/s 请求的复跑约 54 秒后才进入下一阶段、75 秒内未到楼梯，实测峰值约 0.484 m/s，任务仍未通过（[结果](../../build/stair-multifloor-20261028/start-turn-result.md)）。大幅人工挪动后重接路线的连续支撑仍未重新验证。
- v39 低速 yaw 在隔离 MuJoCo 中同样响应偏弱：持续请求 0.2 rad/s 的后 10 秒平均实际角速度约 0.00318 rad/s，0.4 rad/s 请求约 0.235 rad/s。仅 yaw 反馈候选虽然加快首次进入 5°范围，但 45 秒内没有稳定保持在该范围，且边界启停更频繁；该候选未接入生产或当前仿真（[对照](../../build/stair-multifloor-20261028/v39-yaw-findings.md)）。该证据仅适用于此仿真模型，不说明真实机器人存在相同响应。


### 2026-10-30 全局姿态衔接撤除与 MLS 补面核对

- 用户要求撤除全局组合机身、yaw 搜索和转身扫掠准入；实现已从全局候选链移除，SCAN 实时碰撞保留。
- 移除目标激活对中间 yaw 的要求，执行基准同步 XYZ，保留终点朝向。
- 同图同请求同 Release 底层库：1/5 → 2/5；返回房间超时 → 4.96 s 成功。剩余两条搜索超时、最长跨度起点吸附失败。
- 原版 0/5 不是迭代预算耗尽：五次只展开 1/81/149/546/214 个节点；默认支撑/占据/预阻塞规则导致不连通。
- MLS 20 cm/40 cm 新增 217235/417324 个派生表面格；原占据地图未修改，不能视为真实地面新增观测。
- 证据：`build/xyz-global-20261030/README.md`；未部署、未做本轮物理运动验收。
