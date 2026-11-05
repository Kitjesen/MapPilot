# 导航阻塞原因与精简审查

日期：2026-10-28，北京时间。对象为当前工作树的 OctoPlanner3D + SCAN，
包括网页、Gateway、原生导航、驱动及 MuJoCo feeder。本文是源码审查，
不是本轮运动验收，也不表示列出的原因正在同时发生。工作树仍有并行修改。

同一原因的 missing/stale/future 等变体合并列出；配置错误、等待、候选拒绝、
任务终止分别说明。没有把测试工具、未启用后端或不可达旧字符串算成当前运动门槛。

## 判断

确实存在过度适配和接线错误，不应将它们解释成产品必需的安全性。
已确认的例子是整机未知空气拒绝、整身投影逐列证明地面、端点额外假膨胀、
中间阶段固定厘米级到点，以及保留任务时却清掉执行路线。

当前问题还包括输入高度合同不一致、初次搜索与重搜索失败语义不同、
UI 丢失具体原因。不能只继续增加等待门槛或把所有返回 false 都改成 true。
带噪楼板边界曾被错误识别为支撑；这属于错误放行，与过度拒绝同样需要修正。

## 当前会阻止提交、搜索或运动的条件

### 页面与目标入口

| 条件 | 当前行为 | 位置与评价 |
|---|---|---|
| Web 连接断开、没有权威运行状态、状态过期或急停 | 禁止提交运动；Web 状态接收年龄预算为 7 s | [motionTruth.ts](../../web/src/services/motionTruth.ts)、[App.tsx](../../web/src/App.tsx)。属于页面的运行状态判断，不是几何无路 |
| 页面没有有效地图位姿 | 不发送预览，显示等待有效定位 | [SceneView.tsx](../../web/src/components/SceneView.tsx)。此前 sim 时间与墙钟混用已修，不能再把模拟时间小直接判过期 |
| 只读、控制暂停需解除、导航未就绪 | 不提交目标 | 同上。应显示具体的控制状态，不应统称无法生成路线 |
| 预览未完成、目标已改变，或预览起点与当前位置 XY 相差 >0.25 m / Z >0.15 m | 旧预览不用于提交 | [navigationPreview.ts](../../web/src/services/navigationPreview.ts)。这是预览有效性条件，不证明当前目标不可达 |
| 目标 ID、坐标、朝向、速度或到点范围无效；frame 不符合接口 | 拒绝该请求 | [command/goal.cpp](../../src/nav/cpp/endpoint/nav/command/goal.cpp)、[schemas.py](../../src/gateway/schemas.py)。接口输入错误应在边界处理 |
| 当前不是自主模式、巡检拥有任务、人工接管未释放、急停、驱动未就绪 | 拒绝新目标；不等于规划器搜索失败 | [goal/plan.cpp](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp:44)；原因包括 `inspection_run_active`、`goal_not_allowed_in_*`、`operator_takeover_resume_required`、`estop_latched`、driver blocker |
| 请求产生于最近恢复自主之前 | 拒绝旧目标请求 | 同上，`goal_predates_autonomy_resume`。防止恢复后执行旧请求 |
| 外部目标到来时缺少 pose、已有搜索或待启动请求 | `planning_queued`，接受并排队 | [plan.cpp](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp:88)。不能列成任务硬拒绝；内部入口仍可能返回 `odometry_not_ready` / `map_odom_tf_not_ready` / `global_planner_busy` |

### 输入、定位与驱动状态

统一入口是 [input/gate.cpp](../../src/nav/cpp/endpoint/nav/input/gate.cpp:290)，
模式开关在 [config/build.cpp](../../src/nav/cpp/endpoint/nav/app/config/build.cpp:88)。
只返回当前优先级最高的一个原因，不代表其他输入一定正常。

| 条件与原因族 | 当前后果 | 适用范围 |
|---|---|---|
| `input_clock_invalid`；`simulation_clock_missing/stale/regressed` | 暂停运动，不能按错误时间继续积分执行 | sim 时钟由 [health.cpp](../../src/nav/cpp/endpoint/nav/input/health.cpp) 接线 |
| `driver_control_missing/future/stale/not_ready/rejected`；底层 disconnected、motors_disabled、motor_fault、control_not_assured | 零输出、保持；新目标可能在准入时被拒绝 | SCAN 自主、辅助遥控与纯遥控都需要有效驱动 |
| `odom_missing/future/stale`、`odom_velocity_nonfinite/out_of_bounds` | 暂停运动，等待有效位置与速度 | 自主与辅助遥控。速度异常也可能是定位漂移，应查原始估计，不能只抬高上限 |
| `tf_missing/future/stale` | 需要坐标变换却没有可用变换时暂停 | 仅 odom 需要 TF 时；已经在目标坐标系的数据不应再要求一套无用 TF |
| 当前有效健康消息明确报告 `LOST` / catastrophic | 暂停自主执行；等待中的可恢复输入缺口保留任务 | 辅助健康消息缺失、过期和软状态不再单独挡住有效位置输入；本轮物理场景使用 truth，不能归因于 LTLIO |
| `local_collision_missing/future`、`collision_stale` | 暂停避障运动，等待有效局部地图 | check_obstacle 开启的 SCAN；检查的是地图可用性，不是要求机身空气逐格 known-free |
| 输入恢复 | 当前必要数据有效即当拍放行 | 已删除三轮代际恢复计数；现有状态字段固定报告 recovery_frames=1 |
| `cloud_missing/future/stale` | 输入等待 | 原始 cloud gate 是 CMU 条件，不是当前 SCAN 额外必需项 |
| `traversability_missing/future/stale` | 输入等待 | 仅 CMU 且 use_traversability_cost 开启。当前 SCAN 不同时依赖这一门 |

`local_collision_complete` 目前只作遥测，不是停车门槛；`Grid` 不因 complete、
epoch/sequence 尚未积累足够历史而额外拒绝。缺地图、地图边界和真正 occupied 仍分别处理。
纯 Teleop 不要求 odom/cloud/traversability/定位健康；辅助避障遥控与它不同。
相机画面、录制器、原始 IMU 可视化和电机关节曲线不是当前 SCAN 自主导航的额外准入项。

### 全局规划与有向路线

| 条件与原因族 | 当前后果 | 源码与判断 |
|---|---|---|
| `active_octomap_not_configured`、`map_not_ready`、地图读取失败 | 无法搜索 | [octoplanner3d_core.cpp](../../src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp)、[loop.cpp](../../src/nav/cpp/endpoint/nav/app/loop.cpp)。应报告地图服务或文件问题 |
| 活动地图身份缺失、切换或规划期间变化；frame/goal 已变 | 等待地图，或废弃已失效的搜索结果 | [goal/task.cpp](../../src/nav/cpp/endpoint/nav/app/goal/task.cpp)、[goal/plan.cpp](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp)。地图短暂不可读与真正切图要区分 |
| `start_outside_static_map`、`goal_outside_static_map` | 端点不在保存地图范围 | [core.cpp](../../src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp:425)。先看坐标、目标高度和地图覆盖 |
| `start_snap_exhausted`、`goal_snap_exhausted` | 邻近范围找不到合适端点 | 同上。不能据此断言现实中没有通道 |
| `body_pose_input_invalid`、`invalid_pose_request` | 请求的机体或位姿无效 | [heading.cpp](../../src/nav/cpp/planning/global/octoplanner/heading.cpp:102)。属于集成输入错误 |
| `start_pose_blocked` | 起点支撑或边界不可用 | [heading.cpp](../../src/nav/cpp/planning/global/octoplanner/heading.cpp:133)。已删除机身起点单独否决；保存地图中起点占据时可连接同朝向的邻近空闲姿态，第一段交给实时 SCAN 验证。后续碰撞与支撑检查保留，不能把 start_pose_blocked 一律翻译成机身撞墙 |
| `support_disconnected` | 独立支撑连通预搜索耗尽 | [heading.cpp](../../src/nav/cpp/planning/global/octoplanner/heading.cpp:235)。可能是保存地面缺失、错误高度或连通建模，不能翻译成 OBB 碰撞 |
| 邻接移动的真实占据、支撑缺失、步高/坡度超限、转身/平移机身扫掠冲突 | 丢弃该条边，继续搜索；全部失败才无路 | [global_planner.cpp](../../src/nav/cpp/planning/global/octoplanner/vendor/planner/src/global_planner.cpp)、[heading.cpp](../../src/nav/cpp/planning/global/octoplanner/heading.cpp)。单条边失败不是任务失败 |
| `search_iteration_limit`、`global_planning_timeout`、`preview_timeout` | 本次搜索没有完成 | [task.hpp](../../src/nav/cpp/endpoint/nav/app/goal/task.hpp)、[core.cpp](../../src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp)。预览单独采用 7 s 截止，不应报成确定无路 |
| `navigation_busy`、`planner_busy` | 此次预览未启动 | [loop.cpp](../../src/nav/cpp/endpoint/nav/app/loop.cpp:1211)。与运行中导航任务终止无关 |
| `empty_path`、`goal_not_reached`、`body_pose_path_invalid`、`global_planner_exception` | 本次结果不可执行 | [plan.cpp](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp:453)。空结果、未到目标与算法异常应分开 |
| 平移和朝向耦合边不满足 forward tangent heading；路径字段无效 | pose route 编译失败 | [pose_route.cpp](../../src/nav/cpp/navigation/pose_route.cpp)。这是适配合同，不是原版 SCAN 的要求；可选 blend 不成功会回退停转阶段，不能把 blend 未采用算成无路 |
| temporary overlay 元数据、几何或展开预算无效 | 拒绝带无效 overlay 的请求 | [core.cpp](../../src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp:69)。仅实际携带临时叠层时适用 |

当前 global BodyQuery 只给出 `occupied_body` / `temporary_overlay`，没有因未知空气
返回 `unknown_body` 的分支。`start_body_unknown` 仍有旧映射字符串，不能把它当作
已证实可达的当前门槛。全局无 ground_support 的另一种查询模式仍有中心 unknown 政策，
不等于当前 observed-floor 模式要求整个机身 known-free。

当前支撑补洞要求邻近观测围住实际 XY，修正的是原先单侧观测外推地面的错误。
尚无固定平地反例证明应删除它；带噪实图仍会把侧边孤立 occupied 认成支撑，未验收。
20 cm 台阶 smoke 只证明直接边查询；该文件后续的完整 A* 仍期望 Exhausted，
不能用它声称完整楼梯规划通过。

### 局部规划、跟踪与楼梯阶段

| 条件与原因族 | 当前后果 | 来源与判断 |
|---|---|---|
| `no_global_path`、`pose_route_frame_changed`、`invalid_map_from_odom` | 无参考或参考坐标已失效，零输出 | [executor.cpp](../../src/nav/cpp/navigation/executor.cpp:473)。坐标换代应废弃旧参考，短暂掉数据不应无条件清目标 |
| `scan_execution_clock_discontinuity` | 执行时间倒退或跳跃过大，挂起并重建轨迹 | [executor.cpp](../../src/nav/cpp/navigation/executor.cpp:494)。时钟接线条件，不是机身形状造成 |
| `collision_map_invalid`、分辨率不一致、`body_raw_evidence_missing`、无效 Body/yaw | 局部返回 InvalidInput | [grid.cpp](../../src/nav/cpp/planning/local/scan/grid.cpp:46)。组合模型不能把已经膨胀的格再次当原始实测障碍 |
| 实测 occupied 与机身/扫掠相交，或扩展机身越出局部 AABB | 候选碰撞返回 1，数据范围不足返回 -1 | [grid.cpp](../../src/nav/cpp/planning/local/scan/grid.cpp:450)。窗口内未知空气跳过；窗口外缺数据与窗口内未知不能混为一谈 |
| `scan_local_target_blocked`、`scan_target_rejected`、`scan_initialization_failed`、`scan_replan_failed`、首次 Pending | 没有可执行轨迹时零输出；仍安全的旧尾段可以继续 | [backend.cpp](../../src/nav/cpp/planning/local/scan/backend.cpp:520)。候选失败机制主要继承 SCAN；不是每次重规划均停车 |
| `scan_emergency_stop`、`scan_speed_transition_blocked` | 近期预测会碰撞，或换轨/降速过渡不可行 | [backend.cpp](../../src/nav/cpp/planning/local/scan/backend.cpp:425)、[planner_fsm.cpp](../../src/nav/cpp/planning/local/scan/upstream/plan_manage/planner_fsm.cpp:309)。官方有未来碰撞急停，交接与过渡由适配扩展 |
| heading 对齐、外部 hold、速度目标为 0、safetyStop、无效样条/跟踪输入 | 可能只停止 XY 保留 yaw，也可能全零 | [closed_loop_controller.cpp](../../src/nav/cpp/planning/local/scan/upstream/plan_manage/closed_loop_controller.cpp:62)、[follower.cpp](../../src/nav/cpp/tracking/follower.cpp:389)。大朝向误差先转后走是官方已有行为 |
| `settling_before_goal_yaw`、`settling_before_pose_turn`、`aligning_goal_yaw`、`pose_turn_blocked`、`settling_at_goal` | 姿态阶段交接或终点确认期间暂停平移/等待低速 | [executor.cpp](../../src/nav/cpp/navigation/executor.cpp:684)。普通内部路点不应套最终到点精度。最终到点确认包含位置、高度、必要朝向和一次有效低速观测，不再累计 0.5 s |
| 受阻/无进展触发 Recovery；候选为空、不安全、离轨或无进展 | 停止当前动作，使用当前几何继续检查下一候选 | [recovery.cpp](../../src/nav/cpp/navigation/recovery.cpp)、[executor.cpp](../../src/nav/cpp/navigation/executor.cpp)。已删除额外观测版本等待；当前实测障碍仍能拒绝所有候选 |
| `local_recovery_exhausted`；关闭 recovery 后的 `execution_stalled` | 前者交回全局重规划；后者在该配置下保持零直到新路线/重置 | [executor.cpp](../../src/nav/cpp/navigation/executor.cpp:1079)、[autonomy.cpp](../../src/nav/cpp/endpoint/nav/control/autonomy.cpp:152)。不能因 outcome 仍叫 kGoalFailed 就忽略附带的 replan trigger |
| `stair_landing_space`、`stair_entry_required`、`stair_switching`、`stair_profile_unsupported/wait`、`stair_hold_on_flight`、`stair_settling/restoring/route_wait` | 楼梯入口、平台模式确认、梯中保持或出口恢复期间 hold | [stairs.cpp](../../src/nav/cpp/navigation/stairs.cpp:255)。仅有效楼梯段激活；`stair_approach` 本身不是 hold。落脚平台矩形包络及过渡位置/角度/静稳条件的组合应单独审查，尚不能宣称已证实多余 |
| `teleop_intent_idle`、`teleop_intent_direction_changed`、`local_intent_pending`、`teleop_assist_local_stop` | 遥控无输入、换方向或等新局部参考时输出零 | [executor.cpp](../../src/nav/cpp/navigation/executor.cpp:1340)。不意味着键盘消息一定没收到 |

终点等待调整（2026-11-01）：此前执行器要求在目标范围内连续低速 0.5 s，成功终态随后还要等待驱动接受零命令及 8 条低速里程计；现在执行器在有效速度观测低于既有阈值时即可报告几何到达，成功终态等待匹配的驱动零命令回执，不再额外收集 8 条样本。暂停、取消、急停及重规划仍沿用各自原有停止确认。这个调整减少到达反馈延迟，但一次低速观测不能保证之后不会回弹，也不代表已完成连续物理静稳验证。

对照固定上游：[DimOS Basic `26144f7`](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/trajectory_follower/basic/module.py#L148) 在进入水平范围后发零并报告到达；[SCAN `348e8a5` 控制器](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/closed_loop_controller.cpp#L210) 在轨迹完成且水平误差达标后输出零。两者在上述分支均没有本项目原来的 0.5 s 窗口；LingTu 保留的高度、可选最终朝向、即时低速和驱动回执是本地合同，不是上游默认。

同场景的本地组件对照：修改前保留的 Windows 构建中 `Executor.ArrivalRequiresFreshQuietOdometryAndResetsForNewRoute` 通过，测试要求首次有效低速样本不立即到达；修改后 `Executor.ArrivalRequiresValidQuietMotionWithoutAccumulatingSamples` 通过，测试要求首次有效低速样本即可到达，并继续拒绝无效速度、过快转动及过快竖直运动。修改后记录见 [arrival-final.log](../../build/arrival-no-dwell-20261101-executor/arrival-final.log)。两者是不同时间构建的组件测试，不是同场景 Go2 前后实测。

仿真取证边界：启用 `stop_on_nav_goal_reached` 时，组件 feeder 在原生到达和停止事务确认后即结束仿真；成功到达使用仅回执确认后，不能把这一提前结束当成连续物理静稳的证据。若要验收到点后的物理漂移，应延续仿真并独立记录机身运动。

### 任务、遥控和最终输出

| 条件与原因族 | 当前后果 | 源码与判断 |
|---|---|---|
| 初次全局规划普通无路、搜索超时或暂时几何不可用 | 保留已接受的目标，仍为 Planning，按已有 0.5 s 退避重试 | [plan.cpp](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp)。使用同一 task/request/epoch 和新位置；明确取消、地图替换、无效请求或算法异常仍单独终止；不把预览变成运动任务 |
| 运行中局部恢复耗尽，再次全局普通无路 | `PathActive + replan_waiting`，保留目标并安排后续重试 | [retry.cpp](../../src/nav/cpp/endpoint/nav/app/goal/retry.cpp:151)、[runtime.cpp](../../src/nav/cpp/endpoint/nav/app/goal/runtime.cpp)。旧 `replan_budget_consumed` 不再是当前普通重试终止路径 |
| `stop_pending`、`backoff_pending`、`replan_in_flight`、`input_gate_hold_pending`、`driver_control_hold_pending` | 停留/等待，不启动本轮搜索或运动 | 同上。不能按名字中的 hold/pending 算成已经失败 |
| 明确暂停、取消、软件急停、任务被接受的新目标替换 | 按操作者意图暂停或结束旧任务 | [safety/stop.cpp](../../src/nav/cpp/endpoint/nav/safety/stop.cpp)、[goal/runtime.cpp](../../src/nav/cpp/endpoint/nav/app/goal/runtime.cpp)。被拒绝的新请求不应破坏原任务重试，已修相关副作用 |
| 遥控来源没有持有权、被另一来源持有、租约过期、epoch/sequence 不匹配、zero barrier 尚未完成 | 不接纳该输入，必要时保持零命令 | [operator.cpp](../../src/nav/cpp/endpoint/nav/control/operator.cpp)。原因包括 `no_active_authority`、`authority_busy`、`authority_lease_expired`、`zero_barrier_pending` |
| 遥控样本无效/来自未来/过旧/deadline 过期；松开 deadman | 停止沿旧按键指令运动 | 同上及 [admission.cpp](../../src/nav/cpp/endpoint/nav/control/admission.cpp)。默认样本 0.35 s、最大租期 2 s，不代表每 2 s 需人工重新接管 |
| 遥控接管禁用、恢复前的旧样本 | 拒绝该次样本 | `teleop_takeover_disabled`、`teleop_source_predates_resume` 等，源于模式与旧输入处理 |
| 持续控制环 deadline miss | 临时保持零命令，调度恢复后自动释放自身保持，保留路线 | [guard.cpp](../../src/nav/cpp/endpoint/nav/control/guard.cpp)、[loop.cpp](../../src/nav/cpp/endpoint/nav/app/loop.cpp)。不再建立人工接管锁；P95 与历史 miss-ratio 警告本身不建锁 |
| 速度平滑器无效/超时或 commit 失败 | 最终输出零 | [final.cpp](../../src/nav/cpp/endpoint/nav/control/final.cpp)。属于运行故障，不应报几何无路 |
| 命令过期；普通遥控平移小于最小运动量 | 零或限幅；自主跟踪在此将 min_motion 设为 0 | [command.cpp](../../src/nav/cpp/endpoint/nav/safety/command.cpp)、[autonomy.cpp](../../src/nav/cpp/endpoint/nav/control/autonomy.cpp)。限速/限加速度本身通常只改变输出，不是拒绝规划 |
| 实际运动或待发送命令的制动扫掠阻塞 | `scan_actual_motion_blocked`；可缩放时 `scan_actual_motion_limited` | [main.cpp](../../src/nav/cpp/endpoint/nav/main.cpp:1245)。两者输入不同；有轨迹仍可能在这里停止，应追到同一份碰撞证据 |
| 零输出发送失败、停车确认尚未完成、驱动拒绝零命令、急停状态无法保存/清除 | 暂不宣布停车或恢复完成 | [stop.cpp](../../src/nav/cpp/endpoint/nav/safety/stop.cpp:307)。不能以请求已受理代替实际确认 |
| 驱动命令 frame 无效、速度非有限、命令来源/序号/时效无效；watchdog 超时 | 驱动拒绝或停车 | [core.cpp](../../src/drivers/real/motion/core.cpp)、[command_freshness_gate.cpp](../../src/drivers/real/motion/command_freshness_gate.cpp)。驱动不负责重新判断全局路线 |

### 仿真退出与服务故障

- MuJoCo feeder 的姿态/离地高度越界会结束当前试验并停服务：roll/pitch >30°，
  clearance 不在 0.20～1.00 m，或其变化超出证据门槛；这些是相对物理支撑的高度，
  不是世界 Z，不能因上楼世界高度增加就判越界。
  [formal_feeder.py](../../sim/scripts/mujoco/formal_feeder.py:914)、
  [evidence.py](../../sim/scripts/mujoco/evidence.py:18)。
- 已发生过楼板边缘前倾后 feeder 退出、导航状态消失；后续页面无法规划属于服务已失效。
  这不证明应删除支撑/碰撞检查，也不能用进程退出掩盖此前为什么走到边缘。
- 模型或原生库缺失、导入接线错误、DDS/地图/驱动进程退出属于启动或运行故障；
  应修复具体失败，不能给用户显示成目标不可达。

## 已删除或修正的过重行为

以 [error_record](../operations/error_record.md) 的证据与部署边界为准：

1. 整个 OBB / 组合机身每格必须 known-free；未知空气也阻止起步。
2. 由 supportHeight 引入的躯干/腿部投影逐列证明地面，以及新增逐脚接地准入。
3. 真实起终点也增加半段扫掠 padding，造成离障方向仍不能起步。
4. 等价重规划候选无收益也替换，旧尾段有效时重复进入优化。
5. Body + checkObstacle=false 在普通查询与恢复中的语义不一致。
6. 新目标准入失败，提前取消旧目标的重规划机会。
7. 驱动短暂失去控制权时，任务保留但执行路线被清空。
8. 把仿真累计调度落后算成每一帧雷达的真实排队年龄，持续丢弃新帧。
9. Web 以墙钟直接评估模拟源时间，误显示等待有效定位。
10. 中间姿态阶段固定 1 cm XY / 2 cm 实测 Z 阻止交接；当前复用到点/支撑容差，
    验证当前位置转身及接回扫掠后交接，完整跨层验收仍未通过。
11. 输入有效后还要求多个源同步更新三轮；现在当拍判断当前数据，不维护第二个恢复状态机。
12. 脱困每次动作结束或搜索失败后额外等待地图版本变化；现在用当前有效几何重新搜索。
13. 调度暂停误用清路线和人工接管语义；现在只保留自己的临时运动保持。
14. 只指定位置的目标被末段路径朝向补成强制终点 yaw；现在仅显式指定 yaw 才做终点对齐。
15. `/go x y` 省略机身高度而落到 z=0；现在预览与提交共用当前有效机身高度。
16. Web 将不同失败统一折叠为“导航条件未满足”；现在保留阶段原因，未映射代码也可见。
17. 首次全局搜索普通无路立即 Failed，而重规划同类失败保留目标；现在复用现有 pending
    请求保存与退避重试，不另建调度器。等待中的驱动瞬断也不再取消该目标。

这些修正不等于已全部部署实机，也不等于 22 部件隔离候选已经成为所有机器人的正式模型。

## 仍应优先修正的具体问题

1. **继续修支撑证据与执行响应。** 孤立侧边噪声不应证明楼板；v39 小角速度响应差也不应
   靠更精确的中间到点要求弥补。两者都有现存物理失败证据，不能靠删所有检查宣布成功。
2. **审查内部姿态/楼梯交接的严格度。** 保留真正需要的平台切换确认，减少普通路径点的
   停转交接。楼梯 landing/entry 的多个条件同时满足是否过严需要对应场景证据，
   不将它提前记作已确认错误，也不新增另一组开关。

## 上游对照

固定原版 SCAN 是局部规划研究实现，本来包含初始化/重规划/紧停状态、碰撞候选排除、
大朝向误差时先转后走；不含 LingTu 的 Product、地图身份、任务票据、遥控租约或 pose stages。
比较见 [上游审查](scan-upstream-comparison-20261026.md)。

Nav2 官方也有传感器超时停车和独立碰撞监测；默认带恢复的导航会在局部失败后进行
恢复与重试，包括等待、转向、后退，而不是把每次候选失败直接作为任务终态。
借鉴的是错误归属和恢复语义，不引入它的 ROS/二维地图或另一套行为树：
[导航恢复与重试](https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/trees/nav_through_poses_recovery/)、
[碰撞监测](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)。

本页随 10-28 本地精简继续更新。具体回归和物理试验见 error_record；没有部署 Go2，
不能把本地规则修正当作新的实机通过证据。
