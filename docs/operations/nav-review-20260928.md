# 导航职责对照与接口修复（2026-09-28）

本轮基线为 `37e0d413`，修复范围是原生输入原因优先级、Agent 状态读取和目标高度。
没有改变地图证据、全局几何、SCAN 优化方程或运动限值，没有安装实机版本。

## 与参考代码的对照

| 对照项 | 参考实现 | LingTu 的处理 |
| --- | --- | --- |
| 全局路线 | [jie `planAndPublish` / `findNearestFreeCell`](https://github.com/6-robot/jie_3d_nav/blob/995a3a544ce6e7bec6f205569339d5125843d727/octo_planner/src/jie_path_node.cpp) 从吸附候选搜索并发布路径 | 保持全局路线与局部执行分开；此前删除的 raw→snap 连接检查不恢复 |
| 支撑与占据 | [OctoPlanner3D `isCellTraversable` / `hasGroundSupport`](https://github.com/JackJu-HIT/OctoPlanner3D/blob/9a9cc431ea905a5878975cc6fbbce6c9618b31a4/planner/src/global_planner.cpp) 也检查支撑和半径范围内占据 | 不能把全部几何检查说成自研冗余；保留路线所需几何，局部不重复七点支撑与 free-ray 门槛 |
| 局部轨迹 | [SCAN `pathCallback` / `planFromCurrentTraj`](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp) 接收参考路径，生成和重规划局部轨迹 | SCAN 管轨迹与局部碰撞；InputGate 管跨进程观测是否可用。局部暂缺不等于全局无路 |
| 目标高度 | 同一 SCAN 文件的手动目标使用接收到的机身高度；参考路径按其地面路径约定加 body height | Agent 显式 Z 原样传递；省略时使用当前地图机身高度。LingTu 路线已有自身高度约定，不照抄再加一次 body height |
| 未知空间 | jie / OctoPlanner 查询没有节点时并不一律拒绝 | 与本项目 saved-ray 地图约定有意不同。本轮保持现有 unknown 策略，不声称与参考项目完全等价 |

结论：需要去掉重复职责，不需要把参考项目所有参数、地图转换和 ROS 包装一并移植。
Product 会话、控制权及状态流属于本项目的进程接口；它们的存在本身不能证明算法过度防御。

## 确认并修复的问题

1. **局部碰撞等待掩盖定位故障。** 两者同时异常时，首条原因原先是
   `collision_stale` 等可保留全局请求的局部等待。将该组已有判断移到定位、
   traversability 判断之后；不新增检查、不在 Gateway 复制定位规则。
2. **Agent 状态不失效。** 状态、进度、是否执行统一读取已有缓存，按本地单调时钟
   的接收时间使用 2 秒窗口。断流后返回 `UNKNOWN`；新消息恢复显示。
   历史任务结果仍可查询，不添加定时线程。
3. **省略 Z 固定发地图零高度。** 接入已有 odometry 和 map_odom_tf 流，复用
   runtime 的变换解析与旋转。支持非零高度和倾斜坐标变换；map 位姿不重复转换。
   缺少新鲜位姿/变换时要求调用者明确 Z，而不是猜测地面。
4. **部署契约测试引用旧代码。** 更新为当前 MotionWorker 的统计生产与接收、
   观测时间快照、状态发布接口；没有把已删除的同步处理塞回控制循环。
   编排测试另外修正两处旧预期：默认 map 解析成 standard，以及 MuJoCo 不支持
   nav.camera。原有 camera 拒绝测试保留，生产支持范围不变。

## 验证边界

测试覆盖四类局部碰撞等待与定位丢失、缺失、过期、未来时间、不健康和严重故障的组合；
健康定位下的局部等待仍可提交全局目标，实际运动仍保持等待。
Agent 测试覆盖断流与恢复、终态历史、非零高度、显式零高度、坐标倾斜变换、
变换失效和过期，以及真实 Product 的流接线。

本地测试及构建记录位于 `build/review-full-20260928/fixes-*.txt`。
专项 Python 回归 106 项通过、3 项 Bash 平台测试跳过。边界与编排首轮 172 项
通过，两个旧预期失败；修正后相关 13 项定向回归通过。InputGate、GoalPlanController、
GoalReplanRuntimeCoordinator 三项原生测试通过，MSVC Release navd 构建成功，
修改文件的 Ruff 与 diff 空白检查通过。
这些记录不代表 MuJoCo 或 Go2/NX/S100P 实机验收；本轮没有下发运动命令。


## 后续修复：任务恢复统一协调

基线 `eab028d5`。原先有两条竞争链路：Executor 等待并尝试局部脱困；
Endpoint 的持续障碍检测却可在 0.6 秒、两次新观测后直接跳过 Executor，
触发全局换路。局部等待默认 2 秒，因此正常恢复可能根本没有机会执行。

现在的流程为：

```text
ProductControl 启动导航会话
  → 任务接收目标 → 全局生成参考路径 → SCAN 生成轨迹并执行
  → 受阻时等待 / 局部恢复
       ├─ 恢复成功：继续当前路径
       └─ 恢复耗尽：任务协调器确认停止 → 现有退避 → 最多一次全局换路
  → 到达 / 失败 / 取消由同一任务流程结束
```

删除控制拍里的预计算换路入口。持续障碍检测归入现有
`GoalReplanRuntimeCoordinator`，只提供证据，不能抢断执行。
任务协调器只接收局部恢复耗尽作为换路请求；恢复期间新观测更新候选障碍，
局部重新找到可行路径或障碍清除时丢弃候选。换路开始后仍冻结该次请求的
实测体素，保持既有停止确认、目标身份、取消和替换路径提交机制。
没有新增运行开关、调度框架、依赖或放宽几何/运动约束。

参考 [Nav2 的规划、跟踪与恢复行为树](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)：
借鉴上层任务流程协调独立能力的职责分工。Nav2 默认树还含周期重规划，
本项目本轮不照搬该策略，采用局部恢复耗尽后再决定换路的明确顺序。
全局与局部路径接口继续沿用前文 jie / SCAN 对照结论。

定向验证包括：持续障碍不抢断执行、恢复耗尽才换路、恢复期间障碍位置更新、
恢复成功后不沿用旧障碍、停止确认超时禁止换路、原有取消/接管/终态交付。
本地记录为 `build/codex-scan-dynamic/task-recovery-*.log`；这不代表仿真或实机验收。


本轮本地结果：6 个原生测试程序通过（AutonomyTick、恢复协调器、完整换路链路、
持续障碍策略、NavigationRuntime、终态事务）；接口/部署契约 92 项通过、
3 项 Bash 平台测试跳过；MSVC Release `navd` 构建通过。构建仍报告原有
`stop.cpp` 的 `fopen` 弃用提示和 `loop.cpp` 命令分派的局部变量遮蔽提示，
没有本轮新增诊断。未运行 MuJoCo、未构建 ARM 整包、未部署或实机运动。


## 再审查：观测重置与任务缓存

基线 `29aeb698`。复查范围为目标提交、局部恢复、持续障碍、换路事务、
新目标/取消/接管，以及停止确认与终态交付；不是全仓库或实机验收。

发现并复现一项 P2 问题：观测累计重置时，任务协调器的第二份障碍候选缓存
可能没有同步清空。典型顺序是普通障碍云已形成候选，随后 SCAN 提供新的
局部碰撞观测；检测器重置累计并计入第一帧，但协调器仅检查“累计为零”，
因而保留旧候选。此时恢复耗尽会让替换路径请求误用旧障碍。

修复将候选记录移回 `ActivePathBlockagePolicy`，与 bind/reset/clearAccumulation
一起清理，删除协调器内的重复缓存与推测式清理。协调器只在恢复耗尽后读取
候选。重复观测仍不累加、缺少新观测不伪造“障碍已清除”，现有运动检查不变。

新增 `testCollisionObservationResetDiscardsOldObstruction` 在修改前明确失败：
`observation reset reused the old obstacle overlay before new evidence matured`；
修复后通过。原有换路、取消、新目标替换、输入等待、终态交付的测试继续验证。
日志位于 `build/codex-scan-dynamic/recovery-review-*.log`。

最终验证：换路链路、持续障碍策略、任务恢复协调器、NavigationRuntime、终态事务
共 5 个原生测试程序全部通过，MSVC Release `navd` 构建通过。
本轮未修改 Python/网页接口，未重复其无关测试，也未运行 MuJoCo 或实机。
上述审查范围内未再发现新的流程冲突，不能据此宣称整仓库没有缺陷。
