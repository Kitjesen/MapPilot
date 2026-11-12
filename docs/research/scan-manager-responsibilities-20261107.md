# SCAN 协同、manager 职责与 Executor 失败说明

2026-11-07。本轮负责 manager 整理与失败解释。用户确认另一个会话同时修改到达／跟踪逻辑；该部分修改保留，下面的 Executor 数字对应本轮开始时重建的快照，不代表另一个会话完成后的最终状态。

## 执行进度如何协同

Follower 拥有“这条轨迹实际执行到第几秒”。规划器通过 `LocalPlanRequest::execution` 取得轨迹 ID、采样时间、执行时间，Task 在原有锁内传递同一份反馈。Backend 先核对实际执行 ID，FSM 再核对当前曲线 ID；不匹配的旧样本不能修改新轨迹。

例如某条曲线在物理时间 100.00 s 的已执行进度为 2.70 s。规划线程在 100.01 s 读取它：未暂停时外推至 2.71 s；暂停时保持 2.70 s。两者比较的是同一物理时刻的进度，不能把 100.00 s 的旧样本与 100.01 s 的新样本直接当作不同步。

FSM 按未来交接时刻从旧曲线取位置、速度、加速度，manager 求解对应的新曲线。Task 到交接时刻接受，Follower 从新曲线在接收时的实际年龄开始；超期候选沿用既有拒绝规则。轨迹执行进度只有一个来源，异步计算和未来交接仍各自保留职责。

| 代码 | 负责的事 |
| --- | --- |
| `tracking/follower.cpp` | 跟踪已接受曲线，记录实际执行进度与采样时间。 |
| `scan/task.cpp` | 在线程间传递快照，接收／撤回结果。 |
| `plan_manage/planner_fsm.cpp` | 选择何时重规划，安排未来交接。 |
| `plan_manage/planner_manager.cpp` | 按 FSM 给定的边界完成一次曲线求解；不拥有跟踪器的实际执行进度。 |

上游固定版本 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` 通过冻结布尔值累计规划器时间。LingTu 的异步未来交接需要实际进度回传；该回传是本地适配，不能称作上游已有的完整保证。

## manager 已做的整理

| 文件 | 职责 | 行数 |
| --- | --- | ---: |
| `planner_manager.cpp` | 组织初始化、优化、时间调整和结果提交；保留原有规划状态、参考拟合和减速曲线入口。 | 1021 → 525 |
| `seed.cpp` | 初始速度曲线、参考路段采样、参考时间分配和 Z 初始化。 | 273 |
| `retiming.cpp` | 固定首尾位置／速度／加速度后的时间调整、优化、动态约束与碰撞检查。 | 255 |
| `seed.h` | 无状态初始化计算的内部接口。 | 36 |

`retiming.cpp` 继续实现同一个 manager 的私有方法，没有第二个管理器或重复状态。`reboundReplan()` 现在调用一个完整的 `retime()` 操作；失败证据保持原样，连续失败计数仍由主流程更新一次。删除 `reparamBspline()` 无调用者使用的时间增量输出及两个接收变量。`setMotionIntentBoundary` 和 `motionIntentTimeRatio` 改名为 `setSplineBoundary` 和 `dynamicTimeRatio`，与它们同时服务普通路线的实际用途一致。

这是职责分离，不能说整个实现代码减半。上游固定版本的 manager 为 555 行，LingTu 的精确物理边界和三维参考适配仍保留在相应文件。这里的 `planGlobalTraj*` 用于 SCAN 内部参考轨迹拟合，不是外层 OctoPlanner3D 全局搜路。

构建入口 `NavCoreTargets.cmake` 已登记两个新源文件；公开 API、状态所有者、阈值、16 次时间搜索预算、优化目标和三维碰撞语义不变。拆分后完整 SCAN 160/160 通过，保留前一轮真实曲线及边界回归；本轮未新写只检查代码结构的测试。

## 为什么上次是 104/129

上次归档的 25 个失败在本轮前版本也全部失败，证明当时不是执行进度补丁新增。它们并非 25 个独立缺陷：到达类测试中的反馈链不完整、职责变化、首帧时间假设和真实响应未达标混在一起。

本轮按当前已有源码重建，完整运行得到 **120/129，9 项失败**。旧的 25 项中 17 项通过，8 项仍失败；另有终点运动转向用例失败。这些变化包含另一个会话的工作，不能算作本轮 manager 拆分修复了 16 个用例。Executor 使用拆分前已固定的可执行文件运行，manager 拆分另由 160 项 SCAN 验证。

| 本轮快照的失败 | 已查明的失败原因／证据 | 处理归属 |
| --- | --- | --- |
| `ScanArrivalDoesNotCutOffTheMovingTerminalSpline` | 带终点朝向时，仍在移动的末段进入 `aligning_goal_yaw`，tracking 失活，线速度直接变零；断言要求保持末段并按加速度减速。 | 到达／跟踪会话应核对转向接管与减速，不能只改断言。 |
| `ScanHandlesFreshDynamicPrediction` | 测试要求 SCAN 直接生成绕预测障碍的弯线；当前候选查询明确不使用预测障碍，且初次接收的静止曲线首拍可以为零速度。 | 动态避障产品需求与当前分工需对齐；在 FinalControl 验证制动，并明确是否要求主动绕行。 |
| `OdomPredictionTransformKeepsUnavoidableDynamicCollisionStoppedByScan` | 同样在候选层期待预测障碍否决，当前结果为 `spline_control_ready`。 | 预测障碍变换及阻止最终运动应由最终运动检查测试覆盖；候选 ready 不证明可运动。 |
| `ConfiguredBodyGuardsBothXYZRouteAndMotionIntentWithRawGeometry` | 测试用空膨胀图加 raw Body 障碍，期待候选失败；Backend 当前明确清除候选请求的 Body/observedBody，使用双圆柱与三维膨胀占据。 | 在负责 raw Body 的最终检查层验证同一个障碍，不能用该失败证明已发送碰撞命令。 |
| `ConfiguredBodyXYZGuideCannotTurnThroughAnOccupiedCorner` | 用 raw Body 转动扫掠复验普通候选；当前候选使用双圆柱语义，因此第 6 段的两种检查结论不同。 | 核对最终转动／运动检查与产品对整条候选的要求，保留三维障碍证据。 |
| `ScanIgnoresTwoDimensionalTraversabilityInput` | 只在新曲线首拍断言速度大于零；静止边界首拍执行时间为零，不能由这条断言判定二维图影响了规划。 | 测试应推进执行时间，再验证正常运动和输入隔离。 |
| `MapPlanningDoesNotConsumeOdomTransform` | 同样在首拍要求非零前进速度；其余路径／坐标断言没有在此次失败列表中。 | 分开验证坐标结果与时间推进后的运动。 |
| `StairRejectedLocalRouteReplansAfterMeasuredVelocitySettles` | 失败在前置断言：测试构造的轨迹实际被楼梯范围允许，没有产生它要测试的“先拒绝后恢复”条件。 | 修复场景前置条件后再评估恢复逻辑，不能据此断言恢复失败。 |
| `ScanProductMotionLimitsKeepSustainedIntentAcrossReplans` | 前向通过；斜向到 90% 要 3.49 s，要求 ≤3 s；3 s 后速度比例为 0.7846～0.9229，要求 ≥0.95。11 次交接、无冻结帧、无失活帧。 | 存在响应未达验收目标；需要分析平滑加速和滚动重规划速度形状，未以改阈值消除。 |

本轮没有修改这些 Executor 断言、到达行为或最终碰撞政策。尤其“最终控制负责”不等于“已经验证最终控制安全”：上述同场景最终控制验证不在本轮证据内。

## 速度配置澄清

`config/system/settings/limits.yaml` 当前每个机身水平轴均为 **1 m/s**，水平速度向量上限为 **√2 m/s**，用于允许两轴各 1 m/s 的斜向请求。前一轮 0.35 m/s 是单独构造的不等轴测试约束，不是当前产品参数；本轮没有改回该数值。

## 证据与后续

[构建／测试快照](../../artifacts/nav/scan-manager-review-20261107/README.md)保留 manager 拆分前源文件、重构脚本、构建日志、完整 SCAN 结果和 Executor 运行快照。先前车库交接证据见[边界复查](../../artifacts/nav/tora-boundaries-20261107/README.md)。

- [x] manager 按初始化、时间与约束、流程职责拆分；构建登记与内部文档同步。
- [x] 160 项 SCAN 验证，无新增算法参数或限制。
- [x] 重跑并解释一个完整 Executor 快照，区分旧结果和另会话后续修改。
- [ ] 到达／跟踪会话完成后，验收最终停止与转向接管。
- [ ] 按当前负责模块补齐动态预测、raw Body 的最终运动检查场景。
- [ ] 单独定位斜向持续速度未达目标与楼梯测试前置条件。

未做新的物理仿真或实机测试；代码组织改善不等于新增实机资格。
