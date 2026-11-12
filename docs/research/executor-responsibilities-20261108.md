# Executor 当前功能与职责清单

2026-11-08 源码阅读快照。入口为 `src/nav/cpp/navigation/executor.cpp`，本次读取约 1516 行。另一个会话在修改到达／跟踪逻辑，因此行号仅供本次定位。本次不修改执行行为，不将代码存在等同于正式接入或实机验收。

后续同日按职责拆分了实现，并将命令反馈方法缩短为 `commitApplied()`。当前文件分工以 `src/nav/cpp/navigation/README.md` 为准；以下功能清单保留，原阅读快照行数不再代表当前主文件大小。

## 它实际接收和返回什么

Executor 执行已经交给它的路线或辅助运动意图。它不是高层语义任务管理器，不做全局搜路，不管理 DDS 进程，不直接发布到底盘。

- `setRoute(Route)`：保存路径、终点朝向、位置／朝向容差、速度、可选姿态阶段、机身模型及楼梯段。
- `tick(ExecutionInput)`：接收位置、速度、地图、当前时间及模式；模式只有 `Route` 和 `MotionIntent`。
- `ExecutionOutput`：返回候选速度、路径预览、局部目标、到达／等待／失败原因、跟踪／恢复／楼梯状态。命令仍需经过端点 FinalControl。
- `commitApplied()`：将最终真正接受的命令反馈给跟踪器，区别于仅计算出来的候选速度。

## 普通路线每拍怎么走

```text
tick()
  检查姿态路线的坐标版本，更新楼梯阶段
  ├─ MotionIntent → tickIntent()
  └─ Route → tickRoute()
                统一 map / odom 坐标
                → tickInPlanningFrame()
                    处理姿态阶段、终点与楼梯目标
                    准备完整参考或局部短段
                    构造 LocalPlanRequest，调用 planLocal()
                    处理等待、失败及适用的恢复分支
                    调用 Follower，返回候选速度与状态
```

`planLocal()` 调用 `local_planner_.plan()` 并把调试／碰撞证据转成地图坐标。SCAN 求解在规划目录内；跟踪方程在 Follower 内。

## 功能清单

| 功能 | Executor 自己做的决定 | 关键入口／主要协作者 |
| --- | --- | --- |
| 路线生命周期 | 激活／替换／清除路线，保留请求终点，重置路线进度和相关状态。 | `setRoute`、`activateRoute`、`clearRoute`、`resumeRoute` |
| 速度与任务容差 | 应用任务速度上限、终点位置／朝向容差，结合楼梯和转弯限制。 | `setSpeedLimit`、`limitPoseSpeed`；部分数值整理由构造函数承担 |
| 姿态路线 | 调用编译得到行走／转身阶段，选择阶段、切换阶段、尝试连接转角，失败时恢复原分段。 | `activatePoseStage`、`tryBlendPoseStage`；`pose_route.*` 实现几何编译 |
| 坐标适配 | 选择 map 或 odom 规划，将障碍和预测数据转换到同一坐标；坐标版本变化时处理旧状态。 | `tickRoute`；变换与路线工具也在 `route.cpp` |
| 局部规划请求 | 准备路线、运动状态、地图、限速和跟踪器实际进度样本；取得规划结果。 | `makeLocalPlanRequest`、`planLocal`；调用 `LocalPlanner` |
| 普通路线跟踪 | 设置跟踪器的参考、期望朝向、速度、末端距离和慢速倍率；选择路径或样条的状态输入。 | `tickInPlanningFrame`；调用 `Follower::follow` |
| 终点与朝向收尾 | 决定何时调用终点控制，传入位置／高度／朝向与实测运动；按结果设置到达或等待状态；有 Body 时检查转身空间。 | `Follower::finish`、`atGoal`、`turnClear`；不是驱动端停止确认事务 |
| 楼梯段接续 | 使用已提供楼梯段及运动模式反馈，处理入口、出口、方向、限速及局部路线约束；阶段变化时重置局部执行。 | `tickStairTransition`、`stairs_.guideGoal`、`localPathAllowed` |
| 辅助运动意图 | 把平移速度方向转成前方滚动参考，保持操作方向，处理方向／转向变化，调用局部规划与跟踪。 | `tickIntent`、`TeleopReference` |
| 恢复与脱困 | 按规划器能力决定是否启动普通恢复；辅助模式可构造经检查的短距离边界脱离路径，或调用旋转恢复。 | `Recovery`、`recovery_follower_`、`boundaryDepartureClear` |
| 暂停与实际执行反馈 | 保留或丢弃局部曲线，冻结参考进度；记录最终命令被拒绝的时间并接收已应用命令。 | `pauseLinearMotion`、`stopLinearMotion`、`reportFinalMotionBlocked`、`commitApplied` |
| 状态与诊断 | 汇总目标、预览路径、规划错误、跟踪误差、恢复和姿态阶段等输出。 | `ExecutionOutput`、`planLocal`、预览坐标转换 |

## 不能把不同分支混为一谈

1. SCAN 的 `ownsReferencePath=true`，收到完整参考后自己推进局部目标；CMU 使用 Executor 准备的局部短段。普通路线恢复代码由 `!ownsReferencePath` 包围，不能声称 SCAN 普通导航还会走整套 CMU 恢复。
2. SCAN 辅助运动另外支持经过几何检查的短距离边界脱离。当前代码直接构造 0.35 m 的短路径，再交给恢复跟踪器；这不是普通路线恢复。
3. `MotionIntent` 的这里以平移方向为主，平移速度为零会进入 idle；纯原地转向由遥控端点分流，不能说所有遥控都经过 `tickIntent()`。
4. 楼梯是显式输入的楼梯段接续能力，不在本文件里识别点云。当前正式端点的路线激活未填充 `Route.stairs`，正常 tick 也未传入楼梯模式反馈；与 `navigation/README.md` 的实验路径说明一致。不能把这些分支列成已接通的正式自动入梯能力。
5. 有 `posePoints` 才运行显式姿态阶段；普通 XYZ 路线不会无条件执行所有姿态切换与支撑元数据规则。

## 暂停、停止、清除不是同一操作

| 调用 | 保留什么／改变什么 |
| --- | --- |
| `pauseLinearMotion()` | 保留路线和曲线，暂停规划侧参考进度，停止两个 follower 的线运动。 |
| `stopLinearMotion()` | 重置局部规划／当前跟踪目标与辅助运动状态；不等于 `clear()` 删除整条路线。 |
| `resetMotion()` | 清空局部规划、跟踪、恢复及缓存等执行状态，保留已接收路线。 |
| `resumeRoute()` | 执行 `resetMotion()`，把路线进度归零并增加路线版本，从当前位置重新连接。 |
| `clear()` | 清除路线／目标／姿态状态并取消楼梯段。 |

## 为什么看起来职责过多

它不只是一个转发器。普通路线流程里混合了姿态阶段切换、楼梯入口对齐、终点控制、规划器分支和恢复选择。辅助运动流程又维护独立参考、方向重置、脱离路径及恢复状态。

具体的非转发行为包括：楼梯入口在约 8 cm 范围内直接调用 `Follower::turnRate()` 形成角速度；辅助边界脱离直接构造 35 cm 路径；姿态分段合并、失败退回、速度限制也直接在本文件决定。方法调用没有让这些决策消失。

实际执行进度仍由 Follower 产生，Executor 把样本回传给规划请求；这是协调所需的信息传递，不应再次累计一份曲线时间。`route.cpp` 的 `buildSegment/buildReference/atGoal` 与 `state.cpp` 的 `planningKinematics/autonomyMotionStalled` 是同一个 Executor 类的方法，文件拆开没有新增状态所有者。

清理顺序建议围绕已有概念：先让普通路线和辅助运动两条流程各自完整可读，再收拢显式姿态／楼梯的阶段决策，最后核对结果处理和诊断转换是否重复。保留线程交接、坐标、暂停和真正的停止语义；不能仅按函数相似或行数删代码。

## 本次界限

这是功能与责任梳理，不是完整缺陷审计。没有修改 C++、阈值、到达语义或测试，未重新运行此前的 Executor 测试；之前 104/129 或 120/129 的数字不能当作本次源码阅读的测试结果。
