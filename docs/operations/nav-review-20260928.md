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
