# 普通点目标的到达收尾

2026-11-09，针对箱体过门录像末段的大转身，修改普通点目标的到达交接。
XY 到达范围仍为 15 cm，高度容差、机器人包络、碰撞查询、策略和速度参数均未调整。

## 原因

历史录像在 driving 约 20.68 s 已进入 15 cm，yaw 仍约 -3°，但是轨迹 17
只执行了 2.535 / 3.200523 s。Executor 等待局部参考完成，继续下发移动命令，
随后 SCAN 生成短修正曲线，身体为对齐该曲线而大幅旋转。

`atGoal()` 本身只判断请求目标的 XY 距离和独立高度差。多余的等待在
`Executor::tickInPlanningFrame()` 的最终到达分支。较早冻结程序等待
`tracking.finished`；本轮修改前的源码已放宽为参考时钟到期，仍有同一问题。

## 最小修改

- 普通点路线满足既有位置范围后，立即交给现有到达停车事务，不等待样条时钟。
- 显式 pose 最后阶段仍保留原时间条件；楼梯的 `target / hold / arrivalAllowed`
  仍参与判断。普通点目标没有新增终点 yaw 控制。
- `route.cpp` 的 `atGoal()` 使用简短参数名、引用目标，直接表达 XY 与高度比较。
  请求目标仍是最终判断基准，避免把规划调整误差与到达半径相加。

生产修改限于 `navigation/executor.cpp`、`navigation/route.cpp` 及声明中的参数名。
前后精确差异保存在 `artifacts/nav/arrival-fix-20261109/change.patch`。

## 外部源码对照

SCAN 固定提交 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。其控制器确实要求
样条执行时间结束并且终点 XY 误差满足容差，航向来自样条预瞄方向；它没有显式
任务最终 yaw 契约。FSM 的最后目标结束又是独立的参考时间判断，不能混为实际到达验收。

Nav2 固定提交 `35d920348c447babf02889d82812993f3967298a`：PositionGoalChecker
判断 XY 和剩余路径长度，SimpleGoalChecker 再判断目标 yaw，StoppedGoalChecker
再加入线速度和角速度条件。它们均不要求样条时钟走完，也不检查 Z。这些插件的
默认 XY 范围为 25 cm，新版剩余路径长度默认上限为 1 m。LingTu 保留自己的三维高度条件。

- [SCAN 控制器](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/closed_loop_controller.cpp#L186-L224)
- [Nav2 PositionGoalChecker](https://github.com/ros-navigation/navigation2/blob/35d920348c447babf02889d82812993f3967298a/nav2_controller/plugins/position_goal_checker.cpp#L90-L131)
- [Nav2 SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/35d920348c447babf02889d82812993f3967298a/nav2_controller/plugins/simple_goal_checker.cpp#L118-L179)
- [Nav2 StoppedGoalChecker](https://github.com/ros-navigation/navigation2/blob/35d920348c447babf02889d82812993f3967298a/nav2_controller/plugins/stopped_goal_checker.cpp#L99-L140)

完整参数与 RPP 收尾对照见 `artifacts/nav/arrival-fix-20261109/references.md`。

## 验证

新到达回归在修改前失败：已进入任务位置范围、样条仍活动时，旧代码继续运动。
修改后 9 个执行器目标测试、61 个 pose 路线测试和 33 个楼梯测试全部通过。
构建仍有既有上游及端点编译警告；修改处没有新增编译诊断。

同地图、起终点、冻结环境和策略，分别构建当前源码的修改前后程序：

| 指标 | 修改前 | 修改后 |
| --- | ---: | ---: |
| 首次入 15 cm 至到达事件 | 6.44 s | 0.02 s |
| 入圈后累计绝对 yaw 变化 | 177.398° | 0.485° |
| 入圈后新增轨迹 | 1 | 0 |
| 最终记录 XY 误差 | 12.528 cm | 11.841 cm |
| 最终记录 yaw | -175.746° | 16.053° |

两轮均通过组件验收。首次入圈速度约 0.892 / 0.516 m/s，接近目标前的局部轨迹
也存在运行差异，不能把整轮时间差都归因于这项条件修改。历史约 107° 录像是另一轮。

独立关闭 stop-on-goal，运行 40 s，到达后继续观察 15.36 s：全程未出 15 cm，
最后 2 s 的 XY 差分速度均值 0.001651 m/s、最终误差 13.458 cm、yaw -4.802°、
yaw 摆幅 0.101°，命令三轴为零。该独立运行也通过组件验收，其姿态不替代上表的 after。
这是有限窗口内的残余运动记录，不是绝对静止证明。

源码修复已构建为 navd，并将验证过的可执行文件和同版 DDS DLL 同步到本机 Windows
sim 的默认路径 `build/nav-cpp/windows-x64-nav-endpoint/Release/`，与测试文件直接
逐字节比较一致。原默认文件保留在本轮 `default-before/`。`--help` 能加载并输出
既有 usage；其现有实现以异常返回 1，未据此声称完整 Product 启动验证。

## 证据与边界

证据目录为 `artifacts/nav/arrival-fix-20261109/`，包含前后源码差异、红绿测试、
固定程序、参考源码说明、对照图、停车观察和原始运行链接。
录像分别保留在 `artifacts/nav/speed-tracking-20261108/runs/` 下的
`box_door-formal-arrival-after-20261109/navigation-follow.mp4` 与
`box_door-formal-arrival-settle-20261109/navigation-follow.mp4`。

本次消除进入位置范围后的多余追尾段转身，没有新增强制最终 yaw 对齐。
证据是本机 MuJoCo 真值定位组件；NX、实机、完整 ProductControl 和 LTLIO 未验证。
此前 L 弯局部续接失败和长路线控制周期峰值超时，不由本项修复解决。
