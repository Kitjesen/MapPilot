# 已确认修改的正式接入核查

核查日期：2026-11-03。本页后续更新采用用户明确的 BODY2 少量分部模型。
范围是本轮对话中已经确认的导航、模型、建图状态和 Web
地图能力；不把论文建议、隔离实验或设计文档当成正式实现。实机部署未核验。

## 为什么出现“改过，但正式入口仍没使用”

Thunder 的 22 部件只是 `build/thunder-compact-body-20261101/` 内的实验，
real RobotConfig、sim RobotPackage 和重新生成模型的导出器仍选择原来的 38 部件。
这个实验也只是合并部分腿部几何，仍有轮子查询，并没有完成后来确认的整体表示。
它需要改正式选择及再生成入口，不能靠更新设计文字交付。

另一类是组件能力与产品入口没有完整衔接：连续降速算法用于辅助遥控，但普通自主
任务没有保持任务不变的调速入口；MotionIntent 用于辅助遥控，自主恢复仍走独立路径。
必须继续追到输入生产者和执行者，不能把后端有分支当作所有运行模式都已接通。

本页早先把“普通全局路线没有 pose_points”列作缺少接入，分类不准确。
10 月 30 日的项目记录明确删除了 poseGuide、XYZ+yaw 搜索与全局 Body 转身准入。
当前测试有意要求普通路线、重规划和目标替换的 pose_points 为空。
这是后续简化中退役的流程，不应直接接回；全局与局部完整几何统一仍需单独设计。

## 本轮调整

| 项目 | 正式改动 | 验证与边界 |
| --- | --- | --- |
| Thunder BODY2 导航模型 | real/sim 均选择 `thunderv4_navigation.body2`，导出器生成相同的 8 部件资产：4 个机身部件、4 个整腿位净空包络。替换先前未经用户指定的单盒。物理 MJCF 与 CTS v39 不变。 | 每腿 XY 覆盖源髋、大腿、小腿及轮子的名义投影，Z 使用髋／大腿净空；导出 11 项通过。同输入几何比较与物理结果分开记录。 |
| 普通自主任务运行中调速 | 新增同任务 `TaskSpeedLimit` typed DDS 命令，Gateway、Python/C/C++ client、正式 endpoint 均接入；更新活动任务、后续重规划目标和 Executor 上限，不重新提交目标。 | 正在执行的旧全局规划完成后读取最新限速；显式姿态路线跨阶段也保留新上限。软件回归及物理测量见后续证据。 |
| 自主平移恢复复用 SCAN | Recovery 选择有限临时目标，Executor 通过正常 SCAN Task/FSM 和同一 spline follower 执行；Pending 期间保留目标身份，实测到达后恢复原路线。 | 原任务不替换，取消／换目标清理临时状态。CMU、旋转和辅助遥控边界脱离仍有各自执行流程，不能扩大为所有恢复已统一。 |
| Gateway 错误的坐标漂移判据 | 删除绝对地图坐标超过 50 m 就判发散的条件及其无用配置；保留非有限里程计和异常速度检查，日志不再声称直接知道 IEKF 发散。 | 合法远坐标、非有限数、正负速度与线程启停定向回归通过。 |
| Gateway 三轴速度实际输入 | 正式里程计回调补齐 vy/vz，并纳入原有入口有限值检查；速度诊断不再只收到 vx。 | 包含真实回调的里程计／诊断 15 项及线程 3 项通过。 |
| 快照完成被称为保存完成 | 两个 SLAM 后端的快照完成状态统一为 `map_snapshot_ready`；mapd 仍在实际提交完成后报告正式地图保存成功。 | 保留 `map_loaded` 和重定位器安装，因为它们描述实际内存状态；没有改为先加载正式地图或重置定位。保存／几何回归单独记录。 |
| 仿真验收时钟比较 | 状态文件按墙钟判断新鲜度，传感器有效性使用原生输入门槛；删除传感器时间与墙钟的重复比较。 | 前一测试轮 42 项相关回归通过，当前继续使用；没有放宽原生输入、碰撞或运动阈值。 |
| 大地图仿真启动预算 | sim 的 nav 冷启动等待由 60 s 调整为与 feeder 相同的 240 s；地图索引和表面图必须准备完成后才启动控制循环。 | 本轮实际准备耗时 53–69 s，原预算不够；冷启动契约及相关 RunPlan 3 项通过。控制频率、输入新鲜度和运动阈值不变。 |
| Host 地图显示流的仿真时间 | RunPlan.env 经正式 Host 装配传入 HostBus；sim 的 MapScene 源时间不再与墙钟作年龄比较。 | 23 项 HostBus／RunPlan 测试通过。两环境保留接收时间、MapState 墙钟年龄、live、容量、boot、epoch 检查；real 继续校验场景源时间年龄。 |
| 本地正式 Web 构建同步 | 最新源码重新生成 Gateway 默认读取的 web/dist，入口与点云页面脚本更新；源组件中已经存在的调用不重复增加。 | TypeScript/Vite 构建、38 项地图保存／导航状态测试通过，默认静态挂载实际返回新入口与脚本 HTTP 200。远程部署未核验。 |

用户最新明确选择 BODY2 少量分部模型。正式 8 部件总轴向外接尺寸为
0.835000 × 0.598331 × 0.411549 m。每侧 8 cm 水平余量仍在运行时应用一次，
零 yaw 的外接水平范围为 0.995000 × 0.758331 m。源圆柱使用解析投影，修正此前
用斜圆柱的外接 OBB 再合并所增加的角点范围；不是靠删除轮腿的水平占地缩小。
上身-only 的 5/9/12 部件候选因遗漏轮腿每侧约 8.45 cm 水平范围而未采纳。

少量分部保留上方前肩、头尾和传感器周围空角。固定朝向的 matched L/S 几何比较
显示：多数改善来自上述投影修正，分部本身只减少部分拐角相交，不能宣称它必然
通过所有窄弯。新模型属于上方净空代理，仍不覆盖完整摆腿扫掠或证明低物／楼梯
可以通过。实际物理模型未简化。证据见
[8 部件几何对照](../../build/thunder-body2-20261103/README.md)。

来源：[导出器](../../tools/robot/export_navigation_bodies.py)、
[real 选择](../../config/robots/doso/thunder_v4/robot.yaml)、
[sim 选择](../../sim/packages/robots/doso/thunder_v4/robot.package.yaml)、
[模型来源与尺寸](../../config/robots/doso/thunder_v4/body/thunderv4_navigation.json)。

## 已经接入的修改

| 已确认要求 | 正式调用与实际状态 |
| --- | --- |
| 删除 unknown 空气及逐脚支撑的额外拒绝 | SCAN Grid 查询实测占据，未占据的格子直接跳过；prepared-support 回调及逐脚证明已经移除。普通碰撞与边界检查保留。 |
| 集中最终命令和制动 | `control/final.cpp` 从 EndpointState 取得输入，执行实测速度和候选命令制动。局部请求和最终制动共用活动 Body 的选择规则。 |
| nav 不使用独立二维 traversability | real、sim 的 nav RunPlan 都关闭该代价并不启动独立进程；其他 Product 有各自配置。 |
| 短段提前续接、落后时从实测状态重规划 | 普通自主 Executor → Task → Backend/FSM 已调用相应实现；物理效果仍需对应场景验收。 |
| 连续降速和 MotionIntent | 普通同任务调速已接入正式 endpoint；自主平移恢复通过稳定的 MotionIntent 临时参考交给 SCAN。辅助遥控原有入口保留。代码接线、软件测试、物理测量分别列证据。 |
| CTS v39 统一接口 | 正式 sim ControlPlan 的普通／上楼／下楼均选择 v39。real DOSO driver 向外部 Brainstem 发速度，其内部模型不能由本仓库的 sim 配置证明。 |
| 清理旧 0.75 m/s 默认上限 | 配置为每轴 1 m/s、路线范数 √2 m/s；普通自主请求在 Executor 再受 max(maxVx,maxVy) 限制，当前传给 SCAN 通常为 1 m/s。辅助遥控使用 hypot 两轴限制；不能只看配置范数推断实际请求或实发速度。 |
| 在线 Worker 与共享文件读取归属 | Worker/Backend 已在 online_mapping；独立 mapping_io 被离线优化、回放复用，离线读取不再链接整个 Worker。 |
| 保存失败原因、最新在线失败与旧预览 | mapd 优先事务错误；Worker/SLAM/Gateway 传递最新失败计数并保留最后有效几何；Web 识别在线失败状态。 |
| Web 原始 OctoMap 与实际分辨率重建 | 显示及复制后重建入口均有正式调用。HTTP 的 resolution 经 transport 转成 octomap_resolution 后由 C++ 构建器读取；点大小显示选项仍只影响显示。 |

详细源码证据保留在
[导航调用核查](../../build/production-integration-20261103/navigation-audit.md) 和
[地图功能调用核查](../../build/production-integration-20261103/map-delivery-audit.md)，
状态接入见[本轮状态核查](../../build/production-integration-20261103/state-audit.md)，
建图状态边界见[建图职责记录](mapping-responsibilities-20261102.md)。

## 仍未完成，不能标为“已落入”

| 项目 | 缺少的实际能力 |
| --- | --- |
| 全局与局部使用相同完整机体几何 | 全局仍是 XYZ＋圆柱通道，Body 参数用于转交局部。仅更换模型文件不会增加沿途 yaw 搜索。 |
| 导航不中断保存完整旧图与新扫描 | 正式入口仍限 map Product；缺完整地图合并、坐标与版本提交流程，不能只删除模式限制。 |
| 实时射线否定旧推断支撑 | 实时占据／自由更新已工作，但准备好的全局 SurfaceIndex 没有随这些证据撤回推断层。 |
| Web 显示全局实际推断支撑 | 还没有对应的正式传输与渲染链；mapd 局部 ground_height 不能冒充全局推断面。 |
| 跨会话恢复旧 iSAM2 图继续优化 | 保存的 patch／loop 文件不等于恢复完整优化器；loadMap 当前用于重定位，Backend 重置仍创建新图。 |
| 更换旧活动地图后的仿真冷启动 | mapd 启动读取旧图，而目标扫描已绑定新图；冷启动激活晚于输入就绪检查。需要带失败恢复的启动顺序修正，见 [失败分析](../../build/thunder-navigation-20261103/cold-start-map-order.md)。 |

这些是明确剩余工作，不通过一个配置开关冒充实现。本轮不声称它们已经修好。

## 已保留组件与已退役流程

`pose_route`、`poseRouteBlend`、`poseRouteContinuous` 仍支持显式 Route.posePoints
及测试/benchmark；普通点击导航使用 XYZ → SCAN，不进入这些显式阶段。
历史全局 poseGuide 与 heading::search 已从源码和 CMake 删除，只有历史快照。
BodyQuery 当前用于离线诊断。把它们重新接到普通导航会改变执行语义，可能重新
启用独立停转和圆弧参考，不属于补一个漏掉的调用。

来源：[10 月 30 日删除记录](../../build/xyz-global-20261030/README.md)、
[当前全局模块说明](../../src/nav/cpp/planning/global/octoplanner/README.md)、
[正式激活回归](../../tests/nav/cpp/endpoint/test_goal_plan_controller.cpp)。
更细的“组件已用在哪个模式、另一个模式还缺什么”见
[本轮调用核对](../../build/ready-code-audit-20261103/README.md)。

## BODY2 与正式调用链的后续验证

最新代码的 Windows `navd.exe` 与 `lingtu_nav_client.dll` 已重建。正式选择核对
读取了 real/sim 各自 RunPlan 指向的文件内容，两者均为 `BODY2 8`，不只是路径相同。
相关 Python 回归 117 项通过，另有模型导出 11 项、原生 Body 42 项、
RecoverySequence 13 项、RecoveryRotation 3 项及调速／恢复／姿态阶段定向回归通过。
C++ 和 C API 的实际 DDS roundtrip 已覆盖 kind 10 的 payload、身份与 ACK。
协议与 API 文档生成一致性检查通过。完整枚举期望表另有既有的 Inspection/Sensor
状态差异，未将该基线失败隐去。

最新物理记录在 [BODY2 MuJoCo 证据](../../build/body2-motion-20261103/README.md)。
它使用正式 RunPlan 选择的 8 部件模型和 CTS v39、完整 Thunder MJCF、
20 Hz 原生导航循环及真实模拟激光，定位使用 MuJoCo 真值。启动方式是组件验收，
不是完整 ProductControl/Gateway/LTLIO 验收；全局与局部、driver 和物理步进是真实运行。

| 场景 | 实际结果 | 验收范围 |
| --- | --- | --- |
| `straight-02`，1.1 m 直线目标 | 实际净位移 1.016 m，水平终点误差 0.084 m，原生到达，0 场景接触 | 输入就绪、完整录像、物理零命令 ACK、进程退出全部通过 |
| `box-01`，3.6 m 直线距离，途中新增箱子 | 实际轨迹 4.092 m，水平终点误差 0.109 m，原生到达，0 场景接触 | 输入就绪率 99.5%，控制循环超时 p99 6.80 ms；末端朝向响应慢需单独说明 |
| `speed-01`，4 m 直线中途调到 0.3 m/s | 实际净位移 3.983 m，水平终点误差 0.023 m，原生到达，0 场景接触 | 同一任务接收 kind 10 ACK，输入就绪率 100%，完整录像与停车确认通过 |
| `speed-02`，同场景加密诊断记录 | 实际净位移 3.923 m，水平终点误差 0.078 m，原生到达，0 场景接触 | **完整验收失败**：输入就绪率 95.95%（要求 98%），控制循环超时峰值 287.98 ms（上限 250 ms）。控制参数未变，状态记录由 5 Hz 提高到 20 Hz，物理日志由 10 Hz 提高到 50 Hz。 |

保存的旧地图只有空走廊。直接读取 PCD 与 OctoMap 确认，新增箱体区域原本为
已观测自由，因此 box 场景的全局直线符合保存地图；这项证据验证实时 SCAN 绕障，
不是静态地图中的全局绕箱。初版 manifest 对“同场景地图”的描述已修正，原报告保留。

前三项验收通过，但加密诊断复跑没有通过，不能据此称运动质量已经完善：box 的显式末端 yaw=0
请求花了约 12.7 个仿真秒对齐。该段持续下发纯转命令，不是在等停稳；约 0.2 rad/s
命令下实际偏航速度只有约 0.005–0.009 rad/s。到 5° 末端容差才完成。没有修改
容差或加大共享控制参数来掩盖这个 CTS 小幅纯转响应问题。

进一步隔离导航，直接给同一 CTS／完整 MuJoCo 模型 8 秒纯转命令，丢弃前 1 秒后，
0.2、0.4、0.6 rad/s 命令的实际平均角速度分别为 0.003663、0.230860、
0.492600 rad/s。这证明低角速响应弱在执行链内可以独立复现，尚未进一步区分
策略、观测／动作适配和物理接触模型的贡献。见
[纯转原始证据](../../build/body2-motion-20261103/pure-yaw-summary.json)。
终点转向的 0.2–0.6 rad/s 目前仍来自 ExecutorConfig 默认值，没有和普通跟踪
角速度的正式配置统一；本轮未凭三档仿真就修改所有机器人共享的控制参数。

speed-01 在约 13.7 个仿真秒记录到单个零命令，原 10 Hz 状态无法定位产生它的
具体周期。speed-02 的高频记录捕获了约 19.2–19.4 秒的零命令，原因明确为
`collision_stale`；实时碰撞输入恢复后轨迹从 4 切到 5。该证据解释了复跑中的
地图过期停车与重规划，不能倒推第一次短零必然同因，也不能归咎于调速 ACK。
超时周期的线程 CPU 约 15.6 ms、工作墙钟约 338 ms，尚无证据把全部延迟归于
SCAN 求解器。不能通过放宽地图新鲜度或控制健康门槛将此验收改为通过。
另外 straight-02 在距目标约 0.38–0.34 m 与 0.20–0.18 m 时也有短零命令，
相邻状态包含 `local_plan_pending`，不能把它们全部算成到达 0.15 m 半径后的
正常停车。当前验收通过代表所列到达、碰撞、输入等条件通过，并不排除全部顿挫。

第一次直线 `straight-01` 已实际到达，但关闭局部地图调试记录导致录像叠加验收失败；
恢复记录后的 `straight-02` 通过。speed 的辅助状态观察脚本在 ACK 后约 15 秒碰到
Windows 文件读锁而退出；已保存同 task/epoch 的 221 条状态和成功 ACK，主仿真
及完整 motion 记录正常结束。脚本已处理该已观测的瞬时读锁；不把它说成产品失败。

## 早先运行证据（保留失败历史）

基线 [2026-11-03 导航测试](../../build/thunder-navigation-20261103/README.md)：
38 部件模型下，真实位姿与 LTLIO 两次 2 m 目标均未到达，局部轨迹起点碰撞。
本轮新模型回归、失败快照重放和正式 Product 仿真保存在
[`build/production-integration-20261103/`](../../build/production-integration-20261103/)。
只有 RunPlan 确认选中了新模型并记录实际目标／物理位置，才能称为正式仿真证据。

当前源代码编译的离线 Grid 重放已复现两份旧失败快照的原始碰撞位置、时刻和状态。
真实位姿快照在 t=0 拒绝；LTLIO 快照的起点本身空闲，候选轨迹在 t=0.13 s
出现第一次碰撞。新整体模型在两份原失败轨迹段均返回空闲，保留其原始占据、
坐标变换、余量和候选样条。这证明对应几何冲突已消除，不等于完整物理路线通过。
重放脚本和结果位于上述构建目录的 `prepare_replay.py`、`replay-*-candidate.json`。

本轮最终 real/sim RunPlan 再次编译成功，均指向新的 `thunderv4_navigation.body2`，
记录在 `final-runplan-check.log`。Windows 正式 navd、slamd 已重建；SLAM stage 的
独立 DLL 加载检查通过。保存／几何三个目标通过，其中 Windows 几何测试原本在
文件流未关闭时删除目录，修正测试资源释放后通过；原始失败日志保留。

新模型完整 MuJoCo 验收**未通过**。`truth-body-05` 在发出目标前持续触发控制循环
就绪条件，`goals=0`、`paths=0`、`outputs=0`；约 240.9 s 窗口的 IMU/LiDAR 实际
频率为 102.60/5.127 Hz，目标为 200/10 Hz。地图准备约 48.5 s。输入门槛曾为 ready，
控制循环原因记录为 `deadline_miss_ratio_high`。缺少完整分阶段统计，不能把原因
归为某一个 CPU 热点或模型查询。最后物理零命令 ACK 已记录，退出后未见本轮
native 进程残留；此启动失败没有通过 Product 全套清理／回滚验收。回滚另遇到
配置整理过程中的字段错误，最新无进程 RunPlan 核对已通过。
详情见 [运行限制](../../build/production-integration-20261103/current-run-perf/README.md)。
未放宽运动、输入新鲜度或控制健康门槛，也未部署到 NX／实机。
