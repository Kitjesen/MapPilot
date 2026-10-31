# 官方 SCAN 与灵途扩展审查

日期：2026-10-26。审查对象是官方 SCAN 固定提交
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26` 与当前灵途工作树。
工作树含其他并行改动；本文没有把它们当成本轮已部署结果。
错误与时间统一记在 [error_record](../operations/error_record.md)。

## 判断

灵途确实比官方增加了许多产品和执行能力。其中“整个机体已观测为空闲”的额外条件
是错误扩展；它与 OBB、URDF 的选择没有必然关系。另两处已修的是端点采样假膨胀和
没有收益比较的轨迹替换。不能由这些错误推出所有三维几何、支撑或最终命令检查都应删除。

官方是 route-guided 局部规划研究实现。灵途还承担保存地图、定位、全局路线、原生控制权、
实测状态和最终驱动指令；边界不同，代码量不能直接代表合理程度。

## 对比与取舍

### 2026-10-28：朝向对齐对照

固定官方版本仍为 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`，不是未经核对的最新主分支。
官方 `advanced_param.xml:96–102` 配置 `time_forward=0.8`、
`heading_error_threshold=0.8 rad`、`kp_yaw=1.5`、`max_vyaw=1.0 rad/s`。
`closed_loop_controller.cpp:190–200` 在朝向误差绝对值超过阈值时停止 XY 平移、
保留比例控制的 yaw 命令，并冻结执行时间；回到阈值内恢复推进。官方这条分支使用单阈值。

灵途当前 nav Product 的进入阈值为 `0.4 rad`，退出使用 follower 的 `0.35 rad`，
即约 23° 进入、20° 退出。停止平移和冻结时间继承了官方行为；更早进入与退出滞回
属于本地差异，不能把它们说成官方默认。另有显式 heading/yaw-rate 输入和有条件的
切向角速度前馈，不能只改一个阈值便声称完整对齐官方。

今后的导航行为和阈值修改应保留：固定官方提交及参数、对应源码行为、本地差异与理由、
同一场景/输入的修改前后结果。对齐阈值的比较至少覆盖普通弯道、大朝向误差起步和临界
角度附近波动，记录平移冻结时间、路线误差、到达与实体接触；不以一项耗时改善代替全部效果。
本次仅核对并记录，没有将产品阈值改为 0.8，也没有新增物理验证。

官方来源：[示例参数](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml#L96)、
[闭环跟踪](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/closed_loop_controller.cpp#L190)。

| 项目 | 官方固定版本 | 当前灵途扩展 | 判断 |
|---|---|---|---|
| 地图更新 | ROS 回调接点云/位姿，滚动三维网格、raycasting、log-odds、增量膨胀 | 原生 mapd/DDS 维护实时三维占据，保存地图另有所有者，Grid 做适配 | 保留职责划分；不需要把官方 ROS 地图再接成第二份所有者 |
| 未知空气 | raw 记录 unknown；inflated 初始 0，窗口内不因 unknown 身份单独阻挡 | 10-22 显式 Body 路径曾增加整机 known-free；目前局部/全局实体检查已取消 | 已证实错误，删除该额外准入。未知证据本身仍应保留 |
| 机体表示 | 障碍按 XY 圆盘×Z 高度膨胀，沿 yaw 查询前后两个中心 | OBB/显式部件直接检查原始 occupied，可按 RobotConfig 选择 BODY2 | 合理能力；形状、unknown 政策和安全余量应分开，不再重复膨胀 |
| 候选检查 | 最终采样检查最近 2/3 轨迹，点处读双圆柱膨胀值 | 发布候选全时段、段间扫掠、实际朝向与高度检查 | 保留。真实中间旋转碰撞已有回归，不能退回仅检查采样点 |
| 采样余量 | 不使用我们的整部件连续采样合同 | 原先把 half-travel padding 加在真实端点，造成假起步阻塞；现改端点+中点覆盖 | 已证实错误且修复，不等于应该缩小物理模型 |
| 地形/支撑 | 这条 inflated 查询不要求脚下有可承载表面 | 全局地面/步高/坡度与明确要求承重的 contact 部件检查保留 | 已删除根据 route 支撑高度强制整身逐部件投影逐列证明地面的局部前置；它不是官方要求，不以新 gate/flag 替代 |
| 动态障碍 | 上述网格碰撞查询是当前占据层 | 按相应到达时刻检查预测体积并保留实测占据 | 合理扩展；不等于已有完整时空绕行/等待优化 |
| 实测运动与刹车 | 不含灵途这套原生最终命令链 | 实际速度的制动空间，以及将要发送命令的制动空间分别检查 | 保留，输入和时刻不同，不是同一检查简单重复两遍 |
| 重规划衔接 | 从当前轨迹状态重规划，不具备灵途控制线程交接协议 | 异步 PV/A handoff、旧曲线执行时钟、迟收撤回；新增同覆盖收益偏好 | 保留必要交接；无收益反复换线已修，不能保留已受新障碍影响的旧尾段 |
| 转弯朝向 | 主要按路径切向估 yaw，机身为双圆柱近似 | 显式 XYZ+yaw、移动/原地转向阶段，受限连续转弯 | 有必要但未完整交付自然转角。换模型不会自动修正姿态目标与执行合同 |
| 产品控制 | ROS 节点/launch 示例 | ProductControl、原生服务、控制权、急停、松键与时效 | 官方局部规划器没有覆盖完整产品职责，不能据其简单删除这些边界 |
| 诊断 | ROS 日志与可视化 | 最新状态、失败输入快照、回放与人工事故记录 | 保留已存在证据；失败快照非完整历史，本轮只补复盘入口，不造重复日志系统 |

官方证据：
[初始化](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L108)、
[双圆柱查询](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/include/plan_env/grid_map.h#L365)、
[膨胀更新](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L190)、
[候选检查](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/bspline_opt/src/bspline_optimizer.cpp#L1002)、
[FSM](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp)。

## 确认该精简什么

2026-10-28 起点职责调整：全局有向搜索不再因保存地图中的起点机身占据立即返回失败。
占据起点可以连接同朝向的相邻空闲姿态，第一段由实时 SCAN 验证；原本空闲的起点、
后续边及转身仍检查扫掠，支撑不变。这是 LingTu 的全局/局部职责修正，不是官方 SCAN
已有同名入口，也不意味着局部起点实测碰撞可以忽略。

同一固定官方版本的 grid_map.cpp 使用滚动 occupancy_buffer_ 和 log-odds 更新，
不是用一帧扫描完全替换历史占据。灵途 mapd 同样保留融合占据：新扫描未覆盖旧障碍位置时，
更新了快照并不证明该位置已变空。SCAN 已有局部图清除后保持原目标继续规划的回归；
动态箱体物理试验仍需验证射线清除、局部轨迹恢复及实际到达，不能把组件恢复通过写成
整条物理闭环通过。

1. **已去除：整机体 known-free 才允许运动。** 它不是原版要求，也不是用户选择 OBB 时的要求。
   10-22 的冻结源码和测试证明当时已作为显式 Body 合同引入；测试通过只是固化了错误政策。
2. **已修复：数值端点余量假阻塞。** 原尺寸不变，连续覆盖仍在，去掉直接向端点额外扩张。
3. **已修复：无收益曲线替换与失败后重复优化。** 保留真实有效尾段，继承执行时钟；
   内部约 10% 是同覆盖候选的替换偏好，不是新的速度上限。
4. **已删除：supportHeight 整身逐列地面准入。** 早先曾从 AABB 全列改为实际部件
   投影筛选，但后续正式 optional-foot 模型仍因躯干下缺观测而被拒绝。当前整段前置
   已移除；route/global 支撑高度元数据保留，实际占据、明确 required 接触脚与全局
   三维支撑检查未删除。普通无此元数据的接管本来就不走该前置。足部与地面体素
   相交的独立失败仍未解决，删除记录与验证范围见错误记录。
5. **已接入位姿几何复用，保留碰撞保证。** 同一 part/pose 对每个 occupied voxel 调用
   SAT 曾重复变换、三角函数和世界轴计算。现用 [body.cpp](../../src/nav/cpp/collision/body.cpp)
   的栈值 PreparedPart，在全局与局部每个部件位姿内复用，不做跨位姿缓存。
   外层 10 ms 轨迹采样与 1 cm 扫掠的相邻端点仍有重复，本轮没有删除这些检查。
   没有 NX profiling，不能写成现场慢多少或唯一延迟主因。

没有证实的问题不计入错误数：fixed heading 在各未来段重复检查测得 yaw→目标 yaw
可能过保守，但尚缺独立反例和跟踪误差合同核对；本次不把它写成现场根因。
缺少 raw 输入、机身越网格、真实障碍相交、明确支撑缺失，也不能因“简化”一律放行。

## 对附件专业评价的核验与采用

附件对地图层、索引、支撑范围和共享几何提出了有用问题，但只看 Grid 适配层会漏掉
生产者、Product 编译、输入门和最终命令仲裁。结论必须对照真实调用链，而非按函数数量
推断存在多套互相矛盾的安全系统。

| 附件主张 | 当前源码证据与处理 |
|---|---|
| raw/inflated 输入没有区分 | [LocalCollisionMapView](../../src/nav/cpp/planning/local/planner.hpp) 已分别提供 inflated、measured occupied、known-free；[DDS codec](../../src/nav/cpp/endpoint/nav/dds/codec.cpp) 分别接线。无 Body 的双圆柱读 inflated，显式 Body 读 measured，不能再给 Body 叠一次机体膨胀。未新增地图类型包装层。 |
| 非对齐原点会算错格子 | 成立。valid 接受任意有限原点，view/body 查询按相对原点；旧 Grid point/DDA 却用 global floor-round。本轮新增独立 bitmap 回归，统一到既有相对原点合同。当前 mapd 生成对齐原点，不能把这项直接说成现场 mapd 故障根因。 |
| Z clearance 上下方向反了 | 不成立。[native_navigation.py](../../src/lingtu/assembly/config/native_navigation.py) 将障碍向上扩展 below、向下扩展 above，求的是禁止机器人中心进入的位置；动态区间使用同一 Minkowski 关系，保留原方向。 |
| prediction 过期就应全部拒绝 | 不采用。预测是可选 overlay；过期后实测三维地图仍参与碰撞，地图新鲜度由现有输入门处理。已有 InvalidOptionalPredictionKeepsMeasuredObstacles 与过期回归，不增加另一项全局停车条件。 |
| count 没和 vector.size 对照，坐标系不明 | PredictionView 是 pointer+count；[创建者](../../src/nav/cpp/endpoint/nav/input/planner.cpp) 和 Executor/Task 都从同一容器配对 data/size，复制后重绑；planning-frame 合同与 map→odom 变换已有。不凭空添加不存在的 vector.size 访问或重复包装。 |
| 制动有效时间缺物理依据，没用实测速度 | 共同线性衰减 twist 的积分时间为 reaction+T/2，T 取各轴停止时间最大值；[最终命令门](../../src/nav/cpp/endpoint/nav/main.cpp) 还单独检查 odom measured twist。保留该模型；实际减速度、延迟仍需平台标定，代码不能证明实机一定符合模型。 |
| 所有查询都应合并为新 FootprintQuery | 采用共享底层几何，未新增包装层。pose/sweep、动态到达时刻和制动有不同输入职责；显式机体复用同一 PreparedPart/SAT，几何优化不合并其时间语义。 |
| DDA 没有角点处理 | 当前已有同时 crossing 的正/负方向分组和角点接触回归；原点修复继续保留，不另建 supercover 框架。 |
| unknown 必须恢复为整体拒绝 | 不采用。三态证据继续保存；未观测空气本身不构成实测机身障碍。支撑和覆盖范围仍独立证明，不能重引入已经复现的自遮挡起步死锁。 |

参数边界仍需精确描述：Product 的 `_finite_number` 已拒绝 NaN/Inf；直接 in-process
LocalPlannerParams 没有同等完整校验，原始 C++ 环境入口也未统一拒绝 Inf。
本轮未找到受支持 Product 生成这些非法几何参数的路径，因此不把它列为现场根因，
也不宣称库层所有非法输入已处理完整。这项若继续收紧，应在参数创建边界处理一次。

原点修复还保留了两次被否定的数值候选：先相减再除法造成 7 项回归；直接在归一化后
相减虽然已有套件全绿，仍把一个真实发布的 cell187 读成 cell188。最终
[GridAxis](../../src/nav/cpp/planning/local/grid_axis.hpp) 保留 mapd 的整数基准，只校正
非对齐原点的小数偏移。原始 bitmap 直接置位的负体素面测试避免了 fixture 同步变化
掩盖发布层差异，完整红期与计算顺序见错误记录。

几何内核的固定输入对比在 WSL x86_64、Release 下运行，每批 1,114,112 次查询，
7 批交替顺序并包含 prepare 成本。旧入口中位数 90.6143 ms，预计算复用 19.5815 ms，
命中数均为 3,504。四种形状的 136,080 个 cell 旧/新判定及接触豁免一致。
该数字只评价此内核，不外推整体规划速度或 NX 响应。

## URDF 是否已经用于障碍碰撞

**本次上游审查时，支持固定关节 URDF collision 部件与三维障碍体素碰撞，本地 Go2 当时仍默认 OBB。**
同日后续已将 Go2 real、Thunder real/sim 默认切换为平台组合模型，并接入水平余量与当前
机身姿态；最新选择、固定关节限制和验证见
[维护入口](../design/navigation-body-model-and-motion-posture.md#当前决定与实际状态)。

现有路径：

```text
URDF 的 collision 原语 + 指定关节角
    → 离线 FK / BODY2
    → RobotConfig / RunPlan 选择模型
    → 原生 Body
    → 全局路径 / SCAN / 最终制动对三维障碍检查
```

- [body_model.py](../../tools/bench/body_model.py) 去掉 visual，只接受 box/cylinder/sphere；
  当前拒绝 collision mesh，并非任意视觉网格精确碰撞。
- [body_export.py](../../tools/bench/body_export.py) 导出固定关节快照。
  [Go2 资产](../../config/robots/unitree/go2/body/go2_mid360_flatpack.json) 有 34 个部件，
  未完整覆盖外置 D435i/支架，installation_qualified=false。
- [build.cpp](../../src/nav/cpp/endpoint/nav/app/config/build.cpp) 的非空 BODY2 加载分支保留；
  [本地 Go2 配置](../../config/robots/unitree/go2/robot.yaml) 的 navigation_body_file 仍为空，
  选择 0.86×0.41 m、Z[-0.10,+0.24] 的 OBB。最后 `.97` 安装记录是旧 0.96×0.51 m；
  本轮未连接目标机，不用本地 YAML 推断现场。
- 原生 Body 支持 box/sphere/cylinder/capsule；只有 sphere/AABB 使用精确球距，
  其他形状走保守 OBB/AABB SAT。不能把保留圆柱类型解释为精确圆柱窄相已完成。
- 当前正式 navd 没有 JointState reader，创建 ExecutionObservation 时未填 body_geometry；
  最终 [main.cpp](../../src/nav/cpp/endpoint/nav/main.cpp) 也只传 executor_config.body。
  驱动已经发布电机 q/dq/力矩，缺口是订阅→FK→局部与最终的一致接线，不是重做驱动。
- 原始 manual 接管跳过该 SCAN 几何分支；导航、辅助遥控进入。网页画出 URDF 不证明该模式
  正在使用它避障。Thunder MuJoCo 的 measured body 快照组件也不能冒充 Go2 产品实测。

连续转弯特例 [poseRouteYawEnvelope](../../src/nav/cpp/navigation/pose_route.cpp) 目前仅支持
单 box，并生成水平正方形保守包络；分部 URDF 不会自动获得这条 blend 优化。
因此“所有阶段永远同一个形状”也是过度概括：模型来源共用，但某些转弯候选使用派生包络。

可靠推进应复用现有 Body/RobotConfig：补安装几何，接已发布 JointState 的 FK，同步最终
刹车检查；把当前关节姿态与未来行走扫掠分开验证。固定站姿不代表未来摆腿范围，当前 FK
也不会自动预测未来步态。无需为此再建一个模型管理框架。

## 当前姿态和交付边界

已有 80 cm L 形 URDF 回放存在长侧移、末端才转身，属于姿态失败，不能因为几何通过就关闭。
本次阅读时 [heading.hpp](../../src/nav/cpp/planning/global/octoplanner/heading.hpp) 已出现
forward_deviation_weight/translationCost 等并行调整；本文没有运行这条新姿态方案，
不再把旧文档的“无前向偏好”当作所有当前源码的事实，也不把新代码当成验收已通过。
需求与进度以 [机体模型和运动姿态](../design/navigation-body-model-and-motion-posture.md) 为入口。

本轮修改支撑 footprint、部件几何复用和快照坐标索引，新增正式回归并重新构建受影响目标。
最终本机 Release 结果：SCAN118、body27、Task23、core56、support28，独立几何与全局
BodyQuery CTest 2/2，contact 与遥控安全均通过。验证记录见
[错误记录](../operations/error_record.md)，不是复用上一轮绿灯代表本轮通过。
没有改 RobotConfig、服务、地图或急停，没有部署或下发运动。URDF 当前安装几何、实时
FK 和最终检查接线仍属于上面的未完成能力，不能因本轮几何回归通过而宣称已交付。
