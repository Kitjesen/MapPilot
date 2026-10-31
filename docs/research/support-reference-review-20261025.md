# 轮足支撑的定义：公开实现拆解与 LingTu 修订依据

审阅日期：2026-10-25。范围：支撑、机体净空、接触模式及其在规划中的职责。
本轮完成公开源代码与论文核查、独立本地审阅、实验解释修订；没有声称复现外部
项目的性能，也没有将新判据部署到导航。研究代码保存在 build 下，不引入产品依赖。

后续进一步澄清：[可穿越性与足部规划的变量、输出、接入条件](traversability-and-footholds-20261025.md)。
ArtPlanner、感知NMPC与TAMOLS处于不同规划层级，不是同一接口下的三个替代导航器。

## 直接判断

“其他导航没有讨论这个问题”不成立。腿足领域以 foothold selection、
reachability、contact planning、negative obstacles、terrain-aware locomotion
讨论它。普通平面导航通常把承重交给平地假设；不能以其没有落脚检查推出我们也不需要。

当前 LingTu 的问题不是缺少更多检查，而是两个检查的对象定义不准确：

1. 用机身中心列有地面，替代轮足能获得支撑。
2. 用膨胀碰撞包络的完整 XY 投影，替代实际轮足接触区域。

这两条可以同时过于保守，又不足以证明动态稳定。支撑不是一个“是否有占据体素”
的布尔值就能涵盖的物理事实。必须明确当前回答的是候选落脚、指定接触是否有效，
还是整段运动可执行。

## 50 cm 纵向槽究竟是什么

这是人工合成的初始化负例，不是真实地图或导航路线。`scene()` 建立两块厚地板：
左侧 y∈[-1.2,-0.25]，右侧 y∈[0.25,1.2]，x∈[-1.5,3]，地面 z=0。
中间宽 0.50 m 的长带没有地板，方向平行于前进 x；实验没有在槽底另建接住机器人的地板。
“50 cm”是横向宽度，不是深度。

机器人初始放在这条空带上方，用既有策略尝试站立；没有从地板驶近沟槽、选择落脚、
跨过去的动作。该初始构型未建立有效支撑，在预站立阶段掉落。因此可得出的结论是
“该初始构型与策略没有在此地形站住”，不能推导出任意机器人都跨不过 50 cm 的沟。

10/30 cm 槽的意义不同：已有记录中四轮在两侧承重，机身中间下方仍为空。
它们反驳中心支撑的必要性。50 cm 例只能作为不能把真实浮空误判为承重的负例。
旧 `hold_completed` 是实验判定遗漏自由下落，已更正，原始记录没有覆盖掉。

## 第一优先参考：ETH ArtPlanner

来源：[官方仓库](https://github.com/leggedrobotics/art_planner)、
[Field Robotics 2023 论文与出版记录](https://www.research-collection.ethz.ch/entities/publication/d9ba1c65-002c-4ec2-a145-044a84281fa4)。
该论文介绍 CERBERUS 在 DARPA SubT Finals 使用的导航器，有野外部署背景，适合作为
工程参考；不等于其中每个假设都适合 LingTu。

本次逐函数读取的版本：`2f3fde68b8ac55f6f91083acc4d09df2b10f3c1f`。

| 实现位置 | 真实逻辑 | 对 LingTu 的意义 |
|---|---|---|
| `validity_checker.cpp::StateValidityChecker::isValid` | 分别调用 torso 和 feet 检查 | 机体无碰撞与支撑可达不是同一种条件 |
| `validity_checker_body.cpp::ValidityCheckerBody::isValid` | 躯干盒体不得与高度场相交 | 保持明确机体净空 |
| `validity_checker_feet.cpp::ValidityCheckerFeet::isValid` | 四个足部可达盒各自必须与可落脚高度场相交 | 找每条腿可落脚的位置，不要求机腹下面全是地面 |
| `height_map_box_checker.cpp::checkCollision` | ODE 盒体/高度场相交查询 | feet 中“相交”是找到可达地面，body 中“相交”是障碍，语义相反 |
| `map/processors/basic.cpp::setMaskedElevationAndTraversability` | 按可落脚区域大小及边缘风险处理；区分落差和高墙 | 不是把避障膨胀直接变成地面覆盖要求 |

最直接对应本轮临墙误拒的是
[basic.cpp](https://github.com/leggedrobotics/art_planner/blob/2f3fde68b8ac55f6f91083acc4d09df2b10f3c1f/art_planner/src/map/processors/basic.cpp)：
代码单独构造向下落差、向上墙面掩码，处理落脚边缘时避免沿墙一概侵蚀，最终仍保留
原本不可落脚区域。**墙边不等于悬崖边**，躯干避障仍由独立检查负责。

这里有一个根本的量词区别：

- 当前中心条件：机器人参考点下方必须有地面。
- 当前轮足条件：膨胀投影的每一列都必须是同高支撑。
- ArtPlanner 可达抽象：每条腿的可达区域内存在合格地面接触候选。

最后一种只是候选位姿的可达近似，不自动证明四足组合的力学可行、摆腿无碰撞或动态
稳定。不能把它改写成“随便找到一个占据点就能走”。

不直接移植的部分：它采用 2.5D 高度图，包含插值/填补步骤和专用未知区处理；
其学习运动代价具有 ANYmal 训练分布。LingTu 的多层 OctoMap、ROS-free 边界、
未知空间证据和所选 RobotConfig 应保留。借鉴判定分工，不把整套 ROS/高度图栈搬进来。

## 第二优先参考：可落脚区域与接触相位

[elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)
历史 main 版本 `20a8a26b67a995b43eb44c23568854d1fed82a52` 的
`convex_plane_decomposition`，及
[OCS2](https://github.com/leggedrobotics/ocs2)
`26386754b8bf31ab78b503971d0cbf4fdbcd7cb4` 的 perceptive ANYmal 示例，
提供另一条可复核的分工：从局部平面边界生成可落脚区域，再对每只脚限制其落点；
碰撞距离和接触阶段另行处理。

源码审阅发现：轮廓区域可以带孔，落脚边界向内缩；局部足端位置关联到凸可落脚区域；
碰撞代价独立使用 SDF 距离减碰撞球半径。摆动足与支撑足的处理不同，
并非所有足端在所有时刻都要求接地。

固定版本的直接入口：

- [ContourExtraction::extractPlanarRegions](https://github.com/leggedrobotics/elevation_mapping_cupy/blob/20a8a26b67a995b43eb44c23568854d1fed82a52/plane_segmentation/convex_plane_decomposition/src/contour_extraction/ContourExtraction.cpp#L14-L46)：落脚区域侵蚀。
- [SegmentedPlanesTerrainModel](https://github.com/leggedrobotics/ocs2/blob/26386754b8bf31ab78b503971d0cbf4fdbcd7cb4/ocs2_robotic_examples/ocs2_perceptive_anymal/segmented_planes_terrain_model/src/SegmentedPlanesTerrainModel.cpp#L35-L57)：候选足点的局部凸区域。
- [FootPlacementCost](https://github.com/leggedrobotics/ocs2/blob/26386754b8bf31ab78b503971d0cbf4fdbcd7cb4/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_switched_model_interface/src/cost/FootPlacementCost.cpp#L21-L39)：足点区域罚函数。
- [CollisionAvoidanceCost](https://github.com/leggedrobotics/ocs2/blob/26386754b8bf31ab78b503971d0cbf4fdbcd7cb4/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_switched_model_interface/src/cost/CollisionAvoidanceCost.cpp#L21-L43)：独立碰撞罚函数。
- [SwitchedModelPrecomputation](https://github.com/leggedrobotics/ocs2/blob/26386754b8bf31ab78b503971d0cbf4fdbcd7cb4/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_switched_model_interface/src/core/SwitchedModelPrecomputation.cpp#L179-L211)：按接触相位处理足端。

对应论文：[Perceptive Locomotion through Nonlinear Model Predictive Control](https://arxiv.org/html/2208.08373)，
TRO 2023，DOI [10.1109/TRO.2023.3275384](https://doi.org/10.1109/TRO.2023.3275384)。
本轮阅读的是 arXiv v1，并核对最终发表元数据，未逐项比较最终期刊版。
它包含跨缝与踏石场景。可落脚区域、摆动避障、接触约束有明确职责；这是我们缺少的
物理语义参考，而非要求立即引入一套新的 MPC。

版本注意：上述 plane segmentation 是历史算法参考，不能宣传为当前 ROS2/Jazzy
版本开箱即用的全部能力。局部 2.5D 平面处理也不能替代我们的全局多层 3D 地图。

## 运动层参考：TAMOLS，而非把它塞进全局搜索

[TAMOLS 原论文 v2](https://arxiv.org/html/2206.14049v2) 联合处理机身位姿和落脚，
区分触地位置、可达范围、摩擦与动态稳定。它说明地面支撑应服务于接触和运动，
不是要求整块身体投影铺满地面。

但论文也报告局限：简化动力学的可行保证依赖水平接触面；未采用完整运动学，
膝部碰撞仍可能出现；地图漂移会使实际落点偏离。论文的跨缝实验也不是零失误。
不能借“TAMOLS”之名把启发式平滑地面当作真实支撑，更不能把其计算耗时当作
我们的性能承诺。本轮未找到并验证原作者完整控制流水线的可运行开源版本，
不把第三方 TAMOLS-inspired 项目冒充原实现。

## 产品接口参考：Spot 确实区分这些问题

已读取官方 [robot_command.proto](https://github.com/boston-dynamics/spot-sdk/blob/04537ddafe0641d1751af3eb9e76cb785fc18594/protos/bosdyn/api/spot/robot_command.proto)：
`ObstacleParams` 分开声明足部障碍、足部边缘约束、机体避障、机体 padding、负障碍；
`TerrainParams` 则有摩擦提示及格栅地面处理。

这只能证明公开产品接口将这些职责分开，不能反推出闭源落脚算法、具体阈值或
稳定性保证。格栅模式用于特定重复表面的感知处理，绝不等于允许补平任意坑洞。

## 更新近况：2026 年仍在研究

[RB-TRG 2026 预印本](https://arxiv.org/html/2608.16433v1) 专门研究轮足机器人
朝向、转弯时部分支撑丢失及机体/地形干涉，公开了代码链接。它采用有向机体采样
和风险代价，说明问题并未被忽略。风险代价不是接触稳定证明；本轮仅核查论文，
未运行或逐函数逆向其实现，不纳入已经验证的外部基线。

## 独立本地审阅进一步指出的三处关联

### 中心预检必须与最终准入条件相容

`heading.cpp:92` 起点先过 `poseSupportFree()`，再过完整 BodyQuery；候选和
支撑连通性预检也使用中心条件。因此只修 `support.hpp` 不能修复纵向槽的全局拒绝。

如果最终允许两侧轮足承重而中心悬空，那么便宜的前置判定必须也允许这些候选。
否则前置判定会提前删掉正确解。此前“支撑不连通”的结论只对原有强中心规则下的
搜索图成立，不等于所有实际可执行构型都不连通。

### 扫掠余量不是新增承重面积

`BodyQuery::edge()` 用连续运动采样间距产生额外 padding，传到 `pose()`；
后者把它与 body margin 相加，再传给支撑函数。用于防止漏查连续碰撞的余量，
于是变成额外必须有地面的区域。两种物理含义应拆开。

### 60 cm 通道的成功条件必须说准确

修掉支撑误拒后，当前保守 OBB 加余量仍可能触墙，并返回 `occupied_body`。
首先要做到“正确识别承重，并准确区分净空拒绝”，不能承诺仅改支撑就一定放行。
如需缩紧圆柱包络，应保持原有几何余量，与解析圆柱/墙距离另作同条件对照。

## 深入修订应补什么，在哪里补

| 要补的内容 | 主要位置 | 最小可验收结果 |
|---|---|---|
| 接触区域的几何含义 | `collision/support.hpp`、`body.cpp` | 使用指定表面附近的实际接触几何；碰撞膨胀不扩张必需承重面积 |
| 支撑与净空分别判定 | `body_query.cpp`、`planning/local/scan/grid.cpp` | 临墙轮足不再把净空原因报成缺支撑；上方障碍仍检测 |
| 中心规则与完整机体范围相容 | `heading.cpp`、中心支撑查询 | 最终可接受的两侧承重候选不会在入口被中心空洞删除 |
| 控制器的接触模式依据 | 所选 RobotConfig/控制器及现有状态消息 | 区分已测承重、预测承重、摆动；没有控制器能力就不声明能跨越 |
| 正确实验结束条件 | `support_gait_probe.py` | 站立建立失败、接触规则中止、完成运动分开记录 |

不要用“投影覆盖 70%”这样的无依据比例替代接触模型。对轮子，应讨论接触带、
滚动方向与承重；对球足，应讨论底部接触和落脚余量。完整稳定性由接触组合及
实际控制器能力决定，不应在每个全局候选里增加一套全身优化。

保留同一组三维地形、机器人尺寸、margin、控制器和记录姿态，先离线比较两种判定：
纵向槽承重、贴墙轮足、真正浮空、墙侧面、不同层楼板和横向断口。验收应同时看
误拒、误放行和拒绝原因，不能只追求通过率。通过后再跑原生全局→SCAN→Executor
的同路线物理闭环，并测规划耗时；当前 10 例没有完成该闭环。

## 仍未被证实的内容

- 每脚存在候选落点，不等于整组接触力可实现、动态平衡或摆腿无碰撞。
- 实测回放用物理真值设置承重脚，是诊断材料，不是未来接触预测算法。
- `normal.z>=0.98` 对应约 11.48°接触法向，不能判定所有 30°坡或轮缘接触是否可行。
- Thunder 四轮滚动不等于 Go2 交替迈腿；50 cm 槽初始化失败不等于跨沟能力极限。
- 外部项目在自己的机器人/地图/控制器上成功，不等于本仓库已经达到同样能力。

## 复核入口

- [本地实验与原始记录](support-gait-20261025.md)
- [原有连通性验证的前提](navigation-evidence-20261025.md)
- 本地公开代码快照：`build/support-reference-research-20261025/art_planner/`
- 来源版本表：`build/support-reference-research-20261025/sources.json`
- 独立参考审阅：`build/support-reference-research-20261025/source-notes.md`
- 本轮未安装新依赖，未运行外部项目，未改生产导航阈值，未部署或推送。
