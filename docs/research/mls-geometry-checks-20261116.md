# MLS 全局路线的几何检查

核对日期为 2026-11-16。以下详细解释对应本轮开始时的源码和 20:55:46 构建的冻结 `headless`。Thunder 参数通过正式 RunPlan 纯解析取得。运行时未启动，未下发运动。

同日后续会话正在接入全局双圆柱和转身检查。源码已经出现 `body_offset_m`、`turnFree()` 和带 yaw 的扫掠接口，`applyPlannerGeometry()` 已开始传入前后偏移。本文的单圆柱描述属于冻结构建的行为记录，新实现需要随统一构建验收更新。冻结构建的首接入缺口复现见下文。

## 一条路线经过哪些检查

```mermaid
flowchart LR
    A[Rust MLS 候选路线] --> B[C++ 全局几何准入]
    B --> C[SCAN 局部轨迹与机身检查]
    C --> D[FinalControl 当前命令和制动距离检查]
    D --> E[Driver]
```

C++ 全局几何准入检查中心圆柱、支撑、高差、坡度、禁区和相邻点之间的连续占据碰撞。SCAN 继续检查有朝向的局部运动。FinalControl 根据实际速度和候选命令检查制动过程。

全局路线通过，说明它满足全局层声明的几何模型。路线交给局部层后还可能因机身长度、转弯朝向、实时障碍或制动空间被拒绝。

## Thunder 的当前正式参数

通过项目 `.venv` 直接调用 `compile_run_plan("nav", "real", robot="doso/thunder_v4", local_planner="scan")` 获取的环境值如下。

| 参数 | 当前值 | 在检查中的含义 |
| --- | --- | --- |
| 全局圆柱半径 | 0.32 m | 路线中心周围的水平圆形范围 |
| 机身参考点下方范围 | 0.05 m | 圆柱底面到机身参考点的距离 |
| 机身参考点上方范围 | 0.35 m | 机身参考点到圆柱顶面的距离 |
| 支撑高度 | 0.435 m | 地面表面与机身参考点之间的标称距离 |
| 支撑高度容差 | 0.10 m | 允许的离地高度差，另考虑体素的几何范围 |
| SCAN 双圆柱偏移 | 前后各 0.23 m | 圆柱中心沿机身方向偏移 |
| SCAN 双圆柱半径 | 0.32 m | 每个局部碰撞圆柱的水平半径 |
| SCAN 体素分辨率 | 0.05 m | 当前正式局部碰撞体素间距 |
| 最大连接高差 | 0.45 m | 单条检查连接的高度变化限制 |
| 最大坡度 | tan(30°)，约 0.577 | 连接高差与水平距离的比例限制 |

参数来源为 [robot.yaml](../../config/robots/doso/thunder_v4/robot.yaml) 和 [global.yaml](../../config/system/settings/global.yaml)。`support_height` 目前来自模型站姿估计，配置注释明确要求后续实测替换。

`native_navigation.py` 和 `applyPlannerGeometry()` 将上下范围按局部体素大小向上取整。当前默认 0.05 m 分辨率下，结果仍为下方 0.05 m、上方 0.35 m。更换分辨率后应读取新 RunPlan 和原生解析结果。

## 单点机身检查怎么做

把路线点视为机身参考点，在它周围放一个直立圆柱。水平半径 0.32 m，高度从参考点下方 0.05 m 延伸到上方 0.35 m。检查占据体素的整个体积是否与圆柱重叠。

这层模型没有前后方向，因此单点检查不使用 yaw，也不使用 URDF 的腿、关节、roll 或 pitch。它提供全局宽度走廊，局部双圆柱和可选机身几何继续承担有朝向的运动检查。

冻结构建的正常全局查询半径没有加上前后 0.23 m 偏移。局部双圆柱总前后范围为 2 × (0.23 + 0.32) = 1.10 m，全局圆柱直径为 0.64 m。两者的前后覆盖范围不同。

此设计在 `native_navigation.py` 的注释中明确表达，避免直走廊处处要求原地转身空间。它也意味着冻结构建的全局路线通过后，转角或终点姿态仍可能被局部层拒绝。全局请求没有传入完整 `Body` 做第二次全路线姿态检查。

源码入口为 `queryWorld()`、`applyPlannerGeometry()`、`compile_native_nav_config()` 和 `GoalPlanController::startPlanning()`。

## 两点之间怎样检查

`sweptBodyFree()` 将这个直立圆柱从前一点平移到后一点，与沿途占据体素盒做连续相交判断。当前实现覆盖整段平移轨迹。

`sweptRouteFree()` 另外以最多半个地图体素的间距检查支撑和临时阻挡。禁区使用完整线段与膨胀后的三维区域求交。

楼梯段的地面高度是离散变化的。中间点的线性插值高度若不满足标称离地高度，代码会在两端高度之间寻找一个有支撑的站姿，同时保留实际插值轨迹的机身占据扫掠检查。这里没有求解落脚点和步态。

`metricConnectionWithinLimits()` 通常限制单条连接的高差和坡度。如果两端落在同一已观察地面上，也允许机身参考点采用不同的合法离地高度，避免把站姿高度变化误判为地形坡度。

## 支撑和未知区域的实际含义

当前正式配置允许局部补面产生的 inferred surface 作为地面支撑。原始占据证据与补面证据保持区分。

正常物理姿态查询主要检查路线点所在 XY 列下方的地面。`support_patch_radius_cells=0` 和 `support_patch_min_samples=0` 关闭额外支撑采样要求。源码中存在的五点 `hasFootprintGroundSupport()` 不属于这条正常 `queryWorld()` 调用路径。因此这里尚未验证四足的具体落脚和承载面积。

机身空间中的未知体素允许通过，已观察占据体素会导致碰撞拒绝。局部完整机身查询要求模型处于碰撞网格覆盖范围内，同时其碰撞判断仍主要读取 measured occupied 数据。是否观察过自由空间和是否碰撞，是当前代码中的不同证据。

## 起点接入需要继续处理的范围

`isStartConnectionAllowed()` 为实际起点跳过首点的机身和支撑拒绝，避免机器人自身残留点云或不完整起点地面阻塞启动。保存地图中的占据体素并未被删除。

冻结构建对实际起点到图接入点的整条首连接关闭中间支撑检查。连接目标仍要求支撑，沿途机身占据扫掠也保留。

已用正式 `headless` 的 `makePlan()` 复现该缺口。地面左岛覆盖 x 从 −0.6 m 到 0.6 m，右岛覆盖 x 从 1.4 m 到 3.6 m，中间缺地 0.8 m。起点为 `(0.05, 0.05, 0.485)`，终点为 `(2.25, 0.05, 0.485)`。起点岛能容纳配置的双圆柱覆盖范围。关闭补面，要求直接已观察地面支撑，使用上述 Thunder 几何参数和 24 cell 吸附范围，仍返回跨坑的两点直线。

返回结果为 `ok=true`、`used_rust_graph=true`、`rust_fallback=false`。Rust 正常路径还在返回前调用 `validateRoute()`，这一复核也沿用同一首连接豁免。复现输入和输出位于 `build/mls-start-gap-20261116/request.json`、`map.ot`、`result.json` 和 `headless.log`。

`grid_smoke.cpp` 已增加 `supportedStartCannotAttachAcrossMissingGround()`。它要求跨坑输入失败，补齐同一地面后成功，并单独检查首段 `validateRoute()`。后续统一构建需要验证修复后的结果。

## 局部和最终命令检查

`FinalControl::evaluateFinalSafety()` 创建同一个 SCAN `Grid`，检查实测速度对应的制动范围，并为候选命令求可接受比例。阻挡时产生 `brake_blocked` 或 `cmd_blocked`，部分速度允许时产生 `cmd_limited`。

默认局部模型使用前后双圆柱。若调用方传入完整 `Body`，`Grid` 会使用模型中的 box、sphere、cylinder、capsule 部件检查占据，实测 roll/pitch 通过 `observedBody()` 应用于部件变换。当前正式 Thunder 配置构建没有给 Executor 装载完整 `Body`，其常规运行仍采用配置的双圆柱。

局部层承担实时姿态和制动检查具有明确用途。全局层还需要用匹配的转角、终点朝向案例验证，避免持续给出局部无法跟随的参考路线。

## 楼梯和多层测试通过证明什么

当前离线用例可以证明指定保存地图、输入端点和参数下，Rust 能找到路线，路线通过上述全局圆柱、支撑与连接检查。它们尚未包含 SCAN 轨迹、Follower、Driver、MuJoCo 动力学和目标硬件反馈。

DimOS 同图比较需要分别记录官方原始路线、LingTu 全局几何复核和实际执行结果。DimOS 与 LingTu 使用的支撑高度约定、机身模型和检查职责存在差异。某条路线被 LingTu 拒绝，应继续定位具体体素、姿态和模型误差，再判断它属于真实碰撞还是建模过严。拒绝次数本身不足以评判导航质量。

## 源码定位

- [全局几何、支撑与连接实现](../../src/nav/cpp/planning/global/octoplanner/vendor/planner/src/global_planner.cpp)
- [连续圆柱与三维区域求交](../../src/maps/include/lingtu/maps/no_go.hpp)
- [正式配置解析和全局几何映射](../../src/lingtu/assembly/config/native_navigation.py)
- [原生配置的几何取整](../../src/nav/cpp/endpoint/nav/app/config/build.cpp)
- [全局任务与局部机身信息](../../src/nav/cpp/endpoint/nav/app/goal/plan.cpp)
- [SCAN 局部机身与扫掠查询](../../src/nav/cpp/planning/local/scan/grid.cpp)
- [实测姿态应用到部件模型](../../src/nav/cpp/endpoint/nav/input/pose.cpp)
- [最终速度与制动检查](../../src/nav/cpp/endpoint/nav/control/final.cpp)

## 双圆柱新构建的楼梯诊断

同日 `build/mls-unified-20261116/current-dual/report.json` 对应的新构建中，楼梯与多层输入通过 XYZ 回退取得路线。将这些原始 `route.xyz` 直接交给同版本 Rust `checkMetricRoute()` 后，两条路线都通过，C++ `validateRoute()` 也通过。

保持 XY 和地图不变，仅将中间路线点的 Z 调整到最近 MLS surface 中心加 0.435 m，两条路线都发生几何失败。

| 输入 | 原始路线 | 调整后的首失败 | 具体变化 |
| --- | --- | --- | --- |
| stairs | C++ 与 Rust 均通过 | 第 10 段，occupied | 第 11 点 Z 从约 0.735 m 降到 0.635 m |
| multifloor | C++ 与 Rust 均通过 | 第 33 段，slope | 第 33 点降低后，下一段 0.2 m 水平距离升高 0.2 m，超过 0.577 坡度 |

编号从零开始。

楼梯的占据冲突来自前圆柱与台阶体素。第 11 点调整后，圆柱底面从 0.685 m 降到 0.585 m，而前方占据体素顶面为 0.6 m，产生 1.5 cm 竖直重叠。相交占据体素为 `(17, -2…1, 2)`，其 x 范围为 3.4…3.6 m，z 范围为 0.4…0.6 m。保持原姿态高度时，这些体素与圆柱分离。

这证明地面 surface 和机身可行姿态之间需要选择有效高度，固定加标称高度会丢失一条已验证可行路线。仍需通过更新后的 Rust 查询实验，确定补足姿态后能否消除这两个输入的回退。

诊断源码和完整结果位于 `build/mls-dual-diagnostic-20261116/probe.cpp`、`run.py`、`stairs.log` 和 `multifloor.log`。此实验只检查已保存静态路线，未执行局部轨迹和机器人运动。
