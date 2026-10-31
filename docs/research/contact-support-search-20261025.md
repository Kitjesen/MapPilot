# 显式接触模型的中心支撑误拒：团队实施与交付

> 历史实验记录：2026-10-27 已从当前导航代码删除逐脚接地证明和承重门槛。本文的旧测试结果保留用于溯源，不是当前运行规则；当前实现见 [组合碰撞模型设计](../design/navigation-body-model-and-motion-posture.md)。

日期：2026-10-25。承接 [支撑与碰撞余量分离](support-margin-fix-20261025.md)。
本轮落实到原生全局规划，FDE的Linux与产品接线结果另见
[交付记录](contact-support-delivery-20261025.md)。

## 团队如何分工

| 职责 | 工作 | 已有结果 |
|---|---|---|
| 实施负责人 | 修改姿态搜索、快捷路线与公共运动约束查询 | 代码落地，Windows49/49、Linux50/50通过 |
| 资深导航审阅 | 检查剪枝必要条件、精确终点、转向与调用链 | 只读审阅通过，未增加新的通用查询层 |
| 测试工程 | 从独立小地图、四接触部件与真实runPlan构造反例 | 旧版中心空槽失败，新版正负例通过 |
| FDE工程 | 核查RobotConfig/RunPlan、Linux构建与发布边界 | 两平台默认接线已解析；Linux窄测50/50及正式navd构建通过 |

这些是工程职责分工，不代表真人行业专家或厂商认证。

## 本轮采用的研究逻辑

延续[原论文与固定上游源码审阅](traversability-and-footholds-20261025.md)，
ArtPlanner区分躯干避障与足部可达区域接触地形；这说明机身中心下方有地面
不能自动成为所有接触模型的必要条件。本轮只修正当前显式接触模型的这一额外门槛，
没有实现ArtPlanner的足部可达体积、学习运动代价或完整控制器能力模型。

实验直接从OctoMap占据/自由体素、接触部件位置与尺寸、原始起终点构造，调用
现有原生`runPlan()`；不从展示图片反推可行性。修改前运行同一反例，修改后同时
验证正例、沿途断口、精确终点与运动约束。独立审阅追查完整执行调用链，FDE再用
GCC和独立Linux目录构建；两种编译器的一致结果仍不等于独立物理真值。

## 问题与最小修法

此前路径即使每个指定接触部件都沿地面获得支撑，仍先被机身中心的地面检查否决。
这既存在于姿态搜索，也存在于固定朝向的快捷路线。

当前区分两种已有模型语义：

- **没有必须承重部件的Body**：保留原中心支撑与中心连通性预检。
- **存在support_contact且support_required的部件**：由完整BodyQuery检查这些
  部件的地面证据；中心不再额外要求地面。所有机体碰撞和沿途部件支撑仍检查。

`heading::Config::require_center_support` 只由本次请求的Body派生，默认true。
没有新增RobotConfig参数、环境开关或允许运营者跳过检查的选项。

实现位置：

1. `heading.hpp/cpp`：起点、精确目标、候选与终端连接采用一致语义。
   接触模式保留地图边界；中心连通图不再是其必要条件，因此不运行该预检。
2. `vendor/planner/.../global_planner.h/cpp`：增加薄的
   `metricConnectionWithinLimits()`，复用原步高/坡度公式；不复制数学规则。
   原`supportConnectionFree()`也复用它，默认行为不变。
3. `octoplanner3d_core.cpp`：快捷直线、引导路线和姿态搜索使用同一派生值。
   候选路线始终需要完整body pose/edge检查，原始起点与精确终点不能跳过。

没有引入足部轨迹、接触力或步态优化。它修复的是指定固定接触几何的路线准入。

## 修改前后的原始证据

测试工程直接建立0.05m OctoMap：中间沿x方向缺宽0.20m的地面，两侧保留地板。
四个球形接触部件位于机身坐标x=±0.15m、y=±0.25m，半径0.025m。
它是解析几何测试模型，不是Go2/Thunder的真实URDF或动态步态。

路径从(-0.8,0,0.35)到(0.8,0,0.35)。先独立确认BodyQuery沿途支撑与碰撞条件
允许，再把同一地图、Body和目标交给真实`runPlan()`。

- 旧版：回归程序失败，原始日志包含
  `center slot rejected a route with continuously supported contacts`。
- 新版：该请求和非晶格终点请求成功，逐点及逐边复查完整BodyQuery。
- 增加横向地面断口后，起终点仍可支持但中间边不可支持，整体拒绝。
- 把模型换成没有required contacts的普通OBB，仍以`start_pose_blocked`拒绝中心空槽。
- 单XY体素小地图先确认几何连续边允许，宽松metric可以成功；单独收紧最大步高
  或坡度后分别拒绝，避免把本来就碰撞的负例错当成运动约束验证。
- 直接heading测试验证接触语义下精确目标、终端yaw、真实边拒绝与地图边界，
  防止只改快捷路线而漏掉搜索分支。

Windows Release：**49/49个CTest检查通过**。包括25个支撑图测试、21个SCAN
Body测试，以及heading smoke/reference和全局BodyQuery smoke。
既有独立reference中的64例仍按原中心规则验证，没有为了新模式放宽旧oracle。

FDE在独立WSL Ubuntu 24.04、GCC13.3、Linux x86_64构建目录复跑相关检查：
**50/50通过**，比Windows上述集合额外包括`test_body_contact`。
Windows正式`navd.exe`与Linux正式`navd`均已完成编译链接。FDE确认Linux产物为
x86-64 ELF、动态库依赖全部可解析；原始日志与已有告警详见交付说明。
这些构建没有启动导航进程，也不替代NX aarch64或真实控制器验证。

日志中的toy路线查询耗时不能当作完整真实机器人的规划延迟指标。本轮未做
性能对照或重新运行物理闭环。

## 团队额外追查但没有添加代码的地方

曾怀疑pose blend的整机投影支撑会再次拒绝接触路线。完整调用链核查发现：
`poseRouteYawEnvelope()`当前只支持单个无contact标签的Box；required-contact Body
不能提交blend。普通请求也不传blend高度字段，因此该组合不会沿这条产品路径出现。

据此未增加Grid条件，也撤掉了基于错误caller假设的临时测试。当前接触模型继续
使用原有分段位姿和转向流程；圆弧blend支持没有在本轮扩大。

## FDE与现场边界

当前Go2的navigation_body_file为空，Thunder也采用空默认。两者正式SCAN仍构造
无接触标签的单yaw OBB，所以默认准入与运动模式没有因这次修订改变。
解析RunPlan也确认没有为它们写入LINGTU_NAV_BODY_FILE；原生最终运动仍是三轴速度。

显式固定接触模型能通过原生地图查询，不代表实际控制器会保持这些接触。
下一步应先做所选机器人/控制器的完整物理闭环；现场aarch64构建、无运动就绪与
监督运动资格仍要通过既有ProductControl和发布流程确认。没有部署或启动机器人。

## 复核与继续推进

- [本轮展示页](../../build/contact-support-20261025/index.html)
- [旧版失败日志](../../build/contact-support-20261025/red-test.log)
- [Windows测试结果](../../build/contact-support-20261025/green-test-windows.log)
- [Windows完整测试输出](../../build/contact-support-20261025/native-test-output-windows.log)
- [Windows正式导航构建](../../build/contact-support-20261025/navd-build-windows.log)
- [Linux独立测试结果](../../build/contact-support-20261025/fde/linux-tests.log)
- [Linux完整测试输出](../../build/contact-support-20261025/fde/linux-native-test-output.log)
- [Linux正式导航构建](../../build/contact-support-20261025/fde/build-navd-linux.log)
- [FDE接线与Linux记录](contact-support-delivery-20261025.md)
- [runPlan回归源](../../src/nav/cpp/planning/global/octoplanner/body_query_smoke.cpp)
- [直接heading回归源](../../src/nav/cpp/planning/global/octoplanner/heading_smoke.cpp)

后续工作保持聚焦：先验证接触模型的局部执行与当前控制器是否匹配，再决定是否
值得扩展圆弧或缩紧碰撞包络。精确落脚控制继续作为单独的能力升级。
