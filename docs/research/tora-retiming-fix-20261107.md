# Tora 移动重定时修复研究

2026-11-07。生产实现采用有界的原曲线整段重拟合，保持原控制点数 N，让新 h 随总时长增加，随后硬置原首尾 PVA、重建当前 fit 的 reference，并按需执行原目标的约束 refine。保 h、增 N 是此前的中间验证原型，归档四例原生实验曾通过原物理 `1e-9` 容差检查，后续三维旧回归暴露了该原型的固定 Z 不可行输入。最终构建与 171 个不同测试已通过，同车库完整组件对照中，严格自交从 2 次降为 0，沿弦回退指标全部为 0。[证据与重放入口](../../artifacts/nav/tora-retime-fix-20261107/README.md)

## 问题与基线

前一轮分阶段记录在 attempt 22、23、50、66 的本地移动边界重定时 refine 输出首次观察到自交，前面的 seed、拟合与 rebound 保持无环。图与实际采用轨迹匹配沿用先前记录。[分阶段诊断](../../artifacts/nav/tora-curve-upstream-20261106/README.md) · [车库研究](tora-garage-navigation-20261106.md)

新增真实 manager 回归使用三例确定性 begin 输入、完整 194 点参考折线和有效空图。原生产实现中三次重规划均成功、起点 PVA 与终点位置连续、动态界限通过，但 22、50 严格自交，三例沿弦回退分别为 0.436645、0.130774、0.493522 m。修改前回归退出码 1。[实际结果转录](../../artifacts/nav/tora-retime-fix-20261107/evidence/baseline-regression.json)

同车库 before 的 77.85 s 组件运行已到达终点，41 条局部轨迹中 12、25 各有一次严格自交，均位于未执行尾段。对应 after 完整运行与分析已完成，结果列于下文。[before 完整分析](../../artifacts/nav/tora-retime-fix-20261107/evidence/before-full.json)

## 两个独立判断

保 h 量级增加采样段数后，原 soft cost、norm soft cost、降低末速与循环投影候选仍未满足动态界，均被淘汰。22、50 的有界 SLSQP 探针固定同 h/N、首尾 PVA 和全部 Z，找到硬约束可行的无环曲线。第二组保留原 smooth + fitness 目标也成功。两组结果支持该时间边界下存在可行解，以及受约束原目标可以产生无环解。[两组 SLSQP](../../artifacts/nav/tora-retime-fix-20261107/evidence/constrained-refine-summary.json)

第一组 attempt 50 返回 SLSQP status 8，但返回点加速度约为 0.500000000188607 m/s²，在声明的 1e-8 数值容差下可行。第二组两例均正常收敛。离线 SLSQP 为可行性见证，实际实现继续使用 C++ 原生求解路径。

## 三维旧回归带来的修正

后续真实旧回归中，保 h、增 N 原型的三维起步 fit 出现约 0.847 m/s² 的最大 |az|，超过 0.5 m/s² 加速度 norm 上限。约束 refine 固定 Z、仅优化 XY，无法消除该违反量。平地固定输入的可行性结论只适用于其对应 fit，无法直接推广到有非零 Z 运动的输入。

当前 manager 因此在最多 16 个时间候选中，每次从同一原 rebound 执行整曲线 `reparamBspline`，保持原 N，允许 h 随新总时长增加。整条 XYZ 曲线先完成重新采样与拟合，再硬置原首尾 PVA。固定 Z 的速度、加速度可行时，再使用当前 fit 的 reference 执行约束 refine。失败候选只向下一候选提供剩余时间倍率，几何重新取自原 rebound。归档中间原型源码与 JSON 保留原记录。

## 上游方向分支修正

原生约束实验出现 L-BFGS `-1008` 时，instrumentation 捕获 attempt 22 的 trial 越出 bracket。该 fallback 继承 SCAN 的 `a < 0` 判断，原始 liblbfgs 使用 `d > 0`。父代理已在线核对固定提交 `5ad02fbefefdeff339ab03635e673571055a0644`，两份官方源均已存档。SCAN 固定提交为 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。[版本与差异](../../artifacts/nav/tora-retime-fix-20261107/evidence/upstream-pins.json) · [固定 liblbfgs 原文](https://raw.githubusercontent.com/chokkan/liblbfgs/5ad02fbefefdeff339ab03635e673571055a0644/lib/lbfgs.c)

只改方向判断后，同输入、同 ALM 日程完成有界循环，原 `-1008` 消失。该单因素结果解释线搜索失败，同时后续仍需解决动态收敛误差。[宏错误日志](../../artifacts/nav/tora-retime-fix-20261107/evidence/attempt22-alm-linesearch.log) · [单因素日志](../../artifacts/nav/tora-retime-fix-20261107/evidence/attempt22-alm-direction.log)

后续短侧移回归又捕获一次目标停滞的数值终止问题。物理界限已满足，但 line search 用完 40 次预算，目标下降约 8e-17、方向导数约 1.2e-18，返回 `-1008`。约束 refine 启用标准 `past=3`、`delta=1e-8` 后，同输入返回 STOP=1，随后通过独立物理与碰撞检查，最大加速度 0.499999988981950 m/s²。基线、单因素日志和指标均已归档。[基线](../../artifacts/nav/tora-retime-fix-20261107/evidence/flat-trace-baseline.log) · [past3 日志](../../artifacts/nav/tora-retime-fix-20261107/evidence/flat-trace-past3.log) · [指标](../../artifacts/nav/tora-retime-fix-20261107/evidence/flat-trace-metrics.json)

## 采用策略

- 最多 16 个时间候选，每次从原 rebound 执行整曲线 `reparamBspline`，保持原段数和 N，由实际新总时长计算新 h。
- `lengthenTime` 仅扩展中部 `T - 4h`，使用中部倍率 `1 + (ratio - 1) × T / (T - 4h)`，使实际总时长达到 `ratio × T`。
- 全曲线拟合后硬置原首尾 PVA。固定 Z 的速度与加速度可行时，从当前 fit 重建 reference 执行约束 refine。下一候选仅继承剩余时间倍率，重新拟合原 rebound。
- 原 refine smoothness、fitness 和既有 soft feasibility 成本保留，外层增广拉格朗日法加入动态约束，内层复用原生 L-BFGS。
- 约束 refine 阶段固定首尾各三个控制点与当前 fit 的全部 Z，首尾 PVA 与输入末速保持一致。
- 对涉及可调内部控制点的平方约束目标采用相对 `1e-6` 数值余量，纯固定边界约束保留原目标。
- 内层使用 L-BFGS 标准目标停滞判断 `past=3`、`delta=1e-8`。
- 最终独立检查原物理 time ratio `≤ 1 + 1e-9`，维持原速度、body-axis 和加速度上限，并执行已有碰撞检查。

每次约束 refine 最多 16 外层、每个内层最多 200 迭代。归档中保 h、增 N 原型的四例固定输入最终加速度最大值均低于 0.5 m/s²，4001 样本无严格 XY 自交与参考逆投影，首尾 PVA 误差低于 4e-14。记录中的 9.3–65.2 ms 包括日志开销，这些数据对应中间原型，平台运行时延仍待对应平台验收。[四例原生结果](../../artifacts/nav/tora-retime-fix-20261107/evidence/alm-margin-analysis.json)

## 同车库完整对照

两次运行采用相同车库场景，`environment.json`、`navigation.yaml`、`resolved-config.json` 逐文本一致，全局 XYZ 路线数组完全一致。逐条检查实际局部 trajectory ID 的第一份完整 preview，严格自交和沿各自起终弦的回退指标如下。[完整比较数据](../../artifacts/nav/tora-retime-fix-20261107/comparison.json)

| 指标 | 修改前 | 修改后 |
| --- | --- | --- |
| 到达目标 | 是 | 是 |
| 局部轨迹 ID 数 | 41 | 45 |
| 严格自交 | 2，ID 12、25，均为未执行尾段 | 0 |
| 沿弦回退超过 1 cm 的 preview | 5 | 0 |
| 最大沿弦回退 | 0.485676911 m | 0 m |
| 组件观察时长 | 77.85 s | 61.22 s |
| 行进距离 | 52.412191 m | 52.436897 m |
| 最大跟踪误差 | 0.002456017 m | 0.002658527 m |
| collision / invalid query / blocked / limited | 全为 0 | 全为 0 |

对照图选取同弯道旧 ID 25 与新 ID 28，机器人位置相差约 0.102 m，两条曲线的初态不同。77.85→61.22 s 记录这两次理想平面反馈组件运行的观察时长。初次规划 CPU 负载有差异，该时间不用于推断算法 CPU 收益或实机提速。[曲线与完整运行对照图](../../artifacts/nav/tora-retime-fix-20261107/curve-comparison.png)

## 验收状态

后续[边界与场景复查](../../artifacts/nav/tora-boundaries-20261107/README.md)补测了
旋转／不等轴速度、贴限三维起步、返程、L 形转弯和身后目标，并修复固定机身
速度越界时的无效重算，以及首次执行确认、反馈采样的两处时钟不一致。
返程观察到约 21 mm 的交接误差后，定点观测确认旧规划器相位与跟踪器相差一个
控制步，候选 PVA 与接收 age 均正确。随后让规划器使用跟踪器的实际进度，最终
返程峰值降至 2.593 mm，首次交接误差保持约 2.12 mm，45 条完整轨迹零自交。
最终完整回归通过 SCAN 160 项、Task 29 项和 Follower 66 项；Executor 为
104/129。另 25 项在独立构建的本轮前版本全部同样失败，断言行集合一致，
详见边界复查的逐例对照。不能由这些通过项宣称整个导航测试全部通过。
该次独立复查的计数和局限见链接，未混入下述首轮计数。

- [x] 最终生产构建通过，完整 SCAN 154 项、refine solver 3 项、elevation 10 项、spline velocity bounds 4 项，共 171 个不同测试全部通过。17 项定向检查已包含在完整集合中。日志与 XML 已归档。[验收索引](../../artifacts/nav/tora-retime-fix-20261107/evidence/production-verification.json) · [构建日志](../../artifacts/nav/tora-retime-fix-20261107/evidence/final-build.log)
- [x] 同车库 before/after 的完整曲线自交和时间对照已归档。[比较数据](../../artifacts/nav/tora-retime-fix-20261107/comparison.json) · [after summary](../../artifacts/nav/tora-garage-complex-20261106/runs/retime-after-20261107/summary.json)
- [ ] 原生 MuJoCo、现场无运动和监督运动新增证据。
