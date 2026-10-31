# SCAN 全函数目录与代码审阅

这份记录保留修复前的审阅与目录统计。已完成的修复、当前代码目录和验证结果见
[修复记录](scan-fixes-20261025.md)，原理讲解见[读懂SCAN](scan-explained-20261025.md)。
下面的隔离反例针对当时的接口/实现；当前回归请使用修复记录所列新测试源。

日期：2026-10-25。范围为当前 LingTu 原生 SCAN 移植，不等于原上游仓库所有节点。
上游固定版本为 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。

本轮是审阅与定向诊断：没有修改生产算法、部署、启动导航或触发机器人运动。

## 结论与修订优先级

没有在本轮确认新的 P0/P1 运动安全放行缺陷。确认下列 P2 问题；不能由此推导
整个导航没有其他缺陷，也不能将源码审阅当作物理或现场验证。

| 顺序 | 问题 | 受影响路径 | 本轮证据 |
|---|---|---|---|
| 1 | 局部初始化把中间高程改成起终点线性高度，优化又固定 Z | 需要保留坡顶/高平台等非单调高程的局部路线 | 两个审阅者分别追查完整调用，未运行新闭环 |
| 2 | Refine 忽略求解错误码，未同步回滚后的控制点 | 正式 retiming/refine | 求解器与回调源码链；未强制触发求解失败 |
| 3 | A* 降低已入队节点的 fScore 后不调整堆 | 障碍引导路径搜索 | 实际节点/比较器的独立队列复现；未测完整路线影响 |
| 4 | 失败 JSON 未保存 Body/raw evidence/可选支撑高度等完整输入 | 正式 Body 与 pose-blend 失败记录 | 快照结构、捕获和序列化源码链 |

最终轨迹碰撞、速度/加速度验收及实时运动仲裁仍存在。上表不能解释成直接绕过它们。

## getMeanVel 到底做什么

定义在 [polynomial_traj.h:201](../../src/nav/cpp/planning/local/scan/upstream/traj_utils/polynomial_traj.h#L201)。
它的原意是多项式轨迹的平均**速率**：

`mean_speed = sampled_path_length / total_duration`，单位 m/s。

它不是速度向量平均，也不是速度上限、控制速度、实测里程计速度或制动参数。
先通过 `init()` 汇总时长，通过 `getTraj()` 以 10 ms 采样轨迹，再由 `getLength()`
累加相邻样本距离，最后才有可用的 length/time_sum。

当前实现只赋值局部变量，没有 return。执行到末尾将产生未定义行为。
检索当前 src/nav、tests/nav、tools、sim 没有发现它的调用；它不会解释目前的机器人
低速或规划耗时。其他正在使用的多项式求值与初始化方法不能因为它未用就一概视为未用。

同文件还有三个未发现产品调用的统计问题：

| 函数 | 问题 | x(t)=t³、0≤t≤1 的诊断结果 |
|---|---|---|
| getMeanAndMaxVel | 采样循环用段末 ts，未用 eval_t | 平均值3；同样100个采样的正确值0.98505 |
| getMeanAndMaxAcc | 同上 | 平均值6；同样采样的正确值2.97 |
| getAccCost | 仅以二次项的常加速度计算，未积分一般多项式加速度 | 返回0；∫a²dt=12 |

这个例子中 getJerk 返回36，与 ∫jerk²dt=36 一致；单个例子不证明全部输入正确。
getLength/getTraj 本身是离散近似，不能把它与解析弧长、解析峰值混用。

- [可复跑的数值诊断源](../../build/scan-review-20261025/polynomial_metrics_probe.cpp)
- [本轮执行结果](../../build/scan-review-20261025/polynomial-metrics-result.txt)

## 四个产品路径问题的具体机制

### 中间高程丢失

`planner_manager.cpp:144–172` 的 `applyLinearZReference()` 按累计 XY 距离重新设置
全部样本 Z；`:435` 每次 reboundReplan 都调用它。B-spline 拟合的是改写后的样本。
`bspline_optimizer.cpp:1092、1117` 把 Z 梯度置零；retiming 又从已经生成的轨迹重采样，
没有读取原参考中间高度。边界 P/V/A 恢复也不恢复原中间地形轮廓。

起终高度相同、局部目标已位于坡后的路线可能被整体拉平。实时占据看见平台时，
候选通常会被最终碰撞拒绝；这会导致全局可行而局部反复失败。不能泛化为所有坡道
都失败，也不能宣称轨迹因此被允许穿过平台。

最小修订方向：保留现有三维参考的中间高程，分别验证单调坡、同高端点的坡顶/平台、
平地与端点 P/V/A。先证明输入高度没有被覆盖，再讨论是否需要优化 Z。

### 精修使用了最后试探缓存

`bspline_optimizer.cpp:1044–1057` 调用求解器。`combineCostRefine():1098` 每次把
试探变量写入 `cps_.points`，而 `lbfgs.hpp:1190–1196` 在失败时会回滚返回变量。
精修没有同步回滚变量，错误分支为空，随后用回调缓存进行轨迹验证和输出。

这可能验证未被求解器接受的试探点，产生无谓失败或错误的优化成功语义。
它仍经过碰撞与导数验收，本轮没有某个既有物理案例触发该分支的证据。
最小修法是明确允许接受的返回码，并以求解器实际返回的控制点为唯一结果再做原验收。

### A* 开放集顺序失效

`dyn_a_star.cpp:261–264` 在找到更短路径时修改已在 priority_queue 中的节点分数，
没有相应的堆调整。比较器读取节点当前 fScore，但 priority_queue
不会因对象字段变化自动调整结构。

实际 GridNode/NodeComparator 的小复现：已有节点分数10、20、30，入队后把30改为1，
队头仍然是10；重建后的队头才是1。该分支可能破坏预期搜索顺序和预算利用，
不是已测得的导航成功率或耗时下降，也不是最终碰撞校验问题。

修订应在现有 A* 内正确维护开放集：例如用不变的入队优先级与更新后条目，弹出时
跳过过期条目；或采用真正支持修改key的堆。仅再次push同一个已修改指针不能作为
完整修复依据，因为原堆中该对象的比较值已经改变。不需要替换全局规划器。随后用同一离散图的
Dijkstra或正确堆实现比较路径代价、扩展次序与失败原因。

- [实际类型的队列诊断源](../../build/scan-review-20261025/priority_queue_probe.cpp)
- [本轮执行结果](../../build/scan-review-20261025/priority-queue-result.txt)

### 失败记录缺少重放条件

`ScanFailureSnapshot`（`planning/local/planner.hpp:435`）没有可选的支撑高度/容差。
`backend.cpp:575` 捕获后，`nav_status_writer.cpp:664–786` 的 JSON 只写膨胀占据，
没有写 Body/observedBody 和原始 measured occupied/known free。
Body 查询实际依赖这些输入，因此落盘文件不能独立还原相同几何和支撑判断。

这是诊断可复现性缺口，不能称为运动放行漏洞。修订应补齐已有失败记录真正使用的
输入，复用现有序列化结构；无需新增记录服务、版本框架或哈希。

## 函数全量目录与职责

完整清单见 [functions.md](../../build/scan-review-20261025/functions.md)，
结构化记录见 [function-inventory.json](../../build/scan-review-20261025/function-inventory.json)。
范围内的声明、定义、重载、inline 与构造/析构分别列出，避免把无实现旧声明当成可用功能。
本地函数展示页为 [index.html](../../build/scan-review-20261025/index.html)。

已有 libclang 21.1.8 经 Python ctypes 读取完整语法树：**25/25文件解析零error**。
目录包含 **276个函数体定义、5个显式defaulted、6个显式deleted**，合计287个
源级定义条目；另列168条去重非定义声明，其中9个在本范围没有匹配到定义。
这些条目不能相加称为455个独立可用功能。

统计对应当前C++17 host预处理配置。未激活的预处理分支、编译器隐式生成成员、
实例化副本、局部lambda和函数指针typedef不按独立源级函数统计。分析TU预包含
iostream，复现polynomial_traj.cpp先包含它再引入头文件的上下文；不代表独立头文件
构建验收。未安装依赖，生成脚本为
[scan_function_inventory.py](../../tools/bench/scan_function_inventory.py)。

| 模块 | 函数主要处理的内容 | 当前调用含义 |
|---|---|---|
| task.* | 输入持有、worker、100/20Hz调度、取消/接管、结果缓存 | 异步工作入口，调度频率不是求解耗时保证 |
| backend.* | 请求转换、FSM输入、样条输出、状态和失败快照 | 原生适配层 |
| reference.hpp | adaptReferencePath | 坐标高度适配与同一直线冗余点处理 |
| grid.* | 位姿/线段/样条占据、Body、支撑、预测障碍、制动与离开边界 | 三维碰撞/支撑查询，不生成步态 |
| upstream/plan_env | GridMap接口转发、轨迹时间偏移 | 使用原生Grid，不建立第二张地图 |
| upstream/path_searching | A*节点、启发式、坐标、搜索、路径回溯 | 局部避障引导；Z按起终插值，非任意XYZ搜索 |
| upstream/traj_utils | 多段多项式初始化，P/V/A求值，轨迹统计 | 生成样条初始形状；部分统计API未调用 |
| upstream/bspline_opt/uniform_bspline | 参数化、de Boor求值、导数、时间扩展和统计 | 样条表示与运动导数检查 |
| upstream/bspline_opt/bspline_optimizer | rebound/refine、距离/平滑/拟合/可行性成本、最终轨迹检查 | 非凸局部优化；当前主要改XY |
| upstream/bspline_opt/lbfgs | 向量操作、线搜索、区间更新、L-BFGS主迭代 | 求解器依赖，不能把收敛等同于运动可执行 |
| upstream/plan_manage/planner_manager | 多项式种子、局部规划、重定时、边界恢复 | 组织实际求解过程 |
| upstream/plan_manage/planner_fsm | 目标选择、六状态机、重规划、future collision、紧急停止 | 管理局部轨迹生命周期 |
| upstream/plan_manage/closed_loop_controller | 样条参考反馈、世界/机体速度、朝向控制 | 与跟踪适配及最终仲裁共同执行 |
| upstream/plan_manage/plan_container | 参数、全局/局部轨迹缓存及求值 | 内部数据与访问函数 |

### 不属于完整可用API的旧声明

`BsplineOptimizeTraj`、`setCostFunction`、`setTerminateCond`、`setGuidePath`、
`setWaypoints`、`optimize`、`getControlPoints` 及私有 `costFunction/combineCost`
在当前树未找到定义，也未找到实际产品或测试调用。它们不是当前运行故障，
也不能列成“全部已经实现的功能”。正式使用的是 Rebound/Refine 路径。

`UniformBspline::getLength()` 没有补齐最后不足采样步长的尾段；该组 B-spline 统计
方法未发现产品调用。两类轨迹的统计方法不应作为未经验证的论文精度/峰值计算来源。

### 性能与能力边界

Task 内 FSM 与 future-collision callback 串行执行，100/20Hz是调度目标，不能保证
昂贵求解期间仍按10/50ms完成回调。最终控制链的实时检查与该调度应分别评价。

当前 Grid 的 `bodyPoseOccupancy()` 逐体素调用 `collision::intersects(part,pose,...)`，
而 body.cpp:181 每次重新 transform 同一部件。扫掠/制动又会调用多次位姿查询，
`brakingScale()` 还可能进行8次二分查询。这是可以直接从代码看到的重复计算范围。
本轮没有新profile，不能把它写成已测最大瓶颈，或承诺具体加速比例。

局部 DynAStar 只在XY邻居中扩展，Z由起终点插值；它不是全局 OctoPlanner3D 的
XYZ/yaw搜索。此处优先队列问题不能直接解释先前困难全局搜索的10秒耗时。
SCAN没有足迹、关节力矩、接触力或步态求解；jerk平滑代价也不等于严格jerk上限验收。

## 团队覆盖与验证限制

- 主代理逐体读取 Grid、plan_env、DynAStar、PolynomialTraj 及相关契约/测试，
  运行了两个隔离 C++ 诊断；未做完整搜索路线、物理或现场复测。
- 优化审阅代理逐体读取 UniformBspline、BsplineOptimizer 和 L-BFGS 函数，
  没有逐项独立证明插值宏代数或执行有限差分梯度检查。
- 接入审阅代理读取 Task/Backend、FSM/manager/controller、Follower/Executor接线，
  以实际调用筛选问题；不把串行调度、当前三轴速度输出或缺足迹规划直接当代码缺陷。
- 函数清单用已有 Clang 语法树提取。语法树证明文件/函数覆盖，不证明每个函数的
  运行结果；具体解析结果和盲区由清单记录。
- 原有 CTest 源码用于核对已覆盖的行为，本轮未重跑全套，上一轮49/50项结果不是
  上述新发现已经被修复的证据。

独立详细记录：

- [优化/样条审阅](../../build/scan-review-20261025/review-optimizer.md)
- [接入/状态机/执行审阅](../../build/scan-review-20261025/review-integration.md)

## 可复跑诊断

使用现有 WSL Ubuntu、GCC 与 Eigen，无新增依赖：

```sh
g++ -std=c++17 -O1 -Isrc/nav/cpp -I/usr/include/eigen3 \
  build/scan-review-20261025/polynomial_metrics_probe.cpp \
  -o build/scan-review-20261025/polynomial_metrics_probe
build/scan-review-20261025/polynomial_metrics_probe
g++ -std=c++17 -O1 -Isrc/nav/cpp -I/usr/include/eigen3 \
  build/scan-review-20261025/priority_queue_probe.cpp \
  -o build/scan-review-20261025/priority_queue_probe
build/scan-review-20261025/priority_queue_probe
```

诊断程序返回0表示重现报告中的现有缺陷，不能写成“算法修复后测试通过”。
