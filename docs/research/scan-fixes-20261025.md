# SCAN 审阅问题修复与验证

日期：2026-10-25。承接[修复前审阅](scan-function-review-20261025.md)。
本轮修复原生代码、补定向回归及教学说明，没有部署、切换Product或启动机器人。

## 已修复的内容

| 问题 | 修复 | 验证方式 |
|---|---|---|
| 局部初始化丢失中间高程 | FSM传入实际选择的3D投影到目标区间；曲高种子沿参考三维弧长重采样，保留原种子相对加速/巡航进度与总时长 | FSM→manager最终局部样条保留坡顶；移动P/V/A边界不变；已观测障碍仍拒绝 |
| Refine使用最后失败试探缓存 | 始终从L-BFGS返回q同步控制点，非接受返回码明确报optimizer_error，随后保留原最终检查 | 真实ROUNDING_ERROR有限精度例；正常成功、碰撞拒绝及恢复 |
| A*堆内可变分数破坏排序 | 条目保存入队分数；改优重新入队；弹出跳过旧/closed条目 | 真A*地图路线达到独立几何下界；不重复展开旧条目 |
| 复用A*失败后仍有旧路径 | 每次搜索开始清理上次路径 | 同一对象先成功再失败的公共接口回归 |
| getMeanVel缺return | 返回已有缓存长度/时长 | 已有调用顺序下的平均速度解析值 |
| 速度/加速度统计误用段末时间 | 以每个采样时刻计算 | 固定采样时刻与多段导数值 |
| 一般多项式加速度代价不完整 | 按各段解析积分∫‖a(t)‖²dt，保留高阶、交叉项与三轴 | 三次/五次、多段不同阶数及交叉项解析答案 |
| B-spline长度漏尾段 | 补最后不足一步的区间，包含短于步长的轨迹 | 3-4-5直线几何长度与整步边界 |
| Body模式失败记录缺几何回放条件 | 补Body/observedBody、原始occupied/free、支撑参数、预测expiry等现有输入 | 实际Backend失败→JSON→独立重建Grid，对照碰撞/自由/缺支撑 |
| 9个无定义、无调用的旧接口 | 删除旧声明，保持正式Rebound/Refine接口 | 全树调用核查及修复后Clang目录：未匹配声明为0 |

三维种子改变限于参考窗口包含非线性高度的自主参考路线。平地、直线高度参考和
辅助MotionIntent保留原初始化。没有增加任意XYZ优化、落脚规划、步态或地形填充。

实现位置：

- `planning/local/scan/upstream/plan_manage/planner_manager.cpp/.h`
- `planning/local/scan/upstream/plan_manage/planner_fsm.cpp/.h`
- `planning/local/scan/upstream/bspline_opt/bspline_optimizer.cpp/.h`
- `planning/local/scan/upstream/path_searching/dyn_a_star.cpp/.h`
- `planning/local/scan/upstream/traj_utils/polynomial_traj.h`
- `planning/local/scan/upstream/bspline_opt/uniform_bspline.cpp`
- `planning/local/planner.hpp`、`planning/local/scan/backend.cpp`、`endpoint/nav/status/nav_status_writer.cpp`

路径均相对于`src/nav/cpp/`；
可点击函数位置见修复后的[全量目录](../../build/scan-fixes-20261025/functions.md)。

## 修改前的实际失败

第一批公共接口回归4项均失败：

- 原参考起终0.5m、中间1.1m，旧版最终局部样条最高仅约0.5m。
- 真实A*固定图旧路线11.65685424949m；存在路线达到独立下界4+5√2≈11.07106781187m。
- x=t²在0/.01/.02/.03s的采样平均速度应为.03，旧版为.07。
- 实际Backend失败JSON没有body字段，无法重建原查询。

搜索/指标分支另运行15个旧版安全测试，13失败、2通过；缺return的函数没有在修复前
执行，以避免未定义行为。精修分支用修复前库复现ROUNDING_ERROR后仍返回accepted。
它是有限精度隔离例，不代表已有物理案例触发频率。

首轮GREEN有一项测试oracle错误：它要求求解器回滚等于初始输入，但求解器可能已
接受若干迭代。修正后保留错误拒绝、固定边界精确保持和输出有限性，不用放宽容差
掩盖问题，也不宣称公开接口已独立暴露/证明内部最后接受q。

## 已运行的验证

| 检查 | 当前结果 |
|---|---|
| Linux x86_64，11个直接相关目标的CTest | **239/239通过**，包含25个本轮新测试 |
| Windows MSVC Release，本轮新CTest | **25/25通过** |
| Linux正式navd编译链接 | 通过，含最终类型整理后增量构建 |
| Windows正式navd.exe编译链接 | 通过，含最终类型整理后增量构建 |
| 最终类型整理后的直接指标复测 | Linux18/18、Windows18/18通过（属于前述集合，不另算独立覆盖） |
| 独立只读审阅 | 未发现需要阻断GREEN的实际逻辑问题 |
| 修复后Clang目录 | 25/25文件解析成功；288定义=277函数体+5defaulted+6deleted，159条声明，未匹配声明0 |

239与25是CTest条目数，不是仿真次数或实机次数。Windows集合是Linux相关集合的一部分，
两者不能相加宣称264种独立能力。

类型整理仅将新积分实现的下标类型与有界多项式阶数转换写明确，未改变积分公式。
该整理后的直接指标复测与增量构建单独保留，未因此要求重复所有已通过的行为回归。

## 证据与程序位置

- [本轮展示与讲解](../../build/scan-fixes-20261025/index.html)
- [原始4项失败](../../build/scan-fixes-20261025/red-test.log)
- [Linux239项结果](../../build/scan-fixes-20261025/green-tests-linux.log)
- [Linux选中测试名称](../../build/scan-fixes-20261025/green-selection-linux.txt)
- [Windows25项结果](../../build/scan-fixes-20261025/green-new-tests-windows.log)
- [Linux正式程序构建](../../build/scan-fixes-20261025/navd-build-linux.log)
- [Windows构建](../../build/scan-fixes-20261025/build-windows.log)
- [最终Linux增量构建](../../build/scan-fixes-20261025/final-build-linux.log)
- [最终Windows增量构建](../../build/scan-fixes-20261025/final-build-windows.log)
- [最终Linux指标复测](../../build/scan-fixes-20261025/final-metrics-linux.log)
- [最终Windows指标复测](../../build/scan-fixes-20261025/final-metrics-windows.log)
- [搜索/指标红测](../../build/scan-fixes-20261025/search-metrics/red-test.log)
- [精修原始红测XML](../../build/scan-fixes-20261025/refine/red-tests.xml)
- [精修实现与测试边界](../../build/scan-fixes-20261025/refine/implementation.md)
- [失败记录修复说明](../../build/scan-fixes-20261025/failure-snapshot/README.md)

四个新测试源：

- [test_scan_elevation.cpp](../../tests/nav/cpp/test_scan_elevation.cpp)
- [test_scan_refine_solver.cpp](../../tests/nav/cpp/test_scan_refine_solver.cpp)
- [test_scan_search_metrics.cpp](../../tests/nav/cpp/test_scan_search_metrics.cpp)
- [test_scan_failure_snapshot.cpp](../../tests/nav/cpp/test_scan_failure_snapshot.cpp)

## 仍需准确区分的范围

1. 新高程测试证明参考几何没有被压平，且既有验收仍执行；没有运行新物理闭环，
   不能据此认定轮足步态可跨台阶或沟槽。
2. 失败JSON可回放Grid几何/支撑输入；没有保存全部参考多项式与每次优化历史，
   不承诺整个FSM/求解过程位精确重演。
3. getMeanVel仍按已有getTraj→getLength→getMeanVel序列使用；采样mean/max保留
   10ms、段端点排除语义，不是连续时间极值保证。
4. A*测试改善的是本固定离散图的路线代价，不是实机规划耗时改善测量；未承诺加速百分比。
5. 现场NX aarch64和监督运动未验证，默认机器人模型、ProductControl和最终运动权限不变。
6. 构建仍有既有上游符号比较、未使用参数及端点聚合初始化等告警，不能写成全仓无告警。
   MSVC的L-BFGS ret未初始化告警已独立逐路径检查，各退出路径均已赋值；没有发现可达
   的未初始化返回链。937行不可达兜底位于只通过return退出的无限循环之后，未扩改求解器。

## 教学入口

[读懂SCAN：函数、Clang、搜索、轨迹优化和闭环](scan-explained-20261025.md)
从声明/定义和C++所有权开始，进一步解释时间参数化、软代价/硬约束、优先队列、
三维数据流与验证证据。函数数量只用于定位代码，不作为产品完成度指标。
