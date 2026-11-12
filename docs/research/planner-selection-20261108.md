# 局部算法在线选择：实现与本地验证

2026-11-08。本轮完成局部算法在同一个 navd 进程内切换；不包含 Product 建图/导航热切。保存地图与切模式独立，保存后继续建图的语义已写回[模式切换计划](runtime-switch-lightweight-20261108.md)。

## 已接入的调用链

`robot.navigation.select_planner("cmu")` → `POST /api/v1/navigation/planner` → Host 的 `nav.commands` → 原生客户端 → DDS `SelectPlanner`（kind 19）→ navd。

HTTP 示例：

```json
{"name":"cmu","request_id":"planner-1","client_id":"sdk"}
```

只有导航控制模式支持实际更换算法。当前任务（包含暂停任务）、巡检、滚动任务和人工控制权需先结束。调用者随后发选择命令；无需退出 navd，再发新目标即可。没有在运行中的旧目标里自动替用户决定取消任务。

原生处理：

1. 核对 Product/env 已声明的可选算法、SCAN 100 Hz 条件、CMU 路径库。
2. 在后台构造候选规划器与 Executor，主线程继续处理 DDS 和停止输出。
3. 复用现有停止确认，准备好且停止得到确认后，再一起更新执行器、输入要求、规划数据选择和状态配置。
4. 准备失败保留原算法。切换期间拒绝新运动任务；停止/急停取消交接，后台晚到的候选被丢弃。

旧 Executor 与取消后晚到候选的释放也由后台完成。仅将加载移到后台还不够：首轮日志显示，主线程销毁 CMU 对象曾造成约 0.1 秒停顿，已据此修复。

成功 ACK 为 `planner_selected` 或 `planner_unchanged`。地图/定位输入未就绪时，选择算法可以成功，新运动任务仍由既有输入检查拒绝。RunPlan 继续记录启动配置，当前选择由原生状态中的 `local_planner` 报告。

CMU 路径库只给已有适配的机器人提供地址，按需加载；不预先运行两套算法。新原生客户端能力位为 `PLANNER_SELECTION`。DDS 请求新增 `local_planner` 字段，客户端与 navd 必须一起重建/部署。

## 代码归属与官方对照

- `src/nav/cpp/endpoint/nav/app/planner.cpp`：后台加载候选，不新增插件系统。
- `src/nav/cpp/endpoint/nav/app/loop.cpp`：命令接收、空闲边界、停止确认、取消和最终 ACK。
- `src/nav/cpp/endpoint/nav/main.cpp`：在原有对象引用处整体替换执行器并更新输入/状态。
- `src/lingtu/assembly/compile_plan.py`：沿用 Product 可选列表和 env 资格声明，为已适配的 CMU 提供路径库地址。
- HTTP、同步/异步 SDK、原生 C++/C ABI、Host 命令模块已经接线。API 文档由现有生成器更新。

官方参考为 [Nav2 controller_server.cpp，固定提交 35d9203](https://github.com/ros-navigation/navigation2/blob/35d920348c447babf02889d82812993f3967298a/nav2_controller/src/controller_server.cpp#L334)。它支持按 controller_id 选择已加载控制器。本地第一版采用“空闲边界、后台按需构造、停止确认后替换”，不声称这是 Nav2 默认行为，也不声称可以在运动中拼接不同算法轨迹。

修改前源码只有启动时的 LocalPlanner 构造选择，命令协议没有 SelectPlanner。本轮未用旧二进制做计时对比，因此下面不是冷切/热切性能对比；没有调整算法运动阈值、控制频率或到达策略。

## 本地证据

正式源目标已重建：`navd`、`lingtu_nav_client`。测试辅助目标 `lingtu_motion_mock_dds` 增加原有停止契约需要的 DriverControlState 回执，以及非零命令计数。

Windows C++ 四组通过：

- `test_planner_selection`：无效/不允许的算法、缺路径库、SCAN 频率条件、加载失败保留原执行器、重复切换、新输入要求、旧路线不继承。
- `test_nav_endpoint_config`：现有配置契约。
- `test_command_ingress_controller`：请求身份、待完成 ACK、新目标不能复用已完成请求 ID。
- `test_nav_client`：实际 DDS/C ABI 命令往返，目标算法字段和拒绝原因不丢失。

Python 相关选择运行：73 项通过，1 项既有地图 SDK 接口对齐失败。新选择路由、Host 原生回执校验、同步/异步导航 SDK、Product 配置和路径库准备均通过。

实际进程检查使用本机 loopback DDS domain 209 和模拟驱动，没有连接机器人。运行同一个 `navd`：

| 操作 | 返回 | 本次耗时 |
| --- | --- | --- |
| SCAN → CMU | planner_selected | 4.531 秒 |
| CMU → SCAN | planner_selected | 0.407 秒 |
| SCAN → CMU | planner_selected | 4.453 秒 |

PID 始终为 170768。随后在 CMU 加载中发送停止，选择请求返回 `planner_switch_cancelled`；后台加载结束后再次确认仍为 SCAN。模拟驱动记录 655 条命令，非零命令为 0。耗时包含本地加载、DDS 和停止确认，不代表目标机器人的耗时。

同一案例在后台释放修复前/后的末段 600 帧窗口中，主循环工作耗时峰值从 98.7132 ms 降为 8.5077 ms，超过 10 ms 预算的帧数从 3 降为 0。P95 为 0.6602/0.6811 ms，说明这次主要消除了释放对象造成的尖峰；这不是长期实时性保证。修复前原始窗口保存在 `before-retirement.status.json`，修复后在 `no-motion.json`。

可复跑脚本及原始结果：`artifacts/nav/planner-selection-20261108/no_motion_check.py`、`no-motion.json`、`no-motion.log`。当前 CMU 每次按需读路径库仍需约 5 秒；这版实现的是不重启导航进程，不是任意后端瞬时切换。

更广检查中的既有问题单独保留：地图 AsyncMapClient 缺少同步端的 `process_operation`、`download_source`；旧枚举快照缺少 `InspectionTaskState.WAITING_NEXT_LOOP` 和 `SensorControlKind`。新增 SelectPlanner 的数值和生成文件已同步，不通过修改这些无关接口来掩盖基线失败。

未部署实机，未做 MuJoCo 导航或实机运动验证；未新增网页算法选择控件。建图/导航服务热切仍是独立待办。
