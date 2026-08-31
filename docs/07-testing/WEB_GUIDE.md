# 测试与验证指南

LingTu 的验证是分层的。通过的检查只证明它实际覆盖的边界：本地 Module 组合、指定的仿真环境，或指定的现场会话。本指南将这一原则转化为可重复的发布和集成流程。

> **Status:** 当前验证指南<br>
> **Audience:** 开发人员、验证工程师、集成人员和发布负责人<br>
> **Runs on:** 本地开发主机、仿真环境和受监督的现场目标

> **安全边界：** 现场验证从无运动证据开始。任何能提交目标、速度或物理运动命令的测试，都需要单独、经批准的门槛；本地或仿真结果变绿后，绝不能将其视作隐含的下一步。

## 每个验证层证明什么

| 层级 | 典型证据 | 它证明什么 | 它不证明什么 |
| --- | --- | --- | --- |
| 静态/文档检查 | 链接、源码、类型、lint 或 schema 检查 | 被检查修订中的源码一致性。 | 进程能够启动或机器人可达。 |
| 本地运行时 | Stub/profile 测试、Module 图、单元/集成测试 | Module 组合和选定的离线行为。 | 仿真物理、传感器时序或现场安全。 |
| 仿真 | 指定场景/profile 和模拟器门槛 | 该模拟器内指定的软件/数据流路径。 | 标定、网络归属或现场就绪度。 |
| 现场无运动 | 状态、数据流、doctor、路线预览、地图门槛 | 具名目标给出了受限的就绪度结果。 | 移动许可或未来路线的成功。 |
| 现场运动 | 显式的受监督验收流程 | 仅已记录的动作、目标、环境和时间窗口。 | 通用行为或未来部署。 |

不要把结果改写为更宽泛的结论。仿真通过不能验证物理传感器生命周期，Gateway 可达也不能验证定位或路径安全。

详细入口：

- [实机验证](field/README.md)
- [仿真验证](simulation/README.md)
- [带日期的验证记录](field-runs/README.md)

## 1. 验证文档和源码

先运行最小相关检查，再扩大测试范围。文档检查会保持精选链接、元数据、端点脱敏、格式和 Web 指南源码注册的一致性：

    python -m pytest tests/docs/test_documentation_navigation.py -q

对于 Module/运行时变更，先运行框架测试集：

    python -m pytest src/runtime/tests/ -q

**预期结果：** 选定测试在当前 checkout 中通过。失败是源码层阻塞项，不应通过更换 profile 或端点设置来规避。

如果带 locked 选项的 uv 报告 lock drift，请停止并协调预期的依赖状态，然后再声称结果可执行。未锁定运行会改变测试所证明的内容。

## 2. 验证本地产品路径

使用 stub profile 验证不依赖硬件或现场 Gateway 的 Module 图：

    uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run

在 REPL 中，仅使用检查命令：

    health
    connections
    config
    quit

**预期结果：** 该图以 stub driver 启动，报告连接/健康状态，并正常退出。它不会连接机器人，也不证明相机、检测器、SLAM 服务或物理命令写入端可用。

## 3. 验证具名仿真门槛

只安装所选 profile 所需的仿真 extra；启动前检查选定的数据源和命令接收端：

    uv sync --locked --extra dev --extra sim-mujoco
    uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run --json
    uv run --locked python sim/scripts/mujoco/native_navigation_acceptance.py --manifest config/runtime_graph/acceptance/mujoco_industrial_park_60m_navigation_acceptance.json --preflight-only --strict

**预期结果：** Product dry-run 标识模拟命令接收端；native acceptance
preflight 验证当前场景、地图和原生进程输入，但不会启动运动阶段。若预检
识别出物理端点，请停止并在启动前修正配置。

请将场景、profile、修订、可选 extra、命令接收端和断言随结果一起记录，使读者能理解被验证的范围。

## 4. 在不运动的情况下采集现场证据

在获授权的机器人侧 shell 或操作会话中，先使用无运动门槛：

    bash scripts/lingtu status
    curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
    PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict

对于已保存地图工作，请在考虑目标之前验证完整地图包并预览精确路线：

    python scripts/gates/saved_map_artifact_gate.py <map-id> --require-occupancy
    python scripts/gates/system_acceptance_gate.py --maps-root "$LINGTU_MAPS_ROOT" --map <map-name> --goal <x> <y> <yaw>

**预期结果：** 每个门槛都会为选定的 profile、端点、地图、定位状态和规划器报告可追溯结果。请将原始输出、时间戳、修订和拒绝原因随验证记录保存。

**停止条件：** 标定失败、数据流过期、地图溯源无效、定位质量差、安全停止、速度/控制来源不明确，或路线预览不可行。不要为了确认阻塞项是否真实而下发目标。

## 5. 保持运动验证显式化

部分机器人侧验收工具提供可见的 `--allow-motion` 复核标志。该标志是有意设置的授权边界。不要在调试失败的无运动门槛时添加它，也不要将它写入通用 CI 或后台重试循环。

在执行具备运动能力的验收动作前，必须满足以下全部条件：

- 有具名、已批准的现场流程和负责操作员；
- 区域清空、紧急流程完备，并有可见的停止/取消控制；
- 在规划坐标系中存在活动地图和可信定位；
- 无运动地图、就绪度和路线预览门槛均已通过；
- 安全状态、命令来源和控制归属符合预期；以及
- 已收集请求、响应、运行时状态和恢复相关证据。

请阅读[安全与控制边界](../10-safety/README.md)，了解控制路径、租约、停止/取消/reset 和速度仲裁规则。运动回执不证明机器人已安全到达目标。

## 6. 记录持续有用的证据

| 记录项 | 重要性 |
| --- | --- |
| 修订和依赖/资产身份 | 区分已测试的软件/构件与之后的 checkout。 |
| Profile、端点、地图和场景 | 明确实际的数据/命令边界。 |
| 前置条件和环境 | 使隐含的硬件、网络、操作员假设可被复核。 |
| 精确命令和效果分类 | 区分检查、状态变更和可能的运动。 |
| 预期和观测结果 | 避免一段日志被当作宽泛的成功声明。 |
| 停止/恢复操作和剩余风险 | 使故障具有运维价值，而不是被静默重试。 |

带日期的现场运行记录应作为证据保存到 field-runs 目录中。它们对可追溯性很有价值，但没有明确的当前替代或重新验证时，不会成为当前产品契约。

## 发布与集成清单

- [ ] 已在拟议修订上通过有针对性的源码/文档检查。
- [ ] 选定的本地/仿真检查与变更表面相匹配。
- [ ] 已记录可选依赖和原生构建前置条件。
- [ ] 现场就绪证据明确目标、端点、地图和定位，而非笼统声称“机器人可用”。
- [ ] 在请求运动复核前，无运动门槛已通过。
- [ ] 如有运动，具有独立的受监督流程和证据记录。
- [ ] 剩余缺口链接到[已知缺口](../known_gaps.md)或清晰标记的计划，而不是无关的通过测试。

## 下一步

请使用[快速开始](../QUICKSTART.md)选择安全路径，使用[现场部署指南](../04-deployment/WEB_GUIDE.md)准备目标，并使用[运维](../06-operations/README.md)进行诊断和恢复。详细验证清单与带日期的证据保留在测试目录中，供获授权读者了解其精确范围。
