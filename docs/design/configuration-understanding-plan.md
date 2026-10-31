# 配置与架构理解成本收敛计划

状态：规划，尚未实施。日期：2026-10-28。

## 目标

让开发者能回答三个问题：这个 Product 提供什么能力？模型连接配置在哪里？最终 RunPlan 向各模块传了什么？以 `nav/semantic`、`tracking` 和 `inspection` 为首批范围；保留 ProductControl 一次解析、RobotConfig、real/sim 和现有运动安全边界。

## 已确认的现状

1. `nav/semantic` 和 `tracking` 分别声明同一组 Kimi 配置；普通 `nav/standard` 和 `nav/camera` 不装配这条语义规划链。tracking 不装配 AgentPlanner，但 VisualServo 依赖 LLMModule 的视觉客户端，不能删掉其 LLM 配置。
2. 语义规划存在实际配置断链：`semantic_tasks.py` 为 LLMModule 和 AgentPlanner 传入已解析的 backend/model/base URL/key env/timeout，但给 SemanticPlanner 仅传 backend；它内部 GoalResolver 独立创建客户端，因而当前 Kimi 配置会回落到默认 `kimi-k2.6`、10 s。该客户端在导航失败后的 LERa 恢复路径可达；不能说每条指令都会调用它，也不能把这一点归因于服务端。源头：`src/lingtu/assembly/host/stacks/semantic_tasks.py`、`src/decision/modules/semantic_planner.py:282`、`:1543`、`src/decision/goals/resolver.py:185`。
3. `nav.yaml` 最大的重复来自 variant 现行的**整个顶层字段替换**：`topics`、`capabilities`、`host` 等不是递归合并或追加。产品的顶层 capabilities 与 host.capabilities 有不同消费者，不能合并删除。见 `src/lingtu/assembly/graph/loader.py:93` 和 `config/system/README.md`。
4. 直接删 Kimi 的 model/timeout 会让客户端回落到 `kimi-k2.6`/10 s；不能作为纯清理。把 Kimi 写进现有 real `host_config` 也不行：目前 Env 字段晚于 Product 覆盖，会改掉 `inspection` 明确选择的 Qwen。
5. 上次解释“同一组模型配置已经传给所有语义客户端”不准确：当前只明确传给前两个，GoalResolver 仍是上述独立配置路径。
6. `inspection` 只指定 `llm: qwen`。LLMModule 会补 `qwen-turbo` 和 `DASHSCOPE_API_KEY`，但 AgentPlanner 直接构造客户端时使用空配置，GoalResolver 又使用自己的默认值；`QwenClient` 本身没有统一补这两项。必须先核对并明确巡检路径的有效值，不能仅测试 provider 名称就宣称三个客户端一致。

## 责任划分

| 信息 | 负责位置 |
| --- | --- |
| Product 提供的能力、必要进程和 topics | `config/system/products/*.yaml` |
| real/sim 的进程实现、transport 和限定能力 | `config/system/envs/*.yaml` |
| 机器人几何、设备和标定 | `config/robots/<vendor>/<model>/robot.yaml` |
| 可复用的模型连接参数 | 拟在 `config/system/` 增加一个小型共享模型配置，由 assembly 读取；Product 仍明确选择 backend |
| 特定部署的模型服务地址或凭据 | 由已解析配置确定地址，密钥仍从 Host 进程中由 `api_key_env` 指定的环境变量读取；不得把密钥写入 RunPlan |
| 最终启动值 | ProductControl 一次生成的 `RunPlan.host_config`，Host 和运行时只消费这份结果 |

产品是否需要语义能力，与其选择 `kimi` 还是 `qwen` 是两个明确选择。共享设置只提供该 backend 的连接默认值；不得把 backend 本身改成根据 real/sim 推断。

## 实施顺序

### 1. 补齐实际模型配置传递

- [ ] 调查 GoalResolver 所有实际调用及生命周期，沿现有参数通路将 backend、model、base URL、key env、timeout 传到 SemanticPlanner 的 GoalResolver 客户端；考虑直接 Host/模块调用者与现有默认行为。
- [ ] 针对 inspection 的 Qwen，先统一解析其现有 LLMModule 实际使用的默认模型和凭据变量，让各 client 接收到确定值；这会修正此前 Agent/恢复路径的默认值差异，不应称为所有路径行为保持不变。
- [ ] 加一项定向回归：在无网络的测试里从真实 Host 装配路径传入不同于默认的五个字段，检查 LLMModule、AgentPlanner 和 GoalResolver 收到一致的解析值；覆盖明确自定义地址/凭据变量和 Qwen backend。另覆盖没有语义规划的 tracking，确认视觉客户端仍装配。
- [ ] 确保 native 导航、运行时运动所有权、相机标定检查和完成证据逻辑不变。

这一步是故障修正，会让此前漏传的恢复路径实际使用所选配置；不称为纯重排。首先回归证明配置断链被修复，再进行后续整理。

### 2. 把重复的模型连接配置收归一处

- [ ] 将两处完全相同的 Kimi 连接默认值放入一个狭义配置文件，例如 `config/system/model_clients.yaml`，仅包含当前需要复用的配置，不建立多层 profile、继承或插件体系。
- [ ] `nav/semantic` 和 `tracking` 保留明确的 `llm: kimi` 和自身能力；删除重复的 model/base URL/key env/timeout。`inspection` 继续选择 `qwen`。
- [ ] 在现有 assembly 解析点按已选择的 backend 合并默认值，并保留明确的 Product 字段、Env host_config、程序调用 overrides 的当前覆盖语义。优先级必须在实现前用前后对照确定；模型地址是 Endpoint，不能改变 `env=real|sim` 的含义。
- [ ] 合并后的五个现有扁平字段仍写进同一个 `RunPlan.host_config`，Host 不在运行时重新读取模型文件。没有语义能力的 Product 不应额外创建 LLM 客户端。
- [ ] 更新实际配置加载/校验入口及发布打包路径，确认运行时安装包包含所需文件；更新 `config/system/README.md` 中“loader 只读 products/envs”的现有说明。缺文件、未知或未配置 provider 的行为应沿用现有能力与支持范围，避免静默切换模型。
- [ ] 保存实装前后的配置对照：Go2、Thunder 的 `nav/semantic`、tracking、inspection；对比已解析 backend/model/URL/key env/timeout、模块/进程/topic/校准和 ProductControl 路由。只预期新增配置所有权改变，GoalResolver 的参数由第 1 步修复改变。

若最终证明共享文件的加载和发布成本超过两处重复，保留 Product 中的五项并只做第 1、3 步；不能为了省几行默默引入更复杂的通用配置系统。当前仓库没有 `model_clients.yaml` 自动加载机制，不能直接移动文件而不调整 assembly。

### 3. 降低 Product YAML 的阅读负担

- [ ] 在现有整体替换规则下整理顺序，让基础 Product 和实际变体差异更容易对照；只移动完全相同、可以安全继承的字段。
- [ ] `nav` 的 camera/semantic 完全相同的 `processes`、`optional_processes` 和 capabilities 可考虑文件内 YAML alias；只有在少量 alias 确实比重复更好读时才使用。topics 依旧分别列出，避免列表拼接或新增 merge DSL。
- [ ] 不将整个 camera 变体合并给 semantic：camera 有 `require_map_scene: false`，semantic 当前没有相同覆盖；整块复用可能改变地图场景就绪要求。
- [ ] 独立对照 `explore` 的两个变体，相同 `switch_policy` / `online_hot_switch_supported` 可上移至顶层；这些只是可继承候选，必须比较解析结果，不以减少行数替代行为检查。
- [ ] 按 Product/variant 对比整理前后 `RunPlan` 的完整契约，包括 topics、capabilities、进程、模块、readiness 与 native 参数。`nav/semantic` 仍仅 real 支持，原有 sim 和 camera 边界保持。

### 4. 给开发者一条可读路径

- [ ] 更新现有 `config/system/README.md` 和 `src/decision/README.md`，用一张小图写明：Product 能力 + 环境/机器人配置 + 模型连接 → assembly 一次解析 → RunPlan → Host。
- [ ] 用一个实际例子说明如何定位模型、Endpoint 和密钥变量，以及 `nav` 与 `tracking` 为什么都可能需要 LLM。
- [ ] 明确正式 CLI 的 `--set` 当前是 native 参数，不宣称已经支持修改 Host 模型；如以后确需现场选择模型，单独明确需求并扩展 ProductControl，不在本次加新参数。

## 验收与证据

- 模型配置闭环：非默认 Kimi 值传至 3 条实际客户端路径；tracking 不误装 Agent；inspection Qwen 不被 Kimi 默认覆盖；无真实密钥进入 RunPlan/日志。
- 产品行为不变：`nav` standard/camera/semantic、tracking、inspection 的进程、topics、Host 模块、硬件/标定、导航参数及 real/sim 支持边界与实施前一致。
- 保持一次解析：使用 `tests/lingtu/assembly/test_compile.py` 的 resolve-once 测试与 RunPlan 回放验证；有失败时先排查是否发生 Host 二次解析或解析来源漂移。
- 运行最窄相关测试：`tests/lingtu/assembly/test_semantic_product.py`、`tests/runtime/test_product_host_builder.py`、`tests/decision/test_planner_node_init.py`、相关 runtime graph / ProductControl 测试及受影响 Python lint。先记录当前失败基线；只在改动覆盖该面时补充直接相关检查。
- 验证安装包实际包含共享文件和 Host 从已保存 RunPlan 恢复的行为；本地契约测试不等于 MuJoCo、现场无运动或实机运动证据。

## 明确不在本轮

- 不创建新的通用依赖注入、配置继承/列表合并框架或并行 Product lifecycle。
- 不合并三个 LLM 客户端实例：它们涉及事件循环和生命周期，配置一致后是否共享应单独论证。
- 不把所有环境和所有 Product 强制切到 Kimi；不顺带修改模型强度、超时、导航速度或运动权限。
- 不把配置整理当作对话澄清、任务自动收尾或实机功能已完成。
