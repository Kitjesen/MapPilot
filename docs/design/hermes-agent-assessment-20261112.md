# Bot 当前能力与 Hermes 接入评估

日期：2026-11-12。本轮是能力核对、源码对照与 UI 清理；尚未迁移 Agent 后端。

后续用户已明确选择将核心源码迁入 LingTu。以 [v0.23.1 核心迁入与模型登录方案](hermes-core-migration.md) 为当前设计；下文独立外部服务建议保留为此前评估，不是最终接入决策。正式版支持的 Python 已推进到 3.14，本机旧安装的 Python 3.11 不能代表该版本要求。

当前实施状态以 [功能迁入记录](hermes-core-migration.md) 为准：已删除 HermesRuntime/Gateway 桥接，直接迁入 Codex 设备码授权和刷新流程，沿用现有工具循环。下面的独立服务建议和运行探测为历史研究，不代表产品现状。

## 当前实现到哪里

底部 Bot 连接 `AgentPlannerModule` 的持久会话入口，通过已安装的 OpenAI Agents SDK 执行工具循环；不是从零实现模型推理框架。

已实现并有本地证据：模型对话、附件、历史、权限确认、选择卡、工具状态/计时、地图查询、模型配置查询。地图写入和巡检点接口已接入并通过本地契约检查；端到端实调用主要是只读查询，不能据此宣称实机全流程完成。

受限制的机器人能力：导航、查找物体、识别和记忆查询已有工具入口，依赖对应 Host Modules、定位/地图/传感器和运行就绪条件。持久会话当前明确不提供连续跟随。截图“等待机器人位姿”说明导航条件尚未满足；Bot 对话可用不是运动就绪。

**互联网搜索未接入 LingTu Bot。** 核查了 `src/decision`、`src/gateway`、`config/system` 和依赖声明；持久会话的工具目录没有 web_search/web_extract，也没有向 SDK Agent 注册 WebSearchTool。模型在线 API、内部 query_memory、识别现场物体，都不是网络搜索。

> 后续实现已改变本文早期评估中的接入状态和独立服务建议。
> 当前以 [Hermes 功能迁移清单](hermes-core-migration.md) 为准：网页工具、技能、偏好记忆已接入现有 Agent，不运行 Hermes 中转服务。

## 官方项目对照

| 项目 | 适合复用什么 | 对 LingTu 的意义 |
| --- | --- | --- |
| [OpenAI Agents SDK](https://github.com/openai/openai-agents-python) | 工具循环、模型适配、MCP、托管搜索 | 已在使用；仅补搜索时改动最小。官方 WebSearchTool 走 OpenAI Responses，需要可用的 OpenAI 服务账户，不能据此让所有 Kimi/Claude 自动获得搜索 |
| [Hermes Agent](https://github.com/NousResearch/hermes-agent) | 完整 Agent、工具配置、网页搜索/提取、记忆/技能、API 服务、MCP | 最接近“复用整套通用 Agent，少自行拼配置”的诉求 |
| [assistant-ui](https://github.com/assistant-ui/assistant-ui) | 对话、选择卡、工具结果 UI | 是前端参考，不能代替搜索后端或机器人执行 |

Hermes 官方源码检视固定在 `0d75d1c8fea1413ebe703a932257d6277a7cb6f7`，许可证 MIT。本轮未复制 Hermes 实现或安装依赖。

来源：

- [官方 README](https://github.com/NousResearch/hermes-agent/blob/0d75d1c8fea1413ebe703a932257d6277a7cb6f7/README.md)
- [API Server](https://github.com/NousResearch/hermes-agent/blob/0d75d1c8fea1413ebe703a932257d6277a7cb6f7/website/docs/user-guide/features/api-server.md)
- [Web Search](https://github.com/NousResearch/hermes-agent/blob/0d75d1c8fea1413ebe703a932257d6277a7cb6f7/website/docs/user-guide/features/web-search.md)
- [MCP](https://github.com/NousResearch/hermes-agent/blob/0d75d1c8fea1413ebe703a932257d6277a7cb6f7/website/docs/user-guide/features/mcp.md)
- [许可证](https://github.com/NousResearch/hermes-agent/blob/0d75d1c8fea1413ebe703a932257d6277a7cb6f7/LICENSE)
- [SDK 官方搜索工具示例](https://github.com/openai/openai-agents-python/blob/main/examples/tools/web_search.py)

## 本机实际情况

- 已有 Hermes 安装：`C:/Users/99563/AppData/Local/hermes/hermes-agent`。
- 本机提交：`a6f08ff0c8bcb0d97fa333f1c018cba67f21b4a3`，与上述官方主线不同。
- Hermes 独立环境 Python 3.11.15；LingTu 当前环境 Python 3.10.12。应保留独立环境，不把整套依赖塞入 LingTu 环境。
- `~/.hermes/config.yaml` 已有模型提供商配置；本轮未改动，也未读取或展示凭据值。
- 当前 `web` 没有显式后端配置；本机 Hermes 环境没有 ddgs 包。
- 本次探测 `http://127.0.0.1:8642/health` 未连接到服务；默认配置目录中未发现 API_SERVER_KEY/API_SERVER_ENABLED 声明。不能称 API 已接通。
- 最新主线文档声称提供免 key 的公共免费搜索回退；本机版本的搜索文档不含这项新机制。旧版本可选无 key 的 DDGS（仍需对应包），或需要服务/密钥的其他提供商。新版免费服务的连通性与限流没有在本机验证。
- 官方主线 README 当前声明支持原生 Windows，也支持 WSL2；无需把“必须 WSL2”当作事实。

## 建议边界

若目标只是补联网搜索，先用现有 SDK/MCP 扩展最小；若目标是长期复用搜索、浏览器、记忆、技能与插件配置，优先评估现有 Hermes 作为独立通用 Agent 服务。

预期链路：

`现有 Web → LingTu Gateway 会话适配 → Hermes API → 受约束的 LingTu 工具 / MCP → 原有地图和导航服务`

Hermes 替换上层通用 Agent 执行，不替换 SLAM、全局/局部规划、最终运动检查和驱动。避免把 Hermes 当成普通模型端点塞进现有 SDK 工具循环，造成两套循环各管一遍工具和会话。

Hermes 有 OpenAI 兼容 HTTP API，但兼容文本消息不等于已有 LingTu 工具、审批和取消会自动迁移。特别要逐项验证 tool calls、会话 ID、请求幂等、取消、工具计时和确认事件。当前 LingTu 有 MCP Server/SDK 代码，仍需标准客户端实际握手测试；本轮新增地图工作台工具也不是自动全量导出到 MCP。

## 可执行接入顺序

1. 使用本机已有 Hermes，固定选定版本与独立配置档；不覆盖用户现有配置和记忆。
2. 先打通仅包含网页搜索/提取的 API 服务，验证一次真实搜索、来源 URL、限流/失败状态；没有来源就不声称搜索成功。
3. 把现有 Web 消息、工具事件、选项卡与会话取消映射到 Hermes，明确只有一处管理 Agent 执行。
4. 先接只读地图/状态/巡检点查询，再接带现有确认语义的地图修改。
5. 最后接运动任务，继续通过既有任务入口和原生控制仲裁；用仿真及目标平台证据验收。

本轮没有启动 Hermes API、配置新密钥、安装 DDGS、升级 Hermes 或改变机器人控制路径。

## 更新活跃度与 Codex 订阅核对

2026-11-12 再次查询官方 GitHub API：仓库未归档，最近 push 为北京时间 2026-11-12 01:20；返回的最新提交为 `56ac7b5d51945ae7875c4b669baa757fa17f4846`（北京时间 2026-11-12 00:59）。最近正式发布为 v0.23.1（11 月 10 日），前两次为 v0.23.0（11 月 6 日）、v0.22.7（11 月 4 日）。这是持续维护的证据，不代表本机已升级。

- [最近核对的提交](https://github.com/NousResearch/hermes-agent/commit/56ac7b5d51945ae7875c4b669baa757fa17f4846)
- [v0.23.1 正式发布](https://github.com/NousResearch/hermes-agent/releases/tag/v0.23.1)
- [固定版本的 Provider 文档](https://github.com/NousResearch/hermes-agent/blob/56ac7b5d51945ae7875c4b669baa757fa17f4846/website/docs/integrations/providers.md)

Hermes 的 `openai-codex` 支持通过 `hermes model` 选择 ChatGPT or Codex Subscription，使用 OAuth 登录。其官方文档同时明确：可用 ChatGPT 套餐级别、Hermes 请求如何计入 Codex 限额尚未完整说明。因此不能仅凭支持登录就承诺某一账户的额度资格、余额或具体扣减方式。

本机配置已选择 `openai-codex`，默认模型为 `gpt-5.5`；这只是配置事实。本次没有读取认证文件、触发登录或执行真实推理，尚不能宣称账户授权有效。

[OpenAI 认证文档](https://learn.chatgpt.com/docs/auth) 区分 ChatGPT 登录的订阅访问与 API key 的按量计费；API key 不会自动使用 ChatGPT 套餐额度。[官方开放源代码应用的计划用量共享](https://developers.openai.com/siwc/token-sharing-open-source) 支持符合资格的开源、本地应用请求用户授权使用计划用量，但该机制不能直接证明 Hermes 当前 OAuth 实现已采用同一接入流程。付费或远程托管产品另有接入安排。

接入选择：复用独立 Hermes Agent 服务及其工具生态，保留 LingTu Web 和领域执行入口。完整复用后端仍需适配工具事件、选择与确认、文件、取消和会话；不能承诺功能自动完整映射。模型订阅访问也不自动覆盖第三方搜索、浏览器、语音等服务的费用或授权。

## UI 清理与检查

截图中的“显示”入口和整块弹出菜单已删除；一并删除只由该菜单使用的页面全屏状态/监听器、视角按钮块和样式。相机及图层入口保留，图层中的雷达/点云能力继续存在。

直接相关相机/模式切换 7 项与建图语义 1 项通过，TypeScript/ESLint/正式构建通过。额外跑到的 sceneRobotStatusLayout 中另有 5 个当前页面结构断言不匹配（位置槽、里程标签、导航诊断、旧操作布局、遥控面板位置），本次没有修改其对应业务实现，未将这些断言算作已通过。
