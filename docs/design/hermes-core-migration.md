# Hermes 功能迁入 LingTu

更新：2026-11-12。当前采用按功能迁移，不运行整套 Hermes Gateway，也不把 Hermes 作为模型选项。生产默认模型未改变。

## 轻量化决策

Hermes 是一套完整应用，不是单个模型。我们需要模型认证、工具、记忆、技能和 Agent 行为，但不必同时迁入桌面壳、消息平台和定时调度。

当前唯一持久对话工具循环仍是 OpenAI Agents SDK 0.24.0。迁移 Hermes 的能力时复用现有工具执行、确认、取消与历史所有者，避免再套一个服务端 Agent。本轮未安装任何新依赖，也未将 Host 升级到 Python 3.14。

此前 `HermesRuntime` / `hermes_turn` 桥接已删除：它依赖用户本机 Hermes 安装、由 Host 启动进程、丢弃领域工具，且阻塞 HTTP 请求不能取消远端执行。将这条桥接改名为 core 并不等于源码迁移。以前 `/v1/models` 的探测只证明相应本机安装能启动，不能当作固定 v0.23.1、真实模型推理或机器人任务的验收。

## 已迁入：Codex 登录与直接模型调用

固定参考 NousResearch/hermes-agent v0.23.1，提交 `3b421578588243f920886e6fe79a6faa87ea026e`。

- `src/decision/llm/codex.py`：从 Hermes 设备码请求、轮询、授权码交换和刷新流程适配。
- `src/decision/llm/CODEX_NOTICE.md`、`LICENSE.hermes.txt`：上游文件、差异和 MIT 版权；纳入 Python 包数据。
- `src/decision/llm/conversation_models.py`：将 `codex:gpt-6.1-sol` 与 API Key 的 `gpt-6.1-sol` 区分。
- `src/decision/tasks/sdk_agent.py`：选择 Codex 时直接请求 `https://chatgpt.com/backend-api/codex/responses`，由原 SDK 循环处理工具，`store=false`，没有 Hermes 中转。
- `src/decision/tasks/conversation_inputs.py`：Codex 思考设置、图片输入与授权检查；默认 Kimi 连接不可用时仍允许选择已授权 Codex。
- `web/src/components/agentConversation/ModelConnections.tsx`：底部聊天栏“管理模型连接”，设备码、官方页面、等待、取消、过期、重试和断开。

流程：

```text
点击管理模型连接 → 登录 → 官方页面输入设备码
 → LingTu 换取并保存自己的授权 → 选择 GPT-6.1 Sol · Codex
 → OpenAI Responses → 现有工具确认/调用 → 现有历史和对话卡片
```

登录字段不是聊天消息，不进入模型上下文或分享内容。凭据位于 `$XDG_STATE_HOME/lingtu/model_accounts/codex.json`，未设置 XDG_STATE_HOME 时使用用户目录 `.local/state/lingtu/model_accounts/codex.json`；不读取或修改 `~/.codex`、`~/.hermes`。单 Host 锁负责授权、取消和令牌刷新；令牌先写临时文件再原子替换。退出只删除 LingTu 自有凭据，不等于提供商撤销全部客户端授权。

### HTTP 接口

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| GET | `/api/v1/agent/connections/codex` | 授权状态；不返回令牌，不主动刷新 |
| POST | `/api/v1/agent/connections/codex/login` | 创建设备码会话 |
| POST | `/api/v1/agent/connections/codex/login/{id}/poll` | 按服务器间隔推进授权；成功时保存凭据 |
| DELETE | `/api/v1/agent/connections/codex/login/{id}` | 取消尚未完成的授权 |
| DELETE | `/api/v1/agent/connections/codex` | 删除 LingTu 自有授权 |

接口沿现有 Gateway 鉴权和 Agent Module 入口，不新增公开免鉴权路径。模型连接面板可在模型列表未加载时打开，但后端运行模式仍须提供 AgentPlannerModule。

### 验收边界

授权成功不等于模型权限、额度或推理已验证。`codex:gpt-6.1-sol` 是配置的候选模型，实际可用性由账号和服务器决定。本轮只完成模拟上游的授权状态机、真实 SDK 的 HTTP/SSE 契约、工具调用和 UI 回归；没有对用户账户发起授权或消耗模型额度。

这次复用的是 Hermes Codex 设备码协议，不是 OpenAI 新版 SIWC 动态应用注册。新 SIWC 的 client_id、resource、scope 和推理地址不能混用在此流程。商业分发/远程托管需另按提供商接入安排确认。

## 功能清单

技能配置、Hermes 对照和真实调用证据见 [Agent skills](agent-skills.md)。

- [x] 移除整套 Gateway 启动和文本桥接，不再将其记作核心迁移完成。
- [x] 迁入 Codex 设备码认证、间隔轮询、取消、过期、令牌刷新与本地断开。
- [x] 登录 UI 与后端接口贯通；区分订阅授权和现有 API Key。
- [x] Codex 直接 Responses 调用，保留工具确认、历史、附件和取消语义。
- [x] 保存上游 MIT 许可证、固定提交和本地差异。
- [ ] 用户完成一次官方授权；实际 Codex 对话、一次只读工具、图片和额度错误验收。
- [x] 迁入网页搜索和网页提取，接入工具流、来源卡片、配置/服务不可用状态。
- [ ] 配置真实 Tavily 服务并完成联网搜索、正文提取与模型引用的端到端验收。
- [x] 接入技能目录、按需读取正文和参考文档；用真实 Kimi 工具调用验收。
- [x] 接入助手偏好记忆的查看、保存、修改、删除和每轮上下文；复用既有确认及会话数据库，保留机器人空间记忆的所有权。
- [x] 跨轮上下文压缩、持久摘要、近期原文及旧记录读回；合成历史上的真实 Kimi 摘要／回答／恢复验收通过。
- [x] 建立按能力分类的固定上游清单与只读升级比较工具；大规模变化使用完整 Git 树比较。
- [x] 只读地图／观测助手委派已有代码及事件卡片接入；本地测试覆盖父任务取消传播。
- [ ] 助手委派的真实模型端到端验收、可独立打开的子会话关联。
- [ ] 其他提供商账号配置 UI；Claude/Kimi 当前继续使用既有 API Key。
- [ ] 在完整用例覆盖后再判断是否需要迁入 Hermes 更大范围的 Agent 循环；不把单元测试等同于整套迁移。

### 剩余范围与顺序

1. 上下文管理进阶：跨轮压缩已完成；轮内压缩、真实 token 锚点、提供商原生压缩、后台准备和超限恢复尚未迁入。详见 [agent-context.md](agent-context.md)。
2. 联网真实验收：web_search/web_extract 已接入，仍需可用搜索服务凭据。
3. Codex 真实验收：设备码入口已实测可达，用户授权、真实推理／工具／图片还未闭环。
4. 专业助手：现有只读委派已具备，不等于 Hermes 完整子 Agent 会话、恢复及可打开子聊天。
5. 技能进阶：自动学习／生成技能、技能市场安装、脚本及依赖执行尚未迁入。
6. 通用工具生态：Hermes 的 MCP 客户端插件配置、浏览器操作、终端与文件工作区工具尚未迁入当前持久对话链；LingTu 已有的领域工具和 MCP 服务不等于这部分完成。
7. 定时／后台任务、外部消息渠道、语音等扩展没有作为整套 Hermes 能力迁入；应分别对接现有巡检和运行所有者，不再复制一套生命周期。

本轮偏好记忆说明见 [agent-preferences.md](agent-preferences.md)。真实 Kimi 已完成保存、跨对话读取、修改、删除四步，使用独立验收数据库，结束时记忆为空。

后续升级不按单个文件盲搬：能力、固定提交、上游路径、本地差异和测试入口登记在
`config/upstream/hermes.yaml`，流程与命令见 [hermes-updates.md](hermes-updates.md)。

本轮没有改变导航算法、驱动、ProductControl 或运行模式定义。Python 3.10 的现有 Host 能执行这段迁入代码；没有测量真实机器人上的 CPU/RAM 或端到端延迟，不声称任何性能倍数。

## 本地验证记录

- `test_codex_account.py`、`test_sdk_agent.py`、`test_conversation_inputs.py`、`test_agent_conversations.py`、`test_conversations.py`：93 项通过；随后新增真实 SDK HTTP/SSE 工具调用用例单独通过 1 项。
- `modelConnections.test.ts`、`modelConnectionsUi.test.ts`、`agentConversationUi.test.ts`：33 项通过。
- Ruff、相关前端 ESLint、TypeScript 与 Vite 生产构建通过；构建仍有既有大 chunk 提示。
- 所有授权、令牌和模型响应均由测试提供。未发起真实 OAuth、未使用任何真实账户额度、未证明实机运行效果。

## 网页工具与 UI 验收（2026-11-12）

`src/decision/tasks/web.py` 从同一固定 Hermes 提交的 Tavily provider 适配，
在现有 `AgentPlannerModule` 中注册 `web_search(query)` 和 `web_extract(url)`。
两者为只读工具，不创建运动任务；沿用现有会话历史、单次调用计时和取消。
网页内容只作为参考资料，提示模型使用真实返回的 URL 引用、区分摘要与全文，失败不能声称检索成功。

Host 进程需要环境变量 `TAVILY_API_KEY`。将其放入实际启动 Host 的服务环境；
无需写入 `nav.yaml` 或浏览器。当前环境未设置该变量，因此**尚未证明真实搜索可用**。
HTTP 客户端使用现有 httpx，固定访问 `https://api.tavily.com/search` 和 `/extract`，
没有新增依赖、独立 Hermes 进程或模型循环。

搜索最多返回 5 个来源；每个摘要最多 1200 字符。提取一次读取 1 个网页，最多 16000 字符，
超出明确标记截断。30 秒网络超时；缺少凭据、HTTP 错误、不可读正文和取消各自处理。
原始提供商错误正文不进入聊天，避免把服务端凭据等诊断内容带入历史。

`ToolActivity.tsx` 直接在正式工具消息中显示来源标题、URL、摘要和可展开正文；
长工具链也保留来源可见。错误不会显示为成功来源，原始 JSON 仍可展开检查。
模型连接对话框的关闭按钮调整到右侧。

复现 UI：在 `web` 下执行 `npm run dev -- --host 127.0.0.1 --port 3011 --strictPort`，
打开 `http://127.0.0.1:3011/tests/agent-capabilities-preview.html`。
这是独立测试入口，使用正式 `AgentConversation`、`useAgentConversation`、模型连接和工具卡片，
只在测试页面拦截请求。页面明确标注模拟授权/搜索，不接真实账号、模型或机器人，
也不进入生产打包入口。第二次登录模拟授权成功；三个示例覆盖搜索、提取和未配置状态。
录屏和截图在 `artifacts/ui/20261112-agent/`，不是实机或真实联网验收。

本轮验证：网页工具及现有会话回归 48 项通过；模型连接、对话和来源卡片前端回归 38 项通过。
Ruff、相关 ESLint、TypeScript 和 Vite 生产构建通过；保留既有大 chunk 警告。
录制的是实际浏览器中的正式组件，授权/搜索由测试入口模拟；与真实 Host、真实账户和机器人验证分开。

### 上游来源

### 正式代码的本机实连验证

`python -m tools.diagnostics.agent_console` 启动正式 `GatewayModule` 与
`AgentPlannerModule` 的本机 Host，访问 `http://127.0.0.1:4326/`，页面来自 `web/dist`，
没有替换 HTTP 响应或使用演示 fetch。此工具只开放对话/账号写操作，没有启动机器人 Product；
不能当作机器人端部署。默认连接仍读取 `config/system/models.yaml`。

已通过真实会话 API 调用 `kimi-k3`（low、read_only）：纯文本回复“连接成功。”，
端到端 12.88 秒；随后实际执行 `get_model_connections` 并生成答复，端到端 24.69 秒，
工具本身 4222 ms。耗时为这两次请求的观测值，不代表稳定性能。
证据：`artifacts/ui/20261112-agent/kimi-real-conversation.json`、`kimi-real-tools.json`。
工具返回的配置状态不等于所有列出模型均已成功推理。

修复了默认 Kimi 连接失败时阻止其他提供商/首次 Codex 登录展示的问题。
正式模型连接面板最终改为白底紧凑切换卡片：顶部提供商标签，下方仅挂载当前选中的面板。
删除整列 API 卡片和横向授权进度；验证码/复制、等待状态/有效期、操作按钮分别合并为一行。
切换面板保留正在等待的 Codex 授权；关闭整个弹窗仍取消待完成授权。支持方向键、Home/End 和减少动态效果。
授权提供“前往登录”“复制登录链接”“复制验证码”，并显示可手动选择的完整 URL。
剪贴板错误独立显示，不再阻止设备授权轮询。新卡片包含窄屏和减少动态效果样式。
前端连接服务与面板 10 项测试通过；新增单面板切换、保留待授权状态和方向键验证。
正式 `4326` 浏览器窗口曾停在旧页面，强制重载后已核实新的顶部标签与单面板；
正式页面截图为 `artifacts/ui/20261112-agent/compact-connections-real.png`，切换录屏为 `compact-real-switching-demo.mp4`（2 倍速）。
白底版本截图/录屏：`artifacts/ui/20261112-agent/login-white-authorizing.png`、`login-white-card-demo.mp4`。
正式控制台已发起真实 Codex 设备码并点击官方授权链接；账号授权和 Codex 推理结果待用户完成后记录。
本机服务、账号、模型输入相关测试 29 项通过。

2026-11-12 后续真实连接检查：直接执行 `CodexAccount.start()` 成功取得设备码，
状态 pending、轮询间隔 5 秒，约 2.22 秒后完成请求并本地取消，未保存凭据。
普通 HTTP GET 跳转到 `/oauth/authorize` 后返回 403；同一设备授权地址在 Edge 中
实际打开了 `https://auth.openai.com/log-in`，页面为“欢迎回来 - OpenAI”，邮箱为空。
这证明真实设备码接口和浏览器登录入口可达，**不证明账号授权、令牌交换或模型推理成功**。
脱敏结果见 `artifacts/ui/20261112-agent/codex-live-probe.json`。
演示接口已补齐 OpenAI API、Qwen，并在弹窗内部加入模拟状态提示，避免截取弹窗时丢失说明。

- [v0.23.1](https://github.com/NousResearch/hermes-agent/releases/tag/v0.23.1)
- [OAuth 路由](https://github.com/NousResearch/hermes-agent/blob/3b421578588243f920886e6fe79a6faa87ea026e/hermes_cli/web_routers/oauth.py)
- [Codex 认证](https://github.com/NousResearch/hermes-agent/blob/3b421578588243f920886e6fe79a6faa87ea026e/hermes_cli/auth_codex.py)
- [OpenAI 认证说明](https://learn.chatgpt.com/docs/auth)
- [OpenAI SIWC 范围](https://developers.openai.com/siwc/token-sharing-open-source)
