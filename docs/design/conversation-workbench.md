# 对话式地图工作台与多模型接入

更新：2026-11-11。目标是让底部 Agent 使用现有地图／巡检点服务，保留直接框选和参数编辑。
自然语言表达意图，图形界面提供位置与预览；模型不能猜测几何、机身高度或执行成功。

## 参考项目与采用范围

| 项目 | 对照能力 | 本次做法 |
| --- | --- | --- |
| [OpenAI Agents SDK](https://github.com/openai/openai-agents-python) | 工具循环、流式事件、多模型适配 | 继续使用已安装的 SDK，不再添加一套 Agent 框架 |
| [CopilotKit](https://github.com/CopilotKit/CopilotKit) | 页面上下文、共享状态、人工参与 | 发送选中地图、版本和框选范围；模型结果可在现有视图中预览 |
| [AG-UI](https://github.com/ag-ui-protocol/ag-ui) | 工具开始／结束事件与 UI 反馈 | 接通已有 Agent 事件和持久消息，不另建事件协议 |
| [assistant-ui](https://github.com/assistant-ui/assistant-ui) | 工具结果卡片、附件、审批交互 | 收起原始 JSON，突出工具名称、结果及预览入口 |
| [Pydantic AI](https://github.com/pydantic/pydantic-ai) | 类型化工具、多供应商 | 复用项目的 Pydantic 边界校验，不引入第二运行框架 |

以上为设计参考，本次未复制这些项目源码、未安装新依赖。已有 T3 Code EffortGlyph 的许可文件继续保留。

## 模型协议

| 模型 | 接口 | 思考控制 | 服务端环境变量 |
| --- | --- | --- | --- |
| Kimi | 现有 Moonshot Chat Completions | 保留现有各模型支持的档位及 reasoning_content | MOONSHOT_API_KEY |
| GPT-6.1 Sol | OpenAI Responses，复用 OpenAIResponsesModel | low / medium / high / xhigh / max | OPENAI_API_KEY |
| Claude Sonnet / Opus / Haiku 5.5 | Anthropic 官方 OpenAI 兼容入口 | 默认；兼容入口不支持实际 effort 调节，不提供虚假档位 | ANTHROPIC_API_KEY |

官方资料：

- [GPT-6.1 Sol](https://developers.openai.com/api/docs/models/gpt-6.1-sol)、[Function calling](https://developers.openai.com/api/docs/guides/function-calling)。新 GPT 工具调用使用 Responses。
- [Claude 模型与 API](https://platform.claude.com/docs/en/claude_api_primer)、[OpenAI SDK 兼容](https://platform.claude.com/docs/en/cli-sdks-libraries/libraries/openai-sdk)、[Thinking](https://platform.claude.com/docs/en/build-with-claude/thinking)。兼容 URL 为 `https://api.anthropic.com/v1/`；省略 temperature 和强制 tool_choice；reasoning_effort 不生效，不能据此提供思考深度。原生 Messages 的完整思考、缓存能力不属于本次兼容接入。

聊天新增供应商的模型、URL、密钥环境变量名集中在 `src/decision/llm/conversation_models.py`。
Product 默认 LLM 仍由 `config/system/models.yaml` 解析到 RunPlan。聊天选择只影响当前轮，不改 Product 默认连接。
密钥只在服务端配置，不进入浏览器，也不要粘贴到聊天。未配置的模型显示为不可选。

可继续接入的候选（本次研究，不冒充已验证）：

- [Gemini OpenAI compatibility](https://ai.google.dev/gemini-api/docs/openai)：视觉／工具能力；需验证跨轮 thought signatures。
- [Qwen function calling](https://www.alibabacloud.com/help/en/model-studio/qwen-function-calling)：聊天目录已接入 Qwen Plus、Qwen Flash、Qwen3 Max，默认使用北京 `https://dashscope.aliyuncs.com/compatible-mode/v1` 与 `DASHSCOPE_API_KEY`。Plus/Flash 支持默认／快速／深度思考；本次按文本模型开放，不接受图片。无凭据时菜单可见但不可选；尚未做真实 Qwen 推理验收。

### Bot 角色与形象的当前边界

消息接口支持可选 `persona`（最多 2000 字符）：角色偏好随单轮消息保存，重复请求必须保持一致，
并传到 SDK 的角色说明中；不改变服务端工具权限、确认和运动准入。对应单元测试使用模拟模型。
Web 对话头像可点击打开“我的 Bot”：修改名字、三种形状、四种颜色、角色说明及当前对话的工具权限。
形象与角色保存在此浏览器；新消息读取配置，重试沿用原消息快照。工具权限沿用现有会话设置，
不由角色说明决定。活动回合或待重试消息期间禁止修改，避免界面配置与已提交请求不一致。

小头像跟随真实的待命、处理、等待确认、完成和不可用状态；展开的 SVG 角色支持鼠标追视、
点击或键盘挥手，装饰互动不改变任务状态。支持窄屏和减少动态效果。当前没有骨骼 3D 模型、
语音口型、实时语音或跨设备形象同步。

参考 [Grok Bot 配置](https://docs.x.ai/grok-bot/bots)、
[设置](https://docs.x.ai/grok-bot/settings-and-notifications) 与
[动态头像说明](https://x.ai/changelog/bot)。REA 访问 x.ai/bot 时被 Cloudflare 阻挡，
未取得可用于视觉比对的产品界面；本实现为依据公开交互说明设计的原创 LingTu 角色，不声称精确复刻。

2026-11-11 验证：前端构建与相关 ESLint 通过；角色、消息、界面单元测试共 44 项通过。
正式本机页面 `http://127.0.0.1:4326/` 在隔离浏览器中验证保存、取消、刷新、鼠标互动、390px 布局、
减少动态效果及消息参数。创建会话和消息请求使用拦截响应，没有真实模型或机器人调用。
脚本与截图：`artifacts/ui/campus-geography-20261111/bot-studio-check.mjs`、
`bot-studio-results.json`、`bot-studio-appearance.png`、`bot-studio-role.png`、`bot-studio-mobile.png`。
- [DeepSeek tool calls](https://api-docs.deepseek.com/guides/tool_calls/)：思考工具循环需保留其推理字段，不能只改 URL 就宣称兼容。
- [Ollama OpenAI compatibility](https://docs.ollama.com/api/openai-compatibility)：本地模型入口；工具能力取决于模型，无 GPU 时速度需按机器实测。

## 工具与归属

`Gateway → 注入有界工具 → Agent SDK → Gateway 原有地图/巡检点服务 → mapd/位置存储`。
decision 不导入 gateway，不给模型通用 HTTP、Shell 或直发速度工具。

| 工具 | 行为 |
| --- | --- |
| list_saved_maps / inspect_saved_map | 查询地图、实际处理状态及区域 |
| find_inspection_points | 按地图中的名称、区域名、标签检索巡检点 |
| preview_map_edits | 裁剪、清洗、补面的原生预览，只向模型返回数量和操作参数 |
| save_map_copy | 另存新地图，原图保留；是否可导航以实际处理结果为准 |
| set_navigation_region | 保存具有准确三维边界的玻璃／禁区 |
| save_inspection_point | 保存点名、所属区域、机身坐标、停留时间和已有支持的动作 |

只读模式禁止持久修改和运动；默认模式逐次显示动作确认；完整权限模式跳过交互审批，但不能绕过地图版本校验和原有运动控制。
超时不代表保存撤销；先查真实地图状态，不自动重试写入。
地图上下文纳入消息幂等比较，同一个 message_id 不得换地图或选区重放。

## 验证与范围

### 只读多 Agent 协作（2026-11-12）

沿用 openai-agents==0.24.0，通过管理者的 FunctionTool `review_with_specialists(task)` 调用两个独立 Agent/Runner。
这是 agents-as-tools 编排模式的项目适配，不使用 handoff 转移主会话，不新增框架或依赖。
地图助手仅获得 list_saved_maps、inspect_saved_map、find_inspection_points；观测助手仅获得 get_robot_capabilities、
read_robot_sensor、inspect_lidar、inspect_navigation、get_agent_status，并且排除声明 writes 的工具。
每个工具调用仍经过原执行器校验，调用 ID 加子任务前缀。子助手无 Python、写入、运动、ask_user、done 或委派工具。
每回合最多一批、两个子助手并行；每个最多 4 个 SDK turns、45 秒，父回合超时更短时以父回合为准。
子任务取消会取消并等待其 SDK Runner，部分失败保留另一份结果。子助手使用当前选中的模型，但没有完整历史／附件，
由主助手传入具体问题和必要地图 ID；不能臆造现场上下文。

开始／完成／失败／取消通过既有事件回调存入同一条 tool 消息，刷新后可恢复。主助手获得 reviews 汇总后继续生成最终回答。
可用提示：“请地图助手和观测助手协作检查当前地图的巡检条件，只读查询，不执行动作。”简单查询无需协作。
代码入口：src/decision/tasks/specialists.py、sdk_agent.py；持久化：agent_planner.py；展示：ToolActivity.tsx。
使用真实 SDK + 模拟模型离线验证，不代表付费模型或实机已验收；界面演示在静态预览 4327 使用拦截快照。
本轮更新源码与 web/dist，未重启共享 Gateway；后端须下次启动载入。启用协作会增加模型调用量。

正文中的模型名称使用行内品牌标识：只装饰已转义 Markdown 的普通文字节点，匹配固定产品名并引用已有本地图标。
不处理代码块、行内代码、网址、文件路径或 HTML 属性；保留原链接和文本，图标为空替代文字并对读屏隐藏。
该标识不表示模型已配置、正在运行或通过权限校验。正文复用全局 `selectable` 样式以保留选段引用。
本轮仅启用在助手正文中，未修改工具步骤、Bot 或模型选择协议。

2026-11-12 继续扩展识别层：增加 Google Gemini、DeepSeek、Grok、Mistral、Meta/Llama、Perplexity、MiniMax、GLM/Zhipu、
Cohere、Baidu/文心一言等公共 favicon；Logo 带轻量悬停说明“仅用于识别名称”，不声明模型已配置。
参考 [Codex TUI 的活动摘要与展开](https://github.com/openai/codex/blob/main/codex-rs/tui/src/exec_cell/compact.rs)、
[assistant-ui 的结构化工具 UI](https://github.com/assistant-ui/assistant-ui/blob/main/apps/docs/content/docs/tools/tool-ui.mdx)、
[HuggingChat 的工具状态展示](https://github.com/huggingface/chat-ui/blob/main/README.md)。现有对话还保留业务卡片、协作卡片、图片放大、消息回应、选项提问和工具步骤；本轮未新增后端模型权限。

2026-11-12 解析错误显示修复：现场历史含 `Unknown content: {... output_text ...}` 的 failed 回合，
曾被当成 Markdown 正文渲染。当前跨模型历史转换已有将 assistant 内容块转为文本的处理，
通过真实 SDK converter 回归。SDK 新增解析错误的简短用户文案，原异常保留在 failure_reason；
Web 同时兼容旧 failed 消息，将异常按字面文本放入默认折叠的“错误详情”，不改原数据、不改成功状态。
只读访问实际历史会话验证了折叠、完整异常和窄屏布局，证据为
`artifacts/ui/campus-geography-20261111/parse-failure-results.json`。
本轮没有重启共享后端；新增 SDK 兜底随下次启动载入，Web 构建已在本机正式页面生效。

离线测试覆盖供应商路由、参数、缺失密钥、跨模型历史、上下文幂等、工具权限及 UI 构建。
GPT/Claude 的真实认证调用需要对应服务端密钥，不能用离线通过代替联网结果。
实机运动、相机采集动作和导航地图可用性仍需各自验收。
任意倾斜补面、自由多边形区域、自动识别玻璃及通用 UI 点击代理不在本次实现中。


### 本地验收记录

- 2026-11-11，正式 Gateway + 原生 mapd + Kimi K2.6：只读调用 list_saved_maps，返回 4 张地图，回合 completed，无 run_id（没有创建运动任务）。证据：`artifacts/web/agent-workbench-20261111/live-readonly.json`。
- 浏览器首屏、全点地图、多模型菜单、未配置模型不可选、真实工具结果卡片已现场查看。首屏空地图／空 Agent 预览的判断已修复并补入 mapGalleryEditing 回归场景。
- GPT/Claude 两轮工具循环通过本地真实 SDK + 离线模型测试；本机对应密钥未配置，未声称真实认证调用成功。
- 文字／图片附件按所选供应商处理；PDF／Word 提取当前仅对 Kimi 开放，不向其他供应商暗中转发。
- 本地入口：`http://127.0.0.1:4326/?tab=map`。未部署机器人端，也未执行机器人运动。


### 完成项

2026-11-11 对话卡片交互补充：

- 回复下方保留复制、引用追问、重新生成；移除“简短一点／展开说明”及空的更多操作入口。技术细节归入工具记录。
- 过程行遵循用户提供的 Codex 截图：透明底色、灰色文字与分类图标，展开后看详情；取消常驻胶囊背景和嵌套竖线。查询／编辑／命令／图像使用对应图标，运行中保留动态指示、失败保留错误色。正文链接采用蓝色，悬停显示下划线，外部链接附方向标记。浏览器验证折叠、放大、复制、焦点恢复、窄屏及链接样式；截图 `artifacts/ui/campus-geography-20261111/gray-process-links.png` 为测试记录。
- 聊天排版继续收紧：正文与行内代码共用 UI 字体和字号，重点用深青色，行内标识用紫色，链接保留蓝色；消息间距 8px、段间距 5px。工具行和行内详情均透明。Bot 入口在标题栏右侧以 26px 图形显示，空闲时删除底部状态整行；只有工作／待确认／异常保留紧凑状态，形象设置入口继续可用。触控入口保留 44px 点击区域。
- 重新生成只对最新回答开放，保留原回答，作为新只读回合提交；不改变用户选中的会话权限。草稿、附件、待确认请求、运行中的回合或任务存在时禁用。网络失败重试复用请求 ID、角色、权限和原回复引用。
- 工具记录变为可展开标签，区分已发起、调用中、待确认、已返回及未完成。失败原因直接显示，原始结果按需展开；工具已返回不等于任务成功。
- 当前工作文字来自请求、上传、工具记录、审批和回合状态；处理时轻微流光／旋转，审批等待与结束时停止，遵守减少动态效果。
- 消息、工具记录、输入区和标题间距收紧；未改 Bot 形象实现。
- 细节打磨：结束回合中连续、同轮的普通工具记录折叠为一条过程栏；运行中、错误或带地图预览操作的记录保持可见。回复操作靠左、时间靠右，复制成功反馈 1.8 秒后恢复。
- 2026-11-12：工具记录支持行内详情与放大窗口。命令型结果按已有 `command`/`cmd`、`stdout`/`output`、`stderr`、`cwd`、`exit_code` 字段显示；其他结果保持原文或格式化 JSON。复制保留服务端原始结果，放大时继续更新，Escape 逐级关闭并恢复焦点，非零退出码留在摘要。此处仅增加展示，不新增 Shell 工具或执行权限。相关 53 项测试、构建、ESLint 与隔离浏览器验收通过；命令截图为拦截测试记录，没有实际执行命令。证据：`artifacts/ui/campus-geography-20261111/tool-details-results.json`。

开源参考核对：`openai/codex` 提交 `e58bf8f5233b51e7cd1679d947e3028c1ed91b06` 的
[紧凑命令摘要](https://github.com/openai/codex/blob/e58bf8f5233b51e7cd1679d947e3028c1ed91b06/codex-rs/tui/src/exec_cell/compact.rs)、
[活动展开机制](https://github.com/openai/codex/blob/e58bf8f5233b51e7cd1679d947e3028c1ed91b06/codex-rs/tui/src/transcript_view/disclosure.rs)。
它将命令／有限输出预览与完整详情分开，按保留的活动身份管理展开状态，支持鼠标和键盘。
这是 TUI 源码证据，不是 Codex 桌面 App 的前端源码。本实现借鉴交互结构，未复制 Rust 实现。
- 本轮使用正式本机页面与拦截的测试快照验证 UI、复制、标签、只读重新生成及网络重试；无真实模型或机器人调用。截图为测试会话，不是实机执行记录。证据：`artifacts/ui/campus-geography-20261111/conversation-interactions-results.json`。

- [x] GitHub／官方 API 对照与能力归属记录。
- [x] 7 个地图／巡检点工具接入正式会话链及已有权限模式。
- [x] GPT Responses、Claude 官方兼容接口、Kimi 原有协议按轮选择。
- [x] 当前地图／选区上下文与幂等重试。
- [x] 工具结果卡片、图形预览入口、模型配置状态及附件供应商选择。
- [x] 本地构建、只读实调用与浏览器查看。
- [ ] 配置 GPT／Claude 服务端密钥后，验证真实账号的模型权限与工具调用。
- [ ] 对话驱动编辑的完整人工验收：预览 → 确认 → 另存／标注 → 检查保存结果。


## 2026-11-12 T3 Code 第一批增量迁移

基准仍为 `fc6c7c8b27fc94e6486ca1b589ae138ef259f8f5`。本轮包含实时正文、历史正文搜索与可控地图上下文；不包含 token/费用统计、计划模式、会话分支或远程通知。

- 正文增量经 Agent owner 持久化，首块立即显示，后续短片段合并更新；SSE 复用既有 `agent_message`，携带消息身份、递增 revision 与完整累积正文。Web 合并消息，不把片段重复追加；原每秒快照用于断线与刷新恢复。完成、失败、取消和 Host 重启收束生成状态，真实工具/任务终态仍使用原回执。
- 局部迁入 T3 `markdownStreamBlocks.ts`，保留 MIT 与提交来源；普通文本块通过 React memo 保留，避免每次增量重建已完成段落。现有 Markdown 渲染器的标题编号、代码围栏与跨块语法保持全文渲染，图片放大、模型标识、安全转义与引用选择沿用原逻辑。
- 会话列表搜索查询 SQLite 中用户与助手正文，返回至多 30 条命中及文本摘要，支持当前/归档列表。命中可加载超过最近 200 条之外的历史，并滚动聚焦对应消息；查询失败可见，迟到的旧查询不覆盖新结果。搜索不读取工具原始日志，不提交模型请求或机器人动作。
- 输入框上方显示地图来源、内容版本与三维选区。可分别移除地图或选区，也可重新附加；只影响新请求携带的 `map_context`，不改变地图及选区自身。开关随各会话草稿保存；不确定结果的重试保留原消息 ID 和提交时上下文。

API 与 SSE 字段见 [接口契约](../api.md#conversation-search-and-live-text-2026-11-12)。源码接线、离线检查和运行实例是否重载分别记录；本地检查不代表真实模型或机器人验收。

本轮验证：Python 会话、搜索、SDK 假模型、流式终态与 SSE 契约合计 100 个不同测试通过；Web 状态、界面、Markdown、分享、接收页与模态框共 111 项通过。改动文件 Ruff/ESLint、TypeScript、Vite 构建及生成 API 文档检查通过，独立源码审查未发现阻断缺陷。T3 MIT 声明也随 Web public/dist 保留。未重载共享 Host，未运行真实付费模型、机器人或浏览器视觉验收；后端新增搜索与实时正文逻辑随 Host 下次启动载入。

## 2026-11-12 会话分支

已结束轮次的最后一条助手回复提供“从这里开分支”，新建独立可继续的会话，并保留来源及“回到原会话”入口。分支复制截至该回复的已终结公开问答与相关上传附件，引用 ID 同步映射，明确摘录在原引用不属于前缀时仍可保留。工具原始日志、私有模型记录、运行任务、排队项、审批、分享链接和上下文压缩摘要不复制。

边界使用公开消息前缀及选点完成时间，不以工具消息的 seq 截断流式回复；按原有执行顺序复制并重新编号。引用与附件会进入后续模型的可移植历史，不重放源工具调用。复制在一个 SQLite 写事务内完成，失败整体回滚；原会话、未发送草稿和正在执行的工作保持原状态。

`POST /api/v1/agent/conversations/{id}/fork` 的请求为 `{message_id}`，返回新会话元数据及 `forked_from` 来源。Web 在读取新快照成功后切入分支；请求失败时仍保留原会话，不自动重复创建。新分支没有待重试请求或附件草稿，使用当前模型/思考设置和默认工具权限；从归档会话创建的分支进入未归档列表。

分支保留的是公开讨论上下文，不是供应商内部推理、工具运行状态或机器人现场的回滚。旧记录缺少完成时间时仅用公开消息边界；无法从已被后续修改的外部回复重建当时全文，未明确保存的外部引用不引入未来内容。

验证：分支、已有会话、流式、方案与 Gateway 回归 101 项通过，另有原文本/原生图片附件输入 2 项通过；Web 会话状态与界面回归 95 项通过。Ruff/ESLint、TypeScript 与生产构建通过，独立源码审查未发现阻断缺陷。测试覆盖超过 200 条历史、引用/附件、队列与流式交错、两种中途写入失败整笔回滚、源 active owner 保留、分支续聊无旧动作重放、归档来源及迟到旧响应。源码和前端 dist 已更新；未重载共享 Host、未调用真实模型或机器人。

## 2026-11-12 结构化方案成果卡

主会话增加 `propose_plan`，输出 `{title, summary?, steps:[{title, detail?}]}`，步骤限制为 1–12 条。结构在工具入口校验，拒绝执行状态及权限字段；专家子 Agent 不暴露此工具。成功结果使用同一条助手消息的 `kind=plan`、`metadata.plan` 与完整 Markdown 正文持久化，保留现有 SSE、搜索、分享及只读接收页链路。

输入栏“＋ → 制定方案”只追加一条草稿提示，保留原输入，不自动发送。成果卡默认收起，展示标题、摘要及步骤数；展开查看编号步骤，可复制原始 Markdown 或下载 `.md`，剪贴板失败时提供可选中原文。生成回合结束不等于执行步骤，卡片固定标注“方案草稿 · 未执行”，不添加执行或审批按钮，机器人动作仍归原有所有者与准入。

分享改进已经同步到生产组件与前端构建：标题栏保留 Bot 和纯图标入口、原深靛蓝流光卡片、生成后查看会话、独立只读阅读页和浏览器保存 PDF。流程演示使用实际组件和显式演示数据，不代表真实机器人执行或付费模型验收。

本轮验证：SDK、方案与既有流式/会话后端 105 项通过，Web 状态、方案卡、界面、Markdown、分享、阅读页及模态框 118 项通过；Ruff、ESLint、生产构建和生成 API 文档检查通过。独立浏览器实际操作验证 Bot/纯图标入口、首次生成即复制并显示绿色勾、打开只读会话/PDF入口、方案展开/复制/下载，无页面脚本错误。录屏 `C:/Users/99563/Documents/Codex/2026-11-10/new-chat/outputs/lingtu-share-card/lingtu-share-and-plan.mp4` 为 1280×900、约 22 秒，验收记录为同目录 `recording-checks.json`。共享 Host 未重载，新增方案后端随下次启动载入。

## 2026-11-11 卡片交互精修

- 工具执行在 Agent 调用边界计时，用同一个消息 ID 更新调用中和返回状态，耗时随消息持久化。计时不包括模型生成，包含工具等待（含人工确认）；旧记录无时间数据时不伪造数值。
- ask_user 可返回 2–4 个简短选项；UI 显示 A/B/C/D 和“其他”。点击作为带 reply_to_message_id 的正常回复提交；不绕过现有工具审批，不覆盖输入框里的草稿，不自动重放动作。
- 对话定位点只滚动到历史提问，不删除历史、不撤销地图操作；保留回到底部入口。
- 地图铺满工作区；常态模式服务不可用留在操作入口原因中，不再反复占用全宽通知条。真正的模式切换过程和失败仍显示。
- 统一正文／表格／行内代码字族，细浅色无箭头滚动条；新增大对话工作区开关。
- 卡片采用约 190–240 ms、小距离移动和淡入；地图侧栏约 300 ms。系统减少动态效果时停用这些动画。

参考：[assistant-ui 工具时间线](https://www.assistant-ui.com/elements/tool-timeline)、[对话定位](https://www.assistant-ui.com/elements/scroll-anchor)、[工具状态与计时](https://www.assistant-ui.com/docs/api-reference/tools/status)、[Motion 无障碍动效](https://motion.dev/docs/react-accessibility)、[MDN 滚动条](https://developer.mozilla.org/en-US/docs/Web/CSS/Guides/Scrollbars_styling)。本轮使用已有 React/CSS，没有新增依赖或通用插件授权框架。GitHub 等授权卡片必须等真实 OAuth 服务接入后展示真实连接状态。


2026-11-12 验收补充：跨模型历史的 assistant/output_text 已转换为 SDK 可重放的文本内容，并通过实际 SDK Converter 与同会话联网续问验证。连接卡片按供应商归组，避免每个模型都占一张重复卡。实测、失败原始记录和修复后结果见 `artifacts/web/conversation-cards-20261112/README.md`。


本轮交互完成项：

- [x] 重复断连通知收敛、地图铺满及可展开的大对话工作区。
- [x] 统一字体、4 px 细滚动条、轻量卡片动效及减少动画支持。
- [x] A/B/C/D 与其他回复，通过真实 ask_user 和会话回复接口连接。
- [x] 工具同卡更新、独立服务端计时与持久化。
- [x] 对话定位点，修复定位后被自动滚动拉回底部的问题。
- [x] 按供应商合并的模型连接卡片及已配置模型选择入口。
- [x] 本地真实 Kimi 问题卡、连接查询、跨模型后续对话、浏览器验收。

## 2026-11-12 轻量引用与步骤状态

- 引用默认显示为一行摘要，无常驻底色；原生展开控件显示全文，取消引用保留问题草稿。
- 工具记录用细竖线串联。待确认或未收到返回的记录保留灰勾，当前调用显示转圈，返回后变绿，错误显示红色提示。状态直接来自现有消息与调用结果，绿勾表示调用返回，不替代导航任务的到达状态。
- 三项以内的短过程保留展开；更长的已完成过程可折叠。错误和需要预览的结果继续直接显示。
- 新完成的勾使用短描边动画；系统减少动态效果时停用。未新增前端自动推进的正式进度或额外依赖。
- 确认卡片桌面宽度为 340 px，窄屏随容器收缩；计时与操作在同一行。工具勾按短笔画、长笔画顺序描边，并有一次轻微绿色反馈。
- “+”打开独立的添加菜单组件，接入机器人能力草稿和文件／图片选择；Bot 配置仅从右上角 Bot 入口打开，历史入口在标题栏。默认思考强度只隐藏文字，保留滑杆图标与菜单，未修改模型默认参数；执行前确认的盾牌为绿色。
- 本轮经本机 Gateway 使用 Kimi K3.1 Pro Highspeed 完成两项只读调用：`list_saved_maps` 163 ms、`get_model_connections` 275 ms，整轮为 completed。记录见 `artifacts/web/conversation-cards-20261112/live-tools-toolbar.json`；未验证地图写入、机器人运动或未配置供应商。
- 添加面板锚定在输入区上方，与聊天容器左右对齐；按上方可用空间限制高度，菜单内部滚动，不再向下展开。行标题 13 px，辅助说明 12 px 且位于标题右侧，单行省略。Bot 配置和误指向 Bot 的“更多机器人设置”已从组件菜单删除。
- 执行状态文字与当前工具名称使用轻量流光；右上角 Bot 保留工作时的轻摆、扫视和完成表情，并在工作时显示轻微呼吸反馈。等待确认与结束时停止持续动效，减少动态效果设置关闭动效。它们反映会话和工具状态，不代表额外的模型推理文本流。

### 2026-11-12 首批业务结果组件

- `list_saved_maps` 的真实工具返回接入紧凑列表：地图名、record.state、size_mb，展开显示激活条件、版本和 activation_blockers；缺失状态显示未知，不推断可用。
- save_map_copy 审批显示原地图、目标名称和修改项数；原始参数可展开。已允许／已拒绝／已关闭回执保留，只有 pending 显示决策按钮。批准不等于执行成功。
- 参考 assistant-ui ApprovalCard 的请求/回执分离和 DataTable 的结构化结果模式；本轮为现有组件适配实现，未复制上游源码或引入组件包。
- 构建、ESLint、30 项界面测试通过。`artifacts/ui/campus-geography-20261111/business-results-check.json` 记录隔离浏览器验收；图中数据与审批请求均拦截，无地图写入或机器人操作。
- 本轮未接入地图列表跳转三维地图，未改机器人后端。正式 Web 构建已更新；测试使用独立静态预览 4327，因为验收时 4326 未运行。


### 2026-11-12 聊天内点云编辑预览

- `preview_map_edits` 的 `preview_request` 接入独立三维卡片；不必先打开主地图。通过既有 `edit_preview` API 获取几何，保持地图版本、坐标系和操作顺序校验，使用已有 MapGalleryScene 按高度着色。
- 点击“查看三维”才请求数据并按需加载三维组件。单张卡片挂载期间复用已取得的结果；收起时截取当前视角、卸载三维视窗，重开无需再次请求。截图可下载，本轮不持久化到服务端历史。
- “在主地图查看”仅在当前地图名称、版本匹配时可用，直接传递同一份预览与编辑操作，不重新计算或保存地图。卡片标明“处理后 · 尚未保存”，不把预览完成表示为保存成功。
- 90 项相关测试、改动文件 ESLint、TypeScript/Vite 正式构建通过。浏览器以合成房间点云验证独立查看、旋转、缩放、截图、收起重开单次请求、主地图复用、版本不匹配拒绝、空结果和 390 px 窄屏；无浏览器脚本错误。
- 验收脚本、截图和结果：`artifacts/web/conversation-cards-20261112/verify-cloud-preview.py`、`cloud-preview-inline.png`、`cloud-preview-mobile.png`、`cloud-preview-checks.json`。几何响应为拦截的测试数据，不证明真实地图清洗效果或机器人运行；本轮没有地图写入。


### 2026-11-12 机器人观测与分析工具

- 正式对话工具新增 `get_robot_capabilities`、`read_robot_sensor`（imu/joints/motors/voltages）、`capture_camera`、`inspect_lidar`、`inspect_navigation`、`run_python`。它们与原有地图工具通过同一 `workbench_tools` 注入 Agent，沿用调用计时、持久化、失败显示及权限模式。
- 能力发现读取当前选定的观测适配器描述，不扫描网络、不猜设备接口。`configured` 表示存在来源，实时可用性须读取样本确认。Brainstem 的 16 关节/电机契约留在其适配器；已有 HostBus JointState 保留实际名称与数量，支持 Go2 的 12 关节消息。没有接入的 IMU/电机等来源返回不可用，不将里程计冒充 IMU。
- 适配器提供 `capabilities()`（provider、sensors、字段/布局/时钟）及 `sample(sensor, timeout)`（标准单位下的观测值）。硬件转换与校准仍由适配器负责；Agent 根据能力选工具，不根据未知接口自动生成和加载驱动。Brainstem 观测转换与原有 gRPC 接口共用一份实现。
- 相机复用 JPEG 快照，明确可能来自缓存且没有报告拍摄时间；视觉模型通过 SDK 图片输入获得像素。Chat Completions 的工具消息不能携带图片，模型输入过滤器将图片放入关联的用户图片内容；历史仍保留原始工具结果，切换文本模型时不发送这些图片。
- LiDAR 按需订阅现有 scan，2 秒内取一帧后关闭订阅，保留 frame/epoch/sequence/source stamp；聊天最多显示 4096 点。显示扫描不是原生碰撞图或接触检测；停车原因来自原生导航状态。未新增导航准入、碰撞或制动判断。
- Python 使用当前 Host 解释器、临时目录及本轮成功观测的 `data`，10 秒超时，stdout/stderr 各最多展示 64 KiB；取消/超时终止本次进程树。只读模式拒绝执行，默认模式展示代码并审批。它是受信任操作者的 Host 代码执行，不是隔离沙箱；不提供设备直连或后台任务工具，临时文件不持久化。
- 前端新增观测卡片：IMU 单位与数值、关节/电机表格、电压、相机图片及按需三维扫描。“+”增加传感器、相机、雷达与导航的草稿入口，点击不会直接提交或执行。
- 浏览器使用合成测试观测验证展示、12 关节数量、扫描卸载、分析输出和窄屏。证据在 `artifacts/web/robot-tools-20261112/`；后端验证包括真实本机 gRPC 与真实 Python 子进程，模型协议使用离线 SDK 模型，不代表硬件或真实视觉推理验收。

待确认与连续卡片：输入区上方只列 snapshot.approvals 中 pending 的项目；点击定位并聚焦原卡片，不发决策请求。确认记录按服务端顺序连接显示，批准/拒绝回执保留，提交中的脉冲仅跟随 approvalBusy；减少动态效果关闭动画。审批列表只表示等待许可；消息排队使用下述独立 queued 状态，不把模型建议或已批准项称为待执行消息。33 项 UI 测试、构建、ESLint 与拦截浏览器验收通过。


### 2026-11-12 T3 Code 队列与会话交互迁移

参考仓库 `D:/inovxio/references/t3code`，提交 `fc6c7c8b27fc94e6486ca1b589ae138ef259f8f5`（MIT）。沿用 LingTu 的 Python、SQLite、OpenAI Agents SDK 和 React；没有引入 T3 的 Effect、Provider 服务或新依赖。

- Web 提交带 `enqueue: true`。当前执行轮次未结束时，后端持久化为 `queued`；同一会话按接纳顺序领取执行，重复 message_id 不重复执行。已排队内容不进入当前模型上下文。取消/状态查询等原有直接控制指令仍沿原控制路径；摇杆速度不进入对话队列。
- `PATCH /api/v1/agent/conversations/{conversation_id}/queue/{turn_id}` 修改文本；`DELETE` 取消待执行消息。开始执行后返回 409，不把“取消排队”误当作停止正在运动的机器人。权限、模型、引用和附件保持提交时的值。
- 输入区上方显示三条待执行消息，更多折叠；可以编辑、取消或设为下一条；提前处理不打断当前执行。编辑草稿按会话持久化。任务在编辑期间离开队列时，保留未发送的修改，可移回空草稿；不会覆盖已有草稿或自动发送。
- Host 重启把运行和排队轮次标记为 interrupted，不自动恢复旧动作。主动取消当前对话也取消其待执行轮次。
- 最新快照最多读取 200 条常规消息，另外保留待执行消息；`history_cursor` 用于 `GET .../{conversation_id}?before_seq=N` 加载更早消息。条目 revision 阻止迟到的历史页覆盖新工具结果，mutation epoch 阻止动作之前发出的轮询覆盖新回执。轮询失败后继续重试，不重发已受理消息。此实现是快照恢复，不是另建 WebSocket 事件日志；turn/approval 元数据及单条大消息目前未做字节级分页。
- 对话滚动记住本次页面内各会话的位置；向上浏览时停止跟随，回到底部恢复。回到底部按钮延迟 150 ms 出现；动态工具内容尺寸变化在跟随状态下保持到底部。加载早期历史保留阅读位置。
- 审批继续使用现有 conversation_id + approval_id + turn_id 绑定、完成/重启取消 pending 的机制。现有地图和观测子 Agent 仍只读，不能发起写入审批；没有伪造子会话或扩大子 Agent 权限。未加入打断当前模型的 steer；队列提前处理使用持久化 queue_order，后续模型上下文按实际执行顺序整理。

验证：后端队列、对话和 HTTP 测试共 55 项通过；前端对话测试 72 项通过，生产构建及改动文件 ESLint 通过。浏览器脚本 `artifacts/ui/campus-geography-20261111/conversation-queue-check.mjs` 验证提前处理、编辑、取消、折叠、保留滚动和移回草稿，所有写请求拦截到夹具，不调用模型或机器人。交互截图 `conversation-queue.png`。Gateway 新协议需要重载 Host 才能在现有运行实例生效；本轮未重启共享 Host 或部署机器人。


### 2026-11-12 IMU 侧栏与持久记录

- 聊天 IMU 快照提供“打开实时监测与记录”，“+”菜单提供 IMU 实时监测。Agent 的 `open_imu_monitor` 也可打开侧栏；重载历史不会重放旧的开卡操作。桌面聊天让出侧栏位置，手机为可关闭的侧边卡。
- 同一个选定适配器的 `stream("imu")` 为侧栏与记录提供数据。浏览器按序列增量读取，显示 15 秒、1 分钟或 5 分钟窗口，最多保留 30,000 条。暂停只冻结画面；关闭无记录的卡片后释放源订阅并结束空闲线程。
- Record 在 Host 端写入 `LINGTU_RECORDING_ROOT/imu/*.jsonl`，默认 `~/data/lingtu/recordings/imu/`。文件包含来源描述、开始信息、完整收到的规范化观测和结束信息，不受浏览器显示上限影响。原生 DDS MCAP 录制保持原有归属，不把 `/imu/raw` 与机体 IMU 混为一份来源。
- 开始重复请求返回同一个正在记录的 ID；停止必须携带该 ID，旧请求不能停止别的记录。关闭卡片或浏览器后记录继续，重新打开恢复状态；Host 停止记为 interrupted，缺少结束信息或残留半行的文件按中断记录恢复完整行。
- 记录列表支持曲线回看、播放、拖动时间位置、返回实时、CSV 和 JSONL 下载。CSV 包含来源时间、接收时间、间断标记、角速度、加速度和四元数；缺失测量留空。正在记录的文件停止后才能下载。
- 曲线按像素保留极值，历史按桶保留首尾及各轴极值。源时钟回退、重连、被省略的间断和字段缺失变化不画成连续测量。历史 summary 对完整选定区间计算 count/min/max/mean/RMS，不对压缩曲线算平均值。
- 分析当前时间段会追加聊天草稿，保留原草稿，不自动发送。Agent 可调用 start_imu_recording/stop_imu_recording/list_imu_records/read_imu_record，写操作沿用审批权限。记录采样结果供本轮 run_python 使用；模型文字不塞入整份曲线数组。
- 记录操作错误与实时连接状态分别持有，避免正常刷新覆盖保存失败。证据见 `artifacts/web/imu-monitor-20261112/`。本轮为本机合成源、真实服务和文件读写验证，不代表 NX、MuJoCo 或真实 IMU 运行验收。

- 浏览器整进程关闭验收：记录从关闭前 38 条增长到关闭后 281 条；新浏览器恢复同一 ID 后停止并保存 396 条。开始/停止请求若已执行但响应丢失，会用后续权威记录状态确认结果；普通健康刷新不会覆盖真实写入错误。


### 2026-11-12 对话排版收紧

正文与输入框统一 14 px，正文行高 1.55；队列正文 13 px，辅助标签与工具状态 12 px，继续使用产品字体，代码恢复等宽字体。表情回应并入复制/引用/重新生成所在的单行操作区；鼠标悬停和键盘聚焦显现，保留行高避免布局跳动，触屏默认可见且点击区域至少 44 px。队列使用固定三列操作位置，当前状态与耗时分列。无助手回答的取消/失败状态归入对应请求旁，未丢弃错误原因。

展开聊天时提升其地图浮层层级，避免相机/图层控件遮住聊天；不改变地图操作或机器人控制。构建、ESLint、原 37 项 UI 回归和新增状态归属测试通过。浏览器以测试数据验证桌面显隐无跳动、键盘聚焦、队列对齐，以及 390 px 触屏按钮与布局；脚本和截图位于 `artifacts/ui/campus-geography-20261111/conversation-typography-*`。未进行 G20 真机验收。


### 2026-11-12 传感器组件与显示设置

正式入口为“设置 → 传感器”和聊天“＋ → 传感器组件”。Agent 使用 `open_sensor_monitor(sensor)` 请求打开对应卡片；历史工具结果、失败和重复刷新不会重新弹出卡片。聊天工具返回的关节、电机、电压、相机和 LiDAR 观测也可直接打开对应监测。`SensorMonitorProvider` 持有独立卡片集合。多种传感器可同时打开；重复打开定位原卡片，保留字段选择和样本。收起保留采样，放大／还原保留组件状态，关闭只释放对应卡片的读取与媒体订阅。已开始的 Host IMU 记录继续运行。

组件包括 IMU、关节、电机温度与状态、各通道电压、相机、LiDAR、GNSS、热成像、声学、语义真值。目录分别读取机器人能力和测量目录；其中一项读取失败不清空另一项的结果。已配置不表示在线；没有来源不制造零值。深度、触觉、足端力等未提供来源的传感器不生成虚构卡片。

- 关节使用实际来源的名称和数量，支持 12／16 关节；数值曲线选择位置 rad、速度 rad/s、力矩 N·m、电机温度 °C 或通道电压 V。缺失、过期、读取错误和暂停形成断点。
- 数值卡片轮询间隔可设 0.5／1／2 秒，曲线窗口可设 15／60／300 秒，LiDAR 点大小可设 1／2／3 px。浏览器保存显示默认值，重新打开卡片时应用，不更改设备频率、标定或导航参数。
- 数值卡片只保留本次打开期间最近五分钟的显示采样，可下载 CSV，包含接收时间和来源时间。这不是持续录制，也不是原始高频数据；持续 IMU 记录仍由既有 Host 记录服务负责，原始 DDS／MCAP 仍由原生 recorder 负责。
- 相机复用 `/ws/camera`，暂停时复制当前图像并释放订阅；恢复后重新接收。时间标明为接收时间，不等同于拍摄时间。LiDAR 复用 PCLD 解码与三维查看器，显示限制为 2 帧／秒，不改变雷达本身的采样或导航碰撞判断。
- GNSS 不再依赖 Web 未正式接入的 `gnss_status` 事件；读取已有 `/api/v1/geography/location` 原生快照，显示 WGS84 经纬度、精度与定位类型。来源过期或请求失败时撤下位置；请求时间也计入显示年龄。当前接口不提供卫星数量，不虚构该值。
- 热成像／声学／语义组件复用既有测量检查器并按 modality 筛选；保留有效掩码、来源绑定、校准单位、原始 NPZ 和来源确认的控制。语义类别／实例标明为真值，不冒充机器人实际识别结果。设备控制不是浏览器显示设置。

界面验收脚本、合成输入说明、CSV 与截图见 `artifacts/web/sensor-cards-20261112/`。这些证据属于浏览器组件与本地接口验证，不代表 NX／真机传感器上线。


### 2026-11-12 独立传感器卡片 UI / UX

组件选择器是锚定触发按钮的搜索菜单：聊天入口向上展开，工作区顶部入口向下展开；已打开条目显示勾选，重复选择只定位卡片。Escape 返回触发按钮，点击外部区域关闭菜单，不夺走新焦点。菜单与监测卡片各自承担选择和观测，不在卡片内切换传感器类型。

每张卡片独立标题、来源／状态、主要数据和收起／放大／关闭按钮。数值组件复用采样逻辑，相机与 LiDAR 各自拥有媒体生命周期；IMU 保留 Host 记录服务。桌面使用双列卡片工作区，聊天留在另一侧；小屏单列滚动并为聊天保留底部空间。三维画布定位与裁剪在 LiDAR 卡片内部。

主文字 12–13 px，关键数值 30 px，次要说明 10–11 px；颜色用于传感器标识、曲线和真实状态。详细来源、像素读数、设备设置收进可展开区域，原始数组、单位和无效掩码语义保留。深浅主题共用现有变量。入场／悬停过渡 120–180 ms，减少动态效果设置关闭这些过渡。

视觉与交互证据：`artifacts/web/sensor-ui-20261112/`。预览路由为开发测试入口，不进入正式打包；正式 App 使用同一组卡片组件。测试服务输出合成观测，相机重复发送历史 MuJoCo 图像，不能作为机器人当前状态或现场数据。


### 2026-11-12 正式动效与语义色接入

- 中性白灰表面保留，主操作/链接/选中使用蓝色；成功、等待、失败分别使用绿、琥珀、红色小范围强调。参考 Radix Colors 的功能色分层，不新增颜色库依赖。
- Stageshow Collapse 接到工具执行详情，保留退出期间的内容；关闭立即 inert/aria-hidden，Escape 回到触发按钮。参考 Base UI 的隐藏面板与焦点行为，没有更换组件框架。减少动态效果下关闭过渡。
- 地图/观测/搜索结果使用 CSS grid 高度与淡入过渡；菜单出现、按钮按压和审批提交中的反馈统一为短时过渡。批准与执行结果仍由各自真实回执决定；动画不延迟错误/安全状态。
- 来源：https://github.com/radix-ui/colors 、https://github.com/mui/base-ui ，以及已有 Stageshow/Motion Primitives 源码研究。折叠组件沿用已保存的 Stageshow MIT 许可。
- 正式组件预览：`http://127.0.0.1:47683/tests/conversation-style-preview.html`；复用 AgentConversation 与真实控制 hook，fetch 全部由本地 fixture 接管，不转发 Host/OAuth/模型请求。
- 生产构建、43 项 UI 回归、相关 ESLint 通过；`designs/motion-options/check-integrated.mjs` 验证展开收起、快速反转、inert、Escape 焦点、审批提交/回执、减少动态效果与 390 px 布局。浏览器未观察到 API 网络请求。
- 截图和录屏：`designs/motion-options/integrated-colors.png`、`integrated-details.png`、`integrated-compact.png`、`integrated.webm`。本轮没有机器人端部署或真机验收。


### 2026-11-12 对话内已保存地图三维查看

- 新增只读 `view_saved_map(map_id)`：复用 mapd get_record 获取真实内容版本，仅返回受控 map_view 描述，不把点云或私有路径交给模型。工具可由主 Agent 调用；已存在的 map_id 来自上下文/地图库，不猜测名称。
- `SavedMapCard` 自动展开工具返回的指定地图；地图库详情也提供手动入口。复用 `fetchMapThumbnail` 的 GET `/api/v1/maps/{id}/preview?content_epoch=N`，最多 24000 点，继续检查地图名/版本，补齐点数和坐标系字段校验。卡片区分已保存原图与 CloudPreview 的处理后未保存预览。
- 复用 MapGalleryScene，支持旋转、缩放、平移、复位和截图下载。收起释放三维场景、重开复用已读数据；离开可视区域卸载 WebGL。查看不激活地图、不执行导航、不编辑源数据。采样/坐标系未知按返回值显示。
- 正式预览页新增“查看三维地图”，几何是明确的合成房间 fixture。后端工具/预览契约、前端回归、构建与 ESLint 通过。`designs/motion-options/check-map-card.mjs` 验证真实 WebGL 旋转与缩放画面变化、复位/截图、收起卸载、缓存、地图库入口、版本不匹配拒绝及重试、390px 布局，地图请求全部 GET。
- 证据：`saved-map-3d.png`、`saved-map-compact.png`、`saved-map-3d.webm`（同设计目录）。没有真实模型选择工具、机器狗或当前 Host 重载验收；运行中的旧 Host 要重载才注册新工具。
- Stageshow 已移植 Roll/Collapse；Stage 是演示编排容器，未作为机器人地图或执行控制运行时使用。


### 2026-11-12 可展开实时相机卡片

- 新增只读 `view_live_camera` 工具，返回固定 robot 来源的 camera_view 描述；不读取或保存帧、不改变相机配置、不创建运动任务，也不声称设备在线。ComposerAdd 分开提供实时观看与相机快照入口；快照观测下也可展开实时卡片。
- LiveCameraCard 默认收起，点击并进入可视区域后挂载现有 RecordingLiveCamera。复用 `/api/v1/camera/status`、WHEP `/api/v1/webrtc/whep` 与 JPEG `/ws/camera`，不新增采集/编码服务。展开状态来自用户点击，实时/断流来自实际解码帧的新鲜度。
- 收起、离开可视区域、document 进入后台或卸载均移除视频消费者，释放本卡片的 WS/WHEP 与计时器；不关闭机器人相机采集服务。展开保持平滑，重回前台且仍展开时重新连接。RecordingLiveCamera 的 embedded 外观去掉重复标题，独立录制页默认外观保留。
- 本地预览的 camera 来源是标注“本地演示画面”的 canvas JPEG；只在此测试入口替换 WebSocket，不向真实相机转发。截图 `designs/motion-options/camera-live.png`，录屏 `live-camera-card.webm`。
- 后端工具测试 10 项、前端对话/相机测试 55 项通过；另行 WHEP 生命周期与视频新鲜度 11 项通过。最后 embedded 调整后录制组件 4 项回归、构建和 ESLint 通过。浏览器 `check-camera-card.mjs` 验证初始零连接、展开帧、断流隐藏旧帧、收起/后台释放、恢复与390px布局。允许本地 Vite HMR 连接；没有真实相机 WebSocket。
- 未进行真实 WHEP 对端/摄像头/G20 验收；运行中的旧 Host 需重载注册新工具。录像与拍照仍是独立操作，不因观看自动开始记录。


### 2026-11-13 传感器图表与直接缩放

十类传感器标识改为项目内的 SVG 图形，卡片、设置目录和组件选择器使用同一份 SensorMark。取消数值卡片的单字段系统下拉框，改为搜索、分组复选和全选／清空。电机与电压的通道块也可直接选择曲线；默认显示首组三个通道，允许用户清空选择，不强制恢复默认。

IMU 的角速度和线加速度按单位分组，默认逐轴堆叠，可以切换为同单位叠加；每组支持折展。所有数值卡片共享 TraceChart，曲线保留桶内极值与缺失断点，不对缺失观测补零。接收时间轴、悬停／固定的真实采样读数、5–300 秒窗口、加减／滑条／Ctrl 或 Command + 滚轮缩放，以及拖动／左右键回看由同一组件处理。Home 返回最新窗口。原始来源时钟仍留在记录与导出中；浏览器轮询与 Host IMU 接收时钟不构成设备同步保证。

卡片右边、下边、右下角支持指针拖动。拖动期间直接更新尺寸变量，释放后提交和保存尺寸，减少图表状态重建；键盘方向键也可调整，Home 或双击边缘恢复默认。沿用已有布局存储，设置中的重置布局一并恢复卡片尺寸。放大模式通过还原按钮回到可拖动状态。

尺寸变化按容器而非整页宽度处理：完整尺寸显示曲线、通道明细；较矮时保留主要曲线与状态并收起明细；紧凑模式显示选中通道读数和状态，曲线在拉高后恢复。IMU 记录操作保留可见；设备离线与错误码计数保留在电机摘要中。压缩、折展和缩放不清空采样，也不停止 Host 记录。放大与还原保留自定义尺寸和通道选择。

本轮验收在独立的 15182／15183 测试服务上进行，避免故障注入和记录测试干扰正在使用的 15180／15181 演示。截图、录屏、脚本与结果位于 `artifacts/web/sensor-interaction-20261112/`，其中数据仍为合成观测；这些是 UI 和本地接口证据。


### Expandable observation cards (2026-11-13)

- `view_robot_card` returns a read-only view marker for robot, joints, IMU, or navigation. It does not touch the robot or start a recording. The composer offers these entries.
- Opening mounts existing NumericCard/ImuPanel or the state reader; closing unmounts polling. IMU recording remains Host-owned and requires explicit stop; collapsing never ends a record.
- Robot/navigation display uses `/api/v1/state` and the existing navigation freshness/presentation helpers, with a seven-second local receipt expiry. Read failures clear displayed state. This card is observational; existing navigation controls retain command ownership.
- Embedded camera is edge-to-edge 16:9, proportional cover (including rotated WebRTC). It can crop edges; standalone video remains contain. Existing visibility/unmount cleanup is retained.
- Preview at `/tests/conversation-style-preview.html` uses local fixtures only, including clearly marked camera frames and synthetic joint faults; it never forwards robot requests.

- Verification: 56 focused frontend tests and 11 workbench tests passed. Camera browser check proves full-width cover and release/reconnect; four-card browser check proves mount/unmount polling, fault display, and no recording/motion writes. Independent preview: http://127.0.0.1:47684/tests/observation-card-preview.html.
- Full build passed before concurrent Bot edits; final whole-project typecheck is blocked by missing `services/botShapes.ts` in that separate work. No robot deployment or real camera peer tested here.
