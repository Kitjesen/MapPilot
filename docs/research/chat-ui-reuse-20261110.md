# 聊天界面复用选型（2026-11-10）

最初完成官方资料与源码选型；同日按用户后续要求开始局部迁移。当前已接入下文列明的 T3 Code 图标与紧凑菜单，尚未安装完整聊天组件库；其余建议不表示已实现。

## 已落地的第一步

- `web/src/components/agentConversation/EffortGlyph.tsx` 迁入固定版本 T3 Code 的整数像素柱形与 SVG 思考档位图标；同目录 `LICENSE.t3code.txt` 保留 MIT、上游 commit、文件和本地适配说明。
- 聊天输入上方放正文，底部一行放附件、历史、模型、思考、发送。模型菜单复用本项目已有 details 与菜单关闭机制，选择继续走原会话 API。
- 未复制 T3 Chat 网站源码，未安装 T3 编程运行时或新的依赖。流式输出、附件拖拽等后续条目仍待接入。

## 建议

若要成套补齐成熟聊天交互，优先评估 **assistant-ui 的 React 组件与 ExternalStoreRuntime**，接现有 LingTu 会话状态和 API。T3 Code 用作输入区、消息和工具活动的交互参考，只迁移能独立使用的源码。Flowith 的画布适合多分支研究和任务复盘，可作为后续独立工作区的参考。

现在的操作主界面保留三维地图、紧急停止与任务状态。聊天窗口可以提供更舒适的展开视图，无需把整个机器人操作台替换成聊天网站。

## 候选与依据

| 项目 | 定位与可借鉴内容 | 复用判断 |
| --- | --- | --- |
| [Flowith Canvas](https://flowith.io/tools/canvas/) | 对话、资料与结果分支放在空间画布里 | 很可能是用户记得的“flowth”。官方 GitHub 组织公开的是画布 CLI；未找到完整网站前端的公开授权源码，不能据此承诺直接搬代码 |
| [T3 Code](https://github.com/pingdotgg/t3code) | 编程 Agent 的对话、输入、工具活动和工作区 | MIT；固定版本后可迁移代码并保留许可。主组件高度依赖自己的编程状态和内部包，不适合整文件复制 |
| [assistant-ui](https://github.com/assistant-ui/assistant-ui) | 可组合的 React 消息、输入、附件、工具结果与线程组件 | MIT；专门用于嵌入既有应用，是当前最值得试接的底座，不要求使用其托管云 |
| [LibreChat](https://github.com/LibreChat-AI/LibreChat) | 完整多模型聊天产品，含附件、搜索、Agent 等 | MIT；适合对照完整产品能力，整站移植会引入另一套应用后端与状态管理 |
| [LobeHub](https://github.com/lobehub/lobehub) | 多模型、知识资料与 Agent 工作空间 | 当前 Community License 对衍生开发及分发有商业授权要求；适合先参考交互，不能按纯 Apache-2.0 直接处理 |
| [Open WebUI](https://github.com/open-webui/open-webui) | 自托管模型聊天平台 | 当前许可证有品牌保留条款及例外，嵌入 LingTu 前需按具体版本判断；并非单独的 React 聊天控件 |

另一个近似名字是 [Flowise](https://flowiseai.com/)，定位是可视化构建 Agent。不是 Flowith 的对话画布，官网当前还显示 sunset 公告，本轮不将它选为新增基础。

许可证原文：[assistant-ui MIT](https://github.com/assistant-ui/assistant-ui/blob/main/LICENSE)、[LobeHub Community License](https://github.com/lobehub/lobehub/blob/main/LICENSE)、[Open WebUI LICENSE](https://github.com/open-webui/open-webui/blob/main/LICENSE)。Flowith 公开仓库范围见 [官方组织](https://github.com/flowith-ai)。

## T3 Code 固定源码核对

固定提交：[`e89abadaee70e0d73fb19a5ce40a686b1a92310e`](https://github.com/pingdotgg/t3code/commit/e89abadaee70e0d73fb19a5ce40a686b1a92310e)，提交时间 2026-11-10 04:35:26 UTC。[该版本 MIT 许可](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/LICENSE) 需随实质源码复制保留。

以下均相对 `apps/web/src/components/chat/`：

| 文件 | 该版本规模 | 适用方式 |
| --- | --- | --- |
| [markdownStreamBlocks.ts](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/src/components/chat/markdownStreamBlocks.ts) | 47 行，无 import | 可独立迁移的 Markdown 分块函数，等真正接通流式文本时使用 |
| [ComposerSurface.tsx](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/src/components/chat/ComposerSurface.tsx) | 85 行 | 输入区视觉结构；Tailwind 与主题变量转成现有 CSS Modules |
| [EffortGlyph.tsx](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/src/components/chat/EffortGlyph.tsx) | 107 行 | 思考强度 SVG；类型与档位映射到现有后端模型能力 |
| [MessageCopyButton.tsx](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/src/components/chat/MessageCopyButton.tsx) | 65 行 | 复制与反馈交互；需替换 T3 的 Toast、Button、Tooltip 等依赖 |
| [ComposerImageThumbnail.tsx](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/src/components/chat/ComposerImageThumbnail.tsx) | 82 行 | 本地图片预览和 URL 释放；其解码 helper 仍需按本项目支持格式适配 |

主 `ChatComposer.tsx` 为 3,685 行，附件条为 717 行，模型/思考控件为 677 行；它们依赖 T3 的 Environment、Thread、Provider、权限和工作区状态。`ChatMarkdown.tsx` 也接入文件、编辑器、PR 和远端环境。不能用几个文件的名字相似来判断可以直接粘贴编译。

T3 的 [Web package](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/apps/web/package.json) 是私有 workspace，内部依赖包含 `@t3tools/contracts`、`@t3tools/client-runtime` 等；其共享运行时不是独立聊天控件入口。完整移植会引入 Effect、编辑器、UI/样式库和 RPC 状态体系，见 [固定版本架构](https://github.com/pingdotgg/t3code/blob/e89abadaee70e0d73fb19a5ce40a686b1a92310e/docs/internals/overview.md)。

[T3 Chat](https://t3.chat/status) 与 [T3 Code](https://t3.codes/) 是两个产品，前者面向多模型聊天，后者面向编程 Agent。本次 MIT 核对仅覆盖 T3 Code，不能据此推断 T3 Chat 全站源码也能复制。以上是源码核对，没有运行 T3 或完成组件迁移编译。

## 与现有代码的连接

本地事实：`web/package.json` 使用 React 19、TypeScript、Vite 和 CSS Modules，直接运行依赖只有 React、React DOM、Lucide 和 Three.js。尚未安装 assistant-ui、Tailwind、shadcn 或 T3 内部包。

现有 `web/src/hooks/useAgentConversation.ts` 拥有草稿、消息 ID、模型/思考选择、附件引用与快照轮询；`web/src/services/agentConversation.ts` 调用 Gateway。后端 Agent 与 SQLite 拥有持久会话和工具执行记录。

assistant-ui 的 [ExternalStoreRuntime 官方接口](https://www.assistant-ui.com/docs/runtimes/custom/external-store) 允许传入既有消息和发送回调、转换现有消息格式、接自己的附件适配器。采用该入口时，不再引入第二份消息持久化或云端会话。模型菜单继续读取 LingTu 的 `/api/v1/agent/options`。

当前 `sdk_agent.py` 已产生 `text_delta`，但 `agent_planner.py` 调用时传入 `on_event=lambda event: None`；Web 每秒读快照。换 UI 组件不会自动带来逐字流式输出。落地流式显示时还需要接通这一事件出口、传输与刷新恢复。

## 实施顺序建议

1. 固定候选版本，最小页面试接现有会话。证明文本、模型/思考、附件、原请求重试仍由现有 API 处理，再替换现有卡片。
2. 接通真实增量文本，支持跟随最新消息、用户回看时暂停自动滚动。保留完整终态与刷新恢复。
3. 完成附件拖拽/粘贴、图片预览、回复复制、折叠工具活动和对话展开；显示真实工具状态。
4. 后续再考虑历史会话列表、搜索与分支。编辑/重新生成涉及旧动作重放，先定义行为，再显示按钮。

“停止生成”与“停止机器人”分属不同操作。引入的聊天控件不能自己执行运动工具、替代停止确认，或用一次模型回复判断机器人任务结束。模型推理预算控制可以复用菜单；推理文本只展示供应商明确提供、且产品决定公开的内容，不以装饰文字伪造思考过程。

复制 MIT 源码时记录仓库、固定 commit、原文件、许可和本地改动。采用新依赖前列出实际新增依赖与接入范围；当前这一步没有新增依赖，变化限于界面及地图编辑接线。
