# T3 Code 与 Hermes 的能力取舍

核对日期：2026-11-12。T3 Code 官方仓库 `pingdotgg/t3code`，本次远端主线快照
`4fd40e1a16e3800418b8f6a32932b68179098c8d`；GitHub 最新正式发布返回 `v0.1.9`
（2026-11-11）。主线快照不等于该正式版包含全部功能。许可证为 MIT。
已有本地参考仓库为 `D:/inovxio/references/t3code`，提交
`fc6c7c8b27fc94e6486ca1b589ae138ef259f8f5`，本轮未修改该仓库。

本轮是源码／官方文档研究与 LingTu 代码对照，没有迁入新运行模块、安装依赖或验证 T3 产品运行效果。
远端文档快照保存在 `artifacts/research/t3code-20261112/`。

## 已有基础，避免重复迁移

- LingTu 已局部迁入 EffortGlyph、流式 Markdown 分块，保留提交来源和 MIT 许可。
- 排队消息的编辑／取消／提前处理、历史分页和滚动行为已有实现。
- 当前代码已有会话 fork、分支来源与切换入口；本轮只检查代码，不把它说成已做真实机器人验收。
- 已有计划草稿卡、工具计时、附件、map_context、地图三维卡片，以及模型用量元数据。

前期记录见 [agent-conversations.md](agent-conversations.md) 和
[conversation-workbench.md](conversation-workbench.md)。

## 推荐顺序

| 顺序 | T3 Code 参考能力 | LingTu 的具体落点 | 当前差距／迁移边界 |
| --- | --- | --- | --- |
| 1 | Typed context references | 在句子里引用地图、选区、巡检点、相机截图、IMU 时间段；点击可回到对象 | 已有 map_context／附件，但缺统一的对象引用机制；复用精确 ID、版本、时间信息，不把标签当身份 |
| 2 | Connection runtime | 显示机器人连接、会话同步、传感器数据新鲜度；断线回来保留草稿并补拉结果 | 已有 SSE／快照恢复；进一步统一重连归属，不能把重连变成重发运动 |
| 3 | Model manifest / picker | 模型能力、图片支持、思考选项、配置／可调用状态清楚分开 | 当前能力多在 Python 配置代码中；借鉴声明与适配分离，不让远程清单直接改变调用协议 |
| 4 | Usage and resource diagnostics | 展示首字、工具耗时、模型响应轮数、token、设备负载 | 主回合已有 model_metrics，UI 汇总及摘要／子助手总用量尚未闭环；采集应按需，不另起高频扫描 |
| 5 | Voice-to-draft controller | 按住说话→文字进入草稿→编辑／确认发送 | 不能直接宣称机器人离线语音可用；转写模型、设备负载、网络与误识别需验证 |
| 6 | Change review / diff interaction | 地图清洗、补面、禁区修改的前后比较、影响范围和确认 | 是交互借鉴，T3 的代码 diff 不是点云算法；继续使用 LingTu 地图内容版本和编辑服务 |
| 7 | Multi-environment client connections | 同一客户端管理多个机器人端点，切换时保留各自状态 | T3 的环境映射为机器人连接端点，不改变 LingTu env=real/sim 定义；先解决局域网交付 |

建议第一批为对象引用、连接／数据状态和用量诊断。地图中的“这里”、聊天中的“这张图”、
传感器中的“刚才那段”应变成可核对的对象，减少无依据猜测和反复补充上下文。
这只是预期收益，不声称已经测量模型提速。

## T3 Code 与 Hermes 分工

- T3 Code：参考对话工作台、输入草稿、对象引用、事件呈现、连接恢复、模型能力呈现和诊断。
- Hermes：参考通用 Agent 的技能、记忆、网页工具、上下文管理和委派能力。
- LingTu：继续拥有机器人任务、地图、感知、导航、停止结果和 ProductControl。

两者都有后端能力，以上是 LingTu 的取舍，不是对上游项目的完整分类。
不同时引入两套会话所有者、审批状态或工具执行循环。
独立组件／纯函数可以固定提交局部迁入；强耦合能力应连同接口、失败恢复和测试一起适配。

## 明确不照搬的地方

- `client-runtime`、`provider-core` 等是 T3 内部 workspace 包，依赖 Effect／RPC／平台生命周期；
  不能当成安装即用的机器人 SDK。
- T3 Connect 使用 Clerk、relay 和 managed tunnel 等基础设施；读取其源码不等于拥有可用的远程服务。
- Git worktree、终端和代码文件回退不等于机器人任务、地图或物理运动的回退。
- T3 当前 portable handoff 将部分文本压成最多 240 字符的片段，不恢复原生提供商会话、审批或工具状态；
  不能替换 LingTu 的执行证据与原文恢复。
- 模型目录更新不等于账号有额度、模型已经推理成功或工具协议兼容。

## 固定来源

以下链接均固定本次主线提交，避免把未来变化写成当前事实：

- [上下文引用](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/composer-context-references.md)
- [连接运行时](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/connection-runtime.md)
- [模型清单](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/model-manifest.md)
- [资源诊断](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/resource-telemetry.md)
- [语音输入](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/voice-input.md)
- [上下文交接](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/context-handoffs.md)
- [T3 Connect](https://github.com/pingdotgg/t3code/blob/4fd40e1a16e3800418b8f6a32932b68179098c8d/docs/internals/t3-connect.md)

变更审阅交互本轮参考的是已存在的本地固定提交：
[DiffPanel](https://github.com/pingdotgg/t3code/blob/fc6c7c8b27fc94e6486ca1b589ae138ef259f8f5/apps/web/src/components/DiffPanel.tsx)。
后续实施须读对应代码和测试，不能仅凭文件名宣称可直接复用三维编辑。
