# OpenGrokBot → LingTu 迁移矩阵

更新时间：2026-11-15

参考：[OpenGrokBot README](https://github.com/nickylin/OpenGrokBot/blob/main/README.md)

## 结论

OpenGrokBot 最值得迁移的是产品组织方式：每个 Bot 有自己的身份、会话、记忆、例程和工作区权限；跨 Bot 协作通过显式交接和共享工作区完成；需要用户决定的动作进入审批流。

LingTu 已经有会话、审批、工具时间线、分享、Decision Inbox、Grok Avatar、MCP 基础连接，以及 Bot/Robot 两域记忆。下一步应把这些能力挂到稳定的 Bot 身份上。

## 本轮已落地

- 三个预置 Bot：现场巡检、地图导航、设备维护；当前 Bot 会持久化到浏览器。
- 会话创建、列表、搜索、fork 和分享快照带有 ot_id；旧数据库记录迁移到 lingtu-default。
- 当前 Bot 的 Thread 列表按 ot_id 过滤，聊天头部显示名称、Avatar、状态、角色和能力。
- Bot memory 的工具调用强制使用当前会话 Bot；Robot memory 的工具调用强制使用当前机器人身份（LINGTU_ROBOT_ID / DOSO_DEVICE_ID）。

## 能力对照

| 能力 | LingTu 当前状态 | 判断 | 验收 |
| --- | --- | --- | --- |
| Named Bot roster | `BotProfile` / `BotStudio` 主要是单 Bot 入口 | **P0** | 至少两个 Bot 可创建、切换、持久化 |
| Per-Bot thread | 会话、搜索、分享和 fork 已有，但没有稳定 Bot 归属 | **P0** | 会话元数据有 `bot_id`；旧会话归 `lingtu-default` |
| Per-Bot memory | 已有 `bot` / `robot` 两域和 `owner_id`，默认 owner 仍是 `default` | **P0** | Bot memory 按 Bot 隔离；Robot memory 按机器人身份隔离 |
| Shared workspace | 已有地图、文件、录制和 agent workspace，产品层没有统一作用域 | **P0** | 地图、巡检证据、连接器输出按 `workspace_id` 授权 |
| Bot-owned routines | 已有 `InspectionCalendar` / 部分定时接口，尚未绑定 Bot | **P0** | routine 有 `bot_id`、审批策略、启停状态、最近 20 次运行 |
| Routine triggers | 手动/部分定时已有，webhook 和运行历史不足 | **P1** | 手动、schedule、webhook 可审计；失败/审批进入状态卡 |
| Bot-to-Bot handoff | 有 Agent handoff 基础，但没有 Bot 级交接语义 | **P1** | 只传目标 Bot、摘要、选定文件/地图/证据；不复制私有记忆 |
| Group thread / coordinator | 单 Bot 对话为主 | **P1** | 2–6 个 Bot；协调 Bot 串行委派；消息显示 Bot 名和 Avatar |
| MCP connectors | 已有 MCP 连接能力 | **P1** | 健康、范围、Bot 级开关、secret 只写不回显 |
| Interactive forms | Decision Inbox / 审批卡片已有 | **已具备基础** | 继续复用现有审批协议 |
| Skills library | 已有技能和工具体系，缺 Bot 级启用范围 | **P1** | skill 可标记 shared 或 Bot-only |
| Presence / streaming | Bot 状态、Grok Avatar、工具时间线已有 | **已具备基础** | presence 绑定当前 Bot，而非全局单例 |
| Voice | 尚未形成完整产品链路 | **P2** | 先做语音输入，再考虑持续语音会话 |
| Browser control | 尚未形成受控 profile 和审批流程 | **P2** | profile、登录态、Cookie 复制独立于 connector 权限 |
| Computer pane | 已有 Computer Progress 状态卡；完整控制依赖 Host/Windows | **P2** | 先做权限状态和进度，闭环后再开放屏幕操作 |
| Background notifications | 有 SSE/通知基础，缺 routine/approval 通知策略 | **P1** | 回复完成、审批等待、routine 启动/失败有去重通知 |

## 迁移顺序

### P0：让 Bot 成为真实产品对象

1. `BotProfile` 增加稳定 `bot_id`、角色、能力、memory policy、workspace scope。
2. 会话元数据增加 `bot_id`，旧会话归属 `lingtu-default`，保持分享、fork、搜索兼容。
3. 把 `robot_memories.owner_id` 连接到当前 Bot 和当前机器人身份，保留 domain/owner 校验。
4. 给 `InspectionCalendar`/routine 增加 `bot_id` 和最近 20 次运行记录，结果继续走现有工具/审批/状态卡。
5. 为地图、文件、录制、连接器输出建立统一 `workspace_id` 和访问范围。

### P1：协作层

1. Bot-to-Bot 显式 handoff：只传用户选择的摘要、文件和证据。
2. Group thread/coordinator：先限制 2–6 个 Bot、串行委派和明确回退。
3. Connector 管理：健康、范围、secret masking、Bot 级开关。
4. Routine webhook、通知和运行历史。
5. Memory 面板：Bot Memory / Robot Memory 双 Tab，显示 owner、来源、置信度、TTL、最近使用时间。

### P2：宿主能力扩展

浏览器 profile、登录态和 Cookie 审批；完整 Computer Use；voice memo/input/chat；退出应用后运行与睡眠 worker；shared memories；auto-review model。

## 不直接照搬

- Markdown `MEMORY.md`：可作为导入格式，不能替代 LingTu 的结构化运行时记忆。
- 通用 Git workspace：与地图、机器人、巡检证据不是同一类真相；保留 `agent_workspace`，另建产品级 workspace 作用域。
- 跨 Bot 自动同步私有记忆：只通过显式 handoff 或 workspace 共享。
- 退出后/睡眠期间运行承诺：没有持久 worker 和唤醒策略前不对外宣称支持。
- 完整桌面 Computer Use：状态卡先行，真实屏幕操作等 Host/Windows 执行器闭环。

## 最小验收闭环

```text
选择 Bot
  → 打开该 Bot 的 Thread
  → 组装该 Bot 记忆 + 被授权的 Robot/Workspace 上下文
  → 调用工具并进入现有审批/Decision Inbox
  → 结果写入 Thread、routine run 或显式 workspace handoff
```

这条链路跑通后，再增加群组、浏览器、语音和完整 Computer Use。

