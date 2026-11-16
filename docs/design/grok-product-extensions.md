# Grok 产品方向接入计划（2026-11-14）

本轮把 Grok Bot 1.0.1 里出现的产品方向映射到 LingTu 当前真实模块，并完成第一批可运行接入。

## 已接入

### Computer progress 与屏幕操作许可

`ComputerProgressCard` 支持 `running`、`awaiting_permission`、`paused`、`completed` 和 `failed`。结果字段使用 `computer_view`，包含 `action`、`phase`、`elapsed_ms`、`permission` 和 `error` 时会显示对应卡片。卡片按钮只通过回调请求暂停、继续或权限，当前不会执行电脑控制。

它挂在现有 `ToolActivity` 结果区，继续复用工具消息、审批和放大执行详情。真正的 computer use 需要新增屏幕会话拥有者、允许应用列表、录屏/辅助功能权限、动作执行器和审计记录。Grok renderer 中出现的字符串不足以提供这些契约。

### 全屏查看器

`FullscreenViewer` 提供图片、视频、表格和地图四种类型，支持 Escape、背景点击、关闭按钮、焦点、滚动锁定和移动端布局。Markdown 图片已经接入统一查看器。地图和点云可以通过 `children` 复用现有场景。

### 状态、活动和动作

Bot 目录保留 52 个形象。状态目录包含 Grok 的工作状态、表情动作、活动形变、产品状态和展示状态。真实会话会根据工具类型显示搜索、阅读、编程、控制、上传、发送和书写活动。BotStudio 可以预览完整动作目录和九种表情。

## 可复用的现有基础

### Routines 与定时任务

LingTu 已有巡检计划 SQLite、occurrence、next_due、重启恢复和调度器。Grok 式 routine 应该生成计划草稿，再进入现有巡检计划保存和审批流程。不能另造一套后台 scheduler。

### Memories 与 shared memories

偏好记忆已经有读取、保存、更新和删除工具，SQLite 是真源。当前 scope 是 `host_operator`。Shared memories 需要 owner、team scope、ACL、审计字段和撤销流程，然后再挂到分享或邀请入口。

### Plugins、connectors 和跨应用搜索

MCP client 已经提供连接、资源、提示和工具发现，planner 会把 MCP tools 合并进 Agent。下一步可以做 Connector 状态卡和按应用/技能筛选的搜索入口，数据必须来自 MCP client 的真实状态。

### 语音

当前没有录音、转写、语音会话资源锁或浏览器音频权限链。Grok 的 voice 字符串只能作为产品方向证据。真正接入需要录音服务、转写事件、单会话锁和音频权限状态。

### Browser 登录、Cookie 和审批

当前没有浏览器 session owner、Cookie secret boundary、浏览器 approval route 或审计模型。浏览器登录和 Cookie 复制需要独立的凭据边界和审批事件，不能从 renderer 文案直接接入。

### Google／Outlook Calendar

当前只有内部 Asia/Shanghai 巡检日程模型，没有外部 OAuth 和事件同步。外部日历可以复用时间窗和 occurrence 的表示，但需要 connector account、token refresh 和事件冲突策略。

### Notifications 与声音

现有 SSE/realtime 可以支持站内任务和审批通知。通知偏好、浏览器通知权限和声音播放策略还需要持久化设置，第一步适合先做站内通知中心。

### 邀请协作和共享上下文

当前分享是结束会话的只读快照。邀请协作需要成员、角色、共享上下文、记忆权限和撤销审计。分享 token 不能直接升级为实时协作权限。

## 取证与验证

本轮使用 REA 检查当前 Grok Bot ASAR 和 renderer common asset。当前 ASAR SHA-256 是 `ae8f0a14fe65f421881f96fdcb66aaaaed8b28806f79d5239dbefec9ef2ea972`。REA 当前 common Evidence 是 `ev_2c6a01c5b4e658041e4f976ad9d5bd31b19299f772eab2044261f9706259d063`。分析是静态的，覆盖为 partial，没有执行 Grok 应用。

新增 ComputerProgressCard 测试 1 项通过，FullscreenViewer 测试 2 项通过。Bot 目录、活动目录、原生适配、眨眼和形象浏览器检查继续通过。生产构建和触及文件 ESLint 通过。工作区会话 UI 仍有两项模型工具栏文案/排序断言失败，位于 `agentConversationUi.test.ts` 406 和 434 行，与本轮新增组件无关。
