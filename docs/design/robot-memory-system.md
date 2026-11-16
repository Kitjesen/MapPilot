# LingTu 机器人记忆系统

LingTu 的记忆分为两个域和五个兼容层。记忆永远是上下文提示，不是当前传感器真值，也不授予运动权限：

| 记忆域 | 所有者 | 默认层 |
| --- | --- | --- |
| `bot` memory | 一个 Bot 或 Bot roster | `operator`、`procedural`、`working` |
| `robot` memory | 一台机器人或当前机器人实例 | `spatial`、`episodic` |

每条记录都有 `owner_id`。旧数据按层迁移：`operator`、`procedural`、`working` 进入 Bot 域；`spatial`、`episodic` 进入机器人域。`episodic` 和 `procedural` 在后续可以通过显式 domain 写入另一域，不依赖层名猜测所有权。

| 层 | 内容 | 生命周期 |
| --- | --- | --- |
| `operator` | 操作员称呼、偏好和交互习惯 | 持久，需明确请求 |
| `spatial` | 地图、区域、位置语义与来源 | 持久，必须保留来源 |
| `episodic` | 已发生的任务、巡检和结果摘要 | 持久，不能替代原始证据 |
| `procedural` | 巡检流程、操作顺序和复用经验 | 持久，修改前需读取 |
| `working` | 当前任务的短期上下文 | 可设置 TTL，默认不应长期保存 |

## 边界

`read_robot_memory` / `read_bot_memory` 是只读查询；对应的 remember、update 和 forget 工具走现有会话权限与审批边界。写入必须由操作员明确要求，不能保存凭据、运动授权或未经验证的观测。更新和遗忘要求提交读取到的旧内容，并核对 domain / owner，避免跨域或并发修改。

记忆记录保存在 ConversationStore 的 `robot_memories` 表中。遗忘只归档记忆记录，不改写会话历史、分享快照或巡检证据。

## 上下文边界

Bot memory 可以进入该 Bot 的长期上下文；robot memory 只能作为机器人、地图、巡检和传感器的历史线索。当前实时状态、地图版本和运动权限必须来自当前 Host / native 工具回执。跨 Bot 共享通过 Workspace 或显式 handoff，不把私有 Bot memory 默认注入另一个 Bot。

## 后续 UI

聊天 `+` 菜单的“记忆”入口应读取五层记忆并显示来源、置信度、过期时间和“记住/修改/忘记”审批状态；机器人状态卡和地图卡仍应以当前工具结果为准。
