# Grok Bot 第二轮逆向与 LingTu 迁移记录

本轮继续检查本机 `D:\Program Files\Grok Bot\resources\app.asar`，安装包 `package.json` 版本为 1.0.1。当前安装包的 renderer common asset 是 `dist/renderer/assets/index.eager-common-DcDxc78C.js`，并对照已接入的头像适配层。

REA 对当前 ASAR 做了完整清单检查，并对这个 common asset 做了限定静态 JavaScript 分析。当前 ASAR SHA-256 是 `ae8f0a14fe65f421881f96fdcb66aaaaed8b28806f79d5239dbefec9ef2ea972`。本轮 common asset 的 REA Evidence 是 `ev_2c6a01c5b4e658041e4f976ad9d5bd31b19299f772eab2044261f9706259d063`，父 Evidence 是 `ev_d89c2658ed3f31f2bd0cdbe514ac899a766733832f9e4b49a32cea096385dced`。分析静态且覆盖 partial，未执行应用。

## 头像状态机

Grok 的状态表按用途分为五组。

- 工作状态包含 `idle`、`listening`、`thinking`、`streaming`、`searching`、`reading`、`working`、`coding`、`running`、`computing`、`controlling`、`waiting`、`blocked`、`finished`、`failed`、`sleeping`、`waking`。
- 表情动作包含 `happy`、`excited`、`laughing`、`proud`、`curious`、`playful`、`shy`、`surprised`、`suspicious`、`angry`、`confused`、`sad`、`drowsy`、`bored`、`scared`、`celebrate`、`approving`。
- 活动形变包含 `orbit`、`radar`、`progress`、`spawning`、`loading`、`dictating`、`sending`、`receiving`、`uploading`、`writing`、`alerting`、`bouncing`、`powering-down`。
- 产品状态包含 `humming`、`notifying`、`dragging`。
- 展示状态包含 `logo`、`showcase`、`turntable`。

当前 LingTu 已将 52 个状态写入 `botPresentation.ts`，BotStudio 以三组动作选项提供预览。形象库、状态选择和表情预览都不改变会话任务状态。

## 真实会话映射

LingTu 现在使用当前运行轮次的真实工具消息推导活动。

- `web_search`、技能搜索、历史搜索、MCP 资源搜索和记忆查询映射到 `searching`。
- 网页读取、MCP 读取、传感器、LiDAR、导航诊断和相机读取映射到 `reading`。
- Python 与点云编辑预览映射到 `coding`。
- 导航、目标识别、地图写入和巡检点写入映射到 `controlling`。
- 附件上传、发送消息和正文流式输出分别映射到 `uploading`、`sending`、`writing`。
- 等待审批映射到 `waiting`。

旧式 `tool_start` 消息会先解析其 JSON 中的真实工具名，再选择活动。当前轮次边界仍由运行中的 turn 约束，历史工具不会驱动头像活动。

## Grok 的统一活动层

renderer 中还有活动图标映射。思考使用 dots，检索使用 radar，进度使用 progress，生成使用 gather，倾听使用 wave，发送和接收使用 send/receive，上传使用 dock，书写使用 pencil，提醒使用 bang，关机使用 standby。LingTu 当前已经把这些状态接到统一活动目录，平面形象会将检索、阅读、编程、控制、进度和上传映射到对应的头部、目光或环形效果。

`progress` 和 `spawning` 有停留窗口和重新触发节奏。当前没有把定时器直接伪装成真实进度。接下来只有在工具事件提供阶段或进度信息时，才适合把这套窗口用于长时间工具。

## 其他产品功能线索

common asset 与异步 chunk 名称还出现了 routines、memories、plugins/connectors、voice memo、voice chat、computer progress、browser approval、calendar cards、notification preferences、全屏附件查看器和共享上下文。这些证据表明 Grok 产品存在相应界面方向，但本轮没有取得完整业务闭包或运行时流程证据。

LingTu 当前最适合继续迁移的方向是工具进度卡。现有 ToolActivity 已有工具开始、工具结果、运行时长和审批信息，可以增加当前动作、阶段和暂停入口。routines、长期记忆、连接器、语音和电脑控制需要对应的后端契约与权限状态，不能只凭 renderer 字符串接入。

## 验证

- 52 个形象目录测试、完整活动目录测试、Grok 原生适配测试和眨眼测试通过。
- Browser Chrome 检查逐个选择 52 个形象，验证眼睛处于实体内，参考活动会动，减少动态效果会停，52 个 BotStudio 选项可见，48/64 尺寸保存并在刷新后恢复，手机宽度无横向溢出，page error 为零。
- TypeScript、触及文件 ESLint 和生产构建通过。
- 会话 UI 总测试中有两项既有工具栏文案/排序断言失败，位于 `agentConversationUi.test.ts` 的模型与思考深度区域，与本轮 Grok 活动映射无关。其余会话 UI 断言通过。

本轮静态逆向没有执行 Grok 应用，也没有接入语音、浏览器、日历、电脑控制或网络连接器。相关功能需要下一轮以完整业务链路为目标继续取证。
