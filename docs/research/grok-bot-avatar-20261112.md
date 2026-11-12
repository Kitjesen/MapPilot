# Grok Bot 头像动画研究与 LingTu 采用边界（2026-11-12）

LingTu 本轮采用动作编排、眨眼仲裁和状态驱动的思路，保留自己的 8 种形状与 12 种颜色，不接入完整 Grok 渲染器、官方提取源码或原版动作数据。目标是让头像反馈真实会话状态，并让短动作自然衔接；头像表现不证明机器人动作完成，也不表示系统具有生命或意识。

本文记录静态研究、实施决策及下述 LingTu 本地验收；不是官方运行时复现报告，也不构成性能比较。

## 固定来源与取证范围

| 来源 | 本轮固定版本 | 可定位的材料 |
| --- | --- | --- |
| A：[grokbot-animation-anatomy](https://github.com/ArvinYin1/grokbot-animation-anatomy) | `debd41625da89b4c4fde43b3b1f0add6dc113c02`，提交时间 2026-10-21 | 本地 `D:\inovxio\references\grokbot-animation-anatomy`；教学页、手工移植播放器、提取后的 JSON、提取和验证脚本 |
| B：[grok-bot-avatar-engine](https://github.com/RongleCat/grok-bot-avatar-engine) | `a995c0675e36ba113de41ead3a5d9be3fcd567e3` | 本地 `D:\inovxio\references\grok-bot-avatar-engine`；接口绑定、声称逐字提取的渲染代码、声明清单和测试源码 |

A 的 [README.md:67][a-source] 说明研究对象是 macOS Electron 应用解包后的 `app.asar`。其 [grok-recipes.json:2][a-meta] 记录：

- 源文件：`index.eager-platform-DUN5VkaE.js`。
- 提取时间：`2026-10-12T09:25:57.844Z`。
- 记录的源文件 SHA-256：`560912e771f4989101d04ad06dca024cd1c0513cbdcad122222d9a6bd1df1183`。
- 本轮直接读取 JSON 确认为 28 条 clip、53 个状态、9 个表情名、25 个形状名。提取脚本默认读取相邻 `grokbot-src/dist/renderer/assets/`，见 [extract-recipes.mjs:18][a-extract]。

B 的 [manifest.json:2][b-manifest] 标注 `bundleVersion="0.78.1"`，其中 `avatar-engine` 条目标注来自 `app.asar!dist/renderer/assets/index.eager-common-DbK3Oi_r.js`、`count=618`；[avatar-engine.json:2][b-declarations] 的声明数组实际有 618 项。生成代码每项前附源 bundle、偏移、长度及摘要，见 [avatar-engine.js:1][b-code]。这些是仓库内可核对的提取记录；本轮没有对应完整官方安装包、完整原 bundle、签名或独立来源校验，不能把记录本身等同于官方真实性证明。

官方 [Grok Bot 页面](https://x.ai/bot) 与 [安装指引](https://docs.x.ai/grok-bot/get-started) 可确认产品及桌面下载入口。它们不能证明上述两个仓库使用的特定 bundle 版本或其移植结果。两个仓库均不是 xAI 官方发布的头像 SDK。

本轮协调者记录的 REA 父证据：

- A：`ev_a051957558b61a4fa24bf2c10aa022dee24b132236b4da6c192337001e25a122`，覆盖 `grok-player`、`blink`、`lib-shapes`。
- B：`ev_51f04252645cd7dc2e5b224875f9c66a932279b8a4ddc55d5152d69f385e0a8c`，覆盖 `blink-triplet`、`state-driver`、`shuffle-order`、`spring`。

上述证据均限于静态分析，依赖图覆盖为 partial；不表示已重建完整依赖闭包、运行官方应用或完成视觉一致性实测。

## 两个仓库的差异与声明边界

| 主题 | 作者当前声明 | 本轮能确认的事实与限制 |
| --- | --- | --- |
| A 的移植方式 | 播放器逐行移植、与原版逐帧一致 | [播放器文件头][a-player] 明确列出未移植部分：非 sphere 身体回退、部分多身体混合、pencil 身体环等。它是可读的重写播放器，不能称为完整官方引擎 |
| A 的验证 | 与原引擎同条件比较 | [verify-frame.mjs:385][a-verify] 默认计算一个确定性窗口；只有原 bundle 存在时才执行交叉比较，缺失时会跳过。本轮只读脚本，未运行验证，不能把测试文件存在当作验证通过 |
| B 的移植方式 | 0.78.1 客户端渲染器，618 项声明逐字提取 | 清单、带偏移注释的代码及接口绑定确实存在。其测试导入仓库自己的提取代码，见 [engine.test.mjs:12][b-test]；本轮未与独立取得的官方 bundle 或官方运行时交叉验证 |
| B 的渲染覆盖 | 完整网格、挖空眼睛、南瓜及状态驱动；旧正面轮廓近似已删除 | [README.md:95][b-coverage] 当前明确说明网格路径；[:103][b-old] 说明旧近似实现和橙色主 Bot 星形已删除。旧截图、旧版本 README 或对旧近似实现的评价，不能直接用于评价这个提交 |
| 自动眨眼 | A 认为原 bundle 没有独立环境眨眼调度器，唯一 `Math.random` 用于网络退避 | [index.html:324][a-blink-claim] 是作者对其原始 bundle 的分析结论。当前移植代码确实可看到 clip 事件、命令式 blink 和表情切换三条路径；但本轮缺原 bundle，不能独立证明“官方全 bundle 不存在某逻辑” |
| 性能与选型 | 程序化 SVG 胜过 Lottie、视频、WebGL、CSS | 可确认 A 播放器无需动画库、直接写 SVG 属性。横向优劣仍是作者的场景判断；[compare.js:33][a-compare] 是选择向导，不是统一设备和负载下的性能基准 |

A 还存在一个需要谨慎对待“全路径一致”的具体信号：[grok-player.js:1548][a-config] 对 `rb/ob` 的 JSON 字段映射，配合 [:300][a-face] 的相位窗选择，与 [verify-frame.mjs:218][a-window] 对 `blinkHold/blinkClose` 的选择相反。本轮只确认两个本地实现读取字段的方式不一致，不据此断言官方代码有错。clip 的三点眨眼编译路径不依赖这组相位窗；其明确参数可单独核对。

## 值得借鉴的编排机制

### 眨眼属于动作短句，闭合量应统一仲裁

A 的 [grok-player.js:423][a-triplet] 将 `v="blink"` 编译为三个眼睑关键帧：开始睁眼，`0.26 × 0.45 = 0.117` 秒时完全闭合，`0.26` 秒时重新睁开，缓动为 `[0.42, 0, 0.58, 1]`。参数来自 [grok-recipes.json:8070][a-durations]，不是本轮测量真人眨眼所得，也不是 LingTu 必须照抄的固定值。

Idle_A/B/D 的眨眼事件放在各自动作时间轴中；Idle_D 的约 `2.069s` 与 `2.372s` 是一组相邻眨眼，见 [index.html:332][a-double]。自然感因此可以来自“静止—观察—单眨或双眨—回稳”的整体节奏，而不是让两个独立 `scaleY` 动画互相覆盖。

命令式眨眼会检查当前表情过渡，见 [grok-player.js:341][a-manual]；闭眼期间的表情更换也经过仲裁。LingTu 应保留一份眼睑闭合量，让交互反馈、眨眼与状态表情共用它，避免闭眼中途突然被另一段动画强制睁开。

### 变化来自可复现的短句池

[grok-player.js:57][a-shuffle] 有带种子的随机数与洗牌；[:681][a-state] 的状态机从动作池取片段。idle 池为 `Idle_B, Idle_B, Idle_D, Idle_B, Idle_A`，小尺寸使用更克制的 `Idle_B`，见 [grok-recipes.json:5120][a-idle]。首段可选随机起始相位，后续按片段完整节拍播放；不要把它描述成每帧随机改变表情。

LingTu 采用这个原则，自行设计四套短句时间轴并洗牌，避免相邻连续播放同一套。四套短句的时长、曲线与动作内容由 LingTu 原创；不直接拷贝提取的 Idle 数据。可复现的调度有利于测试和录屏，头像实例也不必全部同步眨眼。

### 有明确结束点的过渡与弹簧各司其职

A 的 [bridge:472][a-bridge] 使用有限时长的 Bézier 过渡；弹簧用于身体和装饰的连续运动，子步长在 [grok-player.js:101][a-spring] 为 `1/120s`。不能因为展示标题说“springs, not timers”就把所有状态切换都改成渐近收敛的弹簧。完成、错误和等待反馈需要明确可结束的表现，不应为了演完动画而延迟实际业务状态。

### 不可见时休眠，小预览保持静态

A 的 [宿主循环:1756][a-host] 处理 reduced-motion；[:1782][a-hidden] 在隐藏文档中跳过推进/渲染，离屏观察器在 [:1850][a-offscreen] 停止循环。隐藏文档跳过渲染不等于所有路径都完全取消 rAF，报告不把两者混称为“零开销”。

LingTu 本轮要求隐藏和 reduced-motion 时停止动作调度；形状选择器缩略图始终静态。主头像与预览交互使用各自需要的最少运动，不让几十个缩略图各自运行完整引擎。

## LingTu 本轮采用与不接入的范围

| 采用 | 本轮不接入 |
| --- | --- |
| 现有 8 形、12 色，独立的头部、视线、眼睑层 | 官方形状、颜色身份映射、品牌角色、南瓜实验门控 |
| 四套原创短句、洗牌且不连续播放同套、单一眼睑调度 | 提取的 28 个 clip、53 状态注册表或整段原始曲线 |
| 真实会话与工具事件驱动的 processing / working / writing / waiting / done / error | 根据模型正文关键词、装饰性计时器伪造“正在检索”“正在导航” |
| 隐藏和 reduced-motion 停调度；缩略图静态 | 整包播放器、网格/SDF 管线、运行第三方提取或安装脚本 |

本轮已在 `web/src/components/agentConversation/Bot.tsx`、`Bot.module.css`、`botMotion.ts`、`BotStudio.tsx` / CSS 及 `AgentConversation.tsx` 保存上述实现。没有修改外观存储协议或新增依赖。动作预览选择器仅影响面板预览，不保存为业务状态。

### 本地验收结果

- 动效调度 5 项测试通过：单眨/双眨、片段轮换、换牌边界去重、隐藏与 reduced-motion 暂停恢复、卸载清理。原有资料与外观存储 4 项测试通过。
- 会话 UI 49 项测试通过；最后补充无 running turn 时忽略旧无 turn_id 消息后，相关 6 项定向回归通过。后端真实工具记录按同一消息 ID 原位从 running 更新至终态，因此没有另加工具事件合并机制。
- 触及 TypeScript / TSX 文件 ESLint 通过；全站 `npm run build` 通过。首次构建遇到共享工作区 `sensorChart.ts` 的类型错误，后续共享代码已有修正，本任务未改该文件。
- 本机 Chrome 使用生产 Bot / BotStudio 验证：待机眨眼可见；八种状态可切换；减少动态效果时没有运行中的头像动画、鼠标跟随停止；8 个选型缩略图静态；390px 宽预览可竖向滚动且无横向溢出。会话实际入口可打开新面板，浏览器 page-error 为零。
- 预览状态由按钮模拟，不连接模型或机器人。可打开开发预览 `/tests/bot-motion-preview.html`；会话入口为右上角 Bot → 我的 Bot → 动作预览。未重启共享 Host，未做真实模型或硬件任务验收。

本轮录屏、截图和浏览器断言记录位于 `C:\Users\99563\Documents\Codex\2026-11-10\new-chat\outputs\lingtu-bot-motion\`；REA 完整证据导出在相邻 `grok-bot-rea-evidence-20261112.json`。录屏展示本地组件与模拟事件，不证明官方 Grok 的逐帧一致性或实际机器人动作。

状态必须使用同一业务事实解释：

- **processing**：请求已被接收、确实处于处理阶段；不能仅凭输入框有文字触发。
- **working**：已有真实工具或任务运行事件。它不自动等于机器人正在移动。
- **writing**：已接收助手正文的流式消息，不把私有思考或工具输出冒充正文。
- **waiting**：确有待审批或待用户回答状态。
- **done**：对话轮次或相应已声明操作结束；生成方案完成不等于方案已执行，模型说“到了”也不构成导航完成证据。
- **error**：真实请求、连接或轮次失败；具体故障仍由原错误 UI 解释，头像不替代安全状态。

后续可以细分检索、读图、导航等状态，但只使用已经存在、语义明确的工具生命周期或原生运行时事件。没有相应证据就维持普通处理状态，不先造一个动画状态再补想象中的后端能力。模型检索失败、相机图像只是缓存、导航尚未确认完成时，都应保持原有事实边界。

## 授权与验证记录

A 的 README 末尾仅标注 MIT，本轮未找到独立 LICENSE 文件；同一 README 又明确提取设计与数据归原权利人、仅作学习研究，见 [README.md:67][a-source]。B 有 [MIT LICENSE][b-license]，但它同时声明提取代码“逐字来自客户端”；仓库作者的 MIT 声明不能被推定为原权利人对官方原码、设计数据和商标的额外许可。当前决策是研究机制、原创实现，不把官方提取代码或数据直接放进 LingTu。

本轮完成：固定两个提交、读取 README/清单/源码/验证脚本、检查数据项、核对官方产品链接，整理协调者提供的 REA 静态证据。没有执行第三方安装、提取、构建或测试脚本，没有运行 Grok 官方应用，没有完成官方视觉一致性、CPU/GPU/耗电或真实浏览器验收。后续若声称这些结果，需补充对应版本、输入、运行环境和直接证据。

[a-source]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/README.md#L67
[a-meta]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-recipes.json#L2
[a-extract]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/tools/extract-recipes.mjs#L18
[a-player]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L1
[a-verify]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/tools/verify-frame.mjs#L385
[a-blink-claim]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/index.html#L324
[a-compare]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/sections/compare.js#L33
[a-config]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L1548
[a-face]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L300
[a-window]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/tools/verify-frame.mjs#L218
[a-triplet]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L423
[a-durations]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-recipes.json#L8070
[a-double]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/index.html#L332
[a-manual]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L341
[a-shuffle]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L57
[a-state]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L681
[a-idle]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-recipes.json#L5120
[a-bridge]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L472
[a-spring]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L101
[a-host]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L1756
[a-hidden]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L1782
[a-offscreen]: https://github.com/ArvinYin1/grokbot-animation-anatomy/blob/debd41625da89b4c4fde43b3b1f0add6dc113c02/js/vendor/grok-player.js#L1850
[b-manifest]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/data/extracted/manifest.json#L2
[b-declarations]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/data/extracted/avatar-engine.json#L2
[b-code]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/data/extracted/src/avatar-engine.js#L1
[b-test]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/test/engine.test.mjs#L12
[b-coverage]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/README.md#L95
[b-old]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/README.md#L103
[b-license]: https://github.com/RongleCat/grok-bot-avatar-engine/blob/a995c0675e36ba113de41ead3a5d9be3fcd567e3/LICENSE#L1
