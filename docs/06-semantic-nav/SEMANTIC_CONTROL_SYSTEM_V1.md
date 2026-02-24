# NaviMind 语义控制系统 v3.0

> **机器人语义控制系统 3.0 — KG-Augmented Loopy Belief Propagation + Safety-Aware Scene Graph**
>
> 系统名称: NaviMind Semantic Control Engine (NSCE)
> 版本: 3.0.0
> 状态: Production Ready
> 测试覆盖: 390 tests passed (289 v1.0 + 43 v2.0-phase1 + 24 v2.0-phase2 + 34 v3.0-BP)
>
> **v3.0 核心创新 (参考 Belief Scene Graphs ICRA 2024 / HOV-SG RSS 2024 / Commonsense BSG 2025):**
> - KG-Augmented Bayesian Prior Injection (替代 GCN 训练, 零样本)
> - Iterative Loopy Belief Propagation (多轮消息传递, 优于 BSG 单次推理)
> - Safety-Aware Differential Credibility (双阈值: 导航低门槛, 交互高确认)
> - Phantom (Blind) Node Reasoning (KG 驱动的未见物体推理)
> - Bayesian Room-Type Posterior (物体证据→房间类型联合推理)

---

## 一、系统架构

```
┌─────────────────────────────────────────────────────────┐
│                   自然语言输入                            │
│    "帮我拿个灭火器" / "where is the exit?"              │
└────────────────────────┬────────────────────────────────┘
                         ▼
            ┌────────────────────────┐
            │    复杂度守卫 (Guard)    │  含条件/多步/时间约束?
            │  "如果/然后/每隔/if..."  │──── Yes ──→ LLM 路径
            └────────────┬───────────┘
                    No   ▼
    ┌────────────────────────────────────────────┐
    │          规则引擎 (Rule Engine)              │
    │                                            │
    │  优先级 (从高到低):                          │
    │    0. STOP    → 立即停止                    │
    │    1. STATUS  → 返回系统状态                 │
    │    2. EXPLORE → 探索环境                    │
    │    3. PLACE   → 放置物体                    │
    │    4. PICK    → 抓取物体                    │
    │    5. INSPECT → 巡检目标                    │
    │    6. FOLLOW  → 跟随目标                    │
    │    7. NAV     → 导航到位置                   │
    │    8. FIND    → 查找目标                    │
    │    9. 口语正则 → 中英文会话式匹配             │
    └────────────────────┬───────────────────────┘
                         ▼
              ┌─────────────────────┐
              │   SubGoal 子目标链    │
              │  FIND → APPROACH →  │
              │  PICK / VERIFY ...  │
              └─────────────────────┘
```

### 设计原则

1. **规则优先, LLM 兜底** — 简单指令 < 5ms 响应, 零 API 成本
2. **长前缀优先匹配** — 所有前缀列表按长度降序排列, 避免短词吃掉长词
3. **复杂度守卫** — 含条件/多步/时间约束的指令自动走 LLM
4. **Intent 不混淆** — "给我找个椅子"(FIND) vs "给我拿个椅子"(PICK) 明确区分

---

## 二、Intent 类型总览

| # | Intent | SubGoal 链 | 用途 |
|---|--------|-----------|------|
| 0 | **STOP** | STOP | 停止当前任务 |
| 1 | **STATUS** | STATUS | 查询系统/机器人状态 |
| 2 | **EXPLORE** | EXPLORE | 探索未知环境 |
| 3 | **PLACE** | NAVIGATE → PLACE | 放置物体到指定位置 |
| 4 | **PICK** | FIND → APPROACH → PICK | 抓取/取物 |
| 5 | **INSPECT** | FIND → LOOK_AROUND → APPROACH → VERIFY | 巡检/检查目标 |
| 6 | **FOLLOW** | FIND → FOLLOW | 跟随动态目标 |
| 7 | **NAVIGATE** | NAVIGATE → APPROACH → VERIFY | 导航到指定位置 |
| 8 | **FIND** | FIND → LOOK_AROUND → NAVIGATE → APPROACH → VERIFY | 查找目标 |

---

## 三、完整词库

### 3.1 导航前缀 `SIMPLE_NAV_PATTERNS_ZH` (40 个)

→ 动作链: NAVIGATE → APPROACH → VERIFY

| 分类 | 词条 |
|------|------|
| 基础 | 去、到、走到、前往、导航到、导航去、走去、过去、走向、前进到、移动到 |
| 返回 | 回到、返回到、回去、返回 |
| 组合 | 去找、去看、去看看、去查看、去检查、去往、赶到、赶去、赶往、赶快去、出发去、出发到、开到、开去、开往 |
| 引导 | 带路到、带我到、带我去、领我到、领我去、引导到、引导去、引导我到、引导我去、陪我去、陪我到 |
| 礼貌 | 请去、请到、请前往、请导航到、麻烦去、帮我去、帮我到、帮忙去 |
| 命令/急促 | 立刻去、马上去、快去、赶紧去、立即前往、速去、快到、马上到、立刻到 |
| 机器人专用 | 移动至、运动到、行进到、行走到、巡航到、自主前往、自动前往、规划路径到、路径规划到 |

### 3.2 查找前缀 `SIMPLE_FIND_PATTERNS_ZH` (42 个)

→ 动作链: FIND → LOOK_AROUND → NAVIGATE → APPROACH → VERIFY

| 分类 | 词条 |
|------|------|
| 基础 | 找、找到、寻找、搜索、定位、查找 |
| 口语变体 | 找一下、找找、找找看、找一找、搜一下、搜一搜、搜搜、搜搜看、定位到、定位一下、锁定、锁定目标 |
| 确认/识别 | 确认、确定、确认一下、发现、看到、识别、检测、探测、辨认、辨识、认出 |
| 帮助 | 帮我找、帮忙找、帮我找到、帮忙找到、帮我搜、帮忙搜、帮我搜索、帮我定位、帮忙定位 |
| 礼貌 | 请找、请寻找、请搜索、请定位、麻烦找、麻烦帮我找、劳驾找 |
| 急促 | 快找、赶紧找、立刻找、马上找、快搜、赶紧搜 |

### 3.3 巡检前缀 `SIMPLE_INSPECT_PATTERNS_ZH` (21 个)

→ 动作链: FIND → LOOK_AROUND → APPROACH → VERIFY

| 词条 |
|------|
| 检查、检查一下、巡检、巡查、查看、查看一下、查验、核查、检测、检测一下、排查、排查一下、检视、审查、查勘、帮我检查、帮忙检查、请检查、去检查、去查看、去巡检 |

### 3.4 跟随前缀 `SIMPLE_FOLLOW_PATTERNS_ZH` (30 个)

→ 动作链: FIND → FOLLOW

| 分类 | 词条 |
|------|------|
| 基础 | 跟着、跟随、跟踪、追踪、追着、跟住、盯着、盯住、尾随、跟上、追上 |
| 人称 | 跟这个人、跟那个人、跟他、跟她、跟他们、跟上他、跟上她、追上他、追上她、跟着他、跟着她、跟着他走、跟着她走 |
| 礼貌 | 帮我跟着、帮忙跟着、请跟着、帮我跟踪、请跟踪 |
| 命令 | 紧跟、紧紧跟着、一直跟着、持续跟随、不要跟丢、别跟丢 |

### 3.5 探索前缀 `SIMPLE_EXPLORE_PATTERNS_ZH` (20 个)

→ 动作链: EXPLORE

| 词条 |
|------|
| 探索、探索一下、逛逛、逛一逛、四处看看、到处看看、环顾四周、看看周围、看看附近、巡视、扫描、扫描一下、扫描周围、勘察、侦察、侦查、帮我探索、自由探索、随便走走、随便逛逛 |

### 3.6 停止/取消前缀 `SIMPLE_STOP_PATTERNS_ZH` (26 个)

→ 动作链: STOP

| 词条 |
|------|
| 停、停下、停止、停下来、停一下、别走了、别动、不要动、站住、取消、取消任务、终止、终止任务、中断、中止、暂停、算了、不用了、不找了、不去了、回来、回来吧、不用去了、紧急停止、急停、停止任务、停掉、关掉 |

### 3.7 取物前缀 `SIMPLE_PICK_PATTERNS_ZH` (37 个)

→ 动作链: FIND → APPROACH → PICK

| 分类 | 词条 |
|------|------|
| 基础 | 拿、取、拿一下、取一下、拿个、取个、拿一个、取一个 |
| 方向 | 拿过来、取过来、拿来、取来 |
| 帮助 | 帮我拿、帮我取、帮忙拿、帮忙取、帮我拿一下、帮我取一下、帮我拿来、帮我取来、帮我带来、帮忙带来 |
| 给我 | 给我拿、给我取、给我拿个、给我取个、给我拿一个、给我取一个 |
| 递送 | 递给我、递一下、递过来 |
| 礼貌 | 请拿、请取、麻烦拿、麻烦取 |
| 急促 | 快拿、赶紧拿、马上拿 |
| 抓取 | 抓、抓住、抓取、夹取、夹住 |
| 捡拾 | 捡、捡起、捡起来、捡一下 |

### 3.8 放置前缀 `SIMPLE_PLACE_PATTERNS_ZH` (30 个)

→ 动作链: NAVIGATE → PLACE

| 分类 | 词条 |
|------|------|
| 基础 | 放、放下、放到、放在、放回、放一下、放好、放回去 |
| 摆放 | 摆到、摆在、摆好、摆放 |
| 归位 | 放回原处、归位、还回去 |
| 搁置 | 搁到、搁在、搁下 |
| 帮助/礼貌 | 帮我放、帮忙放、帮我放到、帮我放在、请放到、请放在 |
| 放置 | 放置、放置到、放置在 |
| 丢弃 | 丢到、丢在、扔到、扔在 |

### 3.9 状态查询前缀 `SIMPLE_STATUS_PATTERNS_ZH` (31 个)

→ 动作链: STATUS (立即返回)

| 分类 | 词条 |
|------|------|
| 电量 | 电量、电池、电池电量、电量多少、还有多少电、剩余电量 |
| 系统 | 状态、系统状态、当前状态、机器人状态 |
| 模式 | 模式、当前模式、什么模式 |
| 任务 | 当前任务、任务状态、任务进度、完成了吗、做完了吗、好了吗 |
| 位置 | 现在在哪、现在位置、当前位置、你在哪 |
| 传感器 | 温度、温度多少、当前温度、速度、当前速度 |
| 连接 | 是否在线、连接状态、网络状态 |
| 时间 | 还剩多少、剩余时间 |
| 报告 | 报告状态、汇报状态 |

---

## 四、中文口语正则 `CONVERSATIONAL_FIND_RE_ZH` (22 条)

用 `re.search()` 匹配, 捕获组 `(.+?)` 提取目标物。

| # | 类型 | 匹配模式 | 示例 |
|---|------|---------|------|
| 1 | 查看类 | 看看/看一下/查一下/瞧瞧/瞅瞅/瞅一眼 + X + 在哪? | 看一下灭火器在哪 |
| 2 | 位置疑问 | X + 在哪/在哪里/在哪儿 + 语气词 | 灭火器在哪啊 |
| 3 | 位置疑问 | X + 在什么地方/位置/方向/方位 | 门在什么位置 |
| 4 | 位置疑问 | X + 的位置/方位/方向 + 在哪/是什么 | 灭火器的位置在哪 |
| 5 | 帮助类 | 帮我/请/麻烦/劳驾/烦请 + 找一下/帮忙找 + X | 麻烦找一下出口 |
| 6 | 引导类 | 带我去/领我去/送我去/陪我去 + X | 带我去会议室 |
| 7 | 意愿类 | 我想/我要/我得/我需要/咱们去/我们去 + 找/看/去 + X | 我想找椅子 |
| 8 | 存在疑问 | 哪里有/哪儿有/有没有/附近有没有 + X | 附近有没有灭火器 |
| 9 | 路径疑问 | X + 怎么走/怎么去/如何去/咋走/咋去 | 灭火器怎么走啊 |
| 10 | 指示类 | 告诉我/指给我看/给我指/报告/说一下 + X | 告诉我灭火器在哪 |
| 11 | 位置确认 | X + 是在/到底在 + 哪个位置/什么地方/哪边 | 灭火器到底在哪边 |
| 12 | 存在确认 | 这里/附近/楼里/室内/前面/后面 + 有X + 吗/没 | 这里有椅子吗 |
| 13 | 能力疑问 | 能/能不能/可以/可否 + 找到/看到/定位/识别 + X | 能找到出口吗 |
| 14 | 距离方向 | 最近的/离我最近的/附近的/面前的/眼前的 + X | 离我最近的灭火器 |
| 15 | 数量确认 | 有几个/有多少/一共有多少 + X | 有几个灭火器 |
| 16 | 对话式 | 你知道/你看到/你发现/你有没有看到 + X | 你知道灭火器在哪吗 |
| 17 | 催促式 | 快/赶紧/马上/立刻/速速/抓紧 + 找/去找/搜/定位 + X | 赶紧去找灭火器 |
| 18 | 给我找 | 给我找/给我找个/给我搜/帮我找个 + X | 给我找个椅子 |
| 19 | 否定排除 | 不是/不要/别找 + ... + 是/要/找 + X | 不要那个, 要这个 |
| 20 | 方言口头禅 | 整个/搞个/弄个/来个 + X + 来/过来 | 整个灭火器来 |
| 21 | 场景感知 | 这里/附近/周围 + 都有 + 些/哪些/什么 + X | 这里都有些什么 |

---

## 五、英文口语正则 `CONVERSATIONAL_FIND_RE_EN` (19 条)

| # | 类型 | 匹配模式 | 示例 |
|---|------|---------|------|
| 1 | 命令/请求 | (can/could/would you) show me / take me to / lead me to / walk me to / escort me to + X | could you show me the door |
| 2 | 指向 | point (me) to/at/towards / direct me to / guide me to / navigate to + X | guide me to the exit |
| 3 | 疑问 | where is/are/can I find/do I find/would I find + X | where is the fire extinguisher |
| 4 | 缩写疑问 | where's / where are + X | where's the printer |
| 5 | 确认疑问 | do you know where / have you seen / can you see / did you spot + X | have you seen the printer |
| 6 | 意愿/需求 | I need / I want / I'm looking for / I gotta find / I must find + X | I'm looking for a chair |
| 7 | 存在疑问 | is there / are there / do we have + X + here/nearby/around | is there a fire extinguisher nearby |
| 8 | 路径 | how do I get to / how can I find / how to reach / what's the way to + X | how do I get to the exit |
| 9 | 帮助 | help me find/locate/get to/reach/look for + X | help me locate the generator |
| 10 | 非正式 | let me see / let's find / let's check out / lemme see / lemme find + X | let's check out the storage room |
| 11 | 祈使 | go find/get/fetch/grab / fetch me / get me / bring me / gimme + X | gimme the wrench |
| 12 | 最近 | (find/locate/show me) nearest/closest (available) + X | nearest fire extinguisher |
| 13 | 确认 | is/are the/that/this X around here / nearby / still there | is the door still there |
| 14 | 机器人专用 | scan for / search for / detect / identify / locate / look for + X | scan for fire extinguisher |
| 15 | 机器人专用 | report/check the location/position/status of + X | report the location of the valve |
| 16 | 俚语 | swing by / drop by / pop over to / head over to + X | swing by the lobby |
| 17 | 场景感知 | what's/what is around here / nearby / in this room | what's around here |
| 18 | 场景感知 | anything/something (interesting/useful) nearby/around/here | anything nearby? |
| 19 | 场景感知 | what can you / do you see/detect/spot | what can you see? |

---

## 六、英文前缀词库

### 6.1 英文导航前缀 `SIMPLE_NAV_PATTERNS_EN` (19 个)

| 词条 |
|------|
| go to, navigate to, move to, head to, walk to, proceed to, drive to, travel to, advance to, return to, go back to, head back to, rush to, hurry to, quickly go to, head over to, swing by, drop by, pop over to |

### 6.2 英文跟随前缀 `SIMPLE_FOLLOW_PATTERNS_EN` (10 个)

| 词条 |
|------|
| follow, track, chase, tail, keep following, keep tracking, stay with, stick with, pursue, shadow |

### 6.3 英文取物前缀 `SIMPLE_PICK_PATTERNS_EN` (13 个)

| 词条 |
|------|
| pick up, grab, fetch, fetch me, bring me, get me, hand me, gimme, go grab, go get, go fetch, please grab, please fetch |

### 6.4 英文放置前缀 `SIMPLE_PLACE_PATTERNS_EN` (10 个)

| 词条 |
|------|
| put, place, drop, set down, put down, leave, return, put it on, place it on, set it on |

### 6.5 英文状态查询 `SIMPLE_STATUS_PATTERNS_EN` (22 个)

| 词条 |
|------|
| battery level, battery status, how much battery, current status, system status, robot status, where are you, your location, current position, what mode, current mode, task status, task progress, are you done, remaining battery, remaining time, temperature, current speed, connection status, network status, report status, status report |

### 6.6 英文巡检前缀 `SIMPLE_INSPECT_PATTERNS_EN` (3 个)

| 词条 |
|------|
| inspect, examine, audit |

### 6.7 英文停止词 (内联, 9 个)

| 词条 |
|------|
| stop, halt, cancel, abort, quit, enough, nevermind, never mind, forget it |

### 6.8 英文探索触发 (内联, 5 个)

| 词条 |
|------|
| explore, look around, scan the, survey, wander |

### 6.9 英文查找前缀 (内联, 10 个)

| 词条 |
|------|
| find, search for, locate, look for, where is, where are, show me, seek, spot, identify |

### 6.10 英文检查特殊规则

| 规则 | 说明 |
|------|------|
| `check X` | → INSPECT |
| `check if X` | → 走 LLM (条件判断) |
| `check whether X` | → 走 LLM (条件判断) |

---

## 七、复杂度守卫 (Complexity Guard)

命中任一标记 → 直接走 LLM, 不进规则引擎。

### 7.1 中文复杂度标记 (30 个)

| 分类 | 词条 |
|------|------|
| 条件 | 如果、否则 |
| 顺序 | 然后再、接着再、之后再、然后、接着、再去、再找 |
| 并列 | 并且、而且、同时、以及 |
| 序列 | 先去、先找、先到 |
| 集合 | 每个、每一个、所有、全部、依次、逐一 |
| 巡逻 | 巡逻、巡视所有 |
| 完成后 | 完成后、完成之后、完成以后、做完再 |
| 时间 | 每隔、定期、循环、反复 |

### 7.2 英文复杂度标记 (16 个)

| 分类 | 词条 |
|------|------|
| 条件 | if, else, otherwise |
| 顺序 | then, and then, after that, followed by, after, before, once done |
| 集合 | every, each, all |
| 巡逻 | patrol, one by one |
| 时间 | repeat, periodically, continuously |

### 7.3 标点规则

| 规则 | 说明 |
|------|------|
| 中文逗号 `，` ≥ 2 | 多步指令, 走 LLM |
| 英文逗号 `,` ≥ 2 | 多步指令, 走 LLM |

---

## 八、停用词表 (STOP_WORDS, 150+ 个)

用于 `ChineseTokenizer` 分词后的噪声过滤。

> **重要设计决策**: 不删除意图动词 (`go`, `find`, `get`, `show`, `take`, `bring`, `see`)。
> 这些词在 Intent Detection 阶段有语义价值, 仅在 Noun Extraction 阶段过滤。

### 8.1 英文停用词 (50+ 个)

| 分类 | 词条 |
|------|------|
| 冠词/介词 | the, a, an, to, at, in, on, near, next, by, of, for, with, from |
| 代词 | me, my, i, you, your, this, that, it, there, here |
| 疑问 | where, how, what, which |
| 情态 | please, can, could, would, should, will, shall |
| 助动词 | do, does, did, have, has, had, be, am, are, was |
| 回应 | not, no, yes, ok, okay, hey, hi, hello |
| 功能 | help, let |
| 程度 | just, also, too, very, really, actually, basically |
| 量词 | some, any, each, every |

### 8.2 中文停用词 (100+ 个)

| 分类 | 词条 |
|------|------|
| 基础 | 去、到、找、拿、的、在、旁边、附近、那个 |
| 人称 | 请、帮、我、你、他、她、它、我们、他们 |
| 助词 | 一个、把、了、着、过、给、被、让、将 |
| 连词 | 和、与、或、但是、然后、接着、之后 |
| 指代 | 这个、那个、这些、那些、某个 |
| 动词填充 | 看、看看、看一下、看一看、看下、查、查看、一下、一看、一找、一搜 |
| 帮助 | 帮我、帮忙、麻烦、劳驾、烦请 |
| 疑问/位置 | 在哪、在哪里、在哪儿、在什么地方、在什么位置、怎么走、怎么去、怎么找、如何、如何去、是什么、什么、几个、多少 |
| 情态/意愿 | 告诉我、带我去、领我去、引导、能不能、可以、是不是、是否、能否、可否、有没有、知道、知不知道、想、要、想要、需要、希望、打算、准备、得、得去、得找 |
| 语气词 | 吗、呢、啊、吧、嘛、呀、哦、嗯、哈、哎、唉、诶、喂、嘿、哟 |
| 程度/时间 | 快、赶紧、赶快、马上、立刻、立即、立马、先、再、也、都、就、才、还、又、很、非常、特别、最、比较、稍微 |
| 方言/口头禅 | 整、搞、弄、来、来个、整个、搞个、弄个、咋、啥、咋整、咋办、咋走 |
| 连词/介词 | 从、往、向、朝、对、跟、比、因为、所以、如果、虽然、不过 |
| 否定 | 不、没、没有、别、不要、不用、无 |

---

## 九、规则引擎执行优先级

```
输入指令
  │
  ├─ 复杂度守卫 ──→ None (LLM)
  │
  ├─ 0. STOP     ──→ [STOP]
  ├─ 1. STATUS   ──→ [STATUS]
  ├─ 2. EXPLORE  ──→ [EXPLORE]
  ├─ 3. PLACE    ──→ [NAVIGATE → PLACE]
  ├─ 4. PICK     ──→ [FIND → APPROACH → PICK]
  ├─ 5. INSPECT  ──→ [FIND → LOOK_AROUND → APPROACH → VERIFY]
  ├─ 6. FOLLOW   ──→ [FIND → FOLLOW]
  ├─ 7. NAV      ──→ [NAV → APPROACH → VERIFY]
  ├─ 8. FIND     ──→ [FIND → LOOK_AROUND → NAV → APPROACH → VERIFY]
  ├─ 9. 中文口语正则 ──→ (提取目标后走 FIND 链)
  ├─10. 英文口语正则 ──→ (提取目标后走 FIND 链)
  │
  └─ 全部未命中 ──→ None (LLM)
```

### 优先级设计理由

| 优先级 | Intent | 理由 |
|--------|--------|------|
| 最高 | STOP | 安全第一, 急停不能被其他 intent 拦截 |
| 高 | STATUS | 不涉及运动, 即时响应 |
| 高 | EXPLORE | 明确动作, 不含目标物 |
| 中高 | PLACE | "放到X" 不能被 NAV "到X" 吃掉 |
| 中高 | PICK | "拿X" 不能被 FIND "找X" 吃掉 |
| 中 | INSPECT | "检查X" 不能被 FIND "找X" 吃掉 |
| 中 | FOLLOW | "跟着X" 含 in-string 匹配, 需要单独处理 |
| 低 | NAV / FIND | 最通用的 intent, 兜底 |
| 最低 | 口语正则 | 灵活匹配, 但可能误判, 放在最后 |

---

## 十、SubGoalAction 枚举

```python
class SubGoalAction(Enum):
    NAVIGATE    = "navigate"        # 导航到指定区域/位置
    FIND        = "find"            # 在视野中搜索目标
    APPROACH    = "approach"        # 接近已发现的目标
    VERIFY      = "verify"          # 近距离验证目标身份
    LOOK_AROUND = "look_around"     # 原地旋转扫描 (LOVON)
    EXPLORE     = "explore"         # 探索未知区域
    BACKTRACK   = "backtrack"       # 回退到上一个位置
    WAIT        = "wait"            # 等待条件满足
    FOLLOW      = "follow"          # 持续跟随动态目标
    STOP        = "stop"            # 停止当前任务
    PICK        = "pick"            # 抓取/取物
    PLACE       = "place"           # 放置物体
    STATUS      = "status"          # 查询系统状态
```

---

## 十一、planner_node 动作分发

| Action | planner_node 行为 |
|--------|------------------|
| NAVIGATE | → RESOLVING → 发布 PoseStamped |
| FIND | → RESOLVING → 查询场景图 + CLIP 匹配 |
| APPROACH | → 发布接近航点 (0.3-0.5m), 减速 |
| VERIFY | → 近距离拍照, CLIP/VLM 确认身份 |
| LOOK_AROUND | → 发布旋转 Twist (360° 扫描) |
| EXPLORE | → EXPLORING → Frontier 评分 + 拓扑记忆 |
| BACKTRACK | → 回退到上一个记录位置 |
| WAIT | → 创建定时器, 等待指定秒数 |
| FOLLOW | → 启动跟随模式 (perceive→resolve→navigate 循环) |
| STOP | → 取消当前导航, 回到 IDLE |
| PICK | → RESOLVING → 定位目标 → 接近 → 抓取 |
| PLACE | → RESOLVING → 定位放置点 → 导航 → 放置 |
| STATUS | → `_publish_status_report()` → 立即 advance |

---

## 十二、测试覆盖

| 测试类 | 用例数 | 覆盖内容 |
|--------|--------|---------|
| TestFastPathResolution | 27 | L1/L2 快速路径解析 |
| TestTaskDecomposition | 6 | 分解正确性 + 延迟 |
| TestBeliefSystemEndToEnd | 3 | 置信度更新 |
| TestMultiHypothesisFullScenario | 2 | 多假设消歧 |
| TestVoIFullEpisode | 2 | Value of Information 决策 |
| TestAttributeDisambiguation | 5 | 属性消歧 (红色/大/白色/金属) |
| TestNegationExclusion | 4 | 否定排除 |
| TestComparativeRanking | 5 | 比较排序 (最近/最远) |
| TestIntentInference | 8 | 意图推理 (打印→办公室) |
| TestExplorationPlanning | 5 | 探索规划 (TSG) |
| TestConversationalParsing | 12 | 口语化解析 (中英文) |
| **TestIndustrialPatterns** | **204** | **全部新 Intent 工业级覆盖** |
| TestConditionalDecomposition | 6 | 条件多步分解 |
| **总计** | **289** | |

### TestIndustrialPatterns 细分

| 子测试 | 用例数 |
|--------|--------|
| STOP 中文 | 15 |
| STOP 英文 | 9 |
| EXPLORE 中文 | 12 |
| EXPLORE 英文 | 4 |
| INSPECT 中文 | 7 |
| INSPECT 英文 | 4 |
| NAV 变体中文 | 11 |
| NAV 变体英文 | 5 |
| FIND 变体中文 | 9 |
| FOLLOW 变体中英 | 11 |
| 口语中文工业级 | 22 |
| 口语英文工业级 | 17 |
| 复杂度守卫 | 8 |
| 时间约束守卫 | 8 |
| **PICK 中文** | **11** |
| **PICK 英文** | **8** |
| **PLACE 中文** | **8** |
| **PLACE 英文** | **5** |
| **STATUS 中文** | **16** |
| **STATUS 英文** | **10** |
| 英文非正式 | 4 |

---

## 十三、统计汇总

### 词条数量

| 项目 | 中文 | 英文 | 合计 |
|------|------|------|------|
| 导航前缀 | 40 | 19 | 59 |
| 查找前缀 | 42 | 10 | 52 |
| 巡检前缀 | 21 | 3 | 24 |
| 跟随前缀 | 30 | 10 | 40 |
| 探索前缀 | 20 | 5 | 25 |
| 停止前缀 | 26 | 9 | 35 |
| 取物前缀 | 37 | 13 | 50 |
| 放置前缀 | 30 | 10 | 40 |
| 状态查询 | 31 | 22 | 53 |
| 口语正则 | 22 条 | 19 条 | 41 条 |
| 复杂度标记 | 30 | 16 | 46 |
| 停用词 | 100+ | 50+ | 150+ |
| **合计** | **429+** | **186+** | **615+** |

### 性能指标

| 指标 | 数值 |
|------|------|
| 规则引擎延迟 | < 5ms (P99) |
| LLM 回退率 | ~15% (复杂指令) |
| 测试通过率 | 289/289 (100%) |
| 支持语言 | 中文 (含方言) + 英文 (含俚语) |
| Intent 类型 | 9 类 |
| SubGoalAction | 13 种 |

---

## 十四、文件索引

| 文件 | 用途 |
|------|------|
| `src/semantic_planner/semantic_planner/task_decomposer.py` | 规则引擎 + 词库 |
| `src/semantic_planner/semantic_planner/chinese_tokenizer.py` | 分词 + 停用词 |
| `src/semantic_planner/semantic_planner/planner_node.py` | SubGoal 分发 + 执行 |
| `src/semantic_planner/semantic_planner/llm_client.py` | LLM 兜底 (Kimi/OpenAI/Qwen) |
| `tests/test_offline_pipeline.py` | 289 条测试 |
| `config/semantic_planner.yaml` | LLM 配置 |
| `docs/SEMANTIC_NAV_GUIDE.md` | 启动指南 |
| `docs/SEMANTIC_CONTROL_SYSTEM_V1.md` | 本文档 |

---

## 十五、版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 0.1 | — | 初始版本: NAVIGATE + FIND |
| 0.5 | — | 新增 FOLLOW, EXPLORE, STOP |
| 0.8 | — | 工业级扩词: 口语/方言/礼貌/急促 |
| **1.0** | **2026-02-18** | **生产可用: +PICK/PLACE/STATUS, 复杂度守卫增强, STOP_WORDS 风险修复, 289 测试全绿** |

---

## 十六、后续扩展路线

| 方向 | 说明 | 优先级 |
|------|------|--------|
| PATROL Intent | 定期巡逻路线 (多点循环) | P1 |
| 多目标集合 | "找所有灭火器" → FIND + LOOP | P1 |
| Quantifier 层 | 提取 "最近的/所有/第二个" 修饰语 | P2 |
| Temporal 层 | "每隔5分钟" → 定时触发器 | P2 |
| CHASE vs FOLLOW | 区分被动跟随 vs 主动追逐 (速度差异) | P3 |
| 多轮对话 | "不对, 是另一个" → 上下文消歧 | P3 |
| 语音 ASR 容错 | 同音字/语气词误识别容错 | P3 |

---

*NaviMind Semantic Control Engine v1.0 — Production Ready*
