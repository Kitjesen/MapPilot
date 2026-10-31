# LingTu SDK 六大 Client 架构与改造路线

**Status:** 实施方案；六组命名已确定，代码迁移与功能修复尚未执行
**Audience:** SDK、Gateway、Host、原生运行时与验收维护者
**Runs on:** 本地契约测试、原生仿真及所选 RobotConfig 的 Linux 实机

更新：2026-10-26。

本轮交付是仓库方案，明确改造任务、依赖和验收标准。后续实施将现有 SDK 收拢为一个总入口和六个业务 Client，先修复请求契约与运行接线，再补齐主业务流程。本文的 P0–P3 表示本方案的依赖阶段，不是另一份全局优先级队列；当前执行顺序仍由 [Roadmap](../roadmap.md) 决定。

本方案沿用 [架构](../architecture.md) 和 [验证政策](../testing.md)：ProductControl 拥有 Product 生命周期，Host 装配进程内 Module，原生导航拥有最终运动仲裁，只有 `lingtu-driver` 向 Brainstem 转发检查后的命令。保留多平台 RobotConfig、标定和驱动适配边界。

## 目标接口与当前实现

`LingTuClient` 统一管理 Gateway 地址、API key、超时与请求处理。六个 Client 按业务组织函数；异步版本提供相同契约，不增加业务分组。

| 目标 Client | 目标入口 | 职责 | 当前归属 | 当前公开方法数 |
|---|---|---|---|---:|
| NavigationClient | `robot.navigation` | 导航、定位、探索 | `navigation`，保留 | 15 |
| MapClient | `robot.map` | 地图资产、保存操作、地点标记 | `map`，保留 | 14 |
| MissionClient | `robot.mission` | Agent、跟随、巡检与报告 | `tasks / TaskClient`，改名 | 13 |
| VisionClient | `robot.vision` | 相机快照、场景、历史观测 | 从 `observations` 拆出 | 4 |
| RecordingClient | `robot.recording` | 传感器与巡检证据录制、记录管理 | 从 `observations` 拆出 | 3 |
| ControlClient | `robot.control` | 状态、诊断、运行管理、控制权、软件急停 | `system / SystemClient`，改名 | 14 |
| 合计 | 六个业务入口 | | 当前仍是五组 | 63 |

63 是本次源码快照中公开业务方法的数量，不含根级兼容委托、私有辅助函数和异步镜像；修复与扩展后不要求维持这个数字。当前方法以 [SDK 源码](../../src/lingtu/sdk/) 和 [SDK 使用说明](../../src/lingtu/README.md) 为准。

| 其他入口 | 定位 |
|---|---|
| `AsyncLingTuClient` | 与同步版相同的业务划分、参数、结果和错误语义 |
| 历史 `LocalizationClient` | 保留原有 `relocalize`、`global_relocalize`、`start_map_tracking` 三个命令的兼容入口，委托 Navigation |
| `LingTuMCP` | 独立的 HTTP JSON-RPC 工具适配器，不是第七个业务 Client |
| `ProductControl` | 本地公共生命周期所有者；远程 ControlClient 只调用它的传输入口 |

代码目录的目标如下。`models.py`、`_common.py`、异步适配和现有 MCP 文件继续复用，不另建一套 SDK 框架。

```text
src/lingtu/sdk/
  client.py          LingTuClient
  navigation.py      NavigationClient
  maps.py            MapClient
  mission.py         MissionClient
  vision.py          VisionClient
  recording.py       RecordingClient
  control.py         ControlClient
  localization.py    原有定位兼容入口
  async_client.py    AsyncLingTuClient
  async_domains.py   异步领域适配
  models.py          公共结果类型
  _common.py         现有公共辅助函数
  mcp.py             LingTuMCP
```

SDK 的 `control.py` 位于 `lingtu.sdk`，不取代 `lingtu.control` 中的 ProductControl。MapClient 保存地图资产；切换活动导航地图属于 ProductControl 事务。Mission 方法继续区分 Agent `run_id`、Follow 的 `follow_task_id + product_session_id` 和巡检 `task_id`，不制造一个猜测任务类型的通用 `cancel(id)`。

## 能力状态与证据

每项工作分开记录以下三种状态，不能只用一个“完成”覆盖全部层级。

| 状态 | 必须有什么证据 | 不能据此宣称 |
|---|---|---|
| 方法已定义 | 公共方法、参数、导出和说明存在 | 请求能够到达真正执行者 |
| 调用链已打通 | SDK、实际 Gateway 路由、实际 Host 所有者和下层接口的契约一致 | 已证明仿真运动或实机行为 |
| 功能已验收 | 在命名的平台、版本、Product 和场景中保留成功及失败处理证据 | 其他机器人、地图和部署同样合格 |

例如，当前 `search_memory(query)` 已定义，但 SDK 发文本而服务端要求向量；导航暂停/继续已有 HTTP 路由和原生命令，却缺少实际 GoalService 和账本的接入。录制只公开三个 SDK 方法，但后端已有目录、文件下载、回看和证据绑定，不能按 SDK 方法数估计后端成熟度。

## 通信与所有权

Gateway 是 Host 内的 Module。表中的 Gateway 到 Host 是同一 Python 进程中的协作，不是要求部署第二个 Host 服务。MCP 调用方直接访问 MCP 入口，不经过六个业务 Client；Web 也可直接使用 Gateway 和媒体接口。

| 链路 | 当前通信 | 所有者与边界 |
|---|---|---|
| SDK 到 Gateway | HTTP JSON、JPEG 或文件响应，默认 5050 | 根客户端管理地址、认证和超时；构造不发网络请求，不获取控制权 |
| Gateway 与 Host Module | 进程内调用、消息端口 | Agent、感知、任务与状态各有自己的所有者 |
| 导航命令 | Host 调用原生 C ABI，再由 CycloneDDS 发送命令与接收 ACK | 原生导航决定执行；ACK 不等于到达或停稳 |
| 重定位与地图跟踪 | Gateway 调用本机 `slamctl`，工具访问原生 SLAM | 保留 SLAM 的进程和传感器所有权 |
| 地图管理 | 本机 Unix socket 到 `mapd` | 在线地图数据另由 DDS 传输；下载经校验后的文件访问返回 HTTP 流 |
| 录制控制 | Gateway 调用本机原生命令，解析 JSON 结果 | C++ 录制管理器拥有会话、进程和持久化；数据来自 DDS 和相机 SHM |
| 大图像 | 相机共享内存 SHM | 完整图像不描述为全部经 DDS 传输 |
| Product 控制 | Gateway 转发到独立、本机 HTTP 服务，默认 5051 | ProductControl 独占切换和停止事务，Host 重启不承担事务恢复 |
| 实时视频 | HTTP/WHEP 信令、WebRTC 媒体；JPEG WebSocket 回退 | 包装信令请求不等于 Python 已实现视频接收 |
| 事件与遥控 | SSE、WebSocket | 保留现有事件覆盖范围和遥控会话契约 |
| MCP | HTTP JSON-RPC，默认 8090 | 认证应按明确规则继承或覆盖根配置；工具目录由 Host 提供 |

来源：[SDK 请求实现](../../src/lingtu/sdk/client.py)、[地图传输](../../src/gateway/maps/transport.py)、[录制适配](../../src/gateway/services/recording.py)、[实时接口](../../src/gateway/routes/realtime.py)、[视频接口](../../src/gateway/routes/video.py)、[ProductControl 转发](../../src/gateway/routes/product_control.py)。

沿用现有根客户端的请求函数，集中认证、超时、错误解析和可复用的二进制读取/下载处理。需要复用时提取小函数即可，不增加连接框架、传输插件或新依赖。当前 `urllib` 实现不承诺持久 TCP 会话；异步版仍可复用 `asyncio.to_thread`。

## P0 修复真实契约与接线

以下任务均未实施。P0 可以按独立所有者分工，但同一 SDK 文件的修改须串行整合，随后再进行 P1 移动。

| 任务 | 具体改动 | 主要位置 | 本地通过条件 |
|---|---|---|---|
| P0-01 三维目标与点击朝向 | SDK 导航和兼容入口增加 keyword-only `z`；点击请求增加可选 yaw，区分省略和显式零方向 | `sdk/navigation.py`、根与异步适配、`gateway/schemas.py`、`gateway/navigation/goals.py` | 经真实请求模型/路由后 XYZ 完整；省略方向与显式 0/非零分别正确；非有限数值被拒绝 |
| P0-02 暂停继续接线 | 补 GoalService 入口，扩展现有任务账本的命令类型、ACK 校验和重放状态 | `nav/services/goals.py`、`task_ledger.py`、Gateway 导航调用 | 经真实 GoalService 派发正确命令；身份冲突、错误 ACK、超时和重试行为准确；不提前改变执行状态 |
| P0-03 历史语义契约 | 提供明确的 embedding 检索入口，传递现有过滤字段；原文本入口明确报告尚未支持 | `sdk/observations.py`（P1 后为 vision）、Gateway schema/route、TemporalStore 契约测试 | SDK 到真实路由及临时存储查询成功；文本、空向量、NaN/Inf 明确失败；不宣称已完成文本搜索 |
| P0-04 MCP 配置传递 | API key 与 timeout 默认继承根配置，显式参数覆盖；独立构造保持可用 | `sdk/mcp.py`、`tests/lingtu/sdk/test_mcp.py` | 工具调用、工具列表和健康请求使用预期配置；正确 key 可访问受保护路由，错误 key 仍被拒绝 |

### 点击导航的行为决定

普通 `go()` 保留原有 yaw 默认值和位置参数顺序，新增 keyword-only `z`。点击导航将默认 yaw 设为 `None`，Gateway 同样接受可选 yaw：省略方向保持既有的 position-only 执行含义；显式 `0.0` 和非零方向开始被真实传递。根级 `navigate_click` 和异步入口同步调整。

这是有意的契约修复：旧实现虽然发出了 yaw，服务端却没有这个字段。显式 yaw 生效不能包装成纯重命名。已有目标构造和原生接口支持“不指定朝向”，不需要新目标协议。

来源：[SDK 导航](../../src/lingtu/sdk/navigation.py)、[请求模型](../../src/gateway/schemas.py)、[目标构造](../../src/gateway/navigation/goals.py)。

### 暂停继续的完整修复边界

当前账本准入只接受 `goal/cancel`，ACK 类型映射也未包含暂停和恢复。实现应沿用已有持久准入和事件记录，不绕过账本直接发命令。

| 子任务 | 必须保留的语义 |
|---|---|
| 增加 `submit_pause`、`submit_resume` | 复用取消请求的准入、派发、记录回执顺序；传递 task、request、reason |
| 扩展命令与 ACK 类型 | 账本接受 `pause/resume`，对齐原生 `TASK_PAUSE/TASK_RESUME`；校验 kind、task_id、request_id |
| 重放与冲突 | 相同请求和内容不重复派发；冲突内容不得复用身份；保留 Product/map 身份限制 |
| 超时与拒绝 | 超时保持未确认，原生拒绝保留原因；不因 SDK 重试重放运动 |
| 执行状态 | ACK 只表示请求受理；只有原生生命周期事件推进到 PAUSED 或恢复执行；PAUSED 为非终态 |

测试必须包含 Gateway → **实际 GoalService** → 类型化命令接口的路径。现有路由测试中自行实现缺失方法的 stub 可继续测试路由，但不能证明本次接线修复。

来源：[GoalService](../../src/nav/services/goals.py)、[任务账本](../../src/nav/services/task_ledger.py)、[原生命令适配](../../src/nav/commands/module.py)、[Gateway 控制转发](../../src/gateway/navigation/commands.py)。

### 语义检索的范围决定

P0 新增明确命名的 `search_memory_by_embedding(embedding, *, top_k, since, label)`，复用服务端已有向量查询。原有 `search_memory(query)` 和根级文本签名保留可识别的入口，但返回明确的“文本语义检索尚未配置/实现”错误和迁移说明，不再发送必然失配的文本请求。保留参数名称不代表保留错误的成功承诺。

查询向量必须与历史特征处于同一编码空间。当前历史库保存检测器提供的 `clip_feature`，没有模型身份；仅维数相同不能证明可比较。不能直接复用另一记忆模块的文本编码器，也不能将词法哈希或其他同维模型当作替代。

自然语言检索单列为后续任务：明确历史特征生产模型、权重/预处理和维度，通过现有 Registry 与编码协议装配匹配的文本编码能力；缺少匹配能力时明确不可用。任何新增依赖仍需单独授权。P0 的向量契约通过不关闭这项自然语言功能缺口。

来源：[当前错误请求](../../src/lingtu/sdk/observations.py)、[语义查询路由](../../src/gateway/routes/operations.py)、[历史写入](../../src/memory/modules/temporal.py)、[TemporalStore](../../src/memory/storage/temporal_store.py)、[已有编码协议](../../src/runtime/encoder_protocol.py)。

## P1 完成六组结构迁移

| 任务 | 工作范围 | 验收标准 |
|---|---|---|
| P1-01 迁移实现 | tasks → mission，observations → vision/recording，system → control；根属性、类名、模块名同步 | 业务实现只有一个所属位置；不保留一套旧实现再包一层新类 |
| P1-02 兼容原公开接口 | 保留早于本轮分组的扁平根方法、公共结果类型和三项 Localization 命令 | 原调用除 P0 明确说明的错误契约修正外，参数和请求语义保持 |
| P1-03 同步异步与调用方 | 更新 exports、async adapters、CLI、真实调用方、测试和文档生成源 | 六组方法与异步版参数/结果一致；CLI 已有命令保持；没有遗漏导入 |
| P1-04 共享请求处理 | 复用根 HTTP 辅助函数；消除实际重复的认证、超时与二进制处理 | 六组请求继承同一配置；构造不访问网络、不获取控制权、不启动 Product |

`tasks/observations/system` 是本轮在本地引入的分组，尚未作为这轮改造的部署产物交付；仓库调用方统一迁移，不为它们再建永久兼容层。真正已有的根级 API 与 Localization 兼容入口保留。实施前重新搜索调用方；若发现具体外部部署依赖，应记录该依赖再调整迁移范围。

文档更新同时覆盖 [SDK 指南](../../src/lingtu/README.md)、[API 生成源](../../tools/docs/extract_api_docs.py) 与生成后的 [API](../api.md)。`--check` 只证明生成源与产物一致，不能证明生成源中的手写说明符合真实代码，仍须人工核对名称与示例。

## P2 补齐主业务流程

新增函数名是实施候选，不表示当前已导出。先补实际工作流程需要的调用，不要求一次暴露每个内部端点。

| 任务 | 优先接口 | 已有与待补边界 | 验收标准 |
|---|---|---|---|
| P2-01 Product 生命周期 | `product_status`、`switch_product`、`product_operation`、`stop_product`、`wait_until_ready` | 远程已有状态/操作查询和 map/nav switch；没有远程 stop。新增 stop 只转入已有 ProductControl.stop，不复制事务 | 切换/停止有可追踪回执；Host 重启不丢事务；运行身份不匹配时不停止新会话；就绪由真实状态决定 |
| P2-02 导航任务管理 | `cancel_task`、`pause_task`、`resume_task`、`wait_until_localized` | 指定取消已有完整后端；暂停继续依赖 P0-02 | 按指定任务查询和控制，不把另一任务状态当结果；超时、失败、暂停和终态可区分 |
| P2-03 巡检闭环 | 路线增删查、任务列表、证据查询/复核/下载、指定巡检等待 | Gateway 已有相关能力；Agent/Follow/Inspection 身份保持各自语义 | 路线及修订绑定准确；报告引用实际证据；取消巡检不冒充软件急停；重复查询不推进任务 |
| P2-04 录制管理 | `list/get`、目录与流信息、文件/整包下载、摘要/序列/场景/相机回看、校验与删除；启动补 `topics` | 后端已有。按 `expected_session_id` 停止需补 Gateway 请求和 SDK；原生层已有该参数 | 记录关联正确任务/Product；空 topics 的相机独立录制语义保留；停止身份不匹配不影响其他会话；导出内容对应选中记录 |
| P2-05 常用地图管理 | `delete`、`import_pcd`、`operations` | Gateway/mapd 已有；激活地图仍归 ProductControl | 导入/保存结果与操作身份一致；错误不被报为成功；原生所有者的活动地图约束保留 |
| P2-06 观测查询完整性 | `camera_status`、`media_status`、历史查询的 label/since/near/radius/limit | 后端已有；场景为缓存投影，不是每次重新推理 | 参数完整传递；无数据/服务不可用有明确结果；图像、场景和时间语义准确 |
| P2-07 运行身份与结果 | SessionInfo 暴露 `env/product/product_session_id`；区分请求受理与执行结果 | 服务端已有字段；旧 `set_mode` 不能充当 Product 切换 | SDK 不强迫调用方解析 raw 才能取得运行身份；文档和状态结果区分受理、运行、终态、停止证据 |

P2-01 使用现有请求回执/操作查询机制，并传递 ProductControl 已有的预期运行身份约束。启动由切换事务处理，不增加一套 SDK 进程启动器。软件急停、导航取消、任务取消、停止录制和停止 Product 各自保留明确名称与所有者。

来源：[生命周期传输](../../src/lingtu/control_server.py)、[ProductControl](../../src/lingtu/control.py)、[巡检接口](../../src/gateway/routes/inspection.py)、[录制接口](../../src/gateway/routes/recordings.py)、[地图接口](../../src/gateway/maps/routes.py)、[会话模型](../../src/lingtu/sdk/models.py)。

## P3 高级能力

| 任务 | 前置条件 | 完成标准 |
|---|---|---|
| SSE/WS 订阅 | 现有事件范围与主题明确；基础查询已稳定 | 重连和关闭可控；不把 SSE 没有覆盖的任务事件宣称已订阅；不新增业务服务 |
| Python 实时视频 | 媒体端点与信令可用；确有直接消费视频需求 | 信令、实际帧接收、会话释放分别验证；新增媒体依赖需明确授权 |
| 专业地图编辑 | 地图资产与操作身份已对齐 | 裁剪、区域/体素编辑、另存与规划校验调用现有 mapd 能力；二维展示不授权三维运动 |
| 文本历史语义检索 | 明确且匹配的图像/文本编码空间与 Host 所有者 | 已知样本和真实配置模型的检索有效；模型缺失或空间不匹配明确失败，不以同维向量或假编码通过 |
| 更完整的任务等待和回放工具 | 真实任务身份、事件和录制接口稳定 | 等待跟随指定所有者；回放保留原记录时间与会话，不伪装实时状态 |

音频/TTS 尚未进入本轮公共 SDK 范围；Go2 AudioHub 的平台试验不能自动推广为所有 RobotConfig 的功能。人工遥控若后续公开 SDK，继续复用既有会话、序号、租约及原生仲裁契约。

## 实施依赖与交付边界

```text
P0 独立契约修复及回归
  → P1 六组迁移及调用方一致性
    → P2 主业务流程接口和运行身份
      → P3 按实际需求扩展

每个阶段：本地契约 → 受影响的原生验证 → 所需仿真/实机证据
```

实机与原生仿真验收跟随相关功能推进，不等高级地图编辑或视频功能全部做完才验证主流程。当前 Roadmap 记录的正式 Thunder DDS/MuJoCo 起步阻塞属于已有运行问题，仍按原任务关闭；SDK 命名迁移不能替代其修复，也不据此安排新包实机运动。

| 阶段 | 交付物 | 阶段完成的含义 |
|---|---|---|
| P0 | 已确认缺陷的代码修复、契约回归、行为变更说明 | 已修链路有本地证据；文本检索等仍未实现的能力明确保留缺口 |
| P1 | 六组代码、导出、CLI、同步异步和文档 | 新入口可发现，真实旧调用可迁移；不因此宣称运动验收 |
| P2 | 生命周期与主业务接口、集成测试、指定环境流程证据 | 启动、查询、等待、取消和停止能够闭合，失败状态清楚 |
| P3 | 被选择的高级功能及各自验证记录 | 只对已实际验证的传输、模型和场景作完成声明 |

## 验收矩阵

| 验收项 | 本地合同与接线证据 | 原生或目标环境补充证据 |
|---|---|---|
| 根配置与认证 | 构造无网络；HTTP/MCP 的地址、认证、超时和拒绝行为正确 | 命名发布版本在目标网络中认证和连通 |
| 六组与兼容入口 | 导出、旧根方法、原 Localization、CLI 和文档一致；同步异步参数/结果相同 | 本项本地证据即可，不用运动证明改名 |
| 三维目标与点击方向 | XYZ 不丢失；省略/显式 yaw 分别正确；无效值拒绝 | 原生目标与执行记录的坐标、朝向相符 |
| 暂停继续 | 真实 GoalService 与账本；ACK 身份、幂等、拒绝、超时、重启重放；ACK 不改执行状态 | 同一导航任务暂停后有零输出/所需停稳证据；恢复原任务，不重发新目标 |
| 历史检索 | SDK → 实际 Gateway → 临时 TemporalStore；已知向量排序与过滤；无效输入明确失败 | 文本检索另需真实匹配模型，人工向量不证明语义效果 |
| 地图资产 | 操作身份、保存/下载/重命名/删除及错误处理正确 | 地图真实保存后重启/重载，定位使用该地图的身份与内容 |
| Product 生命周期 | 真实转发协议、操作回执、预期身份和失败恢复；本地 stop 与远程 stop 同一所有者 | 在选定 runner 中切换/停止，进程清理及停止证据完整 |
| 巡检与录制 | 路线/任务/录制身份对应；数据筛选、指定会话停止、证据和导出内容正确 | 记录真实采集及原生任务事实；相机采集按平台支持范围验收 |
| 支持用法中的失败 | 超时、错误认证、服务缺失、重复请求、旧会话引用都有明确结果；不自动重放运动 | 选定场景下验证中断、恢复或明确失败，不虚报执行成功 |
| 完整产品流程 | 用真实所有者和标准入口验证流程编排，不用假的业务服务替代整链路 | **建图 → 保存 → 重启或重载 → 定位 → 自主导航 → 停止 → 再次运行**，保存地图、任务结果、终止输出与清理证据 |

完整流程中的每次 Product 切换都保存自己的 `product_session_id`；建图和导航不要求共用同一 ID。地图身份、版本/发布记录和前后操作应可关联，禁止用不同运行的成功片段拼成一次通过。

验证层级沿用 [Testing](../testing.md)：本地契约、原生仿真、实机无运动、受监督实机运动。每层记录平台、发布版本、RobotConfig、Product、运行身份、输入、实际结果和证据路径。仿真通过不能替代实机标定；进程存在不能证明就绪；命令回执不能证明运动完成。

## 实施时的检查落点

下列是后续实施的检查入口，本轮方案整理不执行运行测试。每次只运行能揭示本次修改失败的检查；失败后修复对应契约、接线或文档，不用整仓反复回归替代定位问题。

| 修改 | 优先复用的测试或检查 | 可揭示的失败 |
|---|---|---|
| SDK 目标、命名、兼容、异步、CLI | [SDK 测试](../../tests/lingtu/sdk/) | 请求字段丢失、旧调用失败、异步遗漏、CLI 指向旧入口 |
| 暂停继续与目标语义 | [GoalService 测试](../../tests/nav/services/test_goal_service.py)、[暂停继续路由测试](../../tests/gateway/test_navigation_task_pause_resume.py)、[Gateway 命令测试](../../tests/gateway/test_gateway_commands.py) | 假服务掩盖缺方法、账本不支持新命令、错误 ACK、执行状态提前改变 |
| 向量检索 | [会话与地图契约测试](../../tests/gateway/test_gateway_session_map_contract.py) 中增加实际临时存储路径，结合 SDK 请求 | SDK 和服务端字段不一致、筛选失效、无特征记录被虚报命中 |
| MCP 认证 | [SDK MCP 测试](../../tests/lingtu/sdk/test_mcp.py)、[Gateway MCP 认证测试](../../tests/gateway/test_mcp_auth.py) | GET/POST 漏认证、配置继承/覆盖错误 |
| Python 修改 | 受影响文件 Ruff；SDK mypy；触及层次所有权时使用现有架构校验 | 新导入、类型及层次方向错误 |
| API 文档 | `python tools/docs/extract_api_docs.py --check`，并核对生成源中的手写说明 | 生成产物过期；手写说明仍需与真实接口逐项核对 |
| 仓库文档 | 现有文档链接/结构校验 | 文件位置错误、目录入口遗漏、源码引用失效 |

所有实施项当前为待执行。实施记录应把“实现完成”和“各平台验收”分开更新，并链接具体证据；不得用本文的任务表覆盖 Roadmap 的当前工作队列。
