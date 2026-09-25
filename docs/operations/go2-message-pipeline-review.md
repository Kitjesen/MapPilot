# Go2/NX 消息链路、容量与拷贝审查（2026-09-23）

审查对象是当前工作区的传感器 → SLAM → mapd/导航 → Gateway → 浏览器链路。
下文保留修复前的 R1–R8 复现和原因。**八项已安装到 NX `.55`，
完成 ARM 候选回归和实机静止建图验证；保存期间与运动中的表现仍待现场检验。** 原始隔离探针保存在
`build/go2-message-review-20260923/`；连接与部署状态见 [现场交接](go2-offline-mapping.md)。

## 本地修复状态

| 项 | 当前处理 | 验证边界 |
| --- | --- | --- |
| R1 IMU 中断 | 只有成功接纳新扫描才推进估计时间；超过 IMU 同步容限进入 `imu_gap_waiting_for_scan` 降级，恢复扫描后返回 Tracking | FastLIO 原生停止/恢复回归通过 |
| R2 同步保存 | 主循环冻结一致快照，PCD、patch、轨迹、校正重投影及重定位地图加载交给一个后台任务；完成后主循环提交并回 ACK | 原生测试在保存未完成时继续处理新扫描；冻结 iKD 树点云仍在主线程，NX 最坏耗时待测 |
| R3 关键帧丢失 | 待处理队列满时从完整 patch 历史重试投递；达到累计上限时暂停建图、保持已采集资料可保存，状态明确降级 | 原生积压和上限回归通过；到上限必须保存并开启新会话，尚不支持无限长采集 |
| R4 回调积压 | 跨线程入口先进入每订阅者有界待处理区，每订阅者只排一个唤醒回调；暴露待处理深度和点云字节数 | 暂停事件循环的 256 帧探针由 256 个回调/8 MiB 降为 1 个回调/64 KiB |
| R5 状态挤掉事件 | 状态按类型保留最新，一次性事件优先；每订阅者独立的 `reliable_seq` 只统计逐次事件，网页只据此检查丢失 | ACK 加 129 条位姿、过滤订阅及 SSE 路由回归通过；事件自身超过容量仍由权威状态刷新兜底 |
| R6 整图重复复制 | 不可变共享点云快照和 saved-map revision；文件快照只在相应扫描/版本更新时写 | 原生重复读取指针恒等回归和 slamd 构建通过 |
| R7 预览峰值 | 增量汇入有界体素预览，优化修正时重新汇入，不再先拼全部历史点数组 | 在线优化原生回归通过；NX 峰值内存和耗时待测 |
| R8 HTTP 挂起 | 每轮请求含响应体读取设 10 秒期限，切换/停止时 abort，超时按原节奏重试 | 网页挂起请求回归 11/11 通过 |

本地 `slamd`、FastLIO、在线优化和 SLAM 契约构建及运行通过；相关 Gateway 队列
与 SSE 路由回归 13 项通过，网页生产构建、定向 ESLint、Python Ruff 通过。
NX 独立候选目录 `/home/unitree/lingtu-message-candidate-20260923/` 的 ARM `slamd`
构建以及 FastLIO、在线优化、SLAM 契约三项程序运行通过；最终候选配置已启用
BBS3D，全局搜索与 `.54` 一样可链接。NX 系统 PCL 1.10 不满足 Small GICP 的
最低版本要求，因此保留 PCL GICP 精配准。发布包基于 `.54` 的其余原生组件，
仅替换 `slamd`、本轮 Gateway 文件和重新构建的网页；安装时校验了清单。

2026-09-23 安装 `v2.3.0-go2.20260923.55`，保留 `.54` 回退目录。NX 的 Python 3.8
暴露原打包脚本在 OTA 清单生成时使用 `str.removeprefix`；已改为对已校验版本号切片，
重新打包及安装器干运行通过。安装后通过 ProductControl 启动 `map / camera`，
会话 `product-c7664ffedc6a42d9bab0ae5fc4e51db1`。静止 10 秒观测序号
`2571 → 2669`、`TRACKING`、雷达和 IMU 丢帧均为 0；Gateway 报告遥控客户端 0、
控制权 `NONE`、运动 `QUIET`，小电脑有线直连网页返回 HTTP 200。`903room` 仍为
`READY`，`floor9` 文件仍在；旧静止保存任务 `save-audit-20260922-54-static` 查询为
`operation_not_found`，不能据此认定那次保存成功。未发送运动指令或验证地图保存/导航。

二进制与候选逐字一致，`navd` 和 driver 与 `.54` 逐字一致。当前 `slamd` 及 `.54`
都仍以固定 RUNPATH 引用 NX 上的 CycloneDDS/BBS3D 开发安装目录；这些目录目前存在且
依赖加载成功，但发布包的原生依赖尚未完全自包含，后续清理开发目录前需改正。
完整 `test_gateway_traffic.py` 另有 6 个既有遥控测试因旧测试输入缺少当前要求的
客户端时间戳而在 `input_expired` 提前失败，与本轮队列行为分开记录。

## 需要修正的发现

### R1 · P1：IMU 中断后，旧位姿仍获得新的时间戳

- 位置：`src/localization/slam/cpp/fastlio.cpp:805,1090,1385,2114`；
  `src/localization/slam/cpp/cyclone_runtime.cpp:2376,2380,2393`。
- 触发：正常 Tracking 后 IMU 停止，雷达继续送帧，无法组成新的同步观测。
- 原因：`feedLidar()` 更新公共 `last_stamp_s_`；同步失败时
  `updateWaitingReason()` 因已有旧位姿/点云而恢复 `tracking`，保留置信度；
  DDS 发布端只比较公共时间戳，重新发布旧 odometry/state，并给 TF 新时间戳。
- 本地原生复现：输出时间 `0.345 → 1.6` 秒，观测序号仍为 `1`，
  点云时间仍为 `0.345`，状态 `TRACKING`、置信度 `1`、旧 odometry 仍可用。
  探针使用实际 FastLIO 后端及已有三平面合成输入，没有启用 odom prior bypass。
- 后果：定位健康与结果新鲜度失真。不能据此断言整条导航必然放行运动，
  其他门控仍存在；但下游不应收到被重新标新时间的旧估计。
- 修法：分开“最近收到的传感器时间”和“最近成功估计时间”；只有接受新估计才推进
  odometry/state 时间。超过已有 IMU 同步容限后报告明确降级原因，保持旧结果年龄。
  保留正常短暂等待下一份 IMU 的行为，不把每次同步等待都当成故障。

### R2 · P1：保存快照仍阻塞 SLAM 的传感器处理线程

- 位置：`src/localization/slam/cpp/cyclone_runtime.cpp:2051`；
  `src/localization/slam/cpp/fastlio.cpp:1236,1272,1281,1317,1345`。
- 触发：地图保存请求进入 slamd，尤其地图和关键帧较多时。
- 原因：请求处理直接调用 `saveMap()`，在同一主循环里写 PCD、读回 PCD、
  复制所有 patch、可能重投影整图、写轨迹和 patch 文件，再加载重定位地图。
  此时不能继续 drain IMU/LiDAR 或正常 tick；mapd 的保存任务异步化没有移走这一段。
- 容量后果：IMU 的 DDS 历史只有 256 条，后端 4000 条缓冲无法接住尚未从 DDS 取出的
  数据。若输入频率为 f Hz，DDS 缓冲覆盖约 `256/f` 秒；例如 200 Hz 时为 1.28 秒。
  这只是容量换算，尚未测本轮 NX 保存阻塞时长，不能把此前整个保存任务耗时算在这里。
- 修法：主线程取得一致的不可变保存快照，耗时落盘/重建交给有界保存任务；完成后由
  主线程更新运行状态。继续保留 mapd 的保存事务与原子发布。评估建图保存时是否需要
  立即加载重定位索引，避免无用工作。不能直接让后台线程并发访问活动 builder。

### R3 · P1：在线建图一次丢帧，会使后续校正地图保存无法靠重试恢复

- 位置：`src/localization/opt/online_mapping.hpp:18`；
  `src/localization/opt/online_mapping.cpp:158`；
  `src/localization/slam/cpp/fastlio.cpp:1243,2166,2177`。
- 触发：后台优化未完成时待处理帧超过 4，或累计接收达到 3000；原始 patch 历史
  超过 3000 也会删掉最早数据。待处理容量和累计容量是两种不同限制。
- 本地原生复现：连续投递 5 帧，第 5 帧被 `online_mapping_backpressure` 拒绝；
  排空后 `busy=0`，`dropped_frames=1` 仍保留。该探针证明丢弃策略，不代表 NX
  实际已发生这次过载。正常关键帧有最小 1 秒间隔，后台持续繁忙才会形成这类积压。
- 后果：一旦已经有成功全局优化，保存检查会返回
  `online_mapping_incomplete_cannot_save_corrected_map`；等待后台结束再保存也无效。
  没有成功优化时不走这条拒绝条件，不能笼统说任何丢帧都会拒绝所有保存。
- 容量边界：当前 1 秒最小间隔加运动门槛，持续满足条件时约 50 分钟可到 3000 帧；
  停留或缓慢运动会更久。这不是按面积限制，也不是固定 50 分钟就失败。
- 修法：保留拒绝不完整校正地图的检查，补齐丢帧后的恢复路径。原始 patch 完整时
  可重建在线优化输入或走已有离线优化；长期采集应增量保存完整 patch，内存仅缓存
  工作集。在真正达到上限前明确提示可保存范围。不能仅增大队列或清零丢帧计数。

### R4 · P2：网页“两帧队列”之前还有不受该容量约束的回调积压

- 位置：`src/gateway/services/sse.py:154`；`src/gateway/services/cloud_ws.py:38`。
- 触发：生产线程持续发布，而 asyncio 事件循环暂时繁忙。
- 原因：每条消息先通过 `call_soon_threadsafe()` 登记一个捕获完整 payload 的回调，
  到回调执行时才进入容量为 2/128 的队列并丢旧消息。
- 本地 Python 复现：暂停消费期间发布 256 个不同的 32 KiB 点云包，应用队列显示
  深度 0、丢帧 0，但外部已有 256 个回调持有共 8 MiB 数据。恢复后才显示队列 2、
  丢帧 254。这是隔离突发输入，不是现场速率或进程 RSS 测量。
- 后果：容量/丢帧指标漏掉真正积压的位置；繁忙时额外持有内存，恢复时还要逐个执行
  已经过时的回调，与遥控 WebSocket 共用事件循环处理时间。
- 修法：在线程投递入口先合并到有界待处理存储，同一订阅至多安排一个唤醒回调；
  点云只留最新帧，并统计待处理字节和源年龄。事件消息使用不同的保留规则。

### R5 · P2：SSE 状态与一次性事件混排，状态洪峰会挤掉事件

- 位置：`src/gateway/services/traffic.py:65,100`。
- 触发：订阅者暂时慢于生产端。128 条容量内仅 `joint_state` 合并同类数据，
  其他消息全部共用 FIFO 和 drop-oldest。
- 本地复现：一个 `command_ack` 后投递 129 个 odometry，最终剩下 128 个旧/新位姿，
  ACK 已不在队列中。同样的淘汰规则适用于任务事件和对话通知。
- 现有缓解：`web/src/hooks/useSSE.ts:410` 已检测序号缺口并刷新权威状态，
  可以纠正状态，不等于重放每个瞬时通知。遥控 WS 的原生 ACK 是另一条链路，
  本发现不证明速度指令或该 ACK 丢失。
- 修法：状态按类型只保留最新值；需要按次消费的事件单独有界排队，溢出后依据已有
  request/task 身份刷新对应结果。复用现有缺口刷新机制，不引入通用消息代理。
  合并状态时需相应定义序号语义，避免正常合并触发无意义的持续 HTTP 刷新。

### R6 · P2：每次读取 SLAM 输出都复制已保存整图

- 位置：`src/localization/slam/cpp/fastlio.cpp:1382`；
  `src/localization/slam/cpp/cyclone_runtime.cpp:2281,2451`。
- 触发：保存过地图后继续 tick，或者调用只需要状态的查询。
- 原因：`SlamOutputs` 中的局部点云和已保存点云是按值的 `optional<Cloud>`，
  `outputs()` 每次复制 vector；DDS 的“是否新帧”判断发生在这之后。
  周期文件快照也反复写未改变的 saved map；global map 已有 revision 判断可复用。
- 原生复现：连续两次 outputs 的 saved/registered 点数组地址不同；
  本地 `sizeof(PointXYZIT)=32`。按 20 万点、默认 50 Hz 推算，仅 saved map
  一项就有约 320 MB/s 的逻辑 payload 复制量；这是算术估算，不是 NX 带宽实测。
- 修法：沿用现有 global map 的不可变共享快照及版本号，状态查询不复制整图，
  文件/网络只在对应版本更新时发布。保持快照生命周期，不能把共享可变 vector
  直接暴露给并发读者。

### R7 · P2：在线预览限制了输出点数，却仍每批重建全部历史点

- 位置：`src/localization/opt/online_mapping.cpp:94,114,126,131`。
- 触发：较长建图会话，每次处理新关键帧后生成 preview snapshot。
- 原因：先遍历所有历史帧，将全部点变换后加入临时 vector，再采样到 20 万点。
  输出上限没有限制本次中间内存和运算量。
- 容量：每个优化子图最多 5000 点、最多 3000 帧；上界是遍历 1500 万点，
  仅 XYZI 临时元素约 240 MB，尚未计 vector 扩容、采样中间结果、历史和原始 patch。
  这是支持参数的上界推算，未在 NX 制造该峰值；不能用它声称当前房间图占了这些内存。
- 修法：无全局校正时增量更新预览体素；有校正时按新 revision 在后台分批重建，
  直接汇入有界预览结构，避免先拼完整中间点数组。原始建图资料仍完整保留。
  这项修复也降低 R3 中后台长期忙碌的概率。

### R8 · P2：HTTP 点云轮询没有应用层超时和取消

- 位置：`web/src/hooks/useBinaryCloud.ts:227,236,241,305`。
- 触发：HTTP 点云请求连上但响应体迟迟不结束，或网络中断后旧请求长时间挂起。
- 原因：`fetch()` 和 `arrayBuffer()` 没有 AbortSignal/截止时间；下一轮只在 finally
  里安排。停止 fallback 只递增 generation、清计时器，并不取消已发请求。
- 后果：当前轮询会话可能长期不再发起新请求；旧响应虽被 generation 拒绝，仍占连接
  和读取资源。源码确认，尚未做真实网线故障实验；不是说浏览器永远不会自行超时。
- 修法：每轮一个可取消请求，给整个响应体读取设期限；停止、切换和卸载时 abort。
  保留一轮在途，超时后按已有节奏重试，避免堆叠请求。

## 前后容量与消息语义

以下为当前源码默认/Go2 未覆盖的配置，不代表已经读取当前 NX 安装值。
队列容量必须连同处理时间、数据年龄和字节数看，不能把各层条数简单相加。

| 链路 | 前层 → 后层 | 审查判断 |
| --- | --- | --- |
| IMU | DDS 256 → 单轮最多取 16×16 → 后端 4000 | 需按顺序覆盖扫描时间；前层丢失无法被后层容量补偿 |
| 原始雷达 | DDS 2、350 ms → drain latest → 后端 64 | DDS 丢旧策略适合防延迟；后端不能长期回放旧扫描 |
| mapd 实时观测 | DDS → 一个 pending，可覆盖 | 有界更新已存在；不能把原始持久化关键帧也按此丢弃 |
| 在线关键帧 | pending 4 → 累计 3000；原始 patch 默认 3000 | 现从 patch 历史补投队列；达到上限时暂停并可保存 |
| 地图/预览 | MapCloud 1、500 ms；MapScene 1、2 s | 小队列不代表小消息，MapScene 接收上限仍为 32 MiB |
| 遥控速度 | 浏览器最多一条待确认 → DDS 1、350 ms → 原生仲裁 | 已有最新值和有效期，不应改成可靠长 FIFO |
| 控制事务/ACK | OperatorMotionControl 32；ACK 64 | 与连续速度样本分开是对的，需按 request/epoch 处理 |
| 网页实时点云 | 有界跨线程待处理 → 队列 2 → Worker 一个执行中＋一个最新待处理 | 回调积压现已受前层容量约束 |
| 网页 SSE | 有界跨线程待处理 → 128 条状态/事件 | 状态合并，逐次事件优先，独立序号检测缺口 |
| 网页累计图 | 有期限的 HTTP 单请求 → PCLD 二进制 → Worker | 保留现有紧凑格式，超时或停止会取消请求 |

出处：`src/transport/dds/qos.hpp`、`fastlio.cpp:60`、
`src/maps/cpp/mapd/engine.cpp:216`、`web/src/services/teleopWsClient.ts:198`、
`src/gateway/services/traffic.py:11`、`web/src/hooks/useBinaryCloud.ts:205`。

## 消息格式和工具的取舍

**保留 native CycloneDDS 作为机器人进程间数据面，保留网页 PCLD + Worker。**
当前证据支持先减少排队、重复拷贝和同步工作；没有换消息中间件的收益证据。

| 数据 | 建议 |
| --- | --- |
| 原始 LiDAR/IMU | 保留 typed DDS 和逐点时间/line/tag 等算法所需字段；不能为省字节破坏去畸变 |
| 浏览器点云 | 现有 PCLD/float32/ArrayBuffer 转移继续使用；累计图默认已有二进制，无需再造一种格式 |
| 遥控与小事务 | 保留小 JSON WebSocket 消息及现有原生 typed command；先处理调度和期限 |
| 同进程大点云 | 不可变共享快照＋revision，避免再次序列化；R6 最直接 |
| DDS 接收 | SLAM 的 `drainReader()` 每次预分配/释放 16 个样本，空读也发生；参考 nav client 已有 reader loan，减少分配/拷贝 |
| 高频健康状态 | 当前每 tick 将健康字段封装为 Text JSON；可把门控必需字段放入 typed IDL，详细诊断降频。收益需测量后再扩大改动 |

本地 Windows DDS SDK 为 11.0.1；NX 候选构建使用现有原生 CycloneDDS 安装，
插件能力和性能本轮未核对。
reader loan 可以避免应用层预分配样本；**它不等于整条链路零拷贝**。
CycloneDDS 官方说明 writer loan 在固定大小、无内部指针的类型和配置 PSMX 等条件下
才可能避免拷贝/序列化。启用共享内存也需要匹配消息大小和内存池，不能直接将当前
可变长点云视为零拷贝。依据：[Loaning](https://cyclonedds.io/docs/cyclonedds/latest/api/loan.html)、
[Shared memory configuration](https://cyclonedds.io/docs/cyclonedds/latest/shared_memory/shared_mem_config.html)。

另一个应测量的调度点：`src/nav/cpp/client/client.cpp:2515` 的 ACK 接收线程同时
调用 `takeMapScenes()`，后者进行大场景解码与复制（上限见 `client_c.h:45`）。
可以考虑将场景处理与 ACK 接收分开；尚无本轮 NX 延迟分布，不能把它定为之前
350 ms 超时的已证实原因。此项先测大场景下 ACK 延迟，再决定线程调整。

## 验收证据与后续现场验证

1. R1：IMU 停止/恢复的原生回归，检查估计时间、观测序号、健康状态和下游过期门控。
2. R2、R3：保存期间持续传感器输入，验证处理不断档；制造优化积压后仍能完整保存，
   不以删除完整性检查换取成功。
3. R4、R5、R8：慢事件循环、慢订阅者、挂起响应测试；验证有界字节/回调数、状态最新、
   事件结果可查、HTTP 超时后恢复，并保留独立遥控确认。
4. R6、R7 和 DDS reader loan：先消除已证实的重复工作，再在 NX 比较峰值内存、
   tick 时长及 ACK 延迟，避免只报告平均吞吐量。

修复前的隔离证据均在 `build/go2-message-review-20260923/`：
`probe_queues.py`、`queue-results.json`、`probe_native.cpp`、`native-results.log`、
`native-build.log`。原生探针复用现有 Windows 构建库和 FastLIO 合成输入，输出
展示的是修复前缺陷；修复后以新增回归为准。`queue-results-fixed.json` 是修复后的
同场景队列探针。已有上述 ARM 编译和三项程序运行证据；尚未取得本轮 MuJoCo 或
现场运动新证据。NX 上还需量测
保存快照冻结阶段的 tick 延迟、长会话峰值内存和大场景下的 ACK 延迟。
