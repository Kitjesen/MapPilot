# 机器人远程控制与感知系统架构方案（产品级 v1.1）

## 文档信息

* **版本**：1.1（产品级草案）
* **日期**：2026-02-04
* **状态**：架构设计阶段（可进入接口/实现）
* **范围约束**：**先不考虑 5G 蜂窝公网**；覆盖 **局域网直连（机器人 AP / 同网段）** 与 **弱网（Wi-Fi 丢包/抖动）** 场景

---

## 1. 产品目标

### 1.1 核心能力

| 能力              | 描述                                       | 优先级 |
| --------------- | ---------------------------------------- | --- |
| **See（看到）**     | 低延迟视频、实时状态（姿态/电量/温度/模式）、SLAM 地图、点云可视化    | P0  |
| **Control（控制）** | 模式切换、任务指令、遥操作（速度/角速度）、急停、本地 Safety 最高优先级 | P0  |
| **Manage（管理）**  | 事件告警、地图/日志下载、OTA、用户权限、审计                 | P1  |

### 1.2 网络环境要求（不含 5G）

* **局域网直连**：机器人 AP 模式（IP 可配，示例 192.168.4.1），或同一 Wi-Fi/有线网段
* **弱网适应**：Wi-Fi 丢包、延迟、抖动、短暂断连（典型 1–10s）

**设计目标**：在弱网下仍保证 **“可控、安全、可观测”**，视频/点云可降级，但控制链路与安全机制不受影响。

### 1.3 AP 配置原则（实施前置）

* **IP 不固定**：统一通过配置文件指定 AP 网段与网关 IP（默认示例 192.168.4.1）
* **发现机制**：mDNS (`robot.local`) + 备用固定 IP（若 mDNS 不稳定）
* **安全**：WPA2/WPA3，强密码；仅开放必要端口
* **隔离**：控制/状态/视频分通道，必要时做带宽限额

---

## 2. 系统架构

### 2.1 系统边界与模块职责

```
┌────────────────────────────────────────────────────────────┐
│  App (iOS/Android/Pad)                                     │
│  ├─ gRPC Client: System / Control / Telemetry / Data        │
│  ├─ WebRTC Client: Video (Audio optional)                   │
│  └─ UI: 状态面板 / 遥操作 / 任务 / 地图点云 / 告警           │
└───────────────────────────────┬────────────────────────────┘
                                │ Wi-Fi / LAN
                                ▼
┌────────────────────────────────────────────────────────────┐
│  Robot                                                      │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ gRPC Gateway (单端口对外)                               │ │
│  │  - SystemService    : 会话/鉴权/心跳/能力查询            │ │
│  │  - ControlService   : 租约/模式/急停/任务/遥操作流        │ │
│  │  - TelemetryService : fast/slow/event 三条只读状态流      │ │
│  │  - DataService      : 点云/地图/文件资源订阅 & 视频控制    │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ WebRTC Media Server (视频)                               │ │
│  │  - 由 DataService 控制启停/参数                           │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ ROS2 / Control Stack                                     │ │
│  │  - SLAM / Planner / Localizer / Diagnostics / Drivers    │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Safety Gate (本地安全门控，强制优先级最高)                │ │
│  │  - deadman（遥操作失联停车）                              │ │
│  │  - 速度/姿态/温度/电量限制                                │ │
│  │  - 硬急停/软急停                                          │ │
│  └────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────┘
```

**关键产品原则**：

* **App 永远不直接访问 ROS2 DDS**；对外只暴露网关（减少攻击面、降低耦合）
* **控制面/状态面/数据面严格隔离**：大数据（点云/地图/视频）必须可降级，不能拖垮控制
* **安全门控在机器人本地闭环**：任何网络问题都不能突破 Safety Gate

---

## 3. 技术选型与端口规划（LAN 优先）

### 3.1 技术选型

| 组件         | 协议/技术                        | 说明                            |
| ---------- | ---------------------------- | ----------------------------- |
| 控制/状态/数据主干 | **gRPC (HTTP/2 + Protobuf)** | 强类型、易版本化、易做鉴权/审计、支持 streaming |
| 视频         | **WebRTC (SRTP)**            | 低延迟、弱网自适应、移动端成熟               |
| WebRTC 信令  | **gRPC**                     | 统一走网关（返回 SDP/ICE 信息）          |
| 发现         | **mDNS + 备用固定 IP**           | `robot.local` / `192.168.4.1` |
| 压缩         | LZ4 / Zstd                   | 点云/地图等二进制流压缩                  |

### 3.2 端口规划（建议最小暴露）

| 端口        | 用途                                      | 协议                        |
| --------- | --------------------------------------- | ------------------------- |
| **50051** | gRPC Gateway（所有 RPC/stream + WebRTC 信令） | TCP                       |
| （可选）8080  | 仅用于调试页面/健康检查（非必需）                       | HTTP                      |
| WebRTC 媒体 | 动态 UDP                                  | WebRTC 内部协商（LAN 不强制 TURN） |

> 说明：先不考虑 5G，LAN 场景下 WebRTC 不一定需要 TURN/STUN；若未来加公网再引入。

---

## 4. gRPC 服务设计（产品级契约）

### 4.1 服务总览（正确职责）

| 服务                   | RPC 类型                   | 用途                                     |
| -------------------- | ------------------------ | -------------------------------------- |
| **SystemService**    | Unary                    | 登录/鉴权、心跳、能力查询、版本信息                     |
| **ControlService**   | Unary + Streaming        | 租约、模式、急停、任务控制、遥操作（stream）              |
| **TelemetryService** | Server Streaming ×3      | fast_state / slow_state / event_stream |
| **DataService**      | Unary + Server Streaming | 资源枚举、点云/地图订阅、文件下载、视频会话控制               |

---

## 5. 会话、权限与控制权（产品必须项）

### 5.1 三级权限

| 角色           | 权限范围                    |
| ------------ | ----------------------- |
| **viewer**   | 只读：视频、状态、地图/点云（低档），告警查看 |
| **operator** | 遥操作、任务下发、软急停、资源订阅（中档）   |
| **admin**    | OTA、参数上限修改、日志下载、危险功能开关  |

### 5.2 Operator Lease（控制租约，避免抢控）

* **同一时间仅允许一个 operator 拥有 Lease**
* Lease 与会话绑定，有 **lease_token**
* Lease 用于授权“遥操作流 + 任务控制命令”的执行权限

建议机制（产品行为）：

* `AcquireLease()` 返回 `lease_token` 和 `lease_ttl_ms`
* 客户端 `RenewLease()` 维持（例如 2Hz）
* 机器人侧若超时 **释放 lease**，并通知 Telemetry（event）

### 5.3 安全门控 Safety Gate（本地强制）

**Safety Gate 不依赖网络**，强制执行：

* **deadman**：遥操作输入超时 **T_deadman = 200–300ms** → 速度归零 + safe_hold
* 速度/角速度限幅（按模式、地形、姿态动态限幅）
* 倾倒保护（IMU）
* 过热/低电禁止高功率动作
* **硬急停优先级最高**（物理按钮/安全继电器链路）

---

## 6. 遥操作与任务控制（关键：语义分离）

### 6.1 为什么要把“遥操作”和“任务命令”分开

* 遥操作：高频、最新值覆盖、可丢旧
* 任务命令：事务型、需要幂等、需要 ACK/错误码/审计

因此 **ControlService 必须分成两类 RPC**：

### 6.2 事务型控制（Unary RPC）

* `SetMode(mode)`
* `EmergencyStop()`（软急停：逻辑停；硬急停另链路）
* `StartTask(task_type, params, request_id)`
* `CancelTask(task_id, request_id)`
* `SetParam(key, value, request_id)`（高危参数仅 admin）

**幂等规则（产品级）**：

* 每个事务命令必须带 `request_id`
* 服务端维护短期去重缓存（例如 10min）
* 重试不会导致重复执行

### 6.3 遥操作控制（Streaming RPC）

* `StreamTeleop( stream TeleopCommand ) returns ( stream TeleopFeedback )`

**产品规则（必须写死）**：

* 只有持有 `lease_token` 的流才被接受
* 服务端执行“新覆盖旧”：若处理不过来，丢弃旧命令，只取最新一条
* `TeleopFeedback` 返回：实际执行值、限幅原因、当前安全状态、控制延迟估计

---

## 7. Telemetry（状态/事件）设计（强产品一致性）

TelemetryService 提供三个独立频道：

| 频道               | 内容                 | 频率      | 语义              |
| ---------------- | ------------------ | ------- | --------------- |
| **fast_state**   | 姿态、速度、IMU、Odom     | 20–60Hz | 最新值优先，客户端可丢旧    |
| **slow_state**   | 电量、温度、模式、连接质量、任务摘要 | 1–2Hz   | 低频稳定，避免大包       |
| **event_stream** | 告警/故障/提示           | 事件驱动    | **必须可回放 + ACK** |

### 7.1 event_stream 必须具备回放

* 每条 event 有 `event_id`（单调递增）
* 客户端连接携带 `last_event_id`
* 服务端补发缺失事件（环形缓冲 / 持久化任选）

### 7.2 ACK 语义要明确

* `AckEvent(event_id)` 表示“客户端已展示并确认/或已处理”（二选一，写死）
  建议：产品上把 ack 定义为“已在 UI 被确认”，用于协作与审计。

---

## 8. DataService（点云/地图/文件/视频控制）

### 8.1 资源模型（只放“真正的数据资源”）

DataService 负责枚举与订阅可选资源：

```
camera/front
camera/rear
map/global
map/local
pointcloud/lidar
pointcloud/fused
file/log_bundle
file/map_package
```

> 注意：**event 与 state 不在 DataService**，它们属于 TelemetryService 的标准管道，避免重复订阅体系。

### 8.2 Subscribe 模型（按需 + profile 协商）

* `ListResources()` → 返回资源列表与支持的 profile 档位
* `Subscribe(resource_id, profile)` → 返回 `subscription_id` 并开始 server-stream
* `Unsubscribe(subscription_id)`

### 8.3 点云/地图流必须有“预算约束”（产品硬规则）

订阅 profile 必须包含/或由服务端协商得出：

* `max_frequency`
* `roi`（例如前方 30m）
* `compress`（LZ4/Zstd）
* **max_points_per_sec**
* **max_bitrate_kbps**

若客户端请求超预算：

* 服务端自动降档并回传“实际生效 profile”
* 或拒绝订阅（返回明确错误码）

### 8.4 地图下载策略（产品体验更好）

* 实时局部图：可 streaming（小块增量）
* 大地图包/日志包：推荐 `DownloadFile(file_id)` 分块下载（或后续加 HTTP）

---

## 9. WebRTC 定位（保持产品边界清晰）

### 9.1 职责边界

* **WebRTC 只负责**：视频（主/副相机）、可选音频
* **不负责**：控制、鉴权、资源权限

### 9.2 通过 gRPC 控制 WebRTC 会话

DataService 提供：

* `StartCamera(camera_id, profile)` → 返回 WebRTC 信令信息（SDP/ICE）
* `StopCamera(session_id)`
* `SwitchProfile(session_id, profile)`（分辨率/帧率/码率）

### 9.3 视频降级策略（产品必须）

网络差时优先保证控制与状态：

1. 降分辨率 → 2) 降帧率 → 3) 降码率 → 4) 暂停视频
   整个过程由服务端根据链路质量自动调整，并在 slow_state 中反馈当前档位。

---

## 10. 弱网策略（符合 gRPC/WebRTC 真实语义）

### 10.1 gRPC Streaming（状态/遥操作）策略

* **fast_state**：客户端 UI 只渲染最新；处理不过来就丢旧
* **StreamTeleop**：服务端只取最新命令；超时触发 Safety Gate deadman
* **event_stream**：断线重连依靠 `last_event_id` 回放，不依赖“重传”概念

### 10.2 连接恢复（产品行为）

* App 断线 → 自动重连 gRPC → 恢复 Telemetry 订阅 → 尝试恢复视频（可选）
* lease 是否保留：由 lease_ttl 决定；若丢失，UI 必须明确提示“已失去控制权”

---

## 11. 实施计划（按产品里程碑）

### Phase 0（Week 0–1）：网络与基础设施 ✅ 大部分完成（2026-02-06）

* [ ] 机器人 AP 配置（SSID/密码/网段/IP 可配置）— 当前通过现有 Wi-Fi 网络可达
* [x] mDNS 配置（`robot.local` → 192.168.66.190）+ avahi gRPC 服务发现（_grpc._tcp）
* [x] 端口白名单与防火墙规则（iptables：SSH/gRPC/mDNS/WebRTC UDP/ICMP，其余 DROP）
* [x] 远程端联通性验证（grpcurl via 192.168.66.190:50051 验证通过）

### Phase 1（Week 1–2）：主干打通 ✅ 已完成

* [x] 定义 proto（System/Control/Telemetry/Data + common）
* [x] gRPC Gateway 跑通：fast_state、AcquireLease、StreamTeleop（空实现可）
* [x] Safety Gate：deadman + 限幅最小闭环
* [x] App：能连上、看 fast_state、发 teleop（基础 UI）
* [x] **[新增]** DataService 图像流（gRPC）跑通

### Phase 1.5（验证 & Bug 修复）✅ 已完成（2026-02-06）

* [x] grpcurl 接口逐一验证（4 个 Service、17 个测试项全部通过）
* [x] Bug 修复：Heartbeat RTT 计算负值 → 使用纳秒精度 + abs()
* [x] Bug 修复：SlowState 模式硬编码 → ModeProvider 回调从 ControlService 实时获取
* [x] 安装 libdatachannel 0.24.1（从源码编译，ARM64）
* [x] WebRTC Bridge 启用：SDP Answer + ICE Candidate 生成验证通过
* [x] WebRTCSignaling 双向流验证通过（Offer → Answer + ICE 完整握手）

### Phase 2（Week 3–4）：控制与任务产品化

* [x] 事务型 RPC：SetMode / StartTask / CancelTask + request_id 幂等（已实现）
* [x] Lease 续约/抢占规则（已实现，AcquireLease/RenewLease/ReleaseLease 验证通过）
* [x] event_stream（含回放与 ack）（EventBuffer 已实现，回放验证通过）
* [ ] 弱网断连恢复流程跑通（含 teleop 失联停车）
* [ ] 事件系统与 ROS2 `/diagnostics` 集成

### Phase 3（Week 5–6）：视频产品化

* [x] WebRTC 信令通道（gRPC 双向流）打通
* [x] libdatachannel 集成（PeerConnection + ICE + SDP）
* [ ] WebRTC 推流（实际视频帧发送，需 H.264 编码器集成）
* [ ] DataService：StartCamera/StopCamera/SwitchProfile（旧接口可用，需配置 webrtc_enabled）
* [ ] Videos 降级策略 + 反馈到 slow_state

### Phase 4（Week 7–8）：数据面与运维能力

* [ ] 点云订阅（profile、预算约束、压缩）— Subscribe 接口已实现，需有 publisher 测试
* [ ] 地图/日志下载（分块）— DownloadFile 接口已实现
* [ ] 权限（viewer/operator/admin）与审计日志（P1）

---

## 12. 下一步行动（可立即开工）

1. **生成 proto 规范（字段级）**：

* `common.proto`（Header、Timestamp、Role、ErrorCode、Profile、ResourceId）
* `system.proto`
* `control.proto`
* `telemetry.proto`
* `data.proto`

2. **确认实现语言与依赖**（不影响架构，但影响进度）：

* Gateway： C++（性能）
* WebRTC：GStreamer webrtcbin / Pion（Go）/ aiortc（Python）
* 压缩：LZ4（低 CPU）优先，Zstd（更高压缩比）作为可选

3. **确定 ROS2 topic 映射清单**：

* fast_state：/imu、/odom、/cmd_vel反馈、姿态估计
* slow_state：/battery、/temps、/mode、/diagnostics、/task_summary
* event：/diagnostics 规则化 → event_id

---

如果你同意这版 v1.1 的方向，我建议下一步直接进入“可写代码的契约层”：我可以把 **proto 文件结构与关键 message 字段**一次性写出来（包含 lease、幂等 request_id、event 回放、点云 profile 预算字段、StartCamera 返回的 signaling 信息等），你们照着实现网关与 App 就行。你只需要回复一句：**“继续，生成 proto（字段级）”**。

第一件事：冻结接口契约（proto 字段级）
把 common.proto / system.proto / control.proto / telemetry.proto / data.proto 五个文件写出来，并在 proto 里明确这些产品级硬约束：request_id 幂等、lease_token、event_id/last_event_id 回放、TeleopCommand 的时间戳/序号、点云订阅的 profile（含 max_points_per_sec、max_bitrate_kbps）。这一步做完，你们前后端/机器人端才能并行开发，避免“边写边改协议”拖垮进度。