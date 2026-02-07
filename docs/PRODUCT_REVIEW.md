# 产品架构评审 — 从产品经理视角

> 最后更新: 2026-02-08
> 状态: 评审完成 → 架构方案已提出

---

## 一、评审结论

**设计思路不失败，但产品化状态失败。**

系统拥有完整的架构蓝图（SLAM → 地形分析 → 规划 → 控制），但处于"功能列表很长、没有一个能真正交付"的状态。每一个功能都是 "代码写了，逻辑看着对，但从未在真机上走通"。

---

## 二、做对了什么

### 2.1 分层架构正确

```
感知 (LiDAR) → SLAM (Fast-LIO2) → 地形分析 → 规划 (全局+局部) → 控制 → 底盘
```

模块划分、坐标系定义 (map→odom→body)、话题设计都是业界标准做法，没有架构硬伤。

### 2.2 安全分层概念正确

7 层安全网：硬件遥控器 → CMS 仲裁 → Bridge Watchdog → SafetyGate → ModeManager → HealthMonitor → GeofenceMonitor。理念上是产品级的。

### 2.3 gRPC 桥接方案正确

把 ROS2 生态封装成 gRPC 服务暴露给 Flutter App，比 rosbridge/websocket 更适合产品化——有强类型 proto、双向流、连接管理。

---

## 三、产品级失败点

### 3.1 单点故障：grpc_gateway 巨型单进程

这是**最严重的架构问题**。`grpc_gateway` 一个进程承载了：

| 模块 | 类别 | 崩溃影响 |
|------|------|---------|
| SafetyGate | **安全关键** | 遥控失去保护 |
| ModeManager | **安全关键** | 模式仲裁失效 |
| HealthMonitor | 安全相关 | 无法检测异常 |
| GeofenceMonitor | 安全相关 | 围栏失效 |
| ControlService | 功能 | 遥控/任务不可用 |
| TelemetryService | 功能 | 状态流中断 |
| DataService (OTA/WebRTC/地图/相机) | 功能 | 数据服务全部中断 |
| TaskManager | 功能 | 任务调度中断 |
| LeaseManager | 功能 | 租约管理中断 |

**后果**: DataService 里一个 OTA 下载线程 OOM，整个安全网关就挂了。用户正在遥控行走时 WebRTC 编码吃满 CPU，SafetyGate 来不及处理——机器人撞墙。

**产品原则**: 安全功能和数据功能不应该在同一个进程里。

### 3.2 一英里宽，一英寸深

| 功能 | 代码存在 | 真机验证 | 产品可交付 |
|------|---------|---------|-----------|
| SLAM (Fast-LIO2) | ✅ | ✅ | ✅ |
| 地形分析 | ✅ | ✅ | ✅ |
| 局部规划 (local_planner) | ✅ | ✅ | ✅ |
| gRPC 连接 + 遥测 | ✅ | ✅ | ⚠️ 单点故障 |
| 遥控行走 | ✅ | ⚠️ 部分 | ❌ 避障刚加,未验证 |
| WebRTC 视频 | ✅ | ❌ 闪退 | ❌ 权限刚修,未验证 |
| OTA 更新 | ✅ | ❌ | ❌ 从未走通完整更新 |
| 全局规划 (PCT) | ✅ | ❌ | ❌ 闭环未验证 |
| 任务管理 | ✅ | ❌ | ❌ 到达检测未标定 |
| 断联降级 | ✅ | ❌ | ❌ 实际断网未测试 |
| 地图管理 | ⚠️ 文件操作 | ❌ | ❌ 无专用 API |
| WebRTC 重连 | ❌ | — | — |
| WebRTC 自适应码率 | ❌ | — | — |
| OTA Ed25519 签名 | ⚠️ 设计完成 | ❌ | ❌ |

**核心问题**: 项目一直在"加功能"，从来没有"做完一个功能"。

### 3.3 零测试 = 零信心

- 没有单元测试
- 没有集成测试
- 没有 CI/CD 跑测试
- 每次改代码都引入回退（参数重复声明、参数未声明等问题）

### 3.4 配置管理是定时炸弹

一个 YAML 文件，90+ 个参数，分散在 6+ 个 cpp 文件中 `declare_parameter`。没有 schema 校验、没有默认值文档、没有启动检查。

### 3.5 WebRTC DataChannel + JPEG 是技术债

| 指标 | 当前 (JPEG DC) | 产品标准 (H.264 MediaTrack) |
|------|---------------|---------------------------|
| 带宽 | ~3-5 Mbps @640x480x15fps | ~0.5-1 Mbps |
| CPU 占用 | 高 (软件编解码) | 低 (硬件编解码) |
| 端到端延迟 | ~200-500ms | ~50-150ms |
| 自适应 | 无 | 带宽自适应 |

---

## 四、产品最小可交付物 (MVP) 定义

一个真正能交付的机器人遥控产品，核心只需要四件事做到 99.9% 可靠：

> **遥控能走 + 不撞东西 + 视频流畅 + App 不崩**

不是 OTA，不是全局规划，不是任务调度——就这四件事。

---

## 五、架构重新设计方案

### 5.1 设计原则

| # | 原则 | 说明 |
|---|------|------|
| 1 | **安全隔离** | 安全关键模块独立进程，不与数据功能同生共死 |
| 2 | **故障域限定** | 任何非安全模块崩溃，机器人仍然安全 |
| 3 | **先深后宽** | 先把核心路径做到产品级，再扩展功能 |
| 4 | **可验证** | 每个模块有明确的验收标准和测试方法 |
| 5 | **渐进式** | 从当前代码演进，不推倒重来 |

### 5.2 进程拆分方案

```
┌─────────────────────────────────────────────────────────────────────┐
│                       PROCESS BOUNDARY MAP                          │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ 进程 1: safety_node (安全关键, 独立存活)                       │   │
│  │ ──────────────────────────────────────────────────────────── │   │
│  │  SafetyGate      — 速度限幅 + 倾斜保护 + 近场避障            │   │
│  │  ModeManager     — 模式状态机 + 仲裁                         │   │
│  │  HealthMonitor   — 订阅各模块心跳, 检测异常                   │   │
│  │  GeofenceMonitor — 围栏检测                                  │   │
│  │                                                              │   │
│  │  暴露:                                                       │   │
│  │    ROS2 话题: /cmd_vel, /stop, /speed, /safety_status       │   │
│  │    ROS2 服务: /set_mode, /emergency_stop, /clear_estop      │   │
│  │                                                              │   │
│  │  特性:                                                       │   │
│  │    - 独立进程, 不依赖 gRPC 或 WebRTC                         │   │
│  │    - 启动最快, 资源占用最小                                   │   │
│  │    - grpc_gateway 崩了, 它还在 → Deadman 超时 → 零速度      │   │
│  │    - 即使无 App 连接, 安全保护仍然生效                        │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ 进程 2: grpc_gateway (通信层, 可重启)                         │   │
│  │ ──────────────────────────────────────────────────────────── │   │
│  │  gRPC Server :50051                                          │   │
│  │    ├ SystemService  — Login, Heartbeat, Relocalize           │   │
│  │    ├ ControlService — StreamTeleop → /teleop_cmd (新话题)    │   │
│  │    │                  StartTask → /task_goal                 │   │
│  │    ├ TelemetryService — StreamFastState, StreamSlowState     │   │
│  │    └ DataService    — Subscribe, OTA, 文件管理               │   │
│  │  LeaseManager                                                │   │
│  │  EventBuffer                                                 │   │
│  │  TaskManager                                                 │   │
│  │                                                              │   │
│  │  特性:                                                       │   │
│  │    - 不直接发 /cmd_vel, 而是发 /teleop_cmd                   │   │
│  │    - safety_node 订阅 /teleop_cmd → 过滤 → 发 /cmd_vel     │   │
│  │    - 崩溃不影响安全, safety_node 的 Deadman 会兜底           │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ 进程 3: webrtc_node (可选, 资源密集型, 可独立重启)            │   │
│  │ ──────────────────────────────────────────────────────────── │   │
│  │  WebRTCBridge     — PeerConnection 管理                      │   │
│  │  JPEG/H.264 编码  — 摄像头订阅 + 编码 + DataChannel 发送    │   │
│  │                                                              │   │
│  │  特性:                                                       │   │
│  │    - CPU 密集型任务隔离, 不影响安全和控制                     │   │
│  │    - 崩溃时 gRPC gateway 仍可用 (fallback gRPC Subscribe)   │   │
│  │    - 未来可独立升级编码策略 (JPEG → H.264)                   │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.3 数据流变化 (关键改动)

#### 当前 (有缺陷):
```
App StreamTeleop → grpc_gateway 内部 SafetyGate → /cmd_vel → 底盘
                                     ↑ 同进程, 同生共死
```

#### 重新设计后:
```
App StreamTeleop → grpc_gateway → /teleop_cmd (ROS2 Topic)
                                       │
                                       ▼
                               safety_node (独立进程)
                                 ├ 模式检查 (TELEOP?)
                                 ├ 速度限幅
                                 ├ 倾斜保护
                                 ├ 近场避障 (/terrain_map)
                                 ├ 急停检查
                                 └──→ /cmd_vel → 底盘

如果 grpc_gateway 崩溃:
  /teleop_cmd 停止 → safety_node Deadman 超时 → /cmd_vel = 0 → 安全停车
  
如果 webrtc_node 崩溃:
  视频断了, 但遥控仍然可用, App 显示 "视频断开, 重连中..."
  
如果 safety_node 崩溃 (极端):
  /cmd_vel 停止 → han_dog_bridge watchdog 200ms 超时 → 底盘零速度
  + 硬件遥控器仍可接管
```

### 5.4 进程间通信方式

| 通信 | 方式 | 原因 |
|------|------|------|
| grpc_gateway ↔ safety_node | ROS2 Topic + Service | 解耦, safety_node 不依赖 gRPC |
| grpc_gateway ↔ webrtc_node | ROS2 Service (启停控制) + 共享内存 (帧传递) | 高效 |
| App ↔ grpc_gateway | gRPC :50051 | 不变 |
| safety_node → 底盘 | ROS2 Topic (/cmd_vel) | 不变 |

### 5.5 新增话题/服务

| 名称 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/teleop_cmd` | TwistStamped | grpc_gateway | safety_node | 未过滤的遥控指令 |
| `/safety_status` | 自定义消息 | safety_node | grpc_gateway | 安全状态 (供 TeleopFeedback) |
| `/set_mode` | Service | grpc_gateway→ | safety_node | 模式切换请求 |
| `/emergency_stop` | Service | grpc_gateway→ | safety_node | 急停请求 |
| `/webrtc/start` | Service | grpc_gateway→ | webrtc_node | 启动视频推流 |
| `/webrtc/status` | Topic | webrtc_node | grpc_gateway | 推流状态 |

### 5.6 Launch 文件设计

```python
# navigation_full.launch.py — 完整系统
def generate_launch_description():
    return LaunchDescription([
        # 阶段 1: 安全层 (最先启动, 最后关闭)
        Node(package='remote_monitoring', executable='safety_node',
             respawn=True, respawn_delay=1.0),  # 崩溃自动重启
        
        # 阶段 2: 通信层
        Node(package='remote_monitoring', executable='grpc_gateway',
             respawn=True, respawn_delay=3.0),
        
        # 阶段 3: 视频层 (可选)
        Node(package='remote_monitoring', executable='webrtc_node',
             respawn=True, respawn_delay=5.0,
             condition=IfCondition(LaunchConfiguration('enable_webrtc'))),
    ])
```

**关键**: `respawn=True` — 非安全进程崩溃自动重启，用户无感。

---

## 六、执行路线图

### Phase 0: 稳定当前系统 (1 周)

不做架构改动，只做验证和修复：

- [ ] 遥控 + 避障端到端冒烟测试（真机，录视频）
- [ ] WebRTC 视频连接测试（权限修复后）
- [ ] 修复所有启动崩溃问题（参数声明遗漏等）
- [ ] 验证 HealthMonitor 不会误触 ESTOP（当前 SLAM 未启动就会触发）

### Phase 1: 安全层拆分 (2 周)

**目标**: safety_node 独立运行，grpc_gateway 崩溃不影响安全。

- [ ] 新建 `safety_node` 可执行文件，包含 SafetyGate + ModeManager
- [ ] grpc_gateway 通过 `/teleop_cmd` 话题发送遥控指令（而非内部调用）
- [ ] safety_node 订阅 `/teleop_cmd` → 过滤 → 发布 `/cmd_vel`
- [ ] 模式切换改为 ROS2 Service 调用
- [ ] grpc_gateway 订阅 `/safety_status` 话题，透传给 App
- [ ] Launch 文件配置 `respawn=True`

### Phase 2: WebRTC 拆分 + 稳定 (1 周)

- [ ] webrtc_node 独立进程
- [ ] 加入重连机制（断开后 3s/6s/12s 指数退避重试）
- [ ] 加入 Offer/Answer 超时（3s 无回应 → failed）
- [ ] grpc_gateway 崩溃时 webrtc_node 继续推流（等新 gateway 起来）

### Phase 3: 测试框架 (1 周)

- [ ] SafetyGate 单元测试：避障分级、模式门禁、急停优先级
- [ ] ModeManager 单元测试：状态转换守卫、ESTOP 路径
- [ ] 集成测试：grpc_gateway 崩溃 → safety_node 停车
- [ ] CI 流水线：push 触发编译 + 测试

### Phase 4: 配置治理 (0.5 周)

- [ ] 参数 schema 定义（YAML 注释 + 类型 + 范围 + 默认值）
- [ ] 启动自检：节点启动后检查关键话题/TF/参数
- [ ] 参数校验：`declare_parameter` 统一在一个文件中管理

### 里程碑

| 时间 | 目标 | 验收标准 |
|------|------|---------|
| +1 周 | Phase 0 完成 | 遥控+避障+视频 端到端视频证明 |
| +3 周 | Phase 1 完成 | 杀掉 grpc_gateway 进程，机器人安全停车 |
| +4 周 | Phase 2 完成 | 视频断开后 5s 内自动恢复 |
| +5 周 | Phase 3 完成 | CI 绿灯，核心模块测试覆盖 > 70% |
| +5.5 周 | Phase 4 完成 | 节点启动时自检通过率 100% |

---

## 七、暂时冻结的功能

以下功能在 Phase 0-4 期间**不投入任何开发资源**：

| 功能 | 原因 |
|------|------|
| OTA Ed25519 签名链 | 核心功能稳定前不需要 |
| 全局规划 PCT_planner 集成 | 局部规划 + 遥控稳定后再做 |
| BehaviorTree 替代状态机 | 当前状态机够用 |
| 断联降级可配置化 | 固定策略即可 |
| WebRTC 自适应码率 | 先确保基础视频可用 |
| WebRTC H.264 编码 | JPEG 方案先顶住 |
| 多客户端并发遥控 | 单客户端先做好 |
| 巡检任务自动化 | 手动遥控先做好 |

---

## 八、成功的定义

### 最小可交付产品 (MVP) 验收清单

- [ ] 用户打开 App → 3s 内连接成功并看到视频画面
- [ ] 用户推摇杆 → 机器人即时响应 (< 100ms 体感延迟)
- [ ] 前方 0.8m 内有障碍物 → 机器人自动停止，App 显示 "obstacle_stop"
- [ ] 视频因网络波动断开 → 5s 内自动恢复，无需手动操作
- [ ] 强制杀掉 grpc_gateway → 机器人 300ms 内停车，无碰撞
- [ ] App 进后台 30s → 回来后自动恢复连接和视频
- [ ] 连续运行 1 小时无崩溃、无内存泄漏、无 CPU 飙升

**当且仅当以上 7 项全部通过**，才能说这个产品"能用"。

---

*文档作者: Product Review Session*
*下一步: 按 Phase 0 开始执行*
