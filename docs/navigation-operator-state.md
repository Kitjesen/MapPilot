# 导航操作者状态模型

本文定义导航状态在操作者界面和客户端 API 中的含义。它不是 SCAN、Follower、
规划器或驱动内部状态机的说明，也不把 Product、探索和巡检任务抽象成同一套任务框架。

## 操作者需要回答的四个问题

导航状态由四个相互独立的轴组成：

| 问题 | 字段 | 操作者心智 |
| --- | --- | --- |
| 当前任务做到哪一步？ | `task` | 规划、执行、恢复、显式暂停，或已经结束 |
| 现在能否下发新目标？ | `goal_admission` | Gateway 能否接收或替换目标，以及阻塞原因 |
| 谁拥有控制权、运动是否被保持？ | `control`、`motion.permission` | 自主、人工或无人持有控制权；输出是否被保持或急停 |
| 机器人是否在动，要求停止时是否已确认停稳？ | `motion.observation`、`motion.stop_confirmation` | 新鲜里程计观察与停止屏障证据 |

不能把这四个问题压成一个“运行中/暂停”状态。任务可以仍在 `EXECUTING`，同时因
地图输入过期而处于 `HELD`，并且停止屏障已经给出 `CONFIRMED`。

## 任务生命周期

```text
IDLE → PLANNING → EXECUTING ↔ RECOVERING
                         ↘ PAUSED
                         ↘ SUCCESS / FAILED / CANCELLED
```

- `PAUSED` 只来自该任务的显式暂停事件。
- 急停、人工接管和 InputGate 阻塞不会把任务阶段改成 `PAUSED`。
- `RECOVERING` 是同一任务上的实时恢复叠加，不是新任务。
- `SUCCESS`、`FAILED`、`CANCELLED` 在同一 navd boot 内保留到新任务出现。
- navd 重启后，实时主视图回到 `IDLE`；既有任务历史继续从任务账本查询。
- 当前任务只使用 task/request 身份匹配的 `NavigationGoalStatus`。不得用其他任务的
  “全局最后一条事件”填补当前任务。

## 公共 API

`GET /api/v1/navigation/status` 和 SSE `navigation_status` 在原有字段之外返回兼容字段
`operator_state`。原有 `state`、`readiness`、`control`、`motion`、
`navigation_state` 以及 DDS `NavigationState` 保持不变。

```yaml
operator_state:
  schema_version: 1
  task:
    state: IDLE|PLANNING|EXECUTING|RECOVERING|PAUSED|SUCCESS|FAILED|CANCELLED|UNKNOWN
    task_id: string
    request_id: string
    terminal: boolean
    progress: number|null
    reason: string
  goal_admission:
    state: ACCEPTING|BLOCKED|UNKNOWN
    blockers: [string]
    advisories: [string]
  control:
    authority: AUTONOMY|OPERATOR|NONE|UNKNOWN
    resume_required: boolean
    reason: string
  motion:
    permission: CLEAR|HELD|ESTOPPED|UNKNOWN
    observation: MOVING|QUIET|UNKNOWN
    stop_confirmation: NOT_REQUESTED|PENDING|CONFIRMED|FAILED|UNKNOWN
    linear_speed_mps: number|null
    angular_speed_radps: number|null
    reason: string
  summary:
    severity: OK|INFO|WARNING|CRITICAL
    code: string
    next_action: string
```

API 只返回稳定英文代码。Web 根据这些代码提供中英文文案，不把后端原因字符串直接当作
面向操作者的提示。

## 关键语义

### 命令接受不等于开始运动

`goal_admission=ACCEPTING` 只说明 Gateway 可以接收或替换目标。命令回执中的
`accepted=true` 只证明命令已被接收，不证明已完成规划、已获得运动许可或机器人已经
开始移动。SDK 应优先使用命令回执的 `task_id` 查询
`/api/v1/navigation/tasks/{task_id}`，避免被其他任务的终态误导。

### 安静不等于已确认停稳

`motion.observation=QUIET` 只表示新鲜里程计样本没有观察到超过阈值的运动。
`motion.stop_confirmation=CONFIRMED` 还要求停止屏障获得 Driver ACK 和规定数量的静止
样本。只发布零速度命令，或者暂时看见零速度，都不足以显示“已确认停稳”。

任何后续非零最终速度输出都会使旧的停止确认失效；连续零保持不会清除已经确认的结果。
状态源缺失或过期时，准入、控制、运动观察或停止确认应返回 `UNKNOWN`，不得乐观推断
“可运动”或“已停稳”。

## 状态来源与所有权

| 输出 | 主要证据 |
| --- | --- |
| `task` | 与当前 task/request 精确关联的 `NavigationGoalStatus`，加同任务恢复叠加 |
| `goal_admission` | 实时 `NavigationState.can_accept_goal`、readiness 和阻塞原因 |
| `control` | 控制权、人工接管和恢复要求 |
| `motion.permission` | 急停、InputGate 和输出保持状态 |
| `motion.observation` | 新鲜里程计及 navd 公布的线速度、角速度阈值 |
| `motion.stop_confirmation` | navd `motion_stop_evidence` 中的停止屏障结果 |

Gateway 负责把这些证据投影成一个一致快照；Web 和 SDK 消费该快照，不各自重建业务
状态机。内部 epoch、route generation、trajectory ID 和 SCAN FSM 仍用于诊断，但不是
操作者完成日常导航所需的心智模型。

## 证据边界

本地单元与契约测试只能证明字段映射、身份关联和界面表达。`CONFIRMED` 的现场结论仍需在
S100P 上完成无运动验证，再进行监督下的低速运动验证。进程存活、仿真通过和现场可安全
运动是不同级别的证据。
