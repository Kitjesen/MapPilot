# 安全与控制边界

**Status:** Current
**Audience:** Developers and field operators
**Runs on:** S100P and simulation host

LingTu 的外部接口提交目标、操作员速度或控制动作；原生 C++ `navd`
拥有规划、跟踪、最终命令安全、控制权、软件急停和 `/nav/cmd_vel`
输出。Host 不运行第二套安全环或速度仲裁器。

> 本页描述仓库的软件控制边界，不是安全认证、现场风险评估或实体运动授权。

## 当前控制链

```text
REST / MCP / Web / CLI
  -> Gateway lease and command gate
  -> Host navigation commands/goals/skills
  -> typed native command
  -> navd
       -> input freshness and readiness
       -> planning / assisted teleop
       -> obstacle, traversability, geofence, and speed checks
       -> control authority and software E-stop
       -> final /nav/cmd_vel
  -> lingtu-driver
  -> robot
```

只有 `lingtu-driver` 可以把最终命令转发给机器人。Gateway 回执表示请求被
接受或拒绝，不表示机器人已经运动、到达或停止。

## 事实来源

| 问题 | 查看 |
| --- | --- |
| Gateway 命令检查与拒绝 | `src/gateway/services/control_commands.py` |
| stop、estop、reset、resume 的原生命令 | `src/gateway/services/native_control.py` |
| REST 控制路由 | `src/gateway/routes/commands.py` |
| Teleop 租约、断连和归零 | `src/gateway/routes/realtime.py` |
| 原生最终控制和安全 | `src/nav/cpp/endpoint/nav/control/`、`nav/safety/`、`nav/input/` |
| 原生端点总合同 | `src/nav/cpp/endpoint/README.md` |

## Gateway 会检查什么

所有运动请求先检查控制租约和活动 Safety STOP。导航目标还检查当前
readiness、odometry 和活动地图身份；原生端点拒绝命令时，Gateway 返回
拒绝，不把本地发布当作成功。

当前 Host 不提供独立的路径安全执行器，原生端点也尚未向 Gateway 暴露
无运动 plan preview。因此 `/api/v1/navigation/plan` 返回
`preview_unavailable` 时，不能把它解释为路径可行。

## 原生端点会检查什么

`navd` 在最终命令输出前统一处理：

- odometry、TF、点云、localization、traversability 和 driver-control 的
  顺序、时间新鲜度与恢复状态；
- 活动地图、目标和全局/局部路径状态；
- 障碍物、traversability、geofence、速度和加速度限制；
- 控制模式、操作员来源租约、接管状态和恢复保持；
- 软件 E-stop 与停止/关机归零。

`autonomy`、`teleop` 和 `teleop_avoid` 使用同一个最终控制与安全路径。
算法之间在 `navd` 内直接调用，不通过另一套 Python 速度链。

## Stop、cancel、reset 与 resume

| 操作 | 当前语义 |
| --- | --- |
| `POST /api/v1/stop` | 请求原生软件 E-stop，并要求原生命令确认。 |
| `POST /api/v1/navigation/cancel` | 取消当前或指定导航任务；不等同于 E-stop。 |
| `POST /api/v1/navigation/tasks/{task_id}/pause` | 请求指定任务进入已确认停止的暂停状态。 |
| `POST /api/v1/navigation/tasks/{task_id}/resume` | 继续同一暂停任务；仍受租约和原生安全状态约束。 |
| `POST /api/v1/estop/reset` | 清除软件 E-stop 闭锁；不会自动恢复旧运动。 |
| `POST /api/v1/navigation/resume` | 解除原生运动保持；不会重放旧目标或旧速度。 |

清除 E-stop 或解除保持后，根据原生回执提交新的目标或新的操作员命令。
进程重启不会清除持久的软件 E-stop。

## Teleop

`/ws/teleop` 获取短期控制租约并通过原生 operator-motion 边界提交样本。
按键释放、显式 stop、失焦、页面隐藏和断连都必须进入归零/释放流程。
客户端不得直接发布 `/nav/cmd_vel`，也不得在原生端点旁增加第二个 mux。

`teleop_avoid` 还要求新鲜 localization、障碍物和 traversability 输入，并在
原生端点内运行 local planner。普通 `teleop` 不因此变成自主导航模式。

## 操作员规则

- 运动前确认 Product、环境、地图、localization、readiness、控制来源和
  可触达的 stop 控制。
- Safety STOP、定位丢失、输入陈旧、地图不匹配、原生命令拒绝或控制来源
  意外变化时，保持停止并检查最窄的失败边界。
- 不通过提高发送频率、改走直接速度、启动旧进程或重启全栈来绕过拒绝。
- 本地测试、仿真和进程存活不能代替 S100P 的无运动或受监督运动证据。

运维入口见 [`scripts/lingtu`](../../scripts/lingtu)，当前 REST 模型和返回字段
见 [Gateway REST API](../api/gateway_rest.md)。
