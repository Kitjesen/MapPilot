# 巡检 MVP

第一版巡检只验证一条闭环：

```text
选路线点 → 导航到点 → 拍一张 overview 照片 → 保存照片和位姿 → 回执成功 → 下一个点
```

新建巡检路线点的默认动作是 `capture:overview`。它保存：

- 一张 RGB JPEG；
- 与照片时间匹配的机器人位姿；
- 当时已有的检测快照；
- `capture_only / not_evaluated` 结果。

MVP 不自动判断“正常”或“异常”，也不启动视觉模型、热成像或声学分析。这样先能验证真实机器人上的到点、停稳、拍照、落盘和回执，再增加分析动作。

底层仍保留同步和完整性要求：RGB 与里程计必须新鲜，时间偏差不能超过配置值；证据目录原子提交；原生 Executor 收到成功回执后才推进点位。这些是为了保证“照片属于这个点”，不是额外的业务分支。

需要检查异常时，再把单个路线点的 action 改成明确动作，例如 `capture:indicator_green`、`capture:door_closed` 或 `capture:thermal`。这些动作不属于 MVP 的默认流程。

当前 Agent 可以列出已保存的巡检路线，并通过 `start_inspection_route` 接入 Inspection Product 的 `start_task` 入口。工具返回的是任务受理回执，不把受理冒充为到达或巡检完成；后续状态、照片和证据仍从原生任务链路读取。MVP 首次验收仍建议先在无运动或仿真环境确认回执，再做实机验证。

对话需要继续跟踪时，Agent 保留回执里的 `task_id`，调用
`get_inspection_status` 读取同一原生任务，直到状态进入成功、失败或取消。
只有终态证据到达后，Agent 才能把本轮巡检标记为完成。
