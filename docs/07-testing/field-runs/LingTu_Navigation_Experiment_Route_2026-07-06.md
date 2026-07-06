# LingTu 导航下一阶段实测技术路线

日期：2026-07-06

目标：把当前 `near_field_stop` 阻塞从“看到一个状态 reason”推进到“知道是哪类数据、哪个点、哪个阈值导致停住”，然后用可重复实验修正生产默认链路。

## 1. 总判断

| 问题 | 是否需要实测 | 原因 |
|---|---:|---|
| live terrain 下 `/nav/cmd_vel` 为 0 | 是 | 必须看真实 registered cloud、terrain、traversability 和 TF |
| `near_field_stop` 是否误停 | 是 | 只看代码无法判断现场前方是否真有障碍 |
| frame / TF 是否错 | 是 | 必须用同一时刻点云和位姿验证点落在机器人哪个位置 |
| 高度字段是否错 | 是 | 必须检查 registered cloud / terrain map 中 z、height、intensity 的实际含义 |
| traversability 是否过硬 | 是 | 必须看 grid risk 和 local planner 使用位置 |
| PathFollower 是否能输出速度 | 已证明，后续只回归 | 非零速度 smoke 已在 sunrise 证明 |
| thunder endpoint / brainstem / CAN | 暂不做 | 当前目标仍是 `/nav/cmd_vel` 之前 |

结论：接下来的核心问题都要通过现场数据实测和离线复现实验推进，不能只靠静态代码审查。

## 2. 实测技术路线总览

| 阶段 | 技术路线 | 目的 | 产出 |
|---|---|---|---|
| A | 现场同步抓取 nav 输入数据 | 还原 `near_field_stop` 发生时 local planner 看到了什么 | 一组 timestamp 对齐的 cloud / terrain / traversability / odom / TF / status |
| B | endpoint 增加 stop debug 输出 | 从状态里直接看到触发停住的点 | `near_field_stop_debug.json` 或 status 字段 |
| C | 离线 replay | 用现场快照复现 `near_field_stop=true` | regression fixture + replay 测试 |
| D | 根因分类实验 | 区分真实障碍、frame 错、高度错、stale、阈值、融合策略 | 分类结论 |
| E | 修复并回归 | 修改对应代码或参数，并用 replay 防回退 | 代码修复 + 测试 |
| F | sunrise production 验收 | `check_obstacle=true` 下跑 official nav goal | 非零 `/nav/cmd_vel` 证据 |
| G | profile 扩展验收 | tracking / teleop / inspection / TARE | 每个 profile 的通过/失败证据 |

## 3. Phase A：现场同步抓取

| 数据 | 来源 | 用途 | 必须记录 |
|---|---|---|---|
| registered cloud | `/slam/registered_cloud` | 判断 SLAM 输出给局部规划的近场点 | frame、stamp、x/y/z、height/intensity |
| terrain map | `/nav/terrain_map` | 判断地形分析结果 | frame、stamp、点数、height |
| terrain map ext | `/nav/terrain_map_ext` | 判断扩展地形/障碍结果 | frame、stamp、点数、height |
| traversability | `/nav/traversability` | 判断可通行性 hard cost | grid origin、resolution、risk value |
| odometry | `/slam/odometry` | 判断机器人当前位姿 | map/odom/body pose、yaw |
| TF | `/tf` | 判断 map->odom、odom->body 是否一致 | parent、child、translation、rotation |
| nav status | `/dev/shm/lingtu/nav_endpoint_status.json` | 判断 nav 当前状态 | plan、local reason、cmd_vel、counters |

通过方式：

1. 在 sunrise 上短时间开启采集脚本。
2. 发送一个会触发 `near_field_stop` 的 goal。
3. 抓取 stop 发生前后 2-5 秒数据。
4. 保存到一个带时间戳的实验目录。

建议目录：

```text
docs/07-testing/field-runs/artifacts/near_field_stop_YYYYMMDD_HHMMSS/
```

## 4. Phase B：stop debug 输出

当前只有：

```text
last_local.reason = near_field_stop
cmd_vel = 0
```

这不够。需要输出触发点明细。

| 字段 | 含义 |
|---|---|
| `source` | registered_cloud / terrain_map / terrain_map_ext / traversability |
| `stamp_s` | 数据时间戳 |
| `frame_id` | 原始 frame |
| `point_map` | map frame 下点坐标 |
| `point_body` | body frame 下相对坐标 |
| `forward_m` | 机器人前向距离 |
| `lateral_m` | 机器人横向距离 |
| `z_m` | z 高度 |
| `height_m` | local planner 使用的 height |
| `risk_cost` | traversability 风险值，如果有 |
| `threshold` | 触发对应检查的阈值 |
| `reason` | height obstacle / hard traversability / no data / stale 等 |

验收标准：

```text
near_field_stop=true 时，status 或 debug JSON 里必须能列出至少一个触发点。
```

## 5. Phase C：离线 replay

目的：不靠现场反复跑，也能复现问题。

| 输入 fixture | replay 使用方式 |
|---|---|
| odom pose | 设置 local planner 当前位姿 |
| map->odom TF | 把 cloud 转到规划 frame |
| registered cloud | 构造 obstacle input |
| terrain map/ext | 构造 planner terrain input |
| traversability grid | 构造 risk input |
| global path | 使用当时的 global_path 或合成安全 goal |

通过标准：

1. 原始 fixture 能复现 `near_field_stop=true`。
2. 临时移除 obstacle input 后能得到 `control_ready` 或至少不是同一个 stop。
3. 修改代码/参数后，fixture 行为变化必须可解释。

## 6. Phase D：根因分类实验

| 实验 | 方法 | 判定 |
|---|---|---|
| 真实障碍判断 | 对触发点做 body frame 可视化，确认是否在机器人前方近场范围内 | 如果点真实存在且高度合理，stop 正确 |
| frame 检查 | 比较 map frame 点、body frame 点、TF 链 | 如果点云整体偏移或旋转，修 TF/frame |
| height 检查 | 比较 z、height、intensity 字段 | 如果地面点 height 被当成障碍高度，修字段解释 |
| stale 检查 | 比较 cloud/terrain/traversability stamp 与 nav tick | 如果超龄仍被使用，修 max-age/cache |
| traversability 检查 | 查询 stop 区域 grid risk | 如果 hard cost 覆盖过大，修 cost 或 inflation |
| 阈值检查 | replay 中扫 nearFieldStopDis、obstacleHeightThre、vehicleWidth | 如果小幅调参即可恢复，先加测试再调 |
| 融合策略检查 | 分别只喂 registered cloud、terrain map、terrain ext、traversability | 定位是哪一路输入导致 stop |

## 7. Phase E：修复策略

| 根因 | 修复方向 | 必须加的回归 |
|---|---|---|
| frame mismatch | 修转换或 frame 判定 | 同一 fixture 点落在正确 body 坐标 |
| height 字段错误 | 修 `cloudToXyzh` 或 terrain height 解释 | 地面点不再触发 height obstacle |
| stale 数据 | 修 max-age、缓存清理、status 暴露 | 超龄数据不参与规划 |
| traversability 过硬 | 调 hard cost / inflation / no-data 策略 | 空旷区域不 hard stop，障碍区域仍 stop |
| 阈值过保守 | 调 near-field 参数 | 安全距离内障碍仍停，空旷 goal 不停 |
| 融合策略错误 | 调 terrain/traversability 优先级 | 单一路输入 replay 可解释 |

## 8. Phase F：sunrise production 验收

要求：不能使用 `LINGTU_NAV_CHECK_OBSTACLE=0`。

| 步骤 | 命令/方式 | 通过标准 |
|---|---|---|
| 1 | `nav start active --initial-pose ...` | relocalize 成功，`map->odom` 有效 |
| 2 | official Gateway/CLI goal | goal accepted |
| 3 | 观察 global path | global_path points > 1 |
| 4 | 观察 local path | local_path points > 1 |
| 5 | 观察 cmd_vel | `/nav/cmd_vel` 非零，reason = `control_ready` |
| 6 | cancel/stop | cmd_vel 清零，active_path false |

最终通过标准：

```text
check_obstacle=true
use_traversability_cost=true
official nav goal accepted
global_path_points > 1
local_path_points > 1
cmd_vel 非零
cancel 后 cmd_vel 清零
```

## 9. Phase G：profile 扩展验收

| 顺序 | Profile | 验什么 |
|---:|---|---|
| 1 | `tracking` | official tracking entry 是否能到 `/nav/cmd_vel` |
| 2 | `teleop` | 手动命令是否能到 mux/output boundary |
| 3 | `teleop_avoid` | obstacle/traversability 是否能影响 teleop |
| 4 | `inspection` | 巡检任务是否能发 official nav goal |
| 5 | `tare_explore` | 探索目标/路径是否能交给 nav |

原则：这些 profile 不能用“代码存在”代替验收，必须有 official entry 的实测证据。

## 10. 实验通过方式

| 验证方式 | 用在哪里 | 是否算 profile 完成 |
|---|---|---:|
| 代码静态检查 | 确认入口、topic、参数存在 | 否 |
| direct DDS injection | 定位子链路是否断 | 否 |
| commissioning smoke | 证明速度生成能力 | 否 |
| sunrise live data | 验证真实数据链路 | 部分 |
| offline replay | 固化 bug 和回归 | 部分 |
| official profile entry | 产品验收 | 是 |

最终验收必须回到 official profile entry，direct DDS injection 和 commissioning smoke 只能作为诊断证据。
