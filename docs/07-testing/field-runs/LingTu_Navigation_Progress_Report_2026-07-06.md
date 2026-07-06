# LingTu 导航闭环进展报告与下一步 Plan

报告日期：2026-07-06

验收对象：sunrise 实机侧 native DDS 导航链路

验收边界：本轮只验证到 `/nav/cmd_vel`。不包含 thunder endpoint、brainstem、CAN、电机出口和真实运动控制。

## 1. 总体结论

| 结论项 | 当前判断 | 说明 |
|---|---|---|
| 是否完成完整自主导航 | 没有完成 | 还没有证明 live obstacle 开启时能持续输出非零速度并到达 goal |
| 是否证明主链路没有断 | 已证明 | 地图、重定位、全局规划、局部路径、PathFollower、`/nav/cmd_vel` 发布都跑到过 |
| 当前最大问题 | `near_field_stop` | 生产默认 live terrain / registered cloud / traversability 输入会把速度压成 0 |
| 是否应该先修真实运动控制 | 不应该 | 当前问题还在 `/nav/cmd_vel` 之前，先修地形/近场安全判断 |
| 下一步核心任务 | 抓 near-field stop 触发点 | 需要知道到底是哪几个点、哪个 frame、哪个高度/risk 导致停住 |

一句话结论：LingTu 的 ROS-free native 导航主链已经推进到 `/nav/cmd_vel`，并且非零速度生成能力已经在 sunrise 上证明；但 live terrain / registered cloud / traversability 参与时仍会触发 `near_field_stop`，所以完整自主导航还没有完成。

## 2. 当前已经完成的工作

| 序号 | 已完成事项 | 实际做了什么 | 证据或结果 |
|---:|---|---|---|
| 1 | sunrise 服务预检 | 检查核心 native DDS 服务状态 | `livox`、`slam`、`traversability`、`nav-dds`、`lingtu` 均 active |
| 2 | 地图包生成 | 通过正式入口跑 `map start / map save / map end` | 生成 `map.pcd`、`octomap.ot`、`occupancy.npz`、`metadata.json` |
| 3 | 地图包给全局规划使用 | 检查 OctoPlanner3D 需要的 map artifact | `navigation_ready=true`，`octomap.ot` 和 metadata 存在 |
| 4 | active map 修复 | 修复 `active` symlink 可能自循环的问题 | 避免 `/maps/active -> active` |
| 5 | `nav start active` 修复 | 把 `active` 解析为真实地图名 | Gateway session 和 SLAM mode 都指向真实 map |
| 6 | seeded relocalize | 用初始位姿做 seeded relocalization | relocalize 成功，`map->odom` 有效 |
| 7 | Gateway/session 修复 | 修复 invalid JSON、非有限值、active map session 解析 | 避免 official entry 被 API 500 或 symlink 问题挡住 |
| 8 | native nav 规划链路 | 直接给 native DDS nav 发较远 goal | 出现 4 点 global path 和 66 点 local path |
| 9 | `/nav/cmd_vel` 发布 | 观察 native endpoint status counter | `/nav/cmd_vel` 发布计数增长 |
| 10 | 非零速度 smoke | 临时不喂 live obstacle 输入 | 输出 `vx=0.0975`、`vy=0.168875`、`wz=0.785398` |
| 11 | 安全默认恢复 | smoke 后恢复生产默认 | `check_obstacle=true`，`use_traversability_cost=true` |

## 3. 当前没有完成、没有证明的工作

| 序号 | 未完成事项 | 当前真实状态 | 影响 |
|---:|---|---|---|
| 1 | live obstacle 开启时持续非零速度 | 未证明 | 生产默认下仍会被 `near_field_stop` 挡住 |
| 2 | 完整 nav 到达 goal | 未证明 | 还没有跑通“持续跟踪直到到达”的闭环 |
| 3 | 真实运动控制 | 未验 | thunder endpoint、brainstem、CAN、电机出口不在本轮范围内 |
| 4 | tracking profile | 未正式验收 | 不能标记为完成 |
| 5 | teleop profile | 未正式验收 | 不能标记为完成 |
| 6 | teleop_avoid profile | 未正式验收 | 不能标记为完成 |
| 7 | inspection profile | 未正式验收 | 巡检任务是否能稳定下发 nav goal 尚未证明 |
| 8 | tare_explore profile | 未正式验收 | 探索输出是否能交给 nav 执行尚未证明 |
| 9 | SLAM bad seed / no seed / 失败恢复 | 未全套验收 | 只证明 good seed 主路线成功 |

## 4. 链路分段状态

| 链路段 | 输入 | 输出 | 当前状态 |
|---|---|---|---|
| Livox DDS | Livox 硬件 | `/lidar/raw_frame`、`/imu/raw` | 在线 |
| SLAM DDS | 点云、IMU | `/slam/odometry`、`/slam/registered_cloud`、`/tf` | 在线，seeded relocalize 成功 |
| Map service | `map.pcd` | `octomap.ot`、`occupancy.npz`、`metadata.json` | 已证明可生成 |
| OctoPlanner3D | 当前位姿、goal、`octomap.ot` | `/nav/global_path` | 已证明可规划 |
| LocalPlanner | global path、odom、registered cloud、terrain、traversability | `/nav/local_path` | 可生成 local path |
| PathFollower | local path、当前状态 | 内部速度指令 | 诊断条件下可输出非零速度 |
| CmdVel Publisher | 内部速度指令 | `/nav/cmd_vel` | 已证明会发布 |
| Live terrain safety | registered cloud、terrain、traversability | near-field safety decision | 当前阻塞点，触发 `near_field_stop` |
| Hardware endpoint | `/nav/cmd_vel` | brainstem / CAN / 电机 | 本轮未验，不纳入当前结论 |

## 5. 关键证据矩阵

| 证据 | 结果 | 解释 |
|---|---|---|
| `map.pcd`、`octomap.ot`、`occupancy.npz`、`metadata.json` 均生成 | 通过 | 地图包生成链路可用 |
| `navigation_ready=true` | 通过 | 地图包具备导航使用条件 |
| seeded relocalize 成功 | 通过 | 初始位姿重定位主路线可用 |
| `map->odom` 有效 | 通过 | nav 可以在 map frame 下规划 |
| global path points = 4 | 通过 | OctoPlanner3D 输出有效路径 |
| local path points = 66 | 通过 | 局部规划路径生成不是断点 |
| `cmd_vel_published=102` | 通过 | `/nav/cmd_vel` 发布链路不是断点 |
| `cmd_vel={vx:0.0975, vy:0.168875, wz:0.785398}` | 通过 | PathFollower 非零速度能力已证明 |
| 生产默认下 `near_field_stop` | 未通过 | live terrain / obstacle 输入仍会让速度归零 |

## 6. 当前最大问题拆解

| 问题 | 当前观察 | 可能原因 | 下一步验证 |
|---|---|---|---|
| `near_field_stop` 触发 | 生产默认下速度为 0 | 真实障碍、frame 错、height 错、stale 数据、阈值过保守、融合策略问题 | 输出触发点明细 |
| registered cloud 进入局部规划 | 数据在线并被消费 | 点可能被转换到错误位置 | 记录 map/body 坐标和 TF |
| terrain map / terrain ext 参与规划 | 数据在线 | 高度或地形分类可能误判 | 记录 z、height、source |
| traversability 参与规划 | 数据在线 | hard cost 可能过硬或范围不对 | 记录 risk cost 和 grid index |
| local planner safety | 当前会硬停 | 阈值可能合理，也可能过保守 | 用 snapshot replay 做回归 |

## 7. 下一步 Plan

| 阶段 | 目标 | 要做的事 | 验收标准 |
|---|---|---|---|
| Phase A | 抓 `near_field_stop` 触发点 | 同步采集 registered cloud、terrain map、terrain ext、traversability、odom、TF、nav status | 能列出导致停住的具体点、坐标、高度、risk 和来源 |
| Phase B | 建立 offline replay | 把现场快照保存为 fixture，离线复现 local planner 的 `near_field_stop=true` | 同一份快照能稳定复现当前行为 |
| Phase C | 分类根因 | 判断是真实障碍、frame mismatch、高度字段错误、stale 数据、阈值问题还是融合策略问题 | 每个 stop 都能归因，不再只看到一个 reason 字符串 |
| Phase D | 修 live terrain 默认链路 | 按根因修 TF/height/stale/threshold/fusion，并保留安全边界 | `check_obstacle=true` 下空旷场景能输出非零 `/nav/cmd_vel` |
| Phase E | 正式 nav profile 验收 | 用 official `nav start active` 和 official goal 跑完整 `/nav/cmd_vel` 验收 | 不依赖 `LINGTU_NAV_CHECK_OBSTACLE=0` 也能通过 |
| Phase F | 扩展 profile 验收 | 依次验 tracking、teleop、teleop_avoid、inspection、tare_explore | 每个 profile 都用官方入口给出通过或失败证据 |

## 8. 近期执行清单

| 优先级 | 任务 | 产出物 | 是否阻塞后续 |
|---:|---|---|---|
| P0 | 在 nav endpoint 输出 near-field stop 触发点 | stop debug JSON / status 字段 | 是 |
| P0 | 采集一次现场 stop snapshot | cloud、terrain、traversability、odom、TF、status | 是 |
| P0 | 做 local planner offline replay | 回归测试 fixture | 是 |
| P1 | 修 frame / height / stale / threshold / fusion 中被确认的问题 | 代码修复 + regression test | 是 |
| P1 | 重新跑 production `check_obstacle=true` 的 nav goal | official nav 验收记录 | 是 |
| P2 | 跑 tracking profile acceptance | tracking 验收报告 | 否 |
| P2 | 跑 teleop / teleop_avoid acceptance | teleop 验收报告 | 否 |
| P2 | 跑 inspection / tare_explore acceptance | 上层 profile 验收报告 | 否 |

## 9. 风险控制

| 风险 | 为什么重要 | 控制方式 |
|---|---|---|
| 长期关闭 `LINGTU_NAV_CHECK_OBSTACLE` | 会绕过 live obstacle safety | 只允许 commissioning smoke 使用，生产默认必须为 1 |
| 直接调低 near-field 阈值 | 可能把误停变成漏停 | 先做 snapshot replay，再调参数 |
| 用 direct DDS injection 代替 profile 验收 | 只能证明子链路，不能证明产品入口 | profile acceptance 必须走 official entry |
| 过早修 hardware endpoint | 会把问题混到 `/nav/cmd_vel` 之后 | 当前先修 `/nav/cmd_vel` 之前的 near-field stop |
| 过早宣布 tracking/inspection/TARE 完成 | 它们还没有正式验收 | 逐 profile 建证据，不用代码存在代替验收 |

## 10. 证据文件

| 文件 | 作用 |
|---|---|
| `docs/07-testing/field-runs/2026-07-06-profile-sequential-validation.md` | 本轮现场顺序验收记录 |
| `.omx/plans/2026-07-06-profile-validation-and-migration-plan.md` | 后续 profile 验收计划 |
| `pytest_runtime_output_cmdvel_check2/before.json` | cmd_vel smoke 前状态 |
| `pytest_runtime_output_cmdvel_check2/during.json` | cmd_vel smoke 中状态，包含非零速度 |
| `pytest_runtime_output_cmdvel_check2/after_cancel.json` | cancel 后状态 |

## 11. 最终判断

| 判断 | 结论 |
|---|---|
| 成功是什么 | 已经把 ROS-free native 导航主链推进到 `/nav/cmd_vel`，并证明能产生非零速度 |
| 还差什么 | live terrain / registered cloud / traversability 参与时仍触发 `near_field_stop` |
| 下一步做什么 | 抓触发点、做 replay、修 terrain/near-field 解释，再跑 production 默认链路 |
| 不应该先做什么 | 不应该先修真实运动控制，也不应该先宣布 tracking/inspection/TARE 完成 |
