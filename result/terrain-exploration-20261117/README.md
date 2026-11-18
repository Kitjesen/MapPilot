# 连续地形实验结果（2026-11-17）

这份目录冻结了本轮连续地形全局规划实验：场景定义、OctoMap 输入、逐请求原始返回值、几何复核、日志、路线文件、论文图和可复现脚本都在这里。

## 统计

- 场景：**36**
- 请求：**86**
- 返回路线：**72**
- 没有路线：**14**
- 其中预设且正确拒绝：**6**
- 未达到“应有路线”预期：**8**
- 按预期判定（通过路线或正确拒绝）：**78/86**

“成功”有两个口径：如果只问“是否生成路线”，答案是 72/86；如果把预设负例的正确拒绝也算作验收通过，则是 78/86。

## 第三个图的红色

你发的总览图第三格是 `ripple_2d`（双向起伏表面）。图中的**红色星号是目标点（goal）**，绿色圆点是起点（start）。这是绘图脚本固定的标记：目标使用红色星号，起点使用绿色圆点。红橙色实线是该图中第一条返回路线；同一格里的其他请求使用蓝色、紫色或绿色线，并用虚实线区分方向。红色不是“碰撞”或“失败”标记。

## 失败清单

### 预设负例（6 组，结果正确）

- `wave_gap`：全宽 0.5 m 断口，正反各 1 组拒绝。
- `wave_ceiling`：局部顶棚净空不足，正反各 1 组拒绝。
- `wave_upper`：上层表面没有连接，正反各 1 组拒绝。

### 当前未达到预期的 8 组

- `cross_55`：名义站姿正反各 1 组无路线；同场景高 5 cm / 10 cm 的站姿请求通过。
- `cross_65`：名义站姿正反各 1 组无路线。
- `wave_35_120`：深短波正反各 1 组无路线。
- `wave_50_200`：大起伏正反各 1 组无路线。

这 8 组的原生公开诊断完全相同：`rust_failure_kind=query`、`rust_failure=Rust surface route exhausted`。当前日志没有把起点连接、终点连接、支撑、边几何、机体碰撞和搜索预算拆开，因此不能把它们分别归因。详细记录在 `data/failure_reasons.csv`，下一步应先补分阶段 reason，再决定是算法问题还是场景本来不可执行。

## 文件位置

- 全部逐请求原始结果：`data/results.json`
- 场景与冻结地形：`data/scenes.json`
- 逐请求采样分析：`data/analysis.json`
- 逐请求索引及结果路径：`data/manifest.csv`
- 轨迹质量诊断：`data/trajectory_quality.csv`
- 横坡边界复测：`data/slope-boundary.json`、`data/slope-boundary-fine.json`
- 新版分阶段失败诊断复测：`data/diagnostic-rerun-20261117.json`
- 表面 shortcut 问题请求复测：`data/after-fix-issue-rerun-20261117.json`
- 86 组 A/B 全量复测：`data/full-rerun-20261117.json`
- 变化路线的几何复核：`data/after-fix-geometry-20261117.json`
- MuJoCo 动态验收摘要：`data/mujoco-motion-summary.json`
- 正式 navd 重建后的 MuJoCo 摘要：`data/mujoco-motion-final-summary.json`
- 失败分类：`data/failure_reasons.csv`
- 每个请求的原始规划／几何结果：`runs/<scene>/<request>/plan.json`、`geometry.json`、`route.xyz`
- OctoMap 与箱体输入：`maps/`
- 所有场景图：`figures/index.html`、`figures/png/`、`figures/pdf/`、`figures/svg/`
- 问题路线与失败场景总览：`figures/issue_atlas.html`、`figures/png/issue_atlas.png`
- 生成与绘图源码快照：`reproduce/`
- 运行日志：`logs/`

## 复现

在仓库根目录 PowerShell 执行：

```powershell
.\result\terrain-exploration-20261117\reproduce\run_reproduce.ps1
```

只重跑一个场景：

```powershell
.\result\terrain-exploration-20261117\reproduce\run_reproduce.ps1 -Scene wave_35_120
```

横坡边界复测：

```powershell
uv run python result/terrain-exploration-20261117/reproduce/slope_boundary.py
```

脚本调用仓库正式的 `build/terrain-exploration-20261117/explore.py` 和现有 Release 原生规划器；不会使用图中虚构路线。它会更新 `build/terrain-exploration-20261117/` 与 `docs/media/terrain-exploration-20261117/` 的结果，再重新生成本目录前应重新复制冻结数据。

## 结果边界

这批数据验证的是正式全局规划入口和离线几何复核，不是 MuJoCo 动力学，也不是 NX 实机运动。横坡通过不等于机器狗已经验证 roll/pitch 稳定；窄脊通过不等于四足落点已经验证。

## 轨迹是否都合理

几何上，72 条 `rust_pass` 路线都满足当前机体、支撑、步高、坡度、禁行区和扫掠检查，并且到达请求目标；这说明它们在当前模型内可通行。轨迹形状上并非全部适合直接交给执行器：质量诊断发现 8 条路线需要复核。

- `crest_30` 正反各 1 条：19 个沿主方向回退的小段，最大转折 180°，绕行比例约 1.42。
- `valley_30` 正反各 1 条：17 个回退小段，最大转折 180°，绕行比例约 1.38。
- `wave_40_400` 正反各 1 条：6 个回退小段，最大转折 180°。
- `chicane_ramp` 正反各 1 条：路线几何有效，但绕行比例约 1.33～1.38，转折较多。

这些路线不是“撞障碍”的路线；它们暴露的是离散表面图在坡面上的路径抖动和绕行质量问题。现有 `smoothRoute` 已经会做直连检查，但连续坡面上直线段的高度不能直接匹配离散表面，导致部分 5 cm 网格级回退无法被安全合并。质量诊断使用回退段、转折角和绕行比例做复核标记，不会把这些阈值当成新的通行约束。

## 横坡边界不是 45°

本轮新增了 46°、48°、50°、52°、53°、54°、55°、56°横坡边界复测，每个角度测正反方向的名义站姿和抬高 5 cm 站姿：

- 46°～52°：名义站姿正反方向均有路线；
- 53°～55°：名义站姿无路线，抬高 5 cm 后正反方向有路线；
- 56°：名义站姿和抬高 5 cm 都无路线。

这说明“超过 45°都走不了”不成立。这里测的是**横坡**，路线沿等高线行走，路线自身的前进坡度接近零；它和沿行进方向的纵向坡度不是同一个量。深波浪 `wave_35_120` 和 `wave_50_200` 的沿程最大坡度约 60.9°／57.3°，而当前路线坡度上限约为 30°，因此它们属于另一类失败。

53°～55°只有在提高输入站姿后才通过，说明固定 yaw 的双圆柱身体包络和横坡两侧的高度差正在主导结果；这不是把所有机器人都抬高 5 cm 就可以解决的问题。56°连抬高后也失败，仍需补充分阶段失败诊断和姿态／足端物理验证。

新版 native planner 已把失败阶段接到 JSON：本次 14 条无路线中，6 条预设负例落在 `edge_search`，`cross_55`／`cross_65` 落在 `goal_attach`，两组深波浪分别落在 `goal_attach` 或 `edge_search`。这说明横坡失败不是最终 `route_geometry` 拒绝，而是在目标连接候选阶段就没有找到可执行终点；深波浪则有一部分在终点连接阶段被挡住，另一部分进入边搜索后耗尽。诊断结果仍不能证明物理姿态安全，但已经把“全部都是一个空路径错误”拆开了。

本轮 surface-aware shortcut 的 A/B 结果：86 组的状态保持 **72 条路线、14 条无路线**；9 条路线的点列发生变化，变化路线逐条通过原有 `verify_path` 的 `valid`、`route_valid` 和 `sweep_valid` 检查。`wave_40_400` 的路线点数和绕行比例有所下降；`crest_30`、`valley_30` 仍保留回退段，因此这部分只能算接入并验证了一轮，尚未完全解决坡面离散抖动。

## MuJoCo 动态结果

MuJoCo 预检和资产生成通过：同源场景生成了 534,921 个点、41,845 个占据体素，mapd、navd、driver bridge 和传感器 publisher 都启动并正常清理。但 45 s motion 阶段没有进入真实导航运动：navd 的全局规划反复在 `edge_search` 失败，`global_path_points=0`、`local_path_points=0`、非零 `cmd_vel=0`，机器狗净位移只有约 5.7 cm，目标距离没有减少。

同一份 MuJoCo OctoMap 的隔离对照说明原因不是坡度和最终控制：

- `max_slope=1.0` 仍无路线；
- 关闭 `ground_support` 后有 148 点路线；
- `robot_radius=0.20～0.28 m` 有路线；
- `robot_radius=0.30～0.32 m` 无路线。

因此当前场景的阻塞是 0.32 m 身体包络与这份地图的可支撑／可通行区域组合，不应通过把正式机器人尺寸改小来掩盖。正确修法是先检查场景地图在 L 走廊、门口和抬高平台的表面支撑连续性，再让 MuJoCo 的场景地图与正式 0.32 m 身体模型一致；修复后必须重新跑同一 motion 验收，确认 global path、SCAN local path、非零 cmd_vel 和实际到达一起出现。

正式 `navd.exe` 已重新构建并用 ABI 11 重新生成 `.nav/profile-0.bin`；最终 motion 日志确认这次不是旧 prepared cache 问题，刷新缓存后仍然在正式 A* 阶段失败。因此下一步应该修 MuJoCo 场景地图／支撑连续性或为该场景提供与生产身体包络一致的可通行地图，而不是继续改终端控制或速度参数。
