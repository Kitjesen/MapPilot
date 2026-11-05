# 地图残影清理（prune）

`prune` 是**保存地图时使用的 C++ 工具**。例如，建图时有人走过走廊，地图留下了他的点云；
人离开后，后续扫描确认那里已经空出来，工具才把残影分离出来。
墙面、地面不能仅因“点少、只扫过一次”就被删除。

## 它在哪里起作用

```text
保存建图记录 → 全局优化（条件满足时）→ prune 清理 → 生成导航地图 → 地图库
```

网页的“保存地图”通过原生地图流程调用它；mapd 默认要求清理成功。
保存工具只处理持久地图，它不处理浏览器正在显示的累计点云。
mapd 与原生 `online_mapping` 预览维持原有地图聚合；增量清理仅保留为离线实验，尚未接入实时链路。

调用位置：[pipeline.cpp](../cpp/build/pipeline.cpp) 的 `RunSavedSourceCleanerJson`；
默认配置：[mapd/main.cpp](../cpp/mapd/main.cpp)。保存时使用 `--overwrite --apply`。

## 怎样清理，同时保护地面和墙面

当前实现 `prune.visibility_surface_v4` 按扫描时间顺序处理：

1. **以前确实看到过它。** 回波要落在旧地图点附近；没有观测依据的区域保留。
2. **后来明确看到那里空了。** 射线经过旧点附近，并在更远处收到回波，才记一次空闲证据。
   没收到回波、没扫到、被前面的物体遮住，都不算空闲。
3. **默认至少 3 帧确认。** 一帧中很多条射线仍只算一票；同帧在旧点附近有回波，
   优先保留并清零空闲票数。新出现的物体不会因更早的空闲记录被删除。
4. **再检查当前仍有支撑的表面。** 地面、墙面和坡面统一使用多帧局部平面保护；在全部扫描处理完成后检查支撑是否仍然成立。
   其余满足多帧空闲条件的旧点才进入删除结果。

默认点到射线距离容差 **5 cm**，回波端点周围 **20 cm** 内不清除，
只检查从雷达起点起 **20 m** 内的射线段。20 cm 体素用于加速查找，
删除仍按实际距离判断。旧点和周围 6 个 2 cm 偏移探针都要被本帧实测射线覆盖；
同帧命中会阻断后方空闲证据，不会把整条粗体素通道清空。

墙面保护依靠实际回波、遮挡和局部平面一致性，稀疏点本身不是删除理由。
平面保护需要 50 cm 内至少 6 个有两帧回波、没有后续空闲票的邻点，排除近似一条线的分布；
至少 80% 邻点与候选点距同一参考平面不超过 4 cm，才保留候选点。这只增加保留，不增加删除。
Patchwork++ 地面分割实验归 `research/maps/ground`，保存清理由自身的可见性和表面几何处理。
同源 903room 的 185 个关键帧对照中，启用与停用地面标签的保留和删除 PCD 完全相同；
新鲜的标签保护本身也是上述局部平面保护的子集，因此移除重复路径及其旧标签缓存。
这不证明所有场景下地面分割都没有价值，也不代表这批真实点云有人工真值。
地面保护由几何和多帧证据决定，低矮物体不会因高度参数获得永久保护。
机身倾斜、坡道、楼梯和配准误差需要实图验收，不能声称完全不会误删。
长期站着的人、未被再次观察的位置，也可能留下残影。

旧版依据“帧数/命中点数不足”删除的规则已取消。保存清理不再计算粗略的 XY 分格运动评分。

### 玻璃的已知局限

当前 prune 没有透明材料分类。稀疏玻璃点或整片玻璃被后续射线穿过时，
可能累积空闲证据并失去有效邻面保护，最终进入删除结果。
局部平面规则保护的前提是周围仍有符合要求的实测支撑。
即使玻璃点在清理后点云中保留，后续 OctoMap 射线重放仍可能把对应格累计更新为 free。
实际原生函数的合成对照和现场归因边界记录在
[玻璃保存机制与复现](../../../docs/research/glass-map-persistence-20261105.md)。

## 需要什么输入

```text
地图目录/
  map.pcd           整张点云，地图坐标系
  poses.txt         按采集顺序排列的扫描位姿
  patches/*.pcd     对应扫描，当前原生 SLAM 导出的是机身坐标系
  scan_origin.txt   雷达原点在扫描坐标系中的位置
```

`poses.txt` 每行：`扫描文件名 tx ty tz qw qx qy qz`，将对应扫描变换到地图坐标系。
`scan_origin.txt` 内容：`lidar_origin_in_patch x y z`，单位为米。
新 SLAM 从机身、IMU、雷达外参计算这个值；优化和入库时保留它，不能默认使用机身原点。

旧记录缺少原点文件时，需提供经过核对的 `--sensor-origin X Y Z`，不能随意填零。
只有整图、没有扫描或轨迹时无法使用。缺少位姿的扫描会跳过并计入报告。
支持 ASCII 和未压缩 binary PCD，不支持 `binary_compressed`。

## 怎么使用

先在地图副本上分析：

```bash
prune --map-dir /path/to/map-copy --dry-run
```

只输出 JSON，不创建目录或写点云；不能与 `--apply` 同时使用。
需要查看具体删除哪些点时运行：

```bash
prune --map-dir /path/to/map-copy
```

| 文件 | 内容 |
|---|---|
| `map.pcd` | 原图，保持不变 |
| `map.clean.pcd` | 保留的点 |
| `map.removed.pcd` | 删除候选，用于检查是否误删墙面、地面 |

检查后应用到副本：

```bash
prune --map-dir /path/to/map-copy --apply --overwrite
```

首次应用创建 `map.before_clean.pcd`，重复应用不覆盖最初的备份。已有旧名
`map.pcd.preclean` 时继续复用旧备份，避免把已清理的点云当作原图再备份。
空结果不能替换原图，截断的 binary PCD 会报错。正式保存流程将备份以新名一起入库，
保留最近一次的 `map.removed.pcd`，不再额外保存独立工具的 `map.clean.pcd` 中间文件。
首次备份与最近一次移除点的时刻不同，不保证前者等于当前点云加当前移除点。
完整归属见[地图文件与失败保存任务](../../../docs/operations/map-files.md)。
命令行直接应用不会同步重建已有的 OctoMap、通行图等产物，正式重建要走地图保存流程。
网页的清理前后对比、选区审查、恢复操作尚未接入。

## 报告和参数

| 报告字段 | 含义 |
|---|---|
| `dry_run` / `applied` | 只分析 / 已替换原图 |
| `source_points`、`kept_points`、`removed_points` | 原图、保留、删除点数 |
| `removed_fraction` | 删除比例，0–1 |
| `decision_counts.kept_uncontradicted` | 未满足多帧空闲矛盾条件的保留点 |
| `decision_counts.kept_supported_surface` | 已有空闲矛盾，但被当前有效表面支撑保护的点 |
| `decision_counts.removed_free_space_contradiction` | 满足空闲矛盾且没有有效表面保护的删除点 |
| `free_space_candidate_points` | 多帧空闲候选数，部分可能被当前连续表面规则保留 |
| `patch_count` / `unmatched_patch_count` | 已匹配 / 缺少位姿的扫描数 |
| `backup_pcd` | 应用时的备份路径 |
| `removal_semantics` | `multi_frame_observed_free_space_not_semantic_motion`：空闲证据，不是语义运动识别 |

常见错误：`missing_sensor_origin` 缺少雷达原点，`empty_clean_map` 全删结果被拒绝，
`no_matched_patch_evidence` 没有可用扫描，`output_exists` 需要另选输出位置或使用 `--overwrite`。

| 参数 | 默认值 | 用途 |
|---|---|---|
| `--sensor-origin X Y Z` | 读取原点文件 | 指定经过标定的雷达原点 |
| `--min-free-frames` | 3，至少 2 | 删除前需要多少帧后续空闲证据 |
| `--voxel-size` | 0.20 m | 查找和统计的格子尺寸，不是导航膨胀半径 |
| `--out-clean` / `--out-removed` | 地图目录内 | 对比点云输出位置 |

完整参数见 `prune --help`。

### 报告精简后的迁移

`--ground-z-threshold`、`--min-frame-support`、`--min-hit-support`、`--instance-grid-m`、
`--moving-score-threshold`、`--min-instance-points` 已删除。旧命令需要移除这些选项，
继续传入会报未知参数并退出。它们原来只影响报告或已经无效，不改变当前可见性与表面保护的删点规则。
此处调整针对默认 `prune`，外部研究算法的参数含义由各自项目定义。

JSON 不再输出 `preset`、`flow`、`evidence_voxels`、`dynamic_candidate_voxels`、
`scored_instances`、`moving_instances`、`score_candidate_points`、`max_moving_score`。
旧的 `kept_ground_geometry` 恒零字段与 `kept_multi_frame`、`kept_hit_support`、
`kept_without_evidence`、`kept_unconfirmed` 支持度归因一并删除。
读取保留原因的调用方改用上表的 `kept_uncontradicted` 和 `kept_supported_surface`，
算法版本继续读取 `algorithm`。历史报告保留原来的字段。

`source_points = kept_points + removed_points`，
`kept_points = kept_uncontradicted + kept_supported_surface`，
`free_space_candidate_points = removed_points + kept_supported_surface`。

## 实现与验证范围

参考 [OctoMap 的射线更新原则](https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html)：
每帧只更新一次，实际回波优先于空闲射线，避免同帧互相清除表面。
[ERASOR2](https://github.com/url-kaist/ERASOR2) 的地面和实例保护用于理解后续改进方向。
当前实现不包含或链接 ERASOR2 GPLv3 代码。默认 prune 的配置与链接不依赖 Patchwork++ 或 Eigen3；
可见性和局部平面实现使用 C++20，地图公共契约、mapd 和整图预览保留 C++17。
实际表面保护计入 `kept_supported_surface`。
完整 UFOMap/FreeDOM 服务没有链接进来；保存期复用了保守空闲证据与当前命中优先，历史增量重分类仅供离线实验。
`third_party/research_nav/ERASOR2` 保留为外部研究参考，不接入产品构建和保存流程。

```bash
cmake -S src/maps/prune/cpp -B build/prune -DBUILD_TESTING=ON
cmake --build build/prune --config Release
ctest --test-dir build/prune -C Release --output-on-failure
```

独立地面分割、增量清理、同源回放和外部算法对照统一查看
[地图研究入口](../../../research/maps/README.md)。

入口：[cpp/prune.cpp](cpp/prune.cpp)；判定：[cpp/core/visibility.cpp](cpp/core/visibility.cpp)；
测试：[prune_test.cpp](../../../tests/maps/cpp/prune_test.cpp)。覆盖多帧残影、遮挡、稀疏静态点、
同帧回波优先、地面/墙面/坡面保护、贴近墙面的独立残影、非零雷达原点和保存恢复。
本地测试不代表任何目标实机平台的性能或实图清理质量已经验收。

离线同源回放输出保存图、增量图、逐帧耗时和容量计数。
真实记录没有地面/墙柱/楼梯语义真值时，删除点数不能当作准确率。
实验增量结果是有容量上限的代表点集合，不能替代整图存储或作为任意地图点的删除掩码。
清理证据不授权导航自由空间。实机性能与静态结构保留仍需目标平台验收。
