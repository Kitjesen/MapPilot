# 地图残影清理（prune）

`prune` 是**保存地图时使用的 C++ 工具**。例如，建图时有人走过走廊，地图留下了他的点云；
人离开后，后续扫描确认那里已经空出来，工具才把残影分离出来。
墙面、地面不能仅因“点少、只扫过一次”就被删除。

## 它在哪里起作用

```text
保存建图记录 → 全局优化（条件满足时）→ prune 清理 → 生成导航地图 → 地图库
```

网页的“保存地图”通过原生地图流程调用它；mapd 默认要求清理成功。
它不处理浏览器正在显示的累计点云，也不处理实时导航障碍。
因此，这次修改不会自动消除正在建图的页面中的残影。

调用位置：[pipeline.cpp](../cpp/build/pipeline.cpp) 的 `RunSavedSourceCleanerJson`；
默认配置：[mapd/main.cpp](../cpp/mapd/main.cpp)。保存时使用 `--overwrite --apply`。

## 怎样清理，同时保护地面和墙面

当前实现 `prune.visibility_v2` 按扫描时间顺序处理：

1. **以前确实看到过它。** 回波要落在旧地图点附近；没有观测依据的区域保留。
2. **后来明确看到那里空了。** 射线经过旧点附近，并在更远处收到回波，才记一次空闲证据。
   没收到回波、没扫到、被前面的物体遮住，都不算空闲。
3. **默认至少 3 帧确认。** 一帧中很多条射线仍只算一票；同帧在旧点附近有回波，
   优先保留并清零空闲票数。新出现的物体不会因更早的空闲记录被删除。
4. **再检查地面和连续表面。** 命中过低高度回波的格子保留；与附近稳定观测点构成连续平面的点也保留。
   其余满足多帧空闲条件的旧点才进入删除结果。

默认点到射线距离容差 **5 cm**，回波端点周围 **20 cm** 内不清除，
只检查从雷达起点起 **20 m** 内的射线段。20 cm 体素用于加速查找，
删除仍按实际距离判断，不会把整条粗体素通道清空。

墙面保护依靠实际回波、遮挡和局部平面一致性，稀疏点本身不是删除理由。
平面保护需要 50 cm 内至少 6 个有两帧回波、没有后续空闲票的邻点，排除近似一条线的分布；
至少 80% 邻点与候选点距同一参考平面不超过 4 cm，才保留候选点。这只增加保留，不增加删除。
地面还有扫描局部 Z ≤ −0.45 m 的额外保护；**这仍是简单高度规则，不是成熟的地形分类器**。
机身倾斜、坡道、楼梯和配准误差需要实图验收，不能声称完全不会误删。
长期站着的人、未被再次观察的位置，也可能留下残影。

旧版依据“帧数/命中点数不足”删除的规则已取消。`moving_instances` 仍是粗略的分格候选统计，
不是识别人或车辆，也不参与删除决定。

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

首次应用保留 `map.pcd.preclean`，重复应用不覆盖最初的备份。空结果不能替换原图，
截断的 binary PCD 会报错。正式保存流程将备份一起入库。
命令行直接应用不会同步重建已有的 OctoMap、通行图等产物，正式重建要走地图保存流程。
网页的清理前后对比、选区审查、恢复操作尚未接入。

## 报告和参数

| 报告字段 | 含义 |
|---|---|
| `dry_run` / `applied` | 只分析 / 已替换原图 |
| `source_points`、`kept_points`、`removed_points` | 原图、保留、删除点数 |
| `removed_fraction` | 删除比例，0–1 |
| `decision_counts` | 每个点最终的保留或删除原因 |
| `free_space_candidate_points` | 多帧空闲候选数，部分可能被地面或连续表面规则保留 |
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
| `--ground-z-threshold` | −0.45 m | 扫描局部坐标下的低高度保护 |
| `--out-clean` / `--out-removed` | 地图目录内 | 对比点云输出位置 |

`--min-frame-support`、`--min-hit-support`、`--instance-grid-m`、`--moving-score-threshold`、
`--min-instance-points` 只影响支持度归因和分格报告，不改变多帧射线的删除条件。
完整参数见 `prune --help`。

## 实现与验证范围

参考 [OctoMap 的射线更新原则](https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html)：
每帧只更新一次，实际回波优先于空闲射线，避免同帧互相清除表面。
[ERASOR2](https://github.com/url-kaist/ERASOR2) 的地面和实例保护用于理解后续改进方向。
当前实现没有完整复现这两个系统；不包含或链接 ERASOR2 GPLv3 代码，也没有新增依赖。
`third_party/research_nav/ERASOR2` 仅为可选参考，边界见 [refs/erasor2](cpp/refs/erasor2/README.md)。

```bash
cmake -S src/maps/prune/cpp -B build/prune -DBUILD_TESTING=ON -DLINGTU_PRUNE_ERASOR2=OFF
cmake --build build/prune --config Release
ctest --test-dir build/prune -C Release --output-on-failure
```

入口：[cpp/prune.cpp](cpp/prune.cpp)；判定：[cpp/core/visibility.cpp](cpp/core/visibility.cpp)；
测试：[prune_test.cpp](../../../tests/maps/cpp/prune_test.cpp)。覆盖多帧残影、遮挡、稀疏静态点、
同帧回波优先、地面/墙面/坡面保护、贴近墙面的独立残影、非零雷达原点和保存恢复。
本地测试不代表 S100P/NX 性能或实图清理质量已经验收。

后续优先做真实记录的删除点审查，再增加地面拟合与连续性保护、网页对比与恢复。
在线累计图若要清理，应在原生建图所有者中实现，不能周期性调用这个保存工具。
