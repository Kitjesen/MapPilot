# NX 地图保存耗时调查与本地精简

## 11 月 5 日继续处理：128 秒降到 47.6 秒

同一份 1,375,003 点、185 patches、同一清理参数的 Windows Release 单次对照：

| 实现 | dry-run 墙钟 |
| --- | ---: |
| 最初当前 v4 | 149.036 s |
| 第一轮探针复用 | 128.094 s |
| 本轮完整精简 | 47.557 s |

比上一轮减少 62.87%，比最初减少 68.09%。该时间仍只衡量 prune，
不含 SLAM 快照、OctoMap 和完整保存事务，不是 NX 的测量。
另行运行了带文件输出的原始实现和最终实现，`clean.pcd`、`removed.pcd`
均逐字节相同，完整 JSON 分类报告也相同。证据在
`build/map-save-latency-20261104/comparison-final.json` 与 `exact-reference/`、`exact-final/`。

实现保持当前判断，减少重复查询：

- 先完成整帧落点确认，再处理自由射线；未曾观测、同帧确认占据及已累计足够自由帧的点
  不再重复计算自由探针。未来重新命中仍重置证据，仍可再次累计自由帧。
- 落点确认只查实际确认半径的轴对齐范围内的格子，随后保持原精确球形距离判断。
- 每格在本帧真正被射线访问时才统计候选数；候选已全部完成时跳过；
  同一条射线重复访问同一格时复用结果，因为探针距离本来就针对整条射线。
- 表面判断按原体素分组复用一份邻桶列表，每点仍用原精确半径、邻居顺序和多层拟合。
  只保留一格缓存和每源点一位结果，输出仍按原点顺序。
- 主平面已通过时不再构造备用拟合的点数组。

曾试过额外缓存相邻 DDA 步的格子范围，整图没有明显收益，已删除。
65 秒中间版本的阶段诊断中，落点确认约 15.05 s、全图候选统计 5.36 s、
射线遍历 33.59 s、表面判断 7.12 s；它用于指导后续修改，不是最终版本的分段耗时。

保存提交还省掉了一次同卷整包复制：将私有 `staging_map` 移动到 `commit_stage`，
仅跨文件系统时退回复制。捕获目录、旧正式地图、原子提交、刷盘和失败恢复仍保留。
同卷移动后的验证失败、取消和服务中断恢复均已通过直接保存测试。
Linux 的工作目录和提交目录通常同在 map root，Windows 的 TEMP 与地图根可能跨卷，
不能把同卷收益计入 Windows 跨盘或上述 prune 时间。
Windows 的实际 C→D 测试发现 `rename` 返回 Access denied，不能仅靠
`cross_device_link` 错误码识别跨盘。实现已对 Windows 明确不同根盘符使用原复制流程；
没有把任意权限错误都降级成复制，同卷仍使用移动。

当前正式 prune、mapd 和 mapctl 已重新构建，19 项清理测试通过。
这些优化不等于地图质量变好：输出故意保持相同，以排除通过少做清理或更改阈值换速度。

## 现场质量与耗时证据的边界

本轮进一步找到 `build/go2-map-save-v6-20260930/source-result.json`：
2026-09-30 NX 的旧 `prune.visibility_v2` 实际用时 437.255381 s，
605 patches、4,452,709 点清为 3,836,924 点。它不是当前 v4 的现场计时。
旧图的地面参考带删点及保留点/OctoMap free 冲突有记录，但不能把每处都认定为误删。
该图缺完整 605 帧原始扫描与标定原点，当前无法用同一输入重放 v4 并判定质量修复。
详细来源、坐标、缺失数据与版本字段在
`build/map-save-latency-20261104/quality-evidence.json`。

下面保留 11 月 4 日的初轮调查与测量范围。

2026-11-04。用户报告 NX 大地图保存慢，要求深挖并先在本地处理。
本轮未连接到 NX：已记录的 192.168.66.65、192.168.123.18 均连接超时。
以下区分当前源码、当前 Windows 测量、历史测量；不将本机耗时作为 NX 耗时。

## 保存按钮实际等待什么

```text
SLAM 捕获匹配地图与关键帧
  → 全量 PCD、轨迹、patches 写出
  → ICP / 可用的 BBS 重定位索引准备
  → SLAM 主循环发布整份地图，然后发送 snapshot ACK
  → mapd 主循环遍历 capture 并逐文件刷盘
  → 后台任务：工作副本 → 必要的离线优化 → 动态点清理
  → 源点云重写 → OctoMap 射线重放 → 提交副本、验证、刷盘
  → 正式目录提交、临时数据清理 → 成功
```

当前 `config/system/settings/maps.yaml` 默认启用且要求动态点清理，
清理超时为 300 s，保存 OctoMap 分辨率为 0.10 m。
在线 `lio_sam_isam2` 快照会跳过离线 PGO，不能把每次保存都归因为再次做 PGO。
Web 每 750 ms 查询同一操作，360 s 是等待上限，不是固定等待时间。

## 已证实的成本与职责问题

1. `startSaveMapAsync()` 在启动线程前，部分分支仍在 SLAM 主循环做
   `mapSnapshot()` 和全图坐标变换。patch 列表使用共享指针，并非此时复制所有扫描。
   见 `src/localization/slam/cpp/ltlio.cpp`。
2. `writeCapturedCloud()` 写全量地图时仍构造完整的内存 `result.cloud`，
   随后逐 patch 写文件，并在快照完成前准备重定位索引。没有回环校正时，
   匹配地图前缀与保存的扫描都会进入输出，因此输入点数还取决于重复观测量。
3. `cyclone_runtime.cpp` 在快照 ACK 前调用 `toDdsCloud()`、`writeSavedMap()`。
   整图 DDS 缓冲为 16 字节/点；千万点对应约 160 MB。是否存在阻塞发布仍需 NX 实测。
4. `SaveCoordinator::Poll()` 位于 mapd 主循环，成功 ACK 后同步调用
   `ProvideSnapshot()`，后者在加入后台队列前 `TreeBytes()`、`SyncTree()`。
   Linux 会逐文件 `fsync`；这段时间也占用地图服务主循环。必要持久化不能直接删除，
   应结合恢复语义移入正确的后台阶段。
5. 后台任务会复制 capture 到工作目录，source commit 再写地图并复制全部辅助文件，
   navigation-package 内部 staging 再复制一遍，外层最终提交又复制整包。
   辅助文件包括 patches、清理前备份及 clean/removed 点云。大包被多次读写。
6. prune 与 OctoMap 分别重放全部扫描射线：前者判断历史点是否被后续自由观测否定，
   后者建立最终占据/自由地图。两者输出不同，不能简单删去其中一个。
   prune 还按原地图点逐点建立证据，重复观测造成的点密度会增加同格候选查询成本。
7. 一个 SaveMap worker 顺序处理任务；排队时长需要与该任务计算时长分开。

主要源码：

- `src/localization/slam/cpp/ltlio.cpp`：捕获、写点云/patch、准备重定位器。
- `src/localization/slam/cpp/cyclone_runtime.cpp`：整图 DDS 发布与快照 ACK 顺序。
- `src/maps/cpp/save.cpp`：快照刷盘、后台阶段、复制、提交、事件日志。
- `src/maps/cpp/build/pipeline.cpp`：动态清理、源点云处理、导航包构建。
- `src/maps/prune/cpp/core/visibility.cpp`：逐射线/逐点可见性计算。
- `src/maps/include/lingtu/maps/octomap/rays.hpp`：保存扫描的 OctoMap 射线重放。

## 测量证据

当前基准在 `build/map-save-latency-20261104/`。输入为 185 个实测 patch、
1,375,003 个重建地图点，使用当前源码重新构建的 Windows Release `prune`。
`--dry-run` 完整执行分类，但不写清理结果，不修改原始输入。
基线单次墙钟为 **149.036 s**，保留 1,354,643 点、删除 20,360 点。
这证明当前数据上的清理计算本身已有分钟级成本，不是磁盘写出才造成的。
它不是完整 SaveMap 测量，也不能推算 NX 的绝对耗时。

同输入的局部精简后单次墙钟为 **128.094 s**，减少 **20.942 s / 14.05%**。
基线与修改后的完整 JSON 分类报告相等。此次 dry-run 没有输出 clean/removed PCD，
不将报告相等描述为百万点输出文件逐字节比较。前后没有并发构建或其他本任务的
重负载测试；仍是单次本机对照，不是统计意义上的稳定加速比。
命令、原始报告与比较见 `build/map-save-latency-20261104/comparison.json`。

历史证据仅作为范围参考：

| 数据 | 测量范围 | 已有耗时 |
| --- | --- | ---: |
| 10,644,022 点，495 patches | 2026-10-22 Linux SLAM stage + target + commit；无 mapd/OctoMap，BBS 不可用 | 24.82 s，峰值 RSS 808 MiB |
| 37,196,794 点，2,816 patches | 同上 | 89.22 s，峰值 RSS 2.40 GiB |
| 46,875 点，25 patches | 2026-10-23 本地完整保存；显式离线 PGO，关闭动态清理 | 12.09 s，其中 PGO 6.13 s |

上述大图原始保存包现存约 1.71 GB、2,826 个文件。包大小说明多次复制的量级，
不表示本次已经测出 NX 的磁盘吞吐或换页情况。
历史 prune profile 中主要成本在射线证据、哈希查询与局部支撑面检查；
历史 90–105 s 结果来自旧版本/不同构建，不与本次 Windows 149 s 作回归比较。

## 本轮局部修改

- prune：同帧七个探针已经全部覆盖时跳过后续重复计算；部分覆盖时跳过已置位探针。
  保留帧重置、占据落点优先、每帧一次投票和全部参数。相关 19 项测试通过。
- source commit：`LoadPcdXyz()` 已在入口去掉非有限点，默认不降采样时直接移动
  已加载数组，避免 `FilterPcdPoints()` 再分配和复制整图。提前保留源点数，
  启用体素降采样时仍调用原逻辑。直接保存/导入两个测试目标通过。

保存测试目标缺少 OctoMap include 路径的既有构建问题，本轮通过构建进程的
`CL /I` 参数使用现有安装完成测试，没有新增依赖或修改 CMake。
数组移动带来的大图峰值内存/耗时尚未单独量测，不算入 prune 的提速数据。
当前本机正式 `build/prune/Release/prune.exe`、
`build/maps-windows/Release/mapd.exe` 和 `lingtu-mapctl.exe` 已重新构建。
这不代表 NX 部署产物已更新。

## 仍需处理与验证

优先把实际慢任务 `.save_jobs/<job_id>/events.jsonl` 的阶段时间取出来，
并读取 `source_report.dynamic_filter.elapsed_sec`。当前没有该次 NX 记录，
无法给出快照、清理、OctoMap 和复制刷盘的现场占比。

下一步结构整理应围绕一个私有工作目录完成后处理，再保留一次最终原子提交，
避免内外事务反复复制整包。重定位索引准备、整图发布和 snapshot ACK 的关系也应
单独梳理，先查清调用者和加载语义；不能直接删去索引准备并宣称所有加载仍可用。
保存恢复、取消、错误传播、原地图保留及最终刷盘保证都需要相应测试。

本轮没有降低地图分辨率、删除动态清理、取消射线证据检查、放宽导航判断或部署 NX。
