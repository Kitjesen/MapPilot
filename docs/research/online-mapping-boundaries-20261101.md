# 在线建图职责与构建边界调查

> 2026-11-02 更新：下文保留调查时的旧路径和待办。在线调度现已迁入
> [`online_mapping/worker.cpp`](../../src/localization/online_mapping/worker.cpp)，
> 文件读取单独构建供离线优化和在线回放共享；关键帧满额时继续里程计、停止新增
> 建图关键帧，并只保存已记录的前缀。当前职责见
> [定位目录说明](../../src/localization/README.md)。

2026-11-01，依据当前工作树源码和构建定义调查。结论是：在线队列与快照 worker
位于 `map_optimization/`，在线图优化 Backend 位于 `online_mapping/`，目录和命名空间
与职责不一致。当前正式在线链只有一个 Backend，不能因目录里还有其他图优化类就
认定两个估计器在竞争。本次没有移动代码、修改运行行为、重跑测试或检查目标机。

## 一条实际运行链

```text
LTLIO 连续里程计 + 去畸变点云
  -> slam/cpp/ltlio.cpp：选择关键帧，保留原始 patch
  -> map_optimization/online_mapping.cpp：待处理队列、异步批任务
  -> online_mapping/backend.cpp：回环验证、iSAM2 图及优化位姿
  -> online_mapping.cpp：历史位姿与有限点数预览快照
  -> ltlio.cpp：接收快照、地图坐标变换、保存及地图几何导出
```

| 部分 | 实际职责 | 入口 |
| --- | --- | --- |
| LTLIO | 激光/IMU 高频里程计；根据时间和运动选择、保存原始 patch | [ltlio.cpp](../../src/localization/slam/cpp/ltlio.cpp) 的 `recordPatchSnapshot` |
| 在线 worker | 保存待处理帧、启动/回收异步批任务、生成预览和历史位姿快照 | 当时的 `map_optimization/online_mapping.cpp`；现为 [worker.cpp](../../src/localization/online_mapping/worker.cpp) |
| 在线 Backend | 接收连续原始里程计及机体系点云，做 ICP 回环、增量图优化 | [backend.cpp](../../src/localization/online_mapping/backend.cpp) 的 `Backend::append` |
| 保存任务 | 冻结完整 patch 与相应位姿；区分在线结果和离线优化输入 | [ltlio.cpp](../../src/localization/slam/cpp/ltlio.cpp)、[save.cpp](../../src/maps/cpp/save.cpp) |

`LtlioBackend` 持有一个 `OnlineMapping`；其 `State` 持有一个 `Backend`，后者持有一个
iSAM2。普通关键帧使用相邻**原始**里程计位姿差生成约束；回环验证通过才向同一个图加
回环因子。LIO 前端的连续位姿不因历史图校正直接跳变。来源是固定 LIO-SAM 的相关图
优化逻辑，本地将 ROS 线程与输入适配成自己的 worker；精确来源和差异见
[UPSTREAM.md](../../src/localization/online_mapping/upstream/UPSTREAM.md)。

`pollOnlineMapping` 与 `pumpOnlineMappingFrames` 在 `Mapping` 模式执行。`Localization`
使用已有 LIO 和定位变换链，并不将每帧输入这个在线图后端。保存时识别
`lio_sam_isam2` 报告；在线快照与显式离线因子冲突会报错，而不是将两者直接混用。
这不等于所有保存情况都运行了优化；是否已有回环校正由快照记录。

## 为什么看起来像有两套队列

`ltlio.cpp` 的 `patch_history_` 保留原始 patch，并用 `next_patch_to_queue_` 记录哪些
尚未送到 worker。`OnlineMapping::pending_` 是 worker 当前待处理批次。前者服务保存和
延后投递，后者安排计算；并非两个独立图优化器。默认 `max_pending=4`、
`max_keyframes=3000`，实际配置及 patch 容量以运行配置为准。

调用方检查 `canEnqueue()`，队列满时暂不推进待投递索引，后续轮询再尝试；达到总容量
则暂停建图。直接调用 `enqueue` 失败会记录 dropped。保存经过优化的地图时还检查
patch 与已优化关键帧数量及身份，避免用不完整历史生成校正地图。

`poll()` 既收取已完成任务，也启动下一批 `std::async`，并非单纯读取状态。每次最多
一个批任务使用当前 Backend。重置时旧任务可以继续收尾，但旧 generation 的结果被
丢弃；旧任务结束前新任务不会启动。这是需要保留的生命周期行为。

预览快照与保存原件也不同：Backend 保存算法所需点云，LTLIO patch 日志保存原始扫描；
worker 的预览有点数上限。它们有不同用途，不能为了减少文件或看似重复而直接删掉。
worker 内的 `optimizations` 当前在回环被接受时递增，不能当成 iSAM2 总更新次数。

## 进一步发现：依赖关系也跨界

`slamd` 通过 [slam CMake](../../src/localization/slam/cpp/CMakeLists.txt) 引入
`map_optimization/online`，该目标构建 worker、`cloud_io.cpp` 并链接在线 Backend；
它没有因此运行离线 Rust PGO。

反向依赖更值得整理：[map_optimization CMake](../../src/localization/map_optimization/CMakeLists.txt)
把整个 `lingtu_online_mapping` 作为离线库的 PUBLIC 依赖。离线代码使用的
`read_poses` 和 `read_point_cloud` 在与 worker 一起编译的
[cloud_io.cpp](../../src/localization/map_optimization/cloud_io.cpp) 中；已核查的离线
源并不调用在线 `Backend`。因此在线 worker 和后端成为离线构建依赖，即使离线作业
不需要运行它们。离线 `gravity_graph` 本身仍使用 GTSAM，解开此依赖不意味着可以
删除离线 GTSAM 依赖。构建/安装副作用及最终可执行链接须在实际重构时验证。

另有当时的 `OnlinePoseGraph` 历史类；该类及其专属测试已于 2026-11-02 删除。
本次在 `src/`、`tools/`、`tests/` 检索到其实现和专属测试，没有发现正式 LTLIO
调用。它编入离线库不等于正式在线运行了另一个估计器；仅凭本次检索也不应直接删除。

## 建议的最小整理

1. 将现有在线 worker 的源文件、命名空间和构建入口归入 `online_mapping/`，继续使用
   现有 Backend；SLAM 的 patch 保存、模式判断、地图坐标和保存事务仍由 SLAM 持有。
2. 理清 `cloud_io.cpp` 的构建归属，让离线目标无需链接在线 worker。优先考虑现有
   源码直接归入所需目标，检查同时链接场景和安装，不为了迁移新建运行时包装层。
3. 同步 SLAM、replay、测试与文档中的路径和 CMake；历史记录保留当时路径。暂不因为
   名字相似删除 `OnlinePoseGraph`，先确认其支持用途。
4. 保持关键帧选择、原始里程计因子、回环判据、队列容量、坐标和保存语义不变；这次
   目录整理不应声称提升精度、吞吐量或现场性能。

迁移验收应针对：队列满后继续投递、reset 时旧结果不进入新 session、首次回环与
连续里程计锚点、保存期间持续采集和完整 patch、回环后地图几何交接；还需构建在线
SLAM、离线 `lt_pgo`、replay 和受影响平台。已有针对性测试位于
[worker_test.cpp](../../tests/localization/cpp/online_mapping/worker_test.cpp)（迁移后的路径）、
[test_ltlio_mock_flow.cpp](../../tests/localization/cpp/slam/test_ltlio_mock_flow.cpp) 与
[test_map_geometry_handoff.cpp](../../tests/localization/cpp/slam/test_map_geometry_handoff.cpp)。
是否存在实际帧延迟、内存峰值或 NX 长期吞吐问题，本次未测量，不能仅凭源码推断。
