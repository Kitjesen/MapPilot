# 建图、定位与保存：职责和数据范围

2026-11-02。依据当前工作树、固定上游代码和下列测试审阅；本记录不代表 NX 部署或实机验收。
它补充 [定位目录说明](../../src/localization/README.md) 和 [地图目录说明](../../src/maps/README.md)。

## 一、运行时真正的分工

```text
LiDAR / IMU 驱动 ── DDS ──> slamd
                              ├─ LTLIO：连续状态估计、去畸变、局部匹配
                              ├─ SLAM 会话：选取关键帧、保存关键帧档案
                              └─ online_mapping：后台图优化与有限点数预览
                                          │
                 新扫描 / 位姿             │ 接受回环后的校正关键帧
                        ↓                 ↓
                      mapd：实时地图、三维占据和自由空间
                        ↓
                  局部导航读取实时占据

保存并激活的 OctoMap ──> 全局路线规划
```

`slamd` 是进程；`Frontend`、`Worker`、`Backend` 是进程内对象，不是三套独立的
SLAM 进程。DDS 主循环接收输入并更新状态；`Worker::update()` 管理一个异步后台
批任务，`Backend` 在该任务内更新 iSAM2。建图模式推进该在线图，定位模式不把
每帧送入它。回环校正历史地图不直接覆盖连续里程计。入口：
[DDS 运行时](../../src/localization/slam/cpp/cyclone_runtime.cpp)、
[LTLIO 前端](../../src/localization/ltlio/frontend.cpp)、
[SLAM 会话](../../src/localization/slam/cpp/ltlio.cpp)、
[在线 Worker](../../src/localization/online_mapping/worker.cpp)、
[图优化 Backend](../../src/localization/online_mapping/backend.cpp)。

| 数据或工作 | 谁持有/负责 | 含义和范围 |
|---|---|---|
| 当前扫描与连续里程计 | LTLIO / SLAM | 前端消费激光和 IMU，成功处理扫描时更新对外扫描时间戳；不是每个 IMU 样本都发布一份新里程计。 |
| 关键帧档案 `patch_history_` | SLAM | 保存所选关键帧的机体系扫描、时间和图优化前的位姿，供保存及地图几何导出。 |
| `pending_` | 在线 Worker | 尚未交给后台批任务的帧；默认上限 4 不包括正在执行的一批或档案中等待投递的帧。 |
| 图与匹配扫描 | 在线 Backend | iSAM2 图、完整 PCL 扫描和滤波扫描；顺序因子来自原始 LIO 相对位姿，回环候选通过 ICP 等验证。 |
| 在线预览 | Worker / SLAM | 使用优化历史产生有限点数预览；SLAM 将其变换到地图坐标并发布。它不是可直接等同于 mapd 占据或完整保存地图的数据。 |
| 实时占据、自由和未知 | mapd `LiveMapEngine` | 根据当前扫描维护在线空间，并接收校正后的历史几何；它与保存事务是不同工作。 |
| 地图持久保存 | mapd `SaveCoordinator` / `SaveMapEngine` / `MapStore` | mapd 请求 SLAM 冻结输入，构建并验证地图产物，再提交地图；SLAM 写出的冻结输入不是最终保存事务的唯一所有者。 |
| 旧地图定位 | `NativeRelocalizer` + SLAM | 读取已保存地图、计算对齐并提交坐标变换；没有发现加载旧 iSAM2 图并跨会话续算的实现。 |

此处“原始关键帧”仅表示**尚未被图优化改写的扫描和位姿**，不是完整原始雷达数据包。
LTLIO 前端会选点、按范围过滤并去畸变，再生成保存的机体系关键帧：
[frontend.cpp](../../src/localization/ltlio/frontend.cpp)、
[recordPatchSnapshot](../../src/localization/slam/cpp/ltlio.cpp)。

## 二、校正实时地图和保存地图是两条链

接受新回环后，SLAM 可以导出冻结的校正关键帧/位姿，mapd 后台重建累计几何、
补上重建期间的新扫描，然后再安装结果并应答。它影响实时地图，不只是网页显示。
失败或数据已被替代时，mapd 不会直接安装半成品：
[SLAM 导出](../../src/localization/slam/cpp/ltlio.cpp)、
[mapd 重建与安装](../../src/maps/cpp/mapd/engine.cpp)。

用户保存则由 mapd 发起一次独立事务：SLAM 冻结关键帧/位姿等输入，mapd 校验后
生成并提交持久产物，包括导航使用的 OctoMap。已完成在线图优化的快照按其报告处理，
不会无条件再跑一遍旧离线 PGO：
[保存协调](../../src/maps/cpp/mapd/save.cpp)、[保存引擎](../../src/maps/cpp/save.cpp)。
加载地图、定位、实时更新和保存新图是不同操作；当前正式保存入口要求 `map` Product，
不能把导航期间的实时层更新表述成已经支持导航不中断地保存合并后的长期地图。

## 三、目前已经理顺的部分

1. 在线调度和后端均位于 `online_mapping/`；`Worker::update()` 明确表示既取结果又推进后台任务。
2. 点云和位姿文件读取编成独立的 `lingtu_mapping_io`，离线优化不再为此链接在线 Worker。
3. 旧 `OnlinePoseGraph` 没有产品或 CLI 调用，已连同专属测试和打包入口清理；离线工具仍按自身需要使用 GTSAM。
4. 有效关键帧上限取档案配置和 Worker 容量较小者；满额后前端继续，归档停止。保存的地图/轨迹截止时间和回执序号取保留前缀。此处“同一截止时间”不保证轨迹覆盖该会话的最早时间，见下一节。
5. Web 从现有状态读取容量提示，说明之后扫描不进入本次保存地图。没有因此自动扩容或跨会话续建。

## 四、仍需解决或明确的事情

| 事项 | 已确认事实 | 影响与证据边界 |
|---|---|---|
| 长时间轨迹范围 | `recordTrajectory()` 只保留最近 10,000 条；保存复制该窗口。 | 关键帧和地图可从更早时间开始，`trajectory.txt` 却可能缺少开头。独立合成回放接受 10002 个扫描，保存仍含 1.00 s 的首关键帧，但轨迹 10000 行从 1.02 s 开始，见 [轨迹对照](../../build/mapping-ownership-audit-20261102/trajectory/README.md)。前次“保存同一前缀”指截止位置一致，不保证完整会话轨迹。以每秒 10 条、无额外轨迹输入估算约 16 分 40 秒开始淘汰；现场速率需测。 |
| 状态字段语义 | `registered_keyframes` 实际是图中的位姿数量；`registration_rejections` 保留公开字段默认零，恒为零的私有副本已删除；`optimizations` 直接取 Backend 已接受回环数。 | 不能把它们解释为逐帧 ICP 成功/失败率或 iSAM2 每次更新计数。`rejected_keyframes=0` 不是回环从未拒绝的证据。先确定产品需要的统计，再修名称或删除无意义输出。见 [worker.cpp](../../src/localization/online_mapping/worker.cpp)。 |
| LiDAR 摄取 | 正式 DDS `drainLidar()` 一次消费一批后只交最后一个有效扫描；IMU 逐样本交付。 | 这是当前策略，不等于现场必然丢帧；现有前端内部掉帧计数不能代表这些被跳过的样本。需要真实记录输入/处理频率和延迟，不能声称每个雷达帧都归档。见 [cyclone_runtime.cpp](../../src/localization/slam/cpp/cyclone_runtime.cpp)。 |
| 预览与实时地图版本 | Worker 检测历史位姿变化并可刷新预览；地图几何导出按接受回环次数增加触发，不按每个 graph revision。 | 两者并非逐版本同步。独立合成闭环试验中，10 次无新回环的追加最大历史相对平移差约 0.45 微米；追加点人为放到远处以排除新回环，并非真实机器人路线。该试验不构成地图级错位证据，也不能证明其他数据永远相同。见 [试验记录](../../build/mapping-ownership-audit-20261102/README.md)。 |
| 内存口径 | `cloud_bytes` 仅统计 Backend 的完整/滤波 PCL 容量。 | 它不包含 SLAM 档案、等待队列、预览、图、保存捕获等，不能视为进程总内存。真实长录制和目标 NX 的峰值尚未测量。见 [backend.cpp](../../src/localization/online_mapping/backend.cpp)。 |
| 实现集中 | `LtlioBackend` 仍同时组织前端、档案、在线图、保存输入、重定位与几何导出。 | 目录整理已经改善，但类的阅读负担并未消失。下一步应围绕实际数据所有权找小而有用的提取，不为每个动作建立管理类。 |

## 五、已验证与未验证

既有验证：独立在线组件 6/6、离线工具 3/3、文件读取和合成回放通过；Linux
`slamd` 构建及容量、回环保存、3000 帧压力测试通过。见
[建图整理测试](../../build/mapping-cleanup-20261102/README.md) 和
[容量测试](../../build/online-mapping-cleanup-20261102/ltlio-capacity-registered-parallel-ctest.log)。
本轮另做了上文所述的无新回环数值对照和轨迹窗口合成测试，并移除一个确认无调用的
旧私有重置函数；删除后的 `slamd` 构建和 SLAM mock 测试通过。上述是代码/合成输入证据，不能推断现场
扫描丢失率、地图精度、长期资源开销或实机动作。

## 六、进一步核对：调度与交接边界

以下基于当前工作树的源码复核，没有新增故障注入或现场测试。

`slamd` 运行循环接收传感器后调用 `tick()`，默认目标调度频率为 50 Hz，运行参数可覆盖。一次 tick 先推进在线建图后台，再尝试处理前端扫描；数据未齐时可以没有新结果。有效结果才更新位姿、速度、点云和观测序号，并尝试记录轨迹和关键帧。

容量满时 `tick()` 和传感器入口继续工作，轨迹及关键帧档案停止追加，后台仍可处理已归档的待投递帧。实时定位与保存范围不同：保存截止到档案末尾。`Status::Ok` 表示调用完成，不等于定位健康；`slam_tick_hz` 是调度频率，不是有效扫描处理频率。参见 [tick](../../src/localization/slam/cpp/ltlio.cpp)、[运行循环](../../src/localization/slam/cpp/cyclone_runtime.cpp) 和 [容量测试](../../tests/localization/cpp/slam/test_ltlio_mock_flow.cpp)。

| 问题 | 已核实的行为 | 建议与证据限制 |
|---|---|---|
| 保存事务错误被健康说明覆盖（已修复） | mapd 在 `success=false` 时已优先使用事务 `message`；为空时才使用健康说明。成功 ACK 的健康判定保持原有语义。SLAM 健康文字改为 `SLAM state is healthy`，不再用 accepted 暗示事务成功。 | 新增消息冲突、两级兜底、多任务逆序回执及重复提交测试；原代码失败，修复后通过。见 [协调器](../../src/maps/cpp/mapd/save.cpp) 和 [测试](../../tests/maps/cpp/mapd_save_coordinator_test.cpp)；未做现场端到端故障注入。 |
| 快照与正式地图的状态边界（状态名称已修复） | SLAM 完成快照后报告 `map_snapshot_ready`，保留点云与已加载的重定位器。`map_loaded` 表示内存中已有地图。mapd 后续完成过滤、构建与正式地图包提交，再报告产品保存结果。 | 已核对调用方均按 `Status.ok` 处理快照成功，未依赖旧的 `map_saved` 字符串。更新两后端及异步保存回归，保留重定位器加载契约。构建和本地测试证据见 [正式接入核查](../../build/production-integration-20261103/state-audit.md)。 |
| 后台最新状态和最后有效几何混放（已修复） | Worker 保留本会话最新失败计数，SLAM 直接发布该值；最后有效几何继续保留。重置时清零，旧批次结果不更新新会话计数。Gateway 同步最新失败计数，保留旧点云的版本、时间与点数。 | 真实 Backend 异常回归在接通计数前失败、修复后通过，同时验证重置与旧结果隔离。Gateway JSON/二进制回归通过。尚未证明真实传感器链自然触发该故障或现场发生。见 [Worker](../../src/localization/online_mapping/worker.cpp)、[SLAM 输出](../../src/localization/slam/cpp/ltlio.cpp)、[Gateway](../../src/gateway/services/global_mapping.py)。 |
| 在线提示使用了另一套状态（已修复） | Web 从现有实时 SLAM 诊断识别 `online_mapping_failed:`，在场景通知处显示，不再依赖选择整图图层；区分已有地图预览与尚无预览。删除了对离线失败状态的误用。 | 正式提示函数的失败、有无旧图、正常及非建图输入测试通过，Web 构建通过；没有进行现场故障注入或浏览器视觉验收。见 [SceneView](../../web/src/components/SceneView.tsx) 和 [提示函数](../../web/src/services/mapSavePresentation.ts)。 |
| 实时观测与归档选帧不同 | mapd 以单个待处理槽保留最新观测，新观测可替换尚未处理的旧观测；接受、替换、处理数分别对外输出，默认占据更新周期为 50 ms。 | 这是实时地图延迟与吞吐的选择，不等于关键帧采样，也不能直接推断保存地图丢失或现场替换比例。先实测计数、频率和地图影响。见 [mapd 引擎](../../src/maps/cpp/mapd/engine.cpp)、[配置](../../src/maps/include/lingtu/maps/mapd/engine.hpp)。 |

现场触发频率、地图影响和修复效果尚需定向测试；本次没有改变建图或机器人运行行为。

后续清理：`mapd/save.cpp` 删除重复 JSON 转义并复用 `JsonEscape`，以一张按请求 ID 索引的表替代两张快照请求索引。低频的取消和重复提交在同一把锁内按任务 ID 查找；回执仍按请求 ID 匹配，保留重试与迟到回执隔离。生产代码净减少 34 行，没有新增类、源文件或依赖。Windows Release 重新构建后，`lingtu_maps_mapd_save_coordinator_test` 与 `lingtu_maps_mapd_service_dispatch_test` 两项通过；这是本地契约验证，不是 NX 部署或实机验证。

在线状态清理：删除 Worker 内部重复的输入关键帧列表及两项派生计数，直接使用 `frames` 和 Backend 的回环计数，保留公开字段及算法参数。Linux Worker 回归通过，记录在 [测试日志](../../build/online-state-fix-20261102/worker-green.log)；Gateway 9 项、Web 24 项定向测试通过，Python Ruff 与 Web 构建通过。快照隐式加载重定位器的既有契约未在这轮改变。

正式 Linux `slamd` 重新构建与链接通过，`messages_ltlio_mock_flow`、`messages_map_geometry_handoff` 两项集成回归通过，见 [SLAM 集成日志](../../build/online-state-fix-20261102/slam-integration.log)。构建仍报告原有 `MapGeometrySnapshot::map_body` 未显式初始化警告，Web 仍提示部分 chunk 大于 500 kB；本轮未扩大到这两项。上述验证未包含真实传感器触发后台异常的完整链路，也不是 NX 部署、浏览器视觉或实机验收。

后续行为保持清理：

- Worker 删除无人读取的 `EnqueueResult.message`（原本等于 `code`）、未使用的 `accept` 参数，以及恒为零的私有拒绝计数副本。
- SLAM 删除只写未读的保存任务时间/坐标字段、两个后端的地图路径私有字段，以及一次立即被覆盖的路径赋值；实际点云的时间、坐标和保存回执身份仍保留。
- 原本三处临时目录析构共用 `TempDir`。写入、等待提交、提交各阶段明确转交目录清理责任；恢复失败仍保留恢复文件，不增加新的任务层。
- DDS 等待保存结果只持有实际使用的五项捕获信息，不再额外保留整份 `SlamOutputs` 及其中点云引用。此处原本也是共享引用，不是每次复制整份点云。
- Linux 正式 `slamd`、回放工具和相应测试目标构建通过；`messages_contract`、`messages_ltlio_mock_flow`、`messages_map_geometry_handoff`、`online_mapping_worker` 四项通过，见 [本轮 Linux 日志](../../build/online-state-fix-20261102/linux-save-cleanup.log)。新增断言覆盖正常完成后的清理、拒绝覆盖已有临时目录、source epoch 失效后的清理。

保存后加载重定位器的接口契约仍然保留。已核实直接用 `loadMap()` 替代保存末尾的安装并不等价：它会清除捕获点云/序号、增加修订，定位模式还会重置对齐状态。该职责分离不能作为无行为变化的字段删除交付。

Windows 补充验证：专用 `build/slam-core-windows-x64` 原 DLL 配置出现 GTSAM/LTLIO 重复符号链接错误；复用本机已有的同版本 GTSAM 4.3.0 静态 SDK 后，新构建和 `messages_ltlio_mock_flow` 通过（1/1）。未安装依赖或使用强制链接选项，原缓存已备份。该测试包含保持旧 PCD 打开造成提交失败、验证旧点云/修订/PCD/位姿文件不变，以及失效任务和暂存清理，见 [Windows 测试日志](../../build/online-state-fix-20261102/windows-static-mock.log)。恢复操作本身也失败的分支仅完成清理所有权审阅，未追加文件系统故障注入；本轮没有部署或实机验证。
