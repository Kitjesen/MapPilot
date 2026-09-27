# Go2/NX 导航部署前自检（2026-09-27）

本轮在 Windows/WSL 本地完成。操作者最后确认机器狗未开机；没有连接 NX、安装发布包、
切换实机 Product 或发送运动目标。结论是可进入 ARM 整包构建和静止验证阶段，尚未通过
普通通道实机导航验收。

本轮修复提交：`9e95926b`（Host 遥测）、`b1e4cc1b`（完整网页发布）、
`06b42e42`（固件/软件版本分离）、`117280ca`（优化输出原点及保存默认值）。

## 修复

1. **可选路径遥测读取失败会终止整个 HostBus。** global/local path 分别隔离原生 ABI
   读取异常，traversability 读取失败使用已有错误字段。一次显示读取失败不会停止下一拍
   控制权/导航状态更新；核心导航状态和目标状态读取失败仍关闭 readiness。未增加自动
   重启框架。新增五个用例中，旧实现三个恢复用例失败；修复后 HostBus 共 16 项通过。
2. **完整发布包可能没有网页。** 标准 `make build` 现在先构建 Web，ARM 发布工作流
   配置项目已经要求的 Node.js 24。打包时要求 `web/dist/index.html` 和 assets 存在，
   自检覆盖缺网页拒绝、完整网页进入包、安装和失败回滚。保留带后缀的候选发布版本，
   不要求它与 Web 基础版本完全相同。
3. **PGO 输出完整性检查漏了雷达原点。** SaveMap 接收优化结果时要求非空
   `scan_origin.txt`，避免把缺少射线原点的结果作为完整优化源提交。现有原生 PGO
   已复制该文件；新增缺文件反例，并验证正常优化地图能通过激活准入。
4. **保存入口默认值过时。** `mapd` 的地面膨胀、补空闲层、空闲膨胀默认由 1/3/1
   对齐为 0/0/0。saved-ray 构建此前已经强制使用零值；这不是本次起点失败的根因，
   也没有改变已通过的射线地图或放宽导航条件。
5. **发布同步错误覆盖硬件固件版本。** `sync_versions.sh` 原来会把 Go2 的
   `firmware_version` 改成 LingTu 软件版本。删除该错误同步，保留当前实际未知的
   硬件固件信息；同时为此脚本声明 LF，修复 Windows 检出后 WSL 无法执行的问题。
   修复后软件版本一致性检查通过，RobotConfig 未修改。

## 完整候选图经过正式地图流水线

输入为本地完整旧 `903room` 的 185 个 patches、同次保存的 poses、scan origin 和
manifest。源 `content_epoch=1790090858010`。使用之前从原始扫描按实际 poses 恢复的
1,375,003 个回波作为候选 `map.pcd`，没有插值、补地面或修改原图。

独立程序直接调用生产 `MapPipelineCore` 和 `MapStore`：

| 检查 | 结果 |
| --- | --- |
| `CommitSavedSourceJson` 必需清理与源事务 | 通过，保留 1,281,066 点，移除 93,937 点；提交阶段未再降采样 |
| `BuildNavigationPackageJson` | 通过，包括现有展示/辅助产物和完整三维 `octomap.ot` |
| 射线证据与版本 | `saved_rays`、`full_log_odds`、builder 0.3.1；六项统计完整 |
| `CheckMapActivation` | 通过，`blockers=[]`；这是本地准入检查，没有激活实机 Product |
| 相同输入再次构建 | 内层回放报告 `reused`，未错误复用旧版本 |
| 正式产物上的三个固定规划案例 | 全部通过，含两个 2 m 目标 |

起点保持 `[0.053,0.173,0.01]`，目标保持 `[0.5,0.17,0.01]`、`[2,0,0]`、
`[0,-2,0]`；使用同一原生规划器及之前固定的生产参数。终点误差分别约 0.00424、
0.02526、0.02526 m。没有为提高通过率换点或关闭支撑/碰撞检查。

六项统计：valid 1,375,003；retained 1,316,496；dropped 58,507；free updates
53,976,422；hit updates 1,059,368；guard suppressions 6,665,426。与此前独立回放
结果一致，证明正式地图事务使用了同一修复。

本机日志和程序在 `build/maps-preflight-20260927/`：`report.json`、`result/source.json`、
`result/build.json`、`result/activation.json`、`result/reuse.json`、`plans.json`，以及
`preflight.cpp`、`run.py`、`plans.py`。地图算法基线提交为 `8bc4766c`，builder 0.3.1。
复用发布后的候选 `content_epoch=1790521949069`。这些是本地离线证据，不是实机状态。

清理约 281.26 秒，完整流水线约 373.67 秒；清理当时保留 300 秒上限。
2026-09-28 校正：OctoMap 的 180 秒配置没有被原生构建器执行，不能称为构建上限，
本次清理移除了该失效参数。清理已接近上限，且本轮存在并行编译负载。没有发生超时，
也未提高限制；NX 的实际
耗时与内存还必须测量，若超时再根据实际热点处理。

## 已检查的导航与发布边界

| 项目 | 本轮证据 |
| --- | --- |
| Go2 配置与全局/局部解耦 | 37 项配置回归；全局半径 0.155 m，SCAN 双圆柱半径 0.25 m、偏移 0.19 m，二维通行代价关闭 |
| 预览与正式提交 | 76 项 Gateway/地图绑定回归；同一全局规划入口，正式目标在 native 重新规划，地图/坐标身份变化会拒绝旧结果 |
| 地图 blockers 展示 | Maps API 40 项、Web Maps 16 项；列表首条和详情完整原因均覆盖 |
| 预览起点与三维高度 | Web helper 4 项；地图、会话和连接变化清除旧预览 |
| 新鲜度、控制权与 readiness | 37 项导航投影/控制循环回归；最终运动仲裁仍在 native navd |
| 状态转发异常恢复 | HostBus 16 项及 ruff；可选读取失败可恢复，核心失败仍拒绝就绪 |
| 停车入口 | 代码核对 `/api/v1/stop` 直达 native estop，不依赖控制租约/导航就绪；实际停车距离尚未测量 |
| 保存/优化事务 | 原生 SaveMap 1/1 通过，58.35 秒；缺原点拒绝与正常优化激活检查覆盖 |
| 原生地图服务 | 更新后的 WSL x86_64 `mapd` 编译链接成功；尚非 ARM 构建 |
| 发布/安装/回滚 | 发布/ABI/限速契约 31 项、ProductControl readiness/回滚 16 项、packager 自检通过；自检使用测试安装目录，不能替代 ARM 真包安装 |

验证入口包括 `tests/lingtu/assembly/test_bus.py`、`tests/maps/cpp/save_map_test.cpp`、
`tests/contracts/test_ros_free_default_build_entrypoints.py`、
`tests/contracts/test_install_layout_contract.py`、
`tests/runtime/test_installer_product_control_boundary.py`、
`tests/runtime/test_map_activation_contract.py` 和 `tests/lingtu/real/test_real_switch.py`
的相关场景，以及 `bash scripts/deploy/package_native_release.sh --self-test`、
`npm --prefix web run build`、`npm --prefix web run test:map-navigation`。
发布测试及网页构建结果记录于本次工具输出，未另存文件日志；HostBus 红/绿回归日志
位于上述本地 `build/maps-preflight-20260927/` 目录。
SaveMap 日志为 `build/ownership-fix-20260927/maps/Testing/Temporary/LastTest.log`；
WSL 地图服务产物为 `build/maps-mapd-wsl/mapd`。编译有既有 WSL 文件时间偏差提示，
最终编译链接退出码为零。

`tools/release/sync_versions.sh --check` 首次因 CRLF 无法执行；修复换行后又暴露
上述硬件固件与软件版本混用。移除错误同步后，同一检查成功返回
`Version metadata is in sync`。发布工作流调用的同步脚本不再改写 Go2 固件版本。

本轮没有改变 SCAN 包络、unknown、地面支撑、步高、坡度或运动安全规则。0.5 m
窄通道仍属于后续独立标定；全局路线通过不等于 SCAN 的完整机身轨迹可执行。

## 上机前最终补充自检

同一恢复候选源（1,375,003 点、185 帧）在无并行负载的 WSL Release 构建上单独计时：
清理由 104.65 秒降到 88.57 秒，峰值内存约 91 MB。`clean.pcd` 与 `removed.pcd`
逐字节一致（保留 1,281,066、移除 93,937），prune 原生回归 17/17 通过。改动只是
逐点帧标记替代每帧哈希集合，并跳过同帧重复计算，判定规则不变。前文 281 秒含并行
编译负载。剩余热点是射线管内逐点距离和体素哈希查找（约 80%），以及支撑面判断
（约 16%）；没有 NX 实测前不做多线程改造。

NX 若首次保存报告 `dynamic_filter_timeout`，采集数据保留，可在
`/etc/lingtu/maps.env` 写入 `LINGTU_MAP_SAVE_DYNAMIC_FILTER_TIMEOUT_SEC=600`，
重启 `lt-maps.service` 后重试保存；同时记录 NX 实际清理耗时，再决定是否优化。

`mapd` 本地管理端口是单线程顺序处理。网页地图工作台的导入 PCD、重建 OctoMap
是同步请求，NX 上可能持续数分钟，期间 Gateway 的活动地图查询会失败，页面短暂
显示无地图并拒绝新目标。这些操作不影响 navd 已加载的地图，但导航时不要执行；
正常保存走异步任务，不受此限制。

## 开机后的剩余验收

1. 从包含本轮修复的最终提交构建 ARM ABI 11 整包，更新 DDS、客户端、Host、mapd、
   navd、prune、网页；核对实机进程和配置，不继续安装历史 `.83` 源码构建物。
2. 机器狗停稳后，通过 ProductControl 使用候选图完成定位、`/ready`、控制权稳定性和
   无运动预览。核对实际起点及环境与旧 `903room` 是否一致。
3. 网页速度下拉选择 **0.20 m/s**。网页当前默认 0.4、Product 上限 0.75，不能误当成
   验收限速。正式目标的 `max_speed_mps=0.2` 会经 Gateway/ABI 传给 navd 限制执行速度。
4. 现场看护下验证普通直行、绕静态障碍、行人经过、断连/重连及停车。避障保持开启。
   记录 NX 保存耗时和内存、真实定位、局部轨迹和运动结果。

仍缺少首选 `903room_v4_5cm_rays` 完整包和现场墙/柱/残影标注，不能用旧图三个路线
通过来宣称所有残影清理正确。只有上述实机运动验证完成，才能报告导航验收通过。
