# 建图迁移接续计划（2026-09-16）

当前交接分支：`codex/development-handoff-20260916`。
这是开发检查点，不是实机发布版本。当前实现、限制和已有验证见
[MIGRATION.md](MIGRATION.md)。不要把本次混合工作区快照中的所有功能都说成已通过验收。

## 先完成换机

新电脑安装 Git（含 Git LFS）、Python 3.10+、uv、Node.js 24、Rust stable、
CMake，以及 Windows 下的 Visual Studio 2022 C++ Build Tools / Windows SDK。
其他系统分别按仓库构建文档准备，不能复用 Windows 编译产物。

```powershell
git lfs install
git clone --branch codex/development-handoff-20260916 https://github.com/Kitjesen/MapPilot.git lingtu
cd lingtu
git lfs pull
git submodule update --init --recursive
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
npm --prefix web ci
npm --prefix web run build
```

私有仓库先用自己的 GitHub 账号登录。SSH 私钥、设备密码、Tailscale 登录和
本机环境变量不在提交中，也不要复制到源码。依赖缓存 `.venv`、`node_modules`、
旧 `CMakeCache.txt` 和编译 DLL 不应跨机器照搬。

原生 SDK 按 [scripts/build/README.md](../../../scripts/build/README.md) 准备，
Windows 用 `prepare_cyclonedds_windows.ps1`、`prepare_slam_dependencies_windows.ps1`
及 `build_slam_core_windows.ps1`，显式指定新电脑实际的依赖路径。
完整运行入口见 [Getting started](../../../docs/getting-started.md)。

真实回放数据不进入 Git：从旧电脑复制
`build/handoff/lingtu-mapping-data-20260916.zip`，在新仓库根目录解压。
内含本轮扩大地图记录、回放结果和临时诊断源码。它不是所有历史录包的备份；
旧电脑其他 `build/`、地图目录及原始录包暂时保留，确认需要后另行迁移。

交接时本地 HEAD 为 `c37e9e03`，远端 main 为 `0de01014`，两侧分别有
30 / 1 个独有提交。先在交接分支恢复开发；后续单独审阅并合并 main，
不要直接 reset 到 main，也不要 force push 覆盖两侧历史。

## 工作顺序与验收条件

| 顺序 | 工作 | 怎么做 | 完成条件 |
| --- | --- | --- | --- |
| 1 | 重现基线 | 新机器构建原生优化器、运行单元测试和同一份 278 帧记录 | 测试通过；235 配准帧、43 拒绝帧、10 闭环、6 优化的基线差异有解释；不以耗时相等为要求 |
| 2 | 修复配准覆盖缺口 | 围绕最后接受的第 234 帧，比较后续帧原始点、降采样、法线、对应关系、姿态与时间；比较连续拒绝后恢复策略 | 解释并处理尾部 43 帧；可恢复场景恢复连接，退化场景明确标记；不能放松到错误闭环，也不能删点掩盖缺口 |
| 3 | 完成保存/重载闭环 | 保存校正后的全分辨率地图、关键帧位姿和轨迹；关闭后重新加载，核对同一坐标下的重建与定位 | 文件一致、地图非空且覆盖正确；原始 patch 可追溯；重载定位和路径预览通过；未完成优化时状态明确 |
| 4 | 实时资源优化 | 用录制时间戳节奏回放；分别测邻接配准、闭环检索、PGO、整图重建、传输；再上 ARM 测 p50/p95/p99、峰值内存和队列 | 代表性长走不持续积压/丢帧；SLAM 和导航频率不受后台优化阻塞；给出目标板测量，不拿桌面串行回放代替 |
| 5 | 整图与质量展示验收 | 打开整图，走出局部窗口，闭环前后对照；切换局部/整图、断链重连、重启及保存地图 | 旧区域仍可见；当前机器人与扫描对齐；不混旧 epoch；质量不足有简短提示；地图不被说成可通行图 |
| 6 | 仿真验证 | 用同一 Product/原生 SLAM 后端覆盖闭环、退化走廊、重启、队列压力和保存重载 | 通过后记录场景、版本和结果；单元测试不能替代这一层 |
| 7 | 实机部署与建图 | ARM 构建并部署同一版本的 slamd/Host/Web；先静止检查，再由现场人员带狗绕圈返回起点 | 校正前后误差、队列、丢帧、定位连续性有记录；能保存并重载；无错误闭环 |
| 8 | 导航验收 | 在校正地图上定位、预览路径，现场监督下做短程、绕行、动态障碍停车、通信中断和恢复 | 可重复到点，障碍与失联可靠停车；通过低速后再验证 0.50 m/s，不能用本次建图测试证明避障安全 |

## 最小可重现命令

在 Visual Studio 开发终端中，先只构建不依赖 PCL/DDS 的优化器测试：

```powershell
cmake -S src/localization/opt -B build/mapping-check -DBUILD_TESTING=ON
cmake --build build/mapping-check --config Release --target online_mapping_test constraint_assembly_test lt_mapping_replay
ctest --test-dir build/mapping-check -C Release -R "online_mapping|constraint_assembly" --output-on-failure
uv run --locked python -m pytest tests/gateway/test_global_mapping.py -q
```

Windows 多配置构建的回放命令（Linux 单配置构建去掉 `Release/`）：

```powershell
build/mapping-check/Release/lt_mapping_replay.exe build/go2-live-validation-20260913/go2_expanded_20260914_0350 build/mapping-check/replay.json
```

这是串行离线回放；下一步仍需按录制节奏回放和目标板测量。
完整 Fast-LIO 原生构建后还需运行 `messages_tf_publication` 和
`messages_odometry_publication` 测试；Windows 测试进程的 PATH 必须包含匹配的
CycloneDDS SDK `bin`。已有证据与警告见 MIGRATION.md，不能把新机器上的失败忽略为旧问题。

## 下一位开发者先做什么

显示修复接续：导航通行图、建图投影在数据过期时不再自动切换到点云；
实时风险层与当前显示投影对齐，离散风险纹理不再使用线性缩小采样；
“全图”优先取当前可见栅格的范围，目标圈落在显示投影上，目标查询高度保持不变。
29 项地图显示/选点/新鲜度测试及 Web 构建已通过，仍需浏览器与实机画面验收。
这些是显示修复，不代表地面分类、障碍膨胀或动态避障已经完成验收。

1. 确认分支、LFS 资产和回放数据到齐，跑上面的最小基线。
2. 从第 234→235 帧开始定位 `insufficient_planar_correspondences`，保存对应点与法线的可视化证据。
3. 同步补齐保存/重载的端到端测试，再优化性能；不要一开始重写整个前端或换 SLAM。
4. 需要上机器时先读当前设备状态和版本，遵守 ProductControl、mapd、navd 的所有权，不沿用旧终端的控制会话。

## 2026-09-16 合并前测试进展

尚未合并。扩大本地回归后，Web 262 项、原生 SLAM 15 项、优化器 5 项、
地面/投影/保存 4 项通过；修正了过期测试和一处“整图只能保存后看”的矛盾说明。
保存链路的源码合同和使用假转换器的保存测试，不等于校正地图实机保存重载通过。

随后网线恢复，已连接 Go2 NX。旧版 `.31` 启动失败定位为 DDS 检查工具未随
MapScene 消息结构更新；同 IDL 重编后，同一 Product 启动成功。
修正了 D435i/RSUSB 相机诊断（从已提交 RunPlan 读取驱动，不强制 V4L2 节点），
修正后的静止 doctor 在 NX 通过。

旧版两分钟静止采样出现 4 次 `map_scene_generation_pending`：两个独立 DDS
通道的最新序号被错误要求完全一致。HostBus 现保留各自新鲜度、运行状态、启动标识、
reset epoch 和容量检查，取消跨通道最新序号相等要求；40 项相关本地回归通过。
NX 候选构建的地图 4 项、SLAM/优化器 5 项、自主控制与动态障碍制动 2 项通过。
这些仍是组件测试；候选安装及安装后的静止和运动验收需单独记录。未发送运动指令。
真实回放的 43 帧配准缺口、ARM 性能、Product 仿真及监督实机验收仍待完成。
本轮详细证据保存在本地 `build/handoff/acceptance-20260916.md`。

## 2026-09-17 NX 候选安装与静止验收

已在 Go2 NX 安装 `v2.3.0-go2.20260916.32`，旧 `.31` 保留。
候选由 NX 源码镜像叠加本地改动构建，不是本地 HEAD 的干净 checkout；
部署身份应使用版本、源码覆盖记录和构建日志，不能仅看镜像 Git HEAD。
通过 ProductControl 启动 `map + camera`，使用候选自带 DDS probe 成功。

- doctor：20 通过、2 提示、0 失败。提示为建图未激活导航会话，以及 RSUSB 无 V4L2 节点。
- 首轮 122 秒、31 个样本：地图就绪全部正常，静止 XY 最大漂移 0.009 m；
  首次 capabilities 请求超过 3 秒，整轮失败。不能用后续成功覆盖这条失败记录。
- 后续 60 秒、16 个样本：严格检查通过，地图就绪始终正常，XY 最大漂移 0.0118 m。
  首次 capabilities 超时原因仍待定位；源码中首次生成 OpenAPI 是候选原因，尚未测证。
- 实机网页刷新后显示累计建图，快照时 63 个关键帧全部配准、0 丢帧、14,975 个预览点；
  静止测试尚无闭环。相机网页实际出图，彩色/深度约 28–29 帧/秒。
- 浏览器一次观察到关节数据过期；相机画面朝向天花板，Go2 相机外参尚未测量。
  这些问题须继续核对，不能把相机出图等同于深度融合可用。

保持静止、无控制者，未发送运动目标。尚未完成停车确认链路、闭环走行、
真实转换器保存重载、定位与监督导航验收，暂不合并主分支。

## 2026-09-17 累计图历史残影：实机证据

用户报告机器狗前方空旷，但累计图存在悬空点团。当前 session 的整图为
25,163 点、1,600 个关键帧，其中 4 帧配准拒绝、0 次闭环优化。
取机身前方 x=0.4–3.0 m、|y|<0.65 m、高于机身中心 z=0.8–1.5 m：
累计图有 164 点，当前扫描为 0，随后约 3 秒内 6 个不同时间戳扫描均为 0。
注意：Fast-LIO 的 `map_cloud` 此处是当前扫描转换到地图坐标后的结果，
不是独立的累计局部地图；此前称为“当前局部图”的表述不准确。
样本和比较脚本保存在 `build/handoff/cloud-ghost-20260917/`。
仅凭缺少回波不足以证明历史点应删除。进一步只取扫描与位姿时间一致的 3 帧，
以标定雷达原点检查历史点附近 6 cm 射线管、至少 30 cm 更远的回波：
164 点全部得到至少 2 帧自由空间证据，其中 135 点得到全部 3 帧支持，
没有相同射线管内更近回波的证据。结果见 `ray-evidence-synchronized-audit.json`。
这支持当前采样区域存在历史残影；分析使用预处理扫描及标定外参，仍不能据此
确定最初的物体类别或配准错误来源，也不证明规划器当前将该处判为障碍。

`OnlineMapping::State::snapshot` 合并历史关键帧并降采样，没有后续可见自由空间
对历史点的清除。静止时快照不更新可能由关键帧运动阈值决定，不能据此认定线程卡死。
另一个显示质量问题：顺序配准失败的关键帧从约束图和锚点中移除，但仍保留在
`frames`，累计预览会继续合入这些点。保留原始数据有价值，但不能把它们和
配准通过的数据混为相同质量；尚无逐帧归因证据说明本次残影来自那 4 个拒绝帧。
保存流程的 `RunSavedSourceCleanerJson` 已有 prune 动态清理；它不清理在线预览，
NX 上 `prune` 二进制存在，mapd 默认启用且要求清理成功，但尚未在本次地图上验收。
先保留原始关键帧，离线复现这些点的来源与射线证据；
采用已观测自由空间与多次观测清理，不按“单帧没出现”删除历史墙面，也不靠调色隐藏。
在线历史残影仍未修复，应作为本轮合并前地图质量问题追踪。

## 2026-09-17 地形、障碍与路径分层展示复核

四层展示尚未完成产品与实机验收。当前源码已接通最低观测高程、原生控制风险、
障碍点、膨胀占据采样、候选路径，以及全局/局部规划结果，但存在以下明确缺口：

- 地形层的值语义是 `min_observed_z_not_ground`，不能称为已经识别的地面。
- `localPlannerLayer.ts` 将障碍点、风险采样、候选和选中路径绑在同一个开关；
  默认高度切片只显示膨胀占据采样，其他诊断仅在三维模式出现。应由 Web 图层入口
  分别控制地形/风险、障碍、候选、规划结果，避免用户必须开启混合诊断。
- 诊断风险采样的显示高度固定为地图 Z=0.04 m；原生完整风险层已有独立坐标和
  投影高度处理。这两个表现不能混为一套正确的地形渲染。
- 选中候选通过 `CatmullRomCurve3` 再拟合，可能偏离后端原始折线；实际全局/局部
  路径的 `pathDisplay.ts` 已按原始线段绘制，应统一保留真实几何。

本轮 34 项高程、风险、局部诊断、坐标与路径时效测试通过。现有诊断测试还把
固定风险高度和切片隐藏候选当作预期，因此测试通过不代表上述产品要求已经满足。
维护责任：Web 的 SceneView/Scene3D 与诊断图层；地面分类语义仍属于原生 maps/terrain。
NX 转发连接被拒绝，未进行新一轮实机展示验证或部署，也未发送运动命令。
