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
