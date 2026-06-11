# scripts/ — LingTu 运维工具集

## 日常操作速查

推荐使用 `python lingtu.py`（主入口），以下脚本用于部署、运维、调试等场景。

| 我想… | 命令 |
|-------|------|
| 启动导航系统 | `python lingtu.py nav` |
| 环境自检 | `python lingtu.py doctor` |
| Shell 兼容入口 | `./scripts/lingtu.sh map / save / nav / status` |
| 部署到 S100P | `bash scripts/deploy_s100p.sh` |
| 健康检查 | 机器人端 `scripts/lingtu health`，或 `python lingtu.py health` |
| 跑全量测试 | `bash scripts/run_tests.sh`（`--quick` 跳过 ros2/sim） |
| OTA 打包上机 | `bash scripts/ota/build_nav_package.sh && bash scripts/ota/deploy_to_robot.sh` |
| 生成 Protobuf | `bash scripts/proto/proto_gen.sh` |
| Rerun 3D 可视化 | `python lingtu.py rerun` |

## 根目录脚本

### 核心入口

| 脚本 | 用途 |
|------|------|
| `lingtu` | **机器人端统一运维 CLI**（status/watch/map/nav/svc/log/health） |
| `lingtu.sh` | Shell 兼容入口；委托 `python lingtu.py` |
| `deploy_s100p.sh` | S100P 一键部署 |
| `run_tests.sh` | CI 测试运行器（fast-first 分层） |
| `doctor.py` | `lingtu doctor` 环境/依赖自检 |

### 构建

| 脚本 | 用途 |
|------|------|
| `build_nav_core.sh` | C++ nav_core 独立构建（`make nav-core` 调用） |
| `build_dufomap.sh` | aarch64 DUFOMap 幂等构建 |
| `clone_semantic_deps.sh` | 克隆 VLN 第三方仓库到 `third_party/`（grounding_dino 后端需要） |

### 运行时验证与 Gate（被测试/CLI 引用）

| 脚本 | 用途 |
|------|------|
| `plan_preview.py` | 规划预览工具（`test_plan_preview_tool.py`） |
| `saved_map_artifact_gate.py` | 保存地图工件 Gate（`cli/main.py` 调用） |
| `real_runtime_evidence_collect.py` / `real_runtime_evidence_gate.py` | 真机运行时证据采集 + Gate |
| `runtime_contract_audit.py` | 运行时契约审计 |
| `soak.py` | 长稳测试（机器人端 `lingtu` 调用） |
| `static_localization_probe.py` | 静置定位漂移探针（机器人端 `lingtu` 调用） |

### 标定与诊断

| 脚本 | 用途 |
|------|------|
| `run_allan_variance.sh` | Mid-360 IMU Allan Variance 一键标定（record/analyze/apply） |
| `record_bag.sh` | ros2 bag 录制（Gateway operations 路由调用） |
| `dufomap_offline_test.py` | 对已有地图离线跑 DUFOMap 验证 |
| `live_detect.py` | 实时目标检测（30fps Web 预览） |
| `live_track.py` | 实时追踪 + CLIP 选人 |
| `rerun_live.py` | `lingtu rerun` Rerun 3D 实时可视化 |
| `run_rerun_mapping.py` | 建图 + Rerun 可视化流程 |

### 其他工具

| 脚本 | 用途 |
|------|------|
| `extract_api_docs.py` | 从代码生成 API 文档（`docs/api/`） |
| `scaffold_robot.py` | 新机器人配置脚手架（sim engine 引用） |
| `install_go2rtc.sh` | go2rtc 视频流服务安装（WebRTC 备选） |

### 手动冒烟脚本（S100P 上手动跑）

| 脚本 | 用途 |
|------|------|
| `test_mapping.py` | 建图链路冒烟（SLAM + LiDAR 数据流 + 存图） |
| `test_nav_planning.py` | 真实 tomogram 规划冒烟 |
| `test_s100p_start.py` | S100P 全栈启动验证 |
| `test_mcp_full.py` | MCP Server 全量验证 |

---

## 子目录

### deploy/ — 部署与服务管理

S100P 真机部署相关的脚本和 systemd 服务文件。Python 依赖统一走 `pip install -r requirements.txt`。

| 脚本 | 用途 |
|------|------|
| `setup_server_ros_pct.sh` | 服务器 ROS2 + PCT 规划器环境一键搭建 |
| `setup_network.sh` | LiDAR 以太网 + Orbbec udev 配置（新机器初始化） |
| `cut_release.sh` | sunrise 上切版本 (/opt/lingtu/current 原子切换 + 回滚) |
| `sync_versions.sh` | VERSION → package.xml / robot_config 版本同步 (`make sync-version`) |
| `sync_sunrise.ps1` | Windows → sunrise 代码同步 (SSH 反向隧道 git pull) |

**s100p/ — systemd 服务：**

| 文件 | 服务 |
|------|------|
| `install_services.sh` | 安装全部 systemd 服务 |
| `lidar.service` | Livox MID-360 驱动 |
| `slam.service` | Fast-LIO2 SLAM |
| `slam_pgo.service` | Pose Graph Optimization |
| `localizer.service` | ICP 定位 |
| `super_lio.service` | Super-LIO 外部 LIO 后端 |
| `super_lio_relocation.service` | Super-LIO 重定位 |

### build/ — 构建脚本

| 脚本 | 用途 |
|------|------|
| `build_ros_workspace.sh` | ROS2 colcon 工作空间构建 |
| `build_tare.sh` | TARE 探索规划器构建 |
| `fetch_ortools.sh` / `build_ortools_from_source.sh` | OR-Tools 获取/源码构建（TARE 依赖） |

### ota/ — OTA 远程更新

打包导航软件为 OTA 包，推送到机器人并安装。

| 脚本 | 用途 |
|------|------|
| `build_nav_package.sh` | 打包导航软件包 |
| `push_to_robot.sh` | 推送到机器人 |
| `deploy_to_robot.sh` | 部署到机器人 (push + install) |
| `install_nav.sh` | 机器人端安装脚本 |
| `setup_robot.sh` | 机器人初始化 (首次部署) |
| `generate_manifest.py` / `manifest_template.json` | OTA manifest 生成 |
| `start_mapping.sh` / `start_nav.sh` | 机器人端启动建图/导航 |
| `mapping.service` / `navigation.service` | systemd 服务 |

### monitor/ — 远程监控

飞书 / Telegram 机器人，实时推送机器人状态告警。详见 `monitor/README.md`。

### proto/ — Protobuf 代码生成

| 文件 | 用途 |
|------|------|
| `proto_gen.sh` / `proto_gen.ps1` | 从 `shared/proto/` 生成 Python/Dart gRPC 代码 (Linux / Windows) |

### manager/ — Web 管理端

| 文件 | 用途 |
|------|------|
| `manager.py` | 轻量 Web 管理服务，由 `lingtu-manager.service` 管理 |

---

**开发/校验工具**见仓库根目录 [`tools/README.md`](../tools/README.md)（`validate/`、BPU 导出、感知 demo 等）。**L3 实机记录**见 [`docs/07-testing/field-runs/`](../docs/07-testing/field-runs/README.md)。
