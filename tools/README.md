# tools/ — 开发、调试、离线分析工具

与 `scripts/` 的分工：

| 目录 | 用途 |
|------|------|
| **`scripts/`** | 部署、OTA、构建、机器人端 `lingtu` 运维 CLI、运行时验收 gate |
| **`tools/`** | 开发/调试/标定/离线分析脚本（不上 systemd，不进 OTA 主路径） |
| **`tests/scripts/`** | pytest 脚本测试和人工硬件/ROS 烟测 |

## 布局

| 路径 | 用途 |
|------|------|
| `validate/` | 配置、topic 契约与架构边界校验（CI / 发版前手动跑） |
| `evaluation/` | 退化/数据集评测脚本 |
| `perception/` | 感知 demo、BPU 导出、ONNX→HBM、图片回放、E2E 感知检查 |
| `reconstruction/` | 离线 3D 重建和数据集回放辅助 |
| `navigation/` | 目标点/里程计/导航辅助小工具 |
| `gateway/` | Gateway demo viewer 与系统健康小服务 |
| `proto_gen/` | 生成的 Protobuf Python 桩代码（由 `scripts/proto/proto_gen.sh` 刷新） |

## 常用命令

```bash
# robot_config.yaml 结构校验
python tools/validate/validate_config.py

# /nav/* topic 与 topic_contract.yaml 对齐
python tools/validate/validate_topics.py

# Module-First 包边界（含函数内 lazy import）
python tools/validate/validate_architecture_boundaries.py

# S100P 感知 E2E（需先 scp 到 /opt/nav/tools/）
bash tools/perception/test_perception_e2e.sh

# YOLO-World → ONNX → BPU
python tools/perception/export_yoloworld_bpu.py
bash tools/perception/convert_onnx_to_hbm.sh <onnx_path>

# 离线 3D 重建
python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/<id>
```

真机部署时部分脚本会同步到 `/opt/nav/tools/`（见 `tools/perception/test_perception_e2e.sh` 注释）。
