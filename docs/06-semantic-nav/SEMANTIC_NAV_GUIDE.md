# NaviMind 语义导航 — 启动指南

## 系统架构

```
Camera RGB-D ──→ SemanticPerceptionNode ──→ /nav/semantic/scene_graph (1Hz)
                   │ YOLO-World 检测                    │
                   │ CLIP 编码                          ▼
                   │ TF2 投影 (camera→map)    SemanticPlannerNode
                   │ InstanceTracker BA-HSG     │ GoalResolver (Fast/Slow)
                   │                            │ TaskDecomposer
                   ▼                            │ VoI Scheduler
              /nav/semantic/detections_3d        ▼
                                         Nav2 NavigateToPose
                                                │
gRPC Gateway ←─── /nav/semantic/status          ▼
Flutter App ←──── TaskManager             local_planner → /cmd_vel → 电机
```

## 快速启动 (3 步)

### 步骤 1: 环境配置

```bash
cd /opt/nav    # 或你的项目目录

# 检查环境 (不安装, 仅查看缺什么)
bash scripts/setup_semantic.sh --check

# 一键安装所有依赖 + 预下载模型
bash scripts/setup_semantic.sh
```

### 步骤 2: 设置 LLM API Key

```bash
# 推荐: Kimi (国内直连, 无需 VPN, 便宜)
# 申请地址: https://platform.moonshot.cn/
export MOONSHOT_API_KEY=sk-xxxxxxxxxxxxxxxx

# 或者: OpenAI
# export OPENAI_API_KEY=sk-xxxxxxxxxxxxxxxx

# 或者: 阿里 Qwen (备选)
# export DASHSCOPE_API_KEY=sk-xxxxxxxxxxxxxxxx

# 建议写入 ~/.bashrc 持久化
echo 'export MOONSHOT_API_KEY=sk-xxx' >> ~/.bashrc
```

### 步骤 3: 启动

```bash
# 方式 A: 随主导航一起启动 (推荐)
source install/setup.bash
ros2 launch navigation_run.launch.py enable_semantic:=true

# 方式 B: 单独启动语义子系统 (调试用)
source install/setup.bash
ros2 launch launch/subsystems/semantic.launch.py

# 方式 C: systemd 服务 (部署用)
sudo systemctl enable nav-semantic
sudo systemctl start nav-semantic
```

## 发送指令

### 方式 1: Flutter App (生产环境)

Flutter 客户端 → 语义导航面板 → 输入指令 → 自动发送

### 方式 2: gRPC 直接调用

端口 50051 是 gRPC Gateway 的端口 (OTA 在 50052, 不冲突)。

```bash
# 通过 grpcurl 发送 (支持口语化自然语言!)
grpcurl -plaintext -d '{
  "task_type": 6,
  "semantic_nav_params": {
    "instruction": "看一下灭火器在哪",
    "language": "zh",
    "explore_if_unknown": true,
    "timeout_sec": 300,
    "arrival_radius": 1.0
  }
}' localhost:50051 robot.v1.ControlService/StartTask
```

指令支持口语化表达, 以下说法都能正确解析:

| 说法 | 提取目标 |
|------|----------|
| "找灭火器" | 灭火器 |
| "看一下灭火器在哪" | 灭火器 |
| "灭火器在哪里" | 灭火器 |
| "帮我找一下门" | 门 |
| "哪里有打印机" | 打印机 |
| "带我去会议室" | 会议室 |
| "where is the fire extinguisher" | fire extinguisher |

### 方式 3: ROS2 Topic (调试用)

```bash
# 发送中文指令
ros2 topic pub --once /nav/semantic/instruction std_msgs/String \
  '{"data": "{\"instruction\": \"找灭火器\", \"language\": \"zh\"}"}'

# 发送英文指令
ros2 topic pub --once /nav/semantic/instruction std_msgs/String \
  '{"data": "{\"instruction\": \"find the fire extinguisher\", \"language\": \"en\"}"}'

# 查看结果
ros2 topic echo /nav/semantic/resolved_goal
ros2 topic echo /nav/semantic/status
```

## 话题列表

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/nav/semantic/instruction` | String (JSON) | 输入 | 用户指令 |
| `/nav/semantic/scene_graph` | String (JSON) | 内部 | BA-HSG 场景图 (1Hz) |
| `/nav/semantic/detections_3d` | String (JSON) | 内部 | 3D 检测结果 |
| `/nav/semantic/resolved_goal` | PoseStamped | 输出 | 解析后的目标 (map frame) |
| `/nav/semantic/status` | String (JSON) | 输出 | 任务状态 |
| `/nav/semantic/cancel` | Empty | 输入 | 取消当前任务 |
| `/nav/semantic/query` | QueryScene (srv) | 服务 | 查询场景图 |

## 配置文件

| 文件 | 说明 |
|------|------|
| `config/semantic_perception.yaml` | 检测器、CLIP、场景图参数 |
| `config/semantic_planner.yaml` | LLM、融合权重、探索策略、Nav2 |

### 关键参数

```yaml
# 切换 LLM 后端
llm:
  backend: "kimi"              # kimi | openai | claude | qwen
  model: "kimi-k2.5"          # 或 moonshot-v1-8k / gpt-4o-mini / qwen-turbo

# 切换检测器
detector_type: "yolo_world"    # yolo_world (推荐) | grounding_dino

# 调整 Fast Path 阈值 (越高→越多走 Slow Path LLM)
fast_path_threshold: 0.75

# Jetson 性能优化
yolo_world:
  model_size: "l"              # s=最快, l=平衡, x=最精确
  tensorrt: true               # 首次编译慢, 之后快 3x
clip:
  model: "ViT-B/32"           # B/32=省内存, L/14=更准
```

## 验证

### 离线测试 (无需 ROS2/相机)

```bash
cd 3d_NAV

# 运行 73 个单元测试
python -m pytest tests/test_offline_pipeline.py -v

# 生成论文指标报告
python tests/test_offline_pipeline.py --report
```

### 在线测试 (需要 ROS2 + 相机)

```bash
# 1. 确认话题在线
ros2 topic list | grep semantic

# 2. 确认场景图在发布
ros2 topic echo /nav/semantic/scene_graph --once

# 3. 发送测试指令
ros2 topic pub --once /nav/semantic/instruction std_msgs/String \
  '{"data": "{\"instruction\": \"find the door\"}"}'

# 4. 查看是否输出目标
ros2 topic echo /nav/semantic/resolved_goal
```

## 常见问题

### Q: "YOLO-World 下载失败"
在 Jetson 上可能需要代理。手动下载 `yolov8l-worldv2.pt` 放到 `~/.ultralytics/` 下。

### Q: "CLIP 模型太大, Jetson 内存不够"
改用小模型: `clip.model: "ViT-B/32"` (默认, 仅 ~0.4GB)。

### Q: "LLM API 调用超时"
1. 检查网络连接和 API Key
2. 切换后端: Kimi (国内直连) 或 Qwen (阿里云)
3. 大部分指令走 Fast Path (无需 LLM, <5ms)

### Q: "TF transform 报错"
确保 SLAM 已启动并正在发布 `map→odom→body` TF 链。

### Q: "colcon build 报错 semantic_perception"
该包使用混合构建 (Python + srv), 确保 `rosidl_default_generators` 已安装:
```bash
sudo apt install ros-humble-rosidl-default-generators
```

## 编译

```bash
# 首次编译
cd /opt/nav
colcon build --packages-select semantic_perception semantic_planner --symlink-install

# 编译后 source
source install/setup.bash
```
