# config/ �?LingTu 配置文件

`config/` 是可提交、跨环境共享的默认配置与运行契约目录。修改这里的文件会影响真机、仿真、开发和打包流程；本地输出、一次性证据、缓存和 secrets 不应放在这里�?

## 配置文件索引

### 核心配置与运行契�?

| 文件 | 用�?| 关键字段 |
|------|------|---------|
| `robot_config.yaml` | 机器人物理参数和传感器配�?| 相机内参/外参、LiDAR 外参、运动控制增益、端口、标定参�?|
| `devices.yaml` | 硬件设备注册�?| 相机、LiDAR、IMU、GNSS、控制器设备声明 |
| `endpoints.yaml` | endpoint / transport 默认配置 | 本地/真机 endpoint、设备配置路径、传输选择 |
| `topic_contract.yaml` | Canonical runtime stream/topic 合约 | ModulePort/Gateway/endpoint adapter 共享的标�?stream token；ROS2 只是可�?adapter 传输 |
| `thunder_lite_package.yaml` | Thunder Lite 打包契约 | include/exclude 路径、lite 依赖边界、runtime 预期规格 |

### 导航、SLAM 与地�?

| 文件 | 用�?| 关键字段 |
|------|------|---------|
| `far_planner.yaml` | 远距离全局规划参数 | 搜索半径、路径平滑、采样密�?|
| `pointlio.yaml` | Point-LIO SLAM 参数 | IMU 噪声 (na/ng/nba/nbg)、LiDAR-IMU 外参、分辨率 |
| `dufomap.toml` | DUFOMap / 地图后端配置 | 地图分辨率、更新范围、持久化参数 |
| `qos_profiles.yaml` | ROS2 QoS 配置 | 各话题的 reliability/durability/depth |

### 语义导航

| 文件 | 用�?| 关键字段 |
|------|------|---------|
| `decision.yaml` | 语义规划器配�?| LLM backend (kimi/openai/claude/qwen)、目标解析阈值、agent_loop 参数 |
| `perception.yaml` | 感知模块配置 | 检测器 backend (bpu/yoloe)、CLIP 编码器、置信度阈�?|
| `semantic_exploration.yaml` | 探索模式覆盖 | frontier 评分权重、探索半径、信息增益参�?|
| `semantic_scoring.yaml` | 语义目标/路径评分 | 可通行性、目标置信度、上下文权重 |

### 部署与传�?

| 文件 | 用�?|
|------|------|
| `go2rtc.yaml` | go2rtc WebRTC/WHEP 摄像头流模板 |
| `cyclonedds.xml` | CycloneDDS 配置 (S100P 默认 DDS 实现) |
| `fastdds_no_shm.xml` | FastDDS 禁用共享内存模式 (Docker 兼容) |

## robot_config.yaml 关键段落

这是最常修改的文件，以下是各段落说明：

### 相机

```yaml
camera:
  position_x/y/z:     # 相机在机体坐标系的安装位�?(m)
  roll/pitch/yaw:      # 相机光轴旋转 (rad)
  fx/fy/cx/cy:         # 内参 (会被 ROS2 CameraInfo 覆盖)
  width/height:        # 分辨�?
  depth_scale:         # 深度值转米的系数
  rotate:              # 安装旋转补偿 (0/90/180/270 �?
  dist_k1..k3/p1/p2:  # 畸变系数
```

### LiDAR

```yaml
lidar:
  offset_x/y/z:       # LiDAR-IMU 平移外参 (m)
  roll/pitch/yaw:      # LiDAR-IMU 旋转外参 (rad)
```

### 服务端口

```yaml
gateway:
  port: 5050           # Dashboard HTTP/WS/SSE
  mcp_port: 8090       # MCP JSON-RPC (AI agent 接口)
brainstem:
  host: "127.0.0.1"
  port: 13145          # Brainstem gRPC (运动控制)
```

### 运动控制

```yaml
control:
  yaw_rate_gain:       # 偏航角速度增益
  max_yaw_rate:        # 最大偏航角速度 (deg/s)
  max_accel:           # 最大加速度 (m/s^2)
```

## 标定参数写入流程

传感器标定结果通过 `calibration/apply_calibration.py` 自动写入 `robot_config.yaml`�?

1. 相机内参 �?`camera.fx/fy/cx/cy + dist_*`
2. IMU 噪声 �?`pointlio.yaml` �?`na/ng/nba/nbg`
3. LiDAR-IMU 外参 �?`lidar.offset_* + roll/pitch/yaw`
4. 相机-LiDAR 外参 �?`camera.position_* + roll/pitch/yaw`

验证：`python calibration/verify.py`

## LLM 配置

语义导航使用�?LLM 通过环境变量 + `decision.yaml` 配置�?

```bash
# 环境变量 (API Key)
export MOONSHOT_API_KEY="..."           # Kimi (默认)
export DASHSCOPE_API_KEY="..."          # Qwen (备�?
export OPENAI_API_KEY="sk-..."          # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..."   # Claude
```

```yaml
# decision.yaml 选择 backend
llm:
  backend: "kimi"
  model: "kimi-k2.5"
```

## 修改建议

- 修改后运�?`python lingtu.py doctor` 自检
- 标定参数不要手改，用 `calibration/apply_calibration.py`
- `topic_contract.yaml` �?`thunder_lite_package.yaml` 是运�?打包契约，修改前确认影响范围；不要把 canonical topic token 等同�?ROS2-only 通信边界
- DDS 配置只在跨机通信�?Docker 部署时需要调�?
- API keys、tokens、passwords、现场私�?IP 覆盖、一次性日志和 runtime evidence 不进 `config/`；用环境变量、部署环境文件或 ignored `artifacts/` 输出承载
