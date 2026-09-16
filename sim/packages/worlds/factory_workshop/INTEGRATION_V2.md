# Factory 2.0 主环境接入验收 — 2026-09-08

结论：**真实场景替换与正式启动已完成，运动验收未通过。**
没有用静态模型预览代替机器人运行，也没有把真值定位当成 SLAM 验收。

## 已接入

- 主工作区：`D:/inovxio/brain/lingtu`。
- 场景：`factory_workshop@2.0.0`，从 `v8-r4-detail-study` 原生导出提升。
  50 组网格/贴图共约 1.02 GB，7,118 个接触体不变。视觉屋面从保留给机器人
  隐藏碰撞体的 group 3 移到 group 2；正式雷达读取隐藏的世界碰撞 group 4。
- Thunder 的 `default.yaml`、`nav.yaml` 已选择新版；出生点 `(61,16.5,0.02)`，
  朝向 +Y。保留 `teleop_avoid` 专用场和 tracking 行人场，不删除工业园。
- 地图：`C:/Users/99563/data/lingtu/maps/factory_workshop_v2/`。
  同源几何 PCD 为 6,684,610 点，native MapPipeline 构建 0.1 m OctoMap 成功，
  报告 8,145,599 occupied voxels。来源明确为 synthetic geometry，`slam_source=none`。
  本机地图是运行资产；其他机器需要按 README 重新导出、构建，不能只复制 Session YAML。

## 正式运行证据

Product session：`product-3e881b1157b34649aac68126c08ff6ec`。

使用 ProductControl 公共接口，`nav + scan + mujoco`，显式
`env_config.localization=truth`。现有普通 CLI 的 Fast-LIO2 默认没有改变。

- ProductControl 完成 map prepared / staged / verified、localization initialized、
  committed，返回 `active`。Driver、IMU、LiDAR、Mapd、Traversability、Navd、
  feeder 和 Host 的就绪检查均通过。
- 实际窗口标题：`MuJoCo: click navigation`。模型和传感器进入真实物理循环，
  使用当前主工作区 `policy_4998.onnx`、正式站姿；本轮没有替换控制策略。
- 连续运行约 300 秒，60,014 个姿态采样。IMU 60,012 次发布，平均约 200 Hz。
  LiDAR 3,000 次调度，2,582 次发布、418 次丢弃，约 8.60 Hz；不能据此宣称全链路流畅。
- Mapd 记录至少 1,932 次 processed observations，Navd 获得 map-frame 真值定位及碰撞地图。
- 随后通过 ProductControl 停止全部本次子进程。终态 `zero_applied`、
  `terminal_ack=true`，本次未强杀机器人进程，当前未继续留着机器人漂移。

原始证据位于仓库 `artifacts/factory_workshop/v2-main-upgrade/`：
`product-start-cold-budget.log`、`run-plan.json`、`ready-nav-status.json`、
`ready-mapd-status.json`、`final-motion.json`、`final-sensors.json`、
`final-feeder-live.json`、`product-stop.json`、`final-stop.json`。

## 本次修好的接线问题

1. 场景合并机器人后，资源引用仍相对旧目录。现在按场景自身 compiler/resource
   目录解析，合并时使用绝对路径，机器人自己的 meshdir 不变。
2. 正式 feeder 快照只复制世界 XML、没有模型和贴图。现在将它直接引用的资源一起
   纳入快照；回归测试删除原始资源后，仍能从快照加载场景及机器人。
3. 主工作区 Mapd 二进制旧于当前碰撞消息定义，Navd 日志明确报反序列化失败。
   仅从当前工作区重建 Mapd，未修改 IDL 或算法；构建存在原有 double→float C4244 警告。
4. 大场景冷启动约 40 秒，Traversability 的 30 秒启动预算先行过期。
   sim env 中将其对齐为其他导航消费者已有的 60 秒；不改变运动安全超时或碰撞阈值。

定向回归：**170 passed**（安装、同源地图、默认 Session、正式快照与场景合并、
feeder 生命周期、Catalog）；本轮修改的 Python 文件 Ruff 通过，相关 diff whitespace
检查通过。另有 Product 编译定向检查通过；未运行全仓测试或实机验收。

## 仍未通过，不能隐去

- **零命令漂移**：同一 session 的最终运动证据为非零命令 0 次，但 live 真值位置
  从出生 XY `(61,16.5)` 到 `(60.43999,4.88344)`，端点偏移约 **11.63 m**。
  原 `motion_evidence` 的路程只记录受命令运动，因此其 0 m 不能表示机器人未移动。
  需要独立对照新场景/平地的零命令策略与接触行为，当前证据不能断言是哪一个造成。
- **目标导航没有成功**：运行中的目标记录报
  `driver_control_lost:driver_control_command_timeout`；只读短路预览返回
  `navigation_busy`。没有交付成功到达的轨迹，不能宣称完成平地导航验收。
- **调度与显示还有开销**：IMU 平均频率达标，但记录过约 4.05 秒最大调度迟到；
  单帧 viewer 样本约 17–65 ms，不是 p95 统计。需要区分渲染开销、控制停顿和地图规划开销。
- **接触统计有语义欠缺**：当前 entity contact 计数把 `fw_r2_site_paving_road0`
  这种正常地面接触也算进去；本次不能用它判断实体碰撞通过/失败。
- 二楼楼梯运动、跨层规划、Fast-LIO2 和实机验证均未通过本次验收；视觉仍非照片级。

后续顺序：零命令稳定性 → 平地短目标与驱动超时 → 渲染/传感器连续性 → 长程与楼梯。
不通过更换为运动学模式、放宽安全门槛或静默切换 CMU 来隐藏问题。

本轮没有提交或推送，也没有修改 SCAN/CMU 数学、策略文件、机器人 MJCF 或 `research/`。
