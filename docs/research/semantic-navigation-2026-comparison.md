# 2026 语义导航对标与 LingTu 实现路线

**核对日期：** 2026-10-06  
**结论性质：** 论文和作者项目页的外部证据；不代表 LingTu 已经复现这些结果。

用户现已选择 **VL-Nav 为主复现目标**。当前代码、服务器进度和未接通环节统一
记录在 [VL-Nav 复现记录](vlnav.md)，本页保留三篇工作的对照。

三篇原文已下载到本地论文目录，并用 `pdfinfo` 验证可读：

- [SysNav PDF](papers/sysnav.pdf)
- [VL-Nav PDF](papers/vl-nav.pdf)
- [From Language to Navigation Goals PDF](papers/language-to-navigation-goals.pdf)

## 三个项目验证在哪里

| 项目 | 仿真平台与数据 | 真实平台 | 语义与规划边界 | 对 LingTu 的用途 |
| --- | --- | --- | --- | --- |
| [SysNav](https://arxiv.org/abs/2603.06914)（2026） | HM3D-v1、HM3D-v2、MP3D、HM3D-OVON；作者项目页报告四个仿真基准 | 定制轮式机器人、Unitree Go2、Unitree G1；作者项目页还报告 15 个真实场景和 4 个 floor-scale 场景 | VLM 做房间级语义决策；房内使用经典探索；各本体执行自己的 waypoint 控制 | 作为主系统架构参考：高层语义、中层原生导航、低层机器人控制分开 |
| [VL-Nav](https://sairlab.org/vlnav/)（IROS 2026） | DARPA TIAMAT Phase 1；室内用 HabitatSim，室外用 IsaacSim | Rover + Jetson Orin NX；另有 Unitree G1 和 Unitree Go2 多楼层演示 | VLM/开放词汇检测生成目标点；动态占据图生成 frontier；神经符号评分选择探索点；几何规划执行 | 作为复杂任务、目标关系和长距离探索参考 |
| [From Language to Navigation Goals](https://arxiv.org/abs/2607.13624)（2026 预印本） | Gazebo + TurtleBot3 Waffle，完成导航闭环 | Go2 + RealSense 验证语义识别与目标定位，不能算实机完整自主导航 | ROS 2 组件把自然语言目标转成 RGB-D 目标位置，再送入 Nav2 | 作为文本 → RGB-D 目标 → Nav2 goal 的集成参考；不能代表大范围语义导航 |

## 重点差异

### SysNav：适合作为 LingTu 的主架构

SysNav 把 VLM 限定在房间级决策，不让模型持续输出低层速度。作者项目页还给出了
“下一房间、提前结束、目标核验”三类查询，并报告 VLM 结果预取以避免机器人停在
原地等待。这和 LingTu 的边界一致：语义层提交候选，原生 3D planner 和运动控制
拥有最终权限。

### VL-Nav：适合作为大范围搜索算法参考

VL-Nav 的官方页面明确写出输入包含 prompt、RGB 图像、位姿和 LiDAR；地图模块维护
动态占据图，frontier 与开放词汇检测得到的实例点合并成候选池，再由神经符号评分
选择探索目标。它在 HabitatSim 室内、IsaacSim 室外和 Go2/G1 实机上展示了多楼层与
长距离任务。这个机制最适合补 LingTu 当前仍缺少的“整栋楼找目标”环节。

### From Language to Navigation Goals：适合作为端到端连接样例

这篇工作更像一个 ROS 2 集成基线：识别语言中的物体，用 RGB-D 估计位置，再调用
Nav2。它有 Go2 实机，但论文目前是 2026 年 7 月提交的 arXiv 预印本，且页面写明
代码将在接收后公开；因此不能把它当成我们当前可以直接复现的完整开源栈。

## LingTu 的实现映射

```text
HabitatSim / 录制传感器
  -> ObservationFrame {rgb, depth, lidar?, pose, calibration, timestamp}
  -> VLM / open-vocabulary detector
  -> InstanceRecord {instance_id, label, geometry_ref, evidence_refs, confidence}
  -> RoomGraph / ObjectGraph
  -> GoalCandidate {geometry_ref, relation, map_identity}
  -> native preview_plan
  -> native goal admission
  -> local planner / driver
```

职责必须保持如下边界：

- VLM 产生语义证据、候选标签和关系假设，不产生最终速度命令；
- `object_memory` 保存地图绑定的实例和证据引用，不复制一份占据图；
- 房间图和对象图可以帮助选目标，但碰撞、支撑、扫掠和可执行路径仍由原生 3D
  导航链确认；
- frontier 由地图/探索所有者提供，Python 语义层只提供目标相关性评分；
- Go2、其他四足和轮式平台继续通过 `RobotConfig` 与适配器选择，不在语义模块中
  写死某一个本体。

## 首个可验证切片

当前最小目标不是立刻复制三篇论文，而是完成一个可追溯的 HabitatSim 数据切片：

1. 使用 HM3D 或 HM3D-OVON 中明确存在的目标类别；
2. 生成 RGB、depth、semantic label、pose 和 calibration 记录；
3. 构建房间节点和目标实例节点，保存 `geometry_ref`；
4. 用文本查询得到目标候选；
5. 将候选送入 LingTu 原生 planner 做只读预检；
6. 记录 `run.json`、`instances.json`、`query.json`、`planner_result.json` 和
   真实仿真轨迹视频；
7. 统计目标检索成功率、规划接受率、路径长度、探索步数和失败原因。

只有这条切片跑通后，才进入多房间 frontier 探索，再进入 Go2 无运动回放，最后才
进行有人监督的实机运动。当前仓库有 Habitat/VLMaps 适配器和 ReplicaCAD 几何转换，
但还没有这一套完整的 HabitatSim 运行产物，因此不能声称已经复现 SysNav 或 VL-Nav。

## 本机运行前置检查

2026-10-06 在项目 `.venv` 和默认 Conda 环境中检查了 `habitat_sim`、`habitat`
和 `gazebo`：三个模块均未安装。该检查只确认运行时缺口，不自动安装依赖；仓库
当前也没有保存过 HabitatSim episode、轨迹视频或指标文件。下一步应先确定受支持的
Linux/CUDA 运行环境和数据集许可，再建立单独的 HabitatSim 复现实验环境，避免把
上游研究代码直接混入 LingTu 产品运行时。
