# 建图档案恢复与 SLAM 模式切换

日期：2026-11-09。状态：原生档案恢复、重定位后续建、正式冷启动入口已实现；完整 Product 建图／导航热切尚未完成。

## 官方方法与本项目选择

- [slam_toolbox，7376b236](https://github.com/SteveMacenski/slam_toolbox/blob/7376b23689d47d0be2a1dd0c58f5eb7085ac8a55/README.md#lifelong-mapping) 保存扫描、位姿图及元数据；加载后先确认当前位置，再继续扩图。导航地图图片不能替代建图档案。
- [RTAB-Map，2a411a8d](https://github.com/introlab/rtabmap_ros/blob/2a411a8dfe15c88c5c8694cdfd8bfaa1dcea683a/rtabmap_slam/README.md#mapping-and-localization) 支持运行时切建图／定位。回到建图产生新的一段，通过回环与旧图建立关联，不能把一个模式开关等同于完整连续图恢复。
- LingTu 仍使用原生 C++/DDS、LTLIO 和已有 iSAM2。按上述方法保存可恢复的测量数据，不引入 ROS。原始里程计约束、已接受回环及其噪声重建同一张图；未接受的候选在新扫描到来后重新验证。
- 跨重启没有连续里程计，新段的首条连接使用已验证的地图配准结果和现有平移／旋转匹配 sigma；不使用精度更强的里程计噪声冒充这个缺口。sigma 是已有工程参数，不是标定的配准协方差。官方 LIO-SAM 来源及本地差异见 `src/localization/online_mapping/upstream/UPSTREAM.md`。

## 本次接通的代码

| 所有者 | 行为 |
| --- | --- |
| `online_mapping/archive.*` | 读取完整 patch/原始位姿/时间戳/回环测量；验证数据完整性、容量、标定和归属信息 |
| `online_mapping/backend.*`、`worker.*` | 在候选图中重建 iSAM2；成功后替换，失败保留原图；新段的连接明确标记为配准连接 |
| `slam/cpp/ltlio.cpp` | 保存坐标与标定；暂停建图记录；定位期间不再淘汰建图档案；恢复后先匹配再追加；旧图编号延续；档案时间与实时传感器时间分开 |
| `slam/cpp/cyclone_runtime.cpp`、`slam_control.cpp` | 运行模式改为实际提交状态；独立 SLAM 进程支持暂停、恢复、建图／定位模式请求；启动指定建图档案时自动匹配，成功后恢复建图 |
| `maps/cpp/build/pipeline.cpp` | 原始存档和正式地图保存事务保留 `mapping_state.yaml` |
| `lingtu/real`、`lingtu/sim`、`control_server.py` | `switch("map", map_name=..., initial_pose=...)` 接入原有 ProductControl；HTTP 转发保留同一语义；不激活一张固定导航地图冒充建图 |

`mapping_state.yaml` 保存已有建图 source epoch、地图与档案的坐标关系、body←IMU / IMU←LiDAR 标定以及完整性。source epoch 是私有档案归属证据，Product 对外仍只有 `product_session_id`。

加载另一份无关地图仍可用于固定地图定位，但不能直接恢复原建图图；需重新选择相应档案。传感器时间跳变已销毁上下文时，也不能把空图当成原图续建。

## 验证与证据

本地构建目录：`build/slam-core-windows-x64`，MSVC x64 Release。没有改动现场机器。

- 原生 Worker：22 个关键帧的闭环图恢复后，节点数、已接受回环数保持；锚定坐标与原图的差异小于 `1e-8 m`；恢复后可追加。损坏的回环索引被拒绝，原活动图仍可继续追加。
- 原生 SLAM：暂停、定位模式期间持续输入超过测试档案容量的扫描；回到建图仍保留原始帧。保存 A，重建后端，验证定位前不追加，匹配成功后追加新帧，再保存 B 并再次恢复。
- 独立进程：CTest 的 `messages_mapping_restart_capture` 写盘并退出；`messages_mapping_restart_resume` 在另一个进程读取并续建。恢复用的传感器时钟从 1 秒重新开始，档案时间延续而实时保存回执保持当前传感器时间。
- 正式 Python real/sim 事务回归：145 项通过；HTTP/Gateway 转发与请求恢复：22 项通过。这些是本地契约测试，不是 systemd 目标机运行验收。
- 地图保存事务 `lingtu_maps_save_map_test` 通过，确认恢复元数据与回环证据一起进入最终地图包，不被后台处理漏掉。
- 独立 `slamd` 加原生 DDS sensor publisher，限制为 loopback/domain 210。只发布合成场景与外部里程计；没有运行 navd 或 driver，也没有运动输出。恢复后关键帧由 4 增加，定位期间冻结，返回建图后继续增长；失败的地图加载保持旧模式。命令结果及运行 PID 见下方证据。

证据目录：`artifacts/localization/mapping-restore-20261109/`。

- `check_dds.py`：可重复的隔离验证，使用现有原生 LTU1/DDS 数据入口。
- `dds-result.json`：阶段状态、同一 PID、真实命令回执和耗时。
- `dds.log`：原生 SLAM 与 sensor publisher 日志。
- `fixture/a/`：小型三平面合成档案；只有合成点云，不是公开数据集或实采地图。

该 Windows 构建未包含 BBS3D，本次通过给定初值做匹配。不能据此声称未知位置的全局重定位、真实 IMU/LiDAR 估计精度、长时间运行资源占用或实机续建质量已通过。

## 正式使用入口

部署匹配版本的 SLAM、地图服务及 Python 代码后，原有入口可以恢复新格式档案：

```python
control.switch("map", map_name="office", initial_pose=(x, y, z, yaw))
```

```text
python -m lingtu.control switch map --robot <vendor/model> --env real --map office --initial-pose X Y Z YAW --json
```

`initial_pose` 是地图中的机身位置，航向为弧度；没有初值时依赖目标平台的全局重定位能力。自动定位成功之前，不向原图加入新关键帧。

HTTP 使用现有 `POST /api/v1/product-control/switch`，传 `product: "map"`、`map_name` 和当前 `expected_product_session_id`，可带 `initial_pose`。此处仍执行原有冷切换，不承诺保留其他服务 PID；Web 尚无新的续建按钮。

只有 PCD/OctoMap 或缺少完整恢复元数据的旧地图仍可用于导航，不能直接完整恢复建图。显式离线重新优化产生的新图也不能冒充原在线图的完整恢复；原始捕获档案保留供继续处理。

## 未完成的完整热切

- [x] 保存／恢复原在线图及其测量。
- [x] 同一 SLAM 对象暂停追加、定位后恢复追加。
- [x] 两个独立进程间的保存、匹配和续建回归。
- [x] 原生 DDS 下同一 slamd PID 的模式往返。
- [x] real/sim ProductControl 冷启动恢复与 HTTP 接线。
- [ ] ProductControl 在离开建图前自动冻结、保存本次成果，等待现有后台队列正式提交。
- [ ] slamd/mapd/navd/Host 在线接受同一个目标 Product 会话，完成失败补偿和停止确认。
- [ ] 将定位地图索引准备放到后台并验证真实传感器连续处理；当前独立 SLAM 的地图加载仍有同步工作，PID 不变不代表没有处理间隔。
- [ ] standard/camera 对应 Host 共同装配和 Web 一次操作完整交接。
- [ ] 全部服务的热切 PID/位姿连续性、目标机无运动及受监督运动验收。

`map.yaml`、`nav.yaml` 的 `online_hot_switch_supported` 仍为 false。独立 SLAM 的模式请求不会绕过受管理 Product：受管理运行仍要求 ProductControl 切换。
