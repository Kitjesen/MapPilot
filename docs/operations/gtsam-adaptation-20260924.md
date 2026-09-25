# GTSAM 重力对齐后端适配

历史阶段 `.70`；当前 `.71` 已切换 [LIO-SAM 原生后端](lio-sam-migration-20260924.md)。
下文的固定重力 batch LM 仍供旧图离线工具使用，不再是当前在线建图后端。

## 采用的边界

保留 Fast-LIO2 前端、LingTu 原生 DDS、扫描配准及回环验收；对已有独立重力参考
的建图与保存图，改用 GTSAM 批量 LM。不是将 ROS 节点直接搬入产品，也不是
FAST-LIO-SAM 的完整移植。未引入未经标定的里程计噪声或回环权重。

参考：

- [FAST-LIO-SAM](https://github.com/engcang/FAST-LIO-SAM)：参考 Fast-LIO 前端与
  GTSAM 后端的分工及连续观测连接方式。上游实际使用 Pose3/iSAM2；本次不宣称
  与它算法或参数完全一致。
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/loop_fusion/src/pose_graph.cpp)：
  参考 IMU 模式的四自由度变量选择；没有复制视觉回环代码、残差权重或 ROS 接口。
- [GTSAM](https://github.com/borglab/gtsam)：使用 NX 已安装的 4.0.2 C++ 库。

## 实际修改

新增 src/localization/opt/gravity_graph.cpp：以原始 LIO 姿态为参考，变量为
world yaw correction 与 XYZ，R=Rz(delta_yaw)*R_reference。横滚/俯仰在整个
求解过程中保留，不通过增大软先验权重近似，也不在求解后只把姿态改回去。
XYZ 均可变化，允许真实坡道、台阶与机身姿态。

保留现有 SE(3) Log 测量残差与完整信息矩阵；对秩四信息做特征分解白化，不给
零空间添加虚构权重。固定首节点从变量集合移除，包含该节点的边成为一元因子。
其余边是四维状态的二元因子，局部 Jacobian 使用中心差分。模型接线由 LingTu
适配，线性化图与非线性求解由 GTSAM 承担。

graph.cpp 在共用输入/连通性验收后，按是否具有独立重力参考进入对应求解器：
有参考走 GTSAM；无参考的通用六自由度接口仍走现有 Rust 内核。在线和保存路径
都有原始重力参考，因此使用新分支。删除 gravity_sigma_rad 软约束参数。
测量旋转与参考重力冲突会被拒绝；达到迭代上限、非有限结果、代价上升也拒绝。
GTSAM 报告填充外层迭代和代价，不伪造其内部接受/拒绝步数。

CMake 新增 GTSAM 依赖，Windows vcpkg manifest 同步加入 gtsam；NX 运行时
检查确认 slamd 可解析 /usr/local/lib/libgtsam.so.4。未完成 Windows 构建或
另一台 S100P 上的依赖安装验证。NX 编译存在 GTSAM 4.0.2 头文件引出的旧 TBB
接口弃用提示，未把提示描述成零警告构建。

## 验证及部署状态

NX ARM 五项回归全部通过：pgo_core、online_graph、online_mapping、
messages_fastlio2_mock_flow、constraint_assembly，总计约 12.16 s。
此前会产生约 3.9° 倾斜并失败的保存端测试现在通过；增加输出重力保持
1e-10 rad 量级和该平地合成图高度跨度小于 1 cm 的检查。还覆盖真实高差、
机身倾斜、偏航自由、污染热启动、固定节点在边的两端、断链恢复与迭代耗尽。

这些是 ARM 测试与离线回放，不是新采集地图的现场精度验收。旧图没有被改写，
经用户确认停稳且无待保留扫描后，已安装 .70 并通过 ProductControl 启动 map/camera。
运行中的 slamd 来自 .70，/proc/<pid>/maps 确认加载 libgtsam.so.4.0.2。
SLAM 为 TRACKING，处理扫描约 10 Hz，驱动输出零速度；没有下发运动目标。旧 .69 不包含此修复，
不要将本轮测试结果用于为 .69 安装背书。

903room_v2 只读回放结果：129 帧，104 帧连着起点，19 次相邻配准拒绝，12 条
回环，10 次优化、0 次求解失败，耗时约 86.15 s，最慢后台帧约 2.47 s。
剩余 25 帧仍未连通，与此前恢复版的覆盖数相同。这是保存后的位姿输入，
缺少旧版原始全速位姿，不能据此声称重现原始采集或修复约 63 cm 的历史高差。
证据归档：build/go2-3d-support-20260924/gtsam-adaptation-evidence.tar.gz。

仍需处理：剩余未连通帧；正确消费 LIO 连续观测及相对不确定性；回环鲁棒损失；
新扫描的闭环高度与三维支撑验收。用 GTSAM 不会自动消除这几项。

## .70 部署证据

版本：v2.3.0-go2.20260924.70；会话：product-7dba9ae583b1440690e744b10b1d39fd。
安装前后发布包验证通过。开机后没有旧 Product 记录，因此安装完成后显式通过
python -m lingtu.control switch map --robot unitree/go2 --env real --variant camera
启动新建图会话，返回 active。现有地图未被覆盖。当前静止只有一个关键帧，尚未
触发现场闭环优化；加载库和离线测试不能替代新的闭环采集验收。
数值对比见 [后端对照](pose-graph-backend-comparison-20260924.md)。
