# LIO-SAM 原生后端迁移

## 当前状态

`.71` 已在 NX ARM 构建、打包并通过四项针对性回归。用户再次确认停稳且无待
保留扫描后，已安装并通过 ProductControl 启动 map/camera。当前会话为
product-b19d160addbb40b985def2763189a958。不是整套原始 ROS LIO-SAM 已部署，
也不是 Go2 闭环导航已验收。安装后 slamd 实际路径来自 `.71`，二进制包含
原生 sam::Backend::append 并加载 GTSAM 4.0.2。SLAM 为 TRACKING，扫描处理
约 9 Hz，1 个关键帧进入新后端，无失败，驱动 commanded_zero。
证据归档：build/go2-3d-support-20260924/lio-sam-deploy71-evidence.tar.gz。

## 采用的上游和边界

选用 [TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/0be1fbe6275fb8366d5b800af4fc8c76a885c869)，
固定提交 `0be1fbe6275fb8366d5b800af4fc8c76a885c869`，BSD-3-Clause。
原始 mapOptmization.cpp 放在 research/localization/lio-sam-upstream，许可保留。
起初考察的 engcang/FAST-LIO-SAM 有非商业限制，未复制其代码进产品。

移植 upstream 的 addOdomFactor、iSAM2 更新顺序、距离/时间回环候选、
历史子图拼接、PCL ICP、回环 BetweenFactor、全历史位姿更新。直接使用 GTSAM
Pose3/PriorFactor/BetweenFactor/iSAM2 和 PCL ICP，不再在线调用自研四自由度
配准、描述子与 batch LM。改动位于 src/localization/sam/backend.*；
与固定源码对应及所有适配差异见 sam/upstream/UPSTREAM.md。

保留当前 MID-360 Fast-LIO 前端、校准、DDS、ProductControl 和三维导航。
输入改接未修正的连续 LIO body 位姿与去畸变 body-local 点云；没有引入原始
LIO-SAM 面向旋转雷达的特征前端或 GPS 分支。因此准确名称是“Fast-LIO 前端 +
LIO-SAM 原生后端移植”，不能宣称完整原封不动 LIO-SAM。

采用上游六自由度 Pose3、全秩固定里程计噪声，而不是 `.70` 的秩四配准因子与
固定重力变量。没有把单帧边缘协方差冒充相对协方差。默认数值沿用上游，尚未
完成 Go2 参数标定；不能保证换成成熟库就自动改善所有场景。

## 产品接线

- OnlineMapping 保留有界队列、单后台线程、epoch 重置及预览接口，内部改用新后端。
- 有效 LIO 相邻观测连续连接；稀疏点云不会导致整个顺序位姿图断链。
- iSAM2 改变历史位姿时重建预览；当前 odometry 不被跳变修正。
- 后端异常后该 generation 停止继续累积并拒绝保存，需新会话恢复。
- 已完成回环的保存使用完整一致图快照；尚无完成回环时保存原始 LIO 快照，
  performed=false，不能把后台待处理的结果宣称已优化。
- mapd 识别完整 lio_sam_isam2 快照，不再第二次运行旧自研 PGO。报告节点数不符
  或混入显式旧因子时拒绝保存。旧地图/离线 PGO 工具仍保留，未混入新在线链路。
- 同时保存 poses.raw.txt、trajectory.raw.txt 和 keyframes.timestamps.txt。
  回放要求原始位姿及实际时间戳，不允许拿优化轨迹或每帧假定 1 秒来冒充输入。

## 验证

NX aarch64 原生库测试：1/1，通过约 0.16 s。逐帧对照上游因子和 iSAM2 更新顺序，
三维姿态/升高保持；时间倒退在改图前拒绝；稀疏扫描连续连接。
合成 16 帧回环接受 5 次 ICP，注入末端 z 漂移 45 mm，输出约 10.7 mm。
这与先前 25 帧模型比较用例不同，不能横向宣传毫米精度改善。

在线建图与 Fast-LIO mock 流：2/2，通过约 1.16 s，覆盖队列容量、epoch 重置、
稀疏帧、实际回环、连续 odometry anchor、异步保存不阻塞新扫描、原始数据保留。
保存：1/1，通过约 7.90 s；指定一个不存在的旧 PGO 可执行文件，SAM 快照仍保存
成功；不完整快照失败。slamd、mapd、lingtu-mapctl 已构建。

发现并修正的集成问题：无回环首批帧后台处理导致保存报忙；测试旧契约断言不应
有优化报告，已改为要求识别新后端且无回环不得声称 performed=true。
构建日志有 GTSAM/TBB 旧接口弃用提示，未宣称零警告；Windows 构建未验证。

## 剩余现场验收

旧 903room_v2 缺少原始关键帧位姿和时间戳，不能精确回放原始采集过程，也没有
被新算法自动修复。应在新候选上小范围闭环采集，回到同一地面，另存新图，比较
原始/优化轨迹高度、回环配准残差和三维重影，再推进大范围采集及导航。
需要警惕上游距离候选在相似房间误匹配、上游默认噪声与 Go2 场景不匹配；目前
没有证据把这两项断言为已经发生的故障，也不为此另造一套回环算法。
