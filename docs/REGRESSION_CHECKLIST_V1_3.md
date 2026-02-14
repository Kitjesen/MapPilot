# v1.3.0 回归测试清单
留着真机测试用的。
## 0. 前置条件

- 机器人在线，`remote_monitoring` 与导航栈已启动
- Flutter App 已连接到目标设备
- 地图与定位状态正常（`map -> odom` TF 可用）

## 1. 定位健康评分 (LocalizationScorer)

- [ ] StatusScreen 显示 LOC HEALTH 卡片，评分 0-100，颜色随分数变化
- [ ] 健康页面显示圆环进度、限速因子百分比
- [ ] 遮挡 LiDAR 或移动到特征稀疏区域，评分应明显下降
- [ ] 评分 < 40 时，遥控速度应被自动限制（降速标签出现）
- [ ] 评分恢复后，速度限制自动解除
- [ ] 定位完全丢失（TF 断开），评分应降至 < 20，`EVENT_TYPE_LOCALIZATION_LOST` 事件生成

## 2. 飞行数据记录器 (FlightRecorder)

- [ ] 触发 E-stop 后，`/opt/robot/flight_records/` 下生成 `.fdr` 文件
- [ ] dump 文件大小 ≈ 256 + 64 × 帧数 字节（如 300 帧 ≈ 19.5KB）
- [ ] 健康等级达到 FAULT 时，自动生成 dump
- [ ] 10 秒内连续触发 E-stop，只生成一个 dump（冷却机制）
- [ ] dump 文件头 magic 为 `FLTREC01`，snapshot_count 与实际帧数一致

## 3. E-stop 全链路

- [ ] 任务运行中触发 E-stop → 任务挂起 + FlightRecorder dump + 事件记录
- [ ] E-stop 状态下启动任务按钮禁用
- [ ] 解除 E-stop → 回到 IDLE，可正常恢复操作

## 4. 断线重连

- [ ] 任务运行中断网 5-10s 后重连，任务状态恢复正确
- [ ] 重连后 mode / SlowState / MapScreen 状态一致
- [ ] 重连后 LOC HEALTH 评分正常更新

## 5. 验收记录

- 执行日期：
- 执行人：
- 环境（设备/固件/分支）：
- 失败项与日志路径：
