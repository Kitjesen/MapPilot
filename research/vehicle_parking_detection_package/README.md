# 车辆违停检测独立交付包

本目录只保留车辆违停检测验证相关内容，用于交给同事继续在 RDK 板端验证。

状态：研究/集成候选，不是当前 LingTu 产品已发布的巡检分析器。它可以产出离线报警记录和样例证据，但尚未接入 `InspectionEvidenceStore` 的可信观察边界，也没有通过主系统的实机故障注入验收。正式产品巡检仍以 `capture:parking` 的 trusted observation 合同为准。

默认板端：

```text
sunrise@192.168.66.65
```

该地址是本研究交付包形成时的历史 RDK 验证板地址，不代表当前 LingTu 实机地址，也不代表已经部署到主线机器人运行路径。迁移到现场机器人前必须改成该现场的独立验证板或经批准的目标主机。

当前验证配置：

```text
point_id: no_parking_01
ROI 数量: 9
场景关键帧特征数量: 10
录像 FPS: 15
推理目标 FPS: 15
车辆检测置信度阈值: 0.15
禁停停留触发阈值: 2 秒
场景匹配阈值: 0.6
ROI 匹配保持时间: 5 秒
报警格式: 关键帧图片 + 时间 + 事件名称 + 地点
```

## 目录结构

```text
D:\vehicle_parking_detection_package
  modules\vehicle_parking_detection\     车辆违停核心代码、配置、工具、部署脚本
  models\rdk_hbm\v2_rect640x320\         板端 HBM 模型
  ops\remote\                            MIPI 摄像头服务脚本
  tests\                                 本机单元测试
  validation\                            ROI 草稿、关键帧、匹配分析样例
  sample_results\                        轻量巡检结果样例，不含大视频
  board_return\                          后续从板端拉回的结果会放这里
  docs\                                  模块说明、操作步骤、风险说明
```

完整巡检视频通常很大，本交付包没有复制历史 `raw_rotated.mp4` 和 `annotated_detect.mp4`。同事重新巡检后，拉回脚本会自动写入 `board_return\vehicle_parking_detection\output`。

## 快捷脚本

先运行一次本机 Python 环境安装：

```bat
00_setup_local_python_env.cmd
```

检查板端连接：

```bat
01_check_board_connection.cmd
```

部署并开始巡检：

```bat
02_start_patrol.cmd
```

查看状态：

```bat
03_status.cmd
```

停止巡检：

```bat
04_stop_patrol.cmd
```

拉回完整结果和报警记录：

```bat
05_fetch_results.cmd
```

从最新拉回视频追加 ROI 或关键帧特征：

```bat
06_mark_roi_from_latest_video.cmd
```

清理板端旧输出，只保留最近一次巡检：

```bat
07_cleanup_board_storage.cmd
```

## 详细文档

```text
docs\模块说明.md
docs\操作步骤.md
docs\已知限制与下一步.md
```

## 当前报警输出格式

每条报警只输出四个字段：

```json
{
  "keyframe_image": "alarm_images/xxx.jpg",
  "time": "2026-07-03T14:22:33+08:00",
  "event_name": "车辆违停",
  "location": "no_parking_zone_01"
}
```

报警记录在板端本地保存，断网巡检结束后再由脚本拉回。

## 接入 LingTu 主系统前必须完成

- 将报警结果转换为 `TrustedParkingObservation`，并通过 `src/runtime/contracts/inspection_evidence.py` 校验。
- 明确 `normal`、`violation`、`unknown`、`capture_only`、`needs_review` 的判定边界。
- 增加与 Gateway/Inspection Workbench 的端到端测试，避免把 research package 的输出直接显示成产品结论。
- 完成板端断网、摄像头 stale、模型不可用、ROI 未匹配、进程重启后的安全行为验证。
