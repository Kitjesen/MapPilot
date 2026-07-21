# 操作步骤

本文用于研究交付包的 RDK 板端验证，不是 LingTu 主巡检系统的生产部署手册。执行结果只能作为车辆违停候选证据；接入主系统前还需要通过 trusted observation 合同和实机验收。

## 1. 本机准备

在 Windows 上打开 `D:\vehicle_parking_detection_package`，先运行：

```bat
00_setup_local_python_env.cmd
```

需要本机有：

```text
Python 3.10+
OpenSSH 客户端 ssh/scp
Windows tar 命令
```

## 2. 确认板端连接

板端开机后等待约 1 分钟，运行：

```bat
01_check_board_connection.cmd
```

成功时会看到：

```text
ping 有回复
ssh 输出 hostname 和 date
```

如果提示密码，输入板端 `sunrise` 用户密码。

## 3. 部署并启动巡检

运行：

```bat
02_start_patrol.cmd
```

它会自动：

```text
上传 modules\vehicle_parking_detection
上传 HBM 模型
上传 MIPI 摄像头服务脚本
启动 MIPI 摄像头
停止旧车辆违停巡检
启动新巡检
```

启动参数：

```text
RUN_SECONDS = 0，表示不限时巡检
POINT_ID = no_parking_01
VIEW = bottom
```

## 4. 出发前检查状态

运行：

```bat
03_status.cmd
```

重点确认：

```text
roi_count=9
configured_roi_with_polygon=9
scene_anchor_count=10
scene_match_min_score=0.6
scene_match_hold_seconds=5
vehicle_confidence_threshold=0.15
min_roi_dwell_seconds=2
camera_watchdog=RUNNING
infer_frame 持续增长
source_stale_seconds 接近 0
error 为空
```

满足这些条件后再拿板端去巡检。

如果 `roi_count`、`scene_anchor_count`、`infer_frame`、`source_stale_seconds`
任一项异常，本次结果只能用于排障，不能作为有效违停结论。

## 5. 巡检结束

先停止巡检：

```bat
04_stop_patrol.cmd
```

再拉回结果：

```bat
05_fetch_results.cmd
```

本机结果目录：

```text
board_return\vehicle_parking_detection\output\<RUN_ID>\
board_return\vehicle_parking_alarms\
```

重点查看：

```text
raw_rotated.mp4             原始旋转后录像
annotated_detect.mp4        带 ROI、场景匹配和车辆框的视频
latest_status.json          最新状态
recording_summary.json      巡检汇总
alarm_events.jsonl          报警记录
latest_alarm_event.json     最新报警
alarm_images\               报警关键帧图片
```

## 6. 追加 ROI 或关键帧

拉回完整结果后，运行：

```bat
06_mark_roi_from_latest_video.cmd
```

不要加 `replace`，这样会在已有 ROI 基础上追加。

新增 ROI：

```text
在视频窗口移动到目标帧
按 R
左键依次点多边形顶点
按 ENTER 保存
```

给已有 ROI 增加附近关键帧特征：

```text
按 1-9 选择 ROI，或用 [ / ] 切换
移动到同一禁停区附近的稳定视角
按 H
```

全部完成后按：

```text
Q
```

工具会自动写入：

```text
modules\vehicle_parking_detection\configs\site_rois.yaml
```

再次巡检前重新运行：

```bat
02_start_patrol.cmd
```

即可把新 ROI 配置部署到板端。

## 7. 清理板端空间

确认结果已经拉回后，可运行：

```bat
07_cleanup_board_storage.cmd
```

它只删除：

```text
临时打包文件
旧巡检 output，只保留最近一次
```

不会删除模型、代码、ROI 配置和摄像头脚本。

## 8. 同步到 LingTu 主系统前

不要直接把 `alarm_events.jsonl` 写入产品巡检结论。应先完成：

```text
alarm_events.jsonl
  -> TrustedParkingObservation
  -> InspectionEvidenceStore 校验
  -> capture:parking verdict
  -> Inspection Workbench 展示
```
