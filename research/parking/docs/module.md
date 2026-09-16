# 模块说明

该模块是独立 RDK 验证包。它复用了 LingTu 的场景目标，但尚未成为产品主线 analyzer；主系统只能接受经过 trusted observation 边界校验的停车观察。

## 核心模块

```text
modules\vehicle_parking_detection\
```

主要文件：

```text
rdk_vehicle_parking_runtime.py   板端运行入口，负责取流、录像、推理、ROI 判断、报警保存
config.py                        读取 site_rois.yaml，生成运行参数和 ROI 配置
scene_matching.py                场景关键帧匹配，支持一个 ROI 绑定多个关键帧特征
parking_engine.py                判断车辆是否进入 ROI，并按停留时间触发报警
tracker.py                       轻量 IOU/中心点跟踪，减少单帧抖动
alarm_store.py                   保存报警关键帧和 JSONL 记录
geometry.py                      点、多边形、ROI 几何判断
models.py                        Detection 和报警触发数据结构
```

## 配置

```text
modules\vehicle_parking_detection\configs\site_rois.yaml
```

当前配置包括：

```text
9 个禁停 ROI
10 个场景关键帧特征
conf = 0.15
dwell_seconds = 2
scene_match_min_score = 0.6
scene_match_hold_seconds = 5
```

## 板端脚本

```text
modules\vehicle_parking_detection\ops\remote\vehicle_parking_service.sh
```

负责：

```text
启动/停止车辆违停检测
打包最新巡检结果
打包报警记录
输出 latest_status.json
摄像头 stale watchdog
```

```text
ops\remote\remote_rdk_mipi_cam_service.sh
```

负责启动/停止 RDK MIPI 摄像头 ROS 服务。

## Windows 操作脚本

```text
modules\vehicle_parking_detection\ops\windows\
```

主要脚本：

```text
deploy_vehicle_parking_192_168_66_65.cmd          部署代码、模型、摄像头脚本到板端
start_vehicle_parking_detection_192_168_66_65.cmd 部署并启动巡检
status_vehicle_parking_192_168_66_65.cmd          查看巡检状态
stop_vehicle_parking_192_168_66_65.cmd            停止巡检
fetch_recording_192_168_66_65.cmd                 拉回完整巡检结果
fetch_alarm_records_192_168_66_65.cmd             只拉回报警记录
mark_roi_from_latest_video.cmd                    从最新拉回视频画 ROI/追加关键帧
cleanup_board_storage_192_168_66_65.cmd           清理板端旧输出
```

## ROI 标注工具

```text
modules\vehicle_parking_detection\tools\interactive_roi_from_video.py
```

能力：

```text
打开最新拉回的 raw_rotated.mp4
在任意帧画多边形 ROI
追加 ROI，不覆盖旧 ROI
给已有 ROI 增加附近关键帧特征
自动写入 site_rois.yaml
```

操作键：

```text
SPACE  播放/暂停
A/D    前一帧/后一帧
J/L    前一秒/后一秒
R      画新的 ROI
ENTER  在画 ROI 窗口保存多边形
1-9    选择已有 ROI
[ / ]  切换 ROI
H      把当前帧加入当前 ROI 的额外场景关键帧
Q      保存并退出
```

## 模型

```text
models\rdk_hbm\v2_rect640x320\hbm\patrol_v2_yolo11s_static_640x320_rect_rgb_int8.hbm
```

板端推理参数：

```text
input_height = 640
input_width = 320
input_format = rgb_i8_centered
hbm_backend = hbm_runtime
```

## 与 LingTu 主线的边界

```text
本包负责：RDK 摄像头取流、HBM 车辆检测、ROI/场景匹配、本地报警记录
主线负责：证据合同、可信观察准入、状态展示、人工复核、最终巡检结论
```

当前不能把本包报警直接等同于主系统 `capture:parking` 的最终 verdict。
