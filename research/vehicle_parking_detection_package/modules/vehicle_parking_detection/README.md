# Vehicle Parking Detection

Standalone validation module for one feature: vehicle no-parking detection.

Status: research/integration candidate. This module runs as a standalone RDK
validation package and is not yet the shipped LingTu inspection analyzer.
Production use must pass through the LingTu trusted parking observation
contract before a verdict is accepted by Inspection Evidence.

The module is intentionally separate from the multi-event patrol runtime. It supports:

- Recording the first RDK camera video for ROI marking.
- Extracting keyframes and creating an ROI draft.
- Saving the reviewed ROI into `configs/site_rois.yaml`.
- Running high-FPS vehicle detection on the RDK board.
- Keeping alarm records locally on the board during offline patrols.
- Fetching only alarm images and structured alarm records after patrol.

## Defaults

```text
board: sunrise@192.168.66.65
point_id: no_parking_01
location: 禁停区01
event_name: 车辆违停
record_fps: 15
target infer_fps: 15
process_interval: 0.0667
infer_interval: 0.0667
input: 640x320 rgb_i8_centered
hbm_backend: hbm_runtime
```

`192.168.66.65` is the historical RDK validation board used by this research
package. It is not the current LingTu robot address and does not imply that the
module is deployed in the main robot runtime path.

Runtime uses a latest-frame strategy. It does not queue old frames if HBM inference is slower than 15 FPS.

Current verdicts are local to this package. Treat package alarms as validation
events until they are converted into LingTu `TrustedParkingObservation` records
and admitted by the main runtime contract.

## Workflow

1. Record the first video and fetch it:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\record_first_video_192_168_66_65.cmd 60 no_parking_01 no_parking_zone_01 bottom
```

For manual-length recording, start continuous recording first:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\start_first_video_continuous_192_168_66_65.cmd no_parking_01 bottom
```

Stop, fetch, and generate the ROI draft when the scene is ready:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\stop_first_video_and_fetch_192_168_66_65.cmd no_parking_01 no_parking_zone_01
```

2. Mark scene-specific ROI from the fetched video:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\mark_roi_from_latest_video.cmd
```

If an old global ROI draft should be discarded and redrawn as scene-specific ROIs:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\mark_roi_from_latest_video.cmd no_parking_01 no_parking_zone_01 replace
```

The script opens the latest fetched `raw_rotated.mp4`. In the video window:

```text
SPACE: play/pause
A/D: previous/next frame
J/L: previous/next second
K: save current keyframe
R: draw polygon ROI on the current frame
Q: finish and apply local ROI memory
```

In the polygon window, left click to add points, right click or `U` to undo, `C` to clear, `ENTER` to save, and `ESC` to cancel. Each saved polygon is bound to the current video frame as a scene anchor. During patrol, the runtime visually matches the current camera frame to the saved anchors and only enables ROIs for the matching scene.

Saved polygons are written to:

```text
<PACKAGE_ROOT>\validation\vehicle_parking_roi\no_parking_01\<RUN_NAME>\roi_draft.yaml
```

The same script also applies non-empty polygons into `configs/site_rois.yaml` with a backup file.

Manual fallback: edit the generated draft and fill `rois[0].polygon` with pixel points from the extracted keyframes or `latest_source_rotated.jpg`.

3. Preview the ROI:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\preview_roi_draft.cmd <PACKAGE_ROOT>\validation\vehicle_parking_roi\no_parking_01\<RUN_NAME>\roi_draft.yaml
```

4. Apply ROI memory:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\apply_roi_draft.cmd <PACKAGE_ROOT>\validation\vehicle_parking_roi\no_parking_01\<RUN_NAME>\roi_draft.yaml
```

5. Start patrol detection:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\start_vehicle_parking_detection_192_168_66_65.cmd 0 no_parking_01 bottom
```

Use `0` seconds for continuous runtime. Use a positive number for a bounded validation run.

6. Fetch alarm records after patrol:

```bat
<PACKAGE_ROOT>\modules\vehicle_parking_detection\ops\windows\fetch_alarm_records_192_168_66_65.cmd
```

## Alarm Output

Each alarm writes one keyframe image and one structured JSON line:

```json
{
  "keyframe_image": "alarm_images/xxx.jpg",
  "time": "2026-07-03T14:22:33+08:00",
  "event_name": "车辆违停",
  "location": "禁停区01"
}
```

The board-side run directory contains:

```text
output/<run_id>/
  raw_rotated.mp4
  latest_source_rotated.jpg
  latest_annotated.jpg
  latest_status.json
  latest_detections.json
  alarm_images/
  alarm_events.jsonl
  latest_alarm_event.json
```

`latest_status.json` includes `infer_fps`, `target_infer_fps`, `capture_fps`, and `alarm_count`.
