@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_ROOT=~/hongsenpang/Yolov11_project/vehicle_parking_detection"
set "REMOTE_SH=%REMOTE_ROOT%/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"
set "REMOTE_MIPI_SH=~/hongsenpang/Yolov11_project/rdk_board_validate/mipi_cam_service/remote_rdk_mipi_cam_service.sh"
set "REMOTE_MODEL=~/hongsenpang/Yolov11_project/rdk_board_validate/v2_yolo11s_static_640x320_rect_rgb_int8"
set "HBM_NAME=patrol_v2_yolo11s_static_640x320_rect_rgb_int8.hbm"

set "RUN_SECONDS=%~1"
if "%RUN_SECONDS%"=="" set "RUN_SECONDS=0"
set "POINT_ID=%~2"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "VIEW=%~3"
if "%VIEW%"=="" set "VIEW=bottom"

set "RECORD_FPS=15"
set "PROCESS_INTERVAL=0.0667"
set "INFER_INTERVAL=0.0667"
set "INPUT_FORMAT=rgb_i8_centered"
set "INPUT_HEIGHT=640"
set "INPUT_WIDTH=320"
set "HBM_BACKEND=hbm_runtime"
set "CONF=0.15"
set "IOU=0.50"
set "DETECTION_TTL=1.0"
set "CAMERA_WATCHDOG=1"
set "CAMERA_STALE_RESTART_SECONDS=8"
set "CAMERA_WATCHDOG_INTERVAL=5"
set "CAMERA_RESTART_COOLDOWN_SECONDS=20"

echo Start standalone vehicle parking detection.
echo Board: %BOARD%
echo run_seconds=%RUN_SECONDS% point_id=%POINT_ID% view=%VIEW%
echo target record_fps=%RECORD_FPS% target infer_interval=%INFER_INTERVAL%
echo.

call %PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\deploy_vehicle_parking_192_168_66_65.cmd
if errorlevel 1 exit /b 1

ssh -tt %BOARD% "ACTION=start bash %REMOTE_MIPI_SH%"
ssh -tt %BOARD% "ACTION=stop bash %REMOTE_SH%"
ssh -tt %BOARD% "ACTION=start_detect RUN_SECONDS=%RUN_SECONDS% POINT_ID=%POINT_ID% VIEW=%VIEW% RECORD_FPS=%RECORD_FPS% PROCESS_INTERVAL=%PROCESS_INTERVAL% INFER_INTERVAL=%INFER_INTERVAL% INPUT_FORMAT=%INPUT_FORMAT% INPUT_HEIGHT=%INPUT_HEIGHT% INPUT_WIDTH=%INPUT_WIDTH% HBM_BACKEND=%HBM_BACKEND% CONF=%CONF% IOU=%IOU% DETECTION_TTL=%DETECTION_TTL% CAMERA_WATCHDOG=%CAMERA_WATCHDOG% CAMERA_STALE_RESTART_SECONDS=%CAMERA_STALE_RESTART_SECONDS% CAMERA_WATCHDOG_INTERVAL=%CAMERA_WATCHDOG_INTERVAL% CAMERA_RESTART_COOLDOWN_SECONDS=%CAMERA_RESTART_COOLDOWN_SECONDS% MIPI_SH=%REMOTE_MIPI_SH% HBM=%REMOTE_MODEL%/hbm/%HBM_NAME% bash %REMOTE_SH%"
if errorlevel 1 (
  echo Start vehicle parking detection failed.
  pause
  exit /b 1
)

echo.
echo Detection started. Check status with:
echo   %PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\status_vehicle_parking_192_168_66_65.cmd
echo.
pause
