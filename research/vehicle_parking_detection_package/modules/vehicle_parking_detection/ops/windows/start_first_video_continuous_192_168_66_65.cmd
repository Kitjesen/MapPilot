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

set "POINT_ID=%~1"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "VIEW=%~2"
if "%VIEW%"=="" set "VIEW=bottom"

set "RECORD_FPS=15"
set "PROCESS_INTERVAL=0.0667"
set "INFER_INTERVAL=0.0667"

echo Start continuous first video recording for ROI marking.
echo Board: %BOARD%
echo point_id=%POINT_ID% view=%VIEW%
echo record_fps=%RECORD_FPS%
echo.

call %PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\deploy_vehicle_parking_192_168_66_65.cmd
if errorlevel 1 exit /b 1

echo [1/3] Start MIPI camera service.
ssh -tt %BOARD% "ACTION=start bash %REMOTE_MIPI_SH%"

echo [2/3] Stop old vehicle parking runtime if running.
ssh -tt %BOARD% "ACTION=stop bash %REMOTE_SH%"

echo [3/3] Start continuous record-only runtime. Stop it later with stop_first_video_and_fetch_192_168_66_65.cmd.
ssh -tt %BOARD% "ACTION=start_record RUN_SECONDS=0 POINT_ID=%POINT_ID% VIEW=%VIEW% RECORD_FPS=%RECORD_FPS% PROCESS_INTERVAL=%PROCESS_INTERVAL% INFER_INTERVAL=%INFER_INTERVAL% bash %REMOTE_SH%"
if errorlevel 1 (
  echo Start continuous recording failed.
  pause
  exit /b 1
)

echo.
echo Continuous recording is running on the board.
echo Stop and fetch later with:
echo   %PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\stop_first_video_and_fetch_192_168_66_65.cmd %POINT_ID%
echo.
pause
