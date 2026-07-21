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
set "REMOTE_TAR=%REMOTE_ROOT%/vehicle_parking_return.tgz"
set "LOCAL_RETURN=%PACKAGE_ROOT%\board_return\vehicle_parking_detection"
set "FETCH_PS1=%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\fetch_latest_recording_chunked.ps1"
set "PY=%PACKAGE_ROOT%\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

set "RUN_SECONDS=%~1"
if "%RUN_SECONDS%"=="" set "RUN_SECONDS=60"
set "POINT_ID=%~2"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "LOCATION=%~3"
if "%LOCATION%"=="" set "LOCATION=no_parking_zone_01"
set "VIEW=%~4"
if "%VIEW%"=="" set "VIEW=bottom"

set "RECORD_FPS=15"
set "PROCESS_INTERVAL=0.0667"
set "INFER_INTERVAL=0.0667"
set /a WAIT_SECONDS=%RUN_SECONDS%+10 >nul 2>nul
if "%WAIT_SECONDS%"=="" set "WAIT_SECONDS=70"

echo Record first ROI video for vehicle parking detection.
echo Board: %BOARD%
echo point_id=%POINT_ID% location=%LOCATION%
echo run_seconds=%RUN_SECONDS% record_fps=%RECORD_FPS% view=%VIEW%
echo.

call %PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\deploy_vehicle_parking_192_168_66_65.cmd
if errorlevel 1 exit /b 1

echo [1/5] Start MIPI camera service.
ssh -tt %BOARD% "ACTION=start bash %REMOTE_MIPI_SH%"

echo [2/5] Stop old vehicle parking runtime if running.
ssh -tt %BOARD% "ACTION=stop bash %REMOTE_SH%"

echo [3/5] Start record-only runtime.
ssh -tt %BOARD% "ACTION=start_record RUN_SECONDS=%RUN_SECONDS% POINT_ID=%POINT_ID% VIEW=%VIEW% RECORD_FPS=%RECORD_FPS% PROCESS_INTERVAL=%PROCESS_INTERVAL% INFER_INTERVAL=%INFER_INTERVAL% bash %REMOTE_SH%"
if errorlevel 1 (
  echo Start record failed.
  pause
  exit /b 1
)

echo [4/5] Waiting %WAIT_SECONDS% seconds for recording to finish.
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Sleep -Seconds %WAIT_SECONDS%"
ssh -tt %BOARD% "ACTION=stop bash %REMOTE_SH%"

echo [5/5] Package and fetch recording.
powershell -NoProfile -ExecutionPolicy Bypass -File "%FETCH_PS1%" -Board "%BOARD%" -RemoteSh "%REMOTE_SH%" -RemoteTar "%REMOTE_TAR%" -LocalReturn "%LOCAL_RETURN%"
if errorlevel 1 (
  echo Fetch failed.
  pause
  exit /b 1
)

echo.
echo Extract ROI keyframes from latest fetched recording.
"%PY%" modules\vehicle_parking_detection\tools\extract_roi_keyframes.py --run-dir latest --point-id "%POINT_ID%" --location "%LOCATION%" --overwrite

echo.
echo Done.
echo Recording return: %LOCAL_RETURN%
echo ROI draft root: %PACKAGE_ROOT%\validation\vehicle_parking_roi\%POINT_ID%
echo Tip: edit roi_draft.yaml later if you want the ROI name/location to be Chinese.
echo.
pause
