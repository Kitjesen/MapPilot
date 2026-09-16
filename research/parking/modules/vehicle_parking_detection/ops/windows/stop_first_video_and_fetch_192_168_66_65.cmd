@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_ROOT=~/hongsenpang/Yolov11_project/vehicle_parking_detection"
set "REMOTE_SH=%REMOTE_ROOT%/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"
set "REMOTE_TAR=%REMOTE_ROOT%/vehicle_parking_return.tgz"
set "LOCAL_RETURN=%PACKAGE_ROOT%\board_return\vehicle_parking_detection"
set "FETCH_PS1=%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\fetch_latest_recording_chunked.ps1"
set "PY=%PACKAGE_ROOT%\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

set "POINT_ID=%~1"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "LOCATION=%~2"
if "%LOCATION%"=="" set "LOCATION=no_parking_zone_01"

echo Stop continuous first video recording, fetch it, and generate ROI draft.
echo Board: %BOARD%
echo point_id=%POINT_ID% location=%LOCATION%
echo.

echo [1/4] Stop recorder to flush mp4.
ssh -tt %BOARD% "ACTION=stop bash %REMOTE_SH%"

echo [2/4] Package and fetch recording.
powershell -NoProfile -ExecutionPolicy Bypass -File "%FETCH_PS1%" -Board "%BOARD%" -RemoteSh "%REMOTE_SH%" -RemoteTar "%REMOTE_TAR%" -LocalReturn "%LOCAL_RETURN%"
if errorlevel 1 (
  echo Fetch failed.
  pause
  exit /b 1
)

echo [3/4] Extract ROI keyframes.
"%PY%" modules\vehicle_parking_detection\tools\extract_roi_keyframes.py --run-dir latest --point-id "%POINT_ID%" --location "%LOCATION%" --overwrite
if errorlevel 1 (
  echo ROI keyframe extraction failed.
  pause
  exit /b 1
)

echo [4/4] Done.
echo Recording return: %LOCAL_RETURN%
echo ROI draft root: %PACKAGE_ROOT%\validation\vehicle_parking_roi\%POINT_ID%
echo Tip: edit roi_draft.yaml later if you want the ROI name/location to be Chinese.
echo.
pause
