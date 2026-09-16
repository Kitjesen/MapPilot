@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_SH=~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"
set "REMOTE_TAR=~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_return.tgz"
set "LOCAL_RETURN=%PACKAGE_ROOT%\board_return\vehicle_parking_detection"
set "FETCH_PS1=%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\fetch_latest_recording_chunked.ps1"

powershell -NoProfile -ExecutionPolicy Bypass -File "%FETCH_PS1%" -Board "%BOARD%" -RemoteSh "%REMOTE_SH%" -RemoteTar "%REMOTE_TAR%" -LocalReturn "%LOCAL_RETURN%"
if errorlevel 1 (
  echo Fetch failed.
  pause
  exit /b 1
)
echo Local return: %LOCAL_RETURN%
pause
