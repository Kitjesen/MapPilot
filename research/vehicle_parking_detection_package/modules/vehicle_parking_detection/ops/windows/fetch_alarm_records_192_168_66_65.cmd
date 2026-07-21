@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_SH=~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"
set "REMOTE_TAR=~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_alarms.tgz"
set "LOCAL_RETURN=%PACKAGE_ROOT%\board_return\vehicle_parking_alarms"

ssh %BOARD% "ACTION=package_alarms bash %REMOTE_SH%"
if not exist "%LOCAL_RETURN%" mkdir "%LOCAL_RETURN%"
scp %BOARD%:%REMOTE_TAR% "%LOCAL_RETURN%\vehicle_parking_alarms.tgz"
if errorlevel 1 (
  echo Fetch alarm records failed.
  pause
  exit /b 1
)
if exist "%LOCAL_RETURN%\output" rmdir /s /q "%LOCAL_RETURN%\output"
if exist "%LOCAL_RETURN%\logs" rmdir /s /q "%LOCAL_RETURN%\logs"
tar -xzf "%LOCAL_RETURN%\vehicle_parking_alarms.tgz" -C "%LOCAL_RETURN%"
echo Local alarm records: %LOCAL_RETURN%
pause
