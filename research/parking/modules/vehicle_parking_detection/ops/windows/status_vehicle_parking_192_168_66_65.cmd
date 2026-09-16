@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_SH=~/hongsenpang/Yolov11_project/vehicle_parking_detection/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"

ssh -tt %BOARD% "ACTION=status bash %REMOTE_SH%"
echo.
pause
