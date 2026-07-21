@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0"
call "%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\start_vehicle_parking_detection_192_168_66_65.cmd" 0 no_parking_01 bottom
