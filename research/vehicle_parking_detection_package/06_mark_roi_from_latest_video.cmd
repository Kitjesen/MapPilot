@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0"
call "%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\mark_roi_from_latest_video.cmd" no_parking_01 no_parking_zone_01
