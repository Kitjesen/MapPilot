@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0"

echo Fetch full patrol result...
call "%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\fetch_recording_192_168_66_65.cmd"
if errorlevel 1 exit /b 1

echo.
echo Fetch alarm-only records...
call "%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\windows\fetch_alarm_records_192_168_66_65.cmd"
if errorlevel 1 exit /b 1

echo.
echo Results:
echo   %PACKAGE_ROOT%board_return\vehicle_parking_detection\output
echo   %PACKAGE_ROOT%board_return\vehicle_parking_alarms
echo.
pause
