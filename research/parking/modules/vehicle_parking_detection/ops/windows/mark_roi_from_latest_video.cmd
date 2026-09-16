@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"
set "PY=%PACKAGE_ROOT%\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

set "POINT_ID=%~1"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "LOCATION=%~2"
if "%LOCATION%"=="" set "LOCATION=no_parking_zone_01"
set "REPLACE_ARG="
if /I "%~3"=="replace" set "REPLACE_ARG=--replace"

echo Open latest fetched vehicle-parking video and mark ROI.
echo point_id=%POINT_ID% location=%LOCATION%
if not "%REPLACE_ARG%"=="" echo replace_existing_draft=true
echo.
echo Stop stale local ROI marker windows if any.
powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'interactive_roi_from_video.py' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" >nul 2>nul
echo.
echo In the video window:
echo   SPACE play/pause
echo   A/D   previous/next frame
echo   J/L   previous/next second
echo   R     draw polygon ROI on current frame
echo   1-9   select existing ROI
echo   [/]   previous/next ROI
echo   H     add current frame as extra scene anchor to selected ROI
echo   Q     finish and apply local ROI memory
echo.

"%PY%" modules\vehicle_parking_detection\tools\interactive_roi_from_video.py --run-dir latest --point-id "%POINT_ID%" --location "%LOCATION%" --apply %REPLACE_ARG%
pause
