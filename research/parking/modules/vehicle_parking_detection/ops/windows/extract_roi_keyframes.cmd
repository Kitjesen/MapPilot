@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"
set "PY=%PACKAGE_ROOT%\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

set "RUN_DIR=%~1"
if "%RUN_DIR%"=="" set "RUN_DIR=latest"
set "POINT_ID=%~2"
if "%POINT_ID%"=="" set "POINT_ID=no_parking_01"
set "LOCATION=%~3"
if "%LOCATION%"=="" set "LOCATION=no_parking_zone_01"

"%PY%" modules\vehicle_parking_detection\tools\extract_roi_keyframes.py --run-dir "%RUN_DIR%" --point-id "%POINT_ID%" --location "%LOCATION%" --overwrite
pause
