@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "LOCAL_SCRIPT=%PACKAGE_ROOT%\modules\vehicle_parking_detection\ops\remote\cleanup_vehicle_parking_storage.sh"
set "REMOTE_SCRIPT=/tmp/cleanup_vehicle_parking_storage.sh"
set "REMOTE_ROOT=/home/sunrise/hongsenpang/Yolov11_project/vehicle_parking_detection"

set "KEEP_RUNS=%~1"
if "%KEEP_RUNS%"=="" set "KEEP_RUNS=1"

echo Clean vehicle parking board storage.
echo Board: %BOARD%
echo Remote root: %REMOTE_ROOT%
echo Keep latest output runs: %KEEP_RUNS%
echo.
echo This deletes package temp files and old output runs only.
echo It does NOT delete ROI config, model files, source code, or camera service.
echo.

if not exist "%LOCAL_SCRIPT%" (
  echo Missing local cleanup script:
  echo   %LOCAL_SCRIPT%
  pause
  exit /b 1
)

scp "%LOCAL_SCRIPT%" %BOARD%:%REMOTE_SCRIPT%
if errorlevel 1 (
  echo Upload cleanup script failed.
  pause
  exit /b 1
)

ssh -tt %BOARD% "chmod +x %REMOTE_SCRIPT% && ROOT='%REMOTE_ROOT%' KEEP_RUNS='%KEEP_RUNS%' bash %REMOTE_SCRIPT%"
if errorlevel 1 (
  echo Board cleanup failed.
  pause
  exit /b 1
)

echo.
echo Board cleanup finished.
pause
