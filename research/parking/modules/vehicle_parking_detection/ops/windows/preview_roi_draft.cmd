@echo off
chcp 65001 > nul
setlocal

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"
set "PY=%PACKAGE_ROOT%\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

set "DRAFT=%~1"
if "%DRAFT%"=="" (
  echo Usage:
  echo   preview_roi_draft.cmd %PACKAGE_ROOT%\validation\vehicle_parking_roi\no_parking_01\RUN_NAME\roi_draft.yaml
  pause
  exit /b 1
)

"%PY%" modules\vehicle_parking_detection\tools\preview_roi_draft.py --draft "%DRAFT%"
pause
