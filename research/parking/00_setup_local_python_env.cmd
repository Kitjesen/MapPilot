@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0"
cd /d "%PACKAGE_ROOT%"

set "PY=python"
where python > nul 2> nul
if errorlevel 1 (
  echo ERROR: Python was not found in PATH.
  echo Install Python 3.10+ first, then run this script again.
  pause
  exit /b 1
)

if not exist "%PACKAGE_ROOT%\.venv\Scripts\python.exe" (
  echo Create local virtual environment...
  %PY% -m venv "%PACKAGE_ROOT%\.venv"
  if errorlevel 1 (
    echo Failed to create .venv.
    pause
    exit /b 1
  )
)

"%PACKAGE_ROOT%\.venv\Scripts\python.exe" -m pip install --upgrade pip
"%PACKAGE_ROOT%\.venv\Scripts\python.exe" -m pip install -r "%PACKAGE_ROOT%\modules\vehicle_parking_detection\requirements.txt"
if errorlevel 1 (
  echo Install Python dependencies failed.
  pause
  exit /b 1
)

echo.
echo LOCAL_PYTHON_ENV_READY
echo.
pause
