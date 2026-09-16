@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "PACKAGE_ROOT=%~dp0..\..\..\.."
for %%I in ("%PACKAGE_ROOT%") do set "PACKAGE_ROOT=%%~fI"
cd /d "%PACKAGE_ROOT%"

set "BOARD=sunrise@192.168.66.65"
set "REMOTE_ROOT=~/hongsenpang/Yolov11_project/vehicle_parking_detection"
set "REMOTE_MODEL=~/hongsenpang/Yolov11_project/rdk_board_validate/v2_yolo11s_static_640x320_rect_rgb_int8"
set "REMOTE_SH=%REMOTE_ROOT%/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh"
set "REMOTE_MIPI_ROOT=~/hongsenpang/Yolov11_project/rdk_board_validate/mipi_cam_service"
set "REMOTE_MIPI_SH=%REMOTE_MIPI_ROOT%/remote_rdk_mipi_cam_service.sh"
set "LOCAL_MODULE=%PACKAGE_ROOT%\modules\vehicle_parking_detection"
set "LOCAL_MIPI_SH=%PACKAGE_ROOT%\ops\remote\remote_rdk_mipi_cam_service.sh"
set "LOCAL_MODEL_ROOT=%PACKAGE_ROOT%\models\rdk_hbm\v2_rect640x320"
set "HBM_NAME=patrol_v2_yolo11s_static_640x320_rect_rgb_int8.hbm"

echo Deploy standalone vehicle parking detection module.
echo Board: %BOARD%
echo Remote root: %REMOTE_ROOT%
echo.

if not exist "%LOCAL_MODULE%\rdk_vehicle_parking_runtime.py" (
  echo ERROR: local module not found: %LOCAL_MODULE%
  pause
  exit /b 1
)
if not exist "%LOCAL_MODEL_ROOT%\hbm\%HBM_NAME%" (
  echo ERROR: local HBM not found:
  echo   %LOCAL_MODEL_ROOT%\hbm\%HBM_NAME%
  pause
  exit /b 1
)

ssh %BOARD% "mkdir -p %REMOTE_ROOT% %REMOTE_MODEL%/hbm %REMOTE_MODEL%/metadata %REMOTE_MIPI_ROOT%"
if errorlevel 1 (
  echo SSH mkdir failed.
  pause
  exit /b 1
)

scp -r "%LOCAL_MODULE%" %BOARD%:%REMOTE_ROOT%/
if errorlevel 1 (
  echo Upload vehicle parking module failed.
  pause
  exit /b 1
)

scp "%LOCAL_MIPI_SH%" %BOARD%:%REMOTE_MIPI_SH%
if errorlevel 1 (
  echo Upload mipi camera service failed.
  pause
  exit /b 1
)

scp "%LOCAL_MODEL_ROOT%\hbm\%HBM_NAME%" %BOARD%:%REMOTE_MODEL%/hbm/%HBM_NAME%
if errorlevel 1 (
  echo Upload HBM failed.
  pause
  exit /b 1
)
if exist "%LOCAL_MODEL_ROOT%\metadata\classes.txt" scp "%LOCAL_MODEL_ROOT%\metadata\classes.txt" %BOARD%:%REMOTE_MODEL%/metadata/
if exist "%LOCAL_MODEL_ROOT%\metadata\model_metadata.yaml" scp "%LOCAL_MODEL_ROOT%\metadata\model_metadata.yaml" %BOARD%:%REMOTE_MODEL%/metadata/

ssh -tt %BOARD% "chmod +x %REMOTE_SH% %REMOTE_MIPI_SH% && cd %REMOTE_ROOT% && export PYTHONPATH=%REMOTE_ROOT% && python3 -m py_compile vehicle_parking_detection/*.py vehicle_parking_detection/tools/*.py"
if errorlevel 1 (
  echo Board py_compile failed.
  pause
  exit /b 1
)

echo.
echo VEHICLE_PARKING_DEPLOY_DONE
echo.
