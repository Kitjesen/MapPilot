@echo off
chcp 65001 > nul
setlocal EnableExtensions

set "BOARD_IP=192.168.66.65"
set "BOARD=sunrise@%BOARD_IP%"

echo Check board network:
ping -n 2 %BOARD_IP%
echo.
echo Check SSH login:
ssh %BOARD% "hostname && date"
echo.
pause
