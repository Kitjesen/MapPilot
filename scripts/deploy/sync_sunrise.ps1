# sync_sunrise.ps1 — 把本地 main 同步到 sunrise@192.168.66.13
# 用法: 在 brain/lingtu 根目录执行 .\scripts\deploy\sync_sunrise.ps1
#
# 原理: SSH 反向隧道把 Windows VPN 代理转发到 sunrise，直接 git pull
# 不需要管理员权限，不需要 bundle 传输

param(
    [string]$SunriseHost = "sunrise@192.168.66.13",
    [string]$RemotePath = "/home/sunrise/data/inovxio/lingtu",
    [string]$ProxyPort = "7890"
)

$ErrorActionPreference = "Stop"

Write-Host "=== LingTu → sunrise 同步 ===" -ForegroundColor Cyan
Write-Host "目标: ${SunriseHost}:$RemotePath"
Write-Host "代理: localhost:$ProxyPort -> sunrise:$ProxyPort (SSH 反向隧道)"
Write-Host ""

Write-Host "[1/2] SSH 反向隧道 + git pull..." -ForegroundColor Yellow

$remote_cmd = @"
export GIT_DISCOVERY_ACROSS_FILESYSTEM=1
cd $RemotePath
https_proxy=http://localhost:$ProxyPort http_proxy=http://localhost:$ProxyPort git pull 2>&1
"@

ssh -o StrictHostKeyChecking=no -R "${ProxyPort}:127.0.0.1:${ProxyPort}" $SunriseHost $remote_cmd
if ($LASTEXITCODE -ne 0) { throw "git pull failed" }

Write-Host ""
Write-Host "[2/2] 验证..." -ForegroundColor Yellow
$result = ssh $SunriseHost "export GIT_DISCOVERY_ACROSS_FILESYSTEM=1; cd $RemotePath && git log --oneline -1"
Write-Host "  sunrise HEAD: $result"

Write-Host ""
Write-Host "=== 同步完成 ===" -ForegroundColor Green

