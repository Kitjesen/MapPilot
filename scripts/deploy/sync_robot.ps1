# Deployment helper: pull the current branch on a robot-side checkout.

param(
    [string]$TargetHost = $env:LINGTU_TARGET_HOST,
    [string]$TargetUser = $env:LINGTU_TARGET_USER,
    [string]$RemotePath = "~/data/inovxio/lingtu",
    [int]$ProxyPort = 0
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($TargetHost)) {
    throw "Set -TargetHost or LINGTU_TARGET_HOST."
}
if ([string]::IsNullOrWhiteSpace($TargetUser)) {
    throw "Set -TargetUser or LINGTU_TARGET_USER."
}
if ($TargetHost -notmatch '^[A-Za-z0-9.-]+$') {
    throw "TargetHost contains unsupported characters."
}
if ($TargetUser -notmatch '^[A-Za-z_][A-Za-z0-9_-]*$') {
    throw "TargetUser contains unsupported characters."
}

$target = "${TargetUser}@${TargetHost}"
$remoteCommand = "export GIT_DISCOVERY_ACROSS_FILESYSTEM=1; cd '$RemotePath' && git pull --ff-only"

Write-Host "=== LingTu robot sync ===" -ForegroundColor Cyan
Write-Host "Target: ${target}:$RemotePath"

if ($ProxyPort -gt 0) {
    ssh -R "${ProxyPort}:127.0.0.1:${ProxyPort}" $target "https_proxy=http://localhost:$ProxyPort http_proxy=http://localhost:$ProxyPort $remoteCommand"
} else {
    ssh $target $remoteCommand
}
if ($LASTEXITCODE -ne 0) {
    throw "git pull failed"
}

$result = ssh $target "cd '$RemotePath' && git log --oneline -1"
if ($LASTEXITCODE -ne 0) {
    throw "remote verification failed"
}
Write-Host "Remote HEAD: $result"
