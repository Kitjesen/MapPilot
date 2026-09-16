# One-time repair for the verified DESKTOP-17EDVN1 laptop, run as Dz/admin.
$ErrorActionPreference = 'Stop'
if ($env:COMPUTERNAME -ne 'DESKTOP-17EDVN1' -or $env:USERNAME -ne 'Dz') {
    throw 'This repair belongs to DESKTOP-17EDVN1 / Dz.'
}
$listener = Get-NetTCPConnection -State Listen -LocalPort 7898
$owner = Get-Process -Id ($listener | Select-Object -First 1).OwningProcess
if ($owner.ProcessName -ne 'verge-mihomo') {
    throw 'The verified independent local proxy is not running on port 7898.'
}
$names = 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'NO_PROXY'
$backup = @{}
foreach ($scope in 'User', 'Machine') {
    $backup[$scope] = @{}
    foreach ($name in $names) {
        $backup[$scope][$name] = [Environment]::GetEnvironmentVariable($name, $scope)
    }
}
$backup['WinHTTP'] = @(netsh winhttp dump)
$folder = Join-Path $env:LOCALAPPDATA 'LingTu'
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$backupPath = Join-Path $folder ('network-before-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.json')
$backup | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $backupPath -Encoding UTF8
foreach ($scope in 'User', 'Machine') {
    foreach ($name in 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY') {
        $old = [Environment]::GetEnvironmentVariable($name, $scope)
        if ($old -in 'http://thunder.lan:7890', 'http://127.0.0.1:7897') {
            [Environment]::SetEnvironmentVariable($name, 'http://127.0.0.1:7898', $scope)
        }
    }
    $entries = @(([Environment]::GetEnvironmentVariable('NO_PROXY', $scope) -split ',') +
        @('localhost', '127.0.0.1', '::1', '192.168.123.18'))
    $bypass = ($entries | Where-Object { $_ } | Select-Object -Unique) -join ','
    [Environment]::SetEnvironmentVariable('NO_PROXY', $bypass, $scope)
}
netsh winhttp reset proxy
if ($LASTEXITCODE -ne 0) { throw 'WinHTTP proxy reset failed.' }
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class LingTuEnvironmentRefresh {
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern IntPtr SendMessageTimeout(IntPtr hWnd, uint msg,
        UIntPtr wParam, string lParam, uint flags, uint timeout, out UIntPtr result);
}
'@
$result = [UIntPtr]::Zero
[void][LingTuEnvironmentRefresh]::SendMessageTimeout([IntPtr]0xffff, 0x1a,
    [UIntPtr]::Zero, 'Environment', 2, 3000, [ref]$result)
Write-Output "Backup: $backupPath"
Write-Output 'Saved: local CLI proxy 127.0.0.1:7898; WinHTTP direct; NX bypass.'
Write-Output 'Restart existing terminals/apps to discard their inherited old proxy values.'
