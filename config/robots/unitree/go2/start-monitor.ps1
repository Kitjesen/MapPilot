<#
.SYNOPSIS
Open the Go2 read-only live monitor through the existing SSH connection.
.DESCRIPTION
The active Product and Gateway must already be running on Go2. This command
only opens a loopback SSH tunnel and the observer page; it does not start a
Product, claim motion control, or send robot commands.
#>
[CmdletBinding()]
param(
    [string]$SshConfig = (Join-Path $env:USERPROFILE '.ssh\lingtu-go2.conf'),
    [string]$SshTarget = 'lingtu-go2-nx',
    [ValidateRange(1024, 65535)]
    [int]$LocalPort = 15050,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
$sshConfigPath = [System.IO.Path]::GetFullPath($SshConfig)
if (-not (Test-Path -LiteralPath $sshConfigPath -PathType Leaf)) {
    throw "SSH configuration is missing: $sshConfigPath"
}
$forward = "127.0.0.1:${LocalPort}:127.0.0.1:5050"
$listener = Get-NetTCPConnection -State Listen -LocalPort $LocalPort -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($listener) {
    $owner = Get-CimInstance Win32_Process -Filter "ProcessId = $($listener.OwningProcess)"
    if ($listener.LocalAddress -ne '127.0.0.1' -or $owner.Name -ne 'ssh.exe' -or
        $owner.CommandLine -notlike "*$forward*" -or
        $owner.CommandLine -notlike "*$SshTarget*") {
        throw "Port $LocalPort belongs to another service. Choose a different -LocalPort."
    }
} else {
    $sshPath = Join-Path $env:SystemRoot 'System32\OpenSSH\ssh.exe'
    $arguments = @(
        '-F', ('"' + $sshConfigPath + '"'), '-N',
        '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes',
        '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
        '-L', $forward, $SshTarget
    )
    Start-Process -FilePath $sshPath -ArgumentList $arguments -WindowStyle Hidden | Out-Null
}

$url = "http://127.0.0.1:${LocalPort}/?observe=1"
$connected = $false
for ($attempt = 0; $attempt -lt 8; $attempt++) {
    try {
        $response = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 2
        $connected = $response.StatusCode -eq 200
    } catch {
        Start-Sleep -Milliseconds 250
    }
    if ($connected) { break }
}
if (-not $connected) {
    throw 'Go2 Gateway is not reachable through SSH. Check that Go2 and Sunrise are online.'
}
Write-Host "Go2 live monitor: $url"
Write-Host 'Read-only display. Use the separate WASD window for authorized motion tests.'
if (-not $NoBrowser) {
    Start-Process $url
}
