# sync_versions.ps1 — Reads VERSION file and propagates to all package manifests.
# Run: .\scripts\sync_versions.ps1
# Windows-equivalent of sync_versions.sh
param(
    [switch]$DryRun   # Show changes without writing
)

$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir   = Split-Path -Parent $ScriptDir
$VersionFile = Join-Path $RootDir 'VERSION'

if (-not (Test-Path $VersionFile)) {
    Write-Error "VERSION file not found at $VersionFile"
    exit 1
}

$Version = (Get-Content $VersionFile -TotalCount 1).Trim()
Write-Host "==> Syncing version: $Version"

# ── 1. ROS2 package.xml files ──
Get-ChildItem -Path (Join-Path $RootDir 'src') -Recurse -Filter 'package.xml' |
    Where-Object { $_.FullName -notlike '*3rdparty*' } |
    ForEach-Object {
        $content = Get-Content $_.FullName -Raw
        if ($content -match '<version>[^<]*</version>') {
            $newContent = $content -replace '<version>[^<]*</version>', "<version>$Version</version>"
            if ($DryRun) {
                Write-Host "  [DRY] [pkg.xml] $($_.FullName)"
            } else {
                Set-Content -Path $_.FullName -Value $newContent -NoNewline
                Write-Host "  [pkg.xml] $($_.FullName)"
            }
        }
    }

# ── 2. Flutter pubspec.yaml ──
$Pubspec = Join-Path $RootDir 'client\flutter_monitor\pubspec.yaml'
if (Test-Path $Pubspec) {
    $content = Get-Content $Pubspec -Raw
    $newContent = $content -replace '(?m)^version:.*', "version: $Version"
    if ($DryRun) {
        Write-Host "  [DRY] [pubspec] $Pubspec"
    } else {
        Set-Content -Path $Pubspec -Value $newContent -NoNewline
        Write-Host "  [pubspec] $Pubspec"
    }
}

# ── 3. MSIX version (needs 4-part: X.Y.Z.0) ──
$MsixVersion = "$Version.0"
if (Test-Path $Pubspec) {
    $content = Get-Content $Pubspec -Raw
    if ($content -match 'msix_version:') {
        $newContent = $content -replace 'msix_version:.*', "msix_version: $MsixVersion"
        if ($DryRun) {
            Write-Host "  [DRY] [msix] msix_version -> $MsixVersion"
        } else {
            Set-Content -Path $Pubspec -Value $newContent -NoNewline
            Write-Host "  [msix] msix_version -> $MsixVersion"
        }
    }
}

# ── 4. Dart kAppVersion constant ──
$SplashFile = Join-Path $RootDir 'client\flutter_monitor\lib\features\connection\splash_screen.dart'
if (Test-Path $SplashFile) {
    $content = Get-Content $SplashFile -Raw
    if ($content -match "const String kAppVersion = '[^']*'") {
        $newContent = $content -replace "const String kAppVersion = '[^']*'", "const String kAppVersion = '$Version'"
        if ($DryRun) {
            Write-Host "  [DRY] [dart] kAppVersion -> $Version"
        } else {
            Set-Content -Path $SplashFile -Value $newContent -NoNewline
            Write-Host "  [dart] kAppVersion -> $Version"
        }
    }
}

# ── 5. robot_config.yaml firmware_version ──
$RobotConfig = Join-Path $RootDir 'config\robot_config.yaml'
if (Test-Path $RobotConfig) {
    $content = Get-Content $RobotConfig -Raw
    if ($content -match 'firmware_version:') {
        $newContent = $content -replace 'firmware_version:\s*"[^"]*"', "firmware_version: `"$Version`""
        if ($DryRun) {
            Write-Host "  [DRY] [yaml] robot_config firmware_version -> $Version"
        } else {
            Set-Content -Path $RobotConfig -Value $newContent -NoNewline
            Write-Host "  [yaml] robot_config firmware_version -> $Version"
        }
    }
}

# ── 6. grpc_gateway.yaml firmware_version ──
$GrpcConfig = Join-Path $RootDir 'src\remote_monitoring\config\grpc_gateway.yaml'
if (Test-Path $GrpcConfig) {
    $content = Get-Content $GrpcConfig -Raw
    if ($content -match 'firmware_version:') {
        $newContent = $content -replace 'firmware_version:\s*"[^"]*"', "firmware_version: `"$Version`""
        if ($DryRun) {
            Write-Host "  [DRY] [yaml] grpc_gateway firmware_version -> $Version"
        } else {
            Set-Content -Path $GrpcConfig -Value $newContent -NoNewline
            Write-Host "  [yaml] grpc_gateway firmware_version -> $Version"
        }
    }
}

Write-Host "==> Done. All versions set to $Version"
