# build_release.ps1 — Build release artifacts with version injection
# Usage: .\scripts\build_release.ps1 [-Platform android|windows|all]

param(
    [string]$Platform = "all"
)

$ErrorActionPreference = "Stop"

# Extract version from pubspec.yaml
$pubspec = Get-Content "pubspec.yaml" -Raw
$version = ($pubspec | Select-String "version:\s*(\S+)").Matches[0].Groups[1].Value
$appVersion = $version.Split('+')[0]
$buildNumber = if ($version.Contains('+')) { $version.Split('+')[1] } else { "1" }
$gitHash = git rev-parse --short HEAD 2>$null
if (-not $gitHash) { $gitHash = "local" }

Write-Host "Building v$appVersion+$buildNumber ($gitHash)" -ForegroundColor Cyan

$dartDefines = @(
    "--dart-define=APP_VERSION=$appVersion",
    "--dart-define=BUILD_NUMBER=$buildNumber",
    "--dart-define=GIT_HASH=$gitHash"
)

if ($Platform -eq "android" -or $Platform -eq "all") {
    Write-Host "`n=== Building Android APK ===" -ForegroundColor Yellow
    flutter build apk --release @dartDefines
    Write-Host "APK: build\app\outputs\flutter-apk\app-release.apk" -ForegroundColor Green
}

if ($Platform -eq "windows" -or $Platform -eq "all") {
    Write-Host "`n=== Building Windows ===" -ForegroundColor Yellow
    flutter build windows --release @dartDefines
    Write-Host "Windows: build\windows\x64\runner\Release\" -ForegroundColor Green
}

Write-Host "`nDone! Version: v$appVersion+$buildNumber ($gitHash)" -ForegroundColor Cyan
