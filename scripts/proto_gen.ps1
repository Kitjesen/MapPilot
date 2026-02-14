# ============================================================
# proto_gen.ps1 — Windows (PowerShell) 版 proto 生成脚本
#
# Proto 唯一源头: src/robot_proto/proto/
# 生成目标:       src/robot_proto/dart/lib/src/
# Flutter 引用:   pubspec.yaml → path: ../../src/robot_proto/dart
#
# 用法:
#   .\scripts\proto_gen.ps1          # 生成 Dart
#   .\scripts\proto_gen.ps1 -Check   # 仅检查工具是否就绪
# ============================================================
[CmdletBinding()]
param(
    [switch]$Check
)

$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir   = Split-Path -Parent $ScriptDir

$ProtoDir = Join-Path $RootDir 'src\robot_proto\proto'
$DartOut  = Join-Path $RootDir 'src\robot_proto\dart\lib\src'

$Protos = @('common.proto', 'system.proto', 'control.proto', 'telemetry.proto', 'data.proto')

# ---- helpers ----
function Write-Info  { param($msg) Write-Host "[INFO]  $msg" -ForegroundColor Cyan }
function Write-Ok    { param($msg) Write-Host "[OK]    $msg" -ForegroundColor Green }
function Write-Fail  { param($msg) Write-Host "[FAIL]  $msg" -ForegroundColor Red; exit 1 }

# ---- pre-flight ----
function Test-Tools {
    # protoc
    $protocPath = Get-Command protoc -ErrorAction SilentlyContinue
    if (-not $protocPath) {
        Write-Fail "protoc not found. Install from https://github.com/protocolbuffers/protobuf/releases or via 'choco install protoc'"
    }
    $protocVer = & protoc --version 2>&1
    Write-Ok "protoc $protocVer"

    # protoc-gen-dart — check PATH including pub-cache
    $pubCacheBin = Join-Path $env:LOCALAPPDATA 'Pub\Cache\bin'
    if ($env:PATH -notlike "*$pubCacheBin*") {
        $env:PATH = "$env:PATH;$pubCacheBin"
    }
    # Also check ~/.pub-cache/bin (Git Bash / WSL convention on Windows)
    $homePubCache = Join-Path $env:USERPROFILE '.pub-cache\bin'
    if ($env:PATH -notlike "*$homePubCache*") {
        $env:PATH = "$env:PATH;$homePubCache"
    }

    $dartPlugin = Get-Command protoc-gen-dart -ErrorAction SilentlyContinue
    if (-not $dartPlugin) {
        Write-Fail "protoc-gen-dart not found. Install: dart pub global activate protoc_plugin"
    }
    Write-Ok "protoc-gen-dart $($dartPlugin.Source)"
}

# ---- generate ----
function Invoke-GenDart {
    Write-Info "Generating Dart gRPC stubs -> robot_proto/dart/lib/src/"

    if (-not (Test-Path $DartOut)) {
        New-Item -ItemType Directory -Path $DartOut -Force | Out-Null
    }

    $protoFiles = @()
    foreach ($p in $Protos) {
        $full = Join-Path $ProtoDir $p
        if (-not (Test-Path $full)) {
            Write-Fail "Missing proto: $full"
        }
        $protoFiles += $full
    }

    & protoc --dart_out="grpc:$DartOut" -I $ProtoDir @protoFiles
    if ($LASTEXITCODE -ne 0) {
        Write-Fail "protoc failed with exit code $LASTEXITCODE"
    }

    # Clean well-known types local copies (Dart protobuf package ships its own)
    $googleDir = Join-Path $DartOut 'google'
    if (Test-Path $googleDir) {
        Remove-Item $googleDir -Recurse -Force
        Write-Info "Removed generated google/ (using package:protobuf instead)"
    }

    $count = (Get-ChildItem -Path $DartOut -Filter '*.dart' -File).Count
    Write-Ok "Generated $count Dart files"
}

# ---- main ----
Write-Info "Proto source: $ProtoDir"
Write-Info "Dart output:  $DartOut"
Write-Host ""

Test-Tools
Write-Host ""

if ($Check) {
    Write-Ok "All checks passed."
    exit 0
}

Invoke-GenDart
Write-Host ""
Write-Info "C++ side: colcon build --packages-select robot_proto remote_monitoring"
Write-Host ""
Write-Ok "Done!"
